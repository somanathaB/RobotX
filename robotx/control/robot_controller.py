"""RETAINED LEGACY PATH -- direct motor drive from the Pi. NOT wired in.

This is the previous production control loop: it reads GPIO sensors and drives
the L298N motor driver directly from the Raspberry Pi. **The application no
longer starts it.** In the target architecture motor authority belongs to the
ESP32, and the Pi's job ends at publishing a `MotionIntent`
(`robotx.control.decision` -> `robotx.control.motion`).

It is retained, not deleted, because it carries safety behaviour that was
deliberately built and reviewed:
  - R-01: MANUAL mode obeys the same obstacle gate as AUTO/RETURN.
  - R-03: any non-VALID ultrasonic status blocks forward motion.
Those rules are the reference for whatever eventually runs on the ESP32, and
the loop remains usable for bench-testing the drivetrain from the Pi.

If you run this, you are driving motors from the Pi. Wheels off the ground.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from robotx.hardware.encoders import EncoderReader
from robotx.hardware.ir import IRSensors
from robotx.hardware.motors import MotorDriver
from robotx.hardware.ultrasonic import UltrasonicReading, UltrasonicSensor, UltrasonicStatus
from robotx.hardware.gps import GPSReader
from robotx.hardware.camera import CameraStream
from robotx.navigation.directions_client import GoogleMapsDirections
from robotx.navigation.route_planner import LatLon, RoutePlanner, haversine_m
from robotx.perception.object_detector import ObjectDetector, summarize_detections
from robotx.hardware.battery import battery_status


logger = logging.getLogger(__name__)


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def bearing_rad(a: LatLon, b: LatLon) -> float:
    lat1 = math.radians(a[0])
    lat2 = math.radians(b[0])
    dlon = math.radians(b[1] - a[1])
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.atan2(y, x)


@dataclass
class ControllerConfig:
    control_hz: float = 10.0
    telemetry_interval_s: float = 1.5
    obstacle_distance_cm: float = 35.0

    target_speed_mps: float = 0.25
    max_cmd: float = 0.7

    avoid_turn_seconds: float = 0.5
    avoid_forward_seconds: float = 0.6

    detection_hz: float = 2.0


class _PI:
    def __init__(self, kp: float = 1.2, ki: float = 0.6, out_min: float = 0.0, out_max: float = 1.0) -> None:
        self.kp = kp
        self.ki = ki
        self.out_min = out_min
        self.out_max = out_max
        self._i = 0.0

    def reset(self) -> None:
        self._i = 0.0

    def update(self, err: float, dt: float) -> float:
        self._i += err * max(0.0, dt)
        u = self.kp * err + self.ki * self._i
        return max(self.out_min, min(self.out_max, u))


class RobotController:
    """Central decision loop: sensors -> decision -> actuation -> telemetry."""

    def __init__(
        self,
        cfg: ControllerConfig,
        motors: MotorDriver,
        encoders: EncoderReader,
        ultrasonic: UltrasonicSensor,
        ir: IRSensors,
        gps: GPSReader,
        camera: CameraStream,
        detector: ObjectDetector,
        planner: RoutePlanner,
        maps: GoogleMapsDirections,
        robot_id: str,
    ) -> None:
        self.cfg = cfg
        self.robot_id = robot_id

        self.motors = motors
        self.encoders = encoders
        self.ultrasonic = ultrasonic
        self.ir = ir
        self.gps = gps
        self.camera = camera
        self.detector = detector
        self.planner = planner
        self.maps = maps

        self._mode = "IDLE"  # IDLE|AUTO|MANUAL|RETURN|STOPPED|ERROR
        self._status_msg = ""
        self._running = False
        self._task: Optional[asyncio.Task] = None

        self._manual_cmd: Dict[str, Any] = {"action": "stop", "speed": 0.4}

        self._home: Optional[LatLon] = None
        self._destination: Optional[LatLon] = None

        self._last_gps_pos: Optional[LatLon] = None
        self._last_heading: Optional[float] = None

        self._speed_pi = _PI(out_min=0.0, out_max=float(self.cfg.max_cmd))
        self._last_control_t = time.monotonic()

        self._last_detection_t = 0.0
        self._last_detections = []

        self._last_telemetry_t = 0.0
        self._telemetry_hook = None

    def _location(self) -> Dict[str, Optional[float]]:
        """GPS position in this loop's legacy `{lat, lon, fix_age_s}` shape.

        `GPSReader` now reports a status-explicit `GpsReading`; only a current
        FIX yields coordinates here, so a stale or absent fix still reads as
        "no position" to the rest of this loop.
        """

        reading = self.gps.get_reading()
        if not reading.has_fix or reading.fix is None:
            return {"lat": None, "lon": None, "fix_age_s": None}
        return {
            "lat": reading.fix.latitude,
            "lon": reading.fix.longitude,
            "fix_age_s": reading.age_s,
        }

    def set_telemetry_hook(self, hook):
        """hook(telemetry_dict) -> awaitable; called periodically."""
        self._telemetry_hook = hook

    def start(self) -> None:
        if self._running:
            return
        self._running = True

        self.motors.start()
        self.encoders.start()
        self.ultrasonic.start()
        self.ir.start()
        self.gps.start()
        self.camera.start()

        self._task = asyncio.create_task(self._loop())

    async def stop(self) -> None:
        self._running = False
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except Exception:
                pass
        try:
            self.motors.stop()
        except Exception:
            pass
        try:
            self.camera.stop()
        except Exception:
            pass
        try:
            self.gps.stop()
        except Exception:
            pass
        try:
            self.ultrasonic.stop()
        except Exception:
            pass
        try:
            self.encoders.stop()
        except Exception:
            pass

    async def handle_command(self, cmd: Dict[str, Any]) -> None:
        ctype = str(cmd.get("type", "")).upper()
        payload = cmd.get("payload") if isinstance(cmd.get("payload"), dict) else {}

        if ctype == "START":
            dest = payload.get("destination")
            if isinstance(dest, dict) and "lat" in dest and "lon" in dest:
                self._destination = (float(dest["lat"]), float(dest["lon"]))
            self._mode = "AUTO"
            self._status_msg = "AUTO started"
            await self._ensure_route()
            return

        if ctype == "STOP":
            self._mode = "STOPPED"
            self._status_msg = "STOP requested"
            self.motors.stop()
            return

        if ctype == "RETURN":
            self._mode = "RETURN"
            self._status_msg = "RETURN requested"
            # return to home (first fix) unless provided
            home = payload.get("home")
            if isinstance(home, dict) and "lat" in home and "lon" in home:
                self._home = (float(home["lat"]), float(home["lon"]))
            await self._ensure_return_route()
            return

        if ctype == "MANUAL":
            self._mode = "MANUAL"
            self._manual_cmd = {
                "action": str(payload.get("action", "stop")).lower(),
                "speed": float(payload.get("speed", 0.4)),
                "left": payload.get("left"),
                "right": payload.get("right"),
            }
            self._status_msg = f"MANUAL {self._manual_cmd.get('action')}"
            return

        logger.info("Unknown command: %s", cmd)

    async def _ensure_route(self) -> None:
        if self._destination is None:
            return

        loc = self._location()
        if loc.get("lat") is None or loc.get("lon") is None:
            return

        start: LatLon = (float(loc["lat"]), float(loc["lon"]))
        try:
            route = await self.maps.get_route(start, self._destination)
            self.planner.set_route(route)
        except Exception as e:
            self._status_msg = f"Route fetch failed: {e}"

    async def _ensure_return_route(self) -> None:
        if self._home is None:
            loc = self._location()
            if loc.get("lat") is not None and loc.get("lon") is not None:
                self._home = (float(loc["lat"]), float(loc["lon"]))

        if self._home is None:
            return

        loc = self._location()
        if loc.get("lat") is None or loc.get("lon") is None:
            return

        start: LatLon = (float(loc["lat"]), float(loc["lon"]))
        try:
            route = await self.maps.get_route(start, self._home)
            self.planner.set_route(route)
        except Exception as e:
            self._status_msg = f"Return route failed: {e}"

    def _update_heading(self, pos: LatLon) -> None:
        if self._last_gps_pos is None:
            self._last_gps_pos = pos
            return
        if haversine_m(self._last_gps_pos, pos) < 1.0:
            return
        self._last_heading = bearing_rad(self._last_gps_pos, pos)
        self._last_gps_pos = pos

    def _safety_blocked(self, ultrasonic: UltrasonicReading, ir_state: Dict[str, Any], detections_summary: Dict[str, Any]) -> bool:
        # Conservative by design: any ultrasonic status other than a fresh VALID
        # reading (TIMEOUT/OUT_OF_RANGE/ERROR/DISCONNECTED/STALE/UNKNOWN) is
        # treated as blocking, not as "no obstacle". A failed/disconnected/
        # not-yet-sampled sensor must not be indistinguishable from a clear one.
        if ultrasonic.status != UltrasonicStatus.VALID:
            return True
        if ultrasonic.distance_cm is not None and ultrasonic.distance_cm < self.cfg.obstacle_distance_cm:
            return True
        if any(v is True for v in ir_state.values() if v is not None):
            return True
        if detections_summary.get("person") or detections_summary.get("obstacle"):
            return True
        return False

    async def _avoid(self, ir_state: Dict[str, Any]) -> None:
        # Simple reactive avoidance. Uses IR hints if present.
        self.motors.stop()
        await asyncio.sleep(0.05)

        prefer_left = True
        if ir_state.get("left") and not ir_state.get("right"):
            prefer_left = False
        elif ir_state.get("right") and not ir_state.get("left"):
            prefer_left = True

        for direction in (["left", "right"] if prefer_left else ["right", "left"]):
            if direction == "left":
                self.motors.turn_left(0.45)
            else:
                self.motors.turn_right(0.45)
            await asyncio.sleep(self.cfg.avoid_turn_seconds)
            self.motors.forward(0.45)
            await asyncio.sleep(self.cfg.avoid_forward_seconds)

            reading = self.ultrasonic.get_reading()
            # Only declare "clear" on a fresh, valid, in-range reading. A
            # failed/stale/disconnected sensor here must not be read as
            # clearance to stop avoiding -- fall through to the next
            # direction (or the final stop+report_blocked below) instead.
            if reading.status == UltrasonicStatus.VALID and (
                reading.distance_cm is None or reading.distance_cm >= self.cfg.obstacle_distance_cm
            ):
                self.motors.stop()
                return

        self.motors.stop()
        self.planner.report_blocked()

    def _route_follow_command(self, pos: LatLon) -> Tuple[float, float]:
        wp = self.planner.next_waypoint()
        if wp is None:
            return (0.0, 0.0)

        base = float(self._speed_pi.update(self.cfg.target_speed_mps - self.encoders.get_speed()["avg_mps"], 0.1))
        base = max(0.2, min(self.cfg.max_cmd, base))

        if self._last_heading is None:
            return (base, base)

        desired = bearing_rad(pos, wp)
        err = _wrap_pi(desired - self._last_heading)

        # Convert angular error to differential drive steering
        steer = max(-1.0, min(1.0, err / 0.7))
        steer_gain = 0.25
        left = base - steer_gain * steer
        right = base + steer_gain * steer
        return (max(-self.cfg.max_cmd, min(self.cfg.max_cmd, left)), max(-self.cfg.max_cmd, min(self.cfg.max_cmd, right)))

    async def _loop(self) -> None:
        period = 1.0 / max(1.0, float(self.cfg.control_hz))

        while self._running:
            t0 = time.monotonic()
            try:
                # Read sensors
                loc = self._location()
                pos = None
                if loc.get("lat") is not None and loc.get("lon") is not None:
                    pos = (float(loc["lat"]), float(loc["lon"]))
                    self._update_heading(pos)
                    if self._home is None:
                        self._home = pos
                    self.planner.update_position(pos)

                ultrasonic_reading = self.ultrasonic.get_reading()
                ir_state = self.ir.read_ir()
                speed = self.encoders.get_speed()

                # Throttle perception
                detections = self._last_detections
                if time.monotonic() - self._last_detection_t >= (1.0 / self.cfg.detection_hz):
                    frame = self.camera.get_frame()
                    detections = await asyncio.to_thread(self.detector.detect_objects, frame)
                    self._last_detections = detections
                    self._last_detection_t = time.monotonic()

                det_summary = summarize_detections(detections)

                blocked = self._safety_blocked(ultrasonic_reading, ir_state, det_summary)

                if self._mode in {"STOPPED", "ERROR"}:
                    self.motors.stop()

                elif self._mode == "MANUAL":
                    self._apply_manual(blocked)

                elif self._mode in {"AUTO", "RETURN"}:
                    if blocked:
                        self._status_msg = "Blocked: avoiding"
                        await self._avoid(ir_state)
                    else:
                        if pos is not None and self.planner.should_reroute():
                            self._status_msg = "Rerouting"
                            if self._mode == "RETURN":
                                await self._ensure_return_route()
                            else:
                                await self._ensure_route()

                        if pos is not None and self.planner.has_route():
                            left, right = self._route_follow_command(pos)
                            self.motors.set_speed(left, right)
                            self._status_msg = "Following route"
                        else:
                            # No route yet - safe stop
                            self.motors.stop()
                            self._status_msg = "Waiting for route/GPS"

                else:  # IDLE
                    self.motors.stop()

                # Telemetry
                if self._telemetry_hook and (time.monotonic() - self._last_telemetry_t) >= self.cfg.telemetry_interval_s:
                    telemetry = {
                        "robot_id": self.robot_id,
                        "mode": self._mode,
                        "status": self._status_msg,
                        "location": loc,
                        "speed": speed,
                        "battery": battery_status(),
                        "sensors": {
                            "ultrasonic_cm": ultrasonic_reading.distance_cm,
                            "ultrasonic_status": ultrasonic_reading.status.value,
                            "ir": ir_state,
                        },
                        "perception": {"summary": det_summary, "detections": detections[:10]},
                        "route": {
                            "has_route": self.planner.has_route(),
                            "progress": self.planner.progress(),
                            "next_waypoint": self.planner.next_waypoint(),
                            "destination": self.planner.destination(),
                        },
                        "ts": time.time(),
                    }
                    await self._telemetry_hook(telemetry)
                    self._last_telemetry_t = time.monotonic()

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.exception("Controller loop error: %s", e)
                self._mode = "ERROR"
                self._status_msg = f"ERROR: {e}"
                try:
                    self.motors.stop()
                except Exception:
                    pass

            dt = time.monotonic() - t0
            await asyncio.sleep(max(0.0, period - dt))

    def _apply_manual(self, blocked: bool) -> None:
        """Execute the current manual command. `blocked` is the same
        centralized safety result (ultrasonic/IR/vision) that AUTO/RETURN
        already obey -- MANUAL must not bypass it. Only net-forward motion
        (forward / a forward-net set_speed) is refused when blocked;
        backward and in-place turning remain available since none of the
        current sensors cover the rear, and refusing all manual motion while
        blocked would remove the operator's ability to steer away from
        whatever tripped the check."""
        action = str(self._manual_cmd.get("action", "stop")).lower()
        speed = float(self._manual_cmd.get("speed", 0.4))
        speed = max(0.0, min(self.cfg.max_cmd, speed))

        if action == "set_speed":
            left = self._manual_cmd.get("left")
            right = self._manual_cmd.get("right")
            if isinstance(left, (int, float)) and isinstance(right, (int, float)):
                left_f, right_f = float(left), float(right)
                if blocked and left_f > 0 and right_f > 0:
                    self.motors.stop()
                    self._status_msg = "MANUAL blocked: safety hold (obstacle/sensor)"
                    return
                self.motors.set_speed(left_f, right_f)
            else:
                self.motors.stop()
            return

        if action == "forward":
            if blocked:
                self.motors.stop()
                self._status_msg = "MANUAL blocked: safety hold (obstacle/sensor)"
                return
            self.motors.forward(speed)
        elif action == "backward":
            self.motors.backward(speed)
        elif action == "left":
            self.motors.turn_left(speed)
        elif action == "right":
            self.motors.turn_right(speed)
        else:
            self.motors.stop()
