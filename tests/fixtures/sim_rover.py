"""A SIMULATED rover around the REAL agent: firmware, wheels, GPS and camera faked.

What is real
------------
`RobotAgent` and its own loop (`_loop`, at its configured rate), the mission
manager, navigator, decision layer, safety gate, obstacle recovery, progress
and applied-motion monitors, and `Esp32Link` -- framing, CRC, sequence, PING
proof, ACK matching and the motion slot -- exactly as on the rover.

What is simulated, and how
--------------------------
- The ESP32: `FirmwareModel` over an in-memory `FakePort`. No serial device is
  ever opened; there is no code path here that could open one.
- The wheels: the rover advances toward the navigator's target waypoint at a
  speed proportional to the firmware's mean *applied* output while that output
  is forward (no side below 0, one above), and not at all otherwise. Steering is idealised (it heads straight for the
  target), so nothing here tests steering or heading: `tests.fixtures.diff_drive_sim` does. This is kinematics
  invented for the test, not a model of the rover.
- GPS: a measured 3D fix at the simulated position every tick, with speed and
  track (so a heading exists once it moves), h_acc 1.5 m.
- Perception: a clear scene every tick, unless `obstacle` (an object dead
  ahead) or `blind` (no camera frame) is set.

A run that completes here proves the Pi's software takes a mission from
admission to completion against simulated feedback. It proves nothing about
the physical rover: no motor, wheel, GPS receiver or ESP32 took part.
"""

from __future__ import annotations

import asyncio
import math
import time
from typing import Any, Dict, Optional, Tuple

from robotx.application.agent import RobotAgent
from robotx.config.settings import Settings
from robotx.esp32.link import Esp32Config, Esp32Link
from robotx.localization.position import bearing_deg, haversine_m
from robotx.perception.types import PerceptionResult, PerceptionStatus
from robotx.state.robot_state import OperatingMode
from tests.fixtures import esp32 as fx
from tests.fixtures.esp32_sim import FirmwareModel
from tests.fixtures.task_assign import offset
from tests.unit.test_agent import FakeGPS, FakePerception

LatLon = Tuple[float, float]

HEALTHY = dict(fx.CLEAR_FRONT, motor_drive_available=True, state="IDLE", block_reason="NONE",
               safety_stop=False, command_timeout=False)

# Simulated metres per second at full applied output (255). The agent's cruise
# (0.45 -> 115 units) then moves ~2.7 m/s: far faster than the real rover, so a
# run takes seconds, and still under the backend's 8.33 m/s evidence ceiling.
FULL_SCALE_MPS = 6.0


def settings_env(robot_id: str, **extra: str) -> Dict[str, str]:
    """An explicit environment: nothing is read from the host's own ROBOTX_* variables."""

    env = {
        "ROBOTX_ROBOT_ID": robot_id,
        "ROBOTX_CAMERA_ENABLED": "0",
        "ROBOTX_PERCEPTION_ENABLED": "0",
        "ROBOTX_GPS_ENABLED": "0",
        "ROBOTX_ESP32_ENABLED": "0",          # the link below is attached by hand, over a fake port
        "ROBOTX_SOCKET_ENABLED": "0",
        "ROBOTX_LOG_LEVEL": "CRITICAL",
        "ROBOTX_AGENT_HZ": "10",
        "ROBOTX_CUSTODY_CONFIRMATION": "operator",
    }
    env.update(extra)
    return env


class SimulatedRover:
    def __init__(self, start: LatLon, *, robot_id: str = "robotx-pi", env: Optional[Dict[str, str]] = None) -> None:
        self.agent = RobotAgent(Settings.from_env({**settings_env(robot_id), **(env or {})}))
        self.gps = FakeGPS()
        self.perception = FakePerception()
        self.agent.gps, self.agent.perception = self.gps, self.perception
        self.port = fx.FakePort()
        self.firmware = FirmwareModel()
        self.link = Esp32Link(Esp32Config(motion_enabled=True), port_factory=lambda: self.port,
                              clock=time.monotonic, wall=time.time)
        self.agent.esp32 = self.link
        self.position: LatLon = start
        self.track: Optional[float] = None
        self.speed = 0.0
        self.fields: Dict[str, Any] = dict(HEALTHY)
        self.obstacle = False
        # The camera produces nothing (perception NO_FRAME) while set.
        self.blind = False
        self.alive = True
        self.uptime = 3_600_000
        self._answered = 0
        self._last_t: Optional[float] = None
        self._last_tel = 0.0
        self._task: Optional[asyncio.Task] = None
        # The lowest wheel value in any DRIVE written: below 0 would be a reverse or pivot.
        self.min_drive_units = 0
        self._publish_fix()
        self.perception.set_clear()

    # --- the simulated world ----------------------------------------------------

    def _publish_fix(self) -> None:
        # As the rover's u-blox states them (ESP32 GPS frame): a 3D fix at 1.5 m.
        self.gps.set_fix(*self.position, speed=self.speed, track=self.track, fix_type="3D", h_acc=1.5)

    def _move(self, dt: float) -> None:
        left, right = self.firmware.applied
        target = self.agent.navigator.state.target_waypoint
        # Forward only (both sides >= 0, one > 0): a steer that slows one side
        # to 0 still moves the rover, along an arc this model straightens out.
        if min(left, right) >= 0 and max(left, right) > 0 and target is not None:
            self.speed = FULL_SCALE_MPS * (left + right) / 2 / 255
            remaining = haversine_m(self.position, target)
            step = min(remaining, self.speed * dt)
            if remaining > 0.01:
                self.track = bearing_deg(self.position, target)
                north = step * math.cos(math.radians(self.track))
                east = step * math.sin(math.radians(self.track))
                self.position = offset(self.position, north_m=north, east_m=east)
        else:
            self.speed = 0.0

    def _esp32(self, now: float) -> None:
        if not self.alive:
            self.link.poll_once()
            return
        out = b""
        commands = self.port.commands()
        for c in commands[self._answered:]:
            if c["cmd"] == "DRIVE":
                self.min_drive_units = min(self.min_drive_units, c["left"], c["right"])
            out += self.firmware.execute(c, self.fields)
        self._answered = len(commands)
        if now - self._last_tel >= 0.2:
            self._last_tel = now
            self.uptime += 200
            out += fx.telemetry(uptime_ms=self.uptime, **{**self.firmware.telemetry_fields(), **self.fields})
        if out:
            self.port.feed(out)
        self.link.poll_once()

    def step(self) -> None:
        """One simulation step (call at ~20 Hz): wheels, GPS, camera, then the ESP32."""

        now = time.monotonic()
        dt = 0.0 if self._last_t is None else now - self._last_t
        self._last_t = now
        self._move(dt)
        self._publish_fix()
        if self.blind:
            self.perception.result = PerceptionResult.unavailable(PerceptionStatus.NO_FRAME)
        elif self.obstacle:
            self.perception.set_detection(area=40000, cx=320)
        else:
            self.perception.set_clear()
        self._esp32(now)

    # --- the agent's own loop ---------------------------------------------------

    async def start(self) -> None:
        """Bring the fake ESP32 up (PING answered) and start the agent's REAL loop.

        `RobotAgent.start()` is not used: it would build the camera, perception
        and GPS from settings and replace the simulated ones. The loop it would
        start -- `_loop` -- is started directly instead.
        """

        for _ in range(10):
            self.step()
            await asyncio.sleep(0.05)
        self.agent.state.set_mode(OperatingMode.IDLE)      # as start() leaves it
        self.agent._running = True
        self._task = asyncio.create_task(self.agent._loop(), name="agent-loop")

    async def run(self, seconds: float, until=None) -> bool:
        """Step the world for up to `seconds`; return True as soon as `until()` holds."""

        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.step()
            if until is not None and until():
                return True
            await asyncio.sleep(0.05)
        return until() if until is not None else True

    async def stop(self) -> None:
        self.agent._running = False
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        self.link.stop()

    # --- reads --------------------------------------------------------------------

    def mission(self):
        return self.agent.state.snapshot().mission

    def drives(self, since: int = 0):
        return [c for c in self.port.commands()[since:] if c["cmd"] == "DRIVE"]

    def mark(self) -> int:
        return len(self.port.commands())


def short_route(start: LatLon, *, leg_m: float = 24.0):
    """A two-leg route from `start`: north to the pickup, then east to the drop. 3 points a leg."""

    pickup = offset(start, north_m=leg_m)
    drop = offset(pickup, east_m=leg_m)
    to_pickup = [start, offset(start, north_m=leg_m / 2), pickup]
    to_drop = [pickup, offset(pickup, east_m=leg_m / 2), drop]
    return pickup, drop, to_pickup, to_drop
