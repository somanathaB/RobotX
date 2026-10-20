"""A closed-loop GPS navigation simulation with a DIFFERENTIAL-DRIVE rover.

Why this exists
---------------
`tests.fixtures.sim_rover` moves its rover straight at the target whatever the
left/right DRIVE says, so it cannot show whether the Pi steers the right way,
or whether its heading loop converges at all. This one closes the loop the way
the physical rover would: the wheel outputs set a yaw rate, the yaw is hidden
from the Pi, and the only heading the Pi gets is what the GPS reports.

What is real (production code, called in `RobotAgent.tick`'s order)
-------------------------------------------------------------------
`reading_from_frame` (the ESP32 GPS adapter), `PositionEstimator` (with the
agent's `driven` rule), `Navigator`/`RoutePlanner`, `MissionManager`,
`DecisionMaker`, `SafetyGate` (with a forward range always expected, as with an
ESP32 attached), `ProgressMonitor`, the heading invalidation on a pivot, and
`to_drive_units`.

What is simulated, and how
--------------------------
- The ESP32 motor layer: DRIVE units with the +/-40 deadband (PROTOCOL.md
  section 9), applied at once. No ACK/TELEMETRY transport: the link is covered
  by `tests.fixtures.sim_rover` and the esp32 suite.
- The wheels: a unicycle model. Side speed = applied/255 x `full_scale_mps`;
  yaw rate = (left - right) / `track_m` (left faster turns right, clockwise).
- GPS: one u-blox NAV-PVT frame per `rate_hz`, with a correlated (Gauss-Markov)
  position error, the stated `h_acc_m`, ground speed with noise, and a
  heading of motion whose noise grows as speed falls -- at a standstill it is
  noise, as on the real receiver.
- Perception: always a usable, clear frame. Range: VALID, clear, unless an
  obstacle window says otherwise.

Time is virtual: a 5-minute drive runs in well under a second, deterministically
for a given seed. A run here is evidence about the Pi's navigation logic against
invented physics -- never about the physical rover. Its numbers (speed, track
width, GPS noise) are assumptions until measured on the ground.
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from robotx.config.settings import Settings
from robotx.control.decision import DecisionConfig, DecisionMaker
from robotx.control.motion import MotionIntent
from robotx.control.safety import RangeReading, RangeStatus, SafetyConfig, SafetyGate
from robotx.esp32.link import to_drive_units
from robotx.localization.esp32_gps import reading_from_frame
from robotx.localization.position import (
    LatLon,
    PositionConfig,
    PositionEstimator,
    bearing_deg,
    haversine_m,
    heading_error_deg,
)
from robotx.mission.manager import MissionManager
from robotx.mission.mission import Mission, MissionStatus
from robotx.navigation.navigator import NavigationConfig, NavigationStatus, Navigator
from robotx.navigation.progress import ProgressConfig, ProgressMonitor
from robotx.perception.types import FrameMetadata, PerceptionResult, PerceptionStatus

METRES_PER_DEG_LAT = 111_320.0
DEADBAND_UNITS = 40


def offset(origin: LatLon, *, north_m: float = 0.0, east_m: float = 0.0) -> LatLon:
    lat, lon = origin
    return (lat + north_m / METRES_PER_DEG_LAT,
            lon + east_m / (METRES_PER_DEG_LAT * math.cos(math.radians(lat))))


def clear_scene() -> PerceptionResult:
    return PerceptionResult(timestamp=time.time(), status=PerceptionStatus.OK, detections=(),
                            frame=FrameMetadata(width=640, height=480, age_s=0.05), backend="sim")


@dataclass(frozen=True)
class Physics:
    """Invented drivetrain numbers. Every one is an assumption until measured."""

    full_scale_mps: float = 0.6     # side speed at DRIVE 255: cruise 0.45 -> ~0.27 m/s
    track_m: float = 0.30           # effective (skid-steer) track width
    deadband: int = DEADBAND_UNITS


@dataclass(frozen=True)
class GpsModel:
    rate_hz: float = 1.0
    h_acc_m: float = 2.0            # what the receiver states (hacc_mm)
    sigma_m: float = 0.7            # true per-axis error, 1 sigma
    tau_s: float = 20.0             # error correlation time
    speed_sigma_mps: float = 0.03
    vel_sigma_mps: float = 0.05     # velocity noise behind the heading of motion
    age_ms: int = 60


@dataclass
class Window:
    start_s: float
    end_s: float
    value: Any = None

    def active(self, t: float) -> bool:
        return self.start_s <= t < self.end_s


@dataclass
class Row:
    t: float
    x: float
    y: float
    yaw: float
    left: int
    right: int
    nav: str
    reason: str
    heading_source: str
    believed_heading: Optional[float]
    desired: Optional[float]
    est_error: Optional[float]
    true_error: Optional[float]
    distance: Optional[float]
    waypoint_index: int = 0


@dataclass
class Result:
    outcome: str                    # COMPLETE | ARRIVED | PAUSED | TIMEOUT
    reason: str
    t: float
    rows: List[Row] = field(default_factory=list)
    path_m: float = 0.0
    total_turn_deg: float = 0.0
    final_distance_m: Optional[float] = None
    drives: int = 0
    reverse_or_pivot_drives: int = 0
    sign_checked: int = 0
    sign_wrong: int = 0             # steered away from the target as the PI believed it
    pause_reasons: List[str] = field(default_factory=list)

    def summary(self) -> Dict[str, Any]:
        return {
            "outcome": self.outcome, "reason": self.reason, "t_s": round(self.t, 1),
            "path_m": round(self.path_m, 1), "turned_deg": round(self.total_turn_deg),
            "rotations": round(self.total_turn_deg / 360.0, 1),
            "final_distance_m": None if self.final_distance_m is None else round(self.final_distance_m, 1),
            "drives": self.drives, "reverse_or_pivot_drives": self.reverse_or_pivot_drives,
            "steer_sign_wrong": f"{self.sign_wrong}/{self.sign_checked}",
        }


def settings(**env: str) -> Settings:
    return Settings.from_env({"ROBOTX_ROBOT_ID": "RBT-SIM-NAV", **env})


class ClosedLoopNav:
    """The Pi's navigation stack, closed through a simulated differential drive."""

    def __init__(
        self,
        start: LatLon,
        yaw_deg: float,
        *,
        route: Optional[Sequence[LatLon]] = None,
        mission: Optional[Mission] = None,
        cfg: Optional[Settings] = None,
        physics: Physics = Physics(),
        gps: GpsModel = GpsModel(),
        seed: int = 1,
        dt: float = 0.1,
    ) -> None:
        self.cfg = cfg or settings()
        self.physics, self.gps_model, self.dt = physics, gps, dt
        self.origin = start
        self.rng = random.Random(seed)
        self.t = 0.0
        self.wall0 = 1_800_000_000.0
        self.x = self.y = 0.0
        self.yaw = yaw_deg % 360.0
        self.applied: Tuple[int, int] = (0, 0)
        self.intent_moving = False
        self.err = [0.0, 0.0]
        self.frame: Optional[Dict[str, Any]] = None
        self.frame_at: Optional[float] = None
        self.next_fix_t = 0.0
        self.paused = False

        self.estimator = PositionEstimator(PositionConfig.from_settings(self.cfg))
        self.navigator = Navigator(NavigationConfig.from_settings(self.cfg))
        self.missions = MissionManager(self.navigator)
        self.decision = DecisionMaker(DecisionConfig.from_settings(self.cfg), clock=lambda: self.t)
        self.safety = SafetyGate(SafetyConfig.from_settings(self.cfg))
        self.progress = ProgressMonitor(ProgressConfig.from_settings(self.cfg), clock=lambda: self.t)
        if mission is not None:
            self.missions.assign(mission)
        elif route is not None:
            self.navigator.set_route(list(route))
        else:
            raise ValueError("a route or a mission is required")

        # Fault windows (virtual seconds).
        self.gps_outages: List[Window] = []
        self.gps_bad_acc: List[Window] = []       # value: stated h_acc in metres
        self.obstacles: List[Window] = []         # value: range in cm
        self.stop_at: Optional[float] = None      # operator STOP (mission aborted)

    # --- the world ------------------------------------------------------------------

    @property
    def here(self) -> LatLon:
        return offset(self.origin, north_m=self.y, east_m=self.x)

    def _gps_error(self, dt: float) -> None:
        m = self.gps_model
        a = math.exp(-dt / m.tau_s)
        q = m.sigma_m * math.sqrt(1 - a * a)
        self.err = [a * e + q * self.rng.gauss(0, 1) for e in self.err]

    def _fix(self, v: float) -> None:
        m = self.gps_model
        self._gps_error(1.0 / m.rate_hz)
        if any(w.active(self.t) for w in self.gps_outages):
            return                                  # no new frame: the last one ages
        lat, lon = offset(self.origin, north_m=self.y + self.err[1], east_m=self.x + self.err[0])
        h_acc = next((w.value for w in self.gps_bad_acc if w.active(self.t)), m.h_acc_m)
        # Velocity as the receiver measures it: true + noise. Its direction is
        # the heading of motion; at a standstill it is pure noise.
        vn = v * math.cos(math.radians(self.yaw)) + self.rng.gauss(0, m.vel_sigma_mps)
        ve = v * math.sin(math.radians(self.yaw)) + self.rng.gauss(0, m.vel_sigma_mps)
        speed = max(0.0, abs(v) + self.rng.gauss(0, m.speed_sigma_mps))
        head = math.degrees(math.atan2(ve, vn)) % 360.0
        self.frame = {"gps_status": "OK", "fix_ok": True, "fix_type": 3,
                      "lat_e7": int(round(lat * 1e7)), "lon_e7": int(round(lon * 1e7)),
                      "age_ms": m.age_ms, "hacc_mm": int(h_acc * 1000), "siv": 11,
                      "speed_mm_s": int(round(speed * 1000)), "head_mot_e5": int(round(head * 1e5))}
        self.frame_at = self.wall0 + self.t

    def _drive(self, intent: MotionIntent) -> Tuple[int, int]:
        units = (to_drive_units(intent.left), to_drive_units(intent.right)) if not intent.is_stop else (0, 0)
        self.applied = tuple(0 if abs(u) < self.physics.deadband else u for u in units)  # type: ignore[assignment]
        return units

    def _integrate(self) -> float:
        p = self.physics
        vl = self.applied[0] / 255.0 * p.full_scale_mps
        vr = self.applied[1] / 255.0 * p.full_scale_mps
        v = (vl + vr) / 2.0
        omega = math.degrees((vl - vr) / p.track_m)
        yaw_mid = self.yaw + omega * self.dt / 2.0
        self.x += v * math.sin(math.radians(yaw_mid)) * self.dt
        self.y += v * math.cos(math.radians(yaw_mid)) * self.dt
        self.yaw = (self.yaw + omega * self.dt) % 360.0
        return v

    # --- one agent tick ---------------------------------------------------------------

    def tick(self, result: Result) -> None:
        v_now = (self.applied[0] + self.applied[1]) / 2.0 / 255.0 * self.physics.full_scale_mps
        if self.t >= self.next_fix_t:
            self._fix(v_now)
            self.next_fix_t += 1.0 / self.gps_model.rate_hz

        # 1. GPS -> position. `driven` mirrors RobotAgent._wheels_driven: the
        #    previous tick's intent moved and the controller applied forward output.
        reading = reading_from_frame(self.frame, self.frame_at, now=self.wall0 + self.t,
                                     stale_after_s=self.cfg.gps_stale_after_s)
        driven = self.intent_moving and min(self.applied) >= 0 and max(self.applied) > 0
        position = self.estimator.update(reading, driven=driven)

        # 3. Navigation, 3b. mission progress.
        navigation = self.navigator.update(position)
        update = self.missions.update(navigation, position_measured=position is not None)
        active = update.active
        if active is not None and active.status is MissionStatus.COMPLETE:
            result.outcome, result.reason = "COMPLETE", "mission complete"
            self.paused = True
        mission_active = not self.paused and (active is None or active.status.is_active)
        if self.stop_at is not None and self.t >= self.stop_at and not self.paused:
            self.missions.abandon("operator stop")
            self.navigator.clear_route()
            self.paused = True
            result.outcome, result.reason = "STOPPED", "operator stop"
            mission_active = False

        # 5. Decision, 5b. safety gate (forward range always expected).
        perception = clear_scene()
        proposed = self.decision.decide(mission_active=mission_active, navigation=navigation,
                                        perception=perception)
        cm = next((w.value for w in self.obstacles if w.active(self.t)), 400.0)
        front = RangeReading(RangeStatus.VALID, distance_cm=cm, age_s=0.1)
        safety = self.safety.evaluate(proposed, mission_active=mission_active, perception=perception,
                                      obstacle=front, range_expected=True)

        # 5b''. Bounded navigation stops.
        stall = self.progress.update(active=mission_active, navigation=navigation,
                                     moving=not safety.intent.is_stop)
        if stall.assist:
            self.paused = True
            result.pause_reasons.append(stall.reason)
            result.outcome, result.reason = "PAUSED", stall.reason
            safety = self.safety.evaluate(MotionIntent.stop(stall.reason), mission_active=False,
                                          perception=perception)
        intent = safety.intent
        self.intent_moving = not intent.is_stop
        if intent.turns:
            self.estimator.invalidate_heading()

        # 5c. What the link would write, and what the motor layer applies.
        units = self._drive(intent)
        if not intent.is_stop:
            result.drives += 1
            if min(units) < 0:
                result.reverse_or_pivot_drives += 1

        here = self.here
        target = navigation.target_waypoint
        true_err = None
        if target is not None:
            true_err = heading_error_deg(bearing_deg(here, target), self.yaw)
        est_err = navigation.heading_error_deg
        if est_err is not None and units[0] != units[1] and min(units) >= 0 and abs(est_err) > 1:
            result.sign_checked += 1
            turning_right = units[0] > units[1]
            if turning_right != (est_err > 0):
                result.sign_wrong += 1
        result.rows.append(Row(
            t=round(self.t, 2), x=self.x, y=self.y, yaw=self.yaw, left=units[0], right=units[1],
            nav=navigation.status.value, reason=intent.reason,
            heading_source=position.heading_source.value if position else "NO_POSITION",
            believed_heading=position.heading_deg if position else None,
            desired=navigation.desired_heading_deg, est_error=est_err, true_error=true_err,
            distance=navigation.distance_to_destination_m, waypoint_index=navigation.waypoint_index))

        # Physics for the next dt.
        x0, y0, yaw0 = self.x, self.y, self.yaw
        self._integrate()
        result.path_m += math.hypot(self.x - x0, self.y - y0)
        result.total_turn_deg += abs(heading_error_deg(self.yaw, yaw0))
        self.t += self.dt

    def run(self, max_s: float = 600.0, *, until_paused: bool = True) -> Result:
        result = Result(outcome="TIMEOUT", reason=f"not finished after {max_s:.0f}s", t=0.0)
        while self.t < max_s:
            self.tick(result)
            if until_paused and self.paused:
                break
            if result.outcome == "TIMEOUT" and self.navigator.state.status is NavigationStatus.ARRIVED \
                    and not self.missions.has_active_mission:
                result.outcome, result.reason = "ARRIVED", "route destination reached"
                break
        result.t = self.t
        dest = self.navigator.planner.destination()
        if dest is None and self.missions.active is not None:
            dest = self.missions.active.mission.drop
        result.final_distance_m = None if dest is None else haversine_m(self.here, dest)
        return result


def straight_route(start: LatLon, *, bearing: float, length_m: float, points: int = 3) -> List[LatLon]:
    """`points` waypoints evenly along a straight line from `start` (start excluded)."""

    n = math.cos(math.radians(bearing))
    e = math.sin(math.radians(bearing))
    return [offset(start, north_m=n * length_m * k / points, east_m=e * length_m * k / points)
            for k in range(1, points + 1)]
