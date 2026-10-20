"""Decision layer: navigation + perception -> MotionIntent.

This is the only place in the Pi agent that decides how the robot should move.
It produces intent; it does not actuate. No GPIO, no motor driver, no serial.

Safety posture (unchanged from the direct-drive controller this replaces):
anything the robot does not positively know is treated as a reason to stop.
Perception that is stale, errored, or has no frame is NOT "the path is clear" --
it is a STOP. A person in view is a STOP regardless of distance. Only an
explicitly usable perception result with a clear corridor allows forward motion.

Every intent this layer produces has both wheel commands >= 0: it stops,
holds, or drives forward (steering by slowing one side). It never pivots or
reverses -- there is no rear sensing it could rely on to do so.

Obstacle proximity is judged by bounding-box area in pixels, which is a coarse
proxy for "close", not a measurement. The camera is monocular; see
`robotx.perception.types` for why no distance is reported anywhere.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional, Tuple

from robotx.config.logging_setup import log_event
from robotx.control.motion import MotionIntent
from robotx.navigation.navigator import NavigationState, NavigationStatus
from robotx.perception.types import Detection, PerceptionResult, PerceptionStatus


logger = logging.getLogger(__name__)

PERSON_LABEL = "person"


@dataclass(frozen=True)
class DecisionConfig:
    cruise_speed: float = 0.45
    turn_speed: float = 0.35
    slow_speed: float = 0.25
    max_speed: float = 0.75

    stop_area_px: int = 30000
    slow_area_px: int = 10000
    min_area_px: int = 1500
    center_zone_ratio: float = 0.33

    # Retained for configuration compatibility; route following no longer
    # steers proportionally (see `_follow_route`).
    steering_full_scale_deg: float = 45.0
    # Below this, do not bother steering -- GPS heading is not precise enough
    # to chase small errors, and doing so just weaves.
    steering_deadband_deg: float = 5.0
    # Bounded turns: the yaw rate assumed while one side drives at cruise speed
    # and the other is stopped, which sizes each turn from a measured error.
    # An assumption until measured on the ground. Too high under-turns (more
    # turns, still converging); more than twice too low overshoots each time.
    turn_rate_dps: float = 90.0
    # No single turn lasts longer than this.
    max_turn_s: float = 2.0

    def __post_init__(self) -> None:
        if not 0 < self.turn_rate_dps <= 720:
            raise ValueError(f"turn_rate_dps must be in (0, 720], got {self.turn_rate_dps}")
        if not 0 < self.max_turn_s <= 10:
            raise ValueError(f"max_turn_s must be in (0, 10], got {self.max_turn_s}")

    @classmethod
    def from_settings(cls, settings: Any) -> "DecisionConfig":
        return cls(
            cruise_speed=settings.motion_cruise_speed,
            turn_speed=settings.motion_turn_speed,
            slow_speed=settings.motion_slow_speed,
            max_speed=settings.motion_max_speed,
            stop_area_px=settings.obstacle_stop_area_px,
            slow_area_px=settings.obstacle_slow_area_px,
            min_area_px=settings.obstacle_min_area_px,
            center_zone_ratio=settings.obstacle_center_zone_ratio,
            steering_full_scale_deg=settings.steering_full_scale_deg,
            turn_rate_dps=settings.nav_turn_rate_dps,
            max_turn_s=settings.nav_max_turn_s,
        )


@dataclass(frozen=True)
class _Turn:
    """One bounded turn in progress: which way, until when, toward what."""

    steer: float
    until: float
    target: Any
    error_deg: float


def center_zone(frame_width: int, ratio: float) -> Tuple[int, int]:
    """Horizontal pixel band the robot would drive through, as (left, right)."""

    width = max(1, int(frame_width))
    band = max(0.0, min(1.0, float(ratio))) * width
    left = int(round((width - band) / 2.0))
    right = int(round((width + band) / 2.0))
    return left, right


class DecisionMaker:
    """Produces the motion intent for one agent tick."""

    def __init__(
        self,
        cfg: Optional[DecisionConfig] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.cfg = cfg or DecisionConfig()
        self.clock = clock
        self._last_reason: Optional[str] = None
        self._turn: Optional[_Turn] = None
        self._turned = False

    def decide(
        self,
        *,
        mission_active: bool,
        navigation: NavigationState,
        perception: PerceptionResult,
    ) -> MotionIntent:
        self._turned = False
        intent = self._decide(
            mission_active=mission_active, navigation=navigation, perception=perception
        )
        # A turn lasts only while route following keeps asking for it: any
        # stop or hold ends it, and what follows starts from a new measurement.
        if not self._turned:
            self._turn = None
        # Log on change only: this runs at the agent loop rate.
        if intent.reason != self._last_reason:
            log_event(
                logger,
                "decision.changed",
                command=intent.command.value,
                reason=intent.reason,
                left=round(intent.left, 2),
                right=round(intent.right, 2),
            )
            self._last_reason = intent.reason
        return intent

    def screen(self, perception: PerceptionResult) -> Optional[MotionIntent]:
        """The camera's own stop, independent of any route, or None.

        The same checks `decide` makes before driving -- perception usable, no
        person in view, nothing in the drive corridor -- for a caller that has
        no route but must not drive past what the camera sees (the bench pulse,
        `robotx.control.bench_motion`). Never a reason to move.
        """

        if not perception.is_usable:
            detail = "no frame metadata" if perception.status is PerceptionStatus.OK else perception.status.value
            return MotionIntent.stop(f"perception unavailable ({detail})")
        if perception.has_label(PERSON_LABEL):
            return MotionIntent.stop("person detected")
        obstacle = self._blocking_obstacle(perception)
        if obstacle is not None:
            return self._avoid(obstacle, perception)
        return None

    def _decide(
        self,
        *,
        mission_active: bool,
        navigation: NavigationState,
        perception: PerceptionResult,
    ) -> MotionIntent:
        if not mission_active:
            return MotionIntent.hold("no active mission")

        # Perception must be positively usable before anything moves.
        if not perception.is_usable:
            # An OK result with no frame metadata is still unusable: the
            # corridor check below needs the frame width.
            detail = (
                "no frame metadata"
                if perception.status is PerceptionStatus.OK
                else perception.status.value
            )
            return MotionIntent.stop(f"perception unavailable ({detail})")

        if perception.has_label(PERSON_LABEL):
            return MotionIntent.stop("person detected")

        # Navigation must know where to go and where the robot is.
        if navigation.status is NavigationStatus.IDLE:
            return MotionIntent.hold("no route loaded")
        if navigation.status is NavigationStatus.NO_POSITION:
            if navigation.position_issue is not None:
                # Fixed text: the issue carries a live accuracy figure, and the
                # reason is logged on every change.
                return MotionIntent.stop("GPS fix not accurate enough to navigate")
            return MotionIntent.stop("no GPS position")
        if navigation.status is NavigationStatus.ARRIVED:
            return MotionIntent.hold("destination reached")
        if navigation.status is NavigationStatus.REROUTE_NEEDED:
            return MotionIntent.stop("off route: reroute needed")

        obstacle = self._blocking_obstacle(perception)
        if obstacle is not None:
            return self._avoid(obstacle, perception)

        return self._follow_route(navigation, perception)

    # --- obstacle handling ---------------------------------------------------

    def _blocking_obstacle(self, perception: PerceptionResult) -> Optional[Detection]:
        """Largest detection inside the drive corridor, if it is big enough."""

        if perception.frame is None:
            return None

        left, right = center_zone(perception.frame.width, self.cfg.center_zone_ratio)
        candidates = [
            d
            for d in perception.detections
            if d.area_px >= self.cfg.min_area_px and left <= d.cx <= right
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda d: d.area_px)

    def _avoid(self, obstacle: Detection, perception: PerceptionResult) -> MotionIntent:
        """Every obstacle in the drive corridor is a STOP (RX-09).

        There is deliberately no avoidance manoeuvre. Turning away in place
        drives one side backwards, which the ESP32 refuses -- and reports as a
        fault -- when its rear sensing is not configured or not healthy; and a
        pivot is invisible to the GPS heading, so the route is lost after it.
        The rover waits instead: `robotx.control.obstacle_recovery` decides when
        the path has been clear long enough to move on, and when to ask for an
        operator. Every reason here contains "obstacle": the agent relies on it.
        """

        if obstacle.area_px >= self.cfg.stop_area_px:
            return MotionIntent.stop(
                f"obstacle ahead (area={obstacle.area_px}px)"
            )

        # Growing fast: treat as closing, even below the stop area. This is a
        # trend, not a measured closing speed.
        if perception.largest_area_delta_px >= self.cfg.slow_area_px:
            return MotionIntent.stop("obstacle growing rapidly")

        frame_width = perception.frame.width if perception.frame else 1
        side = "left of centre" if obstacle.cx < frame_width / 2 else "right of centre"
        return MotionIntent.stop(f"obstacle in path, {side} (area={obstacle.area_px}px)")

    # --- route following -----------------------------------------------------

    def _follow_route(
        self, navigation: NavigationState, perception: PerceptionResult
    ) -> MotionIntent:
        # Something sizeable but off to the side: proceed slowly.
        largest = perception.largest
        cautious = largest is not None and largest.area_px >= self.cfg.slow_area_px
        speed = self.cfg.slow_speed if cautious else self.cfg.cruise_speed
        speed = min(speed, self.cfg.max_speed)

        suffix = " (cautious)" if cautious else ""
        now = self.clock()
        error = navigation.heading_error_deg
        turn = self._turn
        if turn is not None and (turn.target != navigation.target_waypoint or now >= turn.until):
            turn = self._turn = None
        if turn is not None and error is None:
            # Mid-turn the heading is gone (the agent drops it on every steered
            # tick), so the turn runs on what it was sized from. A heading that
            # is present is newer than that -- a fix taken during the turn, or
            # a turn the gate held back -- and re-sizes it below.
            self._turned = True
            return MotionIntent.forward(
                speed, steer=turn.steer,
                reason=f"turning toward route (error={turn.error_deg:.0f}deg)" + suffix,
            )

        if error is None:
            # No heading yet (stationary, no compass, or just turned). Creep
            # forward so GPS can derive a track; steering is impossible until
            # it does.
            return MotionIntent.forward(
                self.cfg.slow_speed, reason="no heading: creeping to establish GPS track"
            )

        if abs(error) <= self.cfg.steering_deadband_deg:
            return MotionIntent.forward(speed, reason="on course" + suffix)

        # One bounded turn per measured error. The heading is GPS course over
        # ground: it describes the direction the rover last moved, and it is
        # measured again only after metres of driving (or each second above
        # walking pace). Steering on it continuously means turning for all that
        # time on a figure that does not change -- the rover circles. So the
        # error is converted into a turn of the duration the assumed yaw rate
        # needs, after which the agent drops the heading and the rover creeps
        # straight to measure the new one.
        #
        # The turn is made moving forward -- one side driven, the other
        # stopped -- never by rotating in place or reversing (no rear sensing).
        rate = self.cfg.turn_rate_dps * speed / max(1e-6, self.cfg.cruise_speed)
        duration = min(self.cfg.max_turn_s, abs(error) / max(1e-6, rate))
        steer = 1.0 if error > 0 else -1.0
        self._turn = _Turn(steer=steer, until=now + duration,
                           target=navigation.target_waypoint, error_deg=error)
        self._turned = True
        return MotionIntent.forward(
            speed, steer=steer, reason=f"following route (error={error:.0f}deg)" + suffix,
        )
