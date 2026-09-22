"""Decision layer: navigation + perception -> MotionIntent.

This is the only place in the Pi agent that decides how the robot should move.
It produces intent; it does not actuate. No GPIO, no motor driver, no serial.

Safety posture (unchanged from the direct-drive controller this replaces):
anything the robot does not positively know is treated as a reason to stop.
Perception that is stale, errored, or has no frame is NOT "the path is clear" --
it is a STOP. A person in view is a STOP regardless of distance. Only an
explicitly usable perception result with a clear corridor allows forward motion.

Obstacle proximity is judged by bounding-box area in pixels, which is a coarse
proxy for "close", not a measurement. The camera is monocular; see
`robotx.perception.types` for why no distance is reported anywhere.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Optional, Tuple

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

    steering_full_scale_deg: float = 45.0
    # Below this, do not bother steering -- GPS heading is not precise enough
    # to chase small errors, and doing so just weaves.
    steering_deadband_deg: float = 5.0

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
        )


def center_zone(frame_width: int, ratio: float) -> Tuple[int, int]:
    """Horizontal pixel band the robot would drive through, as (left, right)."""

    width = max(1, int(frame_width))
    band = max(0.0, min(1.0, float(ratio))) * width
    left = int(round((width - band) / 2.0))
    right = int(round((width + band) / 2.0))
    return left, right


class DecisionMaker:
    """Produces the motion intent for one agent tick."""

    def __init__(self, cfg: Optional[DecisionConfig] = None) -> None:
        self.cfg = cfg or DecisionConfig()
        self._last_reason: Optional[str] = None

    def decide(
        self,
        *,
        mission_active: bool,
        navigation: NavigationState,
        perception: PerceptionResult,
    ) -> MotionIntent:
        intent = self._decide(
            mission_active=mission_active, navigation=navigation, perception=perception
        )
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
        if obstacle.area_px >= self.cfg.stop_area_px:
            return MotionIntent.stop(
                f"obstacle ahead (area={obstacle.area_px}px)"
            )

        # Growing fast: treat as closing, even below the stop area. This is a
        # trend, not a measured closing speed.
        if perception.largest_area_delta_px >= self.cfg.slow_area_px:
            return MotionIntent.stop("obstacle growing rapidly")

        # Turn away from whichever side the obstacle sits on.
        frame_width = perception.frame.width if perception.frame else 1
        if obstacle.cx < frame_width / 2:
            return MotionIntent.turn_right(
                self.cfg.turn_speed, f"obstacle left (area={obstacle.area_px}px)"
            )
        return MotionIntent.turn_left(
            self.cfg.turn_speed, f"obstacle right (area={obstacle.area_px}px)"
        )

    # --- route following -----------------------------------------------------

    def _follow_route(
        self, navigation: NavigationState, perception: PerceptionResult
    ) -> MotionIntent:
        # Something sizeable but off to the side: proceed slowly.
        largest = perception.largest
        cautious = largest is not None and largest.area_px >= self.cfg.slow_area_px
        speed = self.cfg.slow_speed if cautious else self.cfg.cruise_speed
        speed = min(speed, self.cfg.max_speed)

        error = navigation.heading_error_deg
        if error is None:
            # No heading yet (stationary, no compass). Creep forward so GPS can
            # derive a track; steering is impossible until it does.
            return MotionIntent.forward(
                self.cfg.slow_speed, reason="no heading: creeping to establish GPS track"
            )

        if abs(error) <= self.cfg.steering_deadband_deg:
            return MotionIntent.forward(
                speed, reason="on course" + (" (cautious)" if cautious else "")
            )

        # Large heading error: rotate in place rather than driving a wide arc.
        if abs(error) >= 90.0:
            if error > 0:
                return MotionIntent.turn_right(
                    self.cfg.turn_speed, f"turning to course (error={error:.0f}deg)"
                )
            return MotionIntent.turn_left(
                self.cfg.turn_speed, f"turning to course (error={error:.0f}deg)"
            )

        steer = max(-1.0, min(1.0, error / max(1.0, self.cfg.steering_full_scale_deg)))
        return MotionIntent.forward(
            speed,
            steer=steer,
            reason=f"following route (error={error:.0f}deg)"
            + (" (cautious)" if cautious else ""),
        )
