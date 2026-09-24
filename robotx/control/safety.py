"""Safety gate: the last authority between a motion intent and the ESP32.

Navigation and the decision layer produce what the robot *wants* to do. This
module decides whether that request is allowed to leave the Pi at all:

    Navigation -> MotionIntent -> SafetyGate -> ESP32 -> Motors

It is deliberately separate from `robotx.control.decision`, which already
declines to drive when perception is unusable. That is not duplication, it is
defence in depth. The decision layer is where driving *policy* lives and it
will keep changing as navigation improves; this gate is a short, fixed set of
rules that a policy bug cannot talk its way past. A gate that trusted the layer
it is gating would not be a gate.

Fail closed
-----------
Every rule answers one question: is there positive evidence that moving is
safe? Absence of evidence is never read as safety. A stale intent, an unusable
perception result and a range sensor that stopped answering all veto motion,
exactly as a measured obstacle does.

The gate can only ever *reduce* motion. It vetoes down to a stop or clamps a
speed; it has no path that creates motion or increases it. So no failure of
this module -- including an unhandled exception in a caller that drops the
result -- can make the robot drive faster or further than the decision layer
asked for.

What this gate is not
---------------------
It is not the robot's only safety mechanism, and treating it as one would be a
mistake. The Pi runs a multi-hundred-millisecond perception loop on a
non-realtime OS, so it cannot promise a bounded reaction time. The hardware
failsafe -- motor cutoff, watchdog, current limit -- belongs to the ESP32 and
is deliberately out of scope here.

This module touches no GPIO, no serial port and no socket. It is pure: given
the same inputs it returns the same verdict, which is what makes a safety rule
testable without a robot.
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, replace
from enum import Enum
from typing import Any, Dict, Optional

from robotx.config.logging_setup import log_event
from robotx.control.motion import MotionIntent, age_s
from robotx.perception.types import PerceptionResult


logger = logging.getLogger(__name__)


class RangeStatus(str, Enum):
    """What a forward range sensor is currently able to tell us.

    The values deliberately mirror `robotx.hardware.ultrasonic.UltrasonicStatus`,
    and that module is deliberately *not* imported to get them. Importing it
    would pull `RPi.GPIO` into the agent's import graph, which the architecture
    forbids and a regression test enforces -- the Pi agent must not be able to
    reach a motor or a GPIO pin even transitively.

    So the dependency runs the other way round: this is the source-agnostic
    contract the safety layer reasons about, and a driver -- an HC-SR04 on
    GPIO, a sensor read over the ESP32 link, a ToF module -- is whatever
    produces one. The gate does not care which.

    The one distinction that matters, and the reason this is a status rather
    than a bare `Optional[float]`: "the path is clear" and "the sensor did not
    answer" must stay separable all the way to the safety rule. Collapsing them
    is how a robot drives into something its broken sensor never mentioned.
    """

    VALID = "VALID"                # fresh, in-range measurement
    TIMEOUT = "TIMEOUT"            # triggered, no echo came back in time
    OUT_OF_RANGE = "OUT_OF_RANGE"  # an implausible reading
    ERROR = "ERROR"                # the sensor raised while measuring
    DISCONNECTED = "DISCONNECTED"  # no sensor interface available
    STALE = "STALE"                # nothing produced recently; producer stalled
    UNKNOWN = "UNKNOWN"            # nothing measured yet

    @property
    def is_measurement(self) -> bool:
        """Whether this status carries an actual distance to reason about."""

        return self is RangeStatus.VALID


@dataclass(frozen=True)
class RangeReading:
    """One forward-range observation, from whatever sensor produced it."""

    status: RangeStatus
    distance_cm: Optional[float] = None
    age_s: Optional[float] = None


class SafetyVerdict(str, Enum):
    """What the gate did to the intent it was given."""

    ALLOWED = "ALLOWED"  # forwarded unchanged
    CLAMPED = "CLAMPED"  # forwarded, scaled down to the hard speed ceiling
    VETOED = "VETOED"    # replaced with a stop


@dataclass(frozen=True)
class SafetyDecision:
    """The gate's output: the only intent that may be transmitted, and why."""

    intent: MotionIntent
    verdict: SafetyVerdict
    # Which rule decided. A short stable slug rather than prose, so telemetry
    # and logs can be grouped by cause without parsing a sentence.
    rule: str
    reason: str

    @property
    def blocked(self) -> bool:
        return self.verdict is SafetyVerdict.VETOED

    def to_dict(self) -> Dict[str, Any]:
        return {
            "verdict": self.verdict.value,
            "rule": self.rule,
            "reason": self.reason,
        }


# The verdict reported before the gate has run even once. VETOED, not ALLOWED:
# a robot that has not yet been checked has not been cleared.
UNEVALUATED = SafetyDecision(
    intent=MotionIntent.hold("safety gate has not run yet"),
    verdict=SafetyVerdict.VETOED,
    rule="unevaluated",
    reason="no safety evaluation has run yet",
)


@dataclass(frozen=True)
class SafetyConfig:
    """Limits the gate enforces, independently of any other layer's config."""

    # Hard speed ceiling. Separate from `DecisionConfig.max_speed` on purpose:
    # a mis-set decision speed must not be able to raise the real limit.
    max_speed: float = 0.75

    # An intent older than this describes a world that has moved on. At the
    # default 5 Hz agent loop a healthy intent is ~0.2 s old, so 1.0 s means
    # roughly five missed ticks before motion is refused.
    max_intent_age_s: float = 1.0

    # Measured forward range at or inside which motion is refused outright.
    stop_distance_cm: float = 30.0

    # Whether a forward range sensor is expected to be present and reporting.
    #
    # False today, and that is a statement about hardware rather than about
    # risk appetite: this rover has no working range sensor (the HC-SR04 driver
    # cannot run on a Pi 5, and the ESP32 link does not exist), so a gate that
    # vetoed every intent for a missing sensor would just be a rover that never
    # moves. Set this true the moment one is actually wired -- from then on a
    # silent sensor stops the robot instead of being ignored.
    require_range_sensor: bool = False

    @classmethod
    def from_settings(cls, settings: Any) -> "SafetyConfig":
        return cls(
            max_speed=settings.safety_max_speed,
            max_intent_age_s=settings.safety_max_intent_age_s,
            stop_distance_cm=settings.safety_stop_distance_cm,
            require_range_sensor=settings.safety_require_range_sensor,
        )


class SafetyGate:
    """Applies the safety rules to one proposed motion intent.

    Holds exactly one piece of mutable state -- the emergency-stop latch --
    which is guarded because it is set from a different thread than the one
    that reads it: a backend socket callback engages it, the agent loop sees it.
    """

    def __init__(self, cfg: Optional[SafetyConfig] = None) -> None:
        self.cfg = cfg or SafetyConfig()
        self._lock = threading.Lock()
        self._estop_reason: Optional[str] = None
        self._last_rule: Optional[str] = None

    # --- emergency stop -------------------------------------------------------

    def engage_estop(self, reason: str = "emergency stop") -> None:
        """Latch the emergency stop. Nothing moves until it is explicitly cleared.

        Latching is the whole point. A condition that stops the robot for one
        tick and lets it drive again on the next is not an emergency stop, it
        is a stutter -- so this never clears itself, no matter what the sensors
        subsequently say.
        """

        with self._lock:
            already = self._estop_reason is not None
            self._estop_reason = reason

        # Log the transition, not every re-assertion: a backend retrying a stop
        # must not be able to flood the log.
        if not already:
            log_event(logger, "safety.estop_engaged", reason, level=logging.CRITICAL)

    def clear_estop(self, reason: str = "operator clear") -> bool:
        """Release the latch. Returns whether it had actually been engaged."""

        with self._lock:
            was_engaged = self._estop_reason is not None
            self._estop_reason = None

        if was_engaged:
            log_event(logger, "safety.estop_cleared", reason, level=logging.WARNING)
        return was_engaged

    @property
    def estop_engaged(self) -> bool:
        with self._lock:
            return self._estop_reason is not None

    @property
    def estop_reason(self) -> Optional[str]:
        with self._lock:
            return self._estop_reason

    # --- evaluation -----------------------------------------------------------

    def evaluate(
        self,
        intent: MotionIntent,
        *,
        mission_active: bool,
        perception: PerceptionResult,
        obstacle: Optional[RangeReading] = None,
        now: Optional[float] = None,
    ) -> SafetyDecision:
        """Decide what, if anything, may be sent downstream.

        `perception` is required rather than optional: every caller already has
        one (a disabled pipeline still produces an explicit `DISABLED` result),
        and making it omissible would create a way to skip a safety check by
        forgetting an argument.

        `obstacle` is optional because a range sensor genuinely may not be
        fitted. See `SafetyConfig.require_range_sensor` for how that is handled
        rather than assumed.
        """

        decision = self._evaluate(
            intent,
            mission_active=mission_active,
            perception=perception,
            obstacle=obstacle,
            now=now,
        )

        # This runs at the agent loop rate, so log only when the deciding rule
        # changes. A veto that persists for a minute is one line, not three
        # hundred.
        if decision.rule != self._last_rule:
            log_event(
                logger,
                "safety.verdict",
                decision.reason,
                level=(logging.WARNING if decision.blocked else logging.INFO),
                verdict=decision.verdict.value,
                rule=decision.rule,
                requested=intent.command.value,
            )
            self._last_rule = decision.rule

        return decision

    def _evaluate(
        self,
        intent: MotionIntent,
        *,
        mission_active: bool,
        perception: PerceptionResult,
        obstacle: Optional[RangeReading],
        now: Optional[float],
    ) -> SafetyDecision:
        # Rules are ordered by authority, and the first match wins.

        estop = self.estop_reason
        if estop is not None:
            return self._veto("estop", f"emergency stop engaged: {estop}")

        # A request to stop is always safe to forward, and must be forwarded:
        # the ESP32 cannot act on a stop the gate swallowed. Checked ahead of
        # every remaining rule so a robot that is already stopping is never
        # held up by a rule about moving.
        if intent.is_stop:
            return SafetyDecision(
                intent=intent,
                verdict=SafetyVerdict.ALLOWED,
                rule="stop",
                reason=intent.reason or "stop requested",
            )

        # Everything below here concerns an intent that would actually move.

        if not mission_active:
            return self._veto("mode", "motion requested with no active mission")

        age = age_s(intent, now)
        if age > self.cfg.max_intent_age_s:
            return self._veto(
                "stale_intent",
                f"intent is {age:.2f}s old (limit {self.cfg.max_intent_age_s:.2f}s)",
            )

        range_veto = self._range_veto(obstacle)
        if range_veto is not None:
            return range_veto

        if not perception.is_usable:
            return self._veto(
                "perception", f"perception is not usable ({perception.status.value})"
            )

        peak = max(abs(intent.left), abs(intent.right))
        if peak > self.cfg.max_speed:
            return self._clamp(intent, peak)

        return SafetyDecision(
            intent=intent,
            verdict=SafetyVerdict.ALLOWED,
            rule="clear",
            reason=intent.reason or "no safety condition applies",
        )

    def _range_veto(self, obstacle: Optional[RangeReading]) -> Optional[SafetyDecision]:
        """Veto from the forward range sensor, if there is one.

        Three cases, kept apart deliberately:

        - a VALID reading at or inside the stop distance is a *measured*
          obstacle, and always vetoes;
        - any other status means the sensor did not answer, which vetoes only
          when a sensor is expected -- "the sensor is broken" must never be
          read as "the path is clear";
        - no reading at all means no sensor is fitted, which cannot veto unless
          one was required. The gate must not invent a sensor this rover does
          not have.
        """

        if obstacle is None:
            if self.cfg.require_range_sensor:
                return self._veto(
                    "range_missing", "a forward range sensor is required but none reported"
                )
            return None

        if obstacle.status is RangeStatus.VALID and obstacle.distance_cm is not None:
            if obstacle.distance_cm <= self.cfg.stop_distance_cm:
                # The word "obstacle" in this reason is load-bearing: the agent
                # feeds obstacle-caused stops back into route planning.
                return self._veto(
                    "obstacle",
                    f"obstacle at {obstacle.distance_cm:.0f}cm "
                    f"(limit {self.cfg.stop_distance_cm:.0f}cm)",
                )
            return None

        if self.cfg.require_range_sensor:
            return self._veto(
                "range_unreliable",
                f"forward range sensor is not reporting ({obstacle.status.value})",
            )
        return None

    # --- outcomes -------------------------------------------------------------

    def _veto(self, rule: str, reason: str) -> SafetyDecision:
        return SafetyDecision(
            intent=MotionIntent.stop(f"safety: {reason}"),
            verdict=SafetyVerdict.VETOED,
            rule=rule,
            reason=reason,
        )

    def _clamp(self, intent: MotionIntent, peak: float) -> SafetyDecision:
        """Scale an over-speed intent down to the ceiling, preserving its shape.

        Both sides are scaled by the same factor rather than clipped
        independently. Clipping each side would quietly change the ratio
        between them, which *is* the turn -- a robot asked to arc gently could
        be clipped into a much sharper one. Scaling slows the manoeuvre without
        altering it.

        The timestamp is carried over unchanged: this is still the same
        observation the decision layer made, and staleness must keep being
        measured from when it was decided rather than from when it was clamped.
        """

        scale = self.cfg.max_speed / peak
        return SafetyDecision(
            intent=replace(
                intent,
                left=intent.left * scale,
                right=intent.right * scale,
                reason=f"{intent.reason} [safety: clamped to {self.cfg.max_speed:.2f}]",
            ),
            verdict=SafetyVerdict.CLAMPED,
            rule="speed_clamp",
            reason=f"speed {peak:.2f} exceeds ceiling {self.cfg.max_speed:.2f}",
        )
