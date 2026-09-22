"""Motion intent: what the Pi *wants* to happen. Not a motor command.

In the target architecture the ESP32 owns motor authority. The Pi's decision
layer produces a `MotionIntent`; something downstream decides whether to honour
it, and at what PWM. Nothing in this module touches GPIO, and nothing should
ever make it do so.

Velocities are normalized to [-1.0, 1.0] rather than duty cycles or PWM counts.
The Pi does not know the gear ratio, the battery voltage, or the H-bridge's
duty ceiling, so it must not pretend to speak in those units -- converting
`left=0.45` into a duty cycle is the motor controller's job, and keeping it
there means recalibrating the drivetrain does not require touching Pi code.

This is the insertion point for the future Pi → ESP32 link: a transport would
serialize `MotionIntent.to_dict()` (or a compact binary form of the same
fields) and send it over UART. That transport is deliberately not implemented
here.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


class MotionCommand(str, Enum):
    """The coarse shape of the request, for logging and telemetry."""

    STOP = "STOP"              # come to rest now
    HOLD = "HOLD"              # stay at rest (no mission active)
    FORWARD = "FORWARD"        # drive forward, possibly steering
    REVERSE = "REVERSE"        # drive backward
    TURN_LEFT = "TURN_LEFT"    # rotate left
    TURN_RIGHT = "TURN_RIGHT"  # rotate right


@dataclass(frozen=True)
class MotionIntent:
    """A request from the Pi's decision layer, in normalized units."""

    command: MotionCommand = MotionCommand.STOP
    left: float = 0.0   # normalized left-side velocity, [-1.0, 1.0]
    right: float = 0.0  # normalized right-side velocity, [-1.0, 1.0]
    reason: str = ""    # why this intent was produced; carried into telemetry
    timestamp: float = field(default_factory=time.time)

    def __post_init__(self) -> None:
        # Frozen dataclass: clamp through object.__setattr__ so an out-of-range
        # value can never leave this module.
        object.__setattr__(self, "left", _clamp(self.left, -1.0, 1.0))
        object.__setattr__(self, "right", _clamp(self.right, -1.0, 1.0))

    @property
    def is_stop(self) -> bool:
        return self.command in (MotionCommand.STOP, MotionCommand.HOLD)

    @property
    def linear(self) -> float:
        """Mean forward velocity. Negative means reversing."""

        return (self.left + self.right) / 2.0

    @property
    def angular(self) -> float:
        """Turn rate: positive turns right, negative turns left."""

        return (self.left - self.right) / 2.0

    # --- constructors --------------------------------------------------------

    @classmethod
    def stop(cls, reason: str = "") -> "MotionIntent":
        return cls(command=MotionCommand.STOP, left=0.0, right=0.0, reason=reason)

    @classmethod
    def hold(cls, reason: str = "") -> "MotionIntent":
        return cls(command=MotionCommand.HOLD, left=0.0, right=0.0, reason=reason)

    @classmethod
    def forward(cls, speed: float, *, steer: float = 0.0, reason: str = "") -> "MotionIntent":
        """Drive forward at `speed`, steering with `steer` in [-1, 1].

        Positive `steer` turns right: the right side is slowed relative to the
        left. Steering scales the difference, never pushing either side above
        the requested speed.
        """

        speed = _clamp(speed, 0.0, 1.0)
        steer = _clamp(steer, -1.0, 1.0)
        left = speed
        right = speed
        if steer > 0:
            right = speed * (1.0 - steer)
        elif steer < 0:
            left = speed * (1.0 + steer)
        return cls(command=MotionCommand.FORWARD, left=left, right=right, reason=reason)

    @classmethod
    def reverse(cls, speed: float, reason: str = "") -> "MotionIntent":
        speed = _clamp(speed, 0.0, 1.0)
        return cls(command=MotionCommand.REVERSE, left=-speed, right=-speed, reason=reason)

    @classmethod
    def turn_left(cls, speed: float, reason: str = "") -> "MotionIntent":
        speed = _clamp(speed, 0.0, 1.0)
        return cls(command=MotionCommand.TURN_LEFT, left=-speed, right=speed, reason=reason)

    @classmethod
    def turn_right(cls, speed: float, reason: str = "") -> "MotionIntent":
        speed = _clamp(speed, 0.0, 1.0)
        return cls(command=MotionCommand.TURN_RIGHT, left=speed, right=-speed, reason=reason)

    # --- serialization -------------------------------------------------------

    def to_dict(self) -> Dict[str, Any]:
        return {
            "command": self.command.value,
            "left": round(self.left, 3),
            "right": round(self.right, 3),
            "linear": round(self.linear, 3),
            "angular": round(self.angular, 3),
            "reason": self.reason,
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MotionIntent":
        try:
            command = MotionCommand(str(data.get("command", "STOP")))
        except ValueError:
            command = MotionCommand.STOP
        return cls(
            command=command,
            left=float(data.get("left", 0.0) or 0.0),
            right=float(data.get("right", 0.0) or 0.0),
            reason=str(data.get("reason", "")),
            timestamp=float(data.get("timestamp", time.time())),
        )


STOPPED = MotionIntent.stop("initial state")


def age_s(intent: MotionIntent, now: Optional[float] = None) -> float:
    """How long ago this intent was produced. A stale intent must not be obeyed."""

    return max(0.0, (time.time() if now is None else now) - intent.timestamp)
