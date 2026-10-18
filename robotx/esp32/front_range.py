"""The ESP32's front ultrasonic pair, as the Pi safety gate's forward range (RX-07).

The ESP32 already gates every forward wheel against these sensors and latches
`safety_stop` when it has to (ESP32 repo `safety.cpp`, `comm.cpp`). A latch
needs an operator RESET. The Pi reads the same TELEMETRY and stops *first*,
further out, so an ordinary obstacle is an ordinary stop on the Pi -- held and
released by `robotx.control.obstacle_recovery` -- and the ESP32 gate stays the
last resort it is meant to be. Nothing here can make the rover move: it only
produces a `RangeReading` for `SafetyGate.evaluate`, which can only veto.

TELEMETRY contract (PROTOCOL.md section 11, firmware `comm.cpp`
`sendFastTelemetry`, `safety.cpp`), every 200 ms:

- `front_left_cm` / `front_right_cm`: each sensor's median-filtered distance
  in cm, uncalibrated, or null when that sensor has had no good sample for
  1 s. A number may be published while that sensor is not yet valid.
- `front_valid`: BOTH sensors valid (firmware SAFETY_REQUIRE_BOTH_SENSORS 1).
  A missing echo is invalid, never clear.
- `front_obstacle`: the firmware's confirmed obstacle latch (sets at
  SAFETY_STOP_DISTANCE_CM, releases past SAFETY_CLEAR_DISTANCE_CM). An invalid
  sensor cannot clear it.
- `forward_blocked`: `front_obstacle || !front_valid`.

Only a fresh frame from an UP link, with both sensors valid, both distances
present and finite, and nothing blocked, is VALID. A reported obstacle or a
near distance is always passed on so the gate stops on it; everything else is
INVALID, STALE or UNKNOWN -- never clear.
"""

from __future__ import annotations

import math
from typing import Optional

from robotx.control.safety import RangeReading, RangeStatus
from robotx.esp32.state import ControllerTelemetry


# The ESP32's own hard front stop: ESP32 repo config.h SAFETY_STOP_DISTANCE_CM
# (40.0 at firmware f10bb8f). TELEMETRY does not carry it, so it is restated
# here; the Pi's stop distance must be strictly beyond it.
ESP32_FRONT_STOP_CM = 40.0


def check_stop_distance(stop_distance_cm: float) -> None:
    """Refuse a Pi stop distance that is not strictly beyond the ESP32's gate.

    At or inside it, the ESP32 would always gate and latch first, and every
    ordinary obstacle would need an operator RESET.
    """

    if not (math.isfinite(stop_distance_cm) and stop_distance_cm > ESP32_FRONT_STOP_CM):
        raise ValueError(
            f"ROBOTX_SAFETY_STOP_DISTANCE_CM must be greater than the ESP32's "
            f"{ESP32_FRONT_STOP_CM:.0f} cm front stop, got {stop_distance_cm!r}"
        )


def _finite(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(value)


def front_range(
    telemetry: Optional[ControllerTelemetry],
    telemetry_age_s: Optional[float],
    *,
    link_up: bool,
    stale_after_s: float,
) -> RangeReading:
    """The forward range one ESP32 TELEMETRY frame vouches for. Pure.

    `telemetry_age_s` is the link's monotonic age of that frame; `link_up` and
    `stale_after_s` are the link's own freshness verdict and limit, so a frame
    the link no longer believes is never read here either.
    """

    if telemetry is None or telemetry_age_s is None:
        return RangeReading(RangeStatus.UNKNOWN)
    if not link_up or telemetry_age_s > stale_after_s:
        # A stale number describes a world that has moved on: none is passed.
        return RangeReading(RangeStatus.STALE, age_s=telemetry_age_s)

    left, right = telemetry.front_left_cm, telemetry.front_right_cm
    present = [d for d in (left, right) if _finite(d)]
    nearest = min(present) if present else None
    trusted = (
        telemetry.front_valid
        and _finite(left)
        and _finite(right)
        and not telemetry.forward_blocked
    )
    status = RangeStatus.VALID if trusted else RangeStatus.INVALID
    return RangeReading(
        status,
        distance_cm=nearest,
        age_s=telemetry_age_s,
        obstacle_reported=telemetry.front_obstacle,
    )
