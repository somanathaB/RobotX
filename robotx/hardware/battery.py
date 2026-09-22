"""Battery sensing -- which this robot does not have.

There is no fuel-gauge IC, no ADC and no voltage divider on this robot: no
battery-sensing hardware appears in the wiring documentation, the pin
configuration, or anywhere in this codebase. In the target architecture any
such sensor would sit on the ESP32 and arrive over the Pi link, which does not
exist yet.

This module exists so that there is exactly one answer to "what is the battery
doing", and that answer is honestly "unknown". An earlier version of this
repository reported a hardcoded `76.0` percent in telemetry as though it were a
reading (remediation item R-04); a consumer had no way to tell it was invented.

When real hardware is added, replace `battery_status()` with a reader for that
specific device and keep the same `{status, percent, voltage_v, source}` shape,
so every consumer keeps working and can still tell measured from unavailable.
"""

from __future__ import annotations

from typing import Any, Dict


BATTERY_UNAVAILABLE_REASON = "no battery sensing hardware on this robot"


def battery_status() -> Dict[str, Any]:
    """Current battery state. Always UNAVAILABLE until sensing hardware exists."""

    return {
        "status": "UNAVAILABLE",
        "percent": None,
        "voltage_v": None,
        "source": BATTERY_UNAVAILABLE_REASON,
    }
