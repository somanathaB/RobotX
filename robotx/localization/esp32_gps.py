"""The rover's GPS, as the ESP32 reports it (PROTOCOL.md section 11).

The u-blox receiver is wired to the ESP32's I2C bus (0x42), not to the Pi. The
ESP32 polls UBX-NAV-PVT about once a second and sends a `GPS` frame whatever
the receiver's state. This adapter turns the latest frame into the same
`GpsReading` the Pi's own NMEA reader produces, so localization, navigation
and telemetry need no second code path.

What is carried, and what is not:

- `lat`/`lon` only when the ESP32 says `gps_status: OK` (the firmware nulls
  them otherwise), stamped with the instant the PVT was received, not the
  instant the Pi read the frame.
- `fix_type` in the backend's vocabulary: u-blox 2 -> "2D", 3 -> "3D".
  4 (GNSS + dead reckoning) is reported as a fix with NO fix type: it is part
  dead-reckoned, and the backend has no honest value for it. 1 (DR only) and 5
  (time only) are never a fix -- the firmware already reports them NO_FIX.
- `h_acc_m` is the receiver's own horizontal accuracy estimate (`hacc_mm`).

Nothing here invents a position: no frame, or a frame older than the stale
limit, is no fix.
"""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, Optional

from robotx.hardware.gps import GPSStatus, GpsFix, GpsReading


# u-blox NAV-PVT fixType -> the backend's fix vocabulary. Only these two are
# stated; anything else carries no fix type.
UBX_FIX_TYPE = {2: "2D", 3: "3D"}


def _int(value: Any) -> Optional[int]:
    return value if isinstance(value, int) and not isinstance(value, bool) else None


def reading_from_frame(
    frame: Optional[Dict[str, Any]],
    received_at: Optional[float],
    *,
    now: float,
    stale_after_s: float,
) -> GpsReading:
    """One ESP32 GPS frame -> GpsReading. Pure."""

    if frame is None or received_at is None:
        return GpsReading(GPSStatus.STARTING, error="no GPS frame from the ESP32 yet")

    age_ms = _int(frame.get("age_ms"))
    fix_at = received_at - (age_ms / 1000.0 if age_ms is not None else 0.0)
    age_s = max(0.0, now - fix_at)
    status = frame.get("gps_status")

    if status != "OK":
        return GpsReading(GPSStatus.NO_FIX, age_s=None, error=f"ESP32 gps_status {status}")
    lat_e7, lon_e7 = _int(frame.get("lat_e7")), _int(frame.get("lon_e7"))
    if lat_e7 is None or lon_e7 is None:
        return GpsReading(GPSStatus.NO_FIX, error="ESP32 gps_status OK without lat/lon")
    if age_s > stale_after_s:
        return GpsReading(GPSStatus.STALE, age_s=age_s, error=f"last ESP32 fix is {age_s:.1f}s old")

    hacc_mm = _int(frame.get("hacc_mm"))
    speed = _int(frame.get("speed_mm_s"))
    head = _int(frame.get("head_mot_e5"))
    fix = GpsFix(
        latitude=lat_e7 / 1e7,
        longitude=lon_e7 / 1e7,
        timestamp=fix_at,
        satellites=_int(frame.get("siv")),
        speed_mps=None if speed is None else speed / 1000.0,
        track_deg=None if head is None else head / 1e5,
        fix_type=UBX_FIX_TYPE.get(_int(frame.get("fix_type"))),
        h_acc_m=None if hacc_mm is None or hacc_mm <= 0 else hacc_mm / 1000.0,
    )
    return GpsReading(GPSStatus.FIX, fix=fix, age_s=age_s)


class Esp32GpsReader:
    """`GPSReader`'s interface (`get_reading`), backed by the ESP32 link."""

    def __init__(self, link: Any, *, stale_after_s: float = 5.0, wall: Callable[[], float] = time.time) -> None:
        self._link = link
        self._stale_after_s = stale_after_s
        self._wall = wall

    def start(self) -> None:  # the link owns the I/O
        return None

    def stop(self) -> None:
        return None

    def get_reading(self) -> GpsReading:
        frame, received_at = self._link.gps_frame()
        return reading_from_frame(frame, received_at, now=self._wall(), stale_after_s=self._stale_after_s)
