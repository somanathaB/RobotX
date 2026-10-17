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
limit, is no fix. Nor is a frame that says OK but contradicts itself or cannot
be checked -- `fix_ok` not true, a `fix_type` that is not a fix, no `age_ms`
or `hacc_mm`, or coordinates outside the Earth or at exactly (0, 0) -- since
a position the receiver cannot vouch for is one navigation must not drive on.
Each is NO_FIX with the reason in `error`. Whether a *valid* fix is accurate
enough to navigate on is the navigator's call (`NavigationConfig.max_h_acc_m`),
not this adapter's: the fix is still reported, with its `h_acc_m`.

Staleness is measured on the monotonic clock when the link supplies one, so a
wall-clock step (NTP correcting an RTC-less Pi) cannot make an old fix look new.
The fix *timestamp* stays wall time: it is what telemetry reports.
"""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, Optional

from robotx.hardware.gps import GPSStatus, GpsFix, GpsReading


# u-blox NAV-PVT fixType -> the backend's fix vocabulary. Only these two are
# stated; anything else carries no fix type.
UBX_FIX_TYPE = {2: "2D", 3: "3D"}
# The fixType values that are a position fix at all: 2D, 3D, GNSS + dead
# reckoning. 0 (none), 1 (DR only) and 5 (time only) are not.
UBX_POSITION_FIX_TYPES = frozenset({2, 3, 4})

LAT_E7_MAX = 900_000_000
LON_E7_MAX = 1_800_000_000


def _int(value: Any) -> Optional[int]:
    return value if isinstance(value, int) and not isinstance(value, bool) else None


def _not_a_fix(frame: Dict[str, Any]) -> Optional[str]:
    """Why a `gps_status: OK` frame still cannot be used as a fix, or None."""

    if frame.get("fix_ok") is not True:
        return f"ESP32 gps_status OK but fix_ok is {frame.get('fix_ok')!r}"
    fix_type = _int(frame.get("fix_type"))
    if fix_type not in UBX_POSITION_FIX_TYPES:
        return f"ESP32 gps_status OK but fix_type {frame.get('fix_type')!r} is not a position fix"
    lat_e7, lon_e7 = _int(frame.get("lat_e7")), _int(frame.get("lon_e7"))
    if lat_e7 is None or lon_e7 is None:
        return "ESP32 gps_status OK without lat/lon"
    if abs(lat_e7) > LAT_E7_MAX or abs(lon_e7) > LON_E7_MAX:
        return f"ESP32 lat_e7/lon_e7 out of range: {lat_e7}, {lon_e7}"
    if lat_e7 == 0 and lon_e7 == 0:
        return "ESP32 reports exactly (0, 0): not a real position"
    age_ms = _int(frame.get("age_ms"))
    if age_ms is None or age_ms < 0:
        return f"ESP32 gps_status OK but age_ms is {frame.get('age_ms')!r}"
    hacc_mm = _int(frame.get("hacc_mm"))
    if hacc_mm is None or hacc_mm <= 0:
        return f"ESP32 gps_status OK but hacc_mm is {frame.get('hacc_mm')!r}"
    return None


def reading_from_frame(
    frame: Optional[Dict[str, Any]],
    received_at: Optional[float],
    *,
    now: float,
    stale_after_s: float,
    elapsed_s: Optional[float] = None,
) -> GpsReading:
    """One ESP32 GPS frame -> GpsReading. Pure.

    `received_at` is the wall time the Pi parsed the frame; `elapsed_s`, when
    given, is the monotonic time since then and is what staleness is judged
    on. Without it the wall clock is used (`now - received_at`).
    """

    if frame is None or received_at is None:
        return GpsReading(GPSStatus.STARTING, error="no GPS frame from the ESP32 yet")

    status = frame.get("gps_status")
    if status != "OK":
        return GpsReading(GPSStatus.NO_FIX, age_s=None, error=f"ESP32 gps_status {status}")
    problem = _not_a_fix(frame)
    if problem is not None:
        return GpsReading(GPSStatus.NO_FIX, error=problem)

    age_ms = frame["age_ms"]
    since_frame_s = elapsed_s if elapsed_s is not None else now - received_at
    age_s = max(0.0, since_frame_s) + age_ms / 1000.0
    if age_s > stale_after_s:
        return GpsReading(GPSStatus.STALE, age_s=age_s, error=f"last ESP32 fix is {age_s:.1f}s old")

    speed = _int(frame.get("speed_mm_s"))
    head = _int(frame.get("head_mot_e5"))
    fix = GpsFix(
        latitude=frame["lat_e7"] / 1e7,
        longitude=frame["lon_e7"] / 1e7,
        timestamp=received_at - age_ms / 1000.0,
        satellites=_int(frame.get("siv")),
        speed_mps=None if speed is None else speed / 1000.0,
        track_deg=None if head is None else head / 1e5,
        fix_type=UBX_FIX_TYPE.get(frame["fix_type"]),
        h_acc_m=frame["hacc_mm"] / 1000.0,
    )
    return GpsReading(GPSStatus.FIX, fix=fix, age_s=age_s)


class Esp32GpsReader:
    """`GPSReader`'s interface (`get_reading`), backed by the ESP32 link."""

    def __init__(
        self,
        link: Any,
        *,
        stale_after_s: float = 5.0,
        wall: Callable[[], float] = time.time,
    ) -> None:
        self._link = link
        self._stale_after_s = stale_after_s
        self._wall = wall

    def start(self) -> None:  # the link owns the I/O
        return None

    def stop(self) -> None:
        return None

    def get_reading(self) -> GpsReading:
        frame, received_at, elapsed_s = self._link.gps_sample()
        return reading_from_frame(
            frame, received_at, now=self._wall(), stale_after_s=self._stale_after_s, elapsed_s=elapsed_s
        )
