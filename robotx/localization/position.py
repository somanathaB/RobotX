"""Position and heading estimation from GPS fixes.

This sits between GPS acquisition (`robotx.hardware.gps`) and navigation
(`robotx.navigation`): the reader knows about serial ports and NMEA, navigation
knows about waypoints, and this module is the only place that turns one into
the other. It is its own layer rather than part of `robotx.state` so that
navigation can depend on it without the two packages importing each other.

Heading, and what it is actually worth
--------------------------------------
This robot has **no compass, no IMU and no magnetometer**. It therefore has no
heading source while stationary. Two sources are available, both from GPS, and
both are reported with an explicit `HeadingSource` so no consumer can mistake
one for a real orientation sensor:

- `NMEA_TRACK`  -- course over ground from the receiver's RMC sentence. Only
  trusted above `heading_min_speed_mps`, because a stationary receiver's course
  field is noise.
- `GPS_TRACK`   -- bearing between two consecutive fixes at least
  `heading_min_move_m` apart. Same limitation, computed locally.
- `NONE`        -- stationary, or not enough movement yet. Heading is `None`.

Both describe the direction the robot *moved*, not the direction it *faces*.
They are identical only while driving forward in a straight line. Neither can
detect that the robot is facing backwards, sliding, or turning in place. Any
consumer needing true orientation needs hardware this robot does not have.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional, Tuple

from robotx.hardware.gps import GpsReading, GPSStatus


LatLon = Tuple[float, float]

EARTH_RADIUS_M = 6371000.0


def haversine_m(a: LatLon, b: LatLon) -> float:
    """Great-circle distance between two (lat, lon) points, in metres."""

    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(h))


def bearing_deg(a: LatLon, b: LatLon) -> float:
    """Initial true bearing from `a` to `b`, in degrees clockwise from north."""

    lat1, lat2 = math.radians(a[0]), math.radians(b[0])
    dlon = math.radians(b[1] - a[1])
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.degrees(math.atan2(y, x)) % 360.0


def heading_error_deg(desired: float, current: float) -> float:
    """Signed smallest turn from `current` to `desired`, in (-180, 180].

    Positive means turn right (clockwise).
    """

    return (desired - current + 180.0) % 360.0 - 180.0


class HeadingSource(str, Enum):
    NONE = "NONE"              # no heading available
    NMEA_TRACK = "NMEA_TRACK"  # course over ground from the receiver
    GPS_TRACK = "GPS_TRACK"    # bearing between consecutive fixes
    # Integrated from commanded motion, not observed. Drifts without bound and
    # cannot detect that the rover failed to turn; see localization.local_frame.
    DEAD_RECKONED = "DEAD_RECKONED"


class PositionSource(str, Enum):
    """Where a `Position` came from, and therefore what it is worth.

    Separate from `HeadingSource` because the two genuinely differ: a GPS fix
    can carry a dead heading (stationary receiver), and a dead-reckoned pose
    has a heading that is exactly as good as its position. A consumer deciding
    whether it may act on, or publish, a coordinate needs this field and not
    the other one.
    """

    GPS = "GPS"                            # a real fix from a receiver
    DEAD_RECKONING = "DEAD_RECKONING"      # integrated from commanded motion

    @property
    def is_measured(self) -> bool:
        """Whether this position was observed rather than inferred."""

        return self is PositionSource.GPS


@dataclass(frozen=True)
class Position:
    """The robot's best current estimate of where it is."""

    latitude: float
    longitude: float
    timestamp: float
    altitude_m: Optional[float] = None
    speed_mps: Optional[float] = None
    heading_deg: Optional[float] = None
    heading_source: HeadingSource = HeadingSource.NONE
    satellites: Optional[int] = None
    # Defaults to GPS so that every existing producer keeps its meaning without
    # being edited. A position that was *not* measured has to say so explicitly,
    # which is the right way round: inferring a coordinate is the unusual act,
    # and the unusual act is the one that should require a deliberate statement.
    source: PositionSource = PositionSource.GPS

    @property
    def lat_lon(self) -> LatLon:
        return (self.latitude, self.longitude)

    @property
    def is_measured(self) -> bool:
        return self.source.is_measured

    def to_dict(self) -> Dict[str, Any]:
        return {
            "latitude": round(self.latitude, 7),
            "longitude": round(self.longitude, 7),
            "altitude_m": self.altitude_m,
            "speed_mps": None if self.speed_mps is None else round(self.speed_mps, 3),
            "heading_deg": None if self.heading_deg is None else round(self.heading_deg, 1),
            "heading_source": self.heading_source.value,
            "satellites": self.satellites,
            "source": self.source.value,
            "timestamp": self.timestamp,
        }


@dataclass(frozen=True)
class PositionConfig:
    heading_min_move_m: float = 1.5
    heading_min_speed_mps: float = 0.5

    @classmethod
    def from_settings(cls, settings: Any) -> "PositionConfig":
        return cls(
            heading_min_move_m=settings.position_heading_min_move_m,
            heading_min_speed_mps=settings.position_heading_min_speed_mps,
        )


class PositionEstimator:
    """Turns GPS readings into a `Position`, deriving heading where possible."""

    def __init__(self, cfg: Optional[PositionConfig] = None) -> None:
        self.cfg = cfg or PositionConfig()
        self._last_heading: Optional[float] = None
        self._last_heading_source = HeadingSource.NONE
        self._heading_anchor: Optional[LatLon] = None

    def reset(self) -> None:
        self._last_heading = None
        self._last_heading_source = HeadingSource.NONE
        self._heading_anchor = None

    def update(self, reading: GpsReading) -> Optional[Position]:
        """Return the current position, or None when there is no usable fix."""

        if reading.status is not GPSStatus.FIX or reading.fix is None:
            return None

        fix = reading.fix
        point = (fix.latitude, fix.longitude)

        heading, source = self._estimate_heading(point, fix.speed_mps, fix.track_deg)

        return Position(
            latitude=fix.latitude,
            longitude=fix.longitude,
            timestamp=fix.timestamp,
            altitude_m=fix.altitude_m,
            speed_mps=fix.speed_mps,
            heading_deg=heading,
            heading_source=source,
            satellites=fix.satellites,
        )

    def _estimate_heading(
        self,
        point: LatLon,
        speed_mps: Optional[float],
        track_deg: Optional[float],
    ) -> Tuple[Optional[float], HeadingSource]:
        # Preferred: the receiver's own course over ground, but only while
        # actually moving -- a stationary receiver's course field is noise.
        if (
            track_deg is not None
            and speed_mps is not None
            and speed_mps >= self.cfg.heading_min_speed_mps
        ):
            self._last_heading = track_deg % 360.0
            self._last_heading_source = HeadingSource.NMEA_TRACK
            self._heading_anchor = point
            return self._last_heading, HeadingSource.NMEA_TRACK

        # Fallback: bearing between fixes far enough apart to be movement
        # rather than GPS wander.
        if self._heading_anchor is None:
            self._heading_anchor = point
        elif haversine_m(self._heading_anchor, point) >= self.cfg.heading_min_move_m:
            self._last_heading = bearing_deg(self._heading_anchor, point)
            self._last_heading_source = HeadingSource.GPS_TRACK
            self._heading_anchor = point
            return self._last_heading, HeadingSource.GPS_TRACK

        # Not enough movement: hold the last known heading rather than pretend
        # to a new one. It is stale, and its source says where it came from.
        return self._last_heading, self._last_heading_source
