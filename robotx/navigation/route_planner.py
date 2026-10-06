"""Route progress tracking: which waypoint is next, and when to reroute.

Pure logic over a list of (lat, lon) waypoints. No I/O, no hardware, no HTTP.
A route can come from anywhere -- a locally supplied waypoint list or the
optional Directions client -- and this module does not care which.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from robotx.localization.position import EARTH_RADIUS_M, LatLon, haversine_m


__all__ = ["LatLon", "PlannerConfig", "RoutePlanner", "haversine_m", "segment_distance_m"]


def segment_distance_m(point: LatLon, start: LatLon, end: LatLon) -> float:
    """Distance from `point` to the segment `start`-`end`, in metres.

    The perpendicular (cross-track) distance where the point lies alongside the
    segment, and the distance to the nearer end beyond it -- so a robot that
    overshoots the segment's end, or has not reached its start, is measured to
    that end. Computed in a flat local projection centred on `start`
    (equirectangular), which at campus scale differs from the great-circle
    figure by far less than a GPS fix's own uncertainty.
    """

    scale = math.radians(1.0) * EARTH_RADIUS_M  # metres per degree of latitude
    cos_lat = math.cos(math.radians(start[0]))

    def local(p: LatLon) -> Tuple[float, float]:
        return ((p[1] - start[1]) * scale * cos_lat, (p[0] - start[0]) * scale)

    bx, by = local(end)
    px, py = local(point)
    length_sq = bx * bx + by * by
    t = 0.0 if length_sq == 0.0 else max(0.0, min(1.0, (px * bx + py * by) / length_sq))
    return math.hypot(px - t * bx, py - t * by)


@dataclass(frozen=True)
class PlannerConfig:
    waypoint_arrival_m: float = 8.0
    # Half-width of the corridor around the route: how far the robot may be from
    # the segment it is driving before that counts as evidence of being off route.
    off_route_m: float = 25.0
    reroute_after_n: int = 8
    blocked_reroute_after_n: int = 3


class RoutePlanner:
    """Tracks progress along a route and decides whether rerouting is needed."""

    def __init__(self, cfg: Optional[PlannerConfig] = None) -> None:
        self.cfg = cfg or PlannerConfig()
        self._route: List[LatLon] = []
        self._idx = 0
        self._last_pos: Optional[LatLon] = None
        # Where the robot was when this route was given to it: the start of the
        # segment leading to waypoint 0, which has no previous waypoint.
        self._route_start: Optional[LatLon] = None

        self._offroute_count = 0
        self._blocked_count = 0
        self._last_update_t = 0.0

    def set_route(self, route: Sequence[LatLon]) -> None:
        self._route = [(float(lat), float(lon)) for lat, lon in route]
        self._idx = 0
        self._route_start = None
        self._offroute_count = 0
        self._blocked_count = 0

    def has_route(self) -> bool:
        return len(self._route) > 1

    def route_length(self) -> int:
        return len(self._route)

    def waypoint_index(self) -> int:
        return self._idx

    def next_waypoint(self) -> Optional[LatLon]:
        if not self._route:
            return None
        return self._route[min(self._idx, len(self._route) - 1)]

    def destination(self) -> Optional[LatLon]:
        if not self._route:
            return None
        return self._route[-1]

    def active_segment(self) -> Optional[Tuple[LatLon, LatLon]]:
        """The stretch of route being driven: previous waypoint -> next waypoint.

        For waypoint 0 the start is the robot's first position after the route
        was set. None until there is both a route and that position.
        """

        end = self.next_waypoint()
        if end is None:
            return None
        start = self._route[self._idx - 1] if self._idx > 0 else self._route_start
        return None if start is None else (start, end)

    def report_blocked(self) -> None:
        self._blocked_count += 1

    def update_position(self, pos: LatLon) -> None:
        self._last_update_t = time.time()
        self._last_pos = pos

        if not self._route:
            return
        if self._route_start is None:
            self._route_start = pos

        waypoint = self.next_waypoint()
        if waypoint is None:
            return

        # Advance when close enough to the current waypoint.
        if haversine_m(pos, waypoint) <= self.cfg.waypoint_arrival_m:
            self._idx = min(self._idx + 1, len(self._route) - 1)
            self._offroute_count = 0

        # Off-route: consistently outside the corridor around the segment being
        # driven. Measured to the segment, not to the next waypoint -- on a leg
        # longer than the corridor the robot is far from its next waypoint while
        # exactly on the route. One bad fix should not trigger a reroute, so this
        # accumulates and decays.
        segment = self.active_segment()
        if segment is None:
            return
        if segment_distance_m(pos, *segment) > self.cfg.off_route_m:
            self._offroute_count += 1
        else:
            self._offroute_count = max(0, self._offroute_count - 1)

    def arrived(self, pos: Optional[LatLon] = None) -> bool:
        """True once the final waypoint has been reached."""

        destination = self.destination()
        if destination is None:
            return False
        if self._idx < len(self._route) - 1:
            return False

        point = pos if pos is not None else self._last_pos
        if point is None:
            return False
        return haversine_m(point, destination) <= self.cfg.waypoint_arrival_m

    def should_reroute(self) -> bool:
        if self._blocked_count >= self.cfg.blocked_reroute_after_n:
            return True
        return self._offroute_count >= self.cfg.reroute_after_n

    def progress(self) -> float:
        """Fraction of waypoints consumed, 0.0 to 1.0."""

        if len(self._route) < 2:
            return 0.0
        return float(self._idx) / float(len(self._route) - 1)
