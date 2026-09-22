"""Route progress tracking: which waypoint is next, and when to reroute.

Pure logic over a list of (lat, lon) waypoints. No I/O, no hardware, no HTTP.
A route can come from anywhere -- a locally supplied waypoint list or the
optional Directions client -- and this module does not care which.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

from robotx.localization.position import LatLon, haversine_m


__all__ = ["LatLon", "PlannerConfig", "RoutePlanner", "haversine_m"]


@dataclass(frozen=True)
class PlannerConfig:
    waypoint_arrival_m: float = 8.0
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

        self._offroute_count = 0
        self._blocked_count = 0
        self._last_update_t = 0.0

    def set_route(self, route: Sequence[LatLon]) -> None:
        self._route = [(float(lat), float(lon)) for lat, lon in route]
        self._idx = 0
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

    def report_blocked(self) -> None:
        self._blocked_count += 1

    def update_position(self, pos: LatLon) -> None:
        self._last_update_t = time.time()
        self._last_pos = pos

        if not self._route:
            return

        waypoint = self.next_waypoint()
        if waypoint is None:
            return

        # Advance when close enough to the current waypoint.
        if haversine_m(pos, waypoint) <= self.cfg.waypoint_arrival_m:
            self._idx = min(self._idx + 1, len(self._route) - 1)
            self._offroute_count = 0

        # Off-route heuristic: consistently far from the waypoint we are
        # heading for. One bad fix should not trigger a reroute, so this
        # accumulates and decays.
        next_waypoint = self.next_waypoint()
        if next_waypoint is None:
            return
        if haversine_m(pos, next_waypoint) > self.cfg.off_route_m:
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
