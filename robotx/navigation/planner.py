import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple


LatLon = Tuple[float, float]


def haversine_m(a: LatLon, b: LatLon) -> float:
    r = 6371000.0
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * r * math.asin(math.sqrt(h))


@dataclass
class PlannerConfig:
    waypoint_arrival_m: float = 8.0
    off_route_m: float = 25.0
    reroute_after_n: int = 8
    blocked_reroute_after_n: int = 3


class RoutePlanner:
    """Tracks progress along a route and decides if rerouting is needed."""

    def __init__(self, cfg: PlannerConfig = PlannerConfig()) -> None:
        self.cfg = cfg
        self._route: List[LatLon] = []
        self._idx = 0
        self._last_pos: Optional[LatLon] = None

        self._offroute_count = 0
        self._blocked_count = 0
        self._last_update_t = 0.0

    def set_route(self, route: List[LatLon]) -> None:
        self._route = list(route)
        self._idx = 0
        self._offroute_count = 0
        self._blocked_count = 0

    def has_route(self) -> bool:
        return len(self._route) > 1

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

        # Advance waypoint if arrived
        wp = self.next_waypoint()
        if wp is None:
            return

        if haversine_m(pos, wp) <= self.cfg.waypoint_arrival_m:
            self._idx = min(self._idx + 1, len(self._route) - 1)
            self._offroute_count = 0

        # Off-route heuristic: far from next waypoint
        wp2 = self.next_waypoint()
        if wp2 is None:
            return
        if haversine_m(pos, wp2) > self.cfg.off_route_m:
            self._offroute_count += 1
        else:
            self._offroute_count = max(0, self._offroute_count - 1)

    def should_reroute(self) -> bool:
        if self._blocked_count >= self.cfg.blocked_reroute_after_n:
            return True
        if self._offroute_count >= self.cfg.reroute_after_n:
            return True
        return False

    def progress(self) -> float:
        if not self._route:
            return 0.0
        return float(self._idx) / float(max(1, len(self._route) - 1))
