"""Navigation: where the robot is on its route, and where it should aim.

Consumes the robot's position and its route; produces a `NavigationState`
(target waypoint, distance, desired heading, heading error, progress). It does
not decide how fast to drive or whether an obstacle is in the way -- that is
`robotx.control.decision` -- and it never touches GPIO or a motor driver.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Sequence

from robotx.config.logging_setup import log_event
from robotx.navigation.route_planner import PlannerConfig, RoutePlanner
from robotx.localization.position import LatLon, Position, bearing_deg, haversine_m, heading_error_deg


logger = logging.getLogger(__name__)


class NavigationStatus(str, Enum):
    IDLE = "IDLE"              # no route loaded
    NO_POSITION = "NO_POSITION"  # route loaded but no GPS fix to act on
    NAVIGATING = "NAVIGATING"  # driving toward a waypoint
    ARRIVED = "ARRIVED"        # final waypoint reached
    REROUTE_NEEDED = "REROUTE_NEEDED"  # off route or repeatedly blocked


@dataclass(frozen=True)
class NavigationState:
    """Immutable snapshot of navigation, produced once per agent tick."""

    status: NavigationStatus = NavigationStatus.IDLE
    target_waypoint: Optional[LatLon] = None
    destination: Optional[LatLon] = None
    distance_to_target_m: Optional[float] = None
    distance_to_destination_m: Optional[float] = None
    desired_heading_deg: Optional[float] = None
    current_heading_deg: Optional[float] = None
    heading_error_deg: Optional[float] = None
    waypoints_total: int = 0
    waypoint_index: int = 0
    progress: float = 0.0

    @property
    def has_target(self) -> bool:
        return self.target_waypoint is not None

    def to_dict(self) -> Dict[str, Any]:
        def point(p: Optional[LatLon]) -> Optional[Dict[str, float]]:
            return None if p is None else {"lat": round(p[0], 7), "lon": round(p[1], 7)}

        def dist(v: Optional[float]) -> Optional[float]:
            return None if v is None else round(v, 1)

        def deg(v: Optional[float]) -> Optional[float]:
            return None if v is None else round(v, 1)

        return {
            "status": self.status.value,
            "target_waypoint": point(self.target_waypoint),
            "destination": point(self.destination),
            "distance_to_target_m": dist(self.distance_to_target_m),
            "distance_to_destination_m": dist(self.distance_to_destination_m),
            "desired_heading_deg": deg(self.desired_heading_deg),
            "current_heading_deg": deg(self.current_heading_deg),
            "heading_error_deg": deg(self.heading_error_deg),
            "waypoint_index": self.waypoint_index,
            "waypoints_total": self.waypoints_total,
            "progress": round(self.progress, 3),
        }


@dataclass(frozen=True)
class NavigationConfig:
    waypoint_arrival_m: float = 8.0
    off_route_m: float = 25.0
    reroute_after_n: int = 8
    blocked_reroute_after_n: int = 3

    @classmethod
    def from_settings(cls, settings: Any) -> "NavigationConfig":
        return cls(
            waypoint_arrival_m=settings.nav_waypoint_arrival_m,
            off_route_m=settings.nav_off_route_m,
            reroute_after_n=settings.nav_reroute_after_n,
            blocked_reroute_after_n=settings.nav_blocked_reroute_after_n,
        )

    def to_planner_config(self) -> PlannerConfig:
        return PlannerConfig(
            waypoint_arrival_m=self.waypoint_arrival_m,
            off_route_m=self.off_route_m,
            reroute_after_n=self.reroute_after_n,
            blocked_reroute_after_n=self.blocked_reroute_after_n,
        )


class Navigator:
    """Tracks route progress and computes the desired heading to the next point."""

    def __init__(
        self,
        cfg: Optional[NavigationConfig] = None,
        planner: Optional[RoutePlanner] = None,
    ) -> None:
        self.cfg = cfg or NavigationConfig()
        self.planner = planner or RoutePlanner(self.cfg.to_planner_config())
        self._state = NavigationState()
        self._last_status: Optional[NavigationStatus] = None

    @property
    def state(self) -> NavigationState:
        return self._state

    def set_route(self, waypoints: Sequence[LatLon]) -> NavigationState:
        """Load a route: an ordered list of (lat, lon) waypoints."""

        route: List[LatLon] = [(float(lat), float(lon)) for lat, lon in waypoints]
        self.planner.set_route(route)
        log_event(
            logger,
            "navigation.route_set",
            waypoints=len(route),
            destination=route[-1] if route else None,
        )
        self._state = NavigationState(
            status=NavigationStatus.NO_POSITION if route else NavigationStatus.IDLE,
            destination=route[-1] if route else None,
            waypoints_total=len(route),
        )
        return self._state

    def clear_route(self) -> NavigationState:
        self.planner.set_route([])
        log_event(logger, "navigation.route_cleared")
        self._state = NavigationState(status=NavigationStatus.IDLE)
        self._last_status = None
        return self._state

    def report_blocked(self) -> None:
        """Tell the planner the current leg is obstructed (drives rerouting)."""

        self.planner.report_blocked()

    def update(self, position: Optional[Position]) -> NavigationState:
        """Recompute navigation state for the current position."""

        route_length = self.planner.route_length()

        if route_length == 0:
            self._state = NavigationState(status=NavigationStatus.IDLE)
            self._log_transition()
            return self._state

        destination = self.planner.destination()

        if position is None:
            self._state = NavigationState(
                status=NavigationStatus.NO_POSITION,
                destination=destination,
                waypoints_total=route_length,
                waypoint_index=self.planner.waypoint_index(),
                progress=self.planner.progress(),
            )
            self._log_transition()
            return self._state

        here = position.lat_lon
        self.planner.update_position(here)

        target = self.planner.next_waypoint()
        distance_to_target = None if target is None else haversine_m(here, target)
        distance_to_destination = (
            None if destination is None else haversine_m(here, destination)
        )

        desired = None if target is None else bearing_deg(here, target)
        current = position.heading_deg
        error = (
            None
            if desired is None or current is None
            else heading_error_deg(desired, current)
        )

        if self.planner.arrived(here):
            status = NavigationStatus.ARRIVED
        elif self.planner.should_reroute():
            status = NavigationStatus.REROUTE_NEEDED
        else:
            status = NavigationStatus.NAVIGATING

        self._state = NavigationState(
            status=status,
            target_waypoint=target,
            destination=destination,
            distance_to_target_m=distance_to_target,
            distance_to_destination_m=distance_to_destination,
            desired_heading_deg=desired,
            current_heading_deg=current,
            heading_error_deg=error,
            waypoints_total=route_length,
            waypoint_index=self.planner.waypoint_index(),
            progress=self.planner.progress(),
        )
        self._log_transition()
        return self._state

    def _log_transition(self) -> None:
        status = self._state.status
        if status is self._last_status:
            return
        log_event(
            logger,
            "navigation.status_changed",
            status=status.value,
            waypoint=f"{self._state.waypoint_index + 1}/{self._state.waypoints_total}",
        )
        self._last_status = status
