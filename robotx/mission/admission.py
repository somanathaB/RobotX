"""Route admission: is this route one this Rover can start from where it stands?

`Mission.create` already refuses anything that is not a finite, in-range,
structurally complete assignment. What it cannot know is where the Rover is,
and two facts about the route only make sense against that:

- **The pickup leg must start here.** The route planner measures the first
  segment from the Rover's own position to `pathToPickup[0]`, so a leg that
  starts 400 m away is driven as one straight 400 m line, across whatever lies
  between, with nothing able to call it off route. RobotX's Directions route is
  generated from the Rover's reported position, so its first point is within a
  road-snap of it; anything further is a route for somewhere else.
- **The pickup must be near.** A conservative, configurable bound on how far
  the Rover will be sent to collect a parcel, so a mis-addressed task, a stale
  position on the backend or a unit mix-up is refused before the Rover drives
  rather than discovered kilometres later.

And three facts about the route's own consistency, which the Rover depends on
because it judges arrival at the *end of the path*, not at the stop:

- `pathToPickup` ends at the pickup, `pathToDrop` starts there and ends at the
  drop, each within a road-snap distance.

These are bounds on the geometry RobotX supplied, never a change to it: no
point is moved, added or dropped, and no campus boundary is assumed. Each
refusal names the distance and the limit. Pure: no I/O, no clock.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

from robotx.localization.position import LatLon, haversine_m
from robotx.mission.mission import Mission, MissionRejected, MissionRejectReason


# Configuration bounds: wide enough for any real campus deployment, narrow
# enough that a typo cannot disable the check (0 or 1e9 are both refused).
MIN_LIMIT_M = 5.0
MAX_START_OFFSET_M = 200.0
MAX_ENDPOINT_OFFSET_M = 200.0
MAX_PICKUP_DISTANCE_M = 5000.0


@dataclass(frozen=True)
class AdmissionConfig:
    # Rover position -> pathToPickup[0]. The navigator's off-route corridor is
    # 25 m; a route that starts outside it is not a route from here.
    max_start_offset_m: float = 25.0
    # Rover position -> pickup, straight line. Conservative for a rover that
    # drives at ~0.25 m/s: 300 m is a ~20 minute leg.
    max_pickup_distance_m: float = 300.0
    # Each stop vs the path end (and pathToDrop[0] vs the pickup). Directions
    # snaps a stop to the nearest road; 50 m allows that and little else.
    max_endpoint_offset_m: float = 50.0

    def __post_init__(self) -> None:
        for name, upper in (("max_start_offset_m", MAX_START_OFFSET_M),
                            ("max_pickup_distance_m", MAX_PICKUP_DISTANCE_M),
                            ("max_endpoint_offset_m", MAX_ENDPOINT_OFFSET_M)):
            value = getattr(self, name)
            if not (isinstance(value, (int, float)) and math.isfinite(value) and MIN_LIMIT_M <= value <= upper):
                raise ValueError(f"mission admission {name} must be in [{MIN_LIMIT_M:g}, {upper:g}] m, got {value!r}")

    @classmethod
    def from_settings(cls, settings: Any) -> "AdmissionConfig":
        return cls(
            max_start_offset_m=settings.mission_max_start_offset_m,
            max_pickup_distance_m=settings.mission_max_pickup_distance_m,
            max_endpoint_offset_m=settings.mission_max_endpoint_offset_m,
        )


def check_route_consistency(mission: Mission, cfg: AdmissionConfig) -> None:
    """The route's own geometry hangs together, or `MissionRejected` (ROUTE_INCONSISTENT)."""

    def near(a: LatLon, b: LatLon, what: str) -> None:
        d = haversine_m(a, b)
        if not d <= cfg.max_endpoint_offset_m:
            raise MissionRejected(
                MissionRejectReason.ROUTE_INCONSISTENT,
                f"{what} are {d:.0f} m apart (limit {cfg.max_endpoint_offset_m:.0f} m)",
                task_id=mission.task_id,
            )

    near(mission.path_to_pickup[-1], mission.pickup, "the end of pathToPickup and the pickup")
    near(mission.path_to_drop[0], mission.pickup, "the start of pathToDrop and the pickup")
    near(mission.path_to_drop[-1], mission.drop, "the end of pathToDrop and the drop")


def check_start_position(mission: Mission, here: Optional[LatLon], cfg: AdmissionConfig) -> None:
    """The route starts where the Rover is and the pickup is near, or `MissionRejected`.

    `here` must be a measured, navigable position; without one the caller has
    already refused the offer (NO_POSITION_FIX), and this refuses too rather
    than judging distance from nothing.
    """

    if here is None:
        raise MissionRejected(
            MissionRejectReason.PICKUP_TOO_FAR,
            "no measured position to judge the pickup distance from",
            task_id=mission.task_id,
        )
    start = haversine_m(here, mission.path_to_pickup[0])
    if not start <= cfg.max_start_offset_m:
        raise MissionRejected(
            MissionRejectReason.ROUTE_START_TOO_FAR,
            f"pathToPickup starts {start:.0f} m from the Rover (limit {cfg.max_start_offset_m:.0f} m)",
            task_id=mission.task_id,
        )
    pickup = haversine_m(here, mission.pickup)
    if not pickup <= cfg.max_pickup_distance_m:
        raise MissionRejected(
            MissionRejectReason.PICKUP_TOO_FAR,
            f"the pickup is {pickup:.0f} m from the Rover (limit {cfg.max_pickup_distance_m:.0f} m)",
            task_id=mission.task_id,
        )


def check_admissible(mission: Mission, here: Optional[LatLon], cfg: AdmissionConfig) -> None:
    """Every route admission check, consistency first (it needs no position)."""

    check_route_consistency(mission, cfg)
    check_start_position(mission, here, cfg)
