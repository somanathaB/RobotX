"""A synthetic `TASK_ASSIGN`, in exactly the verified RobotX schema.

Why this exists
---------------
The RobotX assignment engine was not delivering tasks to this Rover when this
was written, so there is no captured live payload to test against. This module
is the stand-in, and it is deliberately hand-built from the verified contract
rather than from anything the Pi produces::

    {
      taskId,
      pickup:       {lat, lon},
      drop:         {lat, lon},
      pathToPickup: [{lat, lon}, ...],
      pathToDrop:   [{lat, lon}, ...],
      timestamp
    }

`pathToPickup` and `pathToDrop` stand in for Mapbox Directions output: WGS84
waypoint arrays generated **by RobotX**. Nothing here calls Mapbox, and the
geometry is a plain straight line because the Rover's mission consumer does not
care what shape a route is -- only that it is a validated list of points it can
follow in order.

What it proves, and what it does not
------------------------------------
A test passing against this fixture shows the Pi handles a payload of this
shape correctly. It shows nothing at all about whether RobotX emits one, or
emits it to the right room. Keep the two claims apart.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple


LatLon = Tuple[float, float]

METRES_PER_DEG_LAT = 111_320.0

# How many points RobotX's Directions output is trimmed to per leg here.
# Chosen with the leg lengths below so consecutive waypoints sit roughly 15 m
# apart: comfortably outside the navigator's 8 m arrival radius (so a Rover
# standing on one waypoint has not also arrived at the next) and well inside
# its 25 m off-route threshold (so driving the route exactly is not mistaken
# for leaving it). Tests that care about either boundary set their own.
POINTS_PER_LEG = 6


def offset(origin: LatLon, *, north_m: float = 0.0, east_m: float = 0.0) -> LatLon:
    """A point `north_m`/`east_m` from `origin`, flat-earth at this scale."""

    lat, lon = origin
    east_scale = METRES_PER_DEG_LAT * math.cos(math.radians(lat))
    return (lat + north_m / METRES_PER_DEG_LAT, lon + east_m / east_scale)


# Somewhere unremarkable to put a campus. The same coordinates the other tests
# already use, so a failure is obviously about missions and not geodesy. The
# two legs are stated in metres rather than as decimal degrees, because their
# *lengths* are what the tests depend on.
PICKUP: LatLon = (12.971600, 77.594600)
# Roughly where a Rover would be sitting when the task arrives: ~78 m from the
# pickup, so the first leg is a real drive.
START: LatLon = offset(PICKUP, north_m=-55.0, east_m=-55.0)
# ~85 m beyond the pickup.
DROP: LatLon = offset(PICKUP, north_m=60.0, east_m=60.0)


def leg(start: LatLon, end: LatLon, *, points: int = POINTS_PER_LEG) -> List[LatLon]:
    """A straight run of waypoints from `start` to `end`, inclusive of both."""

    points = max(2, int(points))
    return [
        (
            start[0] + (end[0] - start[0]) * i / (points - 1),
            start[1] + (end[1] - start[1]) * i / (points - 1),
        )
        for i in range(points)
    ]


def wire_point(point: LatLon) -> Dict[str, float]:
    """One waypoint as RobotX puts it on the wire."""

    return {"lat": point[0], "lon": point[1]}


def wire_path(points: Sequence[LatLon]) -> List[Dict[str, float]]:
    return [wire_point(p) for p in points]


def path_to_pickup(points: int = POINTS_PER_LEG) -> List[LatLon]:
    return leg(START, PICKUP, points=points)


def path_to_drop(points: int = POINTS_PER_LEG) -> List[LatLon]:
    return leg(PICKUP, DROP, points=points)


# Distinguishes "the caller said null" from "the caller said nothing", which
# for a timestamp are different payloads and must be refused on their own
# terms.
UNSET = object()


def task_assign_payload(
    *,
    task_id: str = "task-synthetic-1",
    pickup_at: LatLon = PICKUP,
    drop_at: LatLon = DROP,
    to_pickup: Optional[Sequence[LatLon]] = None,
    to_drop: Optional[Sequence[LatLon]] = None,
    timestamp: Any = UNSET,
    **overrides: Any,
) -> Dict[str, Any]:
    """A complete, valid `TASK_ASSIGN` payload.

    `overrides` replaces top-level wire keys verbatim, which is how the
    malformed cases are built: a test states the one field it is breaking --
    `task_assign_payload(pickup={"lat": float("nan"), "lon": 77.6})` -- and
    everything it did not mention stays valid. The keyword arguments are named
    `pickup_at`/`drop_at` precisely so that the wire spellings stay free for
    that.
    """

    payload: Dict[str, Any] = {
        "taskId": task_id,
        "pickup": wire_point(pickup_at),
        "drop": wire_point(drop_at),
        "pathToPickup": wire_path(path_to_pickup() if to_pickup is None else to_pickup),
        "pathToDrop": wire_path(path_to_drop() if to_drop is None else to_drop),
        # Epoch milliseconds, the convention every other RobotX observation
        # uses.
        "timestamp": int(time.time() * 1000) if timestamp is UNSET else timestamp,
    }
    payload.update(overrides)
    return payload
