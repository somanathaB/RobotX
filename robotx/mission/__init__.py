"""The mission the Rover was given, and how far through it the Rover is.

The Rover does not plan routes. RobotX generates them with Mapbox Directions
and hands over a finished pair of waypoint lists in `TASK_ASSIGN`; nothing in
this package contacts Mapbox, reads a tile, or computes a path. What lives here
is the consumer side: validating an assignment, holding it, and tracking which
waypoint of which leg the Rover is on.

    RobotX -> TASK_ASSIGN -> MissionManager -> active route -> Navigator

Split in two on purpose:

- `mission.py` is the domain model -- immutable, pure, and unaware of the wire.
  It imports nothing from `robotx.communication`, so a mission can be reasoned
  about and tested without a backend.
- `manager.py` owns the single mutable fact (which mission is active and how
  far along it is) and drives the existing `Navigator` with it. It contains no
  navigation algorithm of its own.
"""

from robotx.mission.manager import MissionAssignment, MissionManager, MissionUpdate
from robotx.mission.mission import (
    ActiveMission,
    Mission,
    MissionRejected,
    MissionRejectReason,
    MissionSegment,
    MissionStatus,
)


__all__ = [
    "ActiveMission",
    "Mission",
    "MissionAssignment",
    "MissionManager",
    "MissionRejectReason",
    "MissionRejected",
    "MissionSegment",
    "MissionStatus",
    "MissionUpdate",
]
