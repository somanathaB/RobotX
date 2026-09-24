"""The mission model: what RobotX assigned, and what makes it acceptable.

A mission is a delivery: drive the supplied waypoints to a pickup, then drive
the supplied waypoints to a drop. Both waypoint lists are produced by RobotX
from Mapbox Directions and arrive whole. The Rover's job is to check them,
hold them, and follow them -- never to generate, shorten, smooth or replan
them, all of which would silently put the Rover on a route the backend does
not know about.

Validation is the point of this module
--------------------------------------
Everything here is built through `Mission.create`, which refuses anything it
cannot fully account for. There is no lenient path and no partial mission: a
route with one point, a NaN latitude or a missing task id raises
`MissionRejected` rather than producing a Mission with a hole in it. A rover
that accepts a malformed assignment does not fail at assignment time, it fails
later, outdoors, halfway to a coordinate nobody sent.

The values themselves are WGS84 degrees, the same frame the GPS and
`robotx.localization` already speak, so no conversion happens here and no
second coordinate convention enters the codebase.

This module is deliberately wire-ignorant. It never sees a JSON payload, a
camelCase key or an event name -- `robotx.communication.protocol` owns those
and calls `Mission.create` with the values it extracted. So the domain rules
stay testable without a backend, and the wire shape stays in the one module
that is allowed to know it.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional, Tuple

from robotx.localization.position import LatLon


# A route with one point is not a route: it says where to end up and nothing
# about how to get there. The contract states both paths are Directions output,
# which always has at least an origin and a destination.
MIN_PATH_POINTS = 2

# Upper bound on a single leg. A Directions route across a campus is tens of
# points; thousands means either a pathological payload or a unit mistake, and
# either way the agent loop must not be asked to walk it.
MAX_PATH_POINTS = 2000

# Timestamps before this are not plausible as "when RobotX issued this task",
# and usually mean a unit or epoch mix-up upstream. 2020-01-01T00:00:00Z.
EARLIEST_PLAUSIBLE_TIMESTAMP = 1577836800.0


class MissionRejectReason(str, Enum):
    """Why an assignment was refused. Short, stable slugs for logs and counters."""

    MALFORMED = "MALFORMED"                    # not the shape the contract defines
    MISSING_TASK_ID = "MISSING_TASK_ID"        # no taskId to track this run by
    INVALID_COORDINATE = "INVALID_COORDINATE"  # not a finite, in-range lat/lon
    EMPTY_ROUTE = "EMPTY_ROUTE"                # a leg with fewer than two points
    ROUTE_TOO_LONG = "ROUTE_TOO_LONG"          # more points than we will walk
    INVALID_TIMESTAMP = "INVALID_TIMESTAMP"    # missing or not plausibly a time
    WRONG_ROBOT = "WRONG_ROBOT"                # addressed to a different robotId
    DUPLICATE_TASK = "DUPLICATE_TASK"          # this taskId has already been run
    MISSION_ACTIVE = "MISSION_ACTIVE"          # another mission is still running
    ESTOP_ENGAGED = "ESTOP_ENGAGED"            # latched stop; the Rover cannot drive


class MissionRejected(Exception):
    """An assignment the Rover will not carry out, and why.

    Distinct from `robotx.state.robot_state.MissionRefused`, which is about a
    *command* against a mission already held (resume with nothing to resume,
    return with no home). This one is about the assignment itself being
    unacceptable, and it carries a machine-readable reason because the caller
    reports it back over the link rather than only logging prose.
    """

    def __init__(
        self,
        reason: MissionRejectReason,
        detail: str,
        *,
        task_id: Optional[str] = None,
    ) -> None:
        super().__init__(f"{reason.value}: {detail}")
        self.reason = reason
        self.detail = detail
        self.task_id = task_id

    def to_dict(self) -> Dict[str, Any]:
        return {
            "reason": self.reason.value,
            "detail": self.detail,
            "task_id": self.task_id,
        }


class MissionSegment(str, Enum):
    """Which of the two supplied routes the Rover is currently following."""

    TO_PICKUP = "TO_PICKUP"
    TO_DROP = "TO_DROP"


class MissionStatus(str, Enum):
    """Where the Rover is in the mission, as a state rather than a guess.

    `AT_PICKUP` is a real state and not a formality: it is the instant the
    pickup leg finished and before the drop leg begins. For a mission that
    requires custody (every engine OFFER) the Rover *holds* there until the
    parcel is genuinely acquired, and holds at `AT_DROP` until it is genuinely
    released -- the backend moves the Leg on exactly those two reports.
    """

    TO_PICKUP = "TO_PICKUP"  # following pathToPickup
    AT_PICKUP = "AT_PICKUP"  # pickup reached; drop leg not yet started
    TO_DROP = "TO_DROP"      # following pathToDrop
    AT_DROP = "AT_DROP"      # drop reached; custody not yet released
    COMPLETE = "COMPLETE"    # drop reached (and, with custody, released)
    ABORTED = "ABORTED"      # stopped before the drop; will not resume

    @property
    def is_terminal(self) -> bool:
        return self in (MissionStatus.COMPLETE, MissionStatus.ABORTED)

    @property
    def is_active(self) -> bool:
        return not self.is_terminal


def validate_coordinate(lat: Any, lon: Any, *, where: str) -> LatLon:
    """One WGS84 point, or `MissionRejected`.

    Checked rather than coerced. `float("nan")` is a float, sorts oddly, and
    every distance computed from it is NaN -- a rover steering toward a NaN
    waypoint has no defined behaviour at all, so it is refused here where the
    refusal is still cheap and explainable.
    """

    def number(value: Any, axis: str) -> float:
        # bool is an int in Python; True as a latitude is a bug, not a 1.0.
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise MissionRejected(
                MissionRejectReason.INVALID_COORDINATE,
                f"{where}.{axis} is {type(value).__name__}, expected a number",
            )
        as_float = float(value)
        if not math.isfinite(as_float):
            raise MissionRejected(
                MissionRejectReason.INVALID_COORDINATE, f"{where}.{axis} is {as_float}"
            )
        return as_float

    latitude = number(lat, "lat")
    longitude = number(lon, "lon")

    if not -90.0 <= latitude <= 90.0:
        raise MissionRejected(
            MissionRejectReason.INVALID_COORDINATE,
            f"{where}.lat {latitude} is outside [-90, 90]",
        )
    if not -180.0 <= longitude <= 180.0:
        raise MissionRejected(
            MissionRejectReason.INVALID_COORDINATE,
            f"{where}.lon {longitude} is outside [-180, 180]",
        )
    return (latitude, longitude)


def validate_path(points: Any, *, where: str) -> Tuple[LatLon, ...]:
    """One leg of the route: at least two points, every one of them valid."""

    if isinstance(points, (str, bytes)) or not isinstance(points, Sequence):
        raise MissionRejected(
            MissionRejectReason.MALFORMED,
            f"{where} is {type(points).__name__}, expected a list of points",
        )
    if len(points) < MIN_PATH_POINTS:
        raise MissionRejected(
            MissionRejectReason.EMPTY_ROUTE,
            f"{where} has {len(points)} point(s); at least {MIN_PATH_POINTS} are required",
        )
    if len(points) > MAX_PATH_POINTS:
        raise MissionRejected(
            MissionRejectReason.ROUTE_TOO_LONG,
            f"{where} has {len(points)} points; the limit is {MAX_PATH_POINTS}",
        )

    route = []
    for index, point in enumerate(points):
        if isinstance(point, (str, bytes)) or not isinstance(point, Sequence) or len(point) != 2:
            raise MissionRejected(
                MissionRejectReason.MALFORMED,
                f"{where}[{index}] is not a (lat, lon) pair",
            )
        route.append(validate_coordinate(point[0], point[1], where=f"{where}[{index}]"))
    return tuple(route)


def validate_timestamp(value: Any, *, where: str = "timestamp") -> float:
    """The instant RobotX issued the assignment, in unix seconds.

    Required, not optional. It is how a delayed or replayed assignment is
    recognized later, and a mission that cannot say when it was issued cannot
    be aged at all.
    """

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionRejected(
            MissionRejectReason.INVALID_TIMESTAMP,
            f"{where} is {type(value).__name__}, expected unix seconds",
        )
    as_float = float(value)
    if not math.isfinite(as_float) or as_float < EARLIEST_PLAUSIBLE_TIMESTAMP:
        raise MissionRejected(
            MissionRejectReason.INVALID_TIMESTAMP, f"{where} {value!r} is not a plausible time"
        )
    return as_float


@dataclass(frozen=True)
class Mission:
    """A validated assignment. Immutable: progress is tracked separately.

    Keeping the assignment immutable and the progress in `ActiveMission` means
    the route the Rover is following is always exactly the route RobotX sent.
    Nothing in the Rover can edit a waypoint list in place, so a Rover that has
    driven half a mission still reports the same route it was given.
    """

    task_id: str
    pickup: LatLon
    drop: LatLon
    path_to_pickup: Tuple[LatLon, ...]
    path_to_drop: Tuple[LatLon, ...]
    # When RobotX issued the assignment, in unix seconds.
    assigned_at: float

    @classmethod
    def create(
        cls,
        *,
        task_id: Any,
        pickup: Any,
        drop: Any,
        path_to_pickup: Any,
        path_to_drop: Any,
        timestamp: Any,
    ) -> "Mission":
        """Validate every field, or refuse the whole mission.

        `pickup`/`drop` and each path point are `(lat, lon)` pairs. Extracting
        them from whatever the wire shape was is the caller's job; enforcing
        that they are real coordinates is this one's.
        """

        if not isinstance(task_id, str) or not task_id.strip():
            raise MissionRejected(
                MissionRejectReason.MISSING_TASK_ID,
                f"taskId is {task_id!r}; a non-empty string is required",
            )
        identifier = task_id.strip()

        if isinstance(pickup, (str, bytes)) or not isinstance(pickup, Sequence) or len(pickup) != 2:
            raise MissionRejected(
                MissionRejectReason.MALFORMED, "pickup is not a (lat, lon) pair", task_id=identifier
            )
        if isinstance(drop, (str, bytes)) or not isinstance(drop, Sequence) or len(drop) != 2:
            raise MissionRejected(
                MissionRejectReason.MALFORMED, "drop is not a (lat, lon) pair", task_id=identifier
            )

        try:
            return cls(
                task_id=identifier,
                pickup=validate_coordinate(pickup[0], pickup[1], where="pickup"),
                drop=validate_coordinate(drop[0], drop[1], where="drop"),
                path_to_pickup=validate_path(path_to_pickup, where="pathToPickup"),
                path_to_drop=validate_path(path_to_drop, where="pathToDrop"),
                assigned_at=validate_timestamp(timestamp),
            )
        except MissionRejected as e:
            # Re-raise carrying the task id, so a rejection can be reported
            # against the task it was refused for.
            raise MissionRejected(e.reason, e.detail, task_id=identifier) from None

    def route_for(self, segment: MissionSegment) -> Tuple[LatLon, ...]:
        """The waypoints RobotX supplied for one leg. Never a computed route."""

        return (
            self.path_to_pickup
            if segment is MissionSegment.TO_PICKUP
            else self.path_to_drop
        )

    def destination_for(self, segment: MissionSegment) -> LatLon:
        """Where a leg ends: the pickup or the drop, as RobotX stated it."""

        return self.pickup if segment is MissionSegment.TO_PICKUP else self.drop

    def to_dict(self) -> Dict[str, Any]:
        def point(p: LatLon) -> Dict[str, float]:
            return {"lat": round(p[0], 7), "lon": round(p[1], 7)}

        return {
            "task_id": self.task_id,
            "pickup": point(self.pickup),
            "drop": point(self.drop),
            # Lengths rather than the full lists: this goes into telemetry at
            # rate, and re-sending RobotX its own route every second would be
            # bandwidth spent telling it what it already knows.
            "path_to_pickup_points": len(self.path_to_pickup),
            "path_to_drop_points": len(self.path_to_drop),
            "assigned_at": self.assigned_at,
        }


@dataclass(frozen=True)
class ActiveMission:
    """The mission the Rover holds, plus exactly how far through it it is.

    A snapshot, produced whenever progress changes and written into
    `RobotState` like every other per-tick result. Nothing reads mission
    progress from the manager directly, so telemetry, health and the backend
    link all see one consistent answer.
    """

    mission: Mission
    status: MissionStatus
    segment: MissionSegment
    accepted_at: float
    # Index into the *current* leg's waypoint list. Owned by the route planner,
    # mirrored here so one snapshot answers "where in the mission are we".
    waypoint_index: int = 0
    waypoints_total: int = 0
    pickup_reached_at: Optional[float] = None
    completed_at: Optional[float] = None
    # Whether each arrival was decided on a *measured* position. Navigation
    # also runs on a dead-reckoned pose, which with no motor link describes
    # where the Rover was told to go rather than where it went -- so an arrival
    # decided on one is a fact about the plan, not about the Rover.
    pickup_measured: bool = False
    drop_measured: bool = False
    # Custody. Required for engine missions: the Rover then holds at each stop
    # until the handover genuinely happens. The two instants are set only by
    # `MissionManager.record_custody`, i.e. by a real custody observation --
    # never by arriving, by a timer, or by a command.
    custody_required: bool = False
    custody_acquired_at: Optional[float] = None
    custody_released_at: Optional[float] = None
    drop_reached_at: Optional[float] = None

    @property
    def task_id(self) -> str:
        return self.mission.task_id

    @property
    def route(self) -> Tuple[LatLon, ...]:
        """The waypoints currently being followed."""

        return self.mission.route_for(self.segment)

    @property
    def destination(self) -> LatLon:
        return self.mission.destination_for(self.segment)

    @property
    def is_complete(self) -> bool:
        return self.status is MissionStatus.COMPLETE

    @property
    def is_active(self) -> bool:
        return self.status.is_active

    @property
    def arrivals_measured(self) -> bool:
        """Whether both legs ended on a measured position.

        The bar for reporting a completion to the backend. Either leg decided
        on an inferred pose means the Rover cannot show it was ever at the
        pickup or the drop.
        """

        return self.pickup_measured and self.drop_measured

    def to_dict(self) -> Dict[str, Any]:
        return {
            "task_id": self.task_id,
            "status": self.status.value,
            "segment": self.segment.value,
            "waypoint_index": self.waypoint_index,
            "waypoints_total": self.waypoints_total,
            "accepted_at": self.accepted_at,
            "pickup_reached_at": self.pickup_reached_at,
            "completed_at": self.completed_at,
            "pickup_measured": self.pickup_measured,
            "drop_measured": self.drop_measured,
            "custody_required": self.custody_required,
            "custody_acquired_at": self.custody_acquired_at,
            "custody_released_at": self.custody_released_at,
            "drop_reached_at": self.drop_reached_at,
            "mission": self.mission.to_dict(),
        }
