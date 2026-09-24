"""Mission manager: holds the assignment and drives the existing Navigator.

    TASK_ASSIGN -> MissionManager -> Navigator.set_route(supplied waypoints)
                                  -> Navigator.update(pose from localization)
                                  -> NavigationState -> MissionManager.update()

The manager owns exactly one thing: which mission is active and how far
through it the Rover is. It owns no navigation algorithm, no arrival test and
no geometry -- all of that already exists in `Navigator`/`RoutePlanner` and is
called rather than reimplemented. When this module wants to know whether a leg
is finished it asks the navigator, which asked the route planner, which
measured against the pose that `robotx.localization` produced. One authority
per question.

Determinism
-----------
Progression is a function of the navigation state alone, and every transition
happens in `update()`. There is no timer, no hysteresis and no second arrival
radius here: the same sequence of positions always produces the same sequence
of mission states. That is what makes "did the Rover really reach the pickup"
a question a test can answer without a robot.

What this module will not do
----------------------------
It will not shorten, smooth, reorder or regenerate a route; it will not plan
around an obstacle; and it will not invent a waypoint. The route is RobotX's,
generated from Mapbox Directions, and the Rover follows the one it was sent or
stops. Obstacle handling stays where it already is: perception and the
decision layer slow or stop the Rover, and the safety gate has the last word.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, replace
from typing import Optional, Tuple

from robotx.config.logging_setup import log_event
from robotx.localization.position import LatLon
from robotx.mission.mission import (
    ActiveMission,
    Mission,
    MissionRejected,
    MissionRejectReason,
    MissionSegment,
    MissionStatus,
)
from robotx.navigation.navigator import NavigationState, NavigationStatus, Navigator
from robotx.state.robot_state import MissionRefused


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class MissionAssignment:
    """The outcome of accepting one `TASK_ASSIGN`.

    `duplicate` is a first-class result rather than an error: RobotX
    redelivering the mission the Rover is already driving is normal after a
    reconnect, and the correct response is to carry on, not to restart the leg
    from waypoint zero.
    """

    active: ActiveMission
    duplicate: bool = False


@dataclass(frozen=True)
class MissionUpdate:
    """What one progression step did, so the agent can react without guessing."""

    active: Optional[ActiveMission]
    segment_changed: bool = False
    pickup_reached: bool = False
    completed: bool = False


class MissionManager:
    """Tracks the active mission and keeps the navigator pointed at its route."""

    def __init__(self, navigator: Navigator) -> None:
        self._navigator = navigator
        self._active: Optional[ActiveMission] = None

    # --- reads ---------------------------------------------------------------

    @property
    def active(self) -> Optional[ActiveMission]:
        """The mission held right now, including a finished one.

        A completed or aborted mission is kept rather than discarded so that
        telemetry can still say which task ended and how, and so a redelivery
        of that same taskId is recognized as already run instead of driven
        again.
        """

        return self._active

    @property
    def has_active_mission(self) -> bool:
        return self._active is not None and self._active.status.is_active

    def current_route(self) -> Tuple[LatLon, ...]:
        """The waypoints of the leg in progress, empty when nothing is running."""

        if self._active is None or not self._active.status.is_active:
            return ()
        return self._active.route

    # --- assignment ----------------------------------------------------------

    def assign(
        self,
        mission: Mission,
        *,
        custody_required: bool = False,
        now: Optional[float] = None,
    ) -> MissionAssignment:
        """Accept a validated mission and load its pickup leg.

        Refuses rather than overwrites when another mission is still running.
        Silently replacing an in-flight mission would abandon a delivery
        halfway with nothing in the Rover's state saying it happened -- if
        RobotX means to reassign, it can stop the current task first.
        """

        now = time.time() if now is None else now
        current = self._active

        if current is not None and current.task_id == mission.task_id:
            if current.status.is_terminal:
                raise MissionRejected(
                    MissionRejectReason.DUPLICATE_TASK,
                    f"task {mission.task_id} already finished as {current.status.value}",
                    task_id=mission.task_id,
                )
            log_event(
                logger,
                "mission.duplicate",
                "already carrying out this task; continuing from where it is",
                task_id=mission.task_id,
                status=current.status.value,
                waypoint=f"{current.waypoint_index + 1}/{current.waypoints_total}",
            )
            return MissionAssignment(current, duplicate=True)

        if current is not None and current.status.is_active:
            raise MissionRejected(
                MissionRejectReason.MISSION_ACTIVE,
                f"still carrying out task {current.task_id} ({current.status.value}); "
                f"stop it before assigning {mission.task_id}",
                task_id=mission.task_id,
            )

        route = mission.path_to_pickup
        active = ActiveMission(
            mission=mission,
            status=MissionStatus.TO_PICKUP,
            segment=MissionSegment.TO_PICKUP,
            accepted_at=now,
            waypoint_index=0,
            waypoints_total=len(route),
            custody_required=custody_required,
        )
        # The navigator is told the route RobotX supplied, verbatim.
        self._navigator.set_route(route)
        self._active = active
        log_event(
            logger,
            "mission.assigned",
            task_id=mission.task_id,
            pickup_waypoints=len(mission.path_to_pickup),
            drop_waypoints=len(mission.path_to_drop),
        )
        return MissionAssignment(active, duplicate=False)

    def abandon(self, reason: str, *, now: Optional[float] = None) -> Optional[ActiveMission]:
        """End the active mission without completing it.

        Used when the route is taken away from the mission -- an operator STOP,
        a return to base. Marking it ABORTED matters: the alternative is a
        mission left in TO_DROP that the manager would later see "arrive" at
        whatever route replaced it, and report as a delivery that never
        happened.
        """

        active = self._active
        if active is None or active.status.is_terminal:
            return active
        self._active = replace(active, status=MissionStatus.ABORTED)
        log_event(
            logger,
            "mission.aborted",
            reason,
            level=logging.WARNING,
            task_id=active.task_id,
            segment=active.segment.value,
        )
        return self._active

    def clear(self) -> None:
        """Forget the mission entirely, finished or not."""

        self._active = None

    # --- progression ---------------------------------------------------------

    def update(
        self,
        navigation: NavigationState,
        *,
        position_measured: bool = False,
        now: Optional[float] = None,
    ) -> MissionUpdate:
        """Advance the mission by one tick, given the navigator's verdict.

        Called once per agent tick, after `Navigator.update(position)`. Every
        transition is decided here and nowhere else.

        `position_measured` says whether the pose that verdict was computed
        from was observed (GPS) rather than inferred. It does not change any
        transition -- it is recorded on the arrival so the backend link can
        tell a delivery the Rover can vouch for from one it cannot. Defaults
        to False: an arrival nobody vouched for is not a measured one.
        """

        now = time.time() if now is None else now
        active = self._active

        if active is None or active.status.is_terminal:
            return MissionUpdate(active)

        # Mirror the planner's waypoint index rather than counting separately.
        # Two counters would eventually disagree, and then neither could be
        # trusted to say where the Rover is.
        active = replace(
            active,
            waypoint_index=navigation.waypoint_index,
            waypoints_total=navigation.waypoints_total or len(active.route),
        )

        # Pickup reached on a previous tick: start the drop leg. Done as its
        # own step so `AT_PICKUP` is a state the Rover actually passes through
        # and reports, and so the handover hold this leaves room for has an
        # obvious place to live.
        # Holding for a handover. Nothing moves the mission on but the custody
        # observation itself.
        if active.status is MissionStatus.AT_PICKUP and active.custody_required \
                and active.custody_acquired_at is None:
            self._active = active
            return MissionUpdate(active)
        if active.status is MissionStatus.AT_DROP:
            if active.custody_released_at is None:
                self._active = active
                return MissionUpdate(active)
            active = replace(active, status=MissionStatus.COMPLETE,
                             completed_at=active.custody_released_at)
            self._active = active
            log_event(logger, "mission.completed", task_id=active.task_id,
                      duration_s=round(now - active.accepted_at, 1))
            return MissionUpdate(active, completed=True)

        if active.status is MissionStatus.AT_PICKUP:
            drop_route = active.mission.path_to_drop
            active = replace(
                active,
                status=MissionStatus.TO_DROP,
                segment=MissionSegment.TO_DROP,
                waypoint_index=0,
                waypoints_total=len(drop_route),
            )
            self._navigator.set_route(drop_route)
            self._active = active
            log_event(
                logger,
                "mission.leg_started",
                task_id=active.task_id,
                segment=active.segment.value,
                waypoints=len(drop_route),
            )
            return MissionUpdate(active, segment_changed=True)

        if navigation.status is not NavigationStatus.ARRIVED:
            self._active = active
            return MissionUpdate(active)

        if active.segment is MissionSegment.TO_PICKUP:
            active = replace(
                active,
                status=MissionStatus.AT_PICKUP,
                pickup_reached_at=now,
                pickup_measured=position_measured,
            )
            self._active = active
            log_event(
                logger,
                "mission.pickup_reached",
                task_id=active.task_id,
                waypoints=active.waypoints_total,
            )
            return MissionUpdate(active, pickup_reached=True)

        if active.custody_required:
            # At the drop, but not delivered until custody is released.
            active = replace(
                active,
                status=MissionStatus.AT_DROP,
                drop_reached_at=now,
                drop_measured=position_measured,
            )
            self._active = active
            self._navigator.clear_route()
            log_event(logger, "mission.drop_reached", task_id=active.task_id)
            return MissionUpdate(active)

        # Arrived at the end of the drop leg: the delivery is done.
        active = replace(
            active,
            status=MissionStatus.COMPLETE,
            completed_at=now,
            drop_measured=position_measured,
        )
        self._active = active
        # Drop the route so nothing keeps steering toward a destination the
        # Rover is already standing at.
        self._navigator.clear_route()
        log_event(
            logger,
            "mission.completed",
            task_id=active.task_id,
            duration_s=round(now - active.accepted_at, 1),
        )
        return MissionUpdate(active, completed=True)

    # --- custody --------------------------------------------------------------

    def record_custody(self, kind: str, *, source: str, now: Optional[float] = None) -> ActiveMission:
        """Record a genuine handover observed at the stop the Rover is holding at.

        `ACQUIRED` only at the pickup, `RELEASED` only at the drop, each only
        after an arrival decided on a *measured* position (the backend applies
        a custody report only once it has verified that arrival itself), and
        each only once. Anything else raises `MissionRefused`.

        `source` names what observed the handover. It is logged, because a
        custody report is only as good as whatever produced it.
        """

        now = time.time() if now is None else now
        active = self._active
        if active is None or not active.custody_required:
            raise MissionRefused("no mission that requires custody is in progress")

        if kind == "ACQUIRED":
            if active.status is not MissionStatus.AT_PICKUP or not active.pickup_measured:
                raise MissionRefused(
                    f"custody can be acquired only at a measured pickup arrival (status {active.status.value})"
                )
            if active.custody_acquired_at is not None:
                raise MissionRefused("custody was already acquired")
            active = replace(active, custody_acquired_at=now)
        elif kind == "RELEASED":
            if active.status is not MissionStatus.AT_DROP or not active.drop_measured:
                raise MissionRefused(
                    f"custody can be released only at a measured drop arrival (status {active.status.value})"
                )
            if active.custody_acquired_at is None or active.custody_released_at is not None:
                raise MissionRefused("custody cannot be released: not held, or already released")
            active = replace(active, custody_released_at=now)
        else:
            raise MissionRefused(f"unknown custody kind {kind!r}")

        self._active = active
        log_event(logger, "mission.custody", task_id=active.task_id, kind=kind, source=source)
        return active
