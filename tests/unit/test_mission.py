"""The Pi-side mission consumer: validate, hold, follow, finish.

Three layers, tested where each one's rules actually live:

1. `parse_task_assign` -- the wire boundary. Everything RobotX could send that
   the Rover must refuse.
2. `MissionManager` -- progression over the *supplied* waypoints, driven by the
   existing navigator's verdict rather than by any arithmetic of its own.
3. `RobotAgent` -- the whole path, ticked, from assignment to TASK_COMPLETE.

Every payload here is synthetic (`tests.fixtures.task_assign`) and built to the
verified contract by hand. Nothing in this file demonstrates that RobotX emits
such a payload, or emits it to this Rover -- the assignment engine was not live
when these were written, and passing tests must not be read as saying otherwise.
"""

import logging
import math
import time
import unittest

from robotx.application.agent import RobotAgent
from robotx.communication.protocol import build_status_payload, parse_task_assign, ProtocolBinding
from robotx.config.settings import Settings
from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus
from robotx.localization.position import Position
from robotx.mission.manager import MissionManager
from robotx.mission.mission import (
    MAX_PATH_POINTS,
    Mission,
    MissionRejected,
    MissionRejectReason,
    MissionSegment,
    MissionStatus,
)
from robotx.navigation.navigator import NavigationConfig, NavigationStatus, Navigator
from robotx.state.robot_state import OperatingMode
from robotx.state.telemetry import build_telemetry
from tests.fixtures.task_assign import (
    DROP,
    PICKUP,
    START,
    offset,
    path_to_drop,
    path_to_pickup,
    task_assign_payload,
    wire_path,
    wire_point,
)


def setUpModule():
    # These tests drive rejection and abort paths on purpose.
    logging.getLogger("robotx").setLevel(logging.CRITICAL)


def mission_from_fixture(**overrides):
    return parse_task_assign(task_assign_payload(**overrides))


def rejection(test, payload, *, expected_robot_id=None):
    """Parse `payload`, require a refusal, and hand back the reason."""

    with test.assertRaises(MissionRejected) as caught:
        parse_task_assign(payload, expected_robot_id=expected_robot_id)
    return caught.exception


# --- 1. the wire boundary -----------------------------------------------------


class TestValidAssignment(unittest.TestCase):
    def test_the_verified_schema_parses_into_a_mission(self):
        payload = task_assign_payload(task_id="task-42")
        mission = parse_task_assign(payload)

        self.assertIsInstance(mission, Mission)
        self.assertEqual(mission.task_id, "task-42")
        self.assertAlmostEqual(mission.pickup[0], PICKUP[0])
        self.assertAlmostEqual(mission.pickup[1], PICKUP[1])
        self.assertAlmostEqual(mission.drop[0], DROP[0])
        self.assertAlmostEqual(mission.drop[1], DROP[1])
        self.assertEqual(len(mission.path_to_pickup), len(payload["pathToPickup"]))
        self.assertEqual(len(mission.path_to_drop), len(payload["pathToDrop"]))

    def test_waypoints_keep_the_order_and_values_robotx_sent(self):
        """The Rover follows RobotX's route. Reordering or resampling it would
        put the Rover somewhere the backend cannot account for."""

        payload = task_assign_payload()
        mission = parse_task_assign(payload)

        for supplied, parsed in zip(payload["pathToPickup"], mission.path_to_pickup):
            self.assertAlmostEqual(supplied["lat"], parsed[0])
            self.assertAlmostEqual(supplied["lon"], parsed[1])

    def test_millisecond_timestamps_become_unix_seconds(self):
        issued_ms = 1_750_000_000_000
        mission = parse_task_assign(task_assign_payload(timestamp=issued_ms))
        self.assertAlmostEqual(mission.assigned_at, issued_ms / 1000.0, places=3)

    def test_a_mission_is_immutable_once_accepted(self):
        mission = mission_from_fixture()
        with self.assertRaises(Exception):
            mission.task_id = "something-else"  # type: ignore[misc]
        self.assertIsInstance(mission.path_to_pickup, tuple)

    def test_each_leg_is_addressable_by_segment(self):
        mission = mission_from_fixture()
        self.assertEqual(mission.route_for(MissionSegment.TO_PICKUP), mission.path_to_pickup)
        self.assertEqual(mission.route_for(MissionSegment.TO_DROP), mission.path_to_drop)
        self.assertEqual(mission.destination_for(MissionSegment.TO_DROP), mission.drop)

    def test_a_robotid_naming_this_rover_is_accepted(self):
        payload = task_assign_payload(robotId="rover-1")
        self.assertEqual(
            parse_task_assign(payload, expected_robot_id="rover-1").task_id,
            payload["taskId"],
        )


class TestMalformedAssignment(unittest.TestCase):
    """Nothing malformed is absorbed quietly; every refusal names its reason."""

    def test_a_non_object_payload_is_refused(self):
        for payload in ("TASK_ASSIGN", 7, None, ["taskId"]):
            self.assertIs(
                rejection(self, payload).reason, MissionRejectReason.MALFORMED, payload
            )

    def test_a_missing_or_empty_task_id_is_refused(self):
        for value in (None, "", "   ", 17):
            self.assertIs(
                rejection(self, task_assign_payload(taskId=value)).reason,
                MissionRejectReason.MISSING_TASK_ID,
                value,
            )

    def test_a_missing_task_id_key_is_refused(self):
        payload = task_assign_payload()
        del payload["taskId"]
        self.assertIs(rejection(self, payload).reason, MissionRejectReason.MISSING_TASK_ID)

    def test_a_point_that_is_not_an_object_is_refused(self):
        for bad in ([12.97, 77.59], "12.97,77.59", 12.97, None):
            self.assertIs(
                rejection(self, task_assign_payload(pickup=bad)).reason,
                MissionRejectReason.MALFORMED,
                bad,
            )

    def test_a_point_missing_an_axis_is_refused(self):
        self.assertIs(
            rejection(self, task_assign_payload(drop={"lat": 12.97})).reason,
            MissionRejectReason.MALFORMED,
        )

    def test_an_alternative_spelling_is_not_quietly_accepted(self):
        """`lng` is not in the contract. Absorbing it would make the Rover
        understand a second route format nobody agreed to."""

        self.assertIs(
            rejection(self, task_assign_payload(drop={"lat": 12.97, "lng": 77.59})).reason,
            MissionRejectReason.MALFORMED,
        )

    def test_a_path_that_is_not_an_array_is_refused(self):
        for bad in ({"lat": 1, "lon": 2}, "waypoints", None, 5):
            self.assertIs(
                rejection(self, task_assign_payload(pathToDrop=bad)).reason,
                MissionRejectReason.MALFORMED,
                bad,
            )

    def test_a_missing_timestamp_is_refused(self):
        payload = task_assign_payload()
        del payload["timestamp"]
        self.assertIs(rejection(self, payload).reason, MissionRejectReason.INVALID_TIMESTAMP)

    def test_an_unreadable_or_implausible_timestamp_is_refused(self):
        for value in (None, "", "yesterday", 0, -1, 12345):
            self.assertIs(
                rejection(self, task_assign_payload(timestamp=value)).reason,
                MissionRejectReason.INVALID_TIMESTAMP,
                value,
            )

    def test_an_assignment_for_another_robot_is_refused(self):
        refusal = rejection(
            self,
            task_assign_payload(robotId="some-other-rover"),
            expected_robot_id="rover-1",
        )
        self.assertIs(refusal.reason, MissionRejectReason.WRONG_ROBOT)

    def test_a_refusal_carries_the_task_it_refers_to(self):
        refusal = rejection(self, task_assign_payload(task_id="task-9", pathToDrop=[]))
        self.assertEqual(refusal.task_id, "task-9")
        self.assertIn("pathToDrop", str(refusal))


class TestEmptyRoute(unittest.TestCase):
    def test_an_empty_path_is_refused(self):
        for key in ("pathToPickup", "pathToDrop"):
            self.assertIs(
                rejection(self, task_assign_payload(**{key: []})).reason,
                MissionRejectReason.EMPTY_ROUTE,
                key,
            )

    def test_a_single_point_is_not_a_route(self):
        """One point says where to end up and nothing about how to get there."""

        payload = task_assign_payload(pathToPickup=wire_path([PICKUP]))
        self.assertIs(rejection(self, payload).reason, MissionRejectReason.EMPTY_ROUTE)

    def test_an_absurdly_long_route_is_refused_by_the_domain(self):
        too_many = [offset(START, north_m=float(i)) for i in range(MAX_PATH_POINTS + 1)]
        with self.assertRaises(MissionRejected) as caught:
            Mission.create(
                task_id="task-1",
                pickup=PICKUP,
                drop=DROP,
                path_to_pickup=too_many,
                path_to_drop=path_to_drop(),
                timestamp=time.time(),
            )
        self.assertIs(caught.exception.reason, MissionRejectReason.ROUTE_TOO_LONG)

    def test_an_oversized_payload_is_refused_before_it_is_walked(self):
        """A route that long does not fit the inbound size limit either, and
        the size check is what a hostile payload meets first."""

        huge = wire_path([offset(START, north_m=float(i)) for i in range(MAX_PATH_POINTS + 1)])
        self.assertIs(
            rejection(self, task_assign_payload(pathToPickup=huge)).reason,
            MissionRejectReason.MALFORMED,
        )


class TestInvalidCoordinates(unittest.TestCase):
    def test_non_finite_coordinates_are_refused(self):
        for value in (float("nan"), float("inf"), float("-inf")):
            payload = task_assign_payload(pickup={"lat": value, "lon": 77.59})
            self.assertIs(
                rejection(self, payload).reason,
                MissionRejectReason.INVALID_COORDINATE,
                value,
            )

    def test_out_of_range_coordinates_are_refused(self):
        for lat, lon in ((91.0, 77.59), (-90.5, 77.59), (12.97, 181.0), (12.97, -180.5)):
            payload = task_assign_payload(drop={"lat": lat, "lon": lon})
            self.assertIs(
                rejection(self, payload).reason,
                MissionRejectReason.INVALID_COORDINATE,
                (lat, lon),
            )

    def test_non_numeric_coordinates_are_refused(self):
        for value in ("12.9716", None, True, [12.97]):
            payload = task_assign_payload(pickup={"lat": value, "lon": 77.59})
            self.assertIs(
                rejection(self, payload).reason,
                MissionRejectReason.INVALID_COORDINATE,
                value,
            )

    def test_one_bad_waypoint_refuses_the_whole_mission(self):
        """A route with a hole in it is not a shorter route, it is a route
        that goes somewhere nobody chose."""

        points = wire_path(path_to_pickup())
        points[2] = {"lat": float("nan"), "lon": 77.59}
        refusal = rejection(self, task_assign_payload(pathToPickup=points))
        self.assertIs(refusal.reason, MissionRejectReason.INVALID_COORDINATE)
        self.assertIn("pathToPickup[2]", refusal.detail)


# --- 2. progression over the supplied route -----------------------------------


def navigator_at(arrival_m=8.0):
    return Navigator(NavigationConfig(waypoint_arrival_m=arrival_m))


def pose(point, *, heading_deg=None):
    return Position(
        latitude=point[0],
        longitude=point[1],
        timestamp=time.time(),
        heading_deg=heading_deg,
    )


class MissionHarness:
    """A manager, a real navigator, and a way to stand the Rover somewhere."""

    def __init__(self, mission=None, arrival_m=8.0):
        self.navigator = navigator_at(arrival_m)
        self.manager = MissionManager(self.navigator)
        self.mission = mission or parse_task_assign(task_assign_payload())

    def assign(self, mission=None):
        return self.manager.assign(mission or self.mission)

    def stand_at(self, point):
        """One tick with the Rover at `point`. Returns the mission update."""

        navigation = self.navigator.update(pose(point))
        return self.manager.update(navigation)

    def drive(self, points):
        return [self.stand_at(point) for point in points]


class TestWaypointProgression(unittest.TestCase):
    def test_assignment_loads_the_supplied_pickup_route_verbatim(self):
        harness = MissionHarness()
        assignment = harness.assign()

        self.assertFalse(assignment.duplicate)
        self.assertIs(assignment.active.status, MissionStatus.TO_PICKUP)
        self.assertIs(assignment.active.segment, MissionSegment.TO_PICKUP)
        self.assertEqual(
            harness.navigator.planner._route, list(harness.mission.path_to_pickup)
        )
        self.assertEqual(harness.navigator.state.destination, harness.mission.pickup)

    def test_the_waypoint_index_advances_one_waypoint_at_a_time(self):
        harness = MissionHarness()
        harness.assign()
        route = harness.mission.path_to_pickup

        indices = [harness.stand_at(point).active.waypoint_index for point in route[:-1]]
        self.assertEqual(indices, list(range(1, len(route))))

    def test_progression_is_deterministic(self):
        """The same positions in the same order always produce the same
        mission states -- no timers, no hysteresis, nothing to race."""

        def run():
            harness = MissionHarness()
            harness.assign()
            points = list(harness.mission.path_to_pickup) + list(harness.mission.path_to_drop)
            return [
                (u.active.status.value, u.active.segment.value, u.active.waypoint_index)
                for u in harness.drive(points)
            ]

        self.assertEqual(run(), run())

    def test_standing_still_short_of_a_waypoint_does_not_advance(self):
        harness = MissionHarness()
        harness.assign()
        away = offset(harness.mission.path_to_pickup[0], north_m=30.0)

        for _ in range(5):
            update = harness.stand_at(away)
        self.assertEqual(update.active.waypoint_index, 0)
        self.assertIs(update.active.status, MissionStatus.TO_PICKUP)

    def test_the_mission_reports_the_leg_it_is_driving(self):
        harness = MissionHarness()
        active = harness.assign().active
        self.assertEqual(active.waypoints_total, len(harness.mission.path_to_pickup))
        self.assertEqual(active.route, harness.mission.path_to_pickup)
        self.assertEqual(active.destination, harness.mission.pickup)


class TestPickupAndDropTransition(unittest.TestCase):
    def setUp(self):
        self.harness = MissionHarness()
        self.harness.assign()

    def test_reaching_the_pickup_completes_the_first_leg(self):
        updates = self.harness.drive(self.harness.mission.path_to_pickup)
        final = updates[-1]

        self.assertTrue(final.pickup_reached)
        self.assertIs(final.active.status, MissionStatus.AT_PICKUP)
        self.assertIsNotNone(final.active.pickup_reached_at)
        # Still the pickup leg: the drop route has not been loaded yet.
        self.assertIs(final.active.segment, MissionSegment.TO_PICKUP)

    def test_the_next_tick_starts_the_supplied_drop_route(self):
        self.harness.drive(self.harness.mission.path_to_pickup)
        update = self.harness.stand_at(PICKUP)

        self.assertTrue(update.segment_changed)
        self.assertIs(update.active.status, MissionStatus.TO_DROP)
        self.assertIs(update.active.segment, MissionSegment.TO_DROP)
        self.assertEqual(update.active.waypoint_index, 0)
        self.assertEqual(
            self.harness.navigator.planner._route, list(self.harness.mission.path_to_drop)
        )

    def test_the_final_destination_becomes_the_drop(self):
        self.harness.drive(self.harness.mission.path_to_pickup)
        self.harness.stand_at(PICKUP)

        state = self.harness.navigator.update(pose(PICKUP))
        self.assertEqual(state.destination, self.harness.mission.drop)
        self.assertEqual(self.harness.manager.active.destination, DROP)

    def test_pickup_is_only_reached_once(self):
        self.harness.drive(self.harness.mission.path_to_pickup)
        reached_at = self.harness.manager.active.pickup_reached_at

        self.harness.drive(self.harness.mission.path_to_drop)
        self.assertEqual(self.harness.manager.active.pickup_reached_at, reached_at)


class TestMissionCompletion(unittest.TestCase):
    def _run_to_completion(self, harness):
        harness.drive(harness.mission.path_to_pickup)
        harness.stand_at(PICKUP)  # the tick that loads the drop leg
        return harness.drive(harness.mission.path_to_drop)

    def test_reaching_the_drop_completes_the_mission(self):
        harness = MissionHarness()
        harness.assign()
        final = self._run_to_completion(harness)[-1]

        self.assertTrue(final.completed)
        self.assertIs(final.active.status, MissionStatus.COMPLETE)
        self.assertTrue(final.active.is_complete)
        self.assertIsNotNone(final.active.completed_at)

    def test_completion_drops_the_route(self):
        """Nothing should keep steering toward a destination already reached."""

        harness = MissionHarness()
        harness.assign()
        self._run_to_completion(harness)

        self.assertEqual(harness.navigator.planner.route_length(), 0)
        self.assertIs(harness.navigator.update(pose(DROP)).status, NavigationStatus.IDLE)
        self.assertEqual(harness.manager.current_route(), ())

    def test_a_completed_mission_does_not_advance_further(self):
        harness = MissionHarness()
        harness.assign()
        self._run_to_completion(harness)
        completed_at = harness.manager.active.completed_at

        for _ in range(3):
            update = harness.stand_at(DROP)
        self.assertIs(update.active.status, MissionStatus.COMPLETE)
        self.assertEqual(update.active.completed_at, completed_at)
        self.assertFalse(update.completed)

    def test_a_finished_mission_is_kept_so_it_can_still_be_reported(self):
        harness = MissionHarness()
        harness.assign()
        self._run_to_completion(harness)

        self.assertIsNotNone(harness.manager.active)
        self.assertFalse(harness.manager.has_active_mission)

    def test_updates_with_no_mission_are_harmless(self):
        harness = MissionHarness()
        update = harness.stand_at(START)
        self.assertIsNone(update.active)
        self.assertFalse(update.completed)


class TestDuplicateAndConflictingAssignments(unittest.TestCase):
    def test_redelivering_the_same_task_continues_from_where_the_rover_is(self):
        harness = MissionHarness()
        harness.assign()
        harness.stand_at(harness.mission.path_to_pickup[0])
        index_before = harness.manager.active.waypoint_index

        again = harness.manager.assign(harness.mission)

        self.assertTrue(again.duplicate)
        self.assertEqual(again.active.waypoint_index, index_before)
        self.assertEqual(harness.navigator.planner.waypoint_index(), index_before)

    def test_a_redelivery_does_not_restart_the_drop_leg(self):
        harness = MissionHarness()
        harness.assign()
        harness.drive(harness.mission.path_to_pickup)
        harness.stand_at(PICKUP)

        again = harness.manager.assign(harness.mission)

        self.assertTrue(again.duplicate)
        self.assertIs(again.active.segment, MissionSegment.TO_DROP)
        self.assertEqual(
            harness.navigator.planner._route, list(harness.mission.path_to_drop)
        )

    def test_a_second_task_is_refused_while_one_is_running(self):
        harness = MissionHarness()
        harness.assign()
        other = parse_task_assign(task_assign_payload(task_id="task-other"))

        with self.assertRaises(MissionRejected) as caught:
            harness.manager.assign(other)
        self.assertIs(caught.exception.reason, MissionRejectReason.MISSION_ACTIVE)
        # And the Rover keeps driving the one it already has.
        self.assertEqual(harness.manager.active.task_id, harness.mission.task_id)

    def test_a_completed_task_is_not_run_again_on_redelivery(self):
        harness = MissionHarness()
        harness.assign()
        harness.drive(harness.mission.path_to_pickup)
        harness.stand_at(PICKUP)
        harness.drive(harness.mission.path_to_drop)

        with self.assertRaises(MissionRejected) as caught:
            harness.manager.assign(harness.mission)
        self.assertIs(caught.exception.reason, MissionRejectReason.DUPLICATE_TASK)

    def test_a_new_task_is_accepted_once_the_last_one_finished(self):
        harness = MissionHarness()
        harness.assign()
        harness.drive(harness.mission.path_to_pickup)
        harness.stand_at(PICKUP)
        harness.drive(harness.mission.path_to_drop)

        nxt = parse_task_assign(task_assign_payload(task_id="task-next"))
        assignment = harness.manager.assign(nxt)

        self.assertFalse(assignment.duplicate)
        self.assertEqual(assignment.active.task_id, "task-next")
        self.assertIs(assignment.active.status, MissionStatus.TO_PICKUP)

    def test_an_abandoned_task_frees_the_rover_for_the_next_one(self):
        harness = MissionHarness()
        harness.assign()
        harness.manager.abandon("operator stop")

        self.assertIs(harness.manager.active.status, MissionStatus.ABORTED)
        nxt = parse_task_assign(task_assign_payload(task_id="task-next"))
        self.assertFalse(harness.manager.assign(nxt).duplicate)

    def test_an_abandoned_mission_can_never_complete(self):
        """The decisive one: a mission left active after its route was taken
        away would be 'arrived' by whatever route replaced it."""

        harness = MissionHarness()
        harness.assign()
        harness.manager.abandon("operator stop")

        harness.navigator.set_route([START, DROP])
        update = harness.stand_at(DROP)

        self.assertIs(update.active.status, MissionStatus.ABORTED)
        self.assertFalse(update.completed)


# --- 3. the agent, ticked -----------------------------------------------------


class FakeGps:
    """A GPS receiver that reports wherever the test says the Rover is."""

    def __init__(self, point=START):
        self.point = point

    def get_reading(self):
        return GpsReading(
            status=GPSStatus.FIX,
            fix=GpsFix(
                latitude=self.point[0],
                longitude=self.point[1],
                timestamp=time.time(),
                satellites=9,
            ),
            age_s=0.1,
        )

    def stop(self):
        pass


def headless_agent(**overrides):
    env = {
        "ROBOTX_ROBOT_ID": "test-rover",
        "ROBOTX_CAMERA_ENABLED": "0",
        "ROBOTX_PERCEPTION_ENABLED": "0",
        "ROBOTX_GPS_ENABLED": "0",
        "ROBOTX_LOG_LEVEL": "CRITICAL",
    }
    env.update(overrides)
    agent = RobotAgent(Settings.from_env(env))
    agent.gps = FakeGps()
    return agent


def tick_at(agent, point):
    agent.gps.point = point
    return agent.tick()


def run_mission(agent, mission):
    """Drive the whole assignment, one tick per supplied waypoint."""

    for point in mission.path_to_pickup:
        tick_at(agent, point)
    tick_at(agent, PICKUP)  # the tick that starts the drop leg
    for point in mission.path_to_drop:
        tick_at(agent, point)
    return agent.state.snapshot()


class TestAgentMissionConsumer(unittest.TestCase):
    def test_accepting_an_assignment_starts_the_pickup_leg(self):
        agent = headless_agent()
        mission = mission_from_fixture(task_id="task-1")

        assignment = agent.assign_mission(mission)
        snapshot = agent.state.snapshot()

        self.assertFalse(assignment.duplicate)
        self.assertIs(agent.mode, OperatingMode.AUTO)
        self.assertEqual(snapshot.mission.task_id, "task-1")
        self.assertIs(snapshot.mission.segment, MissionSegment.TO_PICKUP)
        self.assertEqual(agent.navigator.planner._route, list(mission.path_to_pickup))

    def test_the_rover_drives_the_supplied_route_to_completion(self):
        agent = headless_agent()
        mission = mission_from_fixture(task_id="task-1")
        agent.assign_mission(mission)

        snapshot = run_mission(agent, mission)

        self.assertIs(snapshot.mission.status, MissionStatus.COMPLETE)
        self.assertEqual(snapshot.mission.task_id, "task-1")
        self.assertIsNotNone(snapshot.mission.completed_at)
        # Mission over: the Rover stands down rather than staying in AUTO with
        # no route.
        self.assertIs(snapshot.mode, OperatingMode.IDLE)

    def test_mission_progress_is_visible_in_state_and_telemetry(self):
        agent = headless_agent()
        mission = mission_from_fixture(task_id="task-7")
        agent.assign_mission(mission)
        for point in mission.path_to_pickup[:2]:
            tick_at(agent, point)

        snapshot = agent.state.snapshot()
        telemetry = build_telemetry(snapshot)
        status = build_status_payload(
            snapshot, robot_id="test-rover", binding=ProtocolBinding()
        )

        self.assertEqual(telemetry["mission"]["task_id"], "task-7")
        self.assertEqual(status["mission"]["taskId"], "task-7")
        self.assertEqual(status["mission"]["status"], MissionStatus.TO_PICKUP.value)
        self.assertGreaterEqual(status["mission"]["waypointIndex"], 1)

    def test_a_rover_with_no_mission_reports_none_rather_than_a_blank_task(self):
        agent = headless_agent()
        agent.tick()
        self.assertIsNone(agent.state.snapshot().mission)
        self.assertIsNone(build_telemetry(agent.state.snapshot())["mission"])

    def test_a_redelivered_assignment_does_not_restart_the_run(self):
        agent = headless_agent()
        mission = mission_from_fixture(task_id="task-1")
        agent.assign_mission(mission)
        for point in mission.path_to_pickup[:2]:
            tick_at(agent, point)
        index_before = agent.state.snapshot().mission.waypoint_index
        self.assertGreater(index_before, 0)

        assignment = agent.assign_mission(mission)

        self.assertTrue(assignment.duplicate)
        self.assertEqual(agent.state.snapshot().mission.waypoint_index, index_before)
        self.assertEqual(agent.navigator.planner.waypoint_index(), index_before)

    def test_stopping_aborts_the_task_rather_than_leaving_it_mid_leg(self):
        agent = headless_agent()
        agent.assign_mission(mission_from_fixture(task_id="task-1"))
        agent.stop_mission("operator stop")

        snapshot = agent.state.snapshot()
        self.assertIs(snapshot.mission.status, MissionStatus.ABORTED)
        self.assertIs(snapshot.mode, OperatingMode.STOPPED)
        self.assertEqual(agent.navigator.planner.route_length(), 0)

    def test_returning_to_base_abandons_the_delivery(self):
        agent = headless_agent(ROBOTX_HOME_LAT="12.9700", ROBOTX_HOME_LON="77.5900")
        agent.assign_mission(mission_from_fixture(task_id="task-1"))
        agent.return_to_base("operator return")

        self.assertIs(agent.state.snapshot().mission.status, MissionStatus.ABORTED)
        # And driving home never reports a delivery.
        snapshot = tick_at(agent, (12.9700, 77.5900))
        self.assertIs(snapshot.mission.status, MissionStatus.ABORTED)

    def test_pausing_and_resuming_keeps_the_leg_being_driven(self):
        agent = headless_agent()
        mission = mission_from_fixture(task_id="task-1")
        agent.assign_mission(mission)
        for point in mission.path_to_pickup:
            tick_at(agent, point)
        tick_at(agent, PICKUP)  # now on the drop leg

        agent.pause_mission("operator pause")
        agent.navigator.clear_route()
        agent.resume_mission("operator resume")

        self.assertIs(agent.mode, OperatingMode.AUTO)
        self.assertEqual(agent.navigator.planner._route, list(mission.path_to_drop))

    def test_an_assignment_is_refused_while_the_emergency_stop_is_latched(self):
        agent = headless_agent()
        agent.emergency_stop("bench test")

        with self.assertRaises(MissionRejected) as caught:
            agent.assign_mission(mission_from_fixture())

        self.assertIs(caught.exception.reason, MissionRejectReason.ESTOP_ENGAGED)
        self.assertIsNone(agent.state.snapshot().mission)
        self.assertIsNot(agent.mode, OperatingMode.AUTO)

    def test_the_rover_fetches_nothing_to_get_a_route(self):
        """Structural: the mission consumer talks to no network at all. If a
        Mapbox client or a route service ever creeps onto the Pi, its import
        shows up here -- the word "Mapbox" in a docstring does not, and should
        not, because explaining whose router made the waypoints is the point."""

        import ast
        import pathlib

        import robotx.mission.manager as manager_module
        import robotx.mission.mission as mission_module

        for module in (manager_module, mission_module):
            tree = ast.parse(pathlib.Path(module.__file__).read_text())
            imported = set()
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    imported.update(alias.name.split(".")[0] for alias in node.names)
                elif isinstance(node, ast.ImportFrom) and node.module:
                    imported.add(node.module.split(".")[0])
            for networking in ("requests", "http", "urllib", "socket", "socketio", "aiohttp"):
                self.assertNotIn(networking, imported, f"{module.__name__}: {networking}")

    def test_the_rover_never_plans_a_route_of_its_own(self):
        """The navigator is handed the supplied waypoints and nothing else."""

        agent = headless_agent()
        mission = mission_from_fixture()
        agent.assign_mission(mission)

        for point in mission.path_to_pickup:
            tick_at(agent, point)
            self.assertIn(
                tuple(agent.navigator.planner.next_waypoint()),
                set(mission.path_to_pickup) | set(mission.path_to_drop),
            )


if __name__ == "__main__":
    unittest.main()
