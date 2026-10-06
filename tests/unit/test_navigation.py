"""Navigation: route progress, rerouting, and desired-heading computation."""

import time
import unittest

from robotx.navigation.navigator import (
    NavigationConfig,
    NavigationStatus,
    Navigator,
)
from robotx.navigation.route_planner import PlannerConfig, RoutePlanner, segment_distance_m
from robotx.localization.position import HeadingSource, Position
from tests.fixtures.task_assign import offset


def position(lat, lon, heading=None):
    return Position(
        latitude=lat,
        longitude=lon,
        timestamp=time.time(),
        heading_deg=heading,
        heading_source=HeadingSource.GPS_TRACK if heading is not None else HeadingSource.NONE,
    )


# Roughly 11 m apart in latitude at this longitude.
A = (51.500000, -0.100000)
B = (51.500100, -0.100000)
C = (51.500200, -0.100000)


class TestRoutePlanner(unittest.TestCase):
    def setUp(self):
        self.planner = RoutePlanner(PlannerConfig(waypoint_arrival_m=8.0, off_route_m=25.0))

    def test_empty_route(self):
        self.assertFalse(self.planner.has_route())
        self.assertIsNone(self.planner.next_waypoint())
        self.assertIsNone(self.planner.destination())
        self.assertEqual(self.planner.progress(), 0.0)

    def test_route_endpoints(self):
        self.planner.set_route([A, B, C])
        self.assertTrue(self.planner.has_route())
        self.assertEqual(self.planner.next_waypoint(), A)
        self.assertEqual(self.planner.destination(), C)
        self.assertEqual(self.planner.route_length(), 3)

    def test_waypoint_advances_on_arrival(self):
        self.planner.set_route([A, B, C])
        self.planner.update_position(A)
        self.assertEqual(self.planner.next_waypoint(), B)

    def test_waypoint_does_not_advance_when_far(self):
        self.planner.set_route([A, B, C])
        self.planner.update_position((51.6, -0.1))  # ~11 km away
        self.assertEqual(self.planner.next_waypoint(), A)

    def test_progress_increases_along_route(self):
        self.planner.set_route([A, B, C])
        self.assertEqual(self.planner.progress(), 0.0)
        self.planner.update_position(A)
        self.assertAlmostEqual(self.planner.progress(), 0.5)
        self.planner.update_position(B)
        self.assertAlmostEqual(self.planner.progress(), 1.0)

    def test_arrived_only_at_final_waypoint(self):
        self.planner.set_route([A, B, C])
        self.planner.update_position(A)
        self.assertFalse(self.planner.arrived(A))
        self.planner.update_position(B)
        self.planner.update_position(C)
        self.assertTrue(self.planner.arrived(C))

    def test_blocked_reports_trigger_reroute(self):
        planner = RoutePlanner(PlannerConfig(blocked_reroute_after_n=3))
        planner.set_route([A, B, C])
        self.assertFalse(planner.should_reroute())
        for _ in range(3):
            planner.report_blocked()
        self.assertTrue(planner.should_reroute())

    def test_off_route_needs_repeated_evidence(self):
        planner = RoutePlanner(PlannerConfig(off_route_m=25.0, reroute_after_n=3))
        planner.set_route([A, B, C])
        # The first fix after a route is set starts the segment to waypoint 0
        # (it has no previous waypoint), so it is on the route by definition.
        planner.update_position(A)
        far = (51.500050, -0.098000)  # ~140 m east of the A->B segment
        planner.update_position(far)
        self.assertFalse(planner.should_reroute())  # one bad fix is not a reroute
        planner.update_position(far)
        planner.update_position(far)
        self.assertTrue(planner.should_reroute())

    def test_setting_a_route_clears_counters(self):
        self.planner.set_route([A, B, C])
        for _ in range(10):
            self.planner.report_blocked()
        self.planner.set_route([A, B])
        self.assertFalse(self.planner.should_reroute())
        self.assertEqual(self.planner.waypoint_index(), 0)


class TestNavigator(unittest.TestCase):
    def setUp(self):
        self.navigator = Navigator(NavigationConfig(waypoint_arrival_m=8.0))

    def test_idle_without_a_route(self):
        state = self.navigator.update(position(*A))
        self.assertIs(state.status, NavigationStatus.IDLE)
        self.assertIsNone(state.target_waypoint)

    def test_route_without_position_reports_no_position(self):
        self.navigator.set_route([B, C])
        state = self.navigator.update(None)
        self.assertIs(state.status, NavigationStatus.NO_POSITION)
        self.assertEqual(state.destination, C)

    def test_navigating_computes_distance_and_heading(self):
        self.navigator.set_route([B, C])
        state = self.navigator.update(position(*A, heading=0.0))

        self.assertIs(state.status, NavigationStatus.NAVIGATING)
        self.assertEqual(state.target_waypoint, B)
        self.assertGreater(state.distance_to_target_m, 5.0)
        self.assertLess(state.distance_to_target_m, 20.0)
        # B is due north of A.
        self.assertAlmostEqual(state.desired_heading_deg, 0.0, places=1)
        self.assertAlmostEqual(state.heading_error_deg, 0.0, places=1)

    def test_heading_error_is_none_without_a_heading(self):
        self.navigator.set_route([B, C])
        state = self.navigator.update(position(*A))
        self.assertIsNone(state.current_heading_deg)
        self.assertIsNone(state.heading_error_deg)
        self.assertIsNotNone(state.desired_heading_deg)

    def test_heading_error_signed_toward_shorter_turn(self):
        self.navigator.set_route([B])
        # Facing east, target is due north: a 90-degree left turn.
        state = self.navigator.update(position(*A, heading=90.0))
        self.assertAlmostEqual(state.heading_error_deg, -90.0, places=1)

    def test_arrival_at_destination(self):
        self.navigator.set_route([B])
        state = self.navigator.update(position(*B, heading=0.0))
        self.assertIs(state.status, NavigationStatus.ARRIVED)

    def test_blocked_reports_lead_to_reroute_status(self):
        navigator = Navigator(NavigationConfig(blocked_reroute_after_n=2))
        navigator.set_route([B, C])
        navigator.report_blocked()
        navigator.report_blocked()
        state = navigator.update(position(*A, heading=0.0))
        self.assertIs(state.status, NavigationStatus.REROUTE_NEEDED)

    def test_clear_route_returns_to_idle(self):
        self.navigator.set_route([B, C])
        self.navigator.clear_route()
        self.assertIs(self.navigator.update(position(*A)).status, NavigationStatus.IDLE)

    def test_state_is_serializable(self):
        self.navigator.set_route([B, C])
        payload = self.navigator.update(position(*A, heading=10.0)).to_dict()
        self.assertEqual(payload["status"], "NAVIGATING")
        self.assertEqual(payload["waypoints_total"], 2)
        self.assertIn("lat", payload["target_waypoint"])
        # Navigation must not invent a distance unit it cannot supply.
        self.assertIsInstance(payload["distance_to_target_m"], float)


class TestRouteCorridor(unittest.TestCase):
    """Gate 3b defect 1: off-route is distance from the segment being driven
    (previous waypoint -> next), within `off_route_m` either side -- not distance
    to the next waypoint, which on a real OFFER leg (segments up to ~45 m) is far
    beyond 25 m while the robot is exactly on the route."""

    O = (12.971600, 77.594600)

    def setUp(self):
        self.W1 = offset(self.O, north_m=45.0)   # a 45 m segment, longer than the corridor
        self.W2 = offset(self.O, north_m=90.0)
        self.cfg = PlannerConfig(waypoint_arrival_m=8.0, off_route_m=25.0, reroute_after_n=8)

    def drive(self, planner, points):
        for point in points:
            planner.update_position(point)

    def test_segment_distance_is_cross_track_alongside_and_to_the_end_beyond(self):
        self.assertAlmostEqual(segment_distance_m(offset(self.O, north_m=20.0, east_m=10.0), self.O, self.W1), 10.0, delta=0.05)
        self.assertAlmostEqual(segment_distance_m(offset(self.O, north_m=20.0), self.O, self.W1), 0.0, delta=0.05)
        # Past the end: measured to the end, not to the infinite line.
        self.assertAlmostEqual(segment_distance_m(offset(self.O, north_m=75.0), self.O, self.W1), 30.0, delta=0.05)
        # Short of the start: measured to the start.
        self.assertAlmostEqual(segment_distance_m(offset(self.O, north_m=-12.0), self.O, self.W1), 12.0, delta=0.05)
        # A degenerate segment is a point.
        self.assertAlmostEqual(segment_distance_m(offset(self.O, east_m=7.0), self.O, self.O), 7.0, delta=0.05)

    def test_A_on_a_segment_longer_than_the_corridor_there_is_no_false_reroute(self):
        navigator = Navigator(NavigationConfig(waypoint_arrival_m=8.0, off_route_m=25.0, reroute_after_n=8))
        navigator.set_route([self.O, self.W1, self.W2])
        statuses, far_from_waypoint = [], 0
        for metres in range(0, 91):  # 1 m per tick, straight up the route
            state = navigator.update(position(*offset(self.O, north_m=float(metres)), heading=0.0))
            statuses.append(state.status)
            if state.distance_to_target_m is not None and state.distance_to_target_m > 25.0:
                far_from_waypoint += 1
        self.assertGreater(far_from_waypoint, 8, "the scenario must hold the robot >25 m from its next waypoint")
        self.assertNotIn(NavigationStatus.REROUTE_NEEDED, statuses)
        self.assertIs(statuses[-1], NavigationStatus.ARRIVED)

    def test_B_genuinely_sideways_of_the_segment_still_reroutes_on_repeated_evidence(self):
        planner = RoutePlanner(self.cfg)
        planner.set_route([self.O, self.W1, self.W2])
        planner.update_position(self.O)
        off = offset(self.O, north_m=20.0, east_m=30.0)
        self.drive(planner, [off] * 7)
        self.assertFalse(planner.should_reroute())   # 7 is not yet enough
        planner.update_position(off)
        self.assertTrue(planner.should_reroute())    # the 8th is

    def test_B_one_stray_fix_decays_away(self):
        planner = RoutePlanner(self.cfg)
        planner.set_route([self.O, self.W1, self.W2])
        planner.update_position(self.O)
        for _ in range(20):
            planner.update_position(offset(self.O, north_m=20.0, east_m=40.0))  # one stray fix
            planner.update_position(offset(self.O, north_m=20.0))                # then back on route
        self.assertFalse(planner.should_reroute())

    def test_boundary_around_the_corridor_half_width(self):
        inside = RoutePlanner(self.cfg)
        inside.set_route([self.O, self.W1, self.W2])
        inside.update_position(self.O)
        self.drive(inside, [offset(self.O, north_m=20.0, east_m=24.0)] * 50)
        self.assertFalse(inside.should_reroute(), "24 m from the segment is inside a 25 m corridor")

        outside = RoutePlanner(self.cfg)
        outside.set_route([self.O, self.W1, self.W2])
        outside.update_position(self.O)
        self.drive(outside, [offset(self.O, north_m=20.0, east_m=26.0)] * 8)
        self.assertTrue(outside.should_reroute(), "26 m from the segment is outside it")

    def test_C_overshooting_the_segment_end_is_off_route_evidence(self):
        planner = RoutePlanner(self.cfg)
        planner.set_route([self.O, self.W1, self.W2])
        planner.update_position(self.O)
        # Passes W1 10 m to the side (outside the 8 m arrival radius, so W1 is
        # never reached) and keeps going: 35 m past W1 it is ~36 m from the segment.
        self.drive(planner, [offset(self.O, north_m=float(n), east_m=10.0) for n in range(0, 81)])
        self.assertEqual(planner.waypoint_index(), 1)
        self.assertTrue(planner.should_reroute())

    def test_D_waypoints_still_advance_only_within_the_arrival_radius(self):
        planner = RoutePlanner(self.cfg)
        planner.set_route([self.O, self.W1, self.W2])
        planner.update_position(self.O)
        self.assertEqual(planner.waypoint_index(), 1)  # at waypoint 0 already
        planner.update_position(offset(self.O, north_m=36.0))
        self.assertEqual(planner.waypoint_index(), 1)  # 9 m short of W1
        planner.update_position(offset(self.O, north_m=38.0))
        self.assertEqual(planner.waypoint_index(), 2)  # 7 m: arrived, next is W2
        self.assertEqual(planner.active_segment(), (self.W1, self.W2))

    def test_waypoint_zero_is_reached_along_the_line_from_where_the_route_was_set(self):
        # A one-waypoint route 100 m away (the bench API's shape): driving straight
        # at it is on route the whole way; leaving that line is not.
        far = offset(self.O, north_m=100.0)
        straight = RoutePlanner(self.cfg)
        straight.set_route([far])
        self.drive(straight, [offset(self.O, north_m=float(n)) for n in range(0, 95)])
        self.assertFalse(straight.should_reroute())
        self.assertEqual(straight.active_segment(), (self.O, far))

        astray = RoutePlanner(self.cfg)
        astray.set_route([far])
        astray.update_position(self.O)
        self.drive(astray, [offset(self.O, north_m=30.0, east_m=30.0)] * 8)
        self.assertTrue(astray.should_reroute())

    def test_a_new_route_starts_a_new_segment_zero(self):
        planner = RoutePlanner(self.cfg)
        planner.set_route([self.W1])
        planner.update_position(self.O)
        planner.set_route([self.W2])
        self.assertIsNone(planner.active_segment())  # no position since the new route
        start = offset(self.O, north_m=50.0)
        planner.update_position(start)
        self.assertEqual(planner.active_segment(), (start, self.W2))


if __name__ == "__main__":
    unittest.main()
