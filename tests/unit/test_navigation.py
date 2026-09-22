"""Navigation: route progress, rerouting, and desired-heading computation."""

import time
import unittest

from robotx.navigation.navigator import (
    NavigationConfig,
    NavigationStatus,
    Navigator,
)
from robotx.navigation.route_planner import PlannerConfig, RoutePlanner
from robotx.localization.position import HeadingSource, Position


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
        planner.update_position((51.6, -0.1))
        self.assertFalse(planner.should_reroute())  # one bad fix is not a reroute
        planner.update_position((51.6, -0.1))
        planner.update_position((51.6, -0.1))
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


if __name__ == "__main__":
    unittest.main()
