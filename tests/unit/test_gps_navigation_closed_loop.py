"""GPS navigation, closed through a SIMULATED differential-drive rover.

Each scenario runs the Pi's real localization, navigation, mission, decision,
safety and progress code against `tests.fixtures.diff_drive_sim`: wheel outputs
turn the rover, its true yaw is hidden, and the only heading the Pi gets is the
GPS's. Simulated physics and GPS only -- none of this is evidence that the
physical rover moves or steers. See docs/operations/GPS_NAVIGATION_TEST.md.
"""

import logging
import math
import random
import time
import unittest

from robotx.localization.esp32_gps import reading_from_frame
from robotx.localization.position import HeadingSource, Position, PositionConfig, PositionEstimator
from robotx.mission.mission import Mission, MissionStatus
from robotx.navigation.navigator import NavigationConfig, NavigationStatus, Navigator
from tests.fixtures.diff_drive_sim import ClosedLoopNav, GpsModel, Physics, Window, offset, settings, straight_route

START = (12.9700000, 77.5900000)
ARRIVAL_M = 8.0


def route(bearing, length_m=30.0):
    return straight_route(START, bearing=bearing, length_m=length_m)


def first_turn(result):
    """(left, right) DRIVE units of the first steered tick."""

    return next((r.left, r.right) for r in result.rows if r.left != r.right)


class SimTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        logging.disable(logging.CRITICAL)

    @classmethod
    def tearDownClass(cls):
        logging.disable(logging.NOTSET)

    def assertArrivedSafely(self, result, max_rotations=1.5):
        self.assertIn(result.outcome, ("ARRIVED", "COMPLETE"), result.summary())
        self.assertLessEqual(result.final_distance_m, ARRIVAL_M + 1.0)
        self.assertEqual(result.reverse_or_pivot_drives, 0, "only forward DRIVEs, never a reverse or pivot")
        self.assertEqual(result.sign_wrong, 0, "every turn toward the side the Pi measured the target on")
        # The defect this guards: the old steering law circled 4-34 times on a
        # held heading and never arrived.
        self.assertLess(result.total_turn_deg / 360.0, max_rotations, result.summary())


class TestDistanceAndBearing(unittest.TestCase):
    """The navigator's distance and desired heading, on exact fixes."""

    def test_each_compass_direction(self):
        for bearing in (0.0, 45.0, 90.0, 180.0, 270.0):
            with self.subTest(bearing=bearing):
                target = offset(START, north_m=30.0 * math.cos(math.radians(bearing)),
                                east_m=30.0 * math.sin(math.radians(bearing)))
                nav = Navigator(NavigationConfig())
                nav.set_route([target])
                state = nav.update(Position(latitude=START[0], longitude=START[1], timestamp=1.0,
                                            heading_deg=0.0, h_acc_m=1.0))
                self.assertAlmostEqual(state.distance_to_target_m, 30.0, delta=0.1)
                self.assertAlmostEqual(state.desired_heading_deg, bearing, delta=0.1)
                expected_error = (bearing + 180.0) % 360.0 - 180.0
                self.assertAlmostEqual(state.heading_error_deg, expected_error, delta=0.1)


class TestClosedLoopNavigation(SimTestCase):
    def test_aligned_start_reaches_the_destination(self):
        result = ClosedLoopNav(START, 0.0, route=route(0.0)).run()
        self.assertArrivedSafely(result)

    def test_destination_to_the_right_turns_right_first(self):
        result = ClosedLoopNav(START, 0.0, route=route(60.0)).run()
        left, right = first_turn(result)
        self.assertGreater(left, right, "a target to the right slows the right side")
        self.assertArrivedSafely(result)

    def test_destination_to_the_left_turns_left_first(self):
        result = ClosedLoopNav(START, 0.0, route=route(300.0)).run()
        left, right = first_turn(result)
        self.assertLess(left, right, "a target to the left slows the left side")
        self.assertArrivedSafely(result)

    def test_converges_across_seeds_and_drivetrains_within_45_degrees(self):
        for physics in (Physics(), Physics(full_scale_mps=1.4), Physics(track_m=0.6)):
            for yaw, seed in ((-40.0, 1), (0.0, 2), (40.0, 3)):
                with self.subTest(physics=physics, yaw=yaw, seed=seed):
                    sim = ClosedLoopNav(START, 20.0 + yaw, route=route(20.0, 40.0), physics=physics, seed=seed)
                    self.assertArrivedSafely(sim.run(), max_rotations=2.5)

    def test_without_a_heading_it_creeps_straight_first(self):
        result = ClosedLoopNav(START, 0.0, route=route(60.0)).run()
        opening = result.rows[:20]
        self.assertTrue(all(r.heading_source == "NONE" for r in opening))
        self.assertTrue(all(r.left == r.right == 64 for r in opening), "slow_speed 0.25 -> 64 units, no steer")

    def test_waypoints_are_consumed_in_order(self):
        corner = offset(START, north_m=25.0)
        legs = [offset(START, north_m=12.0), corner, offset(corner, east_m=12.0), offset(corner, east_m=25.0)]
        result = ClosedLoopNav(START, 0.0, route=legs).run()
        indices = [r.waypoint_index for r in result.rows]
        self.assertEqual(indices, sorted(indices), "waypoints only ever advance")
        self.assertEqual(sorted(set(indices)), [0, 1, 2, 3])
        self.assertArrivedSafely(result, max_rotations=2.0)

    def test_a_two_stop_task_completes_through_the_mission_manager(self):
        pickup = offset(START, north_m=24.0)
        drop = offset(pickup, east_m=24.0)
        mission = Mission.create(
            task_id="SIM-NAV-1", pickup=pickup, drop=drop,
            path_to_pickup=[offset(START, north_m=12.0), pickup],
            path_to_drop=[offset(pickup, east_m=12.0), drop], timestamp=time.time())
        sim = ClosedLoopNav(START, 0.0, mission=mission)
        result = sim.run(900)
        self.assertEqual(result.outcome, "COMPLETE", result.summary())
        self.assertIs(sim.missions.active.status, MissionStatus.COMPLETE)
        self.assertEqual(result.reverse_or_pivot_drives, 0)

    def test_runs_are_deterministic_per_seed(self):
        a = ClosedLoopNav(START, 30.0, route=route(0.0), seed=7).run().summary()
        b = ClosedLoopNav(START, 30.0, route=route(0.0), seed=7).run().summary()
        self.assertEqual(a, b)


class TestGpsFaultsWhileNavigating(SimTestCase):
    def test_a_short_outage_stops_the_rover_then_it_carries_on(self):
        sim = ClosedLoopNav(START, 0.0, route=route(0.0, 40.0))
        sim.gps_outages.append(Window(30.0, 50.0))
        result = sim.run()
        # The last frame goes stale after gps_stale_after_s (5 s).
        held = [r for r in result.rows if 37.0 <= r.t < 50.0]
        self.assertTrue(held and all(r.nav == "NO_POSITION" and r.left == r.right == 0 for r in held))
        self.assertTrue(all(r.reason == "no GPS position" for r in held))
        self.assertArrivedSafely(result)

    def test_a_long_outage_pauses_for_an_operator(self):
        sim = ClosedLoopNav(START, 0.0, route=route(0.0, 40.0))
        sim.gps_outages.append(Window(30.0, 200.0))
        result = sim.run()
        self.assertEqual(result.outcome, "PAUSED")
        self.assertIn("no usable GPS position", result.reason)
        self.assertLess(result.t, 30.0 + 5.0 + 30.0 + 2.0)

    def test_an_inaccurate_fix_stops_the_rover(self):
        sim = ClosedLoopNav(START, 0.0, route=route(0.0, 40.0))
        sim.gps_bad_acc.append(Window(20.0, 35.0, 6.0))
        result = sim.run()
        held = [r for r in result.rows if 21.5 <= r.t < 35.0]
        self.assertTrue(held and all(r.left == r.right == 0 for r in held))
        self.assertTrue(all(r.reason == "GPS fix not accurate enough to navigate" for r in held))
        self.assertArrivedSafely(result)

    def test_stationary_course_over_ground_is_never_a_heading(self):
        rng = random.Random(3)
        estimator = PositionEstimator(PositionConfig())
        wall = 1_800_000_000.0
        for i in range(300):
            frame = {"gps_status": "OK", "fix_ok": True, "fix_type": 3, "siv": 11, "age_ms": 60,
                     "lat_e7": 129700000 + rng.randint(-20, 20), "lon_e7": 775900000 + rng.randint(-20, 20),
                     "hacc_mm": 1980, "speed_mm_s": rng.randint(0, 60),
                     "head_mot_e5": rng.randint(0, 35_999_999)}
            reading = reading_from_frame(frame, wall + i, now=wall + i + 0.1, stale_after_s=5.0)
            position = estimator.update(reading, driven=False)
            self.assertIsNone(position.heading_deg)
            self.assertIs(position.heading_source, HeadingSource.NONE)


class TestStopsWhileNavigating(SimTestCase):
    def test_a_range_veto_stops_every_drive_and_the_route_resumes(self):
        sim = ClosedLoopNav(START, 0.0, route=route(0.0, 40.0))
        sim.obstacles.append(Window(25.0, 40.0, 50.0))             # 50 cm, inside the 90 cm stop distance
        result = sim.run()
        held = [r for r in result.rows if 25.0 <= r.t < 40.0]
        self.assertTrue(held and all(r.left == r.right == 0 for r in held))
        self.assertArrivedSafely(result)

    def test_an_operator_stop_ends_all_motion(self):
        sim = ClosedLoopNav(START, 0.0, route=route(0.0, 40.0))
        sim.stop_at = 30.0
        result = sim.run(until_paused=False, max_s=60.0)
        self.assertEqual(result.outcome, "STOPPED")
        after = [r for r in result.rows if r.t >= 30.0]
        self.assertTrue(after and all(r.left == r.right == 0 for r in after))
        self.assertIs(sim.navigator.state.status, NavigationStatus.IDLE)


class TestTurnRateCalibration(SimTestCase):
    """Why the assumed yaw rate must be measured: what each error does."""

    def test_an_overestimated_rate_still_converges(self):
        sim = ClosedLoopNav(START, 0.0, route=route(60.0), cfg=settings(ROBOTX_NAV_TURN_RATE_DPS="180"))
        self.assertArrivedSafely(sim.run(), max_rotations=2.0)

    def test_a_rate_far_too_low_overshoots_but_is_bounded_by_the_progress_pause(self):
        # True rate ~120 deg/s, assumed 30: every turn overshoots. The rover
        # does not arrive -- and does not run away: it pauses for an operator.
        sim = ClosedLoopNav(START, 0.0, route=route(90.0), physics=Physics(full_scale_mps=1.4),
                            cfg=settings(ROBOTX_NAV_TURN_RATE_DPS="30"))
        result = sim.run()
        self.assertEqual(result.outcome, "PAUSED", result.summary())
        self.assertIn("no progress", result.reason)
        self.assertEqual(result.reverse_or_pivot_drives, 0)

    def test_poor_gps_accuracy_lengthens_every_heading_measurement(self):
        good = ClosedLoopNav(START, 0.0, route=route(0.0, 40.0)).run()
        poor = ClosedLoopNav(START, 0.0, route=route(0.0, 40.0), gps=GpsModel(h_acc_m=4.5, sigma_m=1.5)).run()
        first = lambda r: next(row.t for row in r.rows if row.heading_source == "GPS_TRACK")
        self.assertGreater(first(poor), 2.0 * first(good), "a track must exceed 2 x h_acc before it is a heading")


if __name__ == "__main__":
    unittest.main()
