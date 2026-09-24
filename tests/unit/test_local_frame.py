"""Local metric frame and dead reckoning.

Two things are being protected here. The first is arithmetic: a projection that
is not exactly invertible would make a rover's own waypoints drift away from it
as it drove. The second, and more important, is labelling -- an inferred
position must be impossible to mistake for a measured one, at every layer that
handles it.
"""

from __future__ import annotations

import math
import unittest

from robotx.localization.local_frame import (
    DEFAULT_LOCAL_ORIGIN,
    DeadReckoner,
    DeadReckoningConfig,
    LocalFrame,
)
from robotx.localization.position import (
    HeadingSource,
    PositionSource,
    bearing_deg,
    haversine_m,
)


BANGALORE = (12.9716, 77.5946)


class LocalFrameTests(unittest.TestCase):
    def test_round_trip_is_exact_enough_to_navigate_on(self):
        frame = LocalFrame(origin=BANGALORE)
        for x, y in [(0, 0), (1, 0), (0, 1), (-5, 12), (100, -250), (-1000, 1000)]:
            with self.subTest(x=x, y=y):
                back_x, back_y = frame.to_local(frame.to_latlon(x, y))
                self.assertAlmostEqual(back_x, x, places=6)
                self.assertAlmostEqual(back_y, y, places=6)

    def test_origin_maps_to_zero(self):
        frame = LocalFrame(origin=BANGALORE)
        self.assertEqual(frame.to_latlon(0.0, 0.0), BANGALORE)
        x, y = frame.to_local(BANGALORE)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 0.0)

    def test_axes_point_the_way_they_claim(self):
        """+y must be north and +x must be east, or every heading is wrong."""

        frame = LocalFrame(origin=BANGALORE)

        north = frame.to_latlon(0.0, 100.0)
        self.assertGreater(north[0], BANGALORE[0])
        self.assertAlmostEqual(north[1], BANGALORE[1], places=9)
        self.assertAlmostEqual(bearing_deg(BANGALORE, north), 0.0, places=3)

        east = frame.to_latlon(100.0, 0.0)
        self.assertGreater(east[1], BANGALORE[1])
        self.assertAlmostEqual(bearing_deg(BANGALORE, east), 90.0, places=1)

    def test_local_distance_agrees_with_haversine(self):
        """The projection must not disagree with the metric navigation uses."""

        frame = LocalFrame(origin=BANGALORE)
        for x, y in [(10, 0), (0, 25), (30, 40), (-120, 90)]:
            with self.subTest(x=x, y=y):
                point = frame.to_latlon(x, y)
                expected = math.hypot(x, y)
                self.assertAlmostEqual(
                    haversine_m(BANGALORE, point), expected, delta=max(0.05, expected * 0.001)
                )

    def test_polar_origin_degrades_instead_of_dividing_by_zero(self):
        frame = LocalFrame(origin=(90.0, 0.0))
        lat, lon = frame.to_latlon(100.0, 0.0)
        self.assertTrue(math.isfinite(lat))
        self.assertTrue(math.isfinite(lon))


class DeadReckonerTests(unittest.TestCase):
    def config(self, **overrides):
        base = {"max_speed_mps": 1.0, "turn_rate_dps": 90.0, "max_step_s": 5.0}
        base.update(overrides)
        return DeadReckoningConfig(**base)

    def test_first_integrate_only_starts_the_clock(self):
        """With no previous timestamp there is no interval, so nothing moves."""

        dr = DeadReckoner(self.config())
        pose = dr.integrate(linear=1.0, angular=0.0, now=100.0)
        self.assertEqual((pose.x_m, pose.y_m), (0.0, 0.0))

    def test_driving_north_moves_along_positive_y(self):
        dr = DeadReckoner(self.config())
        dr.integrate(linear=1.0, angular=0.0, now=0.0)
        pose = dr.integrate(linear=1.0, angular=0.0, now=2.0)

        self.assertAlmostEqual(pose.y_m, 2.0, places=6)
        self.assertAlmostEqual(pose.x_m, 0.0, places=6)
        self.assertAlmostEqual(pose.distance_travelled_m, 2.0, places=6)

    def test_turning_right_then_driving_moves_east(self):
        dr = DeadReckoner(self.config())
        dr.integrate(linear=0.0, angular=0.0, now=0.0)
        # A full 1.0 angular for 1 s at 90 deg/s turns exactly 90 degrees right.
        dr.integrate(linear=0.0, angular=1.0, now=1.0)
        self.assertAlmostEqual(dr.pose.heading_deg, 90.0, places=6)

        pose = dr.integrate(linear=1.0, angular=0.0, now=2.0)
        self.assertAlmostEqual(pose.x_m, 1.0, places=6)
        self.assertAlmostEqual(pose.y_m, 0.0, places=6)

    def test_heading_wraps_rather_than_growing(self):
        dr = DeadReckoner(self.config())
        dr.integrate(linear=0.0, angular=0.0, now=0.0)
        for i in range(1, 9):
            dr.integrate(linear=0.0, angular=1.0, now=float(i))
        self.assertGreaterEqual(dr.pose.heading_deg, 0.0)
        self.assertLess(dr.pose.heading_deg, 360.0)

    def test_reversing_moves_backwards_but_still_accrues_uncertainty(self):
        """Distance travelled tracks path length, not displacement."""

        dr = DeadReckoner(self.config())
        dr.integrate(linear=0.0, angular=0.0, now=0.0)
        dr.integrate(linear=1.0, angular=0.0, now=1.0)
        pose = dr.integrate(linear=-1.0, angular=0.0, now=2.0)

        self.assertAlmostEqual(pose.y_m, 0.0, places=6)
        self.assertAlmostEqual(pose.distance_travelled_m, 2.0, places=6)

    def test_an_oversized_step_is_discarded_not_integrated(self):
        """A stalled loop must not be papered over with fabricated motion."""

        dr = DeadReckoner(self.config(max_step_s=0.5))
        dr.integrate(linear=1.0, angular=0.0, now=0.0)
        pose = dr.integrate(linear=1.0, angular=0.0, now=60.0)

        self.assertEqual(pose.y_m, 0.0, "a 60s gap must not become 60m of travel")

    def test_time_going_backwards_is_ignored(self):
        dr = DeadReckoner(self.config())
        dr.integrate(linear=1.0, angular=0.0, now=10.0)
        pose = dr.integrate(linear=1.0, angular=0.0, now=5.0)
        self.assertEqual((pose.x_m, pose.y_m), (0.0, 0.0))

    def test_reset_clears_pose_and_accumulated_drift(self):
        dr = DeadReckoner(self.config())
        dr.integrate(linear=1.0, angular=0.0, now=0.0)
        dr.integrate(linear=1.0, angular=0.0, now=3.0)
        self.assertNotEqual(dr.pose.y_m, 0.0)

        dr.reset(heading_deg=45.0)
        self.assertEqual(dr.pose.x_m, 0.0)
        self.assertEqual(dr.pose.y_m, 0.0)
        self.assertEqual(dr.pose.distance_travelled_m, 0.0)
        self.assertEqual(dr.pose.heading_deg, 45.0)

    def test_reset_also_restarts_the_clock(self):
        """Otherwise the first step after a reset integrates the idle gap."""

        dr = DeadReckoner(self.config(max_step_s=100.0))
        dr.integrate(linear=1.0, angular=0.0, now=0.0)
        dr.reset()
        pose = dr.integrate(linear=1.0, angular=0.0, now=50.0)
        self.assertEqual(pose.y_m, 0.0)


class LabellingTests(unittest.TestCase):
    """An inferred position must never be able to pass as a measured one."""

    def test_produced_position_is_labelled_dead_reckoning(self):
        dr = DeadReckoner(DeadReckoningConfig())
        position = dr.position(LocalFrame(origin=BANGALORE))

        self.assertIs(position.source, PositionSource.DEAD_RECKONING)
        self.assertIs(position.heading_source, HeadingSource.DEAD_RECKONED)
        self.assertFalse(position.is_measured)

    def test_no_fabricated_speed_or_satellites(self):
        dr = DeadReckoner(DeadReckoningConfig())
        position = dr.position(LocalFrame(origin=BANGALORE))

        self.assertIsNone(position.speed_mps, "commanded speed is not measured speed")
        self.assertIsNone(position.satellites, "no receiver means null, not zero")

    def test_source_reaches_the_serialized_form(self):
        dr = DeadReckoner(DeadReckoningConfig())
        payload = dr.position(LocalFrame(origin=BANGALORE)).to_dict()
        self.assertEqual(payload["source"], "DEAD_RECKONING")

    def test_a_gps_position_still_defaults_to_measured(self):
        from robotx.localization.position import Position

        position = Position(latitude=1.0, longitude=2.0, timestamp=0.0)
        self.assertIs(position.source, PositionSource.GPS)
        self.assertTrue(position.is_measured)

    def test_default_origin_is_obviously_synthetic(self):
        self.assertEqual(DEFAULT_LOCAL_ORIGIN, (0.0, 0.0))


class TelemetryRefusalTests(unittest.TestCase):
    """The backend must not receive an inferred coordinate by accident."""

    def snapshot_with_dead_reckoned_position(self):
        from robotx.application.agent import RobotAgent
        from robotx.config.settings import Settings

        agent = RobotAgent(
            Settings.from_env(
                {
                    "ROBOTX_CAMERA_ENABLED": "0",
                    "ROBOTX_PERCEPTION_ENABLED": "0",
                    "ROBOTX_GPS_ENABLED": "0",
                    "ROBOTX_DEADRECKON_ENABLED": "1",
                    "ROBOTX_LOG_LEVEL": "CRITICAL",
                }
            )
        )
        return agent.tick()

    def test_inferred_position_is_refused_by_default(self):
        from robotx.communication.protocol import build_telemetry_payload

        frame = build_telemetry_payload(
            self.snapshot_with_dead_reckoned_position(),
            sequence=1,
            max_position_age_s=30.0,
        )
        self.assertFalse(frame.has_position)
        self.assertNotIn("lat", frame.payload)
        self.assertIn("no usable GPS fix", frame.position_omitted or "")

    def test_there_is_no_way_to_publish_an_inferred_position(self):
        # The opt-in that used to exist is gone: a dead-reckoned pose is
        # integrated from commanded motion, not measured, and the wire has no
        # field that could say so.
        import inspect

        from robotx.communication.protocol import build_telemetry_payload

        self.assertNotIn(
            "allow_inferred_position",
            inspect.signature(build_telemetry_payload).parameters,
        )


class AgentIntegrationTests(unittest.TestCase):
    def agent(self, **overrides):
        from robotx.application.agent import RobotAgent
        from robotx.config.settings import Settings

        env = {
            "ROBOTX_CAMERA_ENABLED": "0",
            "ROBOTX_PERCEPTION_ENABLED": "0",
            "ROBOTX_GPS_ENABLED": "0",
            "ROBOTX_LOG_LEVEL": "CRITICAL",
        }
        env.update(overrides)
        return RobotAgent(Settings.from_env(env))

    def test_disabled_by_default_so_a_gps_rover_is_unaffected(self):
        agent = self.agent()
        self.assertIsNone(agent.dead_reckoner)
        self.assertIsNone(agent.tick().position)

    def test_enabled_gives_the_rover_a_position_without_gps(self):
        """The whole point: navigation stops reporting NO_POSITION."""

        from robotx.navigation.navigator import NavigationStatus

        agent = self.agent(ROBOTX_DEADRECKON_ENABLED="1")
        agent.start_mission([(0.0002, 0.0)])
        snapshot = agent.tick()

        self.assertIsNotNone(snapshot.position)
        self.assertIs(snapshot.position.source, PositionSource.DEAD_RECKONING)
        self.assertIsNot(snapshot.navigation.status, NavigationStatus.NO_POSITION)
        self.assertIsNotNone(snapshot.navigation.distance_to_target_m)

    def test_a_real_fix_takes_precedence_over_dead_reckoning(self):
        """Attaching a GPS must be all it takes; there is no mode to switch."""

        import time as _time

        from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus

        agent = self.agent(ROBOTX_DEADRECKON_ENABLED="1")
        fix = GpsFix(
            latitude=BANGALORE[0],
            longitude=BANGALORE[1],
            timestamp=_time.time(),
            satellites=9,
        )
        agent.gps = type(
            "FakeGps",
            (),
            {"get_reading": lambda self: GpsReading(status=GPSStatus.FIX, fix=fix)},
        )()

        snapshot = agent.tick()
        self.assertIs(snapshot.position.source, PositionSource.GPS)
        self.assertAlmostEqual(snapshot.position.latitude, BANGALORE[0])

    def test_a_vetoed_intent_does_not_move_the_estimate(self):
        """The reckoner is fed the gated intent, so a stopped rover stays put."""

        from robotx.control.motion import MotionIntent
        from robotx.state.robot_state import OperatingMode

        agent = self.agent(ROBOTX_DEADRECKON_ENABLED="1")
        agent.state.set_mode(OperatingMode.AUTO)
        # Perception is disabled, so the gate vetoes this every tick.
        agent.decision.decide = lambda **_: MotionIntent.forward(1.0, reason="go")

        for _ in range(5):
            agent.tick()

        self.assertEqual(agent.dead_reckoner.pose.distance_travelled_m, 0.0)

    def test_mission_start_resets_accumulated_drift(self):
        agent = self.agent(ROBOTX_DEADRECKON_ENABLED="1")
        agent.dead_reckoner.integrate(linear=1.0, angular=0.0, now=0.0)
        agent.dead_reckoner.integrate(linear=1.0, angular=0.0, now=0.4)
        self.assertGreater(agent.dead_reckoner.pose.distance_travelled_m, 0.0)

        agent.start_mission([(0.001, 0.001)])
        self.assertEqual(agent.dead_reckoner.pose.distance_travelled_m, 0.0)


if __name__ == "__main__":
    unittest.main()
