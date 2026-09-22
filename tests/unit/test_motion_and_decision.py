"""Motion intent and the decision layer, including its fail-safe behaviour."""

import time
import unittest

from robotx.control.decision import DecisionConfig, DecisionMaker, center_zone
from robotx.control.motion import MotionCommand, MotionIntent, age_s
from robotx.navigation.navigator import NavigationState, NavigationStatus
from robotx.perception.types import (
    Detection,
    FrameMetadata,
    PerceptionResult,
    PerceptionStatus,
)


FRAME_W, FRAME_H = 640, 480


def detection(label="obstacle", area=5000, cx=320, conf=0.7):
    half = int((area**0.5) / 2)
    bbox = (cx - half, 200, cx + half, 200 + 2 * half)
    return Detection(label=label, confidence=conf, bbox=bbox, area_px=area)


def perception(detections=(), status=PerceptionStatus.OK, delta=0):
    return PerceptionResult(
        timestamp=time.time(),
        status=status,
        detections=tuple(detections),
        frame=FrameMetadata(width=FRAME_W, height=FRAME_H, age_s=0.05),
        backend="test",
        largest_area_delta_px=delta,
    )


def navigating(heading_error=0.0):
    return NavigationState(
        status=NavigationStatus.NAVIGATING,
        target_waypoint=(51.5, -0.1),
        destination=(51.5, -0.1),
        distance_to_target_m=50.0,
        desired_heading_deg=0.0,
        current_heading_deg=-heading_error,
        heading_error_deg=heading_error,
        waypoints_total=2,
    )


class TestMotionIntent(unittest.TestCase):
    def test_stop_is_zero_velocity(self):
        intent = MotionIntent.stop("test")
        self.assertIs(intent.command, MotionCommand.STOP)
        self.assertEqual((intent.left, intent.right), (0.0, 0.0))
        self.assertTrue(intent.is_stop)

    def test_hold_counts_as_stopped(self):
        self.assertTrue(MotionIntent.hold("idle").is_stop)

    def test_forward_is_symmetric_without_steering(self):
        intent = MotionIntent.forward(0.5)
        self.assertEqual(intent.left, 0.5)
        self.assertEqual(intent.right, 0.5)
        self.assertAlmostEqual(intent.linear, 0.5)
        self.assertAlmostEqual(intent.angular, 0.0)

    def test_steering_right_slows_the_right_side(self):
        intent = MotionIntent.forward(0.6, steer=0.5)
        self.assertGreater(intent.left, intent.right)
        self.assertGreater(intent.angular, 0.0)
        # Steering never exceeds the requested speed.
        self.assertLessEqual(max(intent.left, intent.right), 0.6)

    def test_steering_left_slows_the_left_side(self):
        intent = MotionIntent.forward(0.6, steer=-0.5)
        self.assertLess(intent.left, intent.right)
        self.assertLess(intent.angular, 0.0)

    def test_turns_are_opposed(self):
        left = MotionIntent.turn_left(0.4)
        self.assertEqual((left.left, left.right), (-0.4, 0.4))
        right = MotionIntent.turn_right(0.4)
        self.assertEqual((right.left, right.right), (0.4, -0.4))

    def test_reverse_is_negative(self):
        intent = MotionIntent.reverse(0.3)
        self.assertLess(intent.linear, 0.0)

    def test_values_are_clamped_to_normalized_range(self):
        intent = MotionIntent(command=MotionCommand.FORWARD, left=5.0, right=-9.0)
        self.assertEqual(intent.left, 1.0)
        self.assertEqual(intent.right, -1.0)

    def test_speed_arguments_are_clamped(self):
        self.assertEqual(MotionIntent.forward(4.0).left, 1.0)
        self.assertEqual(MotionIntent.forward(-1.0).left, 0.0)

    def test_round_trips_through_a_dict(self):
        original = MotionIntent.forward(0.45, steer=0.2, reason="following route")
        restored = MotionIntent.from_dict(original.to_dict())
        self.assertEqual(restored.command, original.command)
        self.assertAlmostEqual(restored.left, original.left, places=3)
        self.assertAlmostEqual(restored.right, original.right, places=3)
        self.assertEqual(restored.reason, original.reason)

    def test_dict_carries_no_hardware_units(self):
        # The Pi speaks in normalized velocity, never PWM/duty.
        payload = MotionIntent.forward(0.5).to_dict()
        for forbidden in ("pwm", "duty", "rpm", "gpio"):
            self.assertNotIn(forbidden, str(payload).lower())

    def test_from_dict_tolerates_garbage(self):
        intent = MotionIntent.from_dict({"command": "NONSENSE"})
        self.assertIs(intent.command, MotionCommand.STOP)

    def test_age_increases(self):
        intent = MotionIntent.stop()
        self.assertGreaterEqual(age_s(intent, now=intent.timestamp + 2.0), 2.0)


class TestCenterZone(unittest.TestCase):
    def test_zone_is_centered(self):
        left, right = center_zone(640, 0.33)
        self.assertLess(left, 320)
        self.assertGreater(right, 320)
        self.assertAlmostEqual((left + right) / 2, 320, delta=1)

    def test_zone_scales_with_frame_width(self):
        narrow = center_zone(320, 0.33)
        wide = center_zone(1280, 0.33)
        self.assertLess(narrow[1] - narrow[0], wide[1] - wide[0])


class TestDecisionMaker(unittest.TestCase):
    def setUp(self):
        self.decider = DecisionMaker(
            DecisionConfig(
                cruise_speed=0.45,
                turn_speed=0.35,
                slow_speed=0.25,
                stop_area_px=30000,
                slow_area_px=10000,
                min_area_px=1500,
                center_zone_ratio=0.33,
            )
        )

    def decide(self, **kwargs):
        params = {
            "mission_active": True,
            "navigation": navigating(),
            "perception": perception(),
        }
        params.update(kwargs)
        return self.decider.decide(**params)

    # --- fail-safe paths -----------------------------------------------------

    def test_no_mission_holds(self):
        intent = self.decide(mission_active=False)
        self.assertIs(intent.command, MotionCommand.HOLD)

    def test_unusable_perception_stops(self):
        # Not seeing is never the same as seeing a clear path.
        for status in (
            PerceptionStatus.NO_FRAME,
            PerceptionStatus.DETECTOR_ERROR,
            PerceptionStatus.STALE,
            PerceptionStatus.DISABLED,
        ):
            intent = self.decide(perception=PerceptionResult.unavailable(status))
            self.assertIs(intent.command, MotionCommand.STOP, status)
            self.assertIn("perception unavailable", intent.reason)

    def test_person_stops_regardless_of_size(self):
        intent = self.decide(perception=perception([detection(label="person", area=2000)]))
        self.assertIs(intent.command, MotionCommand.STOP)
        self.assertIn("person", intent.reason)

    def test_person_off_to_the_side_still_stops(self):
        intent = self.decide(
            perception=perception([detection(label="person", area=2000, cx=20)])
        )
        self.assertIs(intent.command, MotionCommand.STOP)

    def test_no_position_stops(self):
        intent = self.decide(
            navigation=NavigationState(status=NavigationStatus.NO_POSITION)
        )
        self.assertIs(intent.command, MotionCommand.STOP)
        self.assertIn("GPS", intent.reason)

    def test_no_route_holds(self):
        intent = self.decide(navigation=NavigationState(status=NavigationStatus.IDLE))
        self.assertIs(intent.command, MotionCommand.HOLD)

    def test_arrival_holds(self):
        intent = self.decide(navigation=NavigationState(status=NavigationStatus.ARRIVED))
        self.assertIs(intent.command, MotionCommand.HOLD)
        self.assertIn("destination", intent.reason)

    def test_reroute_needed_stops(self):
        intent = self.decide(
            navigation=NavigationState(status=NavigationStatus.REROUTE_NEEDED)
        )
        self.assertIs(intent.command, MotionCommand.STOP)

    # --- obstacle handling ---------------------------------------------------

    def test_large_obstacle_ahead_stops(self):
        intent = self.decide(perception=perception([detection(area=40000, cx=320)]))
        self.assertIs(intent.command, MotionCommand.STOP)
        self.assertIn("obstacle ahead", intent.reason)

    def test_obstacle_left_of_center_turns_right(self):
        left, _ = center_zone(FRAME_W, 0.33)
        intent = self.decide(perception=perception([detection(area=5000, cx=left + 5)]))
        self.assertIs(intent.command, MotionCommand.TURN_RIGHT)

    def test_obstacle_right_of_center_turns_left(self):
        _, right = center_zone(FRAME_W, 0.33)
        intent = self.decide(perception=perception([detection(area=5000, cx=right - 5)]))
        self.assertIs(intent.command, MotionCommand.TURN_LEFT)

    def test_obstacle_outside_the_corridor_does_not_trigger_avoidance(self):
        intent = self.decide(perception=perception([detection(area=5000, cx=20)]))
        self.assertIs(intent.command, MotionCommand.FORWARD)

    def test_tiny_detection_is_ignored(self):
        intent = self.decide(perception=perception([detection(area=500, cx=320)]))
        self.assertIs(intent.command, MotionCommand.FORWARD)

    def test_rapidly_growing_obstacle_stops(self):
        intent = self.decide(
            perception=perception([detection(area=5000, cx=320)], delta=20000)
        )
        self.assertIs(intent.command, MotionCommand.STOP)
        self.assertIn("growing", intent.reason)

    def test_missing_frame_metadata_stops_rather_than_driving_on(self):
        """Regression: an OK result with no frame once produced FORWARD.

        The corridor check needs the frame width, and silently no-opped when
        frame metadata was absent -- so a 40000px obstacle dead centre fell
        through to route-following and was driven into.
        """

        result = PerceptionResult(
            timestamp=time.time(),
            status=PerceptionStatus.OK,
            detections=(detection(area=40000, cx=320),),
            frame=None,
            backend="test",
        )
        self.assertFalse(result.is_usable, "no frame => not usable for decisions")

        intent = self.decide(perception=result)
        self.assertTrue(intent.is_stop, f"expected a stop, got {intent.command}")
        self.assertIn("no frame metadata", intent.reason)

    def test_missing_frame_metadata_stops_even_with_no_detections(self):
        # Not about the obstacle: without frame geometry the result is unusable
        # regardless of what it contains.
        result = PerceptionResult(
            timestamp=time.time(),
            status=PerceptionStatus.OK,
            detections=(),
            frame=None,
            backend="test",
        )
        self.assertTrue(self.decide(perception=result).is_stop)

    # --- route following -----------------------------------------------------

    def test_clear_path_drives_forward_at_cruise(self):
        intent = self.decide()
        self.assertIs(intent.command, MotionCommand.FORWARD)
        self.assertAlmostEqual(intent.left, 0.45)
        self.assertAlmostEqual(intent.right, 0.45)

    def test_small_heading_error_is_ignored(self):
        intent = self.decide(navigation=navigating(heading_error=2.0))
        self.assertAlmostEqual(intent.left, intent.right)

    def test_moderate_heading_error_steers(self):
        right_turn = self.decide(navigation=navigating(heading_error=30.0))
        self.assertIs(right_turn.command, MotionCommand.FORWARD)
        self.assertGreater(right_turn.left, right_turn.right)

        left_turn = self.decide(navigation=navigating(heading_error=-30.0))
        self.assertLess(left_turn.left, left_turn.right)

    def test_large_heading_error_rotates_in_place(self):
        intent = self.decide(navigation=navigating(heading_error=150.0))
        self.assertIs(intent.command, MotionCommand.TURN_RIGHT)

        intent = self.decide(navigation=navigating(heading_error=-150.0))
        self.assertIs(intent.command, MotionCommand.TURN_LEFT)

    def test_missing_heading_creeps_instead_of_cruising(self):
        state = NavigationState(
            status=NavigationStatus.NAVIGATING,
            target_waypoint=(51.5, -0.1),
            desired_heading_deg=0.0,
            current_heading_deg=None,
            heading_error_deg=None,
        )
        intent = self.decide(navigation=state)
        self.assertIs(intent.command, MotionCommand.FORWARD)
        self.assertAlmostEqual(intent.left, 0.25)
        self.assertIn("no heading", intent.reason)

    def test_nearby_off_corridor_object_slows_travel(self):
        intent = self.decide(perception=perception([detection(area=15000, cx=30)]))
        self.assertIs(intent.command, MotionCommand.FORWARD)
        self.assertAlmostEqual(intent.left, 0.25)
        self.assertIn("cautious", intent.reason)

    def test_every_intent_has_a_reason(self):
        for kwargs in (
            {"mission_active": False},
            {"perception": PerceptionResult.unavailable(PerceptionStatus.NO_FRAME)},
            {"perception": perception([detection(label="person")])},
            {"perception": perception([detection(area=40000)])},
            {},
        ):
            self.assertTrue(self.decide(**kwargs).reason, kwargs)


if __name__ == "__main__":
    unittest.main()
