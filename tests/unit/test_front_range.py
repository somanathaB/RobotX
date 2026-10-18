"""RX-07: the ESP32's front ultrasonic range in the Pi safety gate.

Three levels. `front_range` alone: what one TELEMETRY frame vouches for. The
gate with those readings. Then the whole agent driving a short leg over a fake
ESP32 (`Rig`), where what matters is which DRIVEs reach the wire.

Nothing here opens a serial port, starts a thread or touches hardware.
"""

import logging
import math
import unittest

from robotx.application.agent import RobotAgent
from robotx.config.settings import Settings
from robotx.control.motion import MotionIntent
from robotx.control.safety import RangeReading, RangeStatus, SafetyConfig, SafetyGate, SafetyVerdict
from robotx.esp32.front_range import ESP32_FRONT_STOP_CM, check_stop_distance, front_range
from robotx.esp32.state import ControllerTelemetry
from robotx.perception.types import FrameMetadata, PerceptionResult, PerceptionStatus
from robotx.state.robot_state import OperatingMode
from tests.fixtures import esp32 as fx
from tests.unit.test_esp32_agent import settings as agent_settings
from tests.unit.test_obstacle_recovery import HOLD_S, TICK_S, AgentCase, ticks


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


STALE_AFTER_S = 1.0
PI_STOP_CM = Settings().safety_stop_distance_cm


def tel(**fields):
    """A decoded TELEMETRY frame: the real capture, with a clear front, then `fields`."""

    return ControllerTelemetry.from_frame({**fx.REAL_TELEMETRY_FIELDS, **fx.CLEAR_FRONT, **fields},
                                          received_at=0.0)


def read(telemetry=None, age_s=0.1, link_up=True):
    return front_range(telemetry, age_s, link_up=link_up, stale_after_s=STALE_AFTER_S)


# --- one frame -> one reading ----------------------------------------------------


class TestFrontRangeReading(unittest.TestCase):
    def test_both_valid_and_clear_is_valid_at_the_nearer_sensor(self):
        r = read(tel(front_left_cm=150.0, front_right_cm=120.0))
        self.assertIs(r.status, RangeStatus.VALID)
        self.assertEqual(r.distance_cm, 120.0)
        self.assertFalse(r.obstacle_reported)

    def test_the_firmware_obstacle_latch_is_passed_on(self):
        r = read(tel(front_obstacle=True, forward_blocked=True, front_left_cm=80.0))
        self.assertTrue(r.obstacle_reported)
        self.assertIsNot(r.status, RangeStatus.VALID)

    def test_front_invalid_is_never_valid_but_keeps_a_near_number(self):
        r = read(tel(front_valid=False, forward_blocked=True, front_left_cm=None, front_right_cm=60.0))
        self.assertIs(r.status, RangeStatus.INVALID)
        self.assertEqual(r.distance_cm, 60.0)
        r = read(tel(front_valid=False, forward_blocked=True, front_left_cm=None, front_right_cm=None))
        self.assertIs(r.status, RangeStatus.INVALID)
        self.assertIsNone(r.distance_cm)

    def test_contradictions_are_invalid(self):
        for name, fields in {
            "valid with a null distance": dict(front_right_cm=None),
            "valid but forward blocked": dict(forward_blocked=True),
            "a non-finite distance": dict(front_left_cm=math.nan),
        }.items():
            with self.subTest(name):
                self.assertIs(read(tel(**fields)).status, RangeStatus.INVALID)

    def test_stale_or_unknown_carries_no_distance(self):
        stale = read(tel(), age_s=STALE_AFTER_S + 0.01)
        self.assertIs(stale.status, RangeStatus.STALE)
        self.assertIsNone(stale.distance_cm)
        self.assertIs(read(tel(), link_up=False).status, RangeStatus.STALE)
        self.assertIs(read(None).status, RangeStatus.UNKNOWN)
        self.assertIs(read(tel(), age_s=None).status, RangeStatus.UNKNOWN)


class TestStopDistanceMustBeBeyondTheEsp32Gate(unittest.TestCase):
    def test_at_or_inside_the_esp32_gate_is_refused(self):
        for cm in (ESP32_FRONT_STOP_CM, 30.0, 0.0, -1.0, math.nan, math.inf):
            with self.subTest(cm=cm), self.assertRaises(ValueError):
                check_stop_distance(cm)

    def test_beyond_it_is_accepted_and_the_default_is_beyond_it(self):
        check_stop_distance(ESP32_FRONT_STOP_CM + 0.1)
        check_stop_distance(PI_STOP_CM)
        self.assertGreater(PI_STOP_CM, ESP32_FRONT_STOP_CM)

    def test_the_agent_leaves_the_esp32_link_off_and_says_why(self):
        agent = RobotAgent(agent_settings(ROBOTX_ESP32_ENABLED="1", ROBOTX_SAFETY_STOP_DISTANCE_CM="40"))
        agent._start_esp32()
        self.assertIsNone(agent.esp32)
        self.assertIn("ROBOTX_SAFETY_STOP_DISTANCE_CM", agent._esp32_config_error)


# --- readings through the gate ---------------------------------------------------


def usable():
    return PerceptionResult(timestamp=1.0, status=PerceptionStatus.OK,
                            frame=FrameMetadata(width=640, height=480, source="test", age_s=0.0))


def evaluate(reading, *, range_expected=True, perception=None, intent=None):
    gate = SafetyGate(SafetyConfig(stop_distance_cm=PI_STOP_CM))
    return gate.evaluate(
        intent or MotionIntent.forward(0.45, reason="test"),
        mission_active=True,
        perception=perception or usable(),
        obstacle=reading,
        range_expected=range_expected,
        now=None,
    )


class TestGateWithFrontRange(unittest.TestCase):
    def test_a_valid_near_reading_stops_with_an_obstacle_reason(self):
        d = evaluate(read(tel(front_left_cm=PI_STOP_CM - 20, front_right_cm=150.0)))
        self.assertIs(d.verdict, SafetyVerdict.VETOED)
        self.assertEqual(d.rule, "obstacle")
        self.assertTrue(d.intent.is_stop)
        self.assertIn("obstacle", d.intent.reason)       # obstacle recovery keys on it

    def test_the_threshold_itself_stops(self):
        self.assertEqual(evaluate(read(tel(front_left_cm=PI_STOP_CM))).rule, "obstacle")

    def test_a_near_number_from_an_untrusted_sensor_still_stops(self):
        d = evaluate(read(tel(front_valid=False, forward_blocked=True, front_left_cm=55.0)))
        self.assertEqual(d.rule, "obstacle")

    def test_a_reported_obstacle_stops_whatever_the_distance(self):
        d = evaluate(read(tel(front_obstacle=True, forward_blocked=True,
                              front_left_cm=150.0, front_right_cm=150.0)))
        self.assertEqual(d.rule, "obstacle")

    def test_valid_and_clear_allows_motion_when_everything_else_passes(self):
        d = evaluate(read(tel(front_left_cm=PI_STOP_CM + 1, front_right_cm=200.0)))
        self.assertIs(d.verdict, SafetyVerdict.ALLOWED)
        self.assertFalse(d.intent.is_stop)

    def test_valid_and_clear_is_not_enough_on_its_own(self):
        clear = read(tel())
        blind = PerceptionResult.unavailable(PerceptionStatus.STALE)
        self.assertEqual(evaluate(clear, perception=blind).rule, "perception")
        self.assertEqual(evaluate(clear, intent=MotionIntent.turn_left(0.3)).rule, "negative_wheel")

    def test_invalid_stale_missing_or_unknown_never_allow_motion(self):
        cases = {
            "front invalid, far number": read(tel(front_valid=False, forward_blocked=True)),
            "front invalid, no number": read(tel(front_valid=False, forward_blocked=True,
                                                 front_left_cm=None, front_right_cm=None)),
            "stale": read(tel(), age_s=STALE_AFTER_S + 0.5),
            "link down": read(tel(), link_up=False),
            "no telemetry yet": read(None),
            "contradiction": read(tel(front_right_cm=None)),
            "non-finite": RangeReading(RangeStatus.VALID, distance_cm=math.nan),
            "no reading at all": None,
        }
        for name, reading in cases.items():
            with self.subTest(name):
                d = evaluate(reading)
                self.assertIs(d.verdict, SafetyVerdict.VETOED)
                self.assertTrue(d.intent.is_stop)

    def test_a_bench_pi_without_a_range_source_is_unchanged(self):
        # No ESP32 link: no reading, nothing expected -> the gate decides as before.
        self.assertIs(evaluate(None, range_expected=False).verdict, SafetyVerdict.ALLOWED)

    def test_a_stop_is_always_forwarded(self):
        d = evaluate(read(None), intent=MotionIntent.stop("hold"))
        self.assertIs(d.verdict, SafetyVerdict.ALLOWED)
        self.assertTrue(d.intent.is_stop)


# --- the agent over a fake ESP32 -------------------------------------------------


class TestAgentStopsBeforeTheEsp32Gate(AgentCase):
    """`Rig` drives a leg; `rig.fields` is what each fresh TELEMETRY reports."""

    def set_front(self, **fields):
        self.rig.fields.update(fields)

    def test_a_near_valid_reading_stops_the_pi_before_the_esp32_gate(self):
        rig = self.rig
        # Inside the Pi's limit, outside the ESP32's: the firmware would still
        # drive, so without the Pi's stop the rover would close to 40 cm.
        self.set_front(front_left_cm=70.0, front_right_cm=150.0, front_warning=True)
        rig.step()                                       # this TELEMETRY is read before the tick
        mark = rig.mark()
        snaps = rig.run(3.0)
        self.assertEqual(rig.drives(mark), [], "DRIVE toward a 70 cm obstacle")
        for s in snaps:
            self.assertIs(s.mode, OperatingMode.AUTO)    # an ordinary stop: no pause, no operator
            self.assertTrue(s.motion_intent.is_stop)
            self.assertEqual(s.safety.rule, "obstacle")
        self.assertEqual(rig.port.command_names()[-1], "STOP")

    def test_the_leg_resumes_only_after_the_clear_hold(self):
        rig = self.rig
        self.set_front(front_left_cm=60.0)
        rig.run(2.0)
        self.set_front(front_left_cm=150.0)
        mark = rig.mark()
        held = 0
        while not rig.drives(mark):
            rig.step()
            held += 1
            self.assertLess(held, ticks(HOLD_S) + 3, "the leg never resumed")
        self.assertGreaterEqual(held * TICK_S, HOLD_S - 1e-9)
        self.assertLessEqual(held * TICK_S, HOLD_S + TICK_S + 1e-9)
        self.assertIs(rig.step().mode, OperatingMode.AUTO)

    def test_a_range_that_is_not_valid_restarts_the_clear_hold(self):
        rig = self.rig
        self.set_front(front_left_cm=60.0)
        rig.run(1.0)
        self.set_front(front_left_cm=150.0)
        rig.run(HOLD_S - 2 * TICK_S)                     # most of the hold
        # Valid per the firmware flag but with no distance: not a clear path.
        # The link stays motion-ready (no controller fault), so only the gate
        # and the hold stand between this and a DRIVE.
        self.set_front(front_right_cm=None)
        mark = rig.mark()
        snaps = rig.run(1.0)
        self.assertEqual(rig.drives(mark), [])
        self.assertTrue(all(s.mode is OperatingMode.AUTO for s in snaps))
        self.set_front(front_right_cm=150.0)
        mark = rig.mark()
        rig.run(HOLD_S - 2 * TICK_S)                     # the hold starts over
        self.assertEqual(rig.drives(mark), [])
        rig.run(3 * TICK_S)
        self.assertTrue(rig.drives(mark), "the leg never resumed after a full clear hold")

    def test_a_reported_obstacle_is_never_driven_into(self):
        rig = self.rig
        # The firmware's own latch is on (it stopped for something) but no
        # safety_stop was latched because the Pi was not driving into it.
        self.set_front(front_obstacle=True, forward_blocked=True, front_left_cm=35.0)
        mark = rig.mark()
        snaps = rig.run(3.0)
        self.assertEqual(rig.drives(mark), [])
        self.assertTrue(all(s.motion_intent.is_stop for s in snaps))

    def test_a_latched_esp32_safety_stop_gets_no_drive_and_pauses_for_an_operator(self):
        rig = self.rig
        self.set_front(front_obstacle=True, forward_blocked=True, front_left_cm=30.0, safety_stop=True)
        mark = rig.mark()
        snaps = rig.run(3.0)
        self.assertEqual(rig.drives(mark), [], "DRIVE into a latched ESP32 obstacle fault")
        self.assertIs(snaps[-1].mode, OperatingMode.PAUSED)
        self.assertNotIn("RESET", rig.port.command_names(), "the Pi reset the ESP32 by itself")
        # The obstacle and the latch clear on the ESP32 side: nothing resumes.
        self.set_front(front_obstacle=False, forward_blocked=False, front_left_cm=150.0,
                       safety_stop=False)
        snaps = rig.run(3 * HOLD_S)
        self.assertTrue(all(s.mode is OperatingMode.PAUSED for s in snaps))
        self.assertEqual(rig.drives(mark), [])


if __name__ == "__main__":
    unittest.main()
