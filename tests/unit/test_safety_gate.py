"""Safety gate: the rules that decide whether a motion intent may be sent.

The point of most of these tests is the *negative* case -- that a condition the
robot cannot positively verify results in a stop. A gate that only passed its
happy-path tests would be worthless, because the happy path is the one case
where getting it wrong costs nothing.
"""

from __future__ import annotations

import time
import unittest

from robotx.control.motion import MotionCommand, MotionIntent
from robotx.control.safety import (
    UNEVALUATED,
    RangeReading,
    RangeStatus,
    SafetyConfig,
    SafetyGate,
    SafetyVerdict,
)
from robotx.perception.types import (
    Detection,
    FrameMetadata,
    PerceptionResult,
    PerceptionStatus,
)


def usable_perception() -> PerceptionResult:
    """A perception result the gate is allowed to act on."""

    return PerceptionResult(
        timestamp=time.time(),
        status=PerceptionStatus.OK,
        detections=(),
        frame=FrameMetadata(width=640, height=480, age_s=0.05),
        backend="test",
    )


def gate(**overrides) -> SafetyGate:
    return SafetyGate(SafetyConfig(**overrides))


class AllowedMotionTests(unittest.TestCase):
    """The gate must let a legitimate request through unchanged."""

    def test_clean_forward_intent_is_allowed_unchanged(self):
        intent = MotionIntent.forward(0.5, reason="on course")
        decision = gate().evaluate(
            intent, mission_active=True, perception=usable_perception()
        )

        self.assertIs(decision.verdict, SafetyVerdict.ALLOWED)
        self.assertFalse(decision.blocked)
        # Identity, not just equality: the intent must be passed through rather
        # than rebuilt, so nothing can be quietly altered on the way.
        self.assertIs(decision.intent, intent)

    def test_a_stop_is_always_forwarded(self):
        """A stop must reach the ESP32 even when every other rule would veto.

        Mission inactive *and* perception unusable -- both veto conditions --
        yet the stop still passes, because a stop the gate swallowed is a stop
        the motors never hear about.
        """

        intent = MotionIntent.stop("navigation gave up")
        decision = gate().evaluate(
            intent,
            mission_active=False,
            perception=PerceptionResult.unavailable(PerceptionStatus.DETECTOR_ERROR),
        )

        self.assertIs(decision.verdict, SafetyVerdict.ALLOWED)
        self.assertIs(decision.intent, intent)

    def test_hold_is_forwarded_like_a_stop(self):
        decision = gate().evaluate(
            MotionIntent.hold("idle"),
            mission_active=False,
            perception=PerceptionResult.unavailable(PerceptionStatus.DISABLED),
        )
        self.assertIs(decision.verdict, SafetyVerdict.ALLOWED)


class VetoTests(unittest.TestCase):
    def test_motion_without_an_active_mission_is_vetoed(self):
        decision = gate().evaluate(
            MotionIntent.forward(0.5),
            mission_active=False,
            perception=usable_perception(),
        )

        self.assertIs(decision.verdict, SafetyVerdict.VETOED)
        self.assertEqual(decision.rule, "mode")
        self.assertIs(decision.intent.command, MotionCommand.STOP)

    def test_stale_intent_is_vetoed(self):
        """An intent describing a world that has moved on must not be obeyed."""

        stale = MotionIntent(
            command=MotionCommand.FORWARD,
            left=0.4,
            right=0.4,
            reason="on course",
            timestamp=time.time() - 5.0,
        )
        decision = gate(max_intent_age_s=1.0).evaluate(
            stale, mission_active=True, perception=usable_perception()
        )

        self.assertIs(decision.verdict, SafetyVerdict.VETOED)
        self.assertEqual(decision.rule, "stale_intent")

    def test_fresh_intent_just_inside_the_age_limit_passes(self):
        fresh = MotionIntent(
            command=MotionCommand.FORWARD,
            left=0.4,
            right=0.4,
            timestamp=time.time() - 0.2,
        )
        decision = gate(max_intent_age_s=1.0).evaluate(
            fresh, mission_active=True, perception=usable_perception()
        )
        self.assertIs(decision.verdict, SafetyVerdict.ALLOWED)

    def test_unusable_perception_vetoes_motion(self):
        """Defence in depth: the decision layer already refuses this case.

        The gate must refuse it independently, so that a bug in decision policy
        cannot produce a moving intent that is then simply trusted.
        """

        for status in (
            PerceptionStatus.STALE,
            PerceptionStatus.NO_FRAME,
            PerceptionStatus.DETECTOR_ERROR,
            PerceptionStatus.DISABLED,
        ):
            with self.subTest(status=status):
                decision = gate().evaluate(
                    MotionIntent.forward(0.5),
                    mission_active=True,
                    perception=PerceptionResult.unavailable(status),
                )
                self.assertIs(decision.verdict, SafetyVerdict.VETOED)
                self.assertEqual(decision.rule, "perception")

    def test_ok_perception_without_frame_metadata_is_still_unusable(self):
        """`is_usable` requires frame metadata; the gate must not second-guess it."""

        no_frame = PerceptionResult(
            timestamp=time.time(),
            status=PerceptionStatus.OK,
            detections=(),
            frame=None,
            backend="test",
        )
        decision = gate().evaluate(
            MotionIntent.forward(0.5), mission_active=True, perception=no_frame
        )
        self.assertIs(decision.verdict, SafetyVerdict.VETOED)


class RangeSensorTests(unittest.TestCase):
    def test_measured_obstacle_inside_stop_distance_vetoes(self):
        decision = gate(stop_distance_cm=30.0).evaluate(
            MotionIntent.forward(0.5),
            mission_active=True,
            perception=usable_perception(),
            obstacle=RangeReading(RangeStatus.VALID, distance_cm=12.0, age_s=0.1),
        )

        self.assertIs(decision.verdict, SafetyVerdict.VETOED)
        self.assertEqual(decision.rule, "obstacle")
        # The agent keys route-blocking off this word.
        self.assertIn("obstacle", decision.intent.reason)

    def test_clear_measured_range_does_not_veto(self):
        decision = gate(stop_distance_cm=30.0).evaluate(
            MotionIntent.forward(0.5),
            mission_active=True,
            perception=usable_perception(),
            obstacle=RangeReading(RangeStatus.VALID, distance_cm=180.0, age_s=0.1),
        )
        self.assertIs(decision.verdict, SafetyVerdict.ALLOWED)

    def test_absent_sensor_does_not_veto_when_none_is_required(self):
        """Today's rover has no range sensor; the gate must not invent one."""

        decision = gate(require_range_sensor=False).evaluate(
            MotionIntent.forward(0.5),
            mission_active=True,
            perception=usable_perception(),
            obstacle=None,
        )
        self.assertIs(decision.verdict, SafetyVerdict.ALLOWED)

    def test_absent_sensor_vetoes_when_one_is_required(self):
        decision = gate(require_range_sensor=True).evaluate(
            MotionIntent.forward(0.5),
            mission_active=True,
            perception=usable_perception(),
            obstacle=None,
        )
        self.assertIs(decision.verdict, SafetyVerdict.VETOED)
        self.assertEqual(decision.rule, "range_missing")

    def test_silent_sensor_is_not_read_as_clear(self):
        """The bug this status model exists to prevent.

        TIMEOUT/STALE/ERROR/DISCONNECTED all mean "the sensor did not answer".
        With a sensor expected, every one of them must stop the robot -- none
        may be collapsed into "no obstacle".
        """

        for status in (
            RangeStatus.TIMEOUT,
            RangeStatus.STALE,
            RangeStatus.ERROR,
            RangeStatus.DISCONNECTED,
            RangeStatus.OUT_OF_RANGE,
            RangeStatus.UNKNOWN,
        ):
            with self.subTest(status=status):
                decision = gate(require_range_sensor=True).evaluate(
                    MotionIntent.forward(0.5),
                    mission_active=True,
                    perception=usable_perception(),
                    obstacle=RangeReading(status, distance_cm=None, age_s=0.1),
                )
                self.assertIs(decision.verdict, SafetyVerdict.VETOED)
                self.assertEqual(decision.rule, "range_unreliable")

    def test_valid_status_with_no_distance_is_treated_as_no_answer(self):
        decision = gate(require_range_sensor=True).evaluate(
            MotionIntent.forward(0.5),
            mission_active=True,
            perception=usable_perception(),
            obstacle=RangeReading(RangeStatus.VALID, distance_cm=None, age_s=0.1),
        )
        self.assertIs(decision.verdict, SafetyVerdict.VETOED)


class SpeedClampTests(unittest.TestCase):
    def test_overspeed_intent_is_clamped_not_vetoed(self):
        decision = gate(max_speed=0.5).evaluate(
            MotionIntent.forward(1.0, reason="flat out"),
            mission_active=True,
            perception=usable_perception(),
        )

        self.assertIs(decision.verdict, SafetyVerdict.CLAMPED)
        self.assertAlmostEqual(decision.intent.left, 0.5)
        self.assertAlmostEqual(decision.intent.right, 0.5)

    def test_clamping_preserves_the_turn_shape(self):
        """Scaling both sides keeps the arc; clipping each would tighten it."""

        intent = MotionIntent.forward(1.0, steer=0.5, reason="arc")
        ratio_before = intent.right / intent.left

        decision = gate(max_speed=0.5).evaluate(
            intent, mission_active=True, perception=usable_perception()
        )

        self.assertIs(decision.verdict, SafetyVerdict.CLAMPED)
        self.assertAlmostEqual(decision.intent.right / decision.intent.left, ratio_before)
        self.assertAlmostEqual(max(decision.intent.left, decision.intent.right), 0.5)

    def test_clamping_keeps_the_original_timestamp(self):
        """Staleness is measured from when the intent was decided, not clamped."""

        intent = MotionIntent(
            command=MotionCommand.FORWARD, left=1.0, right=1.0, timestamp=time.time() - 0.3
        )
        decision = gate(max_speed=0.5).evaluate(
            intent, mission_active=True, perception=usable_perception()
        )
        self.assertEqual(decision.intent.timestamp, intent.timestamp)

    def test_speed_at_the_ceiling_is_allowed_not_clamped(self):
        decision = gate(max_speed=0.5).evaluate(
            MotionIntent.forward(0.5),
            mission_active=True,
            perception=usable_perception(),
        )
        self.assertIs(decision.verdict, SafetyVerdict.ALLOWED)


class EmergencyStopTests(unittest.TestCase):
    def test_estop_vetoes_everything_and_outranks_every_other_rule(self):
        g = gate()
        g.engage_estop("bench test")

        decision = g.evaluate(
            MotionIntent.forward(0.3),
            mission_active=True,
            perception=usable_perception(),
        )

        self.assertIs(decision.verdict, SafetyVerdict.VETOED)
        self.assertEqual(decision.rule, "estop")
        self.assertIn("bench test", decision.reason)

    def test_estop_latches_across_ticks(self):
        """It must not clear itself just because conditions look fine again."""

        g = gate()
        g.engage_estop("obstacle")
        for _ in range(5):
            decision = g.evaluate(
                MotionIntent.forward(0.3),
                mission_active=True,
                perception=usable_perception(),
            )
            self.assertIs(decision.verdict, SafetyVerdict.VETOED)
        self.assertTrue(g.estop_engaged)

    def test_clearing_restores_motion_and_reports_whether_it_was_engaged(self):
        g = gate()
        g.engage_estop("test")

        self.assertTrue(g.clear_estop())
        self.assertFalse(g.estop_engaged)
        self.assertFalse(g.clear_estop(), "clearing an unengaged latch reports False")

        decision = g.evaluate(
            MotionIntent.forward(0.3),
            mission_active=True,
            perception=usable_perception(),
        )
        self.assertIs(decision.verdict, SafetyVerdict.ALLOWED)

    def test_a_stop_still_passes_while_estopped(self):
        """The motors must still be told to stop while the latch is shut."""

        g = gate()
        g.engage_estop("test")
        decision = g.evaluate(
            MotionIntent.stop("navigation stop"),
            mission_active=True,
            perception=usable_perception(),
        )
        self.assertIs(decision.verdict, SafetyVerdict.VETOED)
        self.assertIs(decision.intent.command, MotionCommand.STOP)


class InvariantTests(unittest.TestCase):
    """Properties that must hold no matter which rule fires."""

    def test_the_gate_never_increases_speed(self):
        g = gate(max_speed=0.6)
        perception = usable_perception()

        for speed in (0.0, 0.1, 0.45, 0.6, 0.9, 1.0):
            for active in (True, False):
                with self.subTest(speed=speed, active=active):
                    intent = MotionIntent.forward(speed)
                    out = g.evaluate(
                        intent, mission_active=active, perception=perception
                    ).intent
                    self.assertLessEqual(abs(out.left), abs(intent.left) + 1e-9)
                    self.assertLessEqual(abs(out.right), abs(intent.right) + 1e-9)

    def test_every_veto_produces_a_zero_velocity_intent(self):
        g = gate(require_range_sensor=True)
        vetoed = [
            g.evaluate(
                MotionIntent.forward(0.5), mission_active=False, perception=usable_perception()
            ),
            g.evaluate(
                MotionIntent.forward(0.5),
                mission_active=True,
                perception=PerceptionResult.unavailable(PerceptionStatus.STALE),
            ),
            g.evaluate(
                MotionIntent.forward(0.5),
                mission_active=True,
                perception=usable_perception(),
                obstacle=None,
            ),
        ]
        for decision in vetoed:
            with self.subTest(rule=decision.rule):
                self.assertIs(decision.verdict, SafetyVerdict.VETOED)
                self.assertEqual(decision.intent.left, 0.0)
                self.assertEqual(decision.intent.right, 0.0)
                self.assertTrue(decision.intent.is_stop)

    def test_unevaluated_default_is_closed(self):
        """Before the gate has run, the robot counts as not cleared."""

        self.assertIs(UNEVALUATED.verdict, SafetyVerdict.VETOED)
        self.assertTrue(UNEVALUATED.blocked)
        self.assertTrue(UNEVALUATED.intent.is_stop)

    def test_decision_serializes_for_telemetry(self):
        decision = gate().evaluate(
            MotionIntent.forward(0.5), mission_active=False, perception=usable_perception()
        )
        payload = decision.to_dict()
        self.assertEqual(payload["verdict"], "VETOED")
        self.assertEqual(payload["rule"], "mode")
        self.assertIn("mission", payload["reason"])


class AgentWiringTests(unittest.TestCase):
    """The gate must actually sit in the tick, not merely exist next to it.

    A correct gate that nothing calls is the failure mode worth testing for
    here, so these drive the real `RobotAgent.tick()` rather than the gate.
    """

    def agent(self, **overrides):
        from robotx.application.agent import RobotAgent
        from robotx.config.settings import Settings

        env = {
            "ROBOTX_ROBOT_ID": "safety-rover",
            "ROBOTX_CAMERA_ENABLED": "0",
            "ROBOTX_PERCEPTION_ENABLED": "0",
            "ROBOTX_GPS_ENABLED": "0",
            "ROBOTX_LOG_LEVEL": "CRITICAL",
        }
        env.update(overrides)
        return RobotAgent(Settings.from_env(env))

    def test_tick_records_a_safety_verdict(self):
        agent = self.agent()
        snapshot = agent.tick()

        self.assertIsNot(snapshot.safety, UNEVALUATED, "the gate did not run")
        self.assertTrue(snapshot.motion_intent.is_stop)

    def test_state_holds_the_gated_intent_not_the_proposed_one(self):
        """What telemetry and the ESP32 link read must be the safe intent.

        The agent is forced into AUTO with a decision layer that insists on
        driving; with perception disabled the gate must still be what lands in
        state.
        """

        from robotx.state.robot_state import OperatingMode

        agent = self.agent()
        agent.state.set_mode(OperatingMode.AUTO)
        agent.decision.decide = lambda **_: MotionIntent.forward(0.9, reason="go")

        snapshot = agent.tick()

        self.assertIs(snapshot.safety.verdict, SafetyVerdict.VETOED)
        self.assertTrue(snapshot.motion_intent.is_stop)
        self.assertEqual(snapshot.motion_intent.left, 0.0)

    def test_emergency_stop_survives_a_resume_attempt(self):
        """The latch must not be liftable as a side effect of a mode change."""

        from robotx.state.robot_state import MissionRefused, OperatingMode

        agent = self.agent()
        agent.start_mission([(12.9716, 77.5946)])
        agent.emergency_stop("bench test")

        self.assertTrue(agent.emergency_stopped)
        self.assertIs(agent.state.mode, OperatingMode.STOPPED)

        # Whatever the operator does to the mission, the gate stays shut.
        try:
            agent.resume_mission("try to resume")
        except MissionRefused:
            pass
        agent.state.set_mode(OperatingMode.AUTO)
        agent.decision.decide = lambda **_: MotionIntent.forward(0.5, reason="go")

        snapshot = agent.tick()
        self.assertEqual(snapshot.safety.rule, "estop")
        self.assertTrue(snapshot.motion_intent.is_stop)

    def test_clearing_the_estop_does_not_by_itself_start_motion(self):
        agent = self.agent()
        agent.emergency_stop("bench test")
        self.assertTrue(agent.clear_emergency_stop())

        snapshot = agent.tick()
        self.assertFalse(agent.emergency_stopped)
        # Cleared, but still not driving: no mission is active.
        self.assertTrue(snapshot.motion_intent.is_stop)
        self.assertEqual(snapshot.safety.rule, "stop")

    def test_safety_verdict_reaches_telemetry(self):
        from robotx.state.telemetry import build_telemetry

        agent = self.agent()
        payload = build_telemetry(agent.tick())

        self.assertIn("safety", payload)
        self.assertIn(payload["safety"]["verdict"], {"ALLOWED", "CLAMPED", "VETOED"})

    def test_safety_module_does_not_import_gpio(self):
        """The gate reasons about range readings without reaching a pin.

        `robotx.control.safety` defines its own range vocabulary precisely so
        it does not have to import the HC-SR04 driver; importing it must not
        drag GPIO into the agent's import graph.
        """

        import subprocess
        import sys

        result = subprocess.run(
            [
                sys.executable,
                "-c",
                "import robotx.control.safety, sys; "
                "print('RPi.GPIO' in sys.modules or "
                "'robotx.hardware.ultrasonic' in sys.modules)",
            ],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.stdout.strip(), "False", result.stderr)


if __name__ == "__main__":
    unittest.main()
