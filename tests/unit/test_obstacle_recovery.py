"""P4: obstacle recovery (RX-08), no pivots (RX-09), and a recovered link (P3).

Two levels. `ObstacleRecovery` alone, under a fake clock. Then the whole agent
driving a short route over a fake ESP32 link (`Rig`), where what matters is
what reaches the wire: every DRIVE the link wrote, and when.

Nothing here opens a serial port, starts a thread or touches hardware.
"""

import logging
import time
import unittest

from robotx.application.agent import RobotAgent
from robotx.control.decision import DecisionConfig, DecisionMaker, center_zone
from robotx.control.motion import MotionCommand, MotionIntent
from robotx.control.obstacle_recovery import ObstacleRecovery, ObstacleRecoveryConfig
from robotx.control.safety import SafetyConfig, SafetyGate, SafetyVerdict
from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus
from robotx.navigation.navigator import NavigationState, NavigationStatus
from robotx.perception.types import (
    Detection,
    FrameMetadata,
    PerceptionResult,
    PerceptionStatus,
)
from robotx.state.robot_state import Esp32LinkStatus as S, OperatingMode
from tests.fixtures import esp32 as fx
from tests.unit.test_agent import HERE, NORTH, FakeGPS, FakePerception, headless_settings


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


TICK_S = 0.2   # the default 5 Hz agent loop
HOLD_S = ObstacleRecoveryConfig().clear_hold_s
ASSIST_S = ObstacleRecoveryConfig().assist_after_s


def ticks(seconds):
    return int(round(seconds / TICK_S))


# --- the policy on its own ----------------------------------------------------


class TestObstacleRecoveryPolicy(unittest.TestCase):
    def setUp(self):
        self.clock = fx.FakeClock()
        self.rec = ObstacleRecovery(clock=self.clock)

    def tick(self, obstacle=False, usable=True, active=True):
        self.clock.advance(TICK_S)
        return self.rec.update(active=active, obstacle=obstacle, perception_usable=usable)

    def test_no_obstacle_is_never_held(self):
        for _ in range(50):
            self.assertFalse(self.tick().hold)
        self.assertFalse(self.rec.blocked)

    def test_five_second_obstacle_then_a_sustained_clear_path(self):
        for _ in range(ticks(5.0)):
            v = self.tick(obstacle=True)
            self.assertTrue(v.hold)
            self.assertFalse(v.assist)
        held = 0
        while self.tick().hold:
            held += 1
            self.assertLess(held, 100, "never released")
        # Released on the first tick with HOLD_S of clear behind it.
        self.assertEqual(held, ticks(HOLD_S))
        self.assertFalse(self.rec.blocked)

    def test_a_flickering_detection_restarts_the_clear_count(self):
        self.tick(obstacle=True)
        for _ in range(ticks(HOLD_S) - 1):
            self.assertTrue(self.tick().hold)
        self.assertTrue(self.tick(obstacle=True).hold)      # back, just before release
        for _ in range(ticks(HOLD_S)):
            self.assertTrue(self.tick().hold)                # the count started again
        self.assertFalse(self.tick().hold)

    def test_unusable_perception_is_not_a_clear_path(self):
        self.tick(obstacle=True)
        for _ in range(ticks(60)):                           # a minute of no usable vision
            v = self.tick(usable=False)
            self.assertTrue(v.hold)
            self.assertIn("perception unusable", v.reason)
        for _ in range(ticks(HOLD_S)):                       # the full hold, from scratch
            self.assertTrue(self.tick().hold)
        self.assertFalse(self.tick().hold)

    def test_a_persistent_obstacle_escalates_once_and_is_never_cleared_by_time(self):
        assists = []
        for i in range(ticks(10 * ASSIST_S)):
            v = self.tick(obstacle=True)
            self.assertTrue(v.hold)
            if v.assist:
                assists.append(i * TICK_S)              # since the first obstacle tick
        self.assertEqual(len(assists), 1)
        self.assertAlmostEqual(assists[0], ASSIST_S, delta=TICK_S)
        self.assertTrue(self.rec.blocked)

    def test_a_pause_keeps_the_episode_and_resume_rearms_it(self):
        self.tick(obstacle=True)
        for _ in range(ticks(ASSIST_S)):
            self.tick(obstacle=True)
        for _ in range(ticks(HOLD_S) * 3):                   # paused: path looks clear meanwhile
            self.assertFalse(self.tick(active=False).hold)
        self.assertTrue(self.rec.blocked, "pausing proved nothing about the path")
        self.rec.restart()                                   # operator RESUME
        self.assertTrue(self.tick().hold)                    # still needs its clear hold
        self.rec.restart()
        assists = [self.tick(obstacle=True).assist for _ in range(ticks(ASSIST_S) + 1)]
        self.assertEqual(assists.count(True), 1, "a resumed attempt can ask for help again")

    def test_config_is_validated(self):
        with self.assertRaises(ValueError):
            ObstacleRecoveryConfig(clear_hold_s=0)
        with self.assertRaises(ValueError):
            ObstacleRecoveryConfig(clear_hold_s=5, assist_after_s=5)


# --- no negative wheel command (RX-09) -------------------------------------------


def _detection(area, cx, label="obstacle"):
    half = int((area ** 0.5) / 2)
    return Detection(label=label, confidence=0.8, bbox=(cx - half, 100, cx + half, 100 + 2 * half),
                     area_px=area)


def _perception(*detections, delta=0):
    return PerceptionResult(timestamp=time.time(), status=PerceptionStatus.OK, detections=detections,
                            frame=FrameMetadata(width=640, height=480, age_s=0.05),
                            largest_area_delta_px=delta)


class TestNoNegativeWheelCommands(unittest.TestCase):
    def test_no_decision_ever_drives_a_side_backwards(self):
        decider = DecisionMaker(DecisionConfig())
        left, right = center_zone(640, DecisionConfig().center_zone_ratio)
        scenes = [_perception()] + [
            _perception(_detection(area, cx), delta=delta)
            for area in (500, 1500, 5000, 12000, 30000, 60000)
            for cx in (10, left, left + 5, 320, right - 5, right, 630)
            for delta in (0, 20000)
        ] + [_perception(_detection(4000, 320, label="person"))]
        errors = (None, 0.0, 4.0, -30.0, 45.0, -90.0, 150.0, 180.0)
        for scene in scenes:
            for error in errors:
                nav = NavigationState(status=NavigationStatus.NAVIGATING, target_waypoint=NORTH,
                                      heading_error_deg=error, current_heading_deg=None if error is None else 0.0)
                intent = decider.decide(mission_active=True, navigation=nav, perception=scene)
                self.assertGreaterEqual(min(intent.left, intent.right), 0.0, (intent, scene.detections, error))
                self.assertNotIn(intent.command, (MotionCommand.TURN_LEFT, MotionCommand.TURN_RIGHT,
                                                  MotionCommand.REVERSE))

    def test_the_safety_gate_vetoes_any_negative_wheel(self):
        gate = SafetyGate(SafetyConfig())
        for intent in (MotionIntent.turn_left(0.3), MotionIntent.turn_right(0.3), MotionIntent.reverse(0.2),
                       MotionIntent(command=MotionCommand.FORWARD, left=0.4, right=-0.01)):
            d = gate.evaluate(intent, mission_active=True, perception=_perception())
            self.assertIs(d.verdict, SafetyVerdict.VETOED, intent)
            self.assertEqual(d.rule, "negative_wheel")
            self.assertEqual((d.intent.left, d.intent.right), (0.0, 0.0))
        # Forward and steering-forward are untouched.
        for intent in (MotionIntent.forward(0.4), MotionIntent.forward(0.4, steer=1.0)):
            self.assertIs(gate.evaluate(intent, mission_active=True, perception=_perception()).verdict,
                          SafetyVerdict.ALLOWED)

    def test_the_estop_still_outranks_everything(self):
        gate = SafetyGate(SafetyConfig())
        gate.engage_estop("test")
        d = gate.evaluate(MotionIntent.turn_left(0.3), mission_active=True, perception=_perception())
        self.assertEqual(d.rule, "estop")


# --- the agent on a short route over a fake ESP32 ---------------------------------


HEALTHY = dict(motor_drive_available=True, front_obstacle=False, forward_blocked=False,
               state="IDLE", block_reason="NONE")


class Rig:
    """An agent driving one short leg over a fake ESP32 link, on one fake clock.

    One `step()` is one 200 ms agent tick: the ESP32 (while `alive`) answers
    every command written so far and sends fresh TELEMETRY, in one read; the
    agent ticks; the link writes whatever the agent submitted.
    """

    def __init__(self):
        self.agent = RobotAgent(headless_settings())
        self.agent.gps = FakeGPS()
        self.agent.gps.set_fix(*HERE)
        self.agent.perception = FakePerception()
        self.agent.perception.set_clear()
        self.port = fx.FakePort()
        self.link, self.clock = fx.make_link(self.port, motion_enabled=True)
        self.agent.esp32 = self.link
        self.agent.obstacles.clock = self.clock
        self.fields = dict(HEALTHY)
        self.alive = True
        self.uptime = 3_600_000
        self.answered = set()

        self.link.poll_once()
        self.port.feed(fx.telemetry(last_seq=None, uptime_ms=self.uptime, **self.fields))
        self.link.poll_once()
        self.esp32()
        assert self.link.status().motion_ready, self.link.status()

    def esp32(self, telemetry=True):
        out = b""
        for c in self.port.commands():
            if c["seq"] not in self.answered:
                self.answered.add(c["seq"])
                out += fx.ack(c["seq"], cmd=c["cmd"])
        if telemetry:
            self.uptime += int(TICK_S * 1000)
            out += fx.telemetry(uptime_ms=self.uptime, **self.fields)
        if out:
            self.port.feed(out)
        self.link.poll_once()

    def step(self):
        self.clock.advance(TICK_S)
        if self.alive:
            self.esp32()
        else:
            self.link.poll_once()
        snap = self.agent.tick()
        self.link.poll_once()
        return snap

    def run(self, seconds):
        return [self.step() for _ in range(ticks(seconds))]

    def drives(self, since=0):
        return [c for c in self.port.commands()[since:] if c["cmd"] == "DRIVE"]

    def mark(self):
        return len(self.port.commands())

    def start(self):
        self.agent.start_mission([NORTH])
        snaps = self.run(1.0)
        assert snaps[-1].mode is OperatingMode.AUTO, snaps[-1].mode
        assert self.drives(), "the rig never drove"
        return snaps


class AgentCase(unittest.TestCase):
    def setUp(self):
        self.rig = Rig()
        self.rig.start()

    def assert_forward_only(self, drives):
        for d in drives:
            self.assertGreaterEqual(min(d["left"], d["right"]), 0, d)


class TestTransientObstacle(AgentCase):
    def test_a_five_second_obstacle_then_clear_resumes_the_leg(self):
        rig = self.rig
        rig.agent.perception.set_detection(area=40000, cx=320)
        mark = rig.mark()
        snaps = rig.run(5.0)
        self.assertEqual(rig.drives(mark), [], "DRIVE while the obstacle was in view")
        for s in snaps:
            self.assertIs(s.mode, OperatingMode.AUTO)
            self.assertIsNot(s.navigation.status, NavigationStatus.REROUTE_NEEDED)
            self.assertTrue(s.motion_intent.is_stop)

        rig.agent.perception.set_clear()
        mark = rig.mark()
        held = 0
        while not rig.drives(mark):
            snap = rig.step()
            held += 1
            self.assertLess(held, ticks(HOLD_S) + 3, "the leg never resumed")
            self.assertIsNot(snap.navigation.status, NavigationStatus.REROUTE_NEEDED)
        # No DRIVE until the path had been clear for the full hold.
        self.assertGreaterEqual(held * TICK_S, HOLD_S - 1e-9)
        self.assertLessEqual(held * TICK_S, HOLD_S + TICK_S + 1e-9)
        snap = rig.step()
        self.assertIs(snap.mode, OperatingMode.AUTO)
        self.assertIs(snap.navigation.status, NavigationStatus.NAVIGATING)
        self.assert_forward_only(rig.drives())

    def test_a_brief_obstacle_no_longer_makes_reroute_needed_permanent(self):
        # RX-08 as observed: three obstacle ticks were enough, forever.
        rig = self.rig
        rig.agent.perception.set_detection(area=40000, cx=320)
        rig.run(0.6)
        rig.agent.perception.set_clear()
        snaps = rig.run(HOLD_S + 1.0)
        self.assertNotIn(NavigationStatus.REROUTE_NEEDED, [s.navigation.status for s in snaps])
        self.assertFalse(snaps[-1].motion_intent.is_stop)

    def test_a_person_is_held_for_a_sustained_clear_path_too(self):
        rig = self.rig
        rig.agent.perception.set_detection(label="person", area=2000, cx=40)
        rig.run(1.0)
        rig.agent.perception.set_clear()
        mark = rig.mark()
        rig.run(HOLD_S - TICK_S)
        self.assertEqual(rig.drives(mark), [])


class TestPersistentObstacle(AgentCase):
    def test_it_never_drives_and_pauses_for_an_operator(self):
        rig = self.rig
        rig.agent.perception.set_detection(area=40000, cx=320)
        mark = rig.mark()
        snaps = rig.run(ASSIST_S + 1.0)
        self.assertEqual(rig.drives(mark), [])
        self.assertIs(snaps[-1].mode, OperatingMode.PAUSED)
        self.assertIn("operator assistance", snaps[-1].last_error)
        self.assertNotIn(NavigationStatus.REROUTE_NEEDED, [s.navigation.status for s in snaps])

        # The obstacle goes away: still paused -- nothing resumes on its own.
        rig.agent.perception.set_clear()
        snaps = rig.run(3 * HOLD_S)
        self.assertTrue(all(s.mode is OperatingMode.PAUSED for s in snaps))
        self.assertEqual(rig.drives(mark), [])

        # RESUME re-arms; the path still has to prove itself clear first.
        rig.agent.resume_mission("operator")
        self.assertIsNone(rig.agent.state.snapshot().last_error)
        rig.run(HOLD_S - TICK_S)
        self.assertEqual(rig.drives(mark), [], "DRIVE before a sustained clear path after RESUME")
        rig.run(2 * TICK_S)
        self.assertTrue(rig.drives(mark))
        self.assert_forward_only(rig.drives())

    def test_a_resume_into_the_same_obstacle_holds_and_asks_again(self):
        rig = self.rig
        rig.agent.perception.set_detection(area=40000, cx=320)
        mark = rig.mark()
        rig.run(ASSIST_S + 1.0)
        rig.agent.resume_mission("operator")
        snaps = rig.run(ASSIST_S + 1.0)
        self.assertIs(snaps[-1].mode, OperatingMode.PAUSED)
        self.assertEqual(rig.drives(mark), [])


class TestInvalidPerception(AgentCase):
    def test_stale_or_missing_perception_stops_and_does_not_count_as_clear(self):
        rig = self.rig
        rig.agent.perception.set_detection(area=40000, cx=320)
        rig.run(1.0)
        mark = rig.mark()
        for status in (PerceptionStatus.STALE, PerceptionStatus.NO_FRAME, PerceptionStatus.DETECTOR_ERROR):
            rig.agent.perception.result = PerceptionResult.unavailable(status)
            for snap in rig.run(HOLD_S + 1.0):
                self.assertTrue(snap.motion_intent.is_stop)
        rig.agent.perception.result = PerceptionResult(timestamp=time.time(), status=PerceptionStatus.OK)
        rig.run(HOLD_S + 1.0)                                      # OK but no frame metadata
        self.assertEqual(rig.drives(mark), [])

        rig.agent.perception.set_clear()
        rig.run(HOLD_S - TICK_S)
        self.assertEqual(rig.drives(mark), [], "the unusable stretch counted toward the hold")
        rig.run(2 * TICK_S)
        self.assertTrue(rig.drives(mark))

    def test_unusable_perception_with_no_obstacle_episode_still_stops(self):
        rig = self.rig
        rig.agent.perception.result = PerceptionResult.unavailable(PerceptionStatus.STALE)
        mark = rig.mark()
        snaps = rig.run(2.0)
        self.assertEqual(rig.drives(mark), [])
        self.assertTrue(all("perception" in s.motion_intent.reason for s in snaps))


class TestRearSensorFailureDuringAvoidance(AgentCase):
    REAR_FAULT = dict(rear_available=False, rear_sensor_fault=True, reverse_blocked=True,
                      rear_mm=[None, None, None])

    def test_an_off_centre_obstacle_with_a_failed_rear_never_reverses(self):
        rig = self.rig
        rig.fields.update(self.REAR_FAULT)
        rig.step()
        left, right = center_zone(640, rig.agent.decision.cfg.center_zone_ratio)
        mark = rig.mark()
        for cx in (left + 5, right - 5):                           # both formerly pivots
            rig.agent.perception.set_detection(area=5000, cx=cx)
            snaps = rig.run(2.0)
            for s in snaps:
                self.assertTrue(s.motion_intent.is_stop)
                self.assertEqual((s.motion_intent.left, s.motion_intent.right), (0.0, 0.0))
                self.assertIs(s.mode, OperatingMode.AUTO)       # a rear fault is not a forward fault
        self.assertEqual(rig.drives(mark), [])
        self.assertIn("STOP", [c["cmd"] for c in rig.port.commands()[mark:]])

        rig.agent.perception.set_clear()
        rig.run(HOLD_S + 2 * TICK_S)
        self.assertTrue(rig.drives(mark))
        self.assert_forward_only(rig.drives())
        self.assertTrue(rig.link.status().motion_ready)


class TestLinkLossDuringMission(AgentCase):
    def test_an_observed_outage_pauses_and_recovery_resumes_nothing(self):
        rig = self.rig
        mark = rig.mark()
        rig.alive = False
        snaps = rig.run(1.6)                                       # STALE after 1 s
        self.assertIs(snaps[-1].mode, OperatingMode.PAUSED)
        self.assertFalse(rig.link.status().motion_ready)
        lost_at = rig.mark()

        rig.alive = True                                           # back; P3 re-proves the link
        snaps = rig.run(3.0)
        self.assertIs(rig.link.status().link, S.UP)
        self.assertTrue(rig.link.status().motion_ready)
        self.assertTrue(all(s.mode is OperatingMode.PAUSED for s in snaps))
        self.assertEqual(rig.drives(lost_at), [], "a recovered link resumed the mission on its own")
        self.assertGreaterEqual(rig.link.status().controller.counters.link_losses, 1)

        rig.agent.resume_mission("operator")
        rig.run(1.0)
        self.assertTrue(rig.drives(lost_at))
        self.assert_forward_only(rig.drives(mark))

    def test_a_loss_recovered_between_two_ticks_still_pauses(self):
        """The P3 link can lose its proof and re-prove itself while the agent is
        not looking. The agent must not drive on as if nothing happened."""
        rig = self.rig
        rig.step()
        before = rig.link.status().controller.counters.link_losses

        # Between two agent ticks: the ESP32 goes quiet past STALE, comes back,
        # and answers the re-probe PING -- all at the link's own pace.
        rig.clock.advance(1.2)
        rig.link.poll_once()
        self.assertFalse(rig.link.status().motion_ready)
        for _ in range(3):
            rig.clock.advance(0.05)
            rig.esp32()
        st = rig.link.status()
        self.assertIs(st.link, S.UP)
        self.assertTrue(st.motion_ready, st.detail)
        self.assertEqual(st.controller.counters.link_losses, before + 1)

        mark = rig.mark()
        snap = rig.agent.tick()
        rig.link.poll_once()
        self.assertIs(snap.mode, OperatingMode.PAUSED)
        self.assertTrue(snap.motion_intent.is_stop)
        self.assertEqual(rig.drives(mark), [])
        snaps = rig.run(2.0)
        self.assertTrue(all(s.mode is OperatingMode.PAUSED for s in snaps))
        self.assertEqual(rig.drives(mark), [])

    def test_a_disconnect_and_reconnect_between_ticks_still_pauses(self):
        rig = self.rig
        rig.step()
        rig.agent._esp32_interruptions_seen = (
            rig.agent._esp32_interruptions_seen[0],
            rig.agent._esp32_interruptions_seen[1] - 1,           # as if the link had reopened
        )
        snap = rig.agent.tick()
        self.assertIs(snap.mode, OperatingMode.PAUSED)

    def test_an_interruption_while_idle_does_not_pause_a_later_mission(self):
        rig = Rig()
        rig.alive = False
        rig.run(1.6)
        rig.alive = True
        rig.run(1.0)
        self.assertTrue(rig.link.status().motion_ready)
        rig.agent.start_mission([NORTH])
        snaps = rig.run(1.0)
        self.assertIs(snaps[-1].mode, OperatingMode.AUTO)
        self.assertTrue(rig.drives())


class TestResumeNeedsEveryReadinessCondition(AgentCase):
    """After a pause, RESUME alone moves nothing: each missing condition holds."""

    def paused(self):
        rig = self.rig
        rig.agent.pause_mission("test")
        rig.run(0.4)
        return rig, rig.mark()

    def test_no_position(self):
        rig, mark = self.paused()
        rig.agent.gps.reading = FakeGPS().reading                   # NO_FIX
        rig.agent.resume_mission("operator")
        snaps = rig.run(2.0)
        self.assertEqual(rig.drives(mark), [])
        self.assertIs(snaps[-1].navigation.status, NavigationStatus.NO_POSITION)

    def test_inaccurate_position(self):
        rig, mark = self.paused()
        fix = rig.agent.gps.reading.fix
        rig.agent.gps.reading = GpsReading(status=GPSStatus.FIX, age_s=0.1, fix=GpsFix(
            latitude=fix.latitude, longitude=fix.longitude, timestamp=time.time(), h_acc_m=9.0))
        rig.agent.resume_mission("operator")
        snaps = rig.run(2.0)
        self.assertEqual(rig.drives(mark), [])
        self.assertIsNotNone(snaps[-1].navigation.position_issue)

    def test_unusable_perception(self):
        rig, mark = self.paused()
        rig.agent.perception.result = PerceptionResult.unavailable(PerceptionStatus.STALE)
        rig.agent.resume_mission("operator")
        rig.run(2.0)
        self.assertEqual(rig.drives(mark), [])

    def test_controller_fault(self):
        rig, mark = self.paused()
        rig.fields.update(safety_stop=True)
        rig.step()
        rig.agent.resume_mission("operator")
        snaps = rig.run(2.0)
        self.assertIs(snaps[-1].mode, OperatingMode.PAUSED)
        self.assertEqual(rig.drives(mark), [])

    def test_link_not_re_proven(self):
        rig, mark = self.paused()
        rig.alive = False
        rig.run(1.6)                                              # STALE, proof lost
        rig.agent.resume_mission("operator")
        snaps = rig.run(1.0)
        self.assertIs(snaps[-1].mode, OperatingMode.PAUSED)
        self.assertEqual(rig.drives(mark), [])

    def test_estop_latched(self):
        rig, mark = self.paused()
        rig.agent.emergency_stop("test")
        rig.agent.resume_mission("operator")                     # mode only; the latch holds
        rig.run(2.0)
        self.assertEqual(rig.drives(mark), [])
        self.assertTrue(rig.agent.emergency_stopped)

    def test_all_conditions_met_drives_forward(self):
        rig, mark = self.paused()
        rig.agent.resume_mission("operator")
        rig.run(0.4)
        self.assertTrue(rig.drives(mark))
        self.assert_forward_only(rig.drives(mark))


if __name__ == "__main__":
    unittest.main()
