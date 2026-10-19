"""Applied-motion monitoring: GATED/REJECTED DRIVE ACKs and zero applied output.

What is real and what is simulated
----------------------------------
- REAL: `AppliedMotionMonitor`, `RobotAgent`, the safety gate, `Esp32Link`
  (framing, ACK matching, the new DRIVE ACK record) and the settings parser.
- SIMULATED: the ESP32 (`tests.fixtures.esp32_sim.FirmwareModel` over a fake
  serial port), GPS fixes and perception, and the clock.

`applied_*` is the PWM output the firmware reports. Nothing here -- or on the
rover -- observes a wheel turning; a pass shows only that the Pi reacts
correctly to the firmware's report.
"""

from __future__ import annotations

import logging
import unittest

from robotx.application.agent import RobotAgent  # first: robot_state cannot be imported first (pre-existing cycle)
from robotx.config.settings import Settings
from robotx.control.applied_motion import (
    AppliedMotionConfig,
    AppliedMotionMonitor,
    MAX_APPLIED_TIMEOUT_S,
    MIN_APPLIED_TIMEOUT_S,
    driven_sides,
)
from robotx.esp32.state import ControllerState, ControllerTelemetry, DriveAck, LinkCounters, SentMotion
from robotx.state.robot_state import OperatingMode
from tests.fixtures import esp32 as fx
from tests.unit.test_obstacle_recovery import TICK_S, Rig, ticks


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def tel(uptime, left_cmd=120, right_cmd=120, left_applied=0, right_applied=0):
    return ControllerTelemetry.from_frame(
        {**fx.REAL_TELEMETRY_FIELDS, "uptime_ms": uptime, "left_cmd": left_cmd, "right_cmd": right_cmd,
         "left_applied": left_applied, "right_applied": right_applied}, received_at=0.0)


def controller(*, sent=("DRIVE", 120, 120), telemetry=None, age=0.1, ack=None, acks=0):
    return ControllerState(
        telemetry=telemetry,
        telemetry_age_s=None if telemetry is None else age,
        counters=LinkCounters(drive_acks=acks),
        last_motion_sent=None if sent is None else SentMotion(*sent, seq=1, age_s=0.05),
        last_drive_ack=ack,
    )


class MonitorCase(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.monitor = AppliedMotionMonitor(AppliedMotionConfig(applied_timeout_s=1.0), clock=self.clock)
        self.uptime = 10_000
        self.acks = 0

    # 0.25 s steps: exact in binary, so the timeout boundary is not a rounding question.
    def step(self, dt=0.25, *, commanding=True, sent=("DRIVE", 120, 120), applied=(0, 0), cmd=None,
             age=0.1, ack=None, stale_after_s=1.0, telemetry=True):
        self.clock.t += dt
        self.uptime += int(dt * 1000)
        if ack is not None:
            self.acks += 1
        cmd = cmd or (sent[1], sent[2]) if sent else (0, 0)
        frame = tel(self.uptime, cmd[0], cmd[1], applied[0], applied[1]) if telemetry else None
        return self.monitor.update(
            commanding=commanding,
            controller=controller(sent=sent, telemetry=frame, age=age, ack=ack, acks=self.acks),
            stale_after_s=stale_after_s,
        )

    def run_for(self, seconds, **kw):
        return [self.step(**kw) for _ in range(int(round(seconds / 0.25)))]


class TestZeroApplied(MonitorCase):
    def test_zero_applied_beyond_the_timeout_is_a_fault_with_the_reason(self):
        verdicts = self.run_for(1.0)          # 0.75 s of zero output across four frames
        self.assertFalse(any(v.fault for v in verdicts), "tripped before the timeout")
        v = self.step()                       # 1.0 s
        self.assertTrue(v.fault)
        self.assertEqual(v.kind, "ZERO_APPLIED")
        self.assertIn("DRIVE 120/120", v.reason)
        self.assertIn("applies 0/0", v.reason)

    def test_one_fault_per_episode_then_reset_rearms(self):
        faults = [v for v in self.run_for(3.0) if v.fault]
        self.assertEqual(len(faults), 1)
        self.monitor.reset()
        self.assertTrue(any(v.fault for v in self.run_for(1.4)))

    def test_output_reaching_the_motors_clears_the_timer(self):
        self.run_for(0.8)
        self.step(applied=(120, 120))
        self.assertFalse(any(v.fault for v in self.run_for(0.8)))

    def test_needs_two_distinct_frames(self):
        # One frame, re-read for longer than the timeout, is not two observations.
        self.step()
        for _ in range(8):
            self.clock.t += 0.25
            v = self.monitor.update(commanding=True, controller=controller(
                telemetry=tel(self.uptime), acks=self.acks), stale_after_s=5.0)
            self.assertFalse(v.fault)


class TestNoFalseAlarms(MonitorCase):
    def test_stop_commands_never_count(self):
        self.assertFalse(any(v.fault for v in self.run_for(5.0, sent=("STOP", 0, 0))))

    def test_not_commanding_never_counts_and_resets(self):
        self.run_for(0.8)
        self.step(commanding=False)                 # an intentional pause
        self.assertFalse(any(v.fault for v in self.run_for(0.8)))
        self.assertFalse(any(v.fault for v in self.run_for(5.0, commanding=False)))

    def test_motion_disabled_means_nothing_was_sent(self):
        self.assertFalse(any(v.fault for v in self.run_for(5.0, sent=None)))

    def test_drive_inside_the_deadband_is_expected_to_apply_zero(self):
        self.assertFalse(any(v.fault for v in self.run_for(5.0, sent=("DRIVE", 39, 30))))

    def test_only_sides_beyond_the_deadband_are_judged(self):
        ok = self.run_for(5.0, sent=("DRIVE", 30, 120), applied=(0, 120))
        self.assertFalse(any(v.fault for v in ok))
        self.monitor.reset()
        bad = self.run_for(1.4, sent=("DRIVE", 30, 120), applied=(0, 0))
        self.assertTrue(any(v.fault for v in bad))

    def test_stale_telemetry_neither_advances_nor_resets(self):
        self.run_for(0.6)
        stale = self.run_for(5.0, age=3.0)          # older than stale_after_s
        self.assertFalse(any(v.fault for v in stale))
        # The timer was frozen, not reset: fresh zero frames resume it...
        self.assertTrue(any(v.fault for v in self.run_for(1.0)))

    def test_missing_telemetry_is_no_evidence(self):
        self.assertFalse(any(v.fault for v in self.run_for(5.0, telemetry=False)))

    def test_driven_sides(self):
        self.assertEqual(driven_sides(SentMotion("DRIVE", 40, -40), 40), ["left", "right"])
        self.assertEqual(driven_sides(SentMotion("DRIVE", 39, 0), 40), [])
        self.assertEqual(driven_sides(SentMotion("STOP"), 40), [])
        self.assertEqual(driven_sides(None, 40), [])


def drive_ack(result, reason="NONE", seq=7):
    return DriveAck(seq=seq, result=result, reason=reason, req_left=120, req_right=120,
                    gated_left=0, gated_right=0, applied_left=0, applied_right=0)


class TestAcks(MonitorCase):
    def test_rejected_drive_is_an_immediate_fault(self):
        self.step(applied=(120, 120))
        v = self.step(applied=(120, 120), ack=drive_ack("REJECTED", "MOTOR_PWM_UNAVAILABLE"))
        self.assertTrue(v.fault)
        self.assertEqual(v.kind, "DRIVE_REJECTED")
        self.assertIn("MOTOR_PWM_UNAVAILABLE", v.reason)

    def test_stale_seq_rejection_is_the_protocol_resyncing_not_a_fault(self):
        self.step(applied=(120, 120))
        self.assertFalse(self.step(applied=(120, 120), ack=drive_ack("REJECTED", "STALE_SEQ")).fault)

    def test_persistent_gating_is_a_fault_after_the_timeout(self):
        self.step(applied=(120, 120))
        verdicts = [self.step(applied=(120, 120), ack=drive_ack("GATED", "REAR_SENSOR_FAULT")) for _ in range(4)]
        self.assertFalse(any(v.fault for v in verdicts))
        v = self.step(applied=(120, 120), ack=drive_ack("GATED", "REAR_SENSOR_FAULT"))
        self.assertTrue(v.fault)
        self.assertEqual(v.kind, "DRIVE_GATED")
        self.assertIn("REAR_SENSOR_FAULT", v.reason)

    def test_an_accepted_ack_ends_a_gating_episode(self):
        self.step(applied=(120, 120))
        for _ in range(4):
            self.step(applied=(120, 120), ack=drive_ack("GATED", "FRONT_OBSTACLE"))
        self.step(applied=(120, 120), ack=drive_ack("ACCEPTED"))
        self.assertFalse(any(self.step(applied=(120, 120), ack=drive_ack("GATED", "FRONT_OBSTACLE")).fault
                             for _ in range(4)))

    def test_an_ack_already_seen_is_not_counted_again(self):
        # The first sighting only learns the counter; an old REJECTED is not new.
        self.monitor.update(commanding=False, controller=controller(ack=drive_ack("REJECTED", "X"), acks=5),
                            stale_after_s=1.0)
        v = self.monitor.update(commanding=True, controller=controller(
            telemetry=tel(1), ack=drive_ack("REJECTED", "X"), acks=5), stale_after_s=1.0)
        self.assertFalse(v.fault)


class TestConfiguration(unittest.TestCase):
    def test_defaults_and_env(self):
        s = Settings.from_env({})
        self.assertEqual((s.motion_applied_timeout_s, s.motion_deadband_units), (1.5, 40))
        cfg = AppliedMotionConfig.from_settings(Settings.from_env({
            "ROBOTX_MOTION_APPLIED_TIMEOUT_S": "2.5", "ROBOTX_MOTION_DEADBAND_UNITS": "30"}))
        self.assertEqual((cfg.applied_timeout_s, cfg.deadband_units), (2.5, 30))

    def test_bounds_are_enforced(self):
        for bad in (MIN_APPLIED_TIMEOUT_S - 0.01, MAX_APPLIED_TIMEOUT_S + 0.01, float("nan"), 0, -1):
            with self.assertRaises(ValueError, msg=bad):
                AppliedMotionConfig(applied_timeout_s=bad)
        for bad in (-1, 255, True, 40.0):
            with self.assertRaises(ValueError, msg=bad):
                AppliedMotionConfig(deadband_units=bad)


# --- the agent, driving over the SIMULATED firmware ---------------------------------


class AgentCase(unittest.TestCase):
    def setUp(self):
        self.rig = Rig()
        self.rig.start()

    def assert_paused_by_controller(self, snaps, mark, kind_text):
        last = snaps[-1]
        self.assertIs(last.mode, OperatingMode.PAUSED)
        self.assertTrue(last.motion_intent.is_stop)
        self.assertTrue(last.last_error.startswith("controller: "), last.last_error)
        self.assertIn(kind_text, last.last_error)
        self.assertEqual(self.rig.port.command_names()[-1], "STOP")
        # Nothing but STOP after the pause tick.
        paused_at = next(i for i, s in enumerate(snaps) if s.mode is OperatingMode.PAUSED)
        self.assertLessEqual(paused_at, ticks(1.5) + 2)
        return paused_at


class TestAgentHealthyFirmware(AgentCase):
    def test_a_healthy_controller_drives_on_without_a_false_alarm(self):
        mark = self.rig.mark()
        snaps = self.rig.run(10.0)
        self.assertTrue(all(s.mode is OperatingMode.AUTO for s in snaps), [s.last_error for s in snaps][-1])
        self.assertGreater(len(self.rig.drives(mark)), ticks(9.0))
        ack = self.rig.link.status().controller.last_drive_ack
        self.assertEqual((ack.result, ack.applied_left > 0), ("ACCEPTED", True))


class TestAgentFaults(AgentCase):
    def test_zero_applied_output_pauses_with_the_reason(self):
        self.rig.firmware.applied_zero = True
        mark = self.rig.mark()
        snaps = self.rig.run(3.0)
        i = self.assert_paused_by_controller(snaps, mark, "applies 0/0")
        drives_after = self.rig.drives(mark)
        # Drives stop once paused: none after the pause tick's own submission.
        self.assertLessEqual(len(drives_after), i + 1)
        self.assertTrue(all(s.mode is OperatingMode.PAUSED for s in snaps[i:]))

    def test_persistent_gating_pauses_with_the_gate_reason(self):
        self.rig.firmware.gate_reason = "REAR_SENSOR_FAULT"
        snaps = self.rig.run(3.0)
        self.assert_paused_by_controller(snaps, self.rig.mark(), "REAR_SENSOR_FAULT")

    def test_a_rejected_drive_pauses_at_once(self):
        self.rig.firmware.reject_reason = "MOTOR_PWM_UNAVAILABLE"
        snaps = self.rig.run(1.0)
        i = next(i for i, s in enumerate(snaps) if s.mode is OperatingMode.PAUSED)
        self.assertLessEqual(i, 2)
        self.assertIn("REJECTED", snaps[i].last_error)

    def test_resume_is_explicit_and_a_persisting_fault_pauses_again(self):
        self.rig.firmware.applied_zero = True
        self.rig.run(3.0)
        self.assertIs(self.rig.agent.mode, OperatingMode.PAUSED)
        mark = self.rig.mark()
        self.rig.run(2.0)                                   # stays paused on its own
        self.assertEqual(self.rig.drives(mark), [])
        self.rig.agent.resume_mission("operator")
        snaps = self.rig.run(3.0)
        self.assertIs(snaps[-1].mode, OperatingMode.PAUSED)
        self.rig.firmware.applied_zero = False
        self.rig.agent.resume_mission("operator")
        snaps = self.rig.run(5.0)
        self.assertTrue(all(s.mode is OperatingMode.AUTO for s in snaps))

    def test_an_operator_pause_is_not_reported_as_a_controller_fault(self):
        self.rig.firmware.applied_zero = True
        self.rig.run(0.6)
        self.rig.agent.pause_mission("operator pause")
        snaps = self.rig.run(5.0)
        self.assertIs(snaps[-1].mode, OperatingMode.PAUSED)
        self.assertFalse((snaps[-1].last_error or "").startswith("controller:"), snaps[-1].last_error)

    def test_motion_disabled_is_never_a_fault(self):
        from tests.unit.test_agent import HERE, NORTH, FakeGPS, FakePerception, headless_settings

        agent = RobotAgent(headless_settings())
        agent.gps = FakeGPS()
        agent.gps.set_fix(*HERE)
        agent.perception = FakePerception()
        agent.perception.set_clear()
        port = fx.FakePort()
        link, clock = fx.make_link(port, motion_enabled=False)
        agent.esp32, agent.applied_motion.clock = link, clock
        fx.bring_up(link, port, clock)
        agent.start_mission([NORTH])
        uptime = fx.REAL_TELEMETRY_FIELDS["uptime_ms"]
        for _ in range(ticks(5.0)):
            clock.advance(TICK_S)
            uptime += int(TICK_S * 1000)
            port.feed(fx.telemetry(uptime_ms=uptime, **fx.CLEAR_FRONT))
            link.poll_once()
            snap = agent.tick()
            link.poll_once()
            self.assertIs(snap.mode, OperatingMode.AUTO)
        self.assertEqual(port.command_names(), ["PING"])


class TestLinkRecordsDriveAcks(unittest.TestCase):
    def test_the_last_drive_ack_and_sent_motion_are_exposed(self):
        rig = Rig()
        rig.start()
        status = rig.link.status().controller
        self.assertEqual(status.last_motion_sent.cmd, "DRIVE")
        self.assertGreater(status.counters.drive_acks, 0)
        self.assertEqual(status.counters.drive_gated, 0)
        self.assertIn("last_drive_ack", status.to_dict())
        rig.firmware.gate_reason = "FRONT_SENSOR_FAULT"
        rig.run(0.4)
        status = rig.link.status().controller
        self.assertGreater(status.counters.drive_gated, 0)
        self.assertEqual(status.last_drive_ack.reason, "FRONT_SENSOR_FAULT")


if __name__ == "__main__":
    unittest.main()
