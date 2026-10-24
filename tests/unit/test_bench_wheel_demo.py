"""The bench wheel-test demonstration (ROBOTX_BENCH_WHEEL_DEMO, default off).

What is pinned:
  - it is opt-in, and refused at startup unless the motor path, the bench
    route and the backend are all enabled and the offer-acceptance demo is not;
  - a signed OFFER is admitted and ACCEPTed through the real BackendLink with
    nothing started: accepting writes no DRIVE, ever;
  - one bench pulse may then be requested for that task -- a separate operator
    act -- and is refused without the task, for another task, for a task whose
    commitment ended, over a disconnected backend, or after an agent restart;
  - a running pulse stops, with the STOP confirmed by controller TELEMETRY,
    on RECALL / WITHDRAW / ABORT_MISSION, an operator STOP (COMMAND or bare
    STOP event), the commitment ending by any other path, the backend link
    dropping, or its own deadline -- and nothing resumes it;
  - no mission, local route, RESUME or RETURN is ever driven in this mode;
  - with the mode off, a task binding is refused and the old rules hold.

SIMULATED, all of it: the real `RobotAgent`, `Esp32Link` and `BackendLink`, over
a fake serial port answered by `FirmwareModel` and a fake Socket.IO client
(`FakeSio`), with envelopes signed by the fixtures' TEST-ONLY key. A pass shows
the Pi's decisions against *reported* controller feedback. It is not evidence
that a wheel turned or stopped, nor that the real backend or ESP32 behave so.
"""

from __future__ import annotations

import asyncio
import dataclasses
import logging
import unittest

from robotx.application.bench_gate import BenchGate, BenchRejected
from robotx.control.bench_motion import BenchMotionConfig, BenchOutcome, BenchRefused
from robotx.mission.mission import MissionRejected, MissionRejectReason
from robotx.state.robot_state import BackendLinkStatus, MissionRefused, OperatingMode
from tests.fixtures import engine as fx
from tests.fixtures.task_assign import START
from tests.unit.test_backend_link import connect_and_auth
from tests.unit.test_bench_motion import ATTESTED, UNITS, acall, gate_config, run_until_idle
from tests.unit.test_engine_offer import engine_link, names, parsed_offer, real_agent
from tests.unit.test_obstacle_recovery import NORTH, Rig

WHEEL = {
    "ROBOTX_BENCH_WHEEL_DEMO": "1",
    "ROBOTX_ESP32_MOTION_ENABLED": "1",
    "ROBOTX_BENCH_MOTION_ENABLED": "1",
    "ROBOTX_SOCKET_ENABLED": "1",
}
TASK = "T-123"            # fx.offer_payload's taskId
COMMITMENT = "c-7f3a"     # fx.offer_payload's commitmentId


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


def terminating(command):
    return fx.envelope(command=command, fence="43", sequence=1, outbox_id="ob-21")


class WheelRig(Rig):
    """`Rig`, in the bench wheel-test demonstration, with a backend link attached.

    The agent is the real one; its settings are swapped for the demonstration's
    after construction because `Rig` builds it headless. Everything that reads
    the mode reads `agent.settings` at use, so this is the mode as deployed.
    """

    def __init__(self):
        super().__init__()
        self.agent.settings = dataclasses.replace(
            self.agent.settings, bench_wheel_demo=True, esp32_motion_enabled=True,
            bench_motion_enabled=True, socket_enabled=True)
        self.agent.bench.clock = self.clock
        self.agent.state.set_mode(OperatingMode.IDLE)
        self.agent.gps.set_fix(*START)
        # `Rig.link` is the ESP32 link; this is the backend one.
        self.backend_link, self.sio = engine_link(agent=self.agent, state=self.agent.state)
        self.agent.backend = self.backend_link
        asyncio.run(connect_and_auth(self.backend_link))
        self.run(0.6)

    def fire(self, event, data):
        """One backend event, delivered; returns what the Pi emitted for it."""

        async def go():
            before = len(self.sio.emitted)
            await self.sio.fire(event, data)
            return self.sio.emitted[before:]

        return asyncio.run(go())

    def accept(self):
        out = self.fire("OFFER", fx.envelope())
        assert names(out) == ["COMMAND_ACK", "OFFER_ACCEPT"], out
        return out

    def pulse(self, task_id=TASK, **kw):
        kw.setdefault("speed", 0.25)
        kw.setdefault("duration_s", 2.0)
        return self.agent.start_bench_motion(source="test", task_id=task_id, **kw)


class TestConfiguration(unittest.TestCase):
    def test_off_by_default(self):
        from robotx.config.settings import Settings

        self.assertFalse(Settings.from_env({}).bench_wheel_demo)
        self.assertTrue(Settings.from_env({"ROBOTX_BENCH_WHEEL_DEMO": "1"}).bench_wheel_demo)

    def test_refused_unless_every_prerequisite_is_enabled(self):
        real_agent(**WHEEL)
        for missing in ("ROBOTX_ESP32_MOTION_ENABLED", "ROBOTX_BENCH_MOTION_ENABLED", "ROBOTX_SOCKET_ENABLED"):
            with self.assertRaises(ValueError, msg=missing):
                real_agent(**{**WHEEL, missing: "0"})

    def test_never_combined_with_the_offer_acceptance_demo(self):
        with self.assertRaises(ValueError):
            real_agent(**WHEEL, ROBOTX_OFFER_ACCEPTANCE_DEMO="1")


class TestAcceptanceIsNotMotion(unittest.TestCase):
    def test_a_signed_offer_is_accepted_and_nothing_moves(self):
        rig = WheelRig()
        mark = rig.mark()
        out = rig.accept()
        self.assertEqual(out[1][1]["commitmentId"], COMMITMENT)
        rig.run(5.0)
        self.assertEqual(rig.drives(mark), [], "accepting the offer wrote a DRIVE")
        self.assertIs(rig.agent.mode, OperatingMode.IDLE)
        self.assertIsNone(rig.agent.state.snapshot().mission)
        self.assertIsNone(rig.backend_link._held_mission(), "no mission, so no lease renewal")
        status = rig.agent.bench_wheel_status()
        self.assertEqual((status["held_task_id"], status["accepted_tasks"]), (TASK, [TASK]))

    def test_custody_and_perception_do_not_block_acceptance_but_the_motor_link_does(self):
        rig = WheelRig()
        self.assertEqual(rig.agent.readiness_blockers(), [])
        rig.fields.update(front_valid=False)          # the controller cannot carry motion
        rig.run(0.4)
        self.assertEqual(rig.agent.readiness_blockers(), ["NO_MOTOR_LINK"])
        out = rig.fire("OFFER", fx.envelope())
        self.assertEqual(names(out), ["COMMAND_ACK", "OFFER_REJECT"])


class TestThePulseIsBoundToTheAcceptedTask(unittest.TestCase):
    def test_an_operator_pulse_for_the_accepted_task_runs_and_stops(self):
        rig = WheelRig()
        rig.accept()
        mark = rig.mark()
        rig.pulse(duration_s=1.0)
        run_until_idle(rig)
        result = rig.agent.bench.last_result
        self.assertEqual(result["outcome"], BenchOutcome.COMPLETED.value, result)
        self.assertTrue(result["stop_confirmed"])
        self.assertEqual((result["task_id"], result["commitment_id"]), (TASK, COMMITMENT))
        self.assertEqual(result["max_telemetry_applied"], [UNITS, UNITS])
        self.assertTrue(rig.drives(mark))
        self.assertEqual(rig.port.command_names()[-1], "STOP")
        after = rig.mark()
        rig.run(3.0)
        self.assertEqual(rig.drives(after), [], "DRIVE after the pulse ended")

    def refused(self, rig, code, **kw):
        with self.assertRaises(BenchRefused) as ctx:
            rig.pulse(**kw)
        self.assertEqual(ctx.exception.code, code, ctx.exception.detail)
        mark = rig.mark()
        rig.run(0.6)
        self.assertEqual(rig.drives(mark), [], f"{code}: DRIVE after a refusal")

    def test_refused_without_an_accepted_task(self):
        rig = WheelRig()
        self.refused(rig, "TASK_REQUIRED", task_id=None)
        self.refused(rig, "TASK_NOT_ACCEPTED")
        rig.accept()
        self.refused(rig, "TASK_NOT_ACCEPTED", task_id="T-other")

    def test_refused_once_the_commitment_has_ended(self):
        rig = WheelRig()
        rig.accept()
        rig.fire("RECALL", terminating("RECALL"))
        self.refused(rig, "TASK_NOT_ACCEPTED")
        # Even an eligibility left behind cannot outlive the commitment.
        rig.agent._bench_wheel_tasks.add(TASK)
        self.refused(rig, "TASK_NOT_HELD")

    def test_refused_over_a_disconnected_backend(self):
        rig = WheelRig()
        rig.accept()
        rig.backend_link._set_status(BackendLinkStatus.DISCONNECTED, "test")
        self.refused(rig, "BACKEND_NOT_CONNECTED")

    def test_an_agent_restart_makes_the_acceptance_ineligible(self):
        rig = WheelRig()
        rig.accept()
        # A new agent process, with the same persisted commitment still held.
        again = WheelRig()
        again.backend_link.commitments = rig.backend_link.commitments
        self.assertEqual(again.agent.bench_wheel_status()["held_task_id"], TASK)
        mark = again.mark()
        again.run(3.0)
        self.assertEqual(again.drives(mark), [], "a restart started motion on its own")
        self.refused(again, "TASK_NOT_ACCEPTED")


class TestCancellationStopsAPulse(unittest.TestCase):
    """Each ends a running pulse at once with a confirmed STOP, and nothing resumes it."""

    def cancel_during_pulse(self, cancel, expect_code):
        rig = WheelRig()
        rig.accept()
        rig.pulse()
        rig.run(0.6)
        self.assertTrue(rig.agent.bench.driving)
        cancel(rig)
        run_until_idle(rig)
        result = rig.agent.bench.last_result
        self.assertIsNotNone(result, "the pulse never ended")
        self.assertEqual(result["stop_code"], expect_code, result)
        self.assertEqual(result["outcome"], BenchOutcome.ABORTED.value, result)
        self.assertTrue(result["stop_confirmed"], result)
        self.assertLess(result["ran_s"], 1.5, "stopped only at the deadline")
        self.assertEqual(rig.port.command_names()[-1], "STOP")
        mark = rig.mark()
        rig.run(4.0)
        self.assertEqual(rig.drives(mark), [], "motion resumed after the cancellation")
        return rig, result

    def test_every_terminating_engine_command(self):
        for command in ("RECALL", "WITHDRAW", "ABORT_MISSION"):
            with self.subTest(command=command):
                acked = []
                rig, _ = self.cancel_during_pulse(
                    lambda r: acked.extend(names(r.fire(command, terminating(command)))), "TASK_CANCELLED")
                self.assertEqual(acked, ["COMMAND_ACK"])
                self.assertTrue(rig.backend_link.commitments.get(COMMITMENT).tombstoned)
                with self.assertRaises(BenchRefused):
                    rig.pulse()

    def test_a_cancellation_that_bypasses_the_hook_is_caught_on_the_next_tick(self):
        self.cancel_during_pulse(lambda r: r.backend_link.commitments.update(COMMITMENT, tombstoned=True), "TASK_RELEASED")

    def test_the_backend_link_dropping(self):
        self.cancel_during_pulse(lambda r: r.backend_link._set_status(BackendLinkStatus.DISCONNECTED, "test"),
                                 "BACKEND_LINK_LOST")

    def test_an_operator_stop_command_and_the_bare_stop_event(self):
        self.cancel_during_pulse(lambda r: r.fire("COMMAND", {"commandId": "op-1", "type": "STOP"}), "OPERATOR_STOP")
        self.cancel_during_pulse(lambda r: r.fire("STOP", {"taskId": TASK, "reason": "TASK_CANCELLED"}),
                                 "OPERATOR_STOP")

    def test_a_terminating_command_for_another_commitment_leaves_the_pulse_alone(self):
        rig = WheelRig()
        rig.accept()
        rig.pulse(duration_s=1.0)
        rig.run(0.4)
        self.assertFalse(rig.agent.end_bench_task("c-other", "test"))
        run_until_idle(rig)
        self.assertEqual(rig.agent.bench.last_result["stop_code"], "COMPLETED")

    def test_the_controller_going_silent_latches_the_estop(self):
        rig = WheelRig()
        rig.accept()
        rig.pulse()
        rig.run(0.4)
        rig.alive = False
        run_until_idle(rig)
        self.assertEqual(rig.agent.bench.last_result["outcome"], BenchOutcome.STOP_UNCONFIRMED.value)
        self.assertTrue(rig.agent.safety.estop_engaged)


class TestNoAutonomousMotion(unittest.TestCase):
    def test_no_mission_route_resume_or_return_is_driven(self):
        rig = WheelRig()
        from robotx.communication.engine import offer_to_mission

        with self.assertRaises(MissionRejected) as caught:
            rig.agent.assign_mission(offer_to_mission(parsed_offer()), custody_required=True)
        self.assertIs(caught.exception.reason, MissionRejectReason.BENCH_WHEEL_DEMO)
        for act in (lambda: rig.agent.start_mission([NORTH]),
                    lambda: rig.agent.resume_mission("test"),
                    lambda: rig.agent.return_to_base("test")):
            with self.assertRaises(MissionRefused):
                act()
        mark = rig.mark()
        rig.fire("COMMAND", {"commandId": "op-2", "type": "RETURN"})
        rig.run(2.0)
        self.assertEqual(rig.drives(mark), [])
        self.assertIs(rig.agent.mode, OperatingMode.IDLE)


class TestOutsideTheDemonstration(unittest.TestCase):
    def test_a_task_binding_is_refused_and_the_old_rules_hold(self):
        from tests.unit.test_bench_motion import bench_rig

        rig = bench_rig()
        with self.assertRaises(BenchRefused) as ctx:
            rig.agent.start_bench_motion(speed=0.25, duration_s=1.0, source="test", task_id=TASK)
        self.assertEqual(ctx.exception.code, "TASK_BINDING_UNAVAILABLE")
        self.assertFalse(rig.agent.bench_wheel_status()["enabled"])


class TestRoute(unittest.TestCase):
    def test_task_id_is_validated(self):
        limits = BenchMotionConfig()
        for bad in ("", "   ", 7, None, "x" * 200):
            with self.assertRaises(BenchRejected, msg=bad):
                BenchGate.parse(dict(ATTESTED, task_id=bad), limits)
        self.assertEqual(BenchGate.parse(dict(ATTESTED, task_id=TASK), limits)["task_id"], TASK)

    def test_the_route_binds_the_pulse_and_reports_it(self):
        rig = WheelRig()
        rig.accept()
        gate = BenchGate(gate_config())
        status, body = asyncio.run(acall(rig.agent, gate, "POST", "/test/bench-motion/run", ATTESTED))
        self.assertEqual((status, body["code"]), (409, "TASK_REQUIRED"))
        status, body = asyncio.run(acall(rig.agent, gate, "POST", "/test/bench-motion/run",
                                         dict(ATTESTED, task_id=TASK, duration_s=0.5)))
        self.assertEqual(status, 200, body)
        self.assertEqual(body["running"]["task_id"], TASK)
        run_until_idle(rig)
        status, body = asyncio.run(acall(rig.agent, gate, "GET", "/test/bench-motion"))
        self.assertEqual(body["last_result"]["task_id"], TASK)
        self.assertTrue(body["wheel_demo"]["enabled"])


if __name__ == "__main__":
    unittest.main()
