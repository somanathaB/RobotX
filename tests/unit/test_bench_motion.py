"""The bench motor pulse: preconditions, stop conditions, the guarded route.

SIMULATED: the real `RobotAgent`, `SafetyGate` and `Esp32Link` over a fake port,
answered by `FirmwareModel` (tests.fixtures.esp32_sim), on one fake clock. A pass
shows the Pi starts, judges and stops a pulse correctly against the firmware's
*reported* feedback. It proves nothing about a wheel turning.
"""

from __future__ import annotations

import asyncio
import dataclasses
import hashlib
import unittest

import httpx

from robotx.application import main as api
from robotx.application.bench_gate import BenchGate, BenchGateConfig, BenchRejected
from robotx.control.bench_motion import (
    BenchMotion,
    BenchMotionConfig,
    BenchOutcome,
    BenchPhase,
    BenchRefused,
)
from robotx.communication.engine import OfferVerdict
from robotx.state.robot_state import OperatingMode
from tests.unit.test_engine_offer import parsed_offer
from tests.unit.test_obstacle_recovery import NORTH, TICK_S, Rig

UNITS = 64   # 0.25 * 255, rounded


def bench_rig(**fields):
    rig = Rig()
    rig.agent.bench.clock = rig.clock
    rig.agent.state.set_mode(OperatingMode.IDLE)
    rig.fields.update(fields)
    rig.run(0.6)
    return rig


def start(rig, *, speed=0.25, duration_s=2.0):
    return rig.agent.start_bench_motion(speed=speed, duration_s=duration_s, source="test")


def run_until_idle(rig, seconds=6.0):
    snaps = []
    for _ in range(int(seconds / TICK_S)):
        snaps.append(rig.step())
        if not rig.agent.bench.active:
            break
    return snaps


class TestConfigBounds(unittest.TestCase):
    def test_speed_and_duration_are_bounded(self):
        for bad in (dict(speed=0.1), dict(speed=0.5), dict(duration_s=0.2), dict(duration_s=5.0)):
            with self.assertRaises(ValueError, msg=bad):
                BenchMotionConfig(**bad)
        BenchMotionConfig(speed=0.16, duration_s=0.5)
        BenchMotionConfig(speed=0.35, duration_s=3.0)

    def test_idle_by_default_and_never_moves_on_its_own(self):
        bench = BenchMotion()
        self.assertIs(bench.phase, BenchPhase.IDLE)
        self.assertTrue(bench.intent().is_stop)


class TestAPulseThatRunsToItsEnd(unittest.TestCase):
    def test_forward_pulse_then_a_confirmed_stop(self):
        rig = bench_rig()
        mark = rig.mark()
        start(rig, duration_s=1.0)
        run_until_idle(rig)
        result = rig.agent.bench.last_result
        self.assertEqual(result["outcome"], BenchOutcome.COMPLETED.value, result)
        self.assertTrue(result["stop_confirmed"])
        self.assertEqual(result["sent_units"], [UNITS, UNITS])
        self.assertEqual(result["max_telemetry_applied"], [UNITS, UNITS])
        self.assertTrue(result["drive_acks"])
        self.assertTrue(all(a["result"] == "ACCEPTED" for a in result["drive_acks"]))
        drives = rig.drives(mark)
        self.assertTrue(drives)
        self.assertTrue(all((d["left"], d["right"]) == (UNITS, UNITS) for d in drives), drives)
        self.assertLessEqual(len(drives) * TICK_S, 1.0 + 2 * TICK_S, "ran past its deadline")
        self.assertEqual(rig.port.command_names()[-1], "STOP")
        self.assertIs(rig.agent.mode, OperatingMode.IDLE)
        self.assertFalse(rig.agent.safety.estop_engaged)
        after = rig.mark()
        rig.run(2.0)
        self.assertEqual(rig.drives(after), [], "DRIVE after the pulse ended")
        self.assertIn("physical", result["evidence"].lower())

    def test_the_pulse_is_reported_as_a_fault_and_refuses_offers_while_it_runs(self):
        rig = bench_rig()
        start(rig)
        rig.step()
        self.assertIn("BENCH_MOTION_ACTIVE", [f.code for f in rig.agent.active_faults()])
        decision = rig.agent.assess_offer(parsed_offer())
        self.assertIs(decision.verdict, OfferVerdict.REJECT)
        self.assertEqual(decision.reason, "BENCH_TEST_ACTIVE")
        run_until_idle(rig)
        self.assertNotIn("BENCH_MOTION_ACTIVE", [f.code for f in rig.agent.active_faults()])


class TestRefusals(unittest.TestCase):
    def refused(self, rig, code, **kw):
        with self.assertRaises(BenchRefused) as ctx:
            start(rig, **kw)
        self.assertEqual(ctx.exception.code, code, ctx.exception.detail)
        mark = rig.mark()
        rig.run(0.6)
        self.assertEqual(rig.drives(mark), [], f"{code}: DRIVE after a refusal")

    def test_motion_disabled_on_the_pi(self):
        rig = bench_rig()
        rig.link.cfg = dataclasses.replace(rig.link.cfg, motion_enabled=False)
        self.refused(rig, "MOTION_DISABLED")

    def test_emergency_stop_latched(self):
        rig = bench_rig()
        rig.agent.emergency_stop("test")
        self.refused(rig, "ESTOP_LATCHED")

    def test_not_idle(self):
        rig = bench_rig()
        rig.agent.start_mission([NORTH])
        rig.agent.pause_mission("test")             # held, not driving
        self.refused(rig, "MODE_NOT_IDLE")

    def test_front_sensing_invalid_is_a_controller_fault(self):
        self.refused(bench_rig(front_valid=False), "CONTROLLER_NOT_READY")

    def test_drive_unavailable(self):
        self.refused(bench_rig(motor_drive_available=False), "CONTROLLER_NOT_READY")

    def test_an_obstacle_ahead(self):
        self.refused(bench_rig(front_left_cm=60.0, front_right_cm=60.0), "FRONT_NOT_CLEAR")
        self.refused(bench_rig(front_obstacle=True, forward_blocked=True), "FRONT_NOT_CLEAR")

    def test_perception_unusable(self):
        from robotx.perception.types import PerceptionResult, PerceptionStatus

        rig = bench_rig()
        rig.agent.perception.result = PerceptionResult.unavailable(PerceptionStatus.NO_FRAME)
        rig.run(0.4)
        self.refused(rig, "PERCEPTION_UNUSABLE")

    def test_speed_or_duration_beyond_the_configured_limits(self):
        rig = bench_rig()
        self.refused(rig, "BAD_SPEED", speed=0.3)
        self.refused(rig, "BAD_SPEED", speed=0.1)
        self.refused(rig, "BAD_DURATION", duration_s=2.5)

    def test_a_backend_commitment_held(self):
        rig = bench_rig()

        class Held:
            def held(self):
                return object()

        rig.agent.backend = type("Link", (), {"commitments": Held(), "apply_loss_policy": lambda self: None})()
        self.refused(rig, "COMMITMENT_HELD")

    def test_one_at_a_time_then_a_cooldown(self):
        rig = bench_rig()
        start(rig, duration_s=0.5)
        with self.assertRaises(BenchRefused) as ctx:
            start(rig)
        self.assertEqual(ctx.exception.code, "BENCH_RUNNING")
        run_until_idle(rig)
        with self.assertRaises(BenchRefused) as ctx:
            start(rig)
        self.assertEqual(ctx.exception.code, "COOLDOWN")


class TestStopConditions(unittest.TestCase):
    """Each ends the pulse at once, and nothing restarts it."""

    def run_with(self, change, expect_code, *, outcome=BenchOutcome.ABORTED, ticks_first=2):
        rig = bench_rig()
        start(rig)
        for _ in range(ticks_first):
            rig.step()
        change(rig)
        run_until_idle(rig)
        result = rig.agent.bench.last_result
        self.assertIsNotNone(result, "the pulse never ended")
        self.assertEqual(result["stop_code"], expect_code, result)
        self.assertEqual(result["outcome"], outcome.value, result)
        self.assertLess(result["ran_s"], 2.0, "stopped only at the deadline")
        return rig, result

    def assert_nothing_resumes(self, rig, undo=None):
        if undo is not None:
            undo(rig)
        mark = rig.mark()
        rig.run(4.0)                          # well past any clear hold
        self.assertEqual(rig.drives(mark), [], "motion resumed after the stop condition cleared")

    def test_an_obstacle_appears_and_clearing_it_resumes_nothing(self):
        def block(rig):
            rig.fields.update(front_left_cm=50.0, front_right_cm=50.0)

        rig, result = self.run_with(block, "SAFETY_GATE")
        self.assertIn("obstacle", result["stop_reason"])
        self.assert_nothing_resumes(rig, undo=lambda r: r.fields.update(front_left_cm=150.0, front_right_cm=150.0))

    def test_a_person_in_view_stops_it(self):
        rig, result = self.run_with(lambda r: setattr(r, "_person", True) or r.agent.perception.set_detection(
            label="person", area=2000, cx=50), "SAFETY_GATE")
        self.assertIn("person", result["stop_reason"])

    def test_front_sensing_turning_invalid(self):
        rig, _ = self.run_with(lambda r: r.fields.update(front_valid=False), "CONTROLLER_NOT_READY")
        self.assert_nothing_resumes(rig, undo=lambda r: r.fields.update(front_valid=True))

    def test_a_drive_gated_by_the_esp32(self):
        def gate(rig):
            rig.firmware.gate_reason = "FRONT_OBSTACLE"
        self.run_with(gate, "DRIVE_NOT_ACCEPTED", ticks_first=0)

    def test_a_drive_rejected_by_the_esp32(self):
        def reject(rig):
            rig.firmware.reject_reason = "OUT_OF_RANGE"
        self.run_with(reject, "DRIVE_NOT_ACCEPTED", ticks_first=0)

    def test_applied_output_other_than_sent(self):
        def skew(rig):
            rig.firmware.applied_override = (-UNITS, UNITS)
        self.run_with(skew, "ACK_MISMATCH", ticks_first=0)

    def test_nothing_applied(self):
        def zero(rig):
            rig.firmware.applied_zero = True
        _, result = self.run_with(zero, "ACK_MISMATCH", ticks_first=0)
        self.assertEqual(result["max_telemetry_applied"], [0, 0])

    def test_no_ack_stops_it(self):
        def mute(rig):
            rig.firmware.answer = False
        # Nothing was ever executed: TELEMETRY truthfully reports 0/0 after the STOP.
        _, result = self.run_with(mute, "NO_ACK", ticks_first=0)
        self.assertTrue(result["stop_confirmed"])
        self.assertEqual(result["drive_acks"], [])

    def test_output_still_applied_after_the_stop_latches_the_estop(self):
        def mute_while_driving(rig):
            rig.firmware.answer = False          # executed the DRIVE, now ignores the STOP
        rig, result = self.run_with(mute_while_driving, "CONTROLLER_NOT_READY",
                                    outcome=BenchOutcome.STOP_UNCONFIRMED)
        self.assertFalse(result["stop_confirmed"])
        self.assertTrue(rig.agent.safety.estop_engaged, "an unconfirmed stop must latch the e-stop")

    def test_a_silent_esp32(self):
        def silence(rig):
            rig.alive = False
        rig, result = self.run_with(silence, "CONTROLLER_NOT_READY", outcome=BenchOutcome.STOP_UNCONFIRMED)
        self.assertTrue(rig.agent.safety.estop_engaged)

    def test_emergency_stop_and_operator_stop(self):
        self.run_with(lambda r: r.agent.emergency_stop("operator"), "ESTOP")
        self.run_with(lambda r: r.agent.stop_mission("operator"), "OPERATOR_STOP")


# --- the guarded route ------------------------------------------------------------

TOKEN = "bench-token-for-tests-only-0123456789abcdef"
OPERATOR = "operator-token-for-tests-only-0123456789ab"


def digest(token):
    return hashlib.sha256(token.encode()).hexdigest()


def gate_config(**kw):
    base = dict(enabled=True, token_sha256=digest(TOKEN), operator_token_sha256=digest(OPERATOR),
                motor_cutoff_confirmed=True, api_host="127.0.0.1")
    base.update(kw)
    return BenchGateConfig(**base)


async def acall(agent, gate, method, path, payload=None, *, token=TOKEN, headers=None):
    api.state.agent, api.state.bench = agent, gate
    try:
        transport = httpx.ASGITransport(app=api.app, client=("127.0.0.1", 50000))
        async with httpx.AsyncClient(transport=transport, base_url="http://127.0.0.1:8000") as c:
            h = {"Authorization": f"Bearer {token}"} if token else {}
            h.update(headers or {})
            r = await c.request(method, path, json=payload, headers=h)
    finally:
        api.state.agent, api.state.bench = None, None
    return r.status_code, r.json()


ATTESTED = {"wheels_off_ground": True, "motor_cutoff_attended": True}


class TestGateConfiguration(unittest.TestCase):
    def test_every_missing_prerequisite_refuses_everything(self):
        cases = {
            "no cutoff declared": dict(motor_cutoff_confirmed=False),
            "not loopback": dict(api_host="0.0.0.0"),
            "no bench token": dict(token_sha256=None),
            "no operator token": dict(operator_token_sha256=None),
            "token reused": dict(operator_token_sha256=digest(TOKEN)),
            "injection token reused": dict(other_token_digests=(digest(TOKEN),)),
        }
        for name, kw in cases.items():
            self.assertIsNotNone(gate_config(**kw).problem(), name)
        self.assertIsNone(gate_config().problem())
        self.assertIn("CUTOFF", gate_config(motor_cutoff_confirmed=False).problem())

    def test_the_body_must_restate_both_attestations(self):
        limits = BenchMotionConfig()
        for body in ({}, {"wheels_off_ground": True}, {"motor_cutoff_attended": True},
                     dict(ATTESTED, wheels_off_ground="yes"), dict(ATTESTED, left=255), dict(ATTESTED, speed=True)):
            with self.assertRaises(BenchRejected, msg=body):
                BenchGate.parse(body, limits)
        self.assertEqual(BenchGate.parse(dict(ATTESTED), limits), {"speed": 0.25, "duration_s": 2.0})


class TestRoute(unittest.TestCase):
    def test_disabled_is_an_unknown_path(self):
        rig = bench_rig()
        for gate in (None, BenchGate(gate_config(enabled=False))):
            status, _ = asyncio.run(acall(rig.agent, gate, "POST", "/test/bench-motion/run", ATTESTED))
            self.assertEqual(status, 404)
            status, _ = asyncio.run(acall(rig.agent, gate, "GET", "/test/bench-motion"))
            self.assertEqual(status, 404)

    def test_refusals_before_the_agent(self):
        rig = bench_rig()
        cases = [
            (BenchGate(gate_config(motor_cutoff_confirmed=False)), {}, TOKEN, 503, "NOT_CONFIGURED"),
            (BenchGate(gate_config()), {}, None, 401, "UNAUTHORIZED"),
            (BenchGate(gate_config()), {}, "wrong-token-of-sufficient-length-000000000", 401, "UNAUTHORIZED"),
            (BenchGate(gate_config()), {"Origin": "http://evil.example"}, TOKEN, 403, "ORIGIN_PRESENT"),
        ]
        for gate, headers, token, code, reason in cases:
            status, body = asyncio.run(acall(rig.agent, gate, "POST", "/test/bench-motion/run", ATTESTED,
                                             token=token, headers=headers))
            self.assertEqual((status, body["code"]), (code, reason))
        status, body = asyncio.run(acall(rig.agent, BenchGate(gate_config()), "POST", "/test/bench-motion/run", {}))
        self.assertEqual((status, body["code"]), (422, "ATTESTATION_MISSING"))
        self.assertFalse(rig.agent.bench.active)

    def test_the_agent_refuses_with_the_precondition(self):
        rig = bench_rig()
        rig.link.cfg = dataclasses.replace(rig.link.cfg, motion_enabled=False)
        status, body = asyncio.run(acall(rig.agent, BenchGate(gate_config()), "POST", "/test/bench-motion/run", ATTESTED))
        self.assertEqual((status, body["code"]), (409, "MOTION_DISABLED"))

    def test_an_authorized_attested_request_starts_one_pulse(self):
        rig = bench_rig()
        gate = BenchGate(gate_config())
        status, body = asyncio.run(acall(rig.agent, gate, "POST", "/test/bench-motion/run",
                                         dict(ATTESTED, duration_s=0.5)))
        self.assertEqual(status, 200, body)
        self.assertEqual(body["phase"], "DRIVING")
        run_until_idle(rig)
        status, body = asyncio.run(acall(rig.agent, gate, "GET", "/test/bench-motion"))
        self.assertEqual(status, 200)
        self.assertEqual(body["last_result"]["outcome"], "COMPLETED")


if __name__ == "__main__":
    unittest.main()
