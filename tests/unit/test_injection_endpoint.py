"""The opt-in test-only mission ingress: `POST /test/mission/inject`.

Requests go through the real FastAPI route handlers (httpx's in-process ASGI
transport) into `robotx.application.injection.InjectionGate`.

Test doubles, labelled
----------------------
- `Rig` (from `test_obstacle_recovery`): the REAL `RobotAgent`, MissionManager,
  navigator, decision layer, safety gate and `Esp32Link`, over a SIMULATED
  serial port on a fake clock. What matters is what reaches that port.
- GPS fixes and perception results are SYNTHETIC.
- `FakeSio`: a SIMULATED backend socket under a real `BackendLink`.

Nothing here opens a serial port, a socket or a TCP listener. No test is
evidence about the physical Rover or the RobotX backend.
"""

from __future__ import annotations

import asyncio
import dataclasses
import json
import logging
import os
import secrets
import tempfile
import unittest
import unittest.mock
from pathlib import Path
from types import SimpleNamespace

import httpx

import robotx.application.main as api
from robotx.application.injection import (
    AUTH_LOCKOUT_S,
    MAX_AUTH_FAILURES,
    InjectionConfig,
    InjectionGate,
    token_digest,
)
from robotx.communication.engine import OfferVerdict
from robotx.config.settings import Settings
from robotx.hardware.gps import GpsReading, GPSStatus
from robotx.mission.mission import MissionStatus
from robotx.state.robot_state import OperatingMode
from tests.fixtures.esp32 import FakeClock
from tests.fixtures.task_assign import leg, offset
from tests.unit.test_agent import HERE
from tests.unit.test_backend_link import connect_and_auth, make_link
from tests.unit.test_obstacle_recovery import Rig


ROBOT = "test-rover"
PICKUP = offset(HERE, north_m=40.0)
DROP = offset(HERE, north_m=80.0)
TOKEN = secrets.token_hex(32)
OPERATOR_TOKEN = secrets.token_hex(32)
INJECT = "/test/mission/inject"


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


def pt(p):
    return {"lat": p[0], "lon": p[1]}


def body(**overrides):
    raw = {
        "schema_version": 1,
        "test_run_id": "unit-run",
        "task_id": "TEST-INJECTED-0001",
        "robot_id": ROBOT,
        "region": {"center": pt(HERE), "radius_m": 300},
        "pickup": pt(PICKUP),
        "drop": pt(DROP),
        "path_to_pickup": [pt(p) for p in leg(HERE, PICKUP, points=5)],
        "path_to_drop": [pt(p) for p in leg(PICKUP, DROP, points=5)],
    }
    raw.update(overrides)
    return raw


async def acall(agent, gate, method, path, payload=None, *, token=TOKEN, headers=None,
                client=("127.0.0.1", 50000), host="127.0.0.1:8000", raw_body=None):
    api.state.agent, api.state.injection = agent, gate
    try:
        transport = httpx.ASGITransport(app=api.app, client=client)
        async with httpx.AsyncClient(transport=transport, base_url=f"http://{host}") as c:
            h = {"Authorization": f"Bearer {token}"} if token else {}
            h.update(headers or {})
            if raw_body is not None:
                r = await c.request(method, path, content=raw_body, headers=h)
            else:
                r = await c.request(method, path, json=payload, headers=h)
    finally:
        api.state.agent = api.state.injection = None
    return r.status_code, r.json()


def call(*args, **kwargs):
    return asyncio.run(acall(*args, **kwargs))


def ready_rig(*, custody="operator"):
    """The real agent, ready by its own rules: motor link, fix, custody."""

    rig = Rig()
    rig.agent.settings = dataclasses.replace(rig.agent.settings, custody_confirmation=custody)
    rig.run(0.6)
    return rig


class GateCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.log_path = Path(self.tmp.name) / "state" / "test_injections.jsonl"

    def config(self, **overrides):
        values = dict(enabled=True, token_sha256=token_digest(TOKEN), log_path=str(self.log_path),
                      api_host="127.0.0.1", robot_id=ROBOT, operator_token_sha256=token_digest(OPERATOR_TOKEN))
        values.update(overrides)
        return InjectionConfig(**values)

    def gate(self, **overrides):
        return InjectionGate(self.config(**overrides))

    def records(self):
        if not self.log_path.exists():
            return []
        return [json.loads(line) for line in self.log_path.read_text().splitlines() if line.strip()]

    def inject(self, rig, gate=None, **kwargs):
        payload = kwargs.pop("payload", None) or body()
        return call(rig.agent, gate or self.gate(), "POST", INJECT, payload, **kwargs)


# --- disabled by default ------------------------------------------------------------------


class TestDisabledByDefault(GateCase):
    def test_settings_default_to_disabled_with_no_token_and_no_log(self):
        for s in (Settings(), Settings.from_env({})):
            self.assertFalse(s.test_injection_enabled)
            self.assertIsNone(s.test_injection_token_sha256)
            self.assertIsNone(s.test_injection_log_path)
        self.assertFalse(InjectionGate.from_settings(Settings.from_env({})).enabled)

    def test_a_disabled_gate_is_indistinguishable_from_no_route(self):
        rig = ready_rig()
        gate = InjectionGate.from_settings(Settings.from_env({"ROBOTX_ROBOT_ID": ROBOT}))
        self.assertEqual(call(rig.agent, gate, "POST", INJECT, body()), (404, {"detail": "Not Found"}))
        self.assertEqual(call(rig.agent, gate, "GET", "/test/injection"), (404, {"detail": "Not Found"}))
        self.assertEqual(call(rig.agent, None, "POST", INJECT, body())[0], 404)
        self.assertIsNone(rig.agent.missions.active)
        self.assertFalse(self.log_path.exists())

    def test_a_disabled_route_is_404_for_every_method(self):
        rig = ready_rig()
        gate = self.gate(enabled=False)
        for method, path in (("GET", INJECT), ("PUT", INJECT), ("DELETE", INJECT), ("OPTIONS", INJECT),
                             ("POST", "/test/injection"), ("HEAD", "/test/injection")):
            with self.subTest(method=method, path=path):
                api.state.agent, api.state.injection = rig.agent, gate
                try:
                    async def go():
                        transport = httpx.ASGITransport(app=api.app, client=("127.0.0.1", 1))
                        async with httpx.AsyncClient(transport=transport, base_url="http://127.0.0.1:8000") as c:
                            return (await c.request(method, path)).status_code
                    self.assertEqual(asyncio.run(go()), 404)
                finally:
                    api.state.agent = api.state.injection = None
        # Enabled, the wrong method is an ordinary 405 and reaches nothing.
        self.assertEqual(call(rig.agent, self.gate(), "GET", INJECT)[0], 405)
        self.assertIsNone(rig.agent.missions.active)

    def test_the_example_configuration_leaves_injection_off(self):
        repo = Path(__file__).resolve().parents[2]
        env = {}
        for line in (repo / ".env.example").read_text(encoding="utf-8").splitlines():
            if line.strip() and not line.lstrip().startswith("#") and "=" in line:
                key, _, value = line.partition("=")
                env[key.strip()] = value.strip()
        self.assertFalse(any(k.startswith("ROBOTX_TEST_INJECTION") for k in env), "an example line is live")
        settings = Settings.from_env(env)
        self.assertFalse(settings.test_injection_enabled)
        self.assertFalse(InjectionGate.from_settings(settings).enabled)

    def test_disabled_even_when_a_token_digest_and_log_are_configured(self):
        rig = ready_rig()
        gate = self.gate(enabled=False)
        self.assertEqual(self.inject(rig, gate)[0], 404)
        self.assertIsNone(rig.agent.missions.active)

    def test_settings_parse_the_opt_in_and_hide_the_digest(self):
        digest = token_digest(TOKEN)
        s = Settings.from_env({"ROBOTX_TEST_INJECTION_ENABLED": "1",
                               "ROBOTX_TEST_INJECTION_TOKEN_SHA256": digest,
                               "ROBOTX_TEST_INJECTION_LOG_PATH": "/var/lib/robotx/test_injections.jsonl"})
        self.assertTrue(s.test_injection_enabled)
        summary = s.public_summary()
        self.assertEqual(summary["test_injection_token_sha256"], "SET")
        self.assertNotIn(digest, json.dumps(summary))
        self.assertEqual(Settings.from_env({}).public_summary()["test_injection_token_sha256"], "UNSET")


# --- loopback only -------------------------------------------------------------------------


class TestLoopbackOnly(GateCase):
    def test_a_non_loopback_api_host_refuses_everything(self):
        rig = ready_rig()
        for host in ("0.0.0.0", "192.168.1.40", "localhost", "", "::"):
            with self.subTest(host=host):
                gate = self.gate(api_host=host)
                self.assertFalse(gate.available)
                status, reply = self.inject(rig, gate)
                self.assertEqual((status, reply["code"]), (503, "NOT_CONFIGURED"))
        self.assertIsNone(rig.agent.missions.active)

    def test_loopback_api_hosts_are_accepted(self):
        for host in ("127.0.0.1", "::1"):
            self.assertIsNone(self.config(api_host=host).problem(), host)

    def test_a_request_from_another_machine_is_refused(self):
        rig = ready_rig()
        status, reply = self.inject(rig, client=("192.168.1.20", 40000))
        self.assertEqual((status, reply["code"]), (403, "NOT_LOOPBACK"))
        self.assertIsNone(rig.agent.missions.active)

    def test_a_connection_accepted_on_a_non_loopback_socket_is_refused(self):
        # As if the process had been launched bound to every interface, whatever
        # ROBOTX_API_HOST says: the accepting socket's own address decides.
        rig = ready_rig()
        status, reply = self.inject(rig, host="192.168.1.40:8000", headers={"Host": "127.0.0.1:8000"})
        self.assertEqual((status, reply["code"]), (403, "NOT_LOOPBACK"))
        self.assertIsNone(rig.agent.missions.active)

    def test_dns_rebinding_and_browser_requests_are_refused(self):
        rig = ready_rig()
        for host in ("robot.example.com:8000", "127.0.0.1.evil.example:8000", "", "127.0.0.1:8000:x"):
            with self.subTest(host=host):
                status, reply = self.inject(rig, headers={"Host": host})
                self.assertEqual((status, reply["code"]), (403, "HOST_NOT_LOOPBACK"))
        status, reply = self.inject(rig, headers={"Origin": "http://127.0.0.1:8000"})
        self.assertEqual((status, reply["code"]), (403, "ORIGIN_PRESENT"))
        self.assertIsNone(rig.agent.missions.active)
        self.assertEqual(self.records(), [])


# --- authorization ---------------------------------------------------------------------------


class TestAuthorization(GateCase):
    def test_missing_wrong_or_short_tokens_are_refused_before_anything_is_read(self):
        rig = ready_rig()
        gate = self.gate()
        for token, headers in ((None, None), ("wrong" * 10, None), (TOKEN[:20], None),
                               (None, {"Authorization": f"Basic {TOKEN}"}),
                               (None, {"Authorization": TOKEN})):
            with self.subTest(token=token, headers=headers):
                status, reply = self.inject(rig, gate, token=token, headers=headers)
                self.assertEqual((status, reply["code"]), (401, "UNAUTHORIZED"))
                gate._locked_until = 0.0
        self.assertIsNone(rig.agent.missions.active)
        self.assertEqual(self.records(), [])

    def test_no_configured_digest_refuses_every_request(self):
        rig = ready_rig()
        for digest in (None, "", "abc", token_digest(TOKEN).upper(), " " * 64):
            with self.subTest(digest=digest):
                status, reply = self.inject(rig, self.gate(token_sha256=digest))
                self.assertEqual((status, reply["code"]), (503, "NOT_CONFIGURED"))
        # The raw token pasted where its digest belongs is well-formed hex, and authenticates nobody.
        status, reply = self.inject(rig, self.gate(token_sha256=TOKEN))
        self.assertEqual((status, reply["code"]), (401, "UNAUTHORIZED"))
        self.assertIsNone(rig.agent.missions.active)

    def test_repeated_failures_lock_the_route_even_for_the_right_token(self):
        rig = ready_rig()
        clock = FakeClock(1000.0)
        gate = InjectionGate(self.config(), clock=clock)
        for _ in range(MAX_AUTH_FAILURES):
            self.assertEqual(self.inject(rig, gate, token="x" * 40)[0], 401)
        status, reply = self.inject(rig, gate)
        self.assertEqual((status, reply["code"]), (429, "LOCKED_OUT"))
        self.assertTrue(gate.status()["locked_out"])
        self.assertIsNone(rig.agent.missions.active)
        clock.advance(AUTH_LOCKOUT_S + 1)
        self.assertEqual(self.inject(rig, gate)[0], 200)

    def test_the_token_never_appears_in_the_log_response_or_status(self):
        rig = ready_rig()
        gate = self.gate()
        with self.assertLogs("robotx.application.injection", level="INFO") as logs:
            status, reply = self.inject(rig, gate)
            self.inject(rig, gate, token="y" * 40)
        self.assertEqual(status, 200)
        for text in (self.log_path.read_text(), json.dumps(reply), json.dumps(gate.status()),
                     "\n".join(logs.output)):
            self.assertNotIn(TOKEN, text)
            self.assertNotIn(token_digest(TOKEN), text)


# --- fixtures -------------------------------------------------------------------------------


class TestRequestValidation(GateCase):
    def test_a_valid_request_is_accepted_by_the_real_agent(self):
        rig = ready_rig()
        status, reply = self.inject(rig)
        self.assertEqual((status, reply["outcome"]), (200, "ACCEPTED"), reply)
        self.assertEqual(reply["mission"]["task_id"], "TEST-INJECTED-0001")
        self.assertTrue(reply["mission"]["custody_required"])
        self.assertEqual(reply["mission"]["status"], "TO_PICKUP")

    def test_invalid_requests_never_reach_the_agent(self):
        rig = ready_rig()
        seen = []
        rig.agent.assess_offer = lambda offer: seen.append(offer)
        far = offset(HERE, north_m=400.0)
        cases = [
            (body(task_id="T-123"), "TASK_ID_PROVENANCE"),
            (body(robot_id="RBT-9999"), "WRONG_ROBOT"),
            ({k: v for k, v in body().items() if k != "drop"}, "MISSING_FIELD"),
            (body(signature="ab", commitmentId="c-1"), "UNKNOWN_FIELD"),
            (body(pickup={"lat": float("nan"), "lon": 0.0}), "INVALID_COORDINATE"),
            (body(pickup=[51.5, -0.1]), "MALFORMED"),
            (body(pickup={"lat": 91.0, "lon": 0.0}), "INVALID_COORDINATE"),
            (body(drop={"lat": "51.5", "lon": -0.1}), "INVALID_COORDINATE"),
            (body(path_to_pickup=[pt(HERE)]), "EMPTY_ROUTE"),
            (body(path_to_drop=[pt(PICKUP)] * 2001), "ROUTE_TOO_LONG"),
            (body(drop=pt(far)), "OUT_OF_REGION"),
            (body(region={"center": pt(HERE), "radius_m": 5000}), "REGION"),
            (body(schema_version=2), "SCHEMA_VERSION"),
            ([1, 2, 3], "MALFORMED"),
        ]
        for payload, code in cases:
            with self.subTest(code=code):
                status, reply = call(rig.agent, self.gate(), "POST", INJECT, payload)
                self.assertEqual((status, reply["code"]), (422, code), reply)
        self.assertEqual(seen, [])
        self.assertIsNone(rig.agent.missions.active)
        self.assertEqual(self.records(), [])

    def test_numbers_too_large_for_a_float_are_refused_not_a_crash(self):
        rig = ready_rig()
        for payload, code in ((body(pickup={"lat": 10 ** 400, "lon": 0}), "INVALID_COORDINATE"),
                              (body(path_to_drop=[pt(PICKUP), {"lat": 1, "lon": -(10 ** 400)}]), "INVALID_COORDINATE"),
                              (body(region={"center": pt(HERE), "radius_m": 10 ** 400}), "REGION")):
            with self.subTest(code=code):
                status, reply = self.inject(rig, raw_body=json.dumps(payload).encode())
                self.assertEqual((status, reply["code"]), (422, code))
        self.assertIsNone(rig.agent.missions.active)
        self.assertEqual(self.records(), [])

    def test_a_body_that_is_not_json_or_too_large_is_refused(self):
        rig = ready_rig()
        self.assertEqual(self.inject(rig, raw_body=b"{not json")[1]["code"], "MALFORMED")
        self.assertEqual(self.inject(rig, raw_body=b" " * (600 * 1024))[0], 413)

    def test_the_rover_itself_must_be_inside_the_region(self):
        rig = ready_rig()
        rig.agent.gps.set_fix(*offset(HERE, north_m=-400.0))
        rig.run(0.4)
        status, reply = self.inject(rig)
        self.assertEqual((status, reply["code"]), (409, "ROVER_OUT_OF_REGION"))
        self.assertIsNone(rig.agent.missions.active)


# --- duplicates, conflicts ---------------------------------------------------------------------


class TestDuplicatesAndConflicts(GateCase):
    def test_a_task_id_is_injected_once_even_across_a_restart(self):
        rig = ready_rig()
        self.assertEqual(self.inject(rig)[0], 200)
        rig.agent.stop_mission("unit")
        rig.run(0.4)
        status, reply = self.inject(rig)
        self.assertEqual((status, reply["code"]), (409, "DUPLICATE_TASK"))
        # A restarted agent builds a new gate from the same log.
        status, reply = self.inject(ready_rig(), self.gate())
        self.assertEqual((status, reply["code"]), (409, "DUPLICATE_TASK"))

    def test_a_task_id_the_agent_rejected_is_spent_too(self):
        rig = ready_rig(custody="none")
        status, reply = self.inject(rig)
        self.assertEqual((status, reply["reason"]), (409, "NO_CUSTODY_SENSING"))
        self.assertEqual(self.inject(rig)[1]["code"], "DUPLICATE_TASK")

    def test_a_mission_in_progress_is_never_replaced(self):
        rig = ready_rig()
        self.assertEqual(self.inject(rig)[0], 200)
        status, reply = self.inject(rig, payload=body(task_id="TEST-INJECTED-0002"))
        self.assertEqual((status, reply["code"]), (409, "MISSION_IN_PROGRESS"))
        self.assertEqual(rig.agent.missions.active.task_id, "TEST-INJECTED-0001")
        rig.agent.pause_mission("unit")
        status, reply = self.inject(rig, payload=body(task_id="TEST-INJECTED-0003"))
        self.assertEqual((status, reply["code"]), (409, "MISSION_IN_PROGRESS"))

    def test_a_bench_route_in_progress_is_a_conflict(self):
        rig = Rig()
        rig.agent.settings = dataclasses.replace(rig.agent.settings, custody_confirmation="operator")
        rig.start()
        status, reply = self.inject(rig)
        self.assertEqual((status, reply["code"]), (409, "MISSION_IN_PROGRESS"))

    def test_a_held_robotx_commitment_is_never_replaced(self):
        rig = ready_rig()
        rig.agent.backend = SimpleNamespace(commitments=SimpleNamespace(held=lambda: object()))
        status, reply = self.inject(rig)
        self.assertEqual((status, reply["code"]), (409, "BACKEND_COMMITMENT_HELD"))
        self.assertIsNone(rig.agent.missions.active)

    def test_a_real_offer_arriving_during_a_test_mission_is_refused_honestly(self):
        rig = ready_rig()
        self.assertEqual(self.inject(rig)[0], 200)
        from tests.unit.test_engine_offer import parsed_offer

        decision = rig.agent.assess_offer(parsed_offer())
        self.assertIs(decision.verdict, OfferVerdict.REJECT)
        self.assertEqual(decision.reason, "MISSION_IN_PROGRESS")


# --- provenance -----------------------------------------------------------------------------------


class TestProvenance(GateCase):
    def test_every_record_and_reply_carries_test_provenance(self):
        rig = ready_rig()
        status, reply = self.inject(rig)
        self.assertEqual(status, 200)
        records = self.records()
        self.assertEqual([r["event"] for r in records], ["ATTEMPTED", "ACCEPTED"])
        for r in records + [reply]:
            self.assertEqual(r["provenance"]["source"], "test_injected")
            self.assertIs(r["provenance"]["backend_assigned"], False)
            self.assertIn("not by RobotX", r["provenance"]["reason"])
            self.assertEqual((r["task_id"], r["test_run_id"]), ("TEST-INJECTED-0001", "unit-run"))
        self.assertEqual(records[0]["request_summary"]["pickup"], pt(PICKUP))
        self.assertEqual(oct(os.stat(self.log_path).st_mode & 0o777), "0o600")
        status_view = call(rig.agent, self.gate(), "GET", "/test/injection")[1]
        self.assertEqual(status_view["injected_task_ids"], 1)
        # Unauthenticated: whether the ingress is usable, never which tasks ran.
        self.assertNotIn("TEST-INJECTED-0001", json.dumps(status_view))
        self.assertNotIn("unit-run", json.dumps(status_view))

    def test_a_rejection_is_recorded_with_the_agents_reason(self):
        rig = ready_rig()
        rig.agent.gps.reading = GpsReading(status=GPSStatus.NO_FIX)
        rig.run(0.6)
        status, reply = self.inject(rig)
        self.assertEqual((status, reply["outcome"], reply["reason"]), (409, "REJECTED", "NO_POSITION_FIX"))
        self.assertEqual(self.records()[-1]["reason"], "NO_POSITION_FIX")


# --- the agent decides; every gate still holds --------------------------------------------------


class TestRealGatesHold(GateCase):
    def test_acceptance_is_the_agents_assess_then_assign_with_custody(self):
        rig = ready_rig()
        calls = []
        real_assess, real_assign = rig.agent.assess_offer, rig.agent.assign_mission
        rig.agent.assess_offer = lambda o: calls.append(("assess", type(o).__name__)) or real_assess(o)
        rig.agent.assign_mission = lambda m, **k: calls.append(("assign", k)) or real_assign(m, **k)
        before = len(rig.port.commands())
        self.assertEqual(self.inject(rig)[0], 200)
        self.assertEqual(calls, [("assess", "InjectedAssignment"), ("assign", {"custody_required": True})])
        self.assertEqual(len(rig.port.commands()), before, "the request itself wrote to the ESP32")
        self.assertIs(rig.agent.state.mode, OperatingMode.AUTO)
        mark = rig.mark()
        rig.run(1.0)
        self.assertTrue(rig.drives(mark), "the agent loop never drove the accepted mission")

    def test_readiness_refusals_come_from_the_agent(self):
        cases = [
            ("no fix", lambda r: setattr(r.agent.gps, "reading", GpsReading(status=GPSStatus.NO_FIX)),
             "NO_POSITION_FIX"),
            ("front invalid", lambda r: r.fields.update(front_valid=False), "NO_MOTOR_LINK"),
            ("drive unavailable", lambda r: r.fields.update(motor_drive_available=False, state="DRIVE_UNAVAILABLE",
                                                            block_reason="MOTOR_PWM_UNAVAILABLE"), "NO_MOTOR_LINK"),
            ("esp32 silent", lambda r: setattr(r, "alive", False), "NO_MOTOR_LINK"),
            ("estop", lambda r: r.agent.emergency_stop("unit"), "ESTOP_LATCHED"),
        ]
        for name, breaks, reason in cases:
            with self.subTest(name=name):
                rig = ready_rig()
                breaks(rig)
                rig.run(2.0)
                mark = rig.mark()
                status, reply = call(rig.agent, self.gate(), "POST", INJECT,
                                     body(task_id=f"TEST-INJECTED-{name.replace(' ', '-')}"))
                self.assertEqual((status, reply["reason"]), (409, reason), reply)
                rig.run(2.0)
                self.assertEqual(rig.drives(mark), [])

    def test_motion_disabled_on_the_pi_is_no_motor_link(self):
        from tests.unit.test_esp32_agent import agent_with_link

        agent, link, port, clock = agent_with_link(motion=False, drive_available=True,
                                                   ROBOTX_CUSTODY_CONFIRMATION="operator")
        agent.gps = Rig().agent.gps
        agent.tick()
        status, reply = call(agent, self.gate(), "POST", INJECT, body())
        self.assertEqual((status, reply["reason"]), (409, "NO_MOTOR_LINK"))
        self.assertNotIn("DRIVE", port.command_names())

    def test_after_acceptance_each_fault_stops_driving(self):
        cases = [
            ("stale gps", lambda r: setattr(r.agent.gps, "reading", GpsReading(status=GPSStatus.STALE))),
            ("front invalid", lambda r: r.fields.update(front_valid=False)),
            ("obstacle 60cm", lambda r: r.fields.update(front_left_cm=60.0)),
            ("esp32 silent", lambda r: setattr(r, "alive", False)),
            ("estop", lambda r: r.agent.emergency_stop("unit")),
            ("controller safety stop", lambda r: r.fields.update(safety_stop=True)),
        ]
        for name, fault in cases:
            with self.subTest(name=name):
                rig = ready_rig()
                self.assertEqual(self.inject(rig, payload=body(task_id=f"TEST-INJECTED-{name.replace(' ', '-')}"))[0],
                                 200)
                rig.run(1.0)
                fault(rig)
                rig.run(1.5)
                mark = rig.mark()
                rig.run(3.0)
                self.assertEqual(rig.drives(mark), [], f"DRIVE after {name}")

    def test_clearing_an_estop_does_not_resume_a_test_mission(self):
        rig = ready_rig()
        self.assertEqual(self.inject(rig)[0], 200)
        rig.run(1.0)
        rig.agent.emergency_stop("unit")
        rig.run(0.4)
        mark = rig.mark()
        rig.agent.clear_emergency_stop("unit")
        rig.run(3.0)
        self.assertEqual(rig.drives(mark), [])

    def test_injection_changes_no_setting_or_threshold(self):
        rig = ready_rig()
        settings, safety_cfg, link_cfg = rig.agent.settings, rig.agent.safety.cfg, rig.link.cfg
        self.assertEqual(self.inject(rig)[0], 200)
        rig.run(1.0)
        self.assertIs(rig.agent.settings, settings)
        self.assertEqual(rig.agent.safety.cfg, safety_cfg)
        self.assertEqual(rig.agent.safety.cfg.stop_distance_cm, 90.0)
        self.assertEqual(rig.link.cfg, link_cfg)


# --- fail closed --------------------------------------------------------------------------------


class TestFailClosed(GateCase):
    def test_injection_requires_operator_access_with_a_different_token(self):
        rig = ready_rig()
        for digest, why in ((None, "must be set"), ("", "must be set"), ("abc", "must be set"),
                            (token_digest(TOKEN), "must differ")):
            with self.subTest(digest=digest):
                gate = self.gate(operator_token_sha256=digest)
                status, reply = self.inject(rig, gate)
                self.assertEqual((status, reply["code"]), (503, "NOT_CONFIGURED"))
                self.assertIn(why, reply["detail"])
        self.assertIsNone(rig.agent.missions.active)

    def test_no_log_path_or_a_relative_one_refuses(self):
        rig = ready_rig()
        for path in (None, "", "test_injections.jsonl"):
            with self.subTest(path=path):
                status, reply = self.inject(rig, self.gate(log_path=path))
                self.assertEqual((status, reply["code"]), (503, "NOT_CONFIGURED"))

    def test_a_corrupt_log_refuses_rather_than_forgetting(self):
        self.log_path.parent.mkdir(parents=True)
        self.log_path.write_text('{"event": "ATTEMPTED", "task_id": "TEST-INJECTED-0001"}\n{not json\n')
        status, reply = self.inject(ready_rig())
        self.assertEqual((status, reply["code"]), (503, "NOT_CONFIGURED"))
        self.assertIn("cannot be read", reply["detail"])

    def test_an_unwritable_log_injects_nothing(self):
        rig = ready_rig()
        gate = self.gate()
        gate._append = lambda record: False
        status, reply = self.inject(rig, gate)
        self.assertEqual((status, reply["code"]), (503, "PROVENANCE_NOT_RECORDED"))
        self.assertIsNone(rig.agent.missions.active)
        self.assertIs(rig.agent.state.mode, OperatingMode.IDLE)

    def test_an_unrecordable_acceptance_is_stopped(self):
        rig = ready_rig()
        gate = self.gate()
        real = gate._append
        gate._append = lambda record: real(record) if record["event"] == "ATTEMPTED" else False
        status, reply = self.inject(rig, gate)
        self.assertEqual((status, reply["code"]), (503, "PROVENANCE_NOT_RECORDED"))
        rig.run(0.6)
        self.assertIsNot(rig.agent.state.mode, OperatingMode.AUTO)
        mark = rig.mark()
        rig.run(2.0)
        self.assertEqual(rig.drives(mark), [])

    def test_an_unreadable_log_location_never_stops_the_agent_starting(self):
        # Under systemd ProtectHome=yes a path in /home cannot even be stat'ed.
        locked = Path(self.tmp.name) / "locked"
        locked.mkdir()
        os.chmod(locked, 0)
        self.addCleanup(os.chmod, locked, 0o700)
        with self.assertLogs("robotx.application.injection", level="ERROR"):
            gate = self.gate(log_path=str(locked / "test_injections.jsonl"))
        self.assertFalse(gate.available)
        status, reply = self.inject(ready_rig(), gate)
        self.assertEqual((status, reply["code"]), (503, "NOT_CONFIGURED"))

    def test_the_app_starts_with_injection_misconfigured(self):
        locked = Path(self.tmp.name) / "locked"
        locked.mkdir()
        os.chmod(locked, 0)
        self.addCleanup(os.chmod, locked, 0o700)
        settings = Settings.from_env({"ROBOTX_ROBOT_ID": ROBOT, "ROBOTX_CAMERA_ENABLED": "0",
                                      "ROBOTX_PERCEPTION_ENABLED": "0", "ROBOTX_GPS_ENABLED": "0",
                                      "ROBOTX_TEST_INJECTION_ENABLED": "1",
                                      "ROBOTX_TEST_INJECTION_TOKEN_SHA256": token_digest(TOKEN),
                                      "ROBOTX_TEST_INJECTION_LOG_PATH": str(locked / "x.jsonl"),
                                      "ROBOTX_OPERATOR_TOKEN_SHA256": token_digest(OPERATOR_TOKEN)})

        async def start_and_stop():
            with unittest.mock.patch.object(api, "SETTINGS", settings):
                async with api.lifespan(api.app):
                    return api.state.agent is not None, api.state.injection.available

        with self.assertLogs("robotx.application.injection", level="ERROR"):
            self.assertEqual(asyncio.run(start_and_stop()), (True, False))

    def test_a_failing_stop_after_an_unrecordable_acceptance_latches_the_estop(self):
        rig = ready_rig()
        gate = self.gate()
        real = gate._append
        gate._append = lambda record: real(record) if record["event"] == "ATTEMPTED" else False

        def broken_stop(reason):
            raise RuntimeError("bug")

        rig.agent.stop_mission = broken_stop
        with self.assertLogs("robotx.application.injection", level="ERROR"):
            status, reply = self.inject(rig, gate)
        self.assertEqual((status, reply["code"]), (503, "PROVENANCE_NOT_RECORDED"))
        self.assertTrue(rig.agent.emergency_stopped)
        mark = rig.mark()
        rig.run(2.0)
        self.assertEqual(rig.drives(mark), [])

    def test_an_assign_that_fails_part_way_holds_nothing(self):
        rig = ready_rig()
        real_assign = rig.agent.assign_mission

        def half(mission, **kwargs):
            real_assign(mission, **kwargs)
            raise RuntimeError("bug after the mission was taken")

        rig.agent.assign_mission = half
        with self.assertLogs("robotx.application.injection", level="ERROR"):
            status, reply = self.inject(rig)
        self.assertEqual((status, reply["reason"]), (409, "AGENT_ERROR"))
        rig.run(0.6)
        self.assertIsNot(rig.agent.state.mode, OperatingMode.AUTO)
        self.assertFalse(rig.agent.missions.active.is_active)
        mark = rig.mark()
        rig.run(2.0)
        self.assertEqual(rig.drives(mark), [])

    def test_an_agent_error_declines(self):
        rig = ready_rig()

        def broken(_offer):
            raise RuntimeError("bug")

        rig.agent.assess_offer = broken
        with self.assertLogs("robotx.application.injection", level="ERROR"):
            status, reply = self.inject(rig)
        self.assertEqual((status, reply["reason"]), (409, "AGENT_ERROR"))
        self.assertIsNone(rig.agent.missions.active)

    def test_no_running_agent_is_503(self):
        self.assertEqual(call(None, self.gate(), "POST", INJECT, body())[0], 503)


# --- backend: no false claims, genuine telemetry ----------------------------------------------


class TestBackendSuppression(GateCase):
    def test_a_test_mission_driven_to_completion_claims_nothing_to_robotx(self):
        rig = ready_rig()
        link, sio = make_link(state=rig.agent.state, agent=rig.agent)
        binding = link.cfg.binding

        async def publish_everything():
            await link._publish_heartbeat()
            await link._publish_telemetry()
            await link._publish_custody()
            await link._publish_task_complete()
            await link._resend_unacknowledged_reports()

        async def scenario():
            await connect_and_auth(link)
            await publish_everything()
            status, reply = await acall(rig.agent, self.gate(), "POST", INJECT, body())
            assert status == 200, reply
            # SYNTHETIC: the fix walks the supplied route; an operator confirms each handover.
            for point in leg(HERE, PICKUP, points=5):
                rig.agent.gps.set_fix(*point)
                rig.run(1.0)
                await publish_everything()
            assert rig.agent.missions.active.status is MissionStatus.AT_PICKUP, rig.agent.missions.active
            rig.agent.record_custody("ACQUIRED", source="OPERATOR_CONFIRMED")
            for point in leg(PICKUP, DROP, points=5):
                rig.agent.gps.set_fix(*point)
                rig.run(1.0)
                await publish_everything()
            assert rig.agent.missions.active.status is MissionStatus.AT_DROP, rig.agent.missions.active
            rig.agent.record_custody("RELEASED", source="OPERATOR_CONFIRMED")
            rig.run(1.0)
            await publish_everything()

        asyncio.run(scenario())
        self.assertIs(rig.agent.missions.active.status, MissionStatus.COMPLETE)
        self.assertIsNone(link._held_mission(), "a test mission has no RobotX commitment")
        for name in (binding.task_complete, binding.custody_event, binding.offer_accept,
                     binding.offer_reject, binding.offer_defer, binding.command_ack):
            self.assertEqual(sio.events_named(name), [], name)
        self.assertTrue(sio.events_named(binding.heartbeat))
        self.assertTrue(all(beat == {} for beat in sio.events_named(binding.heartbeat)),
                        "a HEARTBEAT renewed a lease for a commitment nobody issued")
        frames = sio.events_named(binding.telemetry)
        self.assertTrue(frames, "telemetry stopped")
        self.assertTrue(any("lat" in f for f in frames), "genuine positions were not reported")
        self.assertFalse(any("TEST-INJECTED" in json.dumps(f) or "taskId" in f for f in frames))


if __name__ == "__main__":
    unittest.main()
