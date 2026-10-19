"""The test-only mission injection harness (`tests.mission_injection`).

Test doubles, labelled
----------------------
- `Rig` (from `test_obstacle_recovery`): the REAL `RobotAgent`, MissionManager,
  navigator, decision layer, safety gate and `Esp32Link`, over a SIMULATED
  serial port (`tests.fixtures.esp32.FakePort`) on a fake clock. What matters
  is what reaches that port.
- GPS fixes and perception results are SYNTHETIC (`FakeGPS`, `FakePerception`).
- The agent's ingress (`POST /test/mission/inject`) is called through the real
  route handlers in-process; its own tests are `test_injection_endpoint`.
- Preflight observations are hand-built dicts in the shape of the live
  `/state`, `/config`, `/backend`, `/health` and `/test/injection` responses.

Nothing here opens a serial port, a socket or the loopback API, and no test is
evidence about the physical Rover or the RobotX backend.
"""

from __future__ import annotations

import copy
import json
import dataclasses
import io
import logging
import os
import re
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

from robotx.application.injection import InjectionConfig, InjectionGate, token_digest
from robotx.mission.mission import MissionStatus
from robotx.state.telemetry import build_telemetry
from tests.fixtures.task_assign import leg, offset
from tests.mission_injection import fixture as fixture_mod, inject as inject_mod, ledger as ledger_mod, preflight
from tests.mission_injection.fixture import FixtureError, load_fixture, validate_fixture
from tests.mission_injection.ledger import DuplicateInjection, Ledger, LedgerError
from tests.mission_injection.lifecycle import LifecycleRecorder
from tests.unit.test_agent import HERE
from tests.unit.test_injection_endpoint import INJECT, OPERATOR_TOKEN, TOKEN, call
from tests.unit.test_obstacle_recovery import Rig


REPO = Path(__file__).resolve().parents[2]
HARNESS = REPO / "tests" / "mission_injection"
ROBOT = "test-rover"
PICKUP = offset(HERE, north_m=40.0)
DROP = offset(HERE, north_m=80.0)


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


def pt(p):
    return {"lat": p[0], "lon": p[1]}


def fixture_dict(**overrides):
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


def make_fixture(**overrides):
    return validate_fixture(fixture_dict(**overrides), expected_robot_id=ROBOT)


class LedgerCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.ledger_path = Path(self.tmp.name) / "ledger.jsonl"
        self.ledger = Ledger(self.ledger_path)


def ready_rig(*, custody="operator"):
    """The real agent, ready by its own rules: motor link, fix, custody."""

    rig = Rig()
    rig.agent.settings = dataclasses.replace(rig.agent.settings, custody_confirmation=custody)
    rig.run(0.6)
    return rig


# --- 1-4: the fixture -----------------------------------------------------------


class TestFixtureValidation(unittest.TestCase):
    def test_1_a_valid_fixture_is_accepted_as_exactly_two_stops(self):
        f = make_fixture()
        self.assertEqual(f.task_id, "TEST-INJECTED-0001")
        self.assertEqual(f.mission.pickup, PICKUP)
        a = f.assignment()
        self.assertEqual([s.sequence for s in a.stops], [0, 1])
        self.assertEqual(a.stops[0].path, f.mission.path_to_pickup)
        self.assertEqual(a.stops[1].path, f.mission.path_to_drop)
        self.assertEqual(a.provenance, "test_injected")

    def test_the_example_fixture_in_the_repository_is_valid(self):
        f = load_fixture(HARNESS / "fixtures" / "example_mission.json", expected_robot_id="RBT-2000")
        self.assertTrue(f.task_id.startswith("TEST-INJECTED-"))

    def test_2_every_required_field_is_required(self):
        for key in fixture_mod.REQUIRED_FIELDS:
            raw = fixture_dict()
            del raw[key]
            with self.subTest(key=key), self.assertRaises(FixtureError) as e:
                validate_fixture(raw)
            self.assertEqual(e.exception.code, "MISSING_FIELD")

    def test_unknown_fields_are_refused_not_ignored(self):
        with self.assertRaises(FixtureError) as e:
            validate_fixture(fixture_dict(commitmentId="c-1", signature="ab"))
        self.assertEqual(e.exception.code, "UNKNOWN_FIELD")

    def test_task_id_must_carry_the_test_provenance_prefix(self):
        for task in ("T-123", "task-TEST-INJECTED-1", "TEST-INJECTED-", "", 7):
            with self.subTest(task=task), self.assertRaises(FixtureError):
                validate_fixture(fixture_dict(task_id=task))

    def test_wrong_robot_is_refused(self):
        with self.assertRaises(FixtureError) as e:
            validate_fixture(fixture_dict(robot_id="RBT-9999"), expected_robot_id=ROBOT)
        self.assertEqual(e.exception.code, "WRONG_ROBOT")

    def test_3_invalid_pickup_or_drop_is_refused_by_the_rovers_own_rules(self):
        cases = {
            "pickup": [{"lat": float("nan"), "lon": 0.0}, {"lat": 91.0, "lon": 0.0},
                       {"lat": "12.9", "lon": 77.5}, {"lat": 1.0}, [1.0, 2.0], None],
            "drop": [{"lat": 0.0, "lon": 181.0}, {"lat": True, "lon": 0.0}, {"lat": 1, "lon": 2, "x": 3}],
        }
        for key, values in cases.items():
            for value in values:
                with self.subTest(key=key, value=value), self.assertRaises(FixtureError) as e:
                    validate_fixture(fixture_dict(**{key: value}))
                self.assertIn(e.exception.code, ("INVALID_COORDINATE", "MALFORMED"))

    def test_4_unsupported_routes_are_refused(self):
        cases = [
            ("path_to_pickup", [pt(HERE)], "EMPTY_ROUTE"),
            ("path_to_drop", [], "EMPTY_ROUTE"),
            ("path_to_pickup", "HERE->PICKUP", "MALFORMED"),
            ("path_to_drop", [pt(PICKUP), [1.0, 2.0]], "MALFORMED"),
            ("path_to_pickup", [pt(HERE)] * 2001, "ROUTE_TOO_LONG"),
        ]
        for key, value, code in cases:
            with self.subTest(key=key, code=code), self.assertRaises(FixtureError) as e:
                validate_fixture(fixture_dict(**{key: value}))
            self.assertEqual(e.exception.code, code)

    def test_out_of_region_points_are_refused(self):
        far = offset(HERE, north_m=400.0)
        for key, value in (("drop", pt(far)), ("path_to_drop", [pt(PICKUP), pt(far)])):
            with self.subTest(key=key), self.assertRaises(FixtureError) as e:
                validate_fixture(fixture_dict(**{key: value}))
            self.assertEqual(e.exception.code, "OUT_OF_REGION")

    def test_region_radius_is_bounded(self):
        for radius in (0, -1, 501, float("inf"), True, "100"):
            with self.subTest(radius=radius), self.assertRaises(FixtureError) as e:
                validate_fixture(fixture_dict(region={"center": pt(HERE), "radius_m": radius}))
            self.assertEqual(e.exception.code, "REGION")


# --- 5-6: duplicates and provenance -----------------------------------------------


class TestLedger(LedgerCase):
    def test_5_a_task_id_is_injected_once(self):
        f = make_fixture()
        self.ledger.claim_attempt(f, ingress="unit")
        with self.assertRaises(DuplicateInjection):
            self.ledger.claim_attempt(f, ingress="unit")
        # Another process, another test run, same task id: still a duplicate.
        with self.assertRaises(DuplicateInjection):
            Ledger(self.ledger_path).claim_attempt(make_fixture(test_run_id="other-run"), ingress="unit")
        attempts = [r for r in self.ledger.records() if r["event"] == "INJECTION_ATTEMPTED"]
        self.assertEqual(len(attempts), 1)

    def test_a_corrupt_ledger_refuses_instead_of_forgetting(self):
        self.ledger.claim_attempt(make_fixture(), ingress="unit")
        with open(self.ledger_path, "a", encoding="utf-8") as h:
            h.write("{not json\n")
        with self.assertRaises(LedgerError):
            self.ledger.claim_attempt(make_fixture(task_id="TEST-INJECTED-0002"), ingress="unit")

    def test_6_every_record_carries_test_provenance(self):
        f = make_fixture()
        self.ledger.append(ledger_mod.FIXTURE_VALIDATED, fixture=f)
        self.ledger.claim_attempt(f, ingress="unit-ingress")
        self.ledger.append(ledger_mod.REJECTED, fixture=f, reason="NO_MOTOR_LINK")
        records = self.ledger.records()
        self.assertEqual(len(records), 3)
        for r in records:
            self.assertEqual(r["provenance"]["source"], "test_injected")
            self.assertIs(r["provenance"]["backend_assigned"], False)
            self.assertIn("cannot assign tasks to physical robots", r["provenance"]["reason"])
            self.assertEqual((r["test_run_id"], r["task_id"]), ("unit-run", "TEST-INJECTED-0001"))
            self.assertTrue(r["at"].endswith("+00:00"))
        attempt = records[1]
        self.assertEqual(attempt["ingress"], "unit-ingress")
        self.assertEqual(attempt["fixture"]["pickup"], pt(PICKUP))
        self.assertEqual(oct(os.stat(self.ledger_path).st_mode & 0o777), "0o600")


# --- 13: a supplied route is not a completed one --------------------------------------


class TestCompletionIsNeverInferred(LedgerCase):
    """The task enters through the agent's real ingress; completion is only what the agent reports."""

    def inject(self, rig, fixture):
        gate = InjectionGate(InjectionConfig(
            enabled=True, token_sha256=token_digest(TOKEN), log_path=str(Path(self.tmp.name) / "agent.jsonl"),
            api_host="127.0.0.1", robot_id=ROBOT, operator_token_sha256=token_digest(OPERATOR_TOKEN)))
        return call(rig.agent, gate, "POST", INJECT, fixture.raw)[0]

    def test_13_standing_still_with_a_route_completes_nothing(self):
        rig = ready_rig()
        f = make_fixture()
        recorder = LifecycleRecorder(f.task_id, self.ledger, test_run_id=f.test_run_id)
        self.assertEqual(self.inject(rig, f), 200)
        for snap in rig.run(10.0):
            recorder.observe(build_telemetry(snap))
        self.assertIs(rig.agent.missions.active.status, MissionStatus.TO_PICKUP)
        self.assertFalse(recorder.completed)
        self.assertNotIn("PICKUP_REACHED", recorder.stages())
        self.assertIn("MISSION_HELD", recorder.stages())

    def test_13_reaching_both_stops_without_custody_is_not_completion(self):
        rig = ready_rig()
        f = make_fixture()
        recorder = LifecycleRecorder(f.task_id, test_run_id=f.test_run_id)
        self.assertEqual(self.inject(rig, f), 200)
        # SYNTHETIC: the fix walks the supplied route, waypoint by waypoint.
        for point in list(f.mission.path_to_pickup) + list(f.mission.path_to_drop):
            rig.agent.gps.set_fix(*point)
            for snap in rig.run(1.0):
                recorder.observe(build_telemetry(snap))
        active = rig.agent.missions.active
        self.assertIs(active.status, MissionStatus.AT_PICKUP, "held for custody at the pickup")
        self.assertIn("PICKUP_REACHED", recorder.stages())
        self.assertNotIn("DROP_REACHED", recorder.stages())
        self.assertFalse(recorder.completed)


# --- 11: backend telemetry failures are reported, and no delivery is claimed ---------


class TestBackendHonesty(unittest.TestCase):
    # That a test mission sends no OFFER_*, CUSTODY_EVENT or TASK_COMPLETE, driven
    # to completion: test_injection_endpoint.TestBackendSuppression.
    def test_11_lost_backend_telemetry_is_reported_as_such(self):
        f = make_fixture()
        recorder = LifecycleRecorder(f.task_id)
        base = {"mode": "AUTO", "mission": {"task_id": f.task_id, "status": "TO_PICKUP"},
                "safety": {"verdict": "ALLOWED", "rule": "clear"}, "navigation": {"status": "NAVIGATING"}}
        for backend in ("STREAMING", "DISCONNECTED", "DISCONNECTED"):
            recorder.observe(dict(base, communication={"backend": backend, "esp32": "UP"}))
        self.assertEqual(recorder.backend_telemetry(), "INTERRUPTED")
        self.assertFalse(recorder.completed)
        never = LifecycleRecorder(f.task_id)
        never.observe(dict(base, communication={"backend": "AUTH_FAILED"}))
        self.assertEqual(never.backend_telemetry(), "FAILED")
        self.assertEqual(LifecycleRecorder(f.task_id).backend_telemetry(), "NOT_OBSERVED")

    def test_11_preflight_fails_a_backend_that_is_not_streaming(self):
        observed = live_like_observation()
        observed["backend"].update(status="DISCONNECTED", streaming=False, authenticated=False)
        checks = {c.name: c for c in preflight.evaluate(observed).checks}
        self.assertIs(checks["backend_auth_and_telemetry"].result, preflight.Result.FAIL)


# --- preflight ---------------------------------------------------------------------------


def live_like_observation():
    """The shape (and the values) the running agent reported on 2026-10-19."""

    tel = {"state": "DRIVE_UNAVAILABLE", "block_reason": "MOTOR_PWM_UNAVAILABLE",
           "motor": {"drive_available": False},
           "front": {"valid": True, "obstacle": True, "warning": True, "left_cm": 20.9, "right_cm": 60.6},
           "rear": {"available": False, "obstacle": False, "sensor_fault": False},
           "gates": {"forward_blocked": True, "reverse_blocked": True, "safety_stop": False}}
    return {
        "errors": {},
        "test_injection": None,
        "test_injection_http_status": 404,
        "service_active": "active",
        "clock_synchronized": True,
        "collected_at": 1792387581.0,
        "health": {"robot_id": "RBT-2000", "ok": True},
        "config": {"robot_id": "RBT-2000", "socket_enabled": True, "esp32_port": "/dev/ttyAMA0",
                   "gps_source": "esp32", "esp32_stale_after_s": 1.0, "esp32_motion_enabled": False,
                   "gps_stale_after_s": 5.0, "nav_max_h_acc_m": 5.0, "safety_stop_distance_cm": 90.0,
                   "safety_require_range_sensor": False, "safety_max_speed": 0.75,
                   "custody_confirmation": "operator"},
        "backend": {"server": preflight.EXPECTED_BACKEND_URL, "tls": True, "status": "STREAMING",
                    "authenticated": True, "streaming": True, "auth_method": "TOKEN", "robot_id": "RBT-2000",
                    "stats": {"telemetry_sent": 199, "emit_failures": 0}},
        "state": {
            "robot_id": "RBT-2000", "mode": "IDLE", "last_error": None, "mission": None,
            "gps": {"status": "FIX", "age_s": 1.86},
            "position": {"latitude": 12.97161, "longitude": 77.59461, "source": "GPS", "fix_type": "3D",
                         "satellites": 14, "h_acc_m": 3.743},
            "safety": {"verdict": "ALLOWED", "rule": "stop", "reason": "no active mission"},
            "communication": {"esp32": "UP", "esp32_detail": "bidirectional", "backend": "STREAMING"},
            "controller": {"telemetry": tel, "telemetry_age_s": 0.18, "bidirectional": True,
                           "motion_enabled": False, "motion_ready": False},
            "controller_diag": {
                "FRONT": {"front_health": "OK", "sensor_map_verified": False},
                "REAR": {"rear_backend": "TCA9548A_NOT_FOUND", "rear_sensor_status": ["UNINITIALISED"] * 3},
                "SYSTEM": {"fw_build": "f10bb8f7208e", "pca_status": "NOT_FOUND",
                           "motor_drive_status": "NOT_FOUND", "stall_count": 7, "last_stall_ms": 1000,
                           "last_stall_source": "GPS"}},
            "power": {"status": "SIMULATED", "source": "DEMO SIMULATION"},
        },
    }


def all_green_observation():
    o = copy.deepcopy(live_like_observation())
    o["config"]["esp32_motion_enabled"] = True
    c = o["state"]["controller"]
    c.update(motion_enabled=True, motion_ready=True)
    c["telemetry"].update(state="IDLE", block_reason="NONE", motor={"drive_available": True},
                          front={"valid": True, "obstacle": False, "left_cm": 150.0, "right_cm": 150.0},
                          rear={"available": True, "sensor_fault": False},
                          gates={"forward_blocked": False, "reverse_blocked": False, "safety_stop": False})
    o["state"]["controller_diag"]["FRONT"]["sensor_map_verified"] = True
    o["test_injection"] = {"enabled": True, "available": True, "problem": None, "locked_out": False,
                           "injected_task_ids": 0}
    o.pop("test_injection_http_status")
    return o


class TestPreflight(unittest.TestCase):
    def test_the_live_observation_is_blocked_for_each_real_reason(self):
        report = preflight.evaluate(live_like_observation())
        r = {c.name: c.result for c in report.checks}
        self.assertEqual(report.verdict, "BLOCKED")
        self.assertIs(r["mission_ingress"], preflight.Result.BLOCKED)
        self.assertIs(r["motor_controller"], preflight.Result.FAIL)
        self.assertIs(r["motion_enabled"], preflight.Result.BLOCKED)
        self.assertIs(r["motion_ready"], preflight.Result.FAIL)
        self.assertIs(r["front_path_clear"], preflight.Result.BLOCKED)
        self.assertIs(r["rear_sensors"], preflight.Result.FAIL)
        self.assertIs(r["front_sensor_map"], preflight.Result.NOT_TESTED)
        self.assertIs(r["uart_ownership"], preflight.Result.NOT_TESTED)
        for name in ("agent_service", "backend_destination", "backend_auth_and_telemetry", "robot_identity",
                     "esp32_link", "front_sensors", "localization", "safety_state", "safety_config",
                     "mission_state", "clock_synchronized"):
            self.assertIs(r[name], preflight.Result.PASS, name)

    def test_not_tested_is_never_a_pass(self):
        report = preflight.evaluate(all_green_observation())
        blockers = {c.name for c in report.blockers()}
        self.assertEqual(blockers, {"uart_ownership"})
        self.assertEqual(report.verdict, "BLOCKED")

    def test_ingress_passes_only_when_the_agent_reports_injection_available(self):
        def ingress(o):
            return {c.name: c for c in preflight.evaluate(o).checks}["mission_ingress"]

        self.assertIs(ingress(all_green_observation()).result, preflight.Result.PASS)
        disabled = ingress(live_like_observation())
        self.assertIs(disabled.result, preflight.Result.BLOCKED)
        self.assertIn("disabled", disabled.detail)
        for change in ({"available": False, "problem": "no digest"}, {"available": False, "locked_out": True}):
            o = all_green_observation()
            o["test_injection"].update(change)
            self.assertIs(ingress(o).result, preflight.Result.BLOCKED, change)
        bench = all_green_observation()
        bench["config"]["socket_enabled"] = False
        self.assertIs(ingress(bench).result, preflight.Result.BLOCKED, "no backend telemetry in bench mode")

    def test_a_404_from_the_injection_status_is_an_answer_not_an_api_error(self):
        import urllib.error

        class Client:
            def get(self, path):
                if path == "/test/injection":
                    raise urllib.error.HTTPError(path, 404, "Not Found", {}, None)
                return {}

        observed = preflight.collect(Client(), run=lambda argv: "active")
        self.assertEqual(observed["errors"], {})
        self.assertEqual(observed["test_injection_http_status"], 404)

    def test_a_wrong_backend_identity_or_threshold_fails(self):
        o = live_like_observation()
        o["backend"]["server"] = "http://192.168.1.5:3000"
        o["backend"]["robot_id"] = "RBT-1"
        o["config"]["safety_stop_distance_cm"] = 30.0
        r = {c.name: c.result for c in preflight.evaluate(o).checks}
        self.assertIs(r["backend_destination"], preflight.Result.FAIL)
        self.assertIs(r["robot_identity"], preflight.Result.FAIL)
        self.assertIs(r["safety_config"], preflight.Result.FAIL)

    def test_stale_esp32_or_gps_fails(self):
        o = live_like_observation()
        o["state"]["controller"]["telemetry_age_s"] = 4.0
        o["state"]["gps"]["age_s"] = 9.0
        r = {c.name: c.result for c in preflight.evaluate(o).checks}
        self.assertIs(r["esp32_link"], preflight.Result.FAIL)
        self.assertIs(r["front_sensors"], preflight.Result.FAIL)
        self.assertIs(r["localization"], preflight.Result.FAIL)

    def test_an_unreachable_agent_is_a_fail_not_a_guess(self):
        o = {"errors": {"/state": "URLError: refused"}, "service_active": "inactive",
             "health": None, "state": None, "config": None, "backend": None}
        report = preflight.evaluate(o)
        self.assertEqual(report.verdict, "BLOCKED")
        self.assertEqual(report.checks[-1].name, "observations")

    def test_the_rover_must_be_inside_the_fixture_region(self):
        here = live_like_observation()["state"]["position"]
        f = validate_fixture(fixture_dict(
            robot_id="RBT-2000", region={"center": pt(HERE), "radius_m": 300}))
        o = live_like_observation()
        r = {c.name: c.result for c in preflight.evaluate(o, fixture=f).checks}
        self.assertIs(r["rover_in_fixture_region"], preflight.Result.FAIL, here)

    def test_the_client_reads_allowlisted_loopback_paths_only(self):
        client = preflight.LoopbackClient()
        for path in ("/camera", "/mission/start", "/safety/estop", "/controller/reset"):
            with self.assertRaises(ValueError):
                client.get(path)
        self.assertFalse(hasattr(client, "post"))
        with self.assertRaises(ValueError):
            preflight.LoopbackClient("https://robotx-admin-dashboard.onrender.com")


# --- 14-15: isolation from production -------------------------------------------------------


class TestIsolation(unittest.TestCase):
    def harness_sources(self):
        return {p.name: p.read_text(encoding="utf-8") for p in HARNESS.glob("*.py")}

    def test_14_production_code_never_imports_the_harness_or_tests(self):
        for path in (REPO / "robotx").rglob("*.py"):
            text = path.read_text(encoding="utf-8")
            self.assertIsNone(re.search(r"^\s*(from|import)\s+tests\b", text, re.M), path)
            self.assertNotIn("mission_injection", text, path)
        for path in (REPO / "deployment").glob("*"):
            if path.is_file():
                text = path.read_text(encoding="utf-8", errors="replace")
                self.assertNotIn("mission_injection", text, path)
                self.assertNotIn("TEST_INJECTION", text, path)

    def test_14_only_the_ingress_and_its_wiring_know_about_test_injection(self):
        knows = {p.relative_to(REPO).as_posix() for p in (REPO / "robotx").rglob("*.py")
                 if re.search(r"TEST-INJECTED|test_injection|injection import", p.read_text(encoding="utf-8"))}
        self.assertEqual(knows, {"robotx/application/injection.py", "robotx/application/main.py",
                                 "robotx/config/settings.py"})

    def test_14_a_freshly_built_agent_holds_no_mission(self):
        rig = Rig()
        snaps = rig.run(1.0)
        self.assertIsNone(snaps[-1].mission)
        self.assertEqual(rig.drives(), [])

    def test_15_the_harness_writes_no_configuration_and_drives_nothing_directly(self):
        forbidden = [
            r"os\.environ\[", r"os\.putenv", r"setenv", r"/etc/robotx", r"systemctl\s+(start|stop|restart)",
            r"\"systemctl\",\s*\"(start|stop|restart)", r"import serial", r"robotx\.esp32\.transport",
            r"robotx\.hardware\.motors", r"import socketio", r"encode_command", r"submit_decision",
            r"esptool", r"\.sign\(", r"COMMAND_SIGNING_KEY\s*=",
        ]
        for name, text in self.harness_sources().items():
            for pattern in forbidden:
                self.assertIsNone(re.search(pattern, text), f"{name}: {pattern}")

    def test_15_the_one_write_is_the_injection_post(self):
        posts = {name: text.count('method="POST"') for name, text in self.harness_sources().items()}
        self.assertEqual({n: c for n, c in posts.items() if c}, {"inject.py": 1})
        self.assertEqual(inject_mod.INJECT_PATH, "/test/mission/inject")
        public = [m for m in vars(inject_mod.InjectionClient) if not m.startswith("_")]
        self.assertEqual(public, ["inject"])
        for base in ("http://192.168.1.5:8000", "http://localhost:8000", "https://robotx-admin-dashboard.onrender.com"):
            with self.assertRaises(ValueError):
                inject_mod.InjectionClient(base)

    def test_15_the_harness_never_touches_backend_url_or_credentials(self):
        for name, text in self.harness_sources().items():
            for word in ("socket_server_url", "robot_token", "pairing_code", "backend_token_path",
                         "command_signing_key", "verify_signature"):
                self.assertNotIn(word, text, f"{name} mentions {word}")


class TestInjectCli(LedgerCase):
    def write_fixture(self):
        path = Path(self.tmp.name) / "fixture.json"
        path.write_text(json.dumps(fixture_dict(robot_id="RBT-2000")))
        return str(path)

    def run_cli(self, *args, observed=None):
        out = io.StringIO()
        never = mock.Mock(side_effect=AssertionError("the CLI posted"))
        with mock.patch.object(inject_mod, "collect", return_value=observed or live_like_observation()), \
                mock.patch.object(inject_mod.InjectionClient, "inject", never), redirect_stdout(out):
            code = inject_mod.main(["--fixture", self.write_fixture(), "--ledger", str(self.ledger_path), *args])
        return code, out.getvalue(), never

    def test_without_authorization_nothing_is_posted(self):
        code, out, post = self.run_cli()
        self.assertEqual(code, 1)
        self.assertIn("NOT_TESTED", out)
        post.assert_not_called()

    def test_an_unready_preflight_blocks_even_with_authorization(self):
        code, out, post = self.run_cli("--authorize-hardware-test", "--token-file", "/nonexistent")
        self.assertEqual(code, 3)
        self.assertIn("BLOCKED: preflight is not READY", out)
        post.assert_not_called()
        events = [r["event"] for r in self.ledger.records()]
        self.assertEqual(events, ["FIXTURE_VALIDATED", "PREFLIGHT", "INJECTION_BLOCKED"])

    def test_the_token_file_must_be_owner_only(self):
        path = Path(self.tmp.name) / "token"
        path.write_text(TOKEN + "\n")
        os.chmod(path, 0o644)
        with self.assertRaises(inject_mod.TokenFileError):
            inject_mod.read_token(str(path))
        os.chmod(path, 0o600)
        self.assertEqual(inject_mod.read_token(str(path)), TOKEN)
        path.write_text("")
        with self.assertRaises(inject_mod.TokenFileError):
            inject_mod.read_token(str(path))


if __name__ == "__main__":
    unittest.main()
