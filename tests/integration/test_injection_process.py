"""The test-only ingress in a real agent process, over a real loopback socket.

Launches the documented command (`python -m robotx.application`) with every
piece of hardware off, the ESP32 link off and the backend link off -- exactly
like `test_local_api_bind` -- so nothing here can reach a serial port, a motor
or RobotX. The agent is genuinely unready (no motor link, no fix), so an
authorized injection must be *refused by the agent*: that is the end-to-end
proof this test offers, not that anything drives.
"""

from __future__ import annotations

import contextlib
import json
import os
import secrets
import subprocess
import sys
import tempfile
import unittest
import urllib.error
import urllib.request
from pathlib import Path

from robotx.application.injection import token_digest
from tests.integration.test_local_api_bind import REPO, accepts, free_port, non_loopback_ipv4, served, wait_healthy


TOKEN = secrets.token_hex(32)
OPERATOR_TOKEN = secrets.token_hex(32)


@contextlib.contextmanager
def served_logging_to(output: Path, **api_env):
    """`test_local_api_bind.served`, keeping the agent's whole log in `output`."""

    env = {k: v for k, v in os.environ.items() if not k.startswith("ROBOTX_")}
    env.update(ROBOTX_ROBOT_ID="bind-test", ROBOTX_CAMERA_ENABLED="0", ROBOTX_PERCEPTION_ENABLED="0",
               ROBOTX_GPS_ENABLED="0", ROBOTX_LOG_LEVEL="INFO", **api_env)
    with open(output, "wb") as sink:
        process = subprocess.Popen([sys.executable, "-m", "robotx.application"], cwd=str(REPO), env=env,
                                   stdout=sink, stderr=subprocess.STDOUT)
        try:
            yield process
        finally:
            process.terminate()
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def fixture(**overrides):
    here, pickup, drop = (51.5, -0.1), (51.50036, -0.1), (51.50072, -0.1)
    raw = {
        "schema_version": 1, "test_run_id": "process-run", "task_id": "TEST-INJECTED-PROC-1",
        "robot_id": "bind-test", "region": {"center": {"lat": here[0], "lon": here[1]}, "radius_m": 300},
        "pickup": {"lat": pickup[0], "lon": pickup[1]}, "drop": {"lat": drop[0], "lon": drop[1]},
        "path_to_pickup": [{"lat": here[0], "lon": here[1]}, {"lat": pickup[0], "lon": pickup[1]}],
        "path_to_drop": [{"lat": pickup[0], "lon": pickup[1]}, {"lat": drop[0], "lon": drop[1]}],
    }
    raw.update(overrides)
    return raw


def request(port, method, path, payload=None, token=None):
    headers = {"Content-Type": "application/json"}
    if token:
        headers["Authorization"] = f"Bearer {token}"
    data = None if payload is None else json.dumps(payload).encode()
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


class TestInjectionInARealProcess(unittest.TestCase):
    def test_disabled_by_default_the_route_does_not_exist(self):
        port = free_port()
        with served(ROBOTX_API_PORT=str(port)) as process:
            wait_healthy("127.0.0.1", port, process)
            self.assertEqual(request(port, "POST", "/test/mission/inject", fixture(), TOKEN)[0], 404)
            self.assertEqual(request(port, "GET", "/test/injection")[0], 404)
            for method in ("GET", "PUT", "DELETE"):
                self.assertEqual(request(port, method, "/test/mission/inject")[0], 404, method)
            config = request(port, "GET", "/config")[1]
            self.assertIs(config["test_injection_enabled"], False)
            self.assertEqual(config["test_injection_token_sha256"], "UNSET")

    def test_the_real_entry_point_refuses_injection_unless_bound_to_a_loopback_ip(self):
        # "localhost" binds loopback, but a name is not an address the gate can
        # vouch for: enabled, it must refuse everything, and the agent must still run.
        port = free_port()
        with tempfile.TemporaryDirectory() as tmp:
            env = dict(ROBOTX_API_HOST="localhost", ROBOTX_API_PORT=str(port),
                       ROBOTX_TEST_INJECTION_ENABLED="1", ROBOTX_TEST_INJECTION_TOKEN_SHA256=token_digest(TOKEN),
                       ROBOTX_TEST_INJECTION_LOG_PATH=str(Path(tmp) / "log.jsonl"),
                       ROBOTX_OPERATOR_TOKEN_SHA256=token_digest(OPERATOR_TOKEN))
            with served(**env) as process:
                self.assertEqual(wait_healthy("127.0.0.1", port, process), 200)
                status = request(port, "GET", "/test/injection")[1]
                self.assertIs(status["available"], False)
                self.assertIn("not a loopback IP address", status["problem"])
                code, reply = request(port, "POST", "/test/mission/inject", fixture(), TOKEN)
                self.assertEqual((code, reply["code"]), (503, "NOT_CONFIGURED"))
            self.assertFalse((Path(tmp) / "log.jsonl").exists())

    def test_enabled_on_loopback_the_agent_itself_decides_and_provenance_is_kept(self):
        port = free_port()
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "test_injections.jsonl"
            agent_output = Path(tmp) / "agent.log"
            env = dict(ROBOTX_API_HOST="127.0.0.1", ROBOTX_API_PORT=str(port),
                       ROBOTX_TEST_INJECTION_ENABLED="1", ROBOTX_TEST_INJECTION_TOKEN_SHA256=token_digest(TOKEN),
                       ROBOTX_TEST_INJECTION_LOG_PATH=str(log), ROBOTX_CUSTODY_CONFIRMATION="operator",
                       ROBOTX_OPERATOR_TOKEN_SHA256=token_digest(OPERATOR_TOKEN))
            with served_logging_to(agent_output, **env) as process:
                wait_healthy("127.0.0.1", port, process)
                lan = non_loopback_ipv4()
                if lan is not None:
                    self.assertFalse(accepts(lan, port), f"reachable on {lan}")

                status = request(port, "GET", "/test/injection")[1]
                self.assertIs(status["available"], True, status)
                self.assertEqual(request(port, "POST", "/test/mission/inject", fixture())[0], 401)
                self.assertEqual(request(port, "POST", "/test/mission/inject", fixture(), "z" * 64)[0], 401)

                code, reply = request(port, "POST", "/test/mission/inject", fixture(), TOKEN)
                self.assertEqual((code, reply["outcome"], reply["reason"]), (409, "REJECTED", "NO_MOTOR_LINK"))
                self.assertIs(reply["provenance"]["backend_assigned"], False)
                self.assertIsNone(request(port, "GET", "/state")[1]["mission"])
                code, reply = request(port, "POST", "/test/mission/inject", fixture(), TOKEN)
                self.assertEqual((code, reply["code"]), (409, "DUPLICATE_TASK"))

                self.assertEqual(request(port, "GET", "/config")[1]["test_injection_token_sha256"], "SET")
            output = agent_output.read_text(errors="replace")
            self.assertIn("test_injection.enabled", output)
            self.assertIn("test_injection.rejected", output)
            self.assertIn("test_injection.unauthorized", output)
            records = [json.loads(line) for line in log.read_text().splitlines()]
            self.assertEqual([r["event"] for r in records], ["ATTEMPTED", "REJECTED"])
            self.assertTrue(all(r["provenance"]["source"] == "test_injected" for r in records))
            for text in (log.read_text(), output):
                self.assertNotIn(TOKEN, text)
                self.assertNotIn(token_digest(TOKEN), text)


def enabled_env(port, log):
    return dict(ROBOTX_API_HOST="127.0.0.1", ROBOTX_API_PORT=str(port), ROBOTX_CUSTODY_CONFIRMATION="operator",
                ROBOTX_TEST_INJECTION_ENABLED="1", ROBOTX_TEST_INJECTION_TOKEN_SHA256=token_digest(TOKEN),
                ROBOTX_TEST_INJECTION_LOG_PATH=str(log), ROBOTX_OPERATOR_TOKEN_SHA256=token_digest(OPERATOR_TOKEN))


def post_with(port, headers):
    req = urllib.request.Request(f"http://127.0.0.1:{port}/test/mission/inject", data=json.dumps(fixture()).encode(),
                                 headers={"Content-Type": "application/json", **headers}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def post_with_path(port, path, headers):
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", data=b"{}", headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


class TestRealProcessHardening(unittest.TestCase):
    """The deployed entry point, hardware off, backend link off. Nothing simulated but absence."""

    def test_uvicorn_proxy_headers_cannot_lend_loopback_or_a_host(self):
        port = free_port()
        with tempfile.TemporaryDirectory() as tmp:
            with served(**enabled_env(port, Path(tmp) / "log.jsonl")) as process:
                wait_healthy("127.0.0.1", port, process)
                auth = {"Authorization": f"Bearer {TOKEN}"}
                # uvicorn trusts X-Forwarded-For from 127.0.0.1 by default and rewrites the client.
                code, reply = post_with(port, {**auth, "X-Forwarded-For": "10.0.0.9"})
                self.assertEqual((code, reply["code"]), (403, "NOT_LOOPBACK"))
                code, reply = post_with(port, {**auth, "X-Forwarded-Host": "attacker.example",
                                               "Host": "attacker.example:8000"})
                self.assertEqual((code, reply["code"]), (403, "HOST_NOT_LOOPBACK"))
                # Even a loopback X-Forwarded-For lends nothing: uvicorn 0.30.6 then sets
                # the client to None (every hop trusted), which the gate refuses.
                code, reply = post_with(port, {**auth, "X-Forwarded-For": "127.0.0.1"})
                self.assertEqual((code, reply["code"]), (403, "NOT_LOOPBACK"))
                # With no proxy header, the token decides.
                self.assertEqual(post_with(port, {})[0], 401)
                self.assertIsNone(request(port, "GET", "/state")[1]["mission"])

    def test_browser_requests_to_release_routes_are_refused_with_no_token_configured(self):
        port = free_port()
        with served(ROBOTX_API_PORT=str(port)) as process:  # defaults: no operator token, no injection
            wait_healthy("127.0.0.1", port, process)
            self.assertEqual(post_with_path(port, "/safety/estop", {"Origin": "http://evil.example"})[0], 200)
            for path in ("/safety/clear", "/controller/reset", "/mission/idle", "/mission/custody"):
                code, reply = post_with_path(port, path, {"Origin": "http://evil.example",
                                                          "Content-Type": "text/plain"})
                self.assertEqual((code, reply["detail"]), (403, "browser requests are refused on this route"), path)
            # The documented curl call (no Origin) is served as before -- and finds the
            # latch still engaged, so none of the browser requests released it.
            code, reply = post_with_path(port, "/safety/clear", {})
            self.assertEqual((code, reply["was_engaged"]), (200, True))

    def test_restart_keeps_spent_task_ids_and_recovers_a_torn_record(self):
        port = free_port()
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "test_injections.jsonl"
            with served(**enabled_env(port, log)) as process:
                wait_healthy("127.0.0.1", port, process)
                code, reply = request(port, "POST", "/test/mission/inject", fixture(), TOKEN)
                self.assertEqual((code, reply["outcome"]), (409, "REJECTED"))
            # As if power failed while the next record was being written.
            with open(log, "ab") as h:
                h.write(b'{"event": "ATTEMPTED", "task_id": "TEST-INJ')
            with served(**enabled_env(port, log)) as process:
                wait_healthy("127.0.0.1", port, process)
                self.assertIs(request(port, "GET", "/test/injection")[1]["available"], True)
                code, reply = request(port, "POST", "/test/mission/inject", fixture(), TOKEN)
                self.assertEqual((code, reply["code"]), (409, "DUPLICATE_TASK"))
                code, reply = request(port, "POST", "/test/mission/inject",
                                      fixture(task_id="TEST-INJECTED-PROC-2"), TOKEN)
                self.assertEqual((code, reply["reason"]), (409, "NO_MOTOR_LINK"))
            lines = log.read_text().splitlines()
            self.assertEqual([json.loads(line)["event"] for line in lines],
                             ["ATTEMPTED", "REJECTED", "ATTEMPTED", "REJECTED"])
            self.assertEqual(len(list(Path(tmp).glob("test_injections.jsonl.torn-*"))), 1)


if __name__ == "__main__":
    unittest.main()
