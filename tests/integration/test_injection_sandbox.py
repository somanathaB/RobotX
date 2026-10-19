"""The test ingress under the production unit's systemd sandbox.

What is real and what is simulated
----------------------------------
- REAL: the documented entry point, run by systemd as a transient unit with the
  hardening directives copied verbatim from `deployment/robotx-agent.service`
  (ProtectSystem=strict, ProtectHome=yes, NoNewPrivileges, UMask=0077, ...),
  enforced by the kernel.
- DIFFERENT from production, and stated: it runs under the *user* service
  manager as the invoking user, not the system manager as `robotx`;
  `PrivateTmp=` is turned off so the code copy under /tmp is visible; and
  `StateDirectory=robotx` (/var/lib/robotx, writable, 0700) is stood in for by
  `ReadWritePaths=` on a temporary directory, because a user unit's
  StateDirectory lives under $HOME, which ProtectHome hides.
- ABSENT: all hardware and the backend link. Nothing here can move anything.

Skipped where no user service manager is available.
"""

from __future__ import annotations

import json
import os
import secrets
import shutil
import subprocess
import tempfile
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

from robotx.application.injection import token_digest
from tests.integration.test_injection_process import fixture
from tests.integration.test_local_api_bind import REPO, free_port

PYTHON = "/opt/robotx/RobotX-Pi/venv/bin/python"
UNIT_FILE = REPO / "deployment" / "robotx-agent.service"
# Hardening the production unit applies, taken from the unit file itself.
SANDBOX_KEYS = ("NoNewPrivileges", "ProtectSystem", "ProtectHome", "ProtectKernelTunables", "ProtectKernelModules",
                "ProtectControlGroups", "RestrictSUIDSGID", "LockPersonality", "RestrictRealtime", "UMask",
                "CapabilityBoundingSet")
TOKEN = secrets.token_hex(32)
OPERATOR_TOKEN = secrets.token_hex(32)


def user_manager_available() -> bool:
    if shutil.which("systemd-run") is None or not Path(PYTHON).exists():
        return False
    state = subprocess.run(["systemctl", "--user", "is-system-running"], capture_output=True, text=True).stdout
    return state.strip() in ("running", "degraded")


def unit_hardening() -> list:
    props = []
    for line in UNIT_FILE.read_text().splitlines():
        key, _, value = line.partition("=")
        if key.strip() in SANDBOX_KEYS:
            props.append(f"{key.strip()}={value.strip()}")
    return props


def http(port, method, path, payload=None, token=None):
    headers = {"Content-Type": "application/json"}
    if token:
        headers["Authorization"] = f"Bearer {token}"
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", method=method, headers=headers,
                                 data=None if payload is None else json.dumps(payload).encode())
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


@unittest.skipUnless(user_manager_available(), "no systemd user service manager")
class TestUnderTheProductionSandbox(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.props = unit_hardening()
        assert "ProtectSystem=strict" in cls.props and "ProtectHome=yes" in cls.props, cls.props
        cls.root = Path(tempfile.mkdtemp(prefix="robotx-sandbox-", dir="/tmp"))
        shutil.copytree(REPO / "robotx", cls.root / "code" / "robotx")
        cls.state = cls.root / "state"
        cls.state.mkdir()

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.root, ignore_errors=True)

    def run_agent(self, log_path: str):
        port = free_port()
        unit = f"robotx-sandbox-test-{secrets.token_hex(4)}"
        env = dict(ROBOTX_ROBOT_ID="bind-test", ROBOTX_CAMERA_ENABLED="0", ROBOTX_PERCEPTION_ENABLED="0",
                   ROBOTX_GPS_ENABLED="0", ROBOTX_LOG_LEVEL="WARNING", ROBOTX_API_HOST="127.0.0.1",
                   ROBOTX_API_PORT=str(port), ROBOTX_CUSTODY_CONFIRMATION="operator",
                   ROBOTX_TEST_INJECTION_ENABLED="1", ROBOTX_TEST_INJECTION_TOKEN_SHA256=token_digest(TOKEN),
                   ROBOTX_TEST_INJECTION_LOG_PATH=log_path,
                   ROBOTX_OPERATOR_TOKEN_SHA256=token_digest(OPERATOR_TOKEN), PYTHONUNBUFFERED="1")
        argv = ["systemd-run", "--user", "--quiet", "--collect", f"--unit={unit}",
                f"--working-directory={self.root / 'code'}", "-p", "PrivateTmp=no",
                "-p", f"ReadWritePaths={self.state}"]
        for prop in self.props:
            argv += ["-p", prop]
        for key, value in env.items():
            argv.append(f"--setenv={key}={value}")
        argv += [PYTHON, "-m", "robotx.application"]
        subprocess.run(argv, check=True, timeout=30)
        self.addCleanup(subprocess.run, ["systemctl", "--user", "stop", unit], capture_output=True, timeout=30)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            try:
                if http(port, "GET", "/health")[0] == 200:
                    return port
            except OSError:
                time.sleep(0.3)
        journal = subprocess.run(["journalctl", "--user", "-u", unit, "--no-pager", "-n", "40"],
                                 capture_output=True, text=True).stdout
        self.fail(f"sandboxed agent never became healthy:\n{journal}")

    def test_a_log_under_home_is_refused_and_the_agent_still_runs(self):
        port = self.run_agent("/home/pi/.robotx/test_injections.jsonl")
        status = http(port, "GET", "/test/injection")[1]
        self.assertIs(status["available"], False)
        self.assertIn("PermissionError", status["problem"])
        self.assertEqual(http(port, "POST", "/test/mission/inject", fixture(), TOKEN)[0], 503)

    def test_a_log_on_the_read_only_system_is_refused_at_start(self):
        port = self.run_agent("/opt/robotx/test_injections.jsonl")
        status = http(port, "GET", "/test/injection")[1]
        self.assertIs(status["available"], False)
        self.assertIn("cannot be written", status["problem"])
        self.assertEqual(http(port, "POST", "/test/mission/inject", fixture(), TOKEN)[0], 503)
        self.assertFalse(Path("/opt/robotx/test_injections.jsonl").exists())

    def test_a_log_in_the_state_directory_works_and_is_private(self):
        log = self.state / "test_injections.jsonl"
        port = self.run_agent(str(log))
        self.assertIs(http(port, "GET", "/test/injection")[1]["available"], True)
        code, reply = http(port, "POST", "/test/mission/inject", fixture(), TOKEN)
        self.assertEqual((code, reply.get("reason")), (409, "NO_MOTOR_LINK"), reply)
        self.assertEqual([json.loads(line)["event"] for line in log.read_text().splitlines()],
                         ["ATTEMPTED", "REJECTED"])
        self.assertEqual(oct(os.stat(log).st_mode & 0o777), "0o600", "UMask=0077 must keep the log private")


if __name__ == "__main__":
    unittest.main()
