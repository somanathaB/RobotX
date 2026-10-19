"""A real agent process, its real Socket.IO backend link, and the test ingress, together.

What is real and what is simulated
----------------------------------
- REAL: the documented entry point (`python -m robotx.application`) as its own
  process; `BackendLink` with its genuine `socketio.AsyncClient`, Engine.IO
  handshake, AUTH, signature verification, fence/sequence admission and OFFER
  handling; the injection route and gate; the agent's own `assess_offer`.
- SIMULATED: the backend. `ContractServer` is a real `socketio.AsyncServer` on
  a loopback port speaking this implementation's reading of the contract -- it
  is not RobotX, and nothing here contacts RobotX. The OFFER is signed with the
  test-only `TEST_SIGNING_KEY`, configured into this throwaway process only.
- ABSENT: every piece of hardware (camera, GPS, ESP32 all disabled). The agent
  therefore genuinely cannot drive, and both a real OFFER and a test injection
  must be refused with `NO_MOTOR_LINK`. That is the property under test: with
  the link live, the ingress neither accepts what the agent refuses nor leaks
  anything to the backend that would imply RobotX assigned it.
"""

from __future__ import annotations

import asyncio
import json
import os
import secrets
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

from robotx.application.injection import token_digest
from tests.fixtures import engine as efx
from tests.integration.test_local_api_bind import REPO, free_port
from tests.integration.test_socketio_transport import VALID_PAIRING_CODE, ContractServer, setUpModule  # noqa: F401


TOKEN = secrets.token_hex(32)
OPERATOR_TOKEN = secrets.token_hex(32)


def http(port, method, path, payload=None, token=None):
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


def fixture():
    here, pickup, drop = (51.5, -0.1), (51.50036, -0.1), (51.50072, -0.1)

    def p(x):
        return {"lat": x[0], "lon": x[1]}

    return {"schema_version": 1, "test_run_id": "backend-process", "task_id": "TEST-INJECTED-BP-1",
            "robot_id": efx.ROBOT_ID, "region": {"center": p(here), "radius_m": 300},
            "pickup": p(pickup), "drop": p(drop),
            "path_to_pickup": [p(here), p(pickup)], "path_to_drop": [p(pickup), p(drop)]}


async def until(predicate, limit_s=30.0):
    deadline = time.monotonic() + limit_s
    while time.monotonic() < deadline:
        if await asyncio.to_thread(predicate):
            return True
        await asyncio.sleep(0.2)
    return False


class TestInjectionWithALiveBackendLink(unittest.TestCase):
    def test_real_link_real_process_concurrent_offer_and_injection(self):
        asyncio.run(self.scenario())

    async def scenario(self):
        server = ContractServer()
        await server.start()
        port = free_port()
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        log = Path(tmp.name) / "test_injections.jsonl"
        env = {k: v for k, v in os.environ.items() if not k.startswith("ROBOTX_")}
        env.update(
            ROBOTX_ROBOT_ID=efx.ROBOT_ID, ROBOTX_CAMERA_ENABLED="0", ROBOTX_PERCEPTION_ENABLED="0",
            ROBOTX_GPS_ENABLED="0", ROBOTX_LOG_LEVEL="INFO", ROBOTX_API_HOST="127.0.0.1", ROBOTX_API_PORT=str(port),
            ROBOTX_SOCKET_ENABLED="1", ROBOTX_SOCKET_SERVER_URL=server.url, ROBOTX_PAIRING_CODE=VALID_PAIRING_CODE,
            ROBOTX_BACKEND_TOKEN_PATH=str(Path(tmp.name) / "session.json"),
            ROBOTX_COMMITMENT_STATE_PATH=str(Path(tmp.name) / "commitments.json"),
            ROBOTX_COMMAND_SIGNING_KEY=efx.TEST_SIGNING_KEY, ROBOTX_CUSTODY_CONFIRMATION="operator",
            ROBOTX_TEST_INJECTION_ENABLED="1", ROBOTX_TEST_INJECTION_TOKEN_SHA256=token_digest(TOKEN),
            ROBOTX_TEST_INJECTION_LOG_PATH=str(log), ROBOTX_OPERATOR_TOKEN_SHA256=token_digest(OPERATOR_TOKEN),
        )
        output = Path(tmp.name) / "agent.log"
        with open(output, "wb") as sink:
            process = subprocess.Popen([sys.executable, "-m", "robotx.application"], cwd=str(REPO), env=env,
                                       stdout=sink, stderr=subprocess.STDOUT)
        try:
            def streaming():
                try:
                    return http(port, "GET", "/backend")[1].get("streaming") is True
                except OSError:
                    return False

            self.assertTrue(await until(streaming), output.read_text(errors="replace")[-2000:])
            self.assertTrue(server.authenticated_sids)
            self.assertIs(http(port, "GET", "/test/injection")[1]["available"], True)

            # The OFFER and the injection race on the agent's one event loop.
            envelope = efx.envelope(commitment_id="c-proc", outbox_id="ob-proc")
            reply, _ = await asyncio.gather(
                asyncio.to_thread(http, port, "POST", "/test/mission/inject", fixture(), TOKEN),
                server.send_engine(envelope),
            )
            self.assertTrue(await until(lambda: bool(server.received["OFFER_REJECT"]), 10.0))
            await asyncio.sleep(3.0)  # several telemetry and heartbeat periods
        finally:
            process.terminate()
            await asyncio.to_thread(process.wait, 15)
            await server.stop()

        status, body = reply
        self.assertEqual((status, body["outcome"], body["reason"]), (409, "REJECTED", "NO_MOTOR_LINK"))
        self.assertEqual([p["commitmentId"] for p in server.received["OFFER_REJECT"]], ["c-proc"])
        self.assertEqual(server.received["OFFER_REJECT"][0]["reason"], "NO_MOTOR_LINK")
        self.assertEqual([p["outboxId"] for p in server.received["COMMAND_ACK"]], ["ob-proc"])
        for name in ("OFFER_ACCEPT", "OFFER_DEFER", "CUSTODY_EVENT", "TASK_COMPLETE"):
            self.assertEqual(server.received[name], [], name)
        self.assertTrue(server.received["HEARTBEAT"])
        self.assertTrue(all(beat in ({}, None) for beat in server.received["HEARTBEAT"]),
                        server.received["HEARTBEAT"][:3])
        self.assertTrue(server.received["TELEMETRY"], "telemetry stopped")
        everything = json.dumps({k: v for k, v in server.received.items()})
        self.assertNotIn("TEST-INJECTED", everything)
        records = [json.loads(line) for line in log.read_text().splitlines()]
        self.assertEqual([r["event"] for r in records], ["ATTEMPTED", "REJECTED"])
        text = output.read_text(errors="replace")
        for secret in (TOKEN, token_digest(TOKEN), OPERATOR_TOKEN, token_digest(OPERATOR_TOKEN)):
            self.assertNotIn(secret, text)


if __name__ == "__main__":
    unittest.main()
