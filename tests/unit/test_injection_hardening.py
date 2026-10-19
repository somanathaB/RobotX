"""Hardening of the test-only ingress: concurrency with real OFFERs, torn logs, proxy headers.

What is real and what is simulated
----------------------------------
- REAL application code: `RobotAgent`, `MissionManager`, the safety gate,
  `Esp32Link`, `BackendLink` (signature, fence, sequence, OFFER handling),
  `InjectionGate`, the FastAPI route handlers, and uvicorn's own
  `ProxyHeadersMiddleware`.
- SIMULATED: the ESP32 serial port (`Rig`), GPS fixes and perception, the
  backend socket (`FakeSio`), and the OFFER itself -- signed with the test-only
  `tests.fixtures.engine.TEST_SIGNING_KEY`, never the backend's key.

No socket, serial port or hardware is touched.
"""

from __future__ import annotations

import asyncio
import json
import os
import tempfile
import unittest
from pathlib import Path

import httpx
from uvicorn.middleware.proxy_headers import ProxyHeadersMiddleware

import robotx.application.main as api
from robotx.application.injection import InjectionConfig, InjectionGate, token_digest
from robotx.communication.engine import OFFER
from tests.fixtures import engine as efx
from tests.unit.test_backend_link import FakeSio, connect_and_auth
from tests.unit.test_engine_offer import engine_link
from tests.unit.test_injection_endpoint import (
    INJECT, OPERATOR_TOKEN, ROBOT, TOKEN, acall, body, ready_rig, setUpModule,  # noqa: F401
)


class Case(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.log_path = Path(self.tmp.name) / "test_injections.jsonl"

    def gate(self, **overrides):
        values = dict(enabled=True, token_sha256=token_digest(TOKEN), log_path=str(self.log_path),
                      api_host="127.0.0.1", robot_id=ROBOT, operator_token_sha256=token_digest(OPERATOR_TOKEN))
        values.update(overrides)
        return InjectionGate(InjectionConfig(**values))


# --- concurrency with a real, signed OFFER -----------------------------------------------


class HookedSio(FakeSio):
    """A SIMULATED backend socket that can run a coroutine at the instant the Pi ACKs."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.on_ack = None

    async def emit(self, event, payload, namespace=None):
        await super().emit(event, payload, namespace)
        if event == "COMMAND_ACK" and self.on_ack is not None:
            hook, self.on_ack = self.on_ack, None
            await hook()


def efx_start():
    from tests.fixtures.task_assign import START

    return START


class TestConcurrencyWithBackendOffers(Case):
    """Exactly one mission wins; nothing is ever claimed for the one that lost."""

    def rig_and_link(self):
        rig = ready_rig()
        sio = HookedSio()
        link, _ = engine_link(agent=rig.agent, state=rig.agent.state, sio=sio)
        rig.agent.backend = link
        return rig, link, sio

    def offer(self):
        return efx.envelope(commitment_id="c-race", outbox_id="ob-race")

    def test_injection_first_then_offer_offer_is_rejected_honestly(self):
        rig, link, sio = self.rig_and_link()
        gate = self.gate()

        async def scenario():
            await connect_and_auth(link)
            status, _ = await acall(rig.agent, gate, "POST", INJECT, body())
            await sio.fire(OFFER, self.offer())
            return status

        self.assertEqual(asyncio.run(scenario()), 200)
        self.assertEqual(rig.agent.missions.active.task_id, "TEST-INJECTED-0001")
        self.assertEqual([p["reason"] for p in sio.events_named("OFFER_REJECT")], ["MISSION_IN_PROGRESS"])
        self.assertEqual(sio.events_named("OFFER_ACCEPT"), [])
        self.assertIsNone(link._held_mission())

    def test_offer_first_then_injection_is_refused_and_the_engine_mission_stands(self):
        rig, link, sio = self.rig_and_link()
        gate = self.gate()

        # The rig stands where the OFFER fixture's pathToPickup starts (route admission).
        rig.agent.gps.set_fix(*efx_start())
        rig.run(0.6)

        async def scenario():
            await connect_and_auth(link)
            await sio.fire(OFFER, self.offer())
            return await acall(rig.agent, gate, "POST", INJECT, body())

        status, reply = asyncio.run(scenario())
        self.assertEqual(len(sio.events_named("OFFER_ACCEPT")), 1)
        self.assertEqual((status, reply["code"]), (409, "BACKEND_COMMITMENT_HELD"))
        self.assertEqual(rig.agent.missions.active.task_id, "T-123")
        self.assertIsNotNone(link._held_mission())
        self.assertFalse(self.log_path.exists() and "ATTEMPTED" in self.log_path.read_text())

    def test_injection_landing_inside_the_offer_handlers_await(self):
        # The one await in the OFFER path between admission and the agent's
        # verdict is the COMMAND_ACK emit. An injection served exactly there
        # wins the Rover; the OFFER then sees it and is rejected, not accepted.
        rig, link, sio = self.rig_and_link()
        gate = self.gate()
        results = []

        async def inject_now():
            results.append(await acall(rig.agent, gate, "POST", INJECT, body()))

        async def scenario():
            await connect_and_auth(link)
            sio.on_ack = inject_now
            await sio.fire(OFFER, self.offer())

        asyncio.run(scenario())
        self.assertEqual(results[0][0], 200)
        self.assertEqual(rig.agent.missions.active.task_id, "TEST-INJECTED-0001")
        self.assertEqual([p["reason"] for p in sio.events_named("OFFER_REJECT")], ["MISSION_IN_PROGRESS"])
        self.assertEqual(sio.events_named("OFFER_ACCEPT"), [])
        self.assertIsNone(link._held_mission())

    def test_concurrent_requests_through_gather_yield_one_mission(self):
        rig, link, sio = self.rig_and_link()
        gate = self.gate()

        async def scenario():
            await connect_and_auth(link)
            return await asyncio.gather(
                acall(rig.agent, gate, "POST", INJECT, body(task_id="TEST-INJECTED-A")),
                sio.fire(OFFER, self.offer()),
                acall(rig.agent, gate, "POST", INJECT, body(task_id="TEST-INJECTED-B")),
            )

        a, _, b = asyncio.run(scenario())
        accepted = [r for r in (a, b) if r[0] == 200] + sio.events_named("OFFER_ACCEPT")
        self.assertEqual(len(accepted), 1, (a, b, sio.emitted))
        if sio.events_named("OFFER_ACCEPT"):
            self.assertIsNotNone(link._held_mission())
        else:
            self.assertIsNone(link._held_mission())
            self.assertTrue(rig.agent.missions.active.task_id.startswith("TEST-INJECTED-"))

    def test_a_bad_signature_is_still_not_admitted_with_injection_enabled(self):
        rig, link, sio = self.rig_and_link()
        self.gate()
        tampered = self.offer()
        tampered["payload"]["taskId"] = "T-TAMPERED"

        async def scenario():
            await connect_and_auth(link)
            await sio.fire(OFFER, tampered)

        asyncio.run(scenario())
        self.assertEqual(sio.events_named("COMMAND_ACK"), [])
        self.assertEqual(link.stats["engine_commands_not_admitted"], 1)
        self.assertIsNone(rig.agent.missions.active)


# --- torn provenance log, restart --------------------------------------------------------


class TestTornLogAndRestart(Case):
    def history(self, *records):
        self.log_path.write_text("".join(json.dumps(r) + "\n" for r in records))
        os.chmod(self.log_path, 0o600)

    def test_a_torn_final_record_is_set_aside_and_history_is_kept(self):
        self.history({"event": "ATTEMPTED", "task_id": "TEST-INJECTED-OLD"},
                     {"event": "ACCEPTED", "task_id": "TEST-INJECTED-OLD"})
        with open(self.log_path, "ab") as h:
            h.write(b'{"event": "ATTEMPTED", "task_id": "TEST-INJ')
        intact = len(self.log_path.read_bytes()) - len(b'{"event": "ATTEMPTED", "task_id": "TEST-INJ')

        with self.assertLogs("robotx.application.injection", level="WARNING") as logs:
            gate = self.gate()
        self.assertTrue(gate.available, gate.status())
        self.assertIn("torn_record_quarantined", "\n".join(logs.output))
        self.assertEqual(len(self.log_path.read_bytes()), intact)
        sides = list(self.log_path.parent.glob("test_injections.jsonl.torn-*"))
        self.assertEqual(len(sides), 1)
        self.assertEqual(sides[0].read_bytes(), b'{"event": "ATTEMPTED", "task_id": "TEST-INJ')
        self.assertEqual(oct(os.stat(sides[0]).st_mode & 0o777), "0o600")

        rig = ready_rig()
        status, reply = asyncio.run(acall(rig.agent, gate, "POST", INJECT, body(task_id="TEST-INJECTED-OLD")))
        self.assertEqual((status, reply["code"]), (409, "DUPLICATE_TASK"))
        self.assertEqual(asyncio.run(acall(rig.agent, gate, "POST", INJECT, body()))[0], 200)
        # The next restart reads a clean log.
        self.assertTrue(self.gate().available)
        self.assertEqual(len(list(self.log_path.parent.glob("*.torn-*"))), 1)

    def test_a_legible_torn_record_still_spends_its_task_id(self):
        self.history({"event": "ATTEMPTED", "task_id": "TEST-INJECTED-OLD"})
        with open(self.log_path, "ab") as h:
            h.write(b'{"event": "ATTEMPTED", "task_id": "TEST-INJECTED-0001"}')  # newline never written
        with self.assertLogs("robotx.application.injection", level="WARNING"):
            gate = self.gate()
        self.assertTrue(gate.available)
        status, reply = asyncio.run(acall(ready_rig().agent, gate, "POST", INJECT, body()))
        self.assertEqual((status, reply["code"]), (409, "DUPLICATE_TASK"))

    def test_a_corrupt_complete_line_still_refuses(self):
        self.log_path.write_text('{"event": "ATTEMPTED", "task_id": "TEST-INJECTED-1"}\n{oops}\n'
                                 '{"event": "ACCEPTED", "task_id": "TEST-INJECTED-1"}\n')
        with self.assertLogs("robotx.application.injection", level="ERROR"):
            gate = self.gate()
        self.assertFalse(gate.available)
        self.assertEqual(gate.status()["injected_task_ids"], 0)

    def test_a_torn_record_that_cannot_be_set_aside_refuses(self):
        self.history({"event": "ATTEMPTED", "task_id": "TEST-INJECTED-OLD"})
        with open(self.log_path, "ab") as h:
            h.write(b"{\"ev")
        os.chmod(self.log_path.parent, 0o500)
        self.addCleanup(os.chmod, self.log_path.parent, 0o700)
        with self.assertLogs("robotx.application.injection", level="ERROR"):
            gate = self.gate()
        self.assertFalse(gate.available)
        self.assertIn("torn record", gate.status()["problem"])

    def test_an_unwritable_log_is_known_at_start_not_on_the_first_request(self):
        read_only = Path(self.tmp.name) / "ro"
        read_only.mkdir()
        os.chmod(read_only, 0o500)
        self.addCleanup(os.chmod, read_only, 0o700)
        with self.assertLogs("robotx.application.injection", level="ERROR"):
            gate = self.gate(log_path=str(read_only / "test_injections.jsonl"))
        self.assertFalse(gate.available)
        self.assertIn("cannot be written", gate.status()["problem"])

    def test_restart_keeps_every_spent_task_id(self):
        rig = ready_rig()
        gate = self.gate()
        self.assertEqual(asyncio.run(acall(rig.agent, gate, "POST", INJECT, body()))[0], 200)
        for _ in range(3):  # three restarts
            gate = self.gate()
            status, reply = asyncio.run(acall(ready_rig().agent, gate, "POST", INJECT, body()))
            self.assertEqual((status, reply["code"]), (409, "DUPLICATE_TASK"))


# --- proxy headers through uvicorn's real middleware ---------------------------------------


class TestProxyHeaders(Case):
    """`uvicorn.run` enables ProxyHeadersMiddleware by default (trusting 127.0.0.1)."""

    def post(self, *, trusted, client, server_host, forwarded_for, host_header="127.0.0.1:8000"):
        rig = ready_rig()
        gate = self.gate()
        app = ProxyHeadersMiddleware(api.app, trusted_hosts=trusted)

        async def go():
            api.state.agent, api.state.injection = rig.agent, gate
            try:
                transport = httpx.ASGITransport(app=app, client=(client, 40000))
                async with httpx.AsyncClient(transport=transport, base_url=f"http://{server_host}:8000") as c:
                    r = await c.post(INJECT, json=body(), headers={
                        "Authorization": f"Bearer {TOKEN}", "X-Forwarded-For": forwarded_for,
                        "Host": host_header})
                    return r.status_code, r.json()
            finally:
                api.state.agent = api.state.injection = None

        return asyncio.run(go()), rig

    def test_a_forwarded_for_from_loopback_naming_elsewhere_is_refused(self):
        (status, reply), rig = self.post(trusted="127.0.0.1", client="127.0.0.1", server_host="127.0.0.1",
                                         forwarded_for="10.0.0.9")
        self.assertEqual((status, reply["code"]), (403, "NOT_LOOPBACK"))
        self.assertIsNone(rig.agent.missions.active)

    def test_an_untrusted_peer_cannot_claim_loopback(self):
        (status, reply), rig = self.post(trusted="127.0.0.1", client="192.168.1.20", server_host="192.168.1.8",
                                         forwarded_for="127.0.0.1")
        self.assertEqual((status, reply["code"]), (403, "NOT_LOOPBACK"))
        self.assertIsNone(rig.agent.missions.active)

    def test_even_trusting_every_proxy_a_lan_socket_is_refused(self):
        # FORWARDED_ALLOW_IPS="*" makes uvicorn believe any X-Forwarded-For. The
        # connection's own local address still says where it arrived.
        (status, reply), rig = self.post(trusted="*", client="192.168.1.20", server_host="192.168.1.8",
                                         forwarded_for="127.0.0.1")
        self.assertEqual((status, reply["code"]), (403, "NOT_LOOPBACK"))
        self.assertIsNone(rig.agent.missions.active)

    def test_a_forwarded_host_header_is_not_honoured(self):
        rig = ready_rig()
        (status, reply), rig = self.post(trusted="*", client="127.0.0.1", server_host="127.0.0.1",
                                         forwarded_for="127.0.0.1", host_header="attacker.example:8000")
        self.assertEqual((status, reply["code"]), (403, "HOST_NOT_LOOPBACK"))


if __name__ == "__main__":
    unittest.main()
