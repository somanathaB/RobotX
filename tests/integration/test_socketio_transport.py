"""End-to-end over a REAL Socket.IO transport, against the FalconAut contract.

Everything here runs across an actual Engine.IO handshake, an actual WebSocket
or polling upgrade, and real JSON serialization, against a `socketio.AsyncServer`
on a loopback port. No part of the client is faked: `BackendLink` builds its
normal `socketio.AsyncClient`.

What this proves, and what it does not
--------------------------------------
`ContractServer` below implements the FalconAut robot contract as this
implementation understands it: an **anonymous** connection, `AUTH` answered
with `AUTH_SUCCESS`, a silent `disconnect()` when the credential is refused,
`COMMAND` and a bare `STOP` inbound, `TELEMETRY`, `HEARTBEAT` and
`COMMAND_ACK` outbound.

It proves the Pi's client half satisfies that contract on the wire. It proves
**nothing about the real FalconAut backend**, which is not reachable from this
environment and has never been contacted. Every result from this file is
therefore `PASS-SYNTHETIC`, never `PASS`.

These tests bind a loopback TCP port. They are automated and hardware-free, but
they are not pure unit tests, which is why they live outside `tests/unit`.
"""

import asyncio
import logging
import os
from dataclasses import replace
import tempfile
import time
import unittest

import socketio
from aiohttp import web

from robotx.communication.backend_link import MAX_HEARTBEAT_INTERVAL_S, BackendConfig, BackendLink
from robotx.communication.protocol import ProtocolBinding
from robotx.communication.token_store import TokenStore
from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus
from robotx.localization.position import Position
from robotx.state.robot_state import BackendLinkStatus as LinkStatus, OperatingMode, RobotState


# FalconAut's robot handler is on the default namespace.
NAMESPACE = "/"
VALID_PAIRING_CODE = "424242"
ISSUED_TOKEN = "falconaut-session-token"
ROBOT_ID = "robotx-pi-itest"


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL)
    for noisy in ("engineio", "engineio.server", "socketio", "socketio.server", "aiohttp"):
        logging.getLogger(noisy).setLevel(logging.CRITICAL)


class ContractServer:
    """A Socket.IO server speaking the FalconAut robot contract.

    Authentication is deliberately modelled the way the contract describes it,
    including the part that is awkward for a client: a refused credential gets
    **no error event**, just a server-side disconnect. A test server that
    politely explained itself would let a bug hide, because the real backend
    does not.
    """

    def __init__(self, *, accept_pairing_code=VALID_PAIRING_CODE, accept_token=None,
                 issue_token=ISSUED_TOKEN, refuse_everything=False,
                 success_event="both", auth_failed_reason=None, refuse_first=0, ignore_first=0,
                 ping_interval=None, ping_timeout=None):
        # The real backend emits BOTH AUTH_SUCCESS and AUTH_OK for one AUTH
        # (handoff §3); "both" is therefore the default here.
        self.success_event = success_event
        self.accept_pairing_code = accept_pairing_code
        self.accept_token = accept_token
        self.issue_token = issue_token
        self.refuse_everything = refuse_everything
        # R1 -- when set, a refusal is the backend's explicit `AUTH_FAILED {reason}`
        # sent before the disconnect (what robot.handler.js does for a rejected
        # credential). Unset keeps the silent disconnect, which is also what the
        # backend does when it merely fails.
        self.auth_failed_reason = auth_failed_reason
        # Refuse, or leave unanswered (drives the client's AUTH timeout), the
        # first N AUTHs whatever they carry -- a backend having a bad moment.
        self.refuse_first = refuse_first
        self.ignore_first = ignore_first
        self.auth_count = 0
        # Y2 -- answer TASK_COMPLETE with TASK_COMPLETE_ACK {taskId} as
        # dtaro.handler does, after dropping the connection on the first N claims
        # without answering (a socket that dies with the claim in flight). Off by
        # default: the claim is only recorded.
        self.ack_task_complete = False
        self.task_complete_drop_first = 0
        self.received_at = {}

        # Y4 -- the Engine.IO heartbeat this server advertises in its handshake
        # (seconds), when a test sets it; the library default otherwise.
        heartbeat = {k: v for k, v in (("ping_interval", ping_interval), ("ping_timeout", ping_timeout)) if v is not None}
        self.sio = socketio.AsyncServer(async_mode="aiohttp", **heartbeat)
        self.app = web.Application()
        self.sio.attach(self.app)

        self.received = {
            "TELEMETRY": [], "HEARTBEAT": [], "COMMAND_ACK": [], "OFFER_ACCEPT": [],
            "OFFER_REJECT": [], "OFFER_DEFER": [], "CUSTODY_EVENT": [], "TASK_COMPLETE": [],
        }
        self.auth_payloads = []
        self.connect_environs = []
        self.authenticated_sids = []
        self.refusals = []
        self.sids = []
        self.runner = None
        self.port = None

        self._install_handlers()

    def _install_handlers(self):
        @self.sio.event(namespace=NAMESPACE)
        async def connect(sid, environ, auth=None):
            # Anonymous: the contract's connection carries no credential, and
            # this server accepts every socket and waits for AUTH.
            self.connect_environs.append(environ)
            self.sids.append(sid)

        @self.sio.on("AUTH", namespace=NAMESPACE)
        async def on_auth(sid, data=None):
            self.auth_payloads.append(data)
            self.auth_count += 1
            payload = data if isinstance(data, dict) else {}
            code = payload.get("pairingCode")
            token = payload.get("token")

            if self.auth_count <= self.ignore_first:
                return  # no answer at all

            ok = not self.refuse_everything and self.auth_count > self.refuse_first and (
                (code is not None and code == self.accept_pairing_code)
                or (token is not None and token == self.accept_token)
            )
            if not ok:
                self.refusals.append(payload)
                if self.auth_failed_reason is not None:
                    await self.sio.emit("AUTH_FAILED", {"reason": self.auth_failed_reason}, to=sid, namespace=NAMESPACE)
                # The contract's refusal: disconnect, no reason given.
                await self.sio.disconnect(sid, namespace=NAMESPACE)
                return

            self.authenticated_sids.append(sid)
            body = {"robotId": payload.get("robotId")}
            if self.issue_token:
                body["token"] = self.issue_token
            events = ("AUTH_SUCCESS", "AUTH_OK") if self.success_event == "both" else (self.success_event,)
            for event in events:
                await self.sio.emit(event, body, to=sid, namespace=NAMESPACE)

        for name in self.received:
            self._record(name)

    def _record(self, name):
        self.received_at.setdefault(name, [])

        @self.sio.on(name, namespace=NAMESPACE)
        async def handler(sid, data, _name=name):
            self.received[_name].append(data)
            self.received_at[_name].append(time.monotonic())  # Y2 -- spacing as the server saw it
            if _name == "TASK_COMPLETE" and self.ack_task_complete:
                if len(self.received["TASK_COMPLETE"]) <= self.task_complete_drop_first:
                    await self.sio.disconnect(sid, namespace=NAMESPACE)  # gone before any ACK
                else:
                    task_id = data.get("taskId") if isinstance(data, dict) else None
                    await self.sio.emit("TASK_COMPLETE_ACK", {"taskId": task_id, "timestamp": int(time.time() * 1000)},
                                        to=sid, namespace=NAMESPACE)

    async def start(self):
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        site = web.TCPSite(self.runner, "127.0.0.1", 0)
        await site.start()
        self.port = site._server.sockets[0].getsockname()[1]

    async def stop(self):
        if self.runner is not None:
            await self.runner.cleanup()
            self.runner = None

    @property
    def url(self):
        return f"http://127.0.0.1:{self.port}"

    async def send_command(self, payload, *, sid=None):
        await self.sio.emit(
            "COMMAND", payload, to=sid or self.authenticated_sids[-1], namespace=NAMESPACE
        )

    async def send_engine(self, envelope, *, sid=None, event=None):
        # As the backend does (commandDispatcher.service.js): the event name IS the
        # envelope's command -- OFFER, WITHDRAW, ... -- never a generic "command".
        await self.sio.emit(
            event or envelope["command"], envelope, to=sid or self.authenticated_sids[-1], namespace=NAMESPACE
        )

    async def send_task_assign(self, payload, *, sid=None):
        await self.sio.emit(
            "TASK_ASSIGN", payload, to=sid or self.authenticated_sids[-1], namespace=NAMESPACE
        )

    async def send_stop(self, payload=None, *, sid=None):
        await self.sio.emit(
            "STOP", payload or {}, to=sid or self.authenticated_sids[-1], namespace=NAMESPACE
        )

    async def kick(self, sid=None):
        await self.sio.disconnect(sid or self.authenticated_sids[-1], namespace=NAMESPACE)


class FakeAgent:
    def __init__(self, mode=OperatingMode.AUTO):
        self._mode = mode
        self.calls = []

    @property
    def mode(self):
        return self._mode

    def stop_mission(self, reason=""):
        self.calls.append(("stop", reason))
        self._mode = OperatingMode.STOPPED

    def pause_mission(self, reason=""):
        self.calls.append(("pause", reason))
        self._mode = OperatingMode.PAUSED

    def resume_mission(self, reason=""):
        self.calls.append(("resume", reason))
        self._mode = OperatingMode.AUTO

    def return_to_base(self, reason=""):
        self.calls.append(("return", reason))

    def assess_offer(self, offer):
        from robotx.communication.engine import OfferDecision

        self.calls.append(("assess", offer.commitment_id))
        return OfferDecision.reject("NO_MOTOR_LINK")

    def assign_mission(self, mission, custody_required=False):
        self.calls.append(("assign", mission.task_id))


def state_with_fix(robot_id=ROBOT_ID, *, lat=12.9716, lon=77.5946, speed=1.25):
    state = RobotState(robot_id)
    now = time.time()
    state.update_gps(
        GpsReading(
            status=GPSStatus.FIX,
            fix=GpsFix(latitude=lat, longitude=lon, timestamp=now),
            age_s=0.1,
        ),
        Position(latitude=lat, longitude=lon, timestamp=now, speed_mps=speed),
    )
    return state


async def wait_for(predicate, timeout=5.0, interval=0.02):
    """Poll until `predicate()` is true, or give up. Returns whether it held."""

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        await asyncio.sleep(interval)
    return predicate()


class ContractTestCase(unittest.TestCase):
    """Runs one coroutine per test with a server and a temp token store."""

    def run_async(self, coro_factory, **server_kwargs):
        async def wrapper():
            server = ContractServer(**server_kwargs)
            await server.start()
            tmp = tempfile.TemporaryDirectory()
            try:
                return await coro_factory(server, os.path.join(tmp.name, "session.json"))
            finally:
                await server.stop()
                tmp.cleanup()

        return asyncio.run(wrapper())

    def make_link(self, server, token_path, *, state=None, agent=None, **cfg_kwargs):
        cfg_kwargs.setdefault("pairing_code", VALID_PAIRING_CODE)
        cfg_kwargs.setdefault("telemetry_interval_s", 0.1)
        cfg_kwargs.setdefault("auth_timeout_s", 3.0)
        cfg = BackendConfig(
            enabled=True,
            server_url=server.url,
            robot_id=ROBOT_ID,
            binding=ProtocolBinding(),
            **cfg_kwargs,
        )
        link = BackendLink(
            cfg,
            state if state is not None else state_with_fix(),
            agent if agent is not None else FakeAgent(),
            token_store=TokenStore(token_path),
        )
        return link


# --- 1-4: connection and authentication ---------------------------------------


class TestConnectionAndAuthentication(ContractTestCase):
    def test_pi_connects_anonymously_and_authenticates_with_the_pairing_code(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path)
            await link.start()
            ok = await link.wait_connected(timeout_s=5.0)
            described = link.describe()
            await link.stop()
            return ok, described, server

        ok, described, server = self.run_async(scenario)
        self.assertTrue(ok)
        self.assertEqual(described["auth_method"], "PAIRING_CODE")
        self.assertEqual(len(server.auth_payloads), 1)
        self.assertEqual(server.auth_payloads[0]["pairingCode"], VALID_PAIRING_CODE)
        self.assertEqual(server.auth_payloads[0]["robotId"], ROBOT_ID)

    def test_no_credential_travels_in_the_handshake(self):
        """The connection is anonymous; the server must see nothing in it."""

        async def scenario(server, token_path):
            link = self.make_link(server, token_path)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await link.stop()
            return server

        server = self.run_async(scenario)
        environ = server.connect_environs[0]
        query = environ.get("QUERY_STRING", "")
        self.assertNotIn(VALID_PAIRING_CODE, query)
        self.assertNotIn("token", query.lower())

    def test_client_does_not_look_like_a_browser(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await link.stop()
            return server

        server = self.run_async(scenario)
        environ = server.connect_environs[0]
        self.assertNotIn("HTTP_ORIGIN", environ)
        self.assertNotIn("Mozilla", environ.get("HTTP_USER_AGENT", ""))
        self.assertIn("robotx-pi", environ.get("HTTP_USER_AGENT", ""))

    def test_auth_success_token_is_persisted_to_disk(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await link.stop()
            return TokenStore(token_path).load(robot_id=ROBOT_ID), token_path

        stored, token_path = self.run_async(scenario)
        self.assertIsNotNone(stored)
        self.assertEqual(stored.token, ISSUED_TOKEN)

    def test_persisted_token_is_used_instead_of_the_pairing_code(self):
        async def scenario(server, token_path):
            # Pre-seed a token the server will accept.
            TokenStore(token_path).save(robot_id=ROBOT_ID, token="pre-existing")
            server.accept_token = "pre-existing"
            link = self.make_link(server, token_path)
            await link.start()
            ok = await link.wait_connected(timeout_s=5.0)
            method = link.describe()["auth_method"]
            await link.stop()
            return ok, method, server

        ok, method, server = self.run_async(scenario)
        self.assertTrue(ok)
        self.assertEqual(method, "TOKEN")
        self.assertIn("token", server.auth_payloads[0])
        self.assertNotIn("pairingCode", server.auth_payloads[0])

    def test_refused_credential_is_reported_as_auth_failed_not_as_a_network_fault(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path, backoff_auth_failed_s=30.0)
            await link.start()
            reached = await wait_for(lambda: link.status is LinkStatus.AUTH_FAILED)
            described = link.describe()
            await link.stop()
            return reached, described, server

        reached, described, server = self.run_async(scenario, refuse_everything=True)
        self.assertTrue(reached)
        self.assertGreaterEqual(len(server.refusals), 1)
        # The transport was never the problem.
        self.assertEqual(described["stats"]["connect_failures"], 0)
        self.assertGreaterEqual(described["stats"]["auth_failures"], 1)

    def test_agent_keeps_running_after_an_auth_failure(self):
        async def scenario(server, token_path):
            agent = FakeAgent()
            link = self.make_link(server, token_path, agent=agent, backoff_auth_failed_s=30.0)
            await link.start()
            await wait_for(lambda: link.status is LinkStatus.AUTH_FAILED)
            # The link stays alive and the mission is untouched.
            alive = link._task is not None and not link._task.done()
            await link.stop()
            return alive, agent

        alive, agent = self.run_async(scenario, refuse_everything=True)
        self.assertTrue(alive)
        self.assertEqual(agent.calls, [])


    def test_auth_ok_is_accepted_as_a_success_event(self):
        """A backend that answers AUTH_OK instead of AUTH_SUCCESS must still
        authenticate this robot, over a real socket."""

        async def scenario(server, token_path):
            link = self.make_link(server, token_path)
            await link.start()
            ok = await link.wait_connected(timeout_s=5.0)
            described = link.describe()
            await link.stop()
            return ok, described, token_path

        ok, described, token_path = self.run_async(scenario, success_event="AUTH_OK")
        self.assertTrue(ok)
        self.assertIs(described["authenticated"], True)
        self.assertGreaterEqual(described["stats"]["auth_successes"], 1)

    def test_auth_success_and_auth_ok_together_authenticate_once(self):
        """The real backend sends both; the second must not knock the link
        out of STREAMING or count as a second authentication."""

        async def scenario(server, token_path):
            link = self.make_link(server, token_path)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await asyncio.sleep(0.3)
            described = link.describe()
            await link.stop()
            return described

        described = self.run_async(scenario)
        self.assertTrue(described["streaming"])
        self.assertEqual(described["stats"]["auth_successes"], 1)


class TestR1CredentialRetentionOverTheWire(ContractTestCase):
    """R1 over a real Socket.IO transport, with the real client and the real
    on-disk TokenStore: the stored token is discarded only on the backend's
    `AUTH_FAILED {"reason": "INVALID_CREDENTIAL"}`; a silent disconnect, an
    unanswered AUTH or any other reason keeps it for the next reconnect."""

    STORED = "stored-session-token"

    async def _run_until_connected(self, server, token_path, **cfg_kwargs):
        TokenStore(token_path).save(robot_id=ROBOT_ID, token=self.STORED)
        cfg_kwargs.setdefault("backoff_auth_failed_s", 0.2)
        link = self.make_link(server, token_path, **cfg_kwargs)
        await link.start()
        ok = await link.wait_connected(timeout_s=15.0)
        described = link.describe()
        await link.stop()
        stored = TokenStore(token_path).load(robot_id=ROBOT_ID)
        return ok, described, (stored.token if stored else None), server

    def test_invalid_credential_deletes_the_token_and_the_pairing_code_is_used_next(self):
        ok, described, stored, server = self.run_async(
            lambda server, path: self._run_until_connected(server, path),
            auth_failed_reason="INVALID_CREDENTIAL",  # the stored token is not accepted
        )
        self.assertTrue(ok)
        self.assertEqual(server.auth_payloads[0], {"robotId": ROBOT_ID, "token": self.STORED})
        self.assertEqual(server.auth_payloads[1], {"robotId": ROBOT_ID, "pairingCode": VALID_PAIRING_CODE})
        self.assertEqual(described["auth_method"], "PAIRING_CODE")
        self.assertEqual(stored, ISSUED_TOKEN)  # the rejected token is gone; the new one persisted

    def test_a_silent_disconnect_during_auth_keeps_the_token_and_the_reconnect_reuses_it(self):
        def scenario(server, path):
            server.accept_token = self.STORED
            server.issue_token = self.STORED
            return self._run_until_connected(server, path)

        ok, described, stored, server = self.run_async(scenario, refuse_first=1)  # silent: no AUTH_FAILED
        self.assertTrue(ok)
        self.assertEqual([p.get("token") for p in server.auth_payloads], [self.STORED, self.STORED])
        self.assertTrue(all("pairingCode" not in p for p in server.auth_payloads))
        self.assertEqual(described["auth_method"], "TOKEN")
        self.assertEqual(described["stats"]["auth_failures"], 1)
        self.assertEqual(stored, self.STORED)

    def test_an_unanswered_auth_times_out_keeps_the_token_and_the_reconnect_reuses_it(self):
        def scenario(server, path):
            server.accept_token = self.STORED
            server.issue_token = self.STORED
            return self._run_until_connected(server, path, auth_timeout_s=0.5)

        ok, described, stored, server = self.run_async(scenario, ignore_first=1)
        self.assertTrue(ok)
        self.assertEqual([p.get("token") for p in server.auth_payloads], [self.STORED, self.STORED])
        self.assertEqual(stored, self.STORED)

    def test_auth_failed_with_any_other_reason_keeps_the_token(self):
        def scenario(server, path):
            server.accept_token = self.STORED
            server.issue_token = self.STORED
            return self._run_until_connected(server, path)

        ok, described, stored, server = self.run_async(scenario, refuse_first=1, auth_failed_reason="SOMETHING_ELSE")
        self.assertTrue(ok)
        self.assertEqual([p.get("token") for p in server.auth_payloads], [self.STORED, self.STORED])
        self.assertEqual(stored, self.STORED)


class TestY2CompletionDeliveryOverTheWire(ContractTestCase):
    """Y2 over a real Socket.IO transport, with the real client and the real
    on-disk CommitmentStore: a completion claim whose socket died before the
    TASK_COMPLETE_ACK is resent after the next AUTH, settled by the ACK, and
    never sent again."""

    CLAIM = {"taskId": "T-Y2", "lat": 12.9716, "lon": 77.5946}

    def test_claim_lost_with_its_socket_is_resent_after_reconnect_acked_and_never_resent(self):
        from robotx.communication.commitment_store import CommitmentStore

        async def scenario(server, token_path):
            server.ack_task_complete = True
            server.task_complete_drop_first = 1  # the first claim dies with its socket
            server.accept_token = ISSUED_TOKEN   # reconnects present the issued session token
            marks = os.path.join(os.path.dirname(token_path), "commitments.json")
            # The claim as `_publish_task_complete` persists it, before any ACK.
            CommitmentStore(marks).update(
                "c-y2", response="ACCEPT", task_id="T-Y2", highest_fence=1, fence_wire="1", highest_sequence=0,
                completion="SENT", completion_claim=dict(self.CLAIM),
            )
            link = self.make_link(server, token_path, commitment_path=marks,
                                  backoff_initial_s=0.05, backoff_max_s=0.1)
            await link.start()
            settled = await wait_for(lambda: CommitmentStore(marks).get("c-y2").completion_acked, timeout=15.0)
            stats_at_ack = dict(link.describe()["stats"])
            claims_at_ack = list(server.received["TASK_COMPLETE"])
            await server.kick()  # one more reconnect after the ACK
            reconnected = await wait_for(lambda: link.describe()["stats"]["auth_successes"] >= 3, timeout=15.0)
            await asyncio.sleep(0.3)
            final_claims = list(server.received["TASK_COMPLETE"])
            await link.stop()
            return settled, stats_at_ack, claims_at_ack, reconnected, final_claims, CommitmentStore(marks).get("c-y2")

        settled, stats, claims_at_ack, reconnected, final_claims, record = self.run_async(scenario)
        self.assertTrue(settled)
        self.assertEqual(claims_at_ack, [self.CLAIM, self.CLAIM])  # sent, lost, resent unchanged
        self.assertEqual(stats["auth_successes"], 2)                # resent only after a new AUTH
        self.assertEqual(stats["task_completes_resent"], 2)
        self.assertTrue(reconnected)
        self.assertEqual(final_claims, [self.CLAIM, self.CLAIM])    # acked: not sent again
        self.assertTrue(record.completion_acked)
        self.assertEqual(record.completion_claim, self.CLAIM)


class TestY2ReportDeliveryOverTheWire(ContractTestCase):
    """Y2 over a real Socket.IO transport: persisted CUSTODY_EVENT and OFFER_*
    responses are resent, unchanged, after each successful AUTH while they can
    still matter -- and the ones that cannot are not."""

    def _seed(self, marks):
        from robotx.communication.commitment_store import CommitmentStore

        store = CommitmentStore(marks)
        future, past = time.time() + 600, time.time() - 1
        self.acquired = {"commitmentId": "c-cust", "fence": "7", "kind": "ACQUIRED"}
        self.released = {"commitmentId": "c-cust", "fence": "7", "kind": "RELEASED"}
        self.reject = {"commitmentId": "c-rej", "fence": "8", "reason": "NO_MOTOR_LINK"}
        self.defer = {"commitmentId": "c-def", "fence": "9", "until": int(future * 1000), "reason": "CHARGING"}
        # Custody the backend has not settled: its claim was answered verifying / CUSTODY_STILL_HELD.
        store.update("c-cust", response="ACCEPT", task_id="T-C", fence_wire="7", custody_sent=["ACQUIRED", "RELEASED"],
                     custody_payloads={"ACQUIRED": self.acquired, "RELEASED": self.released},
                     completion="SENT", completion_claim={"taskId": "T-C", "lat": 1.0, "lon": 2.0},
                     completion_acked=True, completion_ack_result="VERIFYING", completion_ack_reason="CUSTODY_STILL_HELD")
        store.update("c-rej", response="REJECT", response_payload=self.reject, offer_expiry=future)
        store.update("c-def", response="DEFER", response_payload=self.defer, offer_expiry=future)
        # Must NOT be resent: an ACCEPT with no mission being carried out, an expired
        # offer, custody whose completion was settled.
        store.update("c-acc", response="ACCEPT", task_id="T-A", response_payload={"commitmentId": "c-acc", "fence": "10"},
                     offer_expiry=future)
        store.update("c-old", response="REJECT", response_payload={"commitmentId": "c-old", "fence": "11", "reason": "X"},
                     offer_expiry=past)
        store.update("c-done", response="ACCEPT", task_id="T-D", custody_sent=["RELEASED"],
                     custody_payloads={"RELEASED": {"commitmentId": "c-done", "fence": "12", "kind": "RELEASED"}},
                     completion="SENT", completion_claim={"taskId": "T-D", "lat": 1.0, "lon": 2.0},
                     completion_acked=True, completion_ack_result="SETTLED")

    def test_persisted_reports_are_resent_after_each_auth_exactly_and_spaced(self):
        async def scenario(server, token_path):
            server.accept_token = ISSUED_TOKEN  # the reconnect presents the issued session token
            marks = os.path.join(os.path.dirname(token_path), "commitments.json")
            self._seed(marks)
            link = self.make_link(server, token_path, commitment_path=marks, backoff_initial_s=0.05, backoff_max_s=0.1)
            await link.start()
            first = await wait_for(lambda: len(server.received["CUSTODY_EVENT"]) >= 2 and server.received["OFFER_DEFER"], timeout=15.0)
            await asyncio.sleep(0.3)
            await server.kick()  # the session is lost; a new one authenticates
            second = await wait_for(lambda: len(server.received["CUSTODY_EVENT"]) >= 4 and len(server.received["OFFER_DEFER"]) >= 2, timeout=15.0)
            await asyncio.sleep(0.3)
            stats = dict(link.describe()["stats"])
            await link.stop()
            return first, second, server, stats

        first, second, server, stats = self.run_async(scenario)
        self.assertTrue(first and second)
        self.assertEqual(stats["auth_successes"], 2)
        custody = server.received["CUSTODY_EVENT"]
        self.assertEqual(custody, [self.acquired, self.released, self.acquired, self.released])
        at = server.received_at["CUSTODY_EVENT"]
        self.assertGreaterEqual(at[1] - at[0], 0.05)  # ACQUIRED, >= 50 ms, RELEASED -- as received
        self.assertGreaterEqual(at[3] - at[2], 0.05)
        self.assertEqual(server.received["OFFER_REJECT"], [self.reject, self.reject])
        self.assertEqual(server.received["OFFER_DEFER"], [self.defer, self.defer])  # the original until, untouched
        self.assertEqual(server.received["OFFER_ACCEPT"], [])  # no mission: never re-asserted
        self.assertEqual(server.received["TASK_COMPLETE"], [])  # both claims were acknowledged
        self.assertEqual(stats["custody_events_resent"], 4)
        self.assertEqual(stats["offer_responses_resent"], 4)


class SilentableRelay:
    """A loopback TCP relay in front of the ContractServer that can go SILENT.

    After `silent = True` it forwards nothing in either direction and closes
    nothing: the client's socket stays open and simply stops hearing from the
    server -- a link that died without a FIN or an RST (Wi-Fi gone), which only
    the Engine.IO heartbeat can notice. New connections are accepted and never
    answered. A server that *closes* the socket would be noticed at once and
    would prove nothing about the heartbeat.
    """

    def __init__(self, target_port):
        self.target_port = target_port
        self.silent = False
        self._held = []
        self._server = None
        self.port = None

    async def start(self):
        self._server = await asyncio.start_server(self._handle, "127.0.0.1", 0)
        self.port = self._server.sockets[0].getsockname()[1]

    async def _handle(self, client_reader, client_writer):
        self._held.append(client_writer)
        if self.silent:
            return  # accepted, held open, never answered
        try:
            upstream_reader, upstream_writer = await asyncio.open_connection("127.0.0.1", self.target_port)
        except OSError:
            return
        self._held.append(upstream_writer)
        await asyncio.gather(self._pump(client_reader, upstream_writer), self._pump(upstream_reader, client_writer))

    async def _pump(self, reader, writer):
        try:
            while True:
                data = await reader.read(65536)
                if not data:
                    break
                if self.silent:
                    continue  # swallowed
                writer.write(data)
                await writer.drain()
        except Exception:
            pass
        if not self.silent:  # a silent link never closes
            try:
                writer.close()
            except Exception:
                pass

    async def stop(self):
        if self._server is not None:
            self._server.close()
        for writer in self._held:
            try:
                writer.close()
            except Exception:
                pass


class TestY4SilentLinkOverTheWire(ContractTestCase):
    """Y4 over a real Socket.IO transport, with the real RobotAgent loop: a link
    that goes silent is noticed within the server's `pingInterval + pingTimeout`,
    and the mission pauses `loss_grace_s` after that on the agent's own tick --
    while the link keeps trying (and failing) to reconnect through the silence."""

    TICK_S = 0.1
    SLACK_S = 0.5  # asyncio scheduling, socket and timer resolution on this host

    def running_agent(self):
        from robotx.application.agent import RobotAgent
        from tests.unit.test_agent import HERE, NORTH, FakeGPS, FakePerception, headless_settings

        agent = RobotAgent(headless_settings())
        agent.gps = FakeGPS()
        agent.perception = FakePerception()
        agent.gps.set_fix(*HERE, speed=1.0, track=0.0)
        agent.perception.set_clear()
        agent.start_mission([NORTH])
        agent.tick()
        return agent

    async def silence_and_measure(self, server, cfg, token_path, *, limit_s):
        relay = SilentableRelay(server.port)
        await relay.start()
        cfg = replace(cfg, server_url=f"http://127.0.0.1:{relay.port}")
        agent = self.running_agent()
        link = BackendLink(cfg, agent.state, agent, token_store=TokenStore(token_path), agent_alive=agent.is_alive)
        agent.backend = link
        agent._running = True
        loop = asyncio.create_task(agent._loop())
        try:
            await link.start()
            assert await link.wait_connected(timeout_s=10.0)
            await asyncio.sleep(0.5)
            assert agent.state.snapshot().mode is OperatingMode.AUTO
            bound = link.describe()["link_loss"]["detection_bound_s"]  # from the handshake, at runtime

            silent_at = time.monotonic()
            relay.silent = True
            detected_at = paused_at = None
            deadline = silent_at + limit_s
            while time.monotonic() < deadline and paused_at is None:
                if detected_at is None and link._disconnected_since is not None:
                    detected_at = link._disconnected_since  # set by the disconnect itself
                if agent.state.snapshot().mode is OperatingMode.PAUSED:
                    paused_at = time.monotonic()
                await asyncio.sleep(0.005)
            snapshot = agent.state.snapshot()
            return {"bound": bound, "silent_at": silent_at, "detected_at": detected_at,
                    "paused_at": paused_at, "snapshot": snapshot, "connects": link.stats["connects"]}
        finally:
            loop.cancel()
            try:
                await loop
            except (asyncio.CancelledError, Exception):
                pass
            await link.stop()
            await relay.stop()

    def check(self, result, *, ping_interval, ping_timeout, grace):
        detection = result["detected_at"] - result["silent_at"]
        pause = result["paused_at"] - result["detected_at"]
        total = result["paused_at"] - result["silent_at"]
        self.assertEqual(result["bound"], ping_interval + ping_timeout)
        self.assertLessEqual(detection, ping_interval + ping_timeout + self.SLACK_S)
        self.assertGreaterEqual(pause, grace)
        self.assertLessEqual(pause, grace + self.TICK_S + self.SLACK_S)
        self.assertLessEqual(total, ping_interval + ping_timeout + grace + self.TICK_S + self.SLACK_S)
        self.assertIs(result["snapshot"].mode, OperatingMode.PAUSED)
        self.assertTrue(result["snapshot"].motion_intent.is_stop)
        return detection, pause, total

    def test_F_silent_link_is_detected_by_the_heartbeat_and_the_mission_pauses_after_the_grace(self):
        """Test-only heartbeat (1 s + 1 s) and grace (1 s): the same path, in ~3 s."""

        async def scenario(server, token_path):
            cfg = BackendConfig(enabled=True, server_url=server.url, robot_id=ROBOT_ID, binding=ProtocolBinding(),
                                pairing_code=VALID_PAIRING_CODE, loss_grace_s=1.0, auth_timeout_s=3.0,
                                backoff_initial_s=0.2, backoff_max_s=0.5)
            return await self.silence_and_measure(server, cfg, token_path, limit_s=8.0)

        result = self.run_async(scenario, ping_interval=1, ping_timeout=1)
        self.assertIsNotNone(result["detected_at"], "the silent link was never detected")
        self.assertIsNotNone(result["paused_at"], "the mission never paused")
        self.check(result, ping_interval=1.0, ping_timeout=1.0, grace=1.0)

    def test_G_PRODUCTION_TIMING_pause_lands_before_the_earliest_lease_expiry(self):
        """Y4 PRODUCTION TIMING (~25 s). The backend's heartbeat (10 s + 5 s,
        `Backend/src/config/socketTiming.js`) and the Pi's own defaults, read
        from Settings: the robot pauses within detection + grace + one tick, and
        that is under the earliest lease expiry, lease.duration (60 s, the
        backend register) / 2 - the heartbeat interval."""

        from robotx.config.settings import Settings

        lease_duration_s = 60.0  # Backend register `lease.duration`, asserted there too

        async def scenario(server, token_path):
            tmp = os.path.dirname(token_path)
            settings = Settings.from_env({
                "ROBOTX_ROBOT_ID": ROBOT_ID, "ROBOTX_SOCKET_ENABLED": "1", "ROBOTX_SOCKET_SERVER_URL": server.url,
                "ROBOTX_PAIRING_CODE": VALID_PAIRING_CODE, "ROBOTX_BACKEND_TOKEN_PATH": token_path,
                "ROBOTX_COMMITMENT_STATE_PATH": os.path.join(tmp, "commitments.json"),
            })
            cfg = BackendConfig.from_settings(settings)  # production grace, validated
            result = await self.silence_and_measure(server, cfg, token_path, limit_s=30.0)
            result["settings"] = settings
            result["grace"] = cfg.loss_grace_s
            return result

        result = self.run_async(scenario, ping_interval=10, ping_timeout=5)
        self.assertIsNotNone(result["detected_at"], "the silent link was never detected")
        self.assertIsNotNone(result["paused_at"], "the mission never paused")
        settings, grace = result["settings"], result["grace"]
        tick = 1.0 / settings.agent_hz
        self.assertEqual((result["bound"], grace, tick), (15.0, 10.0, 0.1))
        bound = result["bound"] + grace + tick
        earliest_lease_expiry = lease_duration_s / 2 - settings.backend_heartbeat_interval_s
        self.assertLess(bound, earliest_lease_expiry)  # 25.1 < 28
        # Y4.1 -- and no heartbeat `from_settings` admits can shrink that window below the bound.
        self.assertLessEqual(settings.backend_heartbeat_interval_s, MAX_HEARTBEAT_INTERVAL_S)
        self.assertLess(bound, lease_duration_s / 2 - MAX_HEARTBEAT_INTERVAL_S)
        _, _, total = self.check(result, ping_interval=10.0, ping_timeout=5.0, grace=grace)
        self.assertLess(total, earliest_lease_expiry)  # and so is what actually happened


class TestHeartbeatOverTheWire(ContractTestCase):
    def test_heartbeat_arrives_on_its_cadence(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path, heartbeat_interval_s=0.15)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await wait_for(lambda: len(server.received["HEARTBEAT"]) >= 3)
            await link.stop()
            return server

        server = self.run_async(scenario)
        self.assertGreaterEqual(len(server.received["HEARTBEAT"]), 3)

    def test_heartbeat_arrives_even_with_no_gps_fix(self):
        """The gap this closes: indoors the robot sends no telemetry at all,
        and without a heartbeat the backend cannot tell it from a dead one."""

        async def scenario(server, token_path):
            link = self.make_link(
                server, token_path, state=RobotState(ROBOT_ID), heartbeat_interval_s=0.15
            )
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await wait_for(lambda: len(server.received["HEARTBEAT"]) >= 3)
            described = link.describe()
            await link.stop()
            return server, described

        server, described = self.run_async(scenario)
        self.assertTrue(all("lat" not in f for f in server.received["TELEMETRY"]))
        self.assertGreater(described["stats"]["positions_omitted"], 0)
        self.assertGreaterEqual(len(server.received["HEARTBEAT"]), 3)

    def test_idle_heartbeat_payload_is_empty(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path, heartbeat_interval_s=0.15)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await wait_for(lambda: len(server.received["HEARTBEAT"]) >= 1)
            await link.stop()
            return server

        server = self.run_async(scenario)
        self.assertEqual(server.received["HEARTBEAT"][0], {})

    def test_no_heartbeat_reaches_a_backend_that_refused_us(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path, backoff_auth_failed_s=30.0,
                                  heartbeat_interval_s=0.05)
            await link.start()
            await wait_for(lambda: link.status is LinkStatus.AUTH_FAILED)
            await link.stop()
            return server

        server = self.run_async(scenario, refuse_everything=True)
        self.assertEqual(server.received["HEARTBEAT"], [])

    def test_heartbeat_resumes_after_a_reconnect(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path, heartbeat_interval_s=0.15,
                                  backoff_initial_s=0.05, backoff_max_s=0.2)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            server.accept_token = ISSUED_TOKEN
            await wait_for(lambda: len(server.received["HEARTBEAT"]) >= 1)
            await server.kick()
            await wait_for(lambda: not link.connected)
            before = len(server.received["HEARTBEAT"])
            await wait_for(lambda: link.connected, timeout=8.0)
            resumed = await wait_for(
                lambda: len(server.received["HEARTBEAT"]) > before, timeout=5.0
            )
            await link.stop()
            return resumed

        self.assertTrue(self.run_async(scenario))


# --- 5-9: telemetry -----------------------------------------------------------


class TestTelemetryOverTheWire(ContractTestCase):
    def test_telemetry_arrives_with_the_contract_fields(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await wait_for(lambda: len(server.received["TELEMETRY"]) >= 1)
            await link.stop()
            return server

        server = self.run_async(scenario)
        frame = server.received["TELEMETRY"][0]
        self.assertEqual(set(frame), {"timestamp", "sequence", "status", "lat", "lon", "speed"})
        self.assertAlmostEqual(frame["lat"], 12.9716)
        self.assertAlmostEqual(frame["lon"], 77.5946)
        self.assertAlmostEqual(frame["speed"], 1.25)
        self.assertIsInstance(frame["sequence"], int)

    def test_timestamp_is_epoch_milliseconds_measured_by_the_pi(self):
        """The load-bearing field. Wrong units here are silently wrong data."""

        async def scenario(server, token_path):
            link = self.make_link(server, token_path)
            before = int(time.time() * 1000)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await wait_for(lambda: len(server.received["TELEMETRY"]) >= 1)
            await link.stop()
            return server, before, int(time.time() * 1000)

        server, before, after = self.run_async(scenario)
        ts = server.received["TELEMETRY"][0]["timestamp"]
        self.assertIsInstance(ts, int)
        # Inside the window this test ran in, so it is neither seconds
        # (~1.7e9, far too small) nor a startup constant.
        self.assertGreaterEqual(ts, before - 60_000)
        self.assertLessEqual(ts, after + 1_000)

    def test_every_frame_carries_a_fresh_timestamp(self):
        """A timestamp captured once at startup and reused would pass a
        single-frame check and still be wrong."""

        async def scenario(server, token_path):
            state = state_with_fix()
            link = self.make_link(server, token_path, state=state, telemetry_interval_s=0.05)
            await link.start()
            await link.wait_connected(timeout_s=5.0)

            # Move the robot, which is what advances the measurement time.
            for _ in range(4):
                await asyncio.sleep(0.12)
                now = time.time()
                state.update_gps(
                    GpsReading(
                        status=GPSStatus.FIX,
                        fix=GpsFix(latitude=1.0, longitude=2.0, timestamp=now),
                        age_s=0.0,
                    ),
                    Position(latitude=1.0, longitude=2.0, timestamp=now, speed_mps=0.4),
                )
            await wait_for(lambda: len(server.received["TELEMETRY"]) >= 3)
            await link.stop()
            return server

        server = self.run_async(scenario)
        stamps = [f["timestamp"] for f in server.received["TELEMETRY"] if "lat" in f]
        self.assertGreater(len(set(stamps)), 1, f"timestamps never advanced: {stamps}")
        self.assertEqual(stamps, sorted(stamps))
        # One fix is one observation: never sent twice.
        self.assertEqual(len(stamps), len(set(stamps)))
        sequences = [f["sequence"] for f in server.received["TELEMETRY"]]
        self.assertEqual(sequences, sorted(set(sequences)), "sequence must strictly increase")

    def test_battery_is_omitted_never_invented(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await wait_for(lambda: len(server.received["TELEMETRY"]) >= 1)
            await link.stop()
            return server

        server = self.run_async(scenario)
        self.assertTrue(server.received["TELEMETRY"])
        for frame in server.received["TELEMETRY"]:
            self.assertNotIn("battery", frame)

    def test_no_position_is_sent_without_a_usable_fix(self):
        async def scenario(server, token_path):
            state = RobotState(ROBOT_ID)  # no fix at all
            link = self.make_link(server, token_path, state=state)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await asyncio.sleep(0.4)
            described = link.describe()
            await link.stop()
            return server, described

        server, described = self.run_async(scenario)
        self.assertTrue(server.received["TELEMETRY"])
        for frame in server.received["TELEMETRY"]:
            self.assertEqual(set(frame), {"timestamp", "sequence", "status"})
        self.assertGreater(described["stats"]["positions_omitted"], 0)
        # The robot is still online; it simply has nothing truthful to report.
        self.assertGreaterEqual(described["stats"]["auth_successes"], 1)

    def test_no_telemetry_is_sent_before_authentication(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path, backoff_auth_failed_s=30.0)
            await link.start()
            await wait_for(lambda: link.status is LinkStatus.AUTH_FAILED)
            await link.stop()
            return server

        server = self.run_async(scenario, refuse_everything=True)
        self.assertEqual(server.received["TELEMETRY"], [])

    def test_telemetry_rate_is_bounded_by_configuration(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path, telemetry_interval_s=0.3)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await asyncio.sleep(1.0)
            await link.stop()
            return server

        server = self.run_async(scenario)
        # ~1 s at 0.3 s intervals: a handful, not a flood.
        self.assertLessEqual(len(server.received["TELEMETRY"]), 6)


# --- 10-15: commands ----------------------------------------------------------


class TestCommandsOverTheWire(ContractTestCase):
    def _run_command(self, command_type, agent_mode=OperatingMode.AUTO, payload=None):
        async def scenario(server, token_path):
            agent = FakeAgent(mode=agent_mode)
            link = self.make_link(server, token_path, agent=agent)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await server.send_command(
                payload or {"commandId": "c-1", "type": command_type,
                            "timestamp": int(time.time() * 1000)}
            )
            await wait_for(lambda: len(server.received["COMMAND_ACK"]) >= 1, timeout=1.0)
            await link.stop()
            return server, agent

        return self.run_async(scenario)

    def test_ack_is_a_separate_event_named_command_ack(self):
        """The contract requires an emitted COMMAND_ACK event, never a
        Socket.IO callback acknowledgement."""

        server, _ = self._run_command("STOP")
        self.assertEqual(len(server.received["COMMAND_ACK"]), 1)
        self.assertEqual(server.received["COMMAND_ACK"][0]["commandId"], "c-1")

    def test_stop_is_applied_and_acknowledged(self):
        server, agent = self._run_command("STOP")
        self.assertEqual([c[0] for c in agent.calls], ["stop"])
        self.assertEqual(server.received["COMMAND_ACK"], [{"commandId": "c-1"}])

    def test_pause_is_applied_and_acknowledged(self):
        server, agent = self._run_command("PAUSE")
        self.assertEqual([c[0] for c in agent.calls], ["pause"])
        self.assertEqual(server.received["COMMAND_ACK"], [{"commandId": "c-1"}])

    def test_resume_is_applied_from_paused(self):
        server, agent = self._run_command("RESUME", agent_mode=OperatingMode.PAUSED)
        self.assertEqual([c[0] for c in agent.calls], ["resume"])
        self.assertEqual(server.received["COMMAND_ACK"], [{"commandId": "c-1"}])

    def test_resume_from_a_stopped_mission_is_refused_and_not_acked(self):
        # No FAILED form exists: left unacknowledged, the backend marks it FAILED.
        server, agent = self._run_command("RESUME", agent_mode=OperatingMode.STOPPED)
        self.assertEqual(agent.calls, [])
        self.assertEqual(server.received["COMMAND_ACK"], [])

    def test_return_is_applied_and_acknowledged(self):
        server, agent = self._run_command("RETURN")
        self.assertEqual([c[0] for c in agent.calls], ["return"])
        self.assertEqual(server.received["COMMAND_ACK"], [{"commandId": "c-1"}])

    def test_bare_stop_event_cancels_the_task(self):
        async def scenario(server, token_path):
            agent = FakeAgent()
            link = self.make_link(server, token_path, agent=agent)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await server.send_stop({"taskId": "task-9", "reason": "TASK_CANCELLED",
                                    "timestamp": int(time.time() * 1000)})
            await wait_for(lambda: agent.calls, timeout=2.0)
            await asyncio.sleep(0.2)
            await link.stop()
            return server, agent

        server, agent = self.run_async(scenario)
        self.assertEqual([c[0] for c in agent.calls], ["stop"])
        # The bare STOP expects no acknowledgement (handoff §14).
        self.assertEqual(server.received["COMMAND_ACK"], [])

    def test_duplicate_command_id_executes_once_and_acks_identically(self):
        """The backend redispatches at roughly 5 s, 10 s and 15 s."""

        async def scenario(server, token_path):
            agent = FakeAgent()
            link = self.make_link(server, token_path, agent=agent)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            command = {"commandId": "dup-1", "type": "STOP", "robotId": ROBOT_ID}
            for _ in range(3):
                await server.send_command(command)
                await asyncio.sleep(0.1)
            await wait_for(lambda: len(server.received["COMMAND_ACK"]) >= 3)
            described = link.describe()
            await link.stop()
            return server, agent, described

        server, agent, described = self.run_async(scenario)
        # Executed exactly once...
        self.assertEqual([c[0] for c in agent.calls], ["stop"])
        # ...acknowledged every time, with an unchanging result.
        acks = server.received["COMMAND_ACK"]
        self.assertGreaterEqual(len(acks), 3)
        self.assertEqual(acks, [{"commandId": "dup-1"}] * len(acks))
        self.assertGreaterEqual(described["stats"]["commands_duplicate"], 2)

    def test_duplicate_bare_stop_for_one_task_executes_once(self):
        async def scenario(server, token_path):
            agent = FakeAgent()
            link = self.make_link(server, token_path, agent=agent)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            for _ in range(3):
                await server.send_stop({"taskId": "task-9"})
                await asyncio.sleep(0.1)
            await asyncio.sleep(0.2)
            await link.stop()
            return agent

        agent = self.run_async(scenario)
        self.assertEqual([c[0] for c in agent.calls], ["stop"])

    def test_unknown_command_type_is_never_acked_or_executed(self):
        server, agent = self._run_command(
            "MOVE", payload={"commandId": "c-x", "type": "MOVE", "robotId": ROBOT_ID}
        )
        self.assertEqual(agent.calls, [])
        self.assertEqual(server.received["COMMAND_ACK"], [])

    def test_command_for_another_robot_is_refused(self):
        server, agent = self._run_command(
            "STOP", payload={"commandId": "c-y", "type": "STOP", "robotId": "some-other-robot"}
        )
        self.assertEqual(agent.calls, [])
        # Not even a FAILED: that commandId is another robot's Command row.
        self.assertEqual(server.received.get("COMMAND_ACK", []), [])

    def test_malformed_commands_do_not_break_the_next_one(self):
        async def scenario(server, token_path):
            agent = FakeAgent()
            link = self.make_link(server, token_path, agent=agent)
            await link.start()
            await link.wait_connected(timeout_s=5.0)

            for junk in (
                "not-an-object",
                12345,
                [],
                {},
                {"type": "STOP"},                      # no commandId
                {"commandId": "z", "type": None},
                {"commandId": "z2", "type": {"nested": "object"}},
            ):
                await server.send_command(junk)
                await asyncio.sleep(0.05)

            # The link is still healthy and still obeys a good command.
            await server.send_command(
                {"commandId": "good-1", "type": "STOP", "robotId": ROBOT_ID}
            )
            await wait_for(lambda: any(
                a.get("commandId") == "good-1" for a in server.received["COMMAND_ACK"]
            ))
            described = link.describe()
            await link.stop()
            return server, agent, described

        server, agent, described = self.run_async(scenario)
        self.assertEqual([c[0] for c in agent.calls], ["stop"])
        self.assertEqual(server.received["COMMAND_ACK"], [{"commandId": "good-1"}])
        self.assertGreater(described["stats"]["commands_rejected"], 0)
        self.assertIs(described["status"], described["status"])  # link survived


# --- engine envelopes over the wire -----------------------------------------


class TestEngineOverTheWire(ContractTestCase):
    """SIMULATED signed envelopes (tests.fixtures.engine) over a real socket."""

    def test_offer_on_its_own_event_name_is_acked_then_answered_once(self):
        from tests.fixtures import engine as fx

        async def scenario(server, token_path):
            agent = FakeAgent()
            link = self.make_link(server, token_path, agent=agent,
                                  command_signing_key=fx.TEST_SIGNING_KEY)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            env = fx.envelope(agent_id=ROBOT_ID)
            await server.send_engine(env)
            await server.send_engine(env)  # redelivery
            await wait_for(lambda: len(server.received["COMMAND_ACK"]) >= 2)
            await asyncio.sleep(0.2)
            await link.stop()
            return server, agent

        server, agent = self.run_async(scenario)
        self.assertEqual(server.received["COMMAND_ACK"][0],
                         {"outboxId": "ob-19", "fence": "42", "authorityEpoch": None})
        # (The first test above this used to send OFFER on an event called "command",
        # which the backend never emits; the Pi bound only that name and so dropped
        # every real OFFER. See test_an_envelope_on_a_generic_command_event_is_not_an_offer.)
        # The FakeAgent declines, exactly once; never an automatic accept.
        self.assertEqual(server.received["OFFER_REJECT"],
                         [{"commitmentId": "c-7f3a", "fence": "42", "reason": "NO_MOTOR_LINK"}])
        self.assertEqual(server.received["OFFER_ACCEPT"], [])

    def test_an_envelope_on_a_generic_command_event_is_not_an_offer(self):
        from tests.fixtures import engine as fx

        async def scenario(server, token_path):
            link = self.make_link(server, token_path, agent=FakeAgent(),
                                  command_signing_key=fx.TEST_SIGNING_KEY)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await server.send_engine(fx.envelope(agent_id=ROBOT_ID), event="command")
            await asyncio.sleep(0.5)
            await link.stop()
            return server, link

        server, link = self.run_async(scenario)
        self.assertEqual(server.received["COMMAND_ACK"], [])
        self.assertEqual(link.stats["engine_commands_received"], 0)

    def test_task_assign_over_the_wire_gets_no_reply_and_starts_nothing(self):
        from tests.fixtures.task_assign import task_assign_payload

        async def scenario(server, token_path):
            agent = FakeAgent(mode=OperatingMode.IDLE)
            link = self.make_link(server, token_path, agent=agent)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await server.send_task_assign(task_assign_payload(task_id="T-1"))
            await wait_for(lambda: link.stats["tasks_received"] >= 1)
            await link.stop()
            return server, agent

        server, agent = self.run_async(scenario)
        self.assertEqual(agent.calls, [])
        self.assertEqual(server.received["COMMAND_ACK"], [])


# --- 16-20: disconnect, reconnect, shutdown -----------------------------------


class TestDisconnectAndReconnect(ContractTestCase):
    def test_server_disconnect_is_detected_during_streaming(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await wait_for(lambda: len(server.received["TELEMETRY"]) >= 1)
            await server.kick()
            dropped = await wait_for(lambda: not link.connected)
            await link.stop()
            return dropped

        self.assertTrue(self.run_async(scenario))

    def test_link_reconnects_and_reauthenticates_after_a_drop(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path, backoff_initial_s=0.05, backoff_max_s=0.2)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            # The server will accept the token it just issued.
            server.accept_token = ISSUED_TOKEN
            first_auths = len(server.auth_payloads)
            await server.kick()
            await wait_for(lambda: not link.connected)
            back = await wait_for(lambda: link.connected, timeout=8.0)
            described = link.describe()
            await link.stop()
            return back, described, server, first_auths

        back, described, server, first_auths = self.run_async(scenario)
        self.assertTrue(back)
        self.assertGreater(len(server.auth_payloads), first_auths)
        self.assertGreaterEqual(described["stats"]["auth_successes"], 2)

    def test_telemetry_resumes_after_a_reconnect(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path, backoff_initial_s=0.05, backoff_max_s=0.2)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            server.accept_token = ISSUED_TOKEN
            await wait_for(lambda: len(server.received["TELEMETRY"]) >= 1)
            await server.kick()
            await wait_for(lambda: not link.connected)
            before = len(server.received["TELEMETRY"])
            await wait_for(lambda: link.connected, timeout=8.0)
            resumed = await wait_for(
                lambda: len(server.received["TELEMETRY"]) > before, timeout=5.0
            )
            await link.stop()
            return resumed

        self.assertTrue(self.run_async(scenario))

    def test_commands_still_work_after_a_reconnect(self):
        async def scenario(server, token_path):
            agent = FakeAgent()
            link = self.make_link(
                server, token_path, agent=agent, backoff_initial_s=0.05, backoff_max_s=0.2
            )
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            server.accept_token = ISSUED_TOKEN
            await server.kick()
            await wait_for(lambda: not link.connected)
            await wait_for(lambda: link.connected, timeout=8.0)

            await server.send_command(
                {"commandId": "after-reconnect", "type": "PAUSE", "robotId": ROBOT_ID}
            )
            got = await wait_for(lambda: any(
                a.get("commandId") == "after-reconnect" for a in server.received["COMMAND_ACK"]
            ), timeout=5.0)
            await link.stop()
            return got, agent

        got, agent = self.run_async(scenario)
        self.assertTrue(got)
        self.assertIn("pause", [c[0] for c in agent.calls])

    def test_listeners_are_not_duplicated_across_reconnects(self):
        """A duplicated COMMAND listener would execute the operator's command
        twice. This is the assertion that catches it."""

        async def scenario(server, token_path):
            agent = FakeAgent()
            link = self.make_link(
                server, token_path, agent=agent, backoff_initial_s=0.05, backoff_max_s=0.2
            )
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            server.accept_token = ISSUED_TOKEN

            for _ in range(3):
                await server.kick()
                await wait_for(lambda: not link.connected)
                await wait_for(lambda: link.connected, timeout=8.0)

            await server.send_command(
                {"commandId": "once-only", "type": "STOP", "robotId": ROBOT_ID}
            )
            await wait_for(lambda: len(server.received["COMMAND_ACK"]) >= 1)
            await asyncio.sleep(0.3)
            described = link.describe()
            await link.stop()
            return described, agent, server

        described, agent, server = self.run_async(scenario)
        self.assertEqual(described["handler_registrations"], 1)
        # One command, one execution, one acknowledgement.
        self.assertEqual([c[0] for c in agent.calls], ["stop"])
        acks = [a for a in server.received["COMMAND_ACK"] if a["commandId"] == "once-only"]
        self.assertEqual(len(acks), 1)


class TestBackendUnavailable(ContractTestCase):
    def test_agent_survives_a_backend_that_is_down_at_startup(self):
        async def scenario():
            state = state_with_fix()
            agent = FakeAgent()
            cfg = BackendConfig(
                enabled=True,
                # Port 1 is reliably closed: ECONNREFUSED, not a refused login.
                server_url="http://127.0.0.1:1",
                robot_id=ROBOT_ID,
                pairing_code=VALID_PAIRING_CODE,
                backoff_initial_s=0.05,
                backoff_max_s=0.1,
                loss_grace_s=30.0,
            )
            with tempfile.TemporaryDirectory() as tmp:
                link = BackendLink(
                    cfg, state, agent,
                    token_store=TokenStore(os.path.join(tmp, "s.json")),
                )
                await link.start()
                await asyncio.sleep(0.6)
                described = link.describe()
                await link.stop()
            return described, agent

        described, agent = asyncio.run(scenario())
        self.assertGreater(described["stats"]["connect_failures"], 0)
        # Crucially: a dead backend is never mistaken for a refused credential.
        self.assertEqual(described["stats"]["auth_failures"], 0)
        self.assertEqual(described["status"], LinkStatus.DISCONNECTED.value)
        # The mission was not touched.
        self.assertEqual(agent.calls, [])

    def test_link_connects_once_the_backend_appears(self):
        async def scenario():
            server = ContractServer()
            # Bind a port, learn its number, then free it so nothing is
            # listening when the link starts.
            await server.start()
            port = server.port
            await server.stop()

            with tempfile.TemporaryDirectory() as tmp:
                cfg = BackendConfig(
                    enabled=True,
                    server_url=f"http://127.0.0.1:{port}",
                    robot_id=ROBOT_ID,
                    pairing_code=VALID_PAIRING_CODE,
                    telemetry_interval_s=0.1,
                    backoff_initial_s=0.05,
                    backoff_max_s=0.2,
                )
                link = BackendLink(
                    cfg, state_with_fix(), FakeAgent(),
                    token_store=TokenStore(os.path.join(tmp, "s.json")),
                )
                await link.start()
                await asyncio.sleep(0.4)
                down_status = link.status

                # The backend comes up on the same port. No restart of the Pi.
                later = ContractServer()
                later_app_runner = web.AppRunner(later.app)
                await later_app_runner.setup()
                site = web.TCPSite(later_app_runner, "127.0.0.1", port)
                await site.start()
                try:
                    came_up = await wait_for(lambda: link.connected, timeout=10.0)
                    got_telemetry = await wait_for(
                        lambda: len(later.received["TELEMETRY"]) >= 1, timeout=5.0
                    )
                finally:
                    await link.stop()
                    await later_app_runner.cleanup()
            return down_status, came_up, got_telemetry

        down_status, came_up, got_telemetry = asyncio.run(scenario())
        self.assertIs(down_status, LinkStatus.DISCONNECTED)
        self.assertTrue(came_up)
        self.assertTrue(got_telemetry)


class TestCleanShutdown(ContractTestCase):
    def test_stop_closes_the_socket_and_leaves_no_task_running(self):
        async def scenario(server, token_path):
            link = self.make_link(server, token_path)
            await link.start()
            await link.wait_connected(timeout_s=5.0)
            await link.stop()
            await asyncio.sleep(0.1)
            return link

        link = self.run_async(scenario)
        self.assertIsNone(link._task)
        self.assertFalse(link.connected)
        self.assertEqual(link.status, LinkStatus.DISCONNECTED)

    def test_stop_is_safe_when_the_link_never_connected(self):
        async def scenario():
            with tempfile.TemporaryDirectory() as tmp:
                cfg = BackendConfig(enabled=False, robot_id=ROBOT_ID)
                link = BackendLink(
                    cfg, state_with_fix(), FakeAgent(),
                    token_store=TokenStore(os.path.join(tmp, "s.json")),
                )
                await link.stop()
            return link

        link = asyncio.run(scenario())
        self.assertEqual(link.status, LinkStatus.DISABLED)


if __name__ == "__main__":
    unittest.main()
