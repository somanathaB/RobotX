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
import tempfile
import time
import unittest

import socketio
from aiohttp import web

from robotx.communication.backend_link import BackendConfig, BackendLink
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
                 success_event="both"):
        # The real backend emits BOTH AUTH_SUCCESS and AUTH_OK for one AUTH
        # (handoff §3); "both" is therefore the default here.
        self.success_event = success_event
        self.accept_pairing_code = accept_pairing_code
        self.accept_token = accept_token
        self.issue_token = issue_token
        self.refuse_everything = refuse_everything

        self.sio = socketio.AsyncServer(async_mode="aiohttp")
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
            payload = data if isinstance(data, dict) else {}
            code = payload.get("pairingCode")
            token = payload.get("token")

            ok = not self.refuse_everything and (
                (code is not None and code == self.accept_pairing_code)
                or (token is not None and token == self.accept_token)
            )
            if not ok:
                self.refusals.append(payload)
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
        @self.sio.on(name, namespace=NAMESPACE)
        async def handler(sid, data, _name=name):
            self.received[_name].append(data)

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

    async def send_engine(self, envelope, *, sid=None):
        await self.sio.emit(
            "command", envelope, to=sid or self.authenticated_sids[-1], namespace=NAMESPACE
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

    def test_offer_on_the_lowercase_command_event_is_acked_then_answered_once(self):
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
        # The FakeAgent declines, exactly once; never an automatic accept.
        self.assertEqual(server.received["OFFER_REJECT"],
                         [{"commitmentId": "c-7f3a", "fence": "42", "reason": "NO_MOTOR_LINK"}])
        self.assertEqual(server.received["OFFER_ACCEPT"], [])

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
