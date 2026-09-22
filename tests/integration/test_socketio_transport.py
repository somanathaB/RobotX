"""End-to-end over a REAL Socket.IO transport.

Everything here runs across an actual Engine.IO handshake, an actual WebSocket
or polling upgrade, and real JSON serialization, against a `socketio.AsyncServer`
on a loopback port. No part of the client is faked: `BackendLink` builds its
normal `socketio.AsyncClient`.

What this proves, and what it does not
--------------------------------------
It proves the Pi's client half works on the wire: the handshake carries the
credential, a server that refuses it is handled, telemetry arrives as sendable
JSON, a command round-trips into an acknowledgement, and a dropped connection
is detected and re-established.

It does **not** prove anything about the FalconAut backend. The server here
speaks the Pi's own PROVISIONAL binding, because the real event names are not
available in this repository. A passing run means "the transport and the state
machine are correct", not "the integration is done". Every assertion is
therefore about the Pi's behaviour, never about a backend contract.

These tests bind a loopback TCP port. They are automated and hardware-free, but
they are not pure unit tests, which is why they live outside `tests/unit`.
"""

import asyncio
import logging
import time
import unittest

import socketio
from aiohttp import web

from robotx.communication.backend_link import BackendConfig, BackendLink
from robotx.communication.protocol import BindingSource, ProtocolBinding
from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus
from robotx.localization.position import Position
from robotx.state.robot_state import LinkStatus, OperatingMode, RobotState


NAMESPACE = "/robot"
VALID_TOKEN = "integration-test-token"
ROBOT_ID = "robotx-pi-itest"


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL)
    # The rejection tests make the server refuse a handshake on purpose;
    # engineio logs that refusal with a full traceback.
    for noisy in ("engineio", "engineio.server", "socketio", "socketio.server", "aiohttp"):
        logging.getLogger(noisy).setLevel(logging.CRITICAL)


class RecordingServer:
    """A Socket.IO server that records what the Pi sent and can issue commands.

    It validates the handshake credential so that the rejection path is
    exercised against a server that genuinely refuses, rather than against a
    simulated error.
    """

    def __init__(self, *, require_token=True):
        self.require_token = require_token
        self.sio = socketio.AsyncServer(async_mode="aiohttp")
        self.app = web.Application()
        self.sio.attach(self.app)

        self.received = {"robot_hello": [], "telemetry": [], "status": [],
                         "event": [], "command_ack": []}
        self.connections = []
        self.rejections = []
        self.auths = []
        self.sids = []
        self.runner = None
        self.port = None

        self._install_handlers()

    def _install_handlers(self):
        @self.sio.event(namespace=NAMESPACE)
        async def connect(sid, environ, auth=None):
            self.auths.append(auth)
            token = (auth or {}).get("token")
            if self.require_token and token != VALID_TOKEN:
                self.rejections.append(auth)
                # The standard server-side refusal: python-socketio turns this
                # into a client ConnectionError carrying this message.
                raise ConnectionRefusedError("Unauthorized: invalid token")
            self.connections.append(sid)
            self.sids.append(sid)

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
        await self.sio.emit("command", payload, to=sid or self.sids[-1], namespace=NAMESPACE)

    async def kick(self, sid=None):
        await self.sio.disconnect(sid or self.sids[-1], namespace=NAMESPACE)


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


def state_with_fix():
    state = RobotState(ROBOT_ID)
    now = time.time()
    state.update_gps(
        GpsReading(
            status=GPSStatus.FIX,
            fix=GpsFix(latitude=12.9716, longitude=77.5946, timestamp=now),
            age_s=0.1,
        ),
        Position(latitude=12.9716, longitude=77.5946, timestamp=now, speed_mps=0.8),
    )
    return state


async def wait_for(predicate, timeout=6.0, interval=0.05):
    """Poll until `predicate()` is true. Returns whether it became true."""

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        await asyncio.sleep(interval)
    return predicate()


class SocketIOTestCase(unittest.TestCase):
    """Runs one scenario coroutine, guaranteeing server and link teardown."""

    def run_scenario(self, scenario, *, require_token=True, timeout=30.0, **cfg_kwargs):
        async def runner():
            server = RecordingServer(require_token=require_token)
            await server.start()

            cfg = BackendConfig(
                enabled=True,
                server_url=server.url,
                robot_id=ROBOT_ID,
                robot_token=cfg_kwargs.pop("robot_token", VALID_TOKEN),
                binding=cfg_kwargs.pop("binding", ProtocolBinding()),
                telemetry_interval_s=cfg_kwargs.pop("telemetry_interval_s", 0.2),
                status_interval_s=cfg_kwargs.pop("status_interval_s", 0.5),
                backoff_initial_s=cfg_kwargs.pop("backoff_initial_s", 0.1),
                backoff_max_s=cfg_kwargs.pop("backoff_max_s", 0.4),
                backoff_rejected_s=cfg_kwargs.pop("backoff_rejected_s", 0.3),
                **cfg_kwargs,
            )
            state = state_with_fix()
            agent = FakeAgent()
            link = BackendLink(cfg, state, agent)

            try:
                return await asyncio.wait_for(
                    scenario(server, link, agent, state), timeout=timeout
                )
            finally:
                await link.stop()
                await server.stop()

        return asyncio.run(runner())


class TestHandshakeAndRegistration(SocketIOTestCase):
    def test_pi_connects_and_registers_over_the_real_wire(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0), "link never connected")
            self.assertTrue(await wait_for(lambda: server.received["robot_hello"]))
            return server.received["robot_hello"][0]

        register = self.run_scenario(scenario)
        self.assertEqual(register["robotId"], ROBOT_ID)
        self.assertIs(register["simulated"], False)
        self.assertIn("capabilities", register)

    def test_credential_reaches_the_server_in_the_handshake(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            return server.auths[0]

        auth = self.run_scenario(scenario)
        self.assertEqual(auth["robotId"], ROBOT_ID)
        self.assertEqual(auth["token"], VALID_TOKEN)

    def test_state_reports_connected_only_once_it_really_is(self):
        async def scenario(server, link, agent, state):
            self.assertIs(state.snapshot().communication.backend, LinkStatus.DISABLED)
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            return state.snapshot().communication

        comms = self.run_scenario(scenario)
        self.assertIs(comms.backend, LinkStatus.CONNECTED)
        self.assertIsNotNone(comms.backend_last_send_at)

    def test_capabilities_admit_no_battery_and_no_motion(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await wait_for(lambda: server.received["robot_hello"], 8.0))
            return server.received["robot_hello"][0]["capabilities"]

        caps = self.run_scenario(scenario)
        self.assertFalse(caps["battery"])
        self.assertFalse(caps["motion"])


class TestAuthenticationRejection(SocketIOTestCase):
    def test_server_refusing_the_token_is_handled_not_crashed(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertFalse(await link.wait_connected(2.0), "should not have connected")
            # Captured from the poll rather than re-read afterwards: the link
            # cycles REJECTED -> CONNECTING -> REJECTED as it retries, so a
            # later read can legitimately catch it mid-attempt.
            saw_rejected = await wait_for(lambda: link.status is LinkStatus.REJECTED, 5.0)
            return saw_rejected, link.connected, link._last_connect_error, server.rejections

        saw_rejected, connected, error, rejections = self.run_scenario(
            scenario, robot_token="wrong-token"
        )
        self.assertTrue(saw_rejected, "link never reported REJECTED")
        self.assertFalse(connected)
        self.assertGreaterEqual(len(rejections), 1)
        # The Pi can tell it was refused but NOT why: python-socketio does not
        # deliver the server's ConnectionRefusedError message to the client,
        # and the namespace `connect_error` handler is never invoked for a
        # namespace rejection. The recorded error is the library's generic one.
        self.assertIn("namespaces failed to connect", error.lower())

    def test_rejected_link_never_reports_itself_integrated(self):
        async def scenario(server, link, agent, state):
            await link.start()
            await wait_for(lambda: link.status is LinkStatus.REJECTED, 5.0)
            return link.describe()

        described = self.run_scenario(scenario, robot_token="wrong-token")
        self.assertIs(described["integrated"], False)
        self.assertNotEqual(described["status"], "CONNECTED")

    def test_missing_token_is_refused_by_a_validating_server(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertFalse(await link.wait_connected(2.0))
            return link.status

        status = self.run_scenario(scenario, robot_token=None)
        self.assertIsNot(status, LinkStatus.CONNECTED)


class TestTelemetryOverTheWire(SocketIOTestCase):
    def test_telemetry_arrives_as_the_declared_payload(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            self.assertTrue(await wait_for(lambda: server.received["telemetry"], 8.0))
            return server.received["telemetry"][0]

        payload = self.run_scenario(scenario)
        self.assertEqual(payload["robotId"], ROBOT_ID)
        self.assertAlmostEqual(payload["lat"], 12.9716)
        self.assertAlmostEqual(payload["lon"], 77.5946)
        self.assertIsNone(payload["battery"])

    def test_status_arrives_and_carries_the_operating_mode(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await wait_for(lambda: server.received["status"], 8.0))
            return server.received["status"][0]

        payload = self.run_scenario(scenario)
        self.assertEqual(payload["status"], OperatingMode.IDLE.value)
        self.assertTrue(payload["protocol"]["provisional"])

    def test_no_telemetry_is_sent_without_a_position(self):
        async def scenario(server, link, agent, state):
            # Replace the fresh fix with an unusable one before connecting.
            state.update_gps(GpsReading(status=GPSStatus.NO_FIX), None)
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            # Status still flows, so the backend is not left blind.
            self.assertTrue(await wait_for(lambda: server.received["status"], 8.0))
            await asyncio.sleep(1.0)
            return server.received["telemetry"], link.stats["telemetry_skipped"]

        telemetry, skipped = self.run_scenario(scenario)
        self.assertEqual(telemetry, [])
        self.assertGreater(skipped, 0)

    def test_telemetry_rate_is_bounded_by_configuration(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            await asyncio.sleep(2.0)
            return len(server.received["telemetry"])

        # At one frame per second, ~2 s of connected time cannot produce 20.
        count = self.run_scenario(scenario, telemetry_interval_s=1.0)
        self.assertLessEqual(count, 5)
        self.assertGreaterEqual(count, 1)


class TestCommandRoundTrip(SocketIOTestCase):
    def test_command_is_received_applied_and_acknowledged(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            await wait_for(lambda: server.received["robot_hello"], 5.0)

            await server.send_command({"commandId": "cmd-1", "type": "STOP",
                                       "robotId": ROBOT_ID})
            self.assertTrue(await wait_for(lambda: server.received["command_ack"], 8.0))
            return agent.calls, server.received["command_ack"][0]

        calls, ack = self.run_scenario(scenario)
        self.assertEqual(calls[0][0], "stop")
        self.assertEqual(ack["status"], "ACK")
        self.assertEqual(ack["commandId"], "cmd-1")
        self.assertEqual(ack["robotId"], ROBOT_ID)
        self.assertIsInstance(ack["executedAt"], float)

    def test_unknown_command_is_failed_over_the_wire(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            await server.send_command({"commandId": "cmd-x", "type": "SELF_DESTRUCT"})
            self.assertTrue(await wait_for(lambda: server.received["command_ack"], 8.0))
            return agent.calls, server.received["command_ack"][0]

        calls, ack = self.run_scenario(scenario)
        self.assertEqual(calls, [])
        self.assertEqual(ack["status"], "FAILED")
        self.assertIn("UNKNOWN_TYPE", ack["reason"])

    def test_duplicate_command_executes_once_across_the_wire(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            payload = {"commandId": "dup-1", "type": "PAUSE", "robotId": ROBOT_ID}
            await server.send_command(payload)
            await wait_for(lambda: len(server.received["command_ack"]) >= 1, 8.0)
            await server.send_command(payload)
            await wait_for(lambda: len(server.received["command_ack"]) >= 2, 8.0)
            return agent.calls, server.received["command_ack"]

        calls, acks = self.run_scenario(scenario)
        self.assertEqual(len([c for c in calls if c[0] == "pause"]), 1)
        self.assertEqual(len(acks), 2)
        self.assertEqual(acks[0]["executedAt"], acks[1]["executedAt"])

    def test_command_for_another_robot_is_refused_over_the_wire(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            await server.send_command({"commandId": "c", "type": "STOP",
                                       "robotId": "some-other-robot"})
            self.assertTrue(await wait_for(lambda: server.received["command_ack"], 8.0))
            return agent.calls, server.received["command_ack"][0]

        calls, ack = self.run_scenario(scenario)
        self.assertEqual(calls, [])
        self.assertEqual(ack["status"], "FAILED")

    def test_malformed_command_does_not_break_the_next_one(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            await server.send_command("not-an-object")
            await server.send_command({"commandId": "good", "type": "STOP",
                                       "robotId": ROBOT_ID})
            self.assertTrue(await wait_for(lambda: agent.calls, 8.0))
            return agent.calls

        calls = self.run_scenario(scenario)
        self.assertEqual(calls[0][0], "stop")


class TestDisconnectAndReconnect(SocketIOTestCase):
    def test_server_disconnect_is_detected(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            await server.kick()
            detected = await wait_for(lambda: not link.connected, 8.0)
            return detected, state.snapshot().communication.backend

        detected, backend = self.run_scenario(scenario)
        self.assertTrue(detected, "link did not notice the server hanging up")
        self.assertIsNot(backend, LinkStatus.CONNECTED)

    def test_link_reconnects_after_being_dropped(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            first = len(server.connections)

            await server.kick()
            await wait_for(lambda: not link.connected, 8.0)

            reconnected = await wait_for(lambda: len(server.connections) > first, 15.0)
            return reconnected, link.status, link.stats

        reconnected, status, stats = self.run_scenario(scenario)
        self.assertTrue(reconnected, "link never reconnected")
        self.assertIs(status, LinkStatus.CONNECTED)
        self.assertGreaterEqual(stats["connects"], 2)

    def test_telemetry_resumes_after_a_reconnect(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            await wait_for(lambda: server.received["telemetry"], 8.0)
            await server.kick()
            await wait_for(lambda: not link.connected, 8.0)
            before = len(server.received["telemetry"])
            resumed = await wait_for(lambda: len(server.received["telemetry"]) > before, 15.0)
            return resumed

        self.assertTrue(self.run_scenario(scenario), "telemetry did not resume")

    def test_registration_is_repeated_on_reconnect(self):
        # The server assigns a new socketId on every connection, so the robot
        # must re-announce itself rather than assume the old association holds.
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            await wait_for(lambda: server.received["robot_hello"], 8.0)
            await server.kick()
            await wait_for(lambda: not link.connected, 8.0)
            repeated = await wait_for(lambda: len(server.received["robot_hello"]) >= 2, 15.0)
            return repeated

        self.assertTrue(self.run_scenario(scenario))

    def test_commands_work_again_after_a_reconnect(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            await server.kick()
            await wait_for(lambda: not link.connected, 8.0)
            self.assertTrue(await wait_for(lambda: link.connected, 15.0))
            await wait_for(lambda: len(server.sids) >= 2, 5.0)

            await server.send_command({"commandId": "after", "type": "STOP",
                                       "robotId": ROBOT_ID})
            self.assertTrue(await wait_for(lambda: agent.calls, 8.0))
            return agent.calls

        calls = self.run_scenario(scenario)
        self.assertEqual(calls[0][0], "stop")


class TestBackendUnavailable(SocketIOTestCase):
    def test_link_keeps_retrying_when_nothing_is_listening(self):
        async def scenario():
            cfg = BackendConfig(
                enabled=True,
                # Port 1 on loopback: nothing will ever answer.
                server_url="http://127.0.0.1:1",
                robot_id=ROBOT_ID,
                robot_token=VALID_TOKEN,
                backoff_initial_s=0.05,
                backoff_max_s=0.2,
            )
            state = state_with_fix()
            agent = FakeAgent()
            link = BackendLink(cfg, state, agent)
            try:
                await link.start()
                self.assertFalse(await link.wait_connected(1.0))
                await wait_for(lambda: link.stats["connect_failures"] >= 2, 6.0)
                return link.stats["connect_failures"], link.status, agent.calls
            finally:
                await link.stop()

        failures, status, calls = asyncio.run(asyncio.wait_for(scenario(), 25.0))
        self.assertGreaterEqual(failures, 2)
        self.assertIsNot(status, LinkStatus.CONNECTED)
        # A backend that was never reachable must not have touched the mission.
        self.assertEqual(calls, [])

    def test_link_connects_once_the_backend_appears(self):
        # Backend unavailable at boot, then started: the Pi must find it
        # without being restarted.
        async def scenario():
            server = RecordingServer(require_token=True)
            await server.start()
            port = server.port
            await server.stop()  # free the port; the Pi will fail to connect

            cfg = BackendConfig(
                enabled=True,
                server_url=f"http://127.0.0.1:{port}",
                robot_id=ROBOT_ID,
                robot_token=VALID_TOKEN,
                backoff_initial_s=0.1,
                backoff_max_s=0.5,
            )
            link = BackendLink(cfg, state_with_fix(), FakeAgent())
            later = RecordingServer(require_token=True)
            try:
                await link.start()
                await wait_for(lambda: link.stats["connect_failures"] >= 1, 6.0)

                # Bring a server up on the same port the Pi keeps retrying.
                later.app = web.Application()
                later.sio.attach(later.app)
                later.runner = web.AppRunner(later.app)
                await later.runner.setup()
                site = web.TCPSite(later.runner, "127.0.0.1", port)
                await site.start()
                later.port = port

                return await wait_for(lambda: link.connected, 15.0)
            finally:
                await link.stop()
                await later.stop()

        self.assertTrue(asyncio.run(asyncio.wait_for(scenario(), 40.0)),
                        "link did not connect after the backend came up")


class TestIntegrationClaims(SocketIOTestCase):
    def test_a_working_transport_is_still_not_reported_as_integrated(self):
        # The whole point: everything below works, and the link still refuses
        # to claim integration, because the event names are guesses.
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            await wait_for(lambda: server.received["telemetry"], 8.0)
            return link.describe()

        described = self.run_scenario(scenario)
        self.assertEqual(described["status"], "CONNECTED")
        self.assertTrue(described["stats"]["telemetry_sent"] >= 1)
        self.assertIs(described["integrated"], False)

    def test_a_declared_binding_over_a_live_socket_is_reported_integrated(self):
        async def scenario(server, link, agent, state):
            await link.start()
            self.assertTrue(await link.wait_connected(8.0))
            return link.describe()

        described = self.run_scenario(
            scenario, binding=ProtocolBinding(source=BindingSource.FILE)
        )
        self.assertIs(described["integrated"], True)


if __name__ == "__main__":
    unittest.main()
