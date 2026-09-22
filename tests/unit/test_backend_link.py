"""The backend link, driven against a fake Socket.IO client.

The transport is faked here so that connection lifecycle, rates, backoff and
dispatch can be examined deterministically. The real Socket.IO wire is
exercised separately in `tests/integration/test_socketio_transport.py` -- these
two are complements, and neither replaces the other.
"""

import asyncio
import json
import logging
import time
import unittest

from robotx.communication.backend_link import (
    BackendConfig,
    BackendLink,
    BackendLossPolicy,
    _looks_like_rejection,
    _safe_url,
)
from robotx.communication.protocol import BindingSource, EventLevel, ProtocolBinding
from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus
from robotx.localization.position import Position
from robotx.state.robot_state import LinkStatus, OperatingMode, RobotState


def setUpModule():
    # These tests deliberately drive warning paths; the log noise is not the
    # subject under test.
    logging.getLogger("robotx").setLevel(logging.CRITICAL)


class FakeSio:
    """Stands in for `socketio.AsyncClient`, recording what was emitted."""

    def __init__(self, *, fail_connect=None):
        self.handlers = {}
        self.emitted = []
        self.connected = False
        self.disconnect_calls = 0
        self.connect_calls = []
        self._fail_connect = fail_connect

    # --- handler registration (mirrors python-socketio's API) ---------------

    def event(self, *args, namespace=None):
        def decorator(fn):
            self.handlers[fn.__name__] = fn
            return fn

        if args and callable(args[0]):
            return decorator(args[0])
        return decorator

    def on(self, name, namespace=None):
        def decorator(fn):
            self.handlers[name] = fn
            return fn

        return decorator

    # --- transport ----------------------------------------------------------

    async def connect(self, url, namespaces=None, auth=None):
        self.connect_calls.append({"url": url, "namespaces": namespaces, "auth": auth})
        if self._fail_connect is not None:
            raise self._fail_connect
        self.connected = True
        handler = self.handlers.get("connect")
        if handler is not None:
            await handler()

    async def emit(self, event, payload, namespace=None):
        if not self.connected:
            raise RuntimeError("not connected")
        self.emitted.append((event, payload))

    async def disconnect(self):
        self.disconnect_calls += 1
        self.connected = False

    # --- test helpers -------------------------------------------------------

    async def fire(self, event, *args):
        await self.handlers[event](*args)

    async def drop(self):
        self.connected = False
        await self.handlers["disconnect"]()

    def events_named(self, name):
        return [payload for event, payload in self.emitted if event == name]


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


def state_with_fix(robot_id="robotx-pi"):
    state = RobotState(robot_id)
    now = time.time()
    state.update_gps(
        GpsReading(
            status=GPSStatus.FIX,
            fix=GpsFix(latitude=1.0, longitude=2.0, timestamp=now),
            age_s=0.1,
        ),
        Position(latitude=1.0, longitude=2.0, timestamp=now, speed_mps=0.5),
    )
    return state


def make_link(state=None, agent=None, sio=None, **cfg_kwargs):
    cfg = BackendConfig(enabled=True, robot_id="robotx-pi", **cfg_kwargs)
    sio = sio or FakeSio()
    link = BackendLink(
        cfg,
        state or state_with_fix(),
        agent or FakeAgent(),
        client_factory=lambda _cfg: sio,
    )
    return link, sio


class AsyncTestCase(unittest.TestCase):
    def run_async(self, coro):
        return asyncio.run(coro)


class TestConnectionAndAuth(AsyncTestCase):
    def test_token_is_sent_in_the_handshake(self):
        async def scenario():
            link, sio = make_link(robot_token="s3cr3t")
            await link._connect_once()
            return sio

        sio = self.run_async(scenario())
        auth = sio.connect_calls[0]["auth"]
        self.assertEqual(auth["robotId"], "robotx-pi")
        self.assertEqual(auth["token"], "s3cr3t")

    def test_no_token_still_connects_but_sends_none(self):
        async def scenario():
            link, sio = make_link(robot_token=None)
            await link._connect_once()
            return sio

        sio = self.run_async(scenario())
        self.assertNotIn("token", sio.connect_calls[0]["auth"])

    def test_token_never_appears_in_any_emitted_payload(self):
        async def scenario():
            link, sio = make_link(robot_token="s3cr3t")
            await link._connect_once()
            await link._publish_telemetry()
            await link.emit_event(EventLevel.INFO, "hello")
            return sio

        sio = self.run_async(scenario())
        body = json.dumps(sio.emitted, default=str)
        self.assertNotIn("s3cr3t", body)

    def test_connect_sets_connected_state(self):
        async def scenario():
            link, _ = make_link()
            await link._connect_once()
            return link

        link = self.run_async(scenario())
        self.assertIs(link.status, LinkStatus.CONNECTED)
        self.assertTrue(link.connected)

    def test_connect_registers_identity_then_status(self):
        async def scenario():
            link, sio = make_link()
            await link._connect_once()
            return sio

        sio = self.run_async(scenario())
        self.assertEqual([event for event, _ in sio.emitted][:2], ["robot_hello", "status"])
        register = sio.events_named("robot_hello")[0]
        self.assertIs(register["simulated"], False)
        self.assertEqual(register["robotId"], "robotx-pi")

    def test_disconnect_is_reflected_in_state(self):
        async def scenario():
            state = state_with_fix()
            link, sio = make_link(state=state)
            await link._connect_once()
            await sio.drop()
            return link, state

        link, state = self.run_async(scenario())
        self.assertIs(link.status, LinkStatus.DISCONNECTED)
        self.assertIs(state.snapshot().communication.backend, LinkStatus.DISCONNECTED)

    def test_state_never_claims_connected_while_socket_is_down(self):
        async def scenario():
            state = state_with_fix()
            link, sio = make_link(state=state)
            await link._connect_once()
            await sio.drop()
            # An emit attempted after the drop must not resurrect the state.
            await link._publish_telemetry()
            return state

        state = self.run_async(scenario())
        self.assertIsNot(state.snapshot().communication.backend, LinkStatus.CONNECTED)

    def test_disabled_link_start_is_a_no_op(self):
        async def scenario():
            cfg = BackendConfig(enabled=False)
            link = BackendLink(cfg, RobotState("r"), FakeAgent(), client_factory=lambda c: FakeSio())
            await link.start()
            return link

        link = self.run_async(scenario())
        self.assertIs(link.status, LinkStatus.DISABLED)

    def test_stop_without_start_is_safe(self):
        async def scenario():
            link, _ = make_link()
            await link.stop()

        self.run_async(scenario())

    def test_stop_disconnects_the_socket(self):
        async def scenario():
            link, sio = make_link()
            await link._connect_once()
            await link.stop()
            return sio

        sio = self.run_async(scenario())
        self.assertGreaterEqual(sio.disconnect_calls, 1)


class TestBackoff(AsyncTestCase):
    def test_backoff_grows_and_is_capped(self):
        link, _ = make_link(backoff_initial_s=1.0, backoff_max_s=30.0)
        delays = [link.next_backoff_s(n) for n in range(12)]
        for delay in delays:
            self.assertGreater(delay, 0.0)
            self.assertLessEqual(delay, 30.0)
        # Later attempts wait longer than the first, jitter notwithstanding.
        self.assertGreater(max(delays[6:]), max(delays[:2]))

    def test_backoff_is_jittered_not_identical(self):
        # A fleet reconnecting in lockstep would re-crash the backend it is
        # waiting for.
        link, _ = make_link(backoff_initial_s=4.0, backoff_max_s=60.0)
        samples = {round(link.next_backoff_s(3), 6) for _ in range(20)}
        self.assertGreater(len(samples), 1)

    def test_connect_failure_backs_off_and_does_not_raise(self):
        async def scenario():
            sio = FakeSio(fail_connect=ConnectionError("Connection refused"))
            link, _ = make_link(sio=sio, backoff_initial_s=0.01, backoff_max_s=0.02)
            link._running = True
            task = asyncio.create_task(link._run())
            await asyncio.sleep(0.4)
            link._running = False
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            return link

        link = self.run_async(scenario())
        self.assertGreater(link.stats["connect_failures"], 1)

    def test_auth_rejection_is_classified_separately(self):
        # The message python-socketio actually raises when a server's connect
        # handler refuses the namespace. Verified against 5.11.4.
        self.assertTrue(
            _looks_like_rejection(ConnectionError("One or more namespaces failed to connect"))
        )
        self.assertTrue(_looks_like_rejection(ConnectionError("Unauthorized")))
        self.assertTrue(_looks_like_rejection(ConnectionError("403 forbidden")))
        self.assertTrue(_looks_like_rejection(ConnectionError("invalid token")))

    def test_ordinary_network_failures_are_not_auth_rejections(self):
        # ECONNREFUSED is what a *stopped* backend looks like. Putting it on
        # the long rejection backoff would delay noticing the server's return.
        for message in (
            "Connection refused",
            "Cannot connect to host 127.0.0.1:1 ssl:default [Connect call failed]",
            "Cannot connect to host backend:3000 ssl:default [Name or service not known]",
            "Network is unreachable",
            "Connection reset by peer",
            "timed out",
        ):
            self.assertFalse(_looks_like_rejection(ConnectionError(message)), message)

    def test_rejected_connection_is_reported_as_rejected(self):
        async def scenario():
            sio = FakeSio(fail_connect=ConnectionError("Unauthorized"))
            link, _ = make_link(sio=sio, backoff_rejected_s=0.01)
            link._running = True
            task = asyncio.create_task(link._run())
            await asyncio.sleep(0.05)
            link._running = False
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            return link

        link = self.run_async(scenario())
        self.assertIs(link.status, LinkStatus.REJECTED)


class TestTelemetryPublishing(AsyncTestCase):
    def test_telemetry_is_published_when_there_is_a_fix(self):
        async def scenario():
            link, sio = make_link()
            await link._connect_once()
            await link._publish_telemetry()
            return link, sio

        link, sio = self.run_async(scenario())
        self.assertEqual(link.stats["telemetry_sent"], 1)
        self.assertEqual(sio.events_named("telemetry")[0]["lat"], 1.0)

    def test_telemetry_is_skipped_without_a_fix(self):
        async def scenario():
            state = RobotState("robotx-pi")  # no GPS update at all
            link, sio = make_link(state=state)
            await link._connect_once()
            await link._publish_telemetry()
            return link, sio

        link, sio = self.run_async(scenario())
        self.assertEqual(link.stats["telemetry_sent"], 0)
        self.assertEqual(link.stats["telemetry_skipped"], 1)
        self.assertEqual(sio.events_named("telemetry"), [])

    def test_publish_loop_respects_the_telemetry_interval(self):
        async def scenario():
            link, sio = make_link(telemetry_interval_s=10.0, status_interval_s=10.0)
            await link._connect_once()
            link._running = True
            task = asyncio.create_task(link._publish_loop())
            await asyncio.sleep(0.6)
            link._running = False
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            return sio

        sio = self.run_async(scenario())
        # At a 10 s interval, 0.6 s of loop may only produce the first frame.
        self.assertLessEqual(len(sio.events_named("telemetry")), 1)

    def test_fast_interval_produces_multiple_frames(self):
        async def scenario():
            link, sio = make_link(telemetry_interval_s=0.05, status_interval_s=100.0)
            await link._connect_once()
            link._running = True
            task = asyncio.create_task(link._publish_loop())
            await asyncio.sleep(0.8)
            link._running = False
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            return sio

        sio = self.run_async(scenario())
        self.assertGreater(len(sio.events_named("telemetry")), 1)

    def test_emit_failure_is_counted_not_raised(self):
        class Broken(FakeSio):
            async def emit(self, event, payload, namespace=None):
                raise RuntimeError("socket write failed")

        async def scenario():
            sio = Broken()
            link, _ = make_link(sio=sio)
            await link._connect_once()
            await link._publish_telemetry()
            return link

        link = self.run_async(scenario())
        self.assertGreater(link.stats["emit_failures"], 0)
        self.assertEqual(link.stats["telemetry_sent"], 0)


class TestEventChannel(AsyncTestCase):
    def test_event_is_published(self):
        async def scenario():
            link, sio = make_link()
            await link._connect_once()
            sent = await link.emit_event(EventLevel.WARNING, "gps lost")
            return sent, sio

        sent, sio = self.run_async(scenario())
        self.assertTrue(sent)
        self.assertEqual(sio.events_named("event")[0]["type"], "WARNING")

    def test_repeated_identical_events_are_suppressed(self):
        async def scenario():
            link, sio = make_link()
            await link._connect_once()
            await link.emit_event(EventLevel.WARNING, "gps lost")
            second = await link.emit_event(EventLevel.WARNING, "gps lost")
            return second, link, sio

        second, link, sio = self.run_async(scenario())
        self.assertFalse(second)
        self.assertEqual(len(sio.events_named("event")), 1)
        self.assertGreater(link.stats["events_suppressed"], 0)


class TestCommandDispatch(AsyncTestCase):
    def dispatch(self, payload, agent=None, **cfg):
        async def scenario():
            link, sio = make_link(agent=agent or FakeAgent(), **cfg)
            await link._connect_once()
            await sio.fire("command", payload)
            return link, sio

        return self.run_async(scenario())

    def test_valid_command_is_applied_and_acked(self):
        agent = FakeAgent()
        link, sio = self.dispatch({"commandId": "c1", "type": "STOP"}, agent=agent)
        self.assertEqual(agent.calls[0][0], "stop")
        ack = sio.events_named("command_ack")[0]
        self.assertEqual(ack["status"], "ACK")
        self.assertEqual(ack["commandId"], "c1")

    def test_ack_is_emitted_after_the_command_is_applied(self):
        # An ACK sent before the effect would be a claim the Pi cannot back up.
        order = []

        class Recording(FakeAgent):
            def stop_mission(self, reason=""):
                order.append("applied")
                super().stop_mission(reason)

        class Watching(FakeSio):
            async def emit(self, event, payload, namespace=None):
                if event == "command_ack":
                    order.append("acked")
                await super().emit(event, payload, namespace=namespace)

        async def scenario():
            link, sio = make_link(agent=Recording(), sio=Watching())
            await link._connect_once()
            await sio.fire("command", {"commandId": "c1", "type": "STOP"})

        self.run_async(scenario())
        self.assertEqual(order, ["applied", "acked"])

    def test_unknown_command_is_failed_not_executed(self):
        agent = FakeAgent()
        link, sio = self.dispatch({"commandId": "c1", "type": "LAUNCH"}, agent=agent)
        self.assertEqual(agent.calls, [])
        ack = sio.events_named("command_ack")[0]
        self.assertEqual(ack["status"], "FAILED")
        self.assertIn("UNKNOWN_TYPE", ack["reason"])

    def test_malformed_command_produces_no_action(self):
        agent = FakeAgent()
        link, sio = self.dispatch("STOP", agent=agent)
        self.assertEqual(agent.calls, [])
        self.assertEqual(link.stats["commands_rejected"], 1)

    def test_command_without_an_id_is_not_acked(self):
        link, sio = self.dispatch({"type": "STOP"})
        self.assertEqual(sio.events_named("command_ack"), [])
        self.assertEqual(link.stats["commands_rejected"], 1)

    def test_command_for_another_robot_is_refused(self):
        agent = FakeAgent()
        link, sio = self.dispatch({"commandId": "c", "type": "STOP", "robotId": "other"}, agent=agent)
        self.assertEqual(agent.calls, [])
        self.assertEqual(sio.events_named("command_ack")[0]["status"], "FAILED")

    def test_stale_command_is_refused(self):
        agent = FakeAgent()
        link, sio = self.dispatch(
            {"commandId": "c", "type": "STOP", "issuedAt": time.time() - 9999},
            agent=agent,
            command_max_age_s=60.0,
        )
        self.assertEqual(agent.calls, [])
        self.assertIn("STALE", sio.events_named("command_ack")[0]["reason"])

    def test_duplicate_command_executes_once_but_acks_twice(self):
        async def scenario():
            agent = FakeAgent()
            link, sio = make_link(agent=agent)
            await link._connect_once()
            await sio.fire("command", {"commandId": "dup", "type": "STOP"})
            await sio.fire("command", {"commandId": "dup", "type": "STOP"})
            return agent, link, sio

        agent, link, sio = self.run_async(scenario())
        self.assertEqual(len(agent.calls), 1)
        self.assertEqual(len(sio.events_named("command_ack")), 2)
        self.assertEqual(link.stats["commands_duplicate"], 1)

    def test_a_failing_command_does_not_stop_the_next_one(self):
        async def scenario():
            agent = FakeAgent()
            link, sio = make_link(agent=agent)
            await link._connect_once()
            await sio.fire("command", {"type": "GARBAGE"})
            await sio.fire("command", {"commandId": "c2", "type": "STOP"})
            return agent

        agent = self.run_async(scenario())
        self.assertEqual(agent.calls[0][0], "stop")

    def test_unexpected_event_is_counted_not_obeyed(self):
        async def scenario():
            agent = FakeAgent()
            link, sio = make_link(agent=agent)
            await link._connect_once()
            await sio.fire("*", "drive_forward", {"speed": 1.0})
            return agent, link

        agent, link = self.run_async(scenario())
        self.assertEqual(agent.calls, [])
        self.assertEqual(link.stats["unexpected_events"], 1)


class TestLinkLossPolicy(AsyncTestCase):
    def test_pause_policy_suspends_the_mission_after_the_grace_period(self):
        agent = FakeAgent(mode=OperatingMode.AUTO)
        link, _ = make_link(agent=agent, loss_grace_s=0.0,
                            loss_policy=BackendLossPolicy.PAUSE)
        link._disconnected_since = time.monotonic() - 10
        link._apply_loss_policy()
        self.assertEqual(agent.calls[0][0], "pause")

    def test_pause_policy_waits_out_a_brief_blip(self):
        agent = FakeAgent(mode=OperatingMode.AUTO)
        link, _ = make_link(agent=agent, loss_grace_s=60.0,
                            loss_policy=BackendLossPolicy.PAUSE)
        link._disconnected_since = time.monotonic()
        link._apply_loss_policy()
        self.assertEqual(agent.calls, [])

    def test_continue_policy_leaves_the_mission_alone(self):
        agent = FakeAgent(mode=OperatingMode.AUTO)
        link, _ = make_link(agent=agent, loss_grace_s=0.0,
                            loss_policy=BackendLossPolicy.CONTINUE)
        link._disconnected_since = time.monotonic() - 10
        link._apply_loss_policy()
        self.assertEqual(agent.calls, [])

    def test_policy_fires_only_once_per_outage(self):
        agent = FakeAgent(mode=OperatingMode.AUTO)
        link, _ = make_link(agent=agent, loss_grace_s=0.0)
        link._disconnected_since = time.monotonic() - 10
        link._apply_loss_policy()
        link._apply_loss_policy()
        self.assertEqual(len(agent.calls), 1)

    def test_policy_never_starts_or_resumes_anything(self):
        # Losing the backend must only ever reduce motion.
        for mode in (OperatingMode.IDLE, OperatingMode.PAUSED, OperatingMode.STOPPED):
            agent = FakeAgent(mode=mode)
            link, _ = make_link(agent=agent, loss_grace_s=0.0)
            link._disconnected_since = time.monotonic() - 10
            link._apply_loss_policy()
            self.assertEqual(agent.calls, [], mode)


class TestHonestReporting(AsyncTestCase):
    def test_provisional_binding_is_never_reported_as_integrated(self):
        async def scenario():
            link, _ = make_link()
            await link._connect_once()
            return link.describe()

        described = self.run_async(scenario())
        self.assertIs(described["integrated"], False)
        self.assertTrue(described["protocol"]["provisional"])
        self.assertEqual(described["status"], "CONNECTED")

    def test_a_real_binding_plus_a_connection_is_reported_as_integrated(self):
        async def scenario():
            binding = ProtocolBinding(source=BindingSource.FILE)
            link, _ = make_link(binding=binding)
            await link._connect_once()
            return link.describe()

        described = self.run_async(scenario())
        self.assertIs(described["integrated"], True)

    def test_a_real_binding_without_a_connection_is_not_integrated(self):
        link, _ = make_link(binding=ProtocolBinding(source=BindingSource.FILE))
        self.assertIs(link.describe()["integrated"], False)

    def test_describe_does_not_leak_the_token(self):
        link, _ = make_link(robot_token="s3cr3t")
        self.assertNotIn("s3cr3t", json.dumps(link.describe(), default=str))
        self.assertIs(link.describe()["authenticated_client_side"], True)

    def test_url_credentials_are_stripped_for_logging(self):
        self.assertNotIn("hunter2", _safe_url("https://bob:hunter2@example.com:3000/x"))
        self.assertIn("example.com:3000", _safe_url("https://bob:hunter2@example.com:3000/x"))

    def test_tls_is_detected_from_the_url(self):
        insecure, _ = make_link(server_url="http://example.com")
        secure, _ = make_link(server_url="https://example.com")
        self.assertFalse(insecure.cfg.uses_tls)
        self.assertTrue(secure.cfg.uses_tls)


if __name__ == "__main__":
    unittest.main()
