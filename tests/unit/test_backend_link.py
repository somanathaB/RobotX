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
    _safe_url,
)
from robotx.communication.protocol import BindingSource, EventLevel, ProtocolBinding
from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus
from robotx.localization.position import Position
from robotx.mission.manager import MissionAssignment
from robotx.mission.mission import (
    ActiveMission,
    MissionRejected,
    MissionRejectReason,
    MissionSegment,
    MissionStatus,
)
from robotx.state.robot_state import BackendLinkStatus as LinkStatus, OperatingMode, RobotState
from tests.fixtures.task_assign import task_assign_payload


def setUpModule():
    # These tests deliberately drive warning paths; the log noise is not the
    # subject under test.
    logging.getLogger("robotx").setLevel(logging.CRITICAL)


class FakeSio:
    """Stands in for `socketio.AsyncClient`, recording what was emitted."""

    def __init__(self, *, fail_connect=None, auth_mode="success", auth_token="sess-tok"):
        self.handlers = {}
        self.emitted = []
        self.connected = False
        self.disconnect_calls = 0
        self.connect_calls = []
        self._fail_connect = fail_connect
        # How this fake backend answers AUTH:
        #   success           -> AUTH_SUCCESS carrying a token
        #   ok_event          -> AUTH_OK carrying a token (the other name)
        #   no_token          -> AUTH_SUCCESS with nothing usable in it
        #   silent_disconnect -> disconnect(true), which is what FalconAut does
        #   failed            -> an explicit AUTH_FAILED event
        #   none              -> no answer at all (drives the auth timeout)
        self.auth_mode = auth_mode
        self.auth_token = auth_token
        self.registration_count = 0

    # --- handler registration (mirrors python-socketio's API) ---------------

    def event(self, *args, namespace=None):
        def decorator(fn):
            self.registration_count += 1
            self.handlers[fn.__name__] = fn
            return fn

        if args and callable(args[0]):
            return decorator(args[0])
        return decorator

    def on(self, name, namespace=None):
        def decorator(fn):
            self.registration_count += 1
            self.handlers[name] = fn
            return fn

        return decorator

    # --- transport ----------------------------------------------------------

    async def connect(self, url, namespaces=None, auth=None, headers=None):
        self.connect_calls.append(
            {"url": url, "namespaces": namespaces, "auth": auth, "headers": headers}
        )
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
        if event == "AUTH":
            await self._answer_auth()

    async def _answer_auth(self):
        """Behave like the backend's AUTH handler."""

        if self.auth_mode == "success":
            await self.fire("AUTH_SUCCESS", {"token": self.auth_token})
        elif self.auth_mode == "ok_event":
            # The contract's other success event name.
            await self.fire("AUTH_OK", {"token": self.auth_token})
        elif self.auth_mode == "no_token":
            await self.fire("AUTH_SUCCESS", {"ok": True})
        elif self.auth_mode == "failed":
            await self.fire("AUTH_FAILED", {"reason": "invalid pairing code"})
        elif self.auth_mode == "silent_disconnect":
            # What FalconAut actually does: disconnect(true), no error event.
            await self.drop()

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
    def __init__(self, mode=OperatingMode.AUTO, *, refuse=None, duplicate_assignments=False):
        self._mode = mode
        self.calls = []
        self.assigned = []
        self.refuse = refuse
        self.duplicate_assignments = duplicate_assignments

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
        """Declines by default: a fake agent has no hardware to execute with."""

        from robotx.communication.engine import OfferDecision

        self.calls.append(("assess", offer.commitment_id))
        return OfferDecision.reject("FAKE_AGENT")

    def assign_mission(self, mission, custody_required=False):
        """Stands in for the agent's mission entry point.

        Takes a validated `Mission`, never a payload: by the time the link
        calls this, the assignment has already been through
        `parse_task_assign`. `refuse` makes it decline like a real agent would
        (an active mission, a latched estop), which is the interesting case for
        the link -- it must report, not crash.
        """

        self.calls.append(("assign", mission.task_id))
        if self.refuse is not None:
            raise self.refuse
        self.assigned.append(mission)
        self._mode = OperatingMode.AUTO
        return MissionAssignment(
            active=ActiveMission(
                mission=mission,
                status=MissionStatus.TO_PICKUP,
                segment=MissionSegment.TO_PICKUP,
                accepted_at=time.time(),
                waypoints_total=len(mission.path_to_pickup),
            ),
            duplicate=self.duplicate_assignments,
        )


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


class MemoryTokenStore:
    """A TokenStore that never touches the filesystem.

    Tests must not read or write the real `~/.robotx/backend_session.json`: a
    test run would otherwise overwrite the credential of the robot this Pi is
    actually commissioned as.
    """

    def __init__(self, token=None):
        self.token = token
        self.saves = []
        self.clears = 0

    def load(self, *, robot_id):
        if not self.token:
            return None
        from robotx.communication.token_store import StoredSession

        return StoredSession(robot_id=robot_id, token=self.token, obtained_at=time.time())

    def save(self, *, robot_id, token):
        self.saves.append((robot_id, token))
        self.token = token
        return True

    def clear(self):
        self.clears += 1
        self.token = None

    def describe(self, *, robot_id):
        return {"path": "<memory>", "token": "SET" if self.token else "UNSET", "age_s": None}


def make_link(state=None, agent=None, sio=None, tokens=None, **cfg_kwargs):
    cfg_kwargs.setdefault("pairing_code", "123456")
    cfg = BackendConfig(enabled=True, robot_id="robotx-pi", **cfg_kwargs)
    sio = sio or FakeSio()
    link = BackendLink(
        cfg,
        state or state_with_fix(),
        agent or FakeAgent(),
        client_factory=lambda _cfg: sio,
        token_store=tokens if tokens is not None else MemoryTokenStore(),
    )
    return link, sio


def _binding_with_events():
    """The FalconAut binding plus an operator-bound event channel."""

    return ProtocolBinding(source=BindingSource.EXPLICIT, event="event", status="status")


async def connect_and_auth(link):
    """Drive a link through connect -> AUTH -> AUTH_SUCCESS."""

    await link._connect_once()
    return await link._await_authentication()


class AsyncTestCase(unittest.TestCase):
    def run_async(self, coro):
        return asyncio.run(coro)


class TestConnectionAndAuth(AsyncTestCase):
    def test_connection_is_anonymous(self):
        """FalconAut reads no handshake credential, so the Pi sends none."""

        async def scenario():
            link, sio = make_link(robot_token="s3cr3t")
            await link._connect_once()
            return sio

        sio = self.run_async(scenario())
        call = sio.connect_calls[0]
        self.assertIsNone(call["auth"])
        # Nor smuggled into the URL as a query parameter.
        self.assertNotIn("s3cr3t", call["url"])

    def test_client_does_not_present_itself_as_a_browser(self):
        async def scenario():
            link, sio = make_link()
            await link._connect_once()
            return sio

        sio = self.run_async(scenario())
        headers = sio.connect_calls[0]["headers"] or {}
        self.assertNotIn("Origin", headers)
        self.assertNotIn("Mozilla", headers.get("User-Agent", ""))
        self.assertIn("robotx-pi", headers.get("User-Agent", ""))

    def test_auth_is_emitted_after_connect_not_during_handshake(self):
        async def scenario():
            link, sio = make_link()
            await link._connect_once()
            return sio

        sio = self.run_async(scenario())
        self.assertEqual(sio.emitted[0][0], "AUTH")
        self.assertEqual(sio.emitted[0][1]["robotId"], "robotx-pi")

    def test_pairing_code_is_used_when_no_token_is_stored(self):
        async def scenario():
            link, sio = make_link(tokens=MemoryTokenStore(token=None), pairing_code="654321")
            await connect_and_auth(link)
            return link, sio

        link, sio = self.run_async(scenario())
        payload = sio.events_named("AUTH")[0]
        self.assertEqual(payload["pairingCode"], "654321")
        self.assertNotIn("token", payload)
        self.assertEqual(link.describe()["auth_method"], "PAIRING_CODE")

    def test_stored_token_is_preferred_over_the_pairing_code(self):
        """The code is single-use with a 300 s TTL; it must not be burned."""

        async def scenario():
            link, sio = make_link(
                tokens=MemoryTokenStore(token="stored-tok"), pairing_code="654321"
            )
            await connect_and_auth(link)
            return link, sio

        link, sio = self.run_async(scenario())
        payload = sio.events_named("AUTH")[0]
        self.assertEqual(payload["token"], "stored-tok")
        self.assertNotIn("pairingCode", payload)
        self.assertEqual(link.describe()["auth_method"], "TOKEN")

    def test_auth_success_persists_the_session_token(self):
        async def scenario():
            tokens = MemoryTokenStore(token=None)
            link, _ = make_link(tokens=tokens, sio=FakeSio(auth_token="fresh-tok"))
            await connect_and_auth(link)
            return link, tokens

        link, tokens = self.run_async(scenario())
        self.assertEqual(tokens.saves, [("robotx-pi", "fresh-tok")])
        self.assertIs(link.status, LinkStatus.AUTHENTICATED)
        self.assertTrue(link.connected)

    def test_link_is_not_usable_before_auth_succeeds(self):
        async def scenario():
            link, sio = make_link(sio=FakeSio(auth_mode="none"))
            await link._connect_once()
            # Connected, AUTH sent, no answer yet.
            usable = link.connected
            await link._publish_telemetry()
            return link, sio, usable

        link, sio, usable = self.run_async(scenario())
        self.assertFalse(usable)
        self.assertIs(link.status, LinkStatus.AUTHENTICATING)
        # Nothing but AUTH may go out on an unauthenticated socket.
        self.assertEqual([event for event, _ in sio.emitted], ["AUTH"])

    def test_silent_disconnect_during_auth_is_an_auth_failure(self):
        """FalconAut refuses by calling disconnect(true) with no error event."""

        async def scenario():
            link, sio = make_link(sio=FakeSio(auth_mode="silent_disconnect"))
            ok = await connect_and_auth(link)
            return link, ok

        link, ok = self.run_async(scenario())
        self.assertFalse(ok)
        self.assertIn("refused", link._auth_failure_detail)

    def test_explicit_auth_failed_event_is_honoured(self):
        async def scenario():
            link, _ = make_link(sio=FakeSio(auth_mode="failed"))
            ok = await connect_and_auth(link)
            return link, ok

        link, ok = self.run_async(scenario())
        self.assertFalse(ok)
        self.assertIn("invalid pairing code", link._auth_failure_detail)

    def test_auth_times_out_when_the_backend_never_answers(self):
        async def scenario():
            link, _ = make_link(sio=FakeSio(auth_mode="none"), auth_timeout_s=0.05)
            ok = await connect_and_auth(link)
            return link, ok

        link, ok = self.run_async(scenario())
        self.assertFalse(ok)
        self.assertIn("no AUTH_SUCCESS", link._auth_failure_detail)

    def test_refused_token_is_discarded_so_the_pairing_code_is_tried_next(self):
        async def scenario():
            tokens = MemoryTokenStore(token="stale-tok")
            link, _ = make_link(tokens=tokens, sio=FakeSio(auth_mode="silent_disconnect"))
            await connect_and_auth(link)
            await link._handle_auth_failure()
            return tokens

        tokens = self.run_async(scenario())
        self.assertEqual(tokens.clears, 1)
        self.assertIsNone(tokens.token)

    def test_refused_pairing_code_is_kept(self):
        """A 300 s TTL makes expiry far likelier than a wrong code."""

        async def scenario():
            tokens = MemoryTokenStore(token=None)
            link, _ = make_link(tokens=tokens, sio=FakeSio(auth_mode="silent_disconnect"))
            await connect_and_auth(link)
            await link._handle_auth_failure()
            return link, tokens

        link, tokens = self.run_async(scenario())
        self.assertEqual(tokens.clears, 0)
        self.assertEqual(link.cfg.pairing_code, "123456")

    def test_auth_without_any_credential_fails_rather_than_emitting(self):
        async def scenario():
            link, sio = make_link(tokens=MemoryTokenStore(token=None), pairing_code=None)
            ok = await connect_and_auth(link)
            return link, sio, ok

        link, sio, ok = self.run_async(scenario())
        self.assertFalse(ok)
        self.assertEqual(sio.events_named("AUTH"), [])
        self.assertIn("no credential", link._auth_failure_detail)

    def test_auth_success_without_a_token_still_authenticates(self):
        async def scenario():
            tokens = MemoryTokenStore(token=None)
            link, _ = make_link(tokens=tokens, sio=FakeSio(auth_mode="no_token"))
            ok = await connect_and_auth(link)
            return link, tokens, ok

        link, tokens, ok = self.run_async(scenario())
        self.assertTrue(ok)
        self.assertIs(link.status, LinkStatus.AUTHENTICATED)
        self.assertEqual(tokens.saves, [])

    def test_token_never_appears_in_any_emitted_payload_after_auth(self):
        async def scenario():
            link, sio = make_link(
                tokens=MemoryTokenStore(token=None),
                sio=FakeSio(auth_token="s3cr3t"),
            )
            await connect_and_auth(link)
            await link._publish_telemetry()
            return sio

        sio = self.run_async(scenario())
        # AUTH legitimately carries the credential; nothing after it may.
        after_auth = [(e, p) for e, p in sio.emitted if e != "AUTH"]
        self.assertNotIn("s3cr3t", json.dumps(after_auth, default=str))

    def test_auth_ok_authenticates_exactly_as_auth_success_does(self):
        """The contract names two success events; binding one is not enough."""

        async def scenario():
            tokens = MemoryTokenStore(token=None)
            link, _ = make_link(tokens=tokens, sio=FakeSio(auth_mode="ok_event"))
            ok = await connect_and_auth(link)
            return link, tokens, ok

        link, tokens, ok = self.run_async(scenario())
        self.assertTrue(ok)
        self.assertIs(link.status, LinkStatus.AUTHENTICATED)
        # And the token it carried is persisted just the same.
        self.assertEqual(tokens.saves, [("robotx-pi", "sess-tok")])

    def test_both_success_event_names_are_bound_on_the_socket(self):
        async def scenario():
            link, sio = make_link()
            await link._connect_once()
            return sio

        sio = self.run_async(scenario())
        self.assertIn("AUTH_SUCCESS", sio.handlers)
        self.assertIn("AUTH_OK", sio.handlers)

    def test_a_success_event_is_not_registered_twice_when_names_collide(self):
        """An operator correcting one name to match the other must not end up
        with two handlers for one event."""

        binding = ProtocolBinding(
            source=BindingSource.FILE, auth_success="AUTH_OK", auth_ok="AUTH_OK"
        )
        self.assertEqual(binding.auth_success_events(), ("AUTH_OK",))

    def test_handlers_are_registered_exactly_once_across_reconnects(self):
        """socket.io-client reuses its emitter; a second registration would
        execute every inbound COMMAND twice."""

        async def scenario():
            link, sio = make_link()
            await connect_and_auth(link)
            first = sio.registration_count
            await sio.drop()
            await connect_and_auth(link)
            await sio.drop()
            await connect_and_auth(link)
            return link, sio, first

        link, sio, first = self.run_async(scenario())
        self.assertEqual(sio.registration_count, first)
        self.assertEqual(link.handler_registrations, 1)

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

    async def _run_briefly(self, link, seconds=0.05):
        link._running = True
        task = asyncio.create_task(link._run())
        await asyncio.sleep(seconds)
        link._running = False
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        return link

    def test_transport_failure_is_never_an_auth_failure(self):
        """ECONNREFUSED is what a *stopped* backend looks like.

        Putting it on the long auth backoff would delay noticing the server's
        return by up to a minute every time it restarted.
        """

        for message in (
            "Connection refused",
            "Cannot connect to host 127.0.0.1:1 ssl:default [Connect call failed]",
            "Cannot connect to host backend:3000 ssl:default [Name or service not known]",
            "Network is unreachable",
            "Connection reset by peer",
            "timed out",
            # Even a message that *sounds* like authorization: the transport
            # never got far enough for this robot to have been refused.
            "Unauthorized",
        ):
            with self.subTest(message=message):
                async def scenario():
                    sio = FakeSio(fail_connect=ConnectionError(message))
                    link, _ = make_link(sio=sio, backoff_initial_s=0.01)
                    return await self._run_briefly(link)

                link = self.run_async(scenario())
                self.assertIs(link.status, LinkStatus.DISCONNECTED)
                self.assertGreater(link.stats["connect_failures"], 0)
                self.assertEqual(link.stats["auth_failures"], 0)

    def test_refused_credential_is_reported_as_auth_failed(self):
        async def scenario():
            sio = FakeSio(auth_mode="silent_disconnect")
            link, _ = make_link(sio=sio, backoff_auth_failed_s=0.01)
            return await self._run_briefly(link, seconds=0.08)

        link = self.run_async(scenario())
        self.assertIs(link.status, LinkStatus.AUTH_FAILED)
        self.assertGreater(link.stats["auth_failures"], 0)
        # The transport was fine; this must not look like a connect failure.
        self.assertEqual(link.stats["connect_failures"], 0)

    def test_backend_appearing_later_is_connected_to_without_a_restart(self):
        async def scenario():
            sio = FakeSio(fail_connect=ConnectionError("Connection refused"))
            link, _ = make_link(sio=sio, backoff_initial_s=0.01, backoff_max_s=0.02)
            link._running = True
            task = asyncio.create_task(link._run())
            await asyncio.sleep(0.05)
            self.assertIs(link.status, LinkStatus.DISCONNECTED)

            # The backend comes up.
            sio._fail_connect = None
            await asyncio.sleep(0.15)

            link._running = False
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            return link

        link = self.run_async(scenario())
        self.assertGreater(link.stats["auth_successes"], 0)


class TestTelemetryPublishing(AsyncTestCase):
    def test_telemetry_is_published_when_there_is_a_fix(self):
        async def scenario():
            link, sio = make_link()
            await link._connect_once()
            await link._publish_telemetry()
            return link, sio

        link, sio = self.run_async(scenario())
        self.assertEqual(link.stats["telemetry_sent"], 1)
        self.assertEqual(sio.events_named("TELEMETRY")[0]["lat"], 1.0)

    def test_telemetry_without_a_fix_omits_the_position(self):
        async def scenario():
            state = RobotState("robotx-pi")  # no GPS update at all
            link, sio = make_link(state=state)
            await link._connect_once()
            await link._publish_telemetry()
            return link, sio

        link, sio = self.run_async(scenario())
        # The frame still goes -- status is worth reporting -- with no position.
        self.assertEqual(link.stats["telemetry_sent"], 1)
        self.assertEqual(link.stats["positions_omitted"], 1)
        frame = sio.events_named("TELEMETRY")[0]
        self.assertNotIn("lat", frame)
        self.assertNotIn("lon", frame)

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
        self.assertLessEqual(len(sio.events_named("TELEMETRY")), 1)

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
        self.assertGreater(len(sio.events_named("TELEMETRY")), 1)

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
    """The operator-event channel is optional and unbound by default.

    FalconAut's robot contract does not declare it, so the Pi stays silent
    unless an operator binds a name to it. These tests bind one explicitly.
    """

    def test_unbound_event_channel_emits_nothing(self):
        async def scenario():
            link, sio = make_link()
            await connect_and_auth(link)
            sent = await link.emit_event(EventLevel.WARNING, "gps lost")
            return sent, sio

        sent, sio = self.run_async(scenario())
        self.assertFalse(sent)
        self.assertEqual(sio.events_named("event"), [])

    def test_event_is_published(self):
        async def scenario():
            link, sio = make_link(binding=_binding_with_events())
            await connect_and_auth(link)
            sent = await link.emit_event(EventLevel.WARNING, "gps lost")
            return sent, sio

        sent, sio = self.run_async(scenario())
        self.assertTrue(sent)
        self.assertEqual(sio.events_named("event")[0]["type"], "WARNING")

    def test_repeated_identical_events_are_suppressed(self):
        async def scenario():
            link, sio = make_link(binding=_binding_with_events())
            await connect_and_auth(link)
            await link.emit_event(EventLevel.WARNING, "gps lost")
            second = await link.emit_event(EventLevel.WARNING, "gps lost")
            return second, link, sio

        second, link, sio = self.run_async(scenario())
        self.assertFalse(second)
        self.assertEqual(len(sio.events_named("event")), 1)
        self.assertGreater(link.stats["events_suppressed"], 0)


class TestHeartbeat(AsyncTestCase):
    """Liveness must not depend on the robot knowing where it is."""

    async def _pump(self, link, seconds):
        link._running = True
        task = asyncio.create_task(link._publish_loop())
        await asyncio.sleep(seconds)
        link._running = False
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    def test_heartbeat_is_emitted_on_its_own_cadence(self):
        async def scenario():
            link, sio = make_link(heartbeat_interval_s=0.05, telemetry_interval_s=100.0)
            await connect_and_auth(link)
            await self._pump(link, 0.5)
            return link, sio

        link, sio = self.run_async(scenario())
        beats = sio.events_named("HEARTBEAT")
        self.assertGreater(len(beats), 1)
        self.assertGreater(link.stats["heartbeats_sent"], 1)

    def test_heartbeat_flows_with_no_gps_fix_at_all(self):
        """The whole point: telemetry goes silent indoors, liveness must not."""

        async def scenario():
            state = RobotState("robotx-pi")  # never given a fix
            link, sio = make_link(
                state=state, heartbeat_interval_s=0.05, telemetry_interval_s=0.05
            )
            await connect_and_auth(link)
            await self._pump(link, 0.5)
            return link, sio

        link, sio = self.run_async(scenario())
        self.assertTrue(all("lat" not in f for f in sio.events_named("TELEMETRY")))
        self.assertGreater(link.stats["positions_omitted"], 0)
        self.assertGreater(len(sio.events_named("HEARTBEAT")), 1)

    def test_one_heartbeat_is_sent_immediately_on_entering_streaming(self):
        async def scenario():
            link, sio = make_link(heartbeat_interval_s=100.0, telemetry_interval_s=100.0)
            await connect_and_auth(link)
            await self._pump(link, 0.05)
            return sio

        sio = self.run_async(scenario())
        self.assertEqual(len(sio.events_named("HEARTBEAT")), 1)

    def test_idle_heartbeat_payload_is_empty(self):
        """Handoff §4: `{}` when idle. Identity is the socket; time is the server's."""

        async def scenario():
            link, sio = make_link(heartbeat_interval_s=100.0, telemetry_interval_s=100.0)
            await connect_and_auth(link)
            await self._pump(link, 0.05)
            return sio

        sio = self.run_async(scenario())
        self.assertEqual(sio.events_named("HEARTBEAT")[0], {})

    def test_no_heartbeat_before_authentication(self):
        async def scenario():
            link, sio = make_link(sio=FakeSio(auth_mode="none"), heartbeat_interval_s=0.01)
            await link._connect_once()
            await link._publish_heartbeat()
            return link, sio

        link, sio = self.run_async(scenario())
        self.assertEqual(sio.events_named("HEARTBEAT"), [])
        self.assertEqual(link.stats["heartbeats_sent"], 0)


class TestCommandDispatch(AsyncTestCase):
    def dispatch(self, payload, agent=None, **cfg):
        async def scenario():
            link, sio = make_link(agent=agent or FakeAgent(), **cfg)
            await link._connect_once()
            await sio.fire("COMMAND", payload)
            return link, sio

        return self.run_async(scenario())

    def test_valid_command_is_applied_and_acked(self):
        agent = FakeAgent()
        link, sio = self.dispatch({"commandId": "c1", "type": "STOP"}, agent=agent)
        self.assertEqual(agent.calls[0][0], "stop")
        self.assertEqual(sio.events_named("COMMAND_ACK"), [{"commandId": "c1"}])

    def test_ack_is_emitted_after_the_command_is_applied(self):
        # An ACK sent before the effect would be a claim the Pi cannot back up.
        order = []

        class Recording(FakeAgent):
            def stop_mission(self, reason=""):
                order.append("applied")
                super().stop_mission(reason)

        class Watching(FakeSio):
            async def emit(self, event, payload, namespace=None):
                if event == "COMMAND_ACK":
                    order.append("acked")
                await super().emit(event, payload, namespace=namespace)

        async def scenario():
            link, sio = make_link(agent=Recording(), sio=Watching())
            await link._connect_once()
            await sio.fire("COMMAND", {"commandId": "c1", "type": "STOP"})

        self.run_async(scenario())
        self.assertEqual(order, ["applied", "acked"])

    def test_unknown_command_is_never_acked_or_executed(self):
        agent = FakeAgent()
        link, sio = self.dispatch({"commandId": "c1", "type": "LAUNCH"}, agent=agent)
        self.assertEqual(agent.calls, [])
        # Handoff §13: never ACK an unknown type. The backend marks it FAILED.
        self.assertEqual(sio.events_named("COMMAND_ACK"), [])

    def test_malformed_command_produces_no_action(self):
        agent = FakeAgent()
        link, sio = self.dispatch("STOP", agent=agent)
        self.assertEqual(agent.calls, [])
        self.assertEqual(link.stats["commands_rejected"], 1)

    def test_command_without_an_id_is_not_acked(self):
        link, sio = self.dispatch({"type": "STOP"})
        self.assertEqual(sio.events_named("COMMAND_ACK"), [])
        self.assertEqual(link.stats["commands_rejected"], 1)

    def test_command_for_another_robot_is_refused(self):
        agent = FakeAgent()
        link, sio = self.dispatch({"commandId": "c", "type": "STOP", "robotId": "other"}, agent=agent)
        self.assertEqual(agent.calls, [])
        # Never acknowledged, not even as FAILED: the id is another robot's.
        self.assertEqual(sio.events_named("COMMAND_ACK"), [])
        self.assertEqual(link.stats["commands_rejected"], 1)

    def test_stale_command_is_refused(self):
        agent = FakeAgent()
        link, sio = self.dispatch(
            {"commandId": "c", "type": "STOP", "timestamp": int((time.time() - 9999) * 1000)},
            agent=agent,
            command_max_age_s=60.0,
        )
        self.assertEqual(agent.calls, [])
        self.assertEqual(sio.events_named("COMMAND_ACK"), [])

    def test_a_refused_command_is_not_acknowledged(self):
        # RESUME from STOPPED is refused. There is no FAILED ack on this wire,
        # so it is left unacknowledged and the backend marks it FAILED.
        agent = FakeAgent(mode=OperatingMode.STOPPED)
        link, sio = self.dispatch({"commandId": "c", "type": "RESUME"}, agent=agent)
        self.assertEqual(sio.events_named("COMMAND_ACK"), [])
        self.assertEqual(link.stats["commands_refused"], 1)

    def test_duplicate_command_executes_once_but_acks_twice(self):
        async def scenario():
            agent = FakeAgent()
            link, sio = make_link(agent=agent)
            await link._connect_once()
            await sio.fire("COMMAND", {"commandId": "dup", "type": "STOP"})
            await sio.fire("COMMAND", {"commandId": "dup", "type": "STOP"})
            return agent, link, sio

        agent, link, sio = self.run_async(scenario())
        self.assertEqual(len(agent.calls), 1)
        self.assertEqual(len(sio.events_named("COMMAND_ACK")), 2)
        self.assertEqual(link.stats["commands_duplicate"], 1)

    def test_a_failing_command_does_not_stop_the_next_one(self):
        async def scenario():
            agent = FakeAgent()
            link, sio = make_link(agent=agent)
            await link._connect_once()
            await sio.fire("COMMAND", {"type": "GARBAGE"})
            await sio.fire("COMMAND", {"commandId": "c2", "type": "STOP"})
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


class TestTaskAssignRecovery(AsyncTestCase):
    """TASK_ASSIGN is the backend's post-restart recovery re-send (handoff §6).

    It must never start a mission: OFFER is the only way one is assigned.
    Payloads are synthetic (`tests.fixtures.task_assign`).
    """

    def dispatch(self, payload, agent=None, **cfg):
        async def scenario():
            link, sio = make_link(agent=agent or FakeAgent(), **cfg)
            await connect_and_auth(link)
            before = len(sio.emitted)
            await sio.fire("TASK_ASSIGN", payload)
            return link, sio, sio.emitted[before:]

        return self.run_async(scenario())

    def test_a_valid_task_assign_never_starts_a_mission(self):
        agent = FakeAgent(mode=OperatingMode.IDLE)
        link, sio, sent = self.dispatch(task_assign_payload(task_id="task-1"), agent=agent)
        self.assertEqual(agent.assigned, [])
        self.assertEqual(agent.calls, [])
        self.assertEqual(sent, [], "no reply exists for TASK_ASSIGN")
        self.assertEqual(link.stats["tasks_recovery_resends"], 1)

    def test_a_malformed_task_assign_is_counted_and_harmless(self):
        for payload in ("TASK_ASSIGN", {}, task_assign_payload(taskId="")):
            agent = FakeAgent()
            link, sio, sent = self.dispatch(payload, agent=agent)
            self.assertEqual(agent.assigned, [], payload)
            self.assertEqual(link.stats["tasks_rejected"], 1, payload)
            self.assertTrue(sio.connected)

    def test_an_unbound_task_channel_registers_no_handler(self):
        binding = ProtocolBinding(source=BindingSource.EXPLICIT, task_assign="")
        link, sio = make_link(binding=binding)
        self.run_async(link._connect_once())
        self.assertNotIn("TASK_ASSIGN", sio.handlers)


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
    def test_an_unauthenticated_socket_is_not_reported_as_authenticated(self):
        async def scenario():
            link, _ = make_link(sio=FakeSio(auth_mode="none"))
            await link._connect_once()
            return link.describe()

        described = self.run_async(scenario())
        self.assertIs(described["authenticated"], False)
        self.assertIs(described["streaming"], False)
        self.assertEqual(described["status"], "AUTHENTICATING")

    def test_describe_names_the_unconfirmed_binding_entries(self):
        link, _ = make_link()
        unconfirmed = link.describe()["protocol"]["unconfirmed"]
        # The contract attests every name but this one, which it describes
        # only as a silent disconnect.
        self.assertEqual(unconfirmed, ["auth_failed"])

    def test_an_operator_supplied_binding_reports_nothing_unconfirmed(self):
        link, _ = make_link(binding=ProtocolBinding(source=BindingSource.FILE))
        self.assertEqual(link.describe()["protocol"]["unconfirmed"], [])

    def test_describe_does_not_leak_the_token(self):
        link, _ = make_link(
            robot_token="s3cr3t", tokens=MemoryTokenStore(token="stored-s3cr3t")
        )
        described = json.dumps(link.describe(), default=str)
        self.assertNotIn("s3cr3t", described)
        self.assertEqual(link.describe()["credential"]["token"], "SET")

    def test_describe_does_not_leak_the_pairing_code(self):
        link, _ = make_link(pairing_code="987654")
        self.assertNotIn("987654", json.dumps(link.describe(), default=str))
        self.assertEqual(link.describe()["pairing_code"], "SET")

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
