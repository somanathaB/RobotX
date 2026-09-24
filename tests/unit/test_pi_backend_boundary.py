"""P2B-1 acceptance: the Pi side of the physical RobotX <-> backend boundary.

One test class per required item, in the order the P2B-1 brief lists them, so
each acceptance line can be checked against exactly one place.

Test doubles, labelled
----------------------
- `FakeSio` (from `test_backend_link`) is a **fake backend**: an in-process
  stand-in for `socketio.AsyncClient` that records emits and answers AUTH. It
  proves what the Pi puts on the wire, not that any real backend accepts it.
- `FakeAgent` is a **fake RobotAgent** recording mission calls. No motor,
  sensor or ESP32 is involved anywhere in this file, and nothing here is
  evidence about real hardware.
- `state_with_fix()` is a **synthetic GPS fix**. This Rover has no working GPS;
  the fix exists only so the telemetry path has something to publish.
- Addresses use 192.0.2.0/24 (RFC 5737 TEST-NET-1), which is reserved for
  documentation and is nobody's laptop.
"""

from __future__ import annotations

import ipaddress
import re
import time
import unittest
from pathlib import Path

from robotx.communication.backend_link import BackendConfig, validate_backend_url
from robotx.communication.protocol import (
    agent_capabilities,
    build_telemetry_payload,
    now_ms,
)
from robotx.config.settings import Settings
from robotx.control.motion import MotionIntent
from robotx.control.safety import SafetyGate
from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus
from robotx.localization.position import Position, PositionSource
from robotx.mission.mission import MissionStatus
from robotx.perception.types import PerceptionResult, PerceptionStatus
from robotx.state.robot_state import BackendLinkStatus, OperatingMode, RobotState
from tests.fixtures.task_assign import task_assign_payload
from tests.unit.test_backend_link import (
    AsyncTestCase,
    FakeAgent,
    MemoryTokenStore,
    connect_and_auth,
    make_link,
    state_with_fix,
)


REPO = Path(__file__).resolve().parents[2]
TEST_URL = "http://192.0.2.10:4000"  # RFC 5737 documentation address

# Every key that would claim an emergency-stop or safety state on the wire.
_SAFETY_KEY = re.compile(r"e_?stop|emergency|safety", re.IGNORECASE)


def all_keys(payload):
    """Every key in a payload, recursively."""

    if isinstance(payload, dict):
        for key, value in payload.items():
            yield str(key)
            yield from all_keys(value)
    elif isinstance(payload, (list, tuple)):
        for item in payload:
            yield from all_keys(item)


def state_with_position(position, *, fix=True):
    """A state holding `position`, with or without a (synthetic) GPS fix."""

    state = RobotState("robotx-pi")
    reading = (
        GpsReading(
            status=GPSStatus.FIX,
            fix=GpsFix(latitude=position.latitude, longitude=position.longitude,
                       timestamp=position.timestamp),
            age_s=max(0.0, time.time() - position.timestamp),
        )
        if fix
        else GpsReading(status=GPSStatus.NO_FIX)
    )
    state.update_gps(reading, position)
    return state


def state_without_fix(robot_id="robotx-pi"):
    state = RobotState(robot_id)
    state.update_gps(GpsReading(status=GPSStatus.NO_FIX), None)
    return state


# 1 ---------------------------------------------------------------------------


class Test01StableIdentity(AsyncTestCase):
    def test_identity_is_the_configured_id_and_survives_a_restart(self):
        env = {"ROBOTX_ROBOT_ID": "robotx-physical-01"}
        # Two independent reads of the same configuration: a reboot.
        first, second = Settings.from_env(env), Settings.from_env(env)
        self.assertEqual(first.robot_id, "robotx-physical-01")
        self.assertEqual(first.robot_id, second.robot_id)

    def test_default_identity_is_a_constant_not_generated_per_boot(self):
        self.assertEqual(Settings.from_env({}).robot_id, Settings.from_env({}).robot_id)

    def test_every_auth_carries_the_same_id_across_a_reconnect(self):
        async def scenario():
            link, sio = make_link(heartbeat_interval_s=100.0)
            await connect_and_auth(link)
            await link._publish_heartbeat()
            await sio.drop()
            await connect_and_auth(link)
            await link._publish_heartbeat()
            await link._publish_telemetry()
            return sio

        sio = self.run_async(scenario())
        # Not a socket id, not an address: the one configured identity, in
        # AUTH -- and nowhere else, since identity comes from the socket.
        self.assertEqual({p["robotId"] for p in sio.events_named("AUTH")}, {"robotx-pi"})
        for event, payload in sio.emitted:
            if event != "AUTH":
                self.assertNotIn("robotId", payload, event)

    def test_there_is_no_default_identity(self):
        self.assertEqual(Settings.from_env({}).robot_id, "")
        self.assertEqual(BackendConfig().robot_id, "")

    def test_enabled_link_without_an_identity_is_a_configuration_error(self):
        settings = Settings.from_env({"ROBOTX_SOCKET_ENABLED": "1",
                                      "ROBOTX_SOCKET_SERVER_URL": TEST_URL})
        with self.assertRaises(ValueError) as ctx:
            BackendConfig.from_settings(settings)
        self.assertIn("ROBOTX_ROBOT_ID", str(ctx.exception))

    def test_identity_is_used_exactly_as_configured(self):
        # Case-sensitive and untouched: it must equal Robot.robotId.
        settings = Settings.from_env({"ROBOTX_SOCKET_ENABLED": "1", "ROBOTX_ROBOT_ID": "RobotX-Pi",
                                      "ROBOTX_SOCKET_SERVER_URL": TEST_URL})
        self.assertEqual(BackendConfig.from_settings(settings).robot_id, "RobotX-Pi")


# 2 ---------------------------------------------------------------------------


class Test02ConfigurableBackendUrl(AsyncTestCase):
    def test_url_comes_from_configuration_and_is_what_the_link_dials(self):
        settings = Settings.from_env(
            {"ROBOTX_SOCKET_ENABLED": "1", "ROBOTX_SOCKET_SERVER_URL": TEST_URL,
             "ROBOTX_ROBOT_ID": "robotx-pi"}
        )
        cfg = BackendConfig.from_settings(settings)
        self.assertEqual(cfg.server_url, TEST_URL)

        async def scenario():
            link, sio = make_link(server_url=TEST_URL)
            await link._connect_once()
            return sio

        self.assertEqual(self.run_async(scenario()).connect_calls[0]["url"], TEST_URL)

    def test_a_deployed_https_backend_needs_no_code_change(self):
        self.assertEqual(validate_backend_url("https://backend.example.org"),
                         "https://backend.example.org")
        self.assertTrue(BackendConfig(server_url="https://backend.example.org").uses_tls)

    def test_enabled_link_with_no_url_is_a_configuration_error(self):
        settings = Settings.from_env({"ROBOTX_SOCKET_ENABLED": "1", "ROBOTX_ROBOT_ID": "robotx-pi"})
        with self.assertRaises(ValueError):
            BackendConfig.from_settings(settings)

    def test_link_with_no_url_refuses_to_start_and_never_dials(self):
        async def scenario():
            link, sio = make_link(server_url="")
            await link.start()
            return link, sio

        link, sio = self.run_async(scenario())
        self.assertIs(link.status, BackendLinkStatus.DISABLED)
        self.assertIn("ROBOTX_SOCKET_SERVER_URL", link.describe()["detail"])
        self.assertEqual(sio.connect_calls, [])

    def test_malformed_urls_are_refused(self):
        for bad in ("", "   ", "backend:3000", "ftp://192.0.2.10", "http://"):
            with self.assertRaises(ValueError, msg=bad):
                validate_backend_url(bad)


# 3 ---------------------------------------------------------------------------


class Test03AuthPayload(AsyncTestCase):
    def test_auth_is_an_event_after_connect_not_a_handshake_credential(self):
        async def scenario():
            link, sio = make_link(pairing_code="123456")
            await connect_and_auth(link)
            return sio

        sio = self.run_async(scenario())
        self.assertIsNone(sio.connect_calls[0]["auth"])
        self.assertEqual(sio.emitted[0], ("AUTH", {"robotId": "robotx-pi", "pairingCode": "123456"}))

    def test_a_stored_token_is_preferred_and_sent_with_only_the_robot_id(self):
        async def scenario():
            link, sio = make_link(tokens=MemoryTokenStore(token="stored-tok"))
            await connect_and_auth(link)
            return sio

        auth = self.run_async(scenario()).events_named("AUTH")[0]
        self.assertEqual(auth, {"robotId": "robotx-pi", "token": "stored-tok"})

    def test_credentials_never_appear_outside_auth(self):
        async def scenario():
            link, sio = make_link(robot_token="s3cr3t-token", pairing_code="654321")
            await connect_and_auth(link)
            await link._publish_heartbeat()
            await link._publish_telemetry()
            await sio.fire("COMMAND", {"commandId": "c1", "type": "PAUSE"})
            return sio

        for event, payload in self.run_async(scenario()).emitted:
            if event == "AUTH":
                continue
            text = repr(payload)
            self.assertNotIn("s3cr3t-token", text, event)
            self.assertNotIn("654321", text, event)
            self.assertNotIn("token", {k.lower() for k in all_keys(payload)}, event)


# 4 ---------------------------------------------------------------------------


class Test04Heartbeat(AsyncTestCase):
    def test_default_interval_is_two_seconds_inside_the_freshness_budget(self):
        self.assertEqual(Settings().backend_heartbeat_interval_s, 2.0)
        self.assertEqual(BackendConfig().heartbeat_interval_s, 2.0)

    def test_publish_loop_beats_on_its_cadence(self):
        async def scenario():
            link, sio = make_link(heartbeat_interval_s=0.05, telemetry_interval_s=100.0)
            await connect_and_auth(link)
            link._running = True
            import asyncio

            task = asyncio.create_task(link._publish_loop())
            await asyncio.sleep(0.6)
            link._running = False
            await task
            return sio

        self.assertGreaterEqual(len(self.run_async(scenario()).events_named("HEARTBEAT")), 2)

    def test_heartbeat_stops_when_the_agent_loop_is_not_alive(self):
        """An open socket is not a live robot."""

        alive = {"value": False}

        async def scenario():
            link, sio = make_link()
            link._agent_alive = lambda: alive["value"]
            await connect_and_auth(link)
            await link._publish_heartbeat()
            await link._publish_heartbeat()
            alive["value"] = True
            await link._publish_heartbeat()
            return link, sio

        link, sio = self.run_async(scenario())
        self.assertEqual(len(sio.events_named("HEARTBEAT")), 1)
        self.assertEqual(link.stats["heartbeats_suppressed"], 2)

    def test_agent_is_alive_only_while_its_loop_is_ticking(self):
        from robotx.application.agent import AGENT_STALL_S, RobotAgent

        agent = RobotAgent(Settings.from_env({
            "ROBOTX_CAMERA_ENABLED": "0", "ROBOTX_PERCEPTION_ENABLED": "0",
            "ROBOTX_GPS_ENABLED": "0", "ROBOTX_LOG_LEVEL": "CRITICAL",
        }))
        self.assertFalse(agent.is_alive(), "alive before any tick")
        agent._last_tick_at = time.monotonic()
        self.assertTrue(agent.is_alive())
        self.assertFalse(agent.is_alive(now=time.monotonic() + AGENT_STALL_S + 0.1))

    def test_the_agent_wires_its_liveness_into_the_link(self):
        import asyncio
        from unittest import mock

        from robotx.application.agent import RobotAgent

        captured = {}

        class RecordingLink:
            def __init__(self, cfg, state, target, **kwargs):
                captured.update(kwargs, target=target)

            async def start(self):
                pass

        agent = RobotAgent(Settings.from_env({
            "ROBOTX_CAMERA_ENABLED": "0", "ROBOTX_PERCEPTION_ENABLED": "0",
            "ROBOTX_GPS_ENABLED": "0", "ROBOTX_LOG_LEVEL": "CRITICAL",
            "ROBOTX_SOCKET_ENABLED": "1", "ROBOTX_SOCKET_SERVER_URL": TEST_URL,
            "ROBOTX_ROBOT_ID": "robotx-pi",
        }))
        with mock.patch("robotx.communication.backend_link.BackendLink", RecordingLink):
            asyncio.run(agent._start_backend_link())
        self.assertEqual(captured["agent_alive"], agent.is_alive)


# 5 ---------------------------------------------------------------------------


class Test05Reconnect(AsyncTestCase):
    def test_reconnect_reauthenticates_and_restores_heartbeat(self):
        async def scenario():
            tokens = MemoryTokenStore()
            link, sio = make_link(tokens=tokens, pairing_code="123456")
            await connect_and_auth(link)

            await sio.drop()
            down = (link.connected, link.status)
            # Nothing may be sent while the link is down, and nothing is queued
            # to be replayed later as if it were current.
            before = len(sio.emitted)
            await link._publish_heartbeat()
            await link._publish_telemetry()
            sent_while_down = len(sio.emitted) - before

            await connect_and_auth(link)
            await link._publish_heartbeat()
            return link, sio, down, sent_while_down

        link, sio, down, sent_while_down = self.run_async(scenario())
        self.assertEqual(down, (False, BackendLinkStatus.DISCONNECTED))
        self.assertEqual(sent_while_down, 0)

        auths = sio.events_named("AUTH")
        self.assertEqual(len(auths), 2)
        # The second AUTH uses the session token from the first AUTH_SUCCESS,
        # not the single-use pairing code.
        self.assertEqual(auths[1], {"robotId": "robotx-pi", "token": "sess-tok"})
        self.assertEqual(len(sio.events_named("HEARTBEAT")), 1)
        self.assertTrue(link.connected)
        # Listeners registered once, so a replayed command cannot run twice.
        self.assertEqual(link.handler_registrations, 1)

    def test_state_is_not_streaming_while_disconnected(self):
        async def scenario():
            link, sio = make_link()
            await connect_and_auth(link)
            await sio.drop()
            return link

        link = self.run_async(scenario())
        self.assertFalse(link.describe()["streaming"])
        self.assertFalse(link.describe()["authenticated"])


# 6 ---------------------------------------------------------------------------


class Test06TelemetryTimestampPreservation(unittest.TestCase):
    def test_timestamp_is_the_measurement_instant_not_the_send_instant(self):
        measured_at = time.time() - 2.0
        state = state_with_position(
            Position(latitude=1.0, longitude=2.0, timestamp=measured_at)
        )
        frame = build_telemetry_payload(
            state.snapshot(), sequence=1, max_position_age_s=5.0, now=measured_at + 2.0,
        )
        self.assertEqual(frame.payload["timestamp"], now_ms(measured_at))
        self.assertNotEqual(frame.payload["timestamp"], now_ms(measured_at + 2.0))

    def test_an_old_observation_is_not_restamped_and_resent(self):
        measured_at = time.time() - 30.0
        state = state_with_position(
            Position(latitude=1.0, longitude=2.0, timestamp=measured_at)
        )
        frame = build_telemetry_payload(state.snapshot(), sequence=1, max_position_age_s=5.0)
        self.assertFalse(frame.has_position)
        self.assertNotIn("lat", frame.payload)


# 7 ---------------------------------------------------------------------------


class Test07MissingGpsStaysMissing(AsyncTestCase):
    def test_no_fix_means_no_position_on_the_wire(self):
        async def scenario():
            link, sio = make_link(state=state_without_fix())
            await connect_and_auth(link)
            for _ in range(3):
                await link._publish_telemetry()
                await link._publish_heartbeat()
            return link, sio

        link, sio = self.run_async(scenario())
        self.assertEqual(len(sio.events_named("TELEMETRY")), 3)
        self.assertEqual(link.stats["positions_omitted"], 3)
        for _, payload in sio.emitted:
            self.assertFalse({"lat", "lon"} & set(all_keys(payload)))

    def test_a_dead_reckoned_position_is_never_published(self):
        state = state_with_position(
            Position(latitude=1.0, longitude=2.0, timestamp=time.time(),
                     source=PositionSource.DEAD_RECKONING),
            fix=False,
        )
        frame = build_telemetry_payload(state.snapshot(), sequence=1, max_position_age_s=5.0)
        self.assertFalse(frame.has_position)

    def test_a_dead_reckoned_position_is_refused_even_alongside_a_fix(self):
        state = state_with_position(
            Position(latitude=1.0, longitude=2.0, timestamp=time.time(),
                     source=PositionSource.DEAD_RECKONING),
        )
        frame = build_telemetry_payload(state.snapshot(), sequence=1, max_position_age_s=5.0)
        self.assertFalse(frame.has_position)
        self.assertIn("DEAD_RECKONING", frame.position_omitted)


# 8 ---------------------------------------------------------------------------


class Test08MissingBatteryStaysMissing(unittest.TestCase):
    def test_battery_is_omitted_never_null_or_a_number(self):
        for state in (state_with_fix(), state_without_fix()):
            frame = build_telemetry_payload(state.snapshot(), sequence=1, max_position_age_s=5.0)
            self.assertNotIn("battery", frame.payload)
            self.assertFalse({"energy", "soc"} & set(all_keys(frame.payload)))

    def test_the_pi_declares_it_has_no_battery_sensing(self):
        self.assertIs(agent_capabilities()["battery"], False)
        self.assertFalse(RobotState("r").snapshot().power.is_measured)


# 9 ---------------------------------------------------------------------------


class Test09SafetyStopIsNotAnEstop(AsyncTestCase):
    def test_no_estop_or_safety_state_reaches_the_wire(self):
        """Even with the Pi's local software e-stop latched, nothing on the
        backend wire claims an e-stop: that latch is not a physical e-stop
        circuit, and no ESP32 safety_stop reaches the Pi at all."""

        state = state_with_fix()
        gate = SafetyGate()
        gate.engage_estop("test: local software latch")
        state.update_safety(gate.evaluate(
            MotionIntent.hold("test"),
            mission_active=False,
            perception=PerceptionResult.unavailable(PerceptionStatus.DISABLED),
        ))
        self.assertTrue(state.snapshot().safety.blocked)

        async def scenario():
            link, sio = make_link(state=state)
            await connect_and_auth(link)
            await link._publish_heartbeat()
            await link._publish_telemetry()
            await sio.fire("COMMAND", {"commandId": "c1", "type": "STOP"})
            return sio

        sio = self.run_async(scenario())
        self.assertTrue(sio.emitted)
        for event, payload in sio.emitted:
            flagged = [k for k in all_keys(payload) if _SAFETY_KEY.search(k)]
            self.assertEqual(flagged, [], event)


# 10 --------------------------------------------------------------------------


class Test10CommandForAnotherRobot(AsyncTestCase):
    def fire(self, event, payload):
        agent = FakeAgent(mode=OperatingMode.AUTO)

        async def scenario():
            link, sio = make_link(agent=agent)
            await connect_and_auth(link)
            await sio.fire(event, payload)
            return link, sio

        link, sio = self.run_async(scenario())
        return link, sio, agent

    def test_command_is_neither_executed_nor_acknowledged(self):
        link, sio, agent = self.fire(
            "COMMAND", {"commandId": "theirs", "type": "STOP", "robotId": "robotx-sim-7"}
        )
        self.assertEqual(agent.calls, [])
        self.assertEqual(sio.events_named("COMMAND_ACK"), [])
        self.assertEqual(link.stats["commands_rejected"], 1)

    def test_bare_stop_for_another_robot_is_ignored(self):
        link, sio, agent = self.fire("STOP", {"taskId": "t-9", "robotId": "robotx-sim-7"})
        self.assertEqual(agent.calls, [])
        self.assertEqual(sio.events_named("COMMAND_ACK"), [])
        self.assertEqual(link.stats["stop_events_ignored"], 1)

    def test_bare_stop_with_no_robot_id_is_still_obeyed(self):
        link, sio, agent = self.fire("STOP", {"taskId": "t-9"})
        self.assertEqual([c[0] for c in agent.calls], ["stop"])

    def test_task_for_another_robot_is_not_taken(self):
        link, sio, agent = self.fire("TASK_ASSIGN", task_assign_payload(robotId="robotx-sim-7"))
        self.assertEqual(agent.assigned, [])
        self.assertEqual(link.stats["tasks_rejected"], 1)


# 11 --------------------------------------------------------------------------


class Test11DuplicateCommand(AsyncTestCase):
    def test_redelivery_executes_once_and_reacks_the_original_outcome(self):
        agent = FakeAgent(mode=OperatingMode.AUTO)

        async def scenario():
            link, sio = make_link(agent=agent)
            await connect_and_auth(link)
            for _ in range(3):
                await sio.fire("COMMAND", {"commandId": "c-dup", "type": "PAUSE"})
            # ... and once more after a reconnect.
            await sio.drop()
            await connect_and_auth(link)
            await sio.fire("COMMAND", {"commandId": "c-dup", "type": "PAUSE"})
            return link, sio

        link, sio = self.run_async(scenario())
        self.assertEqual([c[0] for c in agent.calls], ["pause"])
        # Re-ACKed on every redelivery (the backend redispatches until acked),
        # never re-applied.
        self.assertEqual(sio.events_named("COMMAND_ACK"), [{"commandId": "c-dup"}] * 4)
        self.assertEqual(link.stats["commands_duplicate"], 3)


# 12 --------------------------------------------------------------------------


class Test12DuplicateCompletion(AsyncTestCase):
    # The link-level completion tests (once only, across a reconnect, never
    # on an inferred position, L1 evidence) live in tests/unit/test_engine_offer.py.

    def test_the_mission_records_which_arrivals_were_measured(self):
        """The agent-side half: a dead-reckoned leg is recorded as unmeasured."""

        from tests.fixtures.task_assign import PICKUP
        from tests.unit.test_mission import MissionHarness, pose

        harness = MissionHarness()
        harness.assign()

        def stand_at(point, measured):
            navigation = harness.navigator.update(pose(point))
            return harness.manager.update(navigation, position_measured=measured)

        for point in harness.mission.path_to_pickup:
            stand_at(point, measured=False)  # pickup leg on dead reckoning
        stand_at(PICKUP, measured=True)  # the tick that loads the drop leg
        for point in harness.mission.path_to_drop:
            update = stand_at(point, measured=True)

        self.assertTrue(update.completed)
        self.assertFalse(update.active.pickup_measured)
        self.assertTrue(update.active.drop_measured)
        self.assertFalse(update.active.arrivals_measured)


# 13 --------------------------------------------------------------------------


class Test13NoFabricatedTelemetry(AsyncTestCase):
    def test_telemetry_carries_exactly_the_measured_fields(self):
        frame = build_telemetry_payload(state_with_fix().snapshot(), sequence=1,
                                        max_position_age_s=5.0)
        self.assertEqual(
            set(frame.payload), {"timestamp", "sequence", "status", "lat", "lon", "speed"}
        )

    def test_nothing_unmeasured_is_ever_present(self):
        for state in (state_with_fix(), state_without_fix()):
            keys = set(all_keys(build_telemetry_payload(
                state.snapshot(), sequence=1, max_position_age_s=5.0).payload))
            for absent in ("battery", "energy", "safety", "faults", "localisation", "heading",
                           "distanceTravelled", "robotId"):
                self.assertNotIn(absent, keys)
            self.assertFalse([k for k in keys if "capabilit" in k.lower()])
            # No placeholders: every value is a real number or a status string.
            payload = build_telemetry_payload(state.snapshot(), sequence=1,
                                              max_position_age_s=5.0).payload
            self.assertNotIn(None, payload.values())
            self.assertNotIn("", payload.values())

    def test_unreported_speed_is_omitted_not_zero(self):
        state = state_with_position(
            Position(latitude=1.0, longitude=2.0, timestamp=time.time(), speed_mps=None)
        )
        frame = build_telemetry_payload(state.snapshot(), sequence=1, max_position_age_s=5.0)
        self.assertNotIn("speed", frame.payload)

    def test_idle_heartbeat_asserts_nothing(self):
        async def scenario():
            link, sio = make_link()
            await connect_and_auth(link)
            await link._publish_heartbeat()
            return sio

        self.assertEqual(self.run_async(scenario()).events_named("HEARTBEAT"), [{}])

    # OFFER handling is covered in tests/unit/test_engine_offer.py.


# 14 --------------------------------------------------------------------------


class Test14NoHardcodedBackendAddress(unittest.TestCase):
    FILES = (
        "robotx/config/settings.py",
        "robotx/communication/backend_link.py",
        "robotx/communication/protocol.py",
        "robotx/application/agent.py",
        ".env.example",
    )

    def test_defaults_name_no_backend_host(self):
        self.assertEqual(Settings().socket_server_url, "")
        self.assertEqual(Settings.from_env({}).socket_server_url, "")
        self.assertEqual(BackendConfig().server_url, "")

    def test_no_private_or_loopback_address_in_backend_configuration(self):
        address = re.compile(r"\b(?:\d{1,3}\.){3}\d{1,3}\b")
        for name in self.FILES:
            text = (REPO / name).read_text()
            self.assertNotRegex(text, r"(?i)localhost", name)
            for match in address.findall(text):
                try:
                    ip = ipaddress.ip_address(match)
                except ValueError:
                    continue
                if ip.is_unspecified:
                    continue  # 0.0.0.0: the local API's bind address, not a host
                self.assertFalse(
                    ip.is_private or ip.is_loopback,
                    f"{name} contains a hardcoded address {match}",
                )


if __name__ == "__main__":
    unittest.main()
