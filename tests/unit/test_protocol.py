"""The wire contract: payload shape, refusal to fabricate, inbound validation.

These tests exist mostly to pin down what the Pi must *not* put on the wire.
The interesting assertions are the negative ones: no battery number, no stale
position, no self-asserted `isOnline`, no token in anything loggable.
"""

import json
import os
import tempfile
import time
import unittest

from robotx.communication.protocol import (
    MAX_INBOUND_PAYLOAD_BYTES,
    WIRE_SCHEMA_VERSION,
    BindingSource,
    CommandRejection,
    CommandStatus,
    CommandType,
    EventLevel,
    InboundCommand,
    ProtocolBinding,
    RejectionReason,
    agent_capabilities,
    build_command_result_payload,
    build_event_payload,
    build_register_payload,
    build_status_payload,
    build_telemetry_payload,
    mode_for_command,
    parse_command,
    parse_timestamp,
    redact,
)
from robotx.control.motion import MotionIntent
from robotx.diagnostics.health import ComponentHealth, HealthReport, HealthStatus
from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus
from robotx.localization.position import HeadingSource, Position
from robotx.navigation.navigator import NavigationState, NavigationStatus
from robotx.perception.types import PerceptionResult, PerceptionStatus
from robotx.state.robot_state import (
    CommunicationState,
    OperatingMode,
    RobotSnapshot,
)


def make_snapshot(
    *,
    gps_status=GPSStatus.FIX,
    position_age_s=0.0,
    speed_mps=1.25,
    mode=OperatingMode.AUTO,
    now=None,
):
    """A snapshot with a controllable GPS status and position age."""

    now = time.time() if now is None else now
    stamp = now - position_age_s
    position = Position(
        latitude=12.9716,
        longitude=77.5946,
        timestamp=stamp,
        speed_mps=speed_mps,
        heading_deg=91.0,
        heading_source=HeadingSource.NMEA_TRACK,
        satellites=9,
    )
    reading = GpsReading(
        status=gps_status,
        fix=GpsFix(latitude=12.9716, longitude=77.5946, timestamp=stamp, satellites=9),
        age_s=position_age_s,
    )
    health = HealthReport(
        status=HealthStatus.HEALTHY,
        components={
            "camera": ComponentHealth("camera", HealthStatus.HEALTHY),
            "gps": ComponentHealth("gps", HealthStatus.HEALTHY),
        },
    )
    return RobotSnapshot(
        robot_id="robotx-pi",
        mode=mode,
        started_at=now - 60,
        updated_at=now,
        uptime_s=60.0,
        gps=reading,
        position=position,
        navigation=NavigationState(status=NavigationStatus.NAVIGATING, waypoints_total=3),
        perception=PerceptionResult.unavailable(PerceptionStatus.OK),
        motion_intent=MotionIntent.forward(0.4, reason="clear"),
        communication=CommunicationState(),
        health=health,
    )


class TestProtocolBinding(unittest.TestCase):
    def test_builtin_binding_is_marked_provisional(self):
        binding = ProtocolBinding()
        self.assertIs(binding.source, BindingSource.PROVISIONAL)
        self.assertTrue(binding.is_provisional)

    def test_binding_loaded_from_file_is_not_provisional(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "binding.json")
            with open(path, "w") as handle:
                json.dump(
                    {
                        "namespace": "/robots",
                        "telemetry": "robot:telemetry",
                        "command": "robot:command",
                    },
                    handle,
                )
            binding = ProtocolBinding.load(path)

        self.assertFalse(binding.is_provisional)
        self.assertIs(binding.source, BindingSource.FILE)
        self.assertEqual(binding.namespace, "/robots")
        self.assertEqual(binding.telemetry, "robot:telemetry")
        self.assertEqual(binding.command, "robot:command")
        # Unspecified names keep their defaults rather than becoming empty.
        self.assertEqual(binding.status, "status")

    def test_nested_binding_file_is_accepted(self):
        binding = ProtocolBinding.from_mapping(
            {"namespace": "/ns", "outbound": {"telemetry": "t"}, "inbound": {"command": "c"}},
            source=BindingSource.FILE,
        )
        self.assertEqual((binding.namespace, binding.telemetry, binding.command), ("/ns", "t", "c"))

    def test_missing_binding_file_raises_rather_than_falling_back(self):
        # Falling back silently would let an operator believe the real
        # contract was loaded while the Pi used guessed names.
        with self.assertRaises(OSError):
            ProtocolBinding.load("/nonexistent/binding.json")

    def test_malformed_binding_file_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "b.json")
            with open(path, "w") as handle:
                handle.write("[1, 2, 3]")
            with self.assertRaises(ValueError):
                ProtocolBinding.load(path)

    def test_no_path_returns_provisional_binding(self):
        self.assertTrue(ProtocolBinding.load(None).is_provisional)
        self.assertTrue(ProtocolBinding.load("").is_provisional)


class TestTelemetryPayload(unittest.TestCase):
    def test_payload_carries_exactly_the_telemetry_model_fields(self):
        frame = build_telemetry_payload(
            make_snapshot(), robot_id="robotx-pi", max_position_age_s=5.0
        )
        self.assertTrue(frame.sendable)
        payload = frame.payload
        self.assertEqual(payload["robotId"], "robotx-pi")
        self.assertAlmostEqual(payload["lat"], 12.9716)
        self.assertAlmostEqual(payload["lon"], 77.5946)
        self.assertAlmostEqual(payload["speed"], 1.25)
        self.assertEqual(payload["schemaVersion"], WIRE_SCHEMA_VERSION)

    def test_battery_is_null_never_a_number(self):
        # There is no battery hardware. A plausible number here would be
        # indistinguishable from a measurement.
        frame = build_telemetry_payload(
            make_snapshot(), robot_id="robotx-pi", max_position_age_s=5.0
        )
        self.assertIn("battery", frame.payload)
        self.assertIsNone(frame.payload["battery"])

    def test_backend_owned_fields_are_never_sent(self):
        frame = build_telemetry_payload(
            make_snapshot(), robot_id="robotx-pi", max_position_age_s=5.0
        )
        for field in ("isOnline", "socketId", "lastSeenAt", "id", "createdAt", "currentTaskId"):
            self.assertNotIn(field, frame.payload)

    def test_no_telemetry_without_a_fix(self):
        for status in (GPSStatus.NO_FIX, GPSStatus.STALE, GPSStatus.DISCONNECTED,
                       GPSStatus.UNAVAILABLE, GPSStatus.STARTING):
            frame = build_telemetry_payload(
                make_snapshot(gps_status=status), robot_id="robotx-pi", max_position_age_s=5.0
            )
            self.assertFalse(frame.sendable, status)
            self.assertIn(status.value, frame.skipped_reason)

    def test_stale_position_is_refused_not_resent(self):
        frame = build_telemetry_payload(
            make_snapshot(position_age_s=30.0), robot_id="robotx-pi", max_position_age_s=5.0
        )
        self.assertFalse(frame.sendable)
        self.assertIn("old", frame.skipped_reason)

    def test_fresh_position_within_limit_is_sent(self):
        frame = build_telemetry_payload(
            make_snapshot(position_age_s=4.0), robot_id="robotx-pi", max_position_age_s=5.0
        )
        self.assertTrue(frame.sendable)

    def test_unreported_speed_is_null_not_zero(self):
        # "Not measured" and "stationary" are different claims.
        frame = build_telemetry_payload(
            make_snapshot(speed_mps=None), robot_id="robotx-pi", max_position_age_s=5.0
        )
        self.assertIsNone(frame.payload["speed"])

    def test_payload_is_json_serializable(self):
        frame = build_telemetry_payload(
            make_snapshot(), robot_id="robotx-pi", max_position_age_s=5.0
        )
        json.dumps(frame.payload)


class TestStatusPayload(unittest.TestCase):
    def setUp(self):
        self.payload = build_status_payload(
            make_snapshot(), robot_id="robotx-pi", binding=ProtocolBinding()
        )

    def test_status_is_the_operating_mode(self):
        self.assertEqual(self.payload["status"], OperatingMode.AUTO.value)

    def test_status_never_asserts_online(self):
        self.assertNotIn("isOnline", self.payload)
        self.assertNotIn("socketId", self.payload)
        self.assertNotIn("lastSeenAt", self.payload)

    def test_status_position_is_labelled_with_its_age(self):
        block = self.payload["position"]
        self.assertIn("ageS", block)
        self.assertTrue(block["isFresh"])

    def test_stale_status_position_is_marked_not_fresh(self):
        payload = build_status_payload(
            make_snapshot(gps_status=GPSStatus.STALE, position_age_s=60.0),
            robot_id="robotx-pi",
            binding=ProtocolBinding(),
        )
        self.assertFalse(payload["position"]["isFresh"])
        self.assertGreater(payload["position"]["ageS"], 30.0)

    def test_status_flags_a_provisional_protocol(self):
        self.assertTrue(self.payload["protocol"]["provisional"])

    def test_status_battery_is_null(self):
        self.assertIsNone(self.payload["battery"])

    def test_status_is_json_serializable(self):
        json.dumps(self.payload)


class TestOtherPayloads(unittest.TestCase):
    def test_register_declares_a_physical_robot(self):
        payload = build_register_payload(
            robot_id="robotx-pi",
            binding=ProtocolBinding(),
            agent_version="2.1",
            capabilities=agent_capabilities(),
        )
        self.assertIs(payload["simulated"], False)
        self.assertEqual(payload["robotId"], "robotx-pi")

    def test_register_never_carries_the_token(self):
        payload = build_register_payload(
            robot_id="robotx-pi",
            binding=ProtocolBinding(),
            agent_version="2.1",
            capabilities=agent_capabilities(),
        )
        self.assertNotIn("token", json.dumps(payload).lower())

    def test_capabilities_admit_what_the_robot_cannot_do(self):
        caps = agent_capabilities()
        self.assertFalse(caps["battery"])
        self.assertFalse(caps["motion"])
        self.assertEqual(caps["commands"], ["STOP", "PAUSE", "RETURN", "RESUME"])

    def test_event_payload_uses_backend_levels(self):
        payload = build_event_payload(
            robot_id="r", level=EventLevel.CRITICAL, message="perception failed"
        )
        self.assertEqual(payload["type"], "CRITICAL")
        self.assertEqual(payload["message"], "perception failed")
        self.assertNotIn("taskId", payload)

    def test_event_includes_task_only_when_one_is_held(self):
        payload = build_event_payload(
            robot_id="r", level=EventLevel.INFO, message="m", task_id="task-7"
        )
        self.assertEqual(payload["taskId"], "task-7")

    def test_event_message_is_bounded(self):
        payload = build_event_payload(robot_id="r", level=EventLevel.INFO, message="x" * 5000)
        self.assertLessEqual(len(payload["message"]), 500)

    def test_command_result_reports_ack_with_execution_time(self):
        payload = build_command_result_payload(
            robot_id="r", command_id="c1", status=CommandStatus.ACK, executed_at=1000.0
        )
        self.assertEqual(payload["status"], "ACK")
        self.assertEqual(payload["commandId"], "c1")
        self.assertEqual(payload["executedAt"], 1000.0)

    def test_robot_may_not_report_sent(self):
        # SENT is the backend's own state for "issued". A robot claiming it
        # would be overwriting the server's record of what it did.
        with self.assertRaises(ValueError):
            build_command_result_payload(robot_id="r", command_id="c", status=CommandStatus.SENT)


class TestCommandParsing(unittest.TestCase):
    def parse(self, data, **kwargs):
        kwargs.setdefault("expected_robot_id", "robotx-pi")
        return parse_command(data, **kwargs)

    def test_valid_command_parses(self):
        result = self.parse({"commandId": "c1", "type": "STOP", "robotId": "robotx-pi"})
        self.assertIsInstance(result, InboundCommand)
        self.assertIs(result.type, CommandType.STOP)
        self.assertEqual(result.command_id, "c1")

    def test_all_four_backend_commands_are_accepted(self):
        for name in ("STOP", "PAUSE", "RETURN", "RESUME"):
            result = self.parse({"commandId": "c", "type": name})
            self.assertIsInstance(result, InboundCommand, name)

    def test_lowercase_type_is_accepted(self):
        self.assertIsInstance(self.parse({"commandId": "c", "type": "stop"}), InboundCommand)

    def test_unknown_command_is_rejected_not_guessed(self):
        result = self.parse({"commandId": "c", "type": "SELF_DESTRUCT"})
        self.assertIsInstance(result, CommandRejection)
        self.assertIs(result.reason, RejectionReason.UNKNOWN_TYPE)
        self.assertEqual(result.command_id, "c")

    def test_non_object_payload_is_rejected(self):
        for bad in ("STOP", 42, None, ["STOP"]):
            result = self.parse(bad)
            self.assertIsInstance(result, CommandRejection, bad)
            self.assertIs(result.reason, RejectionReason.MALFORMED)

    def test_missing_type_is_rejected(self):
        result = self.parse({"commandId": "c"})
        self.assertIs(result.reason, RejectionReason.MALFORMED)

    def test_missing_command_id_is_rejected_and_unackable(self):
        result = self.parse({"type": "STOP"})
        self.assertIs(result.reason, RejectionReason.MISSING_ID)
        self.assertFalse(result.is_ackable)

    def test_id_field_is_accepted_as_command_id(self):
        result = self.parse({"id": "c9", "type": "PAUSE"})
        self.assertEqual(result.command_id, "c9")

    def test_command_for_another_robot_is_rejected(self):
        result = self.parse({"commandId": "c", "type": "STOP", "robotId": "some-other-robot"})
        self.assertIs(result.reason, RejectionReason.WRONG_ROBOT)

    def test_command_without_robot_id_is_accepted(self):
        # The server routed it to this socket; absence is not a mismatch.
        self.assertIsInstance(self.parse({"commandId": "c", "type": "STOP"}), InboundCommand)

    def test_stale_command_is_rejected(self):
        now = time.time()
        result = self.parse(
            {"commandId": "c", "type": "STOP", "issuedAt": now - 600}, max_age_s=120.0, now=now
        )
        self.assertIs(result.reason, RejectionReason.STALE)

    def test_recent_command_passes_the_age_check(self):
        now = time.time()
        result = self.parse(
            {"commandId": "c", "type": "STOP", "issuedAt": now - 5}, max_age_s=120.0, now=now
        )
        self.assertIsInstance(result, InboundCommand)

    def test_unparseable_issued_at_does_not_become_now(self):
        # Treating a bad timestamp as "now" would make every stale command
        # look fresh.
        result = self.parse({"commandId": "c", "type": "STOP", "issuedAt": "not-a-date"},
                            max_age_s=1.0)
        self.assertIsInstance(result, InboundCommand)
        self.assertIsNone(result.issued_at)

    def test_oversized_payload_is_rejected(self):
        result = self.parse({"commandId": "c", "type": "STOP", "junk": "x" * (MAX_INBOUND_PAYLOAD_BYTES + 10)})
        self.assertIs(result.reason, RejectionReason.OVERSIZED)

    def test_non_json_field_does_not_raise(self):
        # The size check serializes with `default=str`, so an exotic value is
        # measured rather than exploding the handler.
        result = self.parse({"commandId": "c", "type": "STOP", "bad": object()})
        self.assertIsInstance(result, InboundCommand)

    def test_parsing_never_raises(self):
        for payload in ({}, {"type": {}}, {"commandId": [], "type": "STOP"}, {"type": 3.5}):
            try:
                self.parse(payload)
            except Exception as e:  # pragma: no cover - the assertion is the point
                self.fail(f"parse_command raised on {payload!r}: {e!r}")


class TestTimestampParsing(unittest.TestCase):
    def test_epoch_seconds(self):
        self.assertAlmostEqual(parse_timestamp(1700000000), 1700000000.0)

    def test_epoch_milliseconds(self):
        self.assertAlmostEqual(parse_timestamp(1700000000000), 1700000000.0)

    def test_iso8601_with_zulu(self):
        self.assertIsNotNone(parse_timestamp("2026-01-02T03:04:05Z"))

    def test_iso8601_with_offset(self):
        self.assertIsNotNone(parse_timestamp("2026-01-02T03:04:05+05:30"))

    def test_unparseable_values_are_none(self):
        for bad in (None, "", "yesterday", True, {}, []):
            self.assertIsNone(parse_timestamp(bad), bad)


class TestRedaction(unittest.TestCase):
    def test_token_is_redacted(self):
        out = redact({"robotId": "r", "token": "super-secret"})
        self.assertEqual(out["token"], "<redacted>")
        self.assertNotIn("super-secret", json.dumps(out))

    def test_nested_credentials_are_redacted(self):
        out = redact({"auth": {"token": "s3cr3t"}, "outer": {"password": "p"}})
        self.assertNotIn("s3cr3t", json.dumps(out))
        self.assertEqual(out["outer"]["password"], "<redacted>")

    def test_non_secret_values_survive(self):
        out = redact({"robotId": "robotx-pi", "lat": 1.0})
        self.assertEqual(out["robotId"], "robotx-pi")

    def test_deeply_nested_payload_is_truncated_not_recursed_forever(self):
        payload = current = {}
        for _ in range(50):
            current["next"] = {}
            current = current["next"]
        self.assertIsNotNone(redact(payload))


class TestCommandModeMapping(unittest.TestCase):
    def test_commands_map_to_the_expected_modes(self):
        self.assertIs(mode_for_command(CommandType.STOP), OperatingMode.STOPPED)
        self.assertIs(mode_for_command(CommandType.PAUSE), OperatingMode.PAUSED)
        self.assertIs(mode_for_command(CommandType.RESUME), OperatingMode.AUTO)

    def test_return_has_no_fixed_mode(self):
        self.assertIsNone(mode_for_command(CommandType.RETURN))


if __name__ == "__main__":
    unittest.main()
