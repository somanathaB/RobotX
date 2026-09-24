"""The FalconAut authentication pieces: AUTH payloads, tokens, commissioning.

These cover the parts of the contract that only appear once per connection and
are therefore easy to get wrong without noticing: which credential is presented,
what is done with the one that comes back, and the exact units of the
load-bearing timestamp.
"""

import json
import logging
import os
import stat
import tempfile
import time
import unittest


def setUpModule():
    # These tests deliberately drive the warning paths (corrupt file, wrong
    # robot, unwritable location); the log noise is not the subject under test.
    logging.getLogger("robotx").setLevel(logging.CRITICAL)

from robotx.communication.commissioning import (
    CommissioningError,
    PairingCode,
    extract_pairing_code,
)
from robotx.communication.protocol import (
    AuthMethod,
    build_auth_payload,
    build_telemetry_payload,
    now_ms,
    parse_auth_success,
    parse_stop_event,
    CommandType,
)
from robotx.communication.token_store import TokenStore
from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus
from robotx.localization.position import Position
from robotx.state.robot_state import RobotState


class TestEpochMilliseconds(unittest.TestCase):
    """`timestamp` is load-bearing on the backend; its units are not optional."""

    def test_now_ms_is_an_int_in_milliseconds(self):
        value = now_ms()
        self.assertIsInstance(value, int)
        self.assertNotIsInstance(value, bool)
        # Milliseconds since 1970 are a 13-digit number in this era; seconds
        # would be 10. This catches the single most likely unit mistake.
        self.assertEqual(len(str(value)), 13)

    def test_now_ms_tracks_the_wall_clock(self):
        self.assertAlmostEqual(now_ms() / 1000.0, time.time(), delta=1.0)

    def test_now_ms_converts_a_given_instant_rather_than_using_now(self):
        self.assertEqual(now_ms(1_700_000_000.123), 1_700_000_000_123)

    def test_now_ms_rounds_rather_than_truncating(self):
        self.assertEqual(now_ms(1.9996), 2000)

    def test_telemetry_timestamp_is_the_measurement_time_not_the_build_time(self):
        """A frame built later must still report when the fix was taken."""

        measured = time.time() - 2.0
        state = RobotState("r")
        state.update_gps(
            GpsReading(
                status=GPSStatus.FIX,
                fix=GpsFix(latitude=1.0, longitude=2.0, timestamp=measured),
                age_s=2.0,
            ),
            Position(latitude=1.0, longitude=2.0, timestamp=measured, speed_mps=0.1),
        )
        frame = build_telemetry_payload(
            state.snapshot(), sequence=1, max_position_age_s=10.0
        )
        self.assertTrue(frame.has_position)
        self.assertEqual(frame.payload["timestamp"], now_ms(measured))
        # Which is meaningfully older than "now".
        self.assertLess(frame.payload["timestamp"], now_ms() - 1000)


class TestAuthPayload(unittest.TestCase):
    def test_pairing_code_is_used_when_there_is_no_token(self):
        payload, method = build_auth_payload(robot_id="r1", pairing_code="123456")
        self.assertIs(method, AuthMethod.PAIRING_CODE)
        self.assertEqual(payload, {"robotId": "r1", "pairingCode": "123456"})

    def test_token_wins_over_the_pairing_code(self):
        payload, method = build_auth_payload(
            robot_id="r1", token="tok", pairing_code="123456"
        )
        self.assertIs(method, AuthMethod.TOKEN)
        self.assertEqual(payload, {"robotId": "r1", "token": "tok"})

    def test_no_credential_raises_rather_than_emitting_a_doomed_auth(self):
        with self.assertRaises(ValueError):
            build_auth_payload(robot_id="r1")


class TestAuthSuccessParsing(unittest.TestCase):
    def test_plain_token_field(self):
        self.assertEqual(parse_auth_success({"token": "abc"}).token, "abc")

    def test_alternative_spellings_are_accepted(self):
        for key in ("sessionToken", "robotToken", "accessToken"):
            self.assertEqual(parse_auth_success({key: "abc"}).token, "abc")

    def test_nested_payloads_are_searched(self):
        self.assertEqual(parse_auth_success({"robot": {"token": "abc"}}).token, "abc")
        self.assertEqual(parse_auth_success({"data": {"sessionToken": "xyz"}}).token, "xyz")

    def test_a_payload_without_a_token_is_not_an_error(self):
        result = parse_auth_success({"ok": True})
        self.assertFalse(result.has_token)
        self.assertIsNone(result.token)

    def test_non_dict_payloads_do_not_raise(self):
        for junk in (None, "yes", 42, []):
            self.assertFalse(parse_auth_success(junk).has_token)

    def test_blank_tokens_are_not_accepted(self):
        self.assertFalse(parse_auth_success({"token": "   "}).has_token)


class TestBareStopEvent(unittest.TestCase):
    def test_stop_is_keyed_on_the_task_it_cancels(self):
        command = parse_stop_event({"taskId": "t-1"})
        self.assertIs(command.type, CommandType.STOP)
        self.assertEqual(command.command_id, "stop:t-1")

    def test_command_id_is_preferred_when_present(self):
        self.assertEqual(parse_stop_event({"commandId": "c-9"}).command_id, "stop:c-9")

    def test_a_stop_with_no_identity_still_produces_a_command(self):
        command = parse_stop_event({})
        self.assertIs(command.type, CommandType.STOP)
        self.assertTrue(command.command_id.startswith("stop:anonymous:"))

    def test_malformed_stop_payloads_never_raise(self):
        for junk in (None, "stop", 7, [], {"taskId": None}):
            command = parse_stop_event(junk)
            self.assertIs(command.type, CommandType.STOP)

    def test_two_stops_for_one_task_share_an_id_so_they_deduplicate(self):
        first = parse_stop_event({"taskId": "t-1"})
        second = parse_stop_event({"taskId": "t-1"})
        self.assertEqual(first.command_id, second.command_id)


class TestTokenStore(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self._tmp.name, "nested", "session.json")
        self.store = TokenStore(self.path)

    def tearDown(self):
        self._tmp.cleanup()

    def test_round_trip(self):
        self.assertTrue(self.store.save(robot_id="r1", token="tok-1"))
        loaded = self.store.load(robot_id="r1")
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded.token, "tok-1")
        self.assertEqual(loaded.robot_id, "r1")

    def test_missing_file_is_not_an_error(self):
        self.assertIsNone(self.store.load(robot_id="r1"))

    def test_a_token_for_another_robot_is_ignored(self):
        """Presenting robot A's token while claiming to be B gets a silent
        disconnect that would be very hard to diagnose."""

        self.store.save(robot_id="r1", token="tok-1")
        self.assertIsNone(self.store.load(robot_id="r2"))

    def test_corrupt_file_is_ignored_rather_than_raising(self):
        self.store.save(robot_id="r1", token="tok-1")
        with open(self.path, "w") as handle:
            handle.write("{not json")
        self.assertIsNone(self.store.load(robot_id="r1"))

    def test_file_is_not_readable_by_other_accounts(self):
        self.store.save(robot_id="r1", token="tok-1")
        mode = stat.S_IMODE(os.stat(self.path).st_mode)
        self.assertEqual(mode, 0o600)

    def test_clear_removes_the_token(self):
        self.store.save(robot_id="r1", token="tok-1")
        self.store.clear()
        self.assertIsNone(self.store.load(robot_id="r1"))

    def test_clear_on_a_missing_file_is_safe(self):
        self.store.clear()  # must not raise

    def test_overwriting_leaves_exactly_one_token_and_no_temp_files(self):
        self.store.save(robot_id="r1", token="tok-1")
        self.store.save(robot_id="r1", token="tok-2")
        self.assertEqual(self.store.load(robot_id="r1").token, "tok-2")
        leftovers = [n for n in os.listdir(os.path.dirname(self.path)) if n.startswith(".session-")]
        self.assertEqual(leftovers, [])

    def test_describe_reports_presence_never_the_token(self):
        self.store.save(robot_id="r1", token="sup3rs3cr3t")
        described = self.store.describe(robot_id="r1")
        self.assertEqual(described["token"], "SET")
        self.assertNotIn("sup3rs3cr3t", json.dumps(described))

    def test_describe_reports_unset_when_there_is_nothing(self):
        self.assertEqual(self.store.describe(robot_id="r1")["token"], "UNSET")

    def test_an_unwritable_location_is_reported_not_raised(self):
        """Failing to cache a token must never stop the agent booting."""

        store = TokenStore("/proc/definitely/not/writable/session.json")
        self.assertFalse(store.save(robot_id="r1", token="tok"))


class TestCommissioningResponseParsing(unittest.TestCase):
    def test_pairing_code_field(self):
        pairing = extract_pairing_code({"pairingCode": "123456"})
        self.assertEqual(pairing.code, "123456")
        self.assertTrue(pairing.is_well_formed)
        self.assertEqual(pairing.expires_in_s, 300)

    def test_alternative_field_names(self):
        self.assertEqual(extract_pairing_code({"code": "123456"}).code, "123456")
        self.assertEqual(extract_pairing_code({"pairing_code": "123456"}).code, "123456")

    def test_nested_response_envelope(self):
        self.assertEqual(
            extract_pairing_code({"data": {"pairingCode": "654321"}}).code, "654321"
        )

    def test_explicit_ttl_is_honoured(self):
        self.assertEqual(
            extract_pairing_code({"pairingCode": "1", "expiresIn": 120}).expires_in_s, 120
        )

    def test_a_response_without_a_code_is_an_error(self):
        with self.assertRaises(CommissioningError):
            extract_pairing_code({"status": "ok"})

    def test_a_non_object_response_is_an_error(self):
        with self.assertRaises(CommissioningError):
            extract_pairing_code(["123456"])

    def test_a_code_of_the_wrong_shape_is_flagged_not_silently_accepted(self):
        self.assertFalse(PairingCode(code="12ab").is_well_formed)
        self.assertFalse(PairingCode(code="1234567").is_well_formed)
        self.assertTrue(PairingCode(code="123456").is_well_formed)


if __name__ == "__main__":
    unittest.main()
