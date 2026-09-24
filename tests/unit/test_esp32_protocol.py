"""ESP32 protocol v2: framing, CRC, strict decoding and command encoding.

Ground truth is PROTOCOL.md and real frames captured from the physical ESP32
(tests/fixtures/esp32.py). Where the reference implementation `rover_link.py`
is present at the repository root, this module is also cross-checked against
it, unmodified.
"""

import importlib.util
import pathlib
import unittest

from robotx.esp32 import protocol as p
from robotx.esp32.protocol import FrameError, Frame, Rejected
from robotx.esp32.transport import LineAssembler
from tests.fixtures import esp32 as fx

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]


def load_reference():
    path = REPO_ROOT / "rover_link.py"
    if not path.exists():
        return None
    spec = importlib.util.spec_from_file_location("rover_link_reference", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestCrc(unittest.TestCase):
    def test_check_value(self):
        self.assertEqual(p.crc16(b"123456789"), 0x29B1)

    def test_protocol_md_example(self):
        payload = b'{"type":"COMMAND","seq":42,"cmd":"DRIVE","left":150,"right":150}'
        self.assertEqual(p.crc16(payload), 0xE0DC)

    def test_every_real_frame_validates(self):
        for line in (fx.REAL_TELEMETRY, fx.REAL_DIAG_FRONT, fx.REAL_DIAG_REAR, fx.REAL_DIAG_SYSTEM):
            self.assertIsInstance(p.decode_line(line), Frame, line[:40])


class TestDecodeValid(unittest.TestCase):
    def test_real_telemetry(self):
        f = p.decode_line(fx.REAL_TELEMETRY)
        self.assertEqual(f.type, "TELEMETRY")
        self.assertEqual(f.data["uptime_ms"], 3588443)
        self.assertIs(f.data["motor_drive_available"], False)
        self.assertEqual(f.data["rear_mm"], [None, None, None])
        self.assertEqual(f.data["front_left_cm"], 37.9)

    def test_real_diag_sections_including_the_nested_link_object(self):
        system = p.decode_line(fx.REAL_DIAG_SYSTEM)
        self.assertEqual(system.data["section"], "SYSTEM")
        self.assertEqual(system.data["link"]["rx_ok"], 13)
        self.assertEqual(p.decode_line(fx.REAL_DIAG_FRONT).data["section"], "FRONT")
        self.assertEqual(p.decode_line(fx.REAL_DIAG_REAR).data["section"], "REAR")

    def test_decoded_data_is_read_only(self):
        f = p.decode_line(fx.REAL_TELEMETRY)
        with self.assertRaises(TypeError):
            f.data["state"] = "X"

    def test_ack_error_event(self):
        self.assertEqual(p.decode_line(fx.ack(101)).type, "ACK")
        e = p.decode_line(fx.error("INVALID_CRC"))
        self.assertEqual((e.type, e.data["seq"], e.data["reason"]), ("ERROR", None, "INVALID_CRC"))
        echoed = p.decode_line(fx.error("INVALID_MESSAGE_TYPE", seq=17, field="type"))
        self.assertEqual(echoed.data["seq"], 17)
        r = p.decode_line(fx.ready())
        self.assertEqual((r.type, r.data["event"], r.data["proto"]), ("EVENT", "READY", 2))

    def test_crlf_is_tolerated(self):
        line = fx.REAL_TELEMETRY[:-1] + b"\r\n"
        self.assertIsInstance(p.decode_line(line), Frame)

    def test_lowercase_crc_is_accepted(self):
        payload = fx.REAL_TELEMETRY[: fx.REAL_TELEMETRY.rindex(b"*")]
        self.assertIsInstance(p.decode_line(payload + b"*%04x\n" % p.crc16(payload)), Frame)

    def test_line_without_its_lf(self):
        self.assertIsInstance(p.decode_line(fx.REAL_TELEMETRY[:-1]), Frame)


class TestDecodeRejects(unittest.TestCase):
    def reason(self, line):
        result = p.decode_line(line)
        self.assertIsInstance(result, Rejected, line[:60])
        return result.reason

    def test_empty_and_cr_only(self):
        self.assertIs(self.reason(b"\n"), FrameError.EMPTY)
        self.assertIs(self.reason(b"\r\n"), FrameError.EMPTY)

    def test_bad_crc(self):
        line = fx.frame({"type": "ERROR", "seq": None, "reason": "X"}, crc=0x0000)
        self.assertIs(self.reason(line), FrameError.BAD_CRC)

    def test_a_corrupted_payload_under_its_original_crc(self):
        corrupted = fx.REAL_TELEMETRY.replace(b'"left_cmd":0', b'"left_cmd":9')
        self.assertIs(self.reason(corrupted), FrameError.BAD_CRC)

    def test_boot_rom_text_and_misaligned_garbage(self):
        self.assertIs(self.reason(fx.ROM_LINE), FrameError.NOT_A_FRAME)
        self.assertIs(self.reason(fx.REAL_PARTIAL_ON_OPEN), FrameError.NOT_ASCII)

    def test_a_stray_byte_before_a_valid_frame(self):
        # Seen physically: 0xF0 followed by a complete DIAG frame.
        self.assertIs(self.reason(b"\xf0" + fx.REAL_DIAG_FRONT), FrameError.NOT_ASCII)
        self.assertIs(self.reason(b"xx" + fx.REAL_DIAG_FRONT), FrameError.NOT_A_FRAME)

    def test_missing_or_short_trailer(self):
        payload = fx.REAL_TELEMETRY[: fx.REAL_TELEMETRY.rindex(b"*")]
        self.assertIs(self.reason(payload + b"\n"), FrameError.NOT_A_FRAME)
        self.assertIs(self.reason(payload + b"*123\n"), FrameError.NOT_A_FRAME)

    def test_too_long(self):
        self.assertIs(self.reason(b"{" + b"a" * p.INBOUND_LINE_MAX + b"}*0000\n"), FrameError.TOO_LONG)

    def test_malformed_json(self):
        self.assertIs(self.reason(fx.frame('{"type":"ACK",}')), FrameError.BAD_JSON)
        self.assertIs(self.reason(fx.frame('{"type":"ACK"} extra')), FrameError.NOT_A_FRAME)

    def test_duplicate_keys_anywhere(self):
        self.assertIs(self.reason(fx.frame('{"type":"ERROR","seq":null,"reason":"A","reason":"B"}')),
                      FrameError.BAD_JSON)

    def test_non_finite_numbers(self):
        self.assertIs(self.reason(fx.frame('{"type":"ERROR","seq":NaN,"reason":"A"}')),
                      FrameError.BAD_JSON)

    def test_unknown_and_missing_types(self):
        self.assertIs(self.reason(fx.frame({"type": "FOO"})), FrameError.UNKNOWN_TYPE)
        # COMMAND is Pi -> ESP32 only; one arriving at the Pi is not understood.
        self.assertIs(self.reason(fx.frame({"type": "COMMAND", "seq": 1, "cmd": "PING"})),
                      FrameError.UNKNOWN_TYPE)
        self.assertIs(self.reason(fx.frame({"seq": 1})), FrameError.UNKNOWN_TYPE)
        self.assertIs(self.reason(fx.frame('[1,2]')), FrameError.NOT_A_FRAME)

    def test_telemetry_with_a_missing_documented_field(self):
        data = dict(fx.REAL_TELEMETRY_FIELDS)
        del data["motor_drive_available"]
        self.assertIs(self.reason(fx.frame(data)), FrameError.BAD_FIELDS)

    def test_telemetry_field_types_are_checked(self):
        self.assertIs(self.reason(fx.telemetry(motor_drive_available=1)), FrameError.BAD_FIELDS)
        self.assertIs(self.reason(fx.telemetry(left_cmd=True)), FrameError.BAD_FIELDS)
        self.assertIs(self.reason(fx.telemetry(uptime_ms=1.5)), FrameError.BAD_FIELDS)
        self.assertIs(self.reason(fx.telemetry(front_warning="yes")), FrameError.BAD_FIELDS)

    def test_null_where_documented_is_accepted(self):
        self.assertIsInstance(p.decode_line(fx.telemetry(front_left_cm=None, last_seq=None)), Frame)

    def test_undocumented_ack_result_and_diag_section(self):
        self.assertIs(self.reason(fx.ack(5, result="MAYBE")), FrameError.BAD_FIELDS)
        self.assertIs(self.reason(fx.frame({"type": "DIAG", "section": "X", "uptime_ms": 1})),
                      FrameError.BAD_FIELDS)
        self.assertIs(self.reason(fx.frame({"type": "EVENT", "event": "READY", "seq": None})),
                      FrameError.BAD_FIELDS)


class TestEncode(unittest.TestCase):
    def test_exact_frames(self):
        # The two PING frames sent to the real ESP32 during verification.
        self.assertEqual(p.encode_command(101, "PING"),
                         b'{"type":"COMMAND","seq":101,"cmd":"PING"}*6FCA\n')
        self.assertEqual(p.encode_command(42, "DRIVE", left=150, right=150),
                         b'{"type":"COMMAND","seq":42,"cmd":"DRIVE","left":150,"right":150}*E0DC\n')
        self.assertEqual(p.encode_command(7, "STOP"), fx.frame('{"type":"COMMAND","seq":7,"cmd":"STOP"}'))

    def test_only_ping_stop_and_drive_exist(self):
        self.assertEqual(set(p.ALLOWED_COMMANDS), {"PING", "STOP", "DRIVE"})
        for forbidden in ("MOVE", "MOTORTEST", "RESET", "I2CSCAN", "TOFTEST", "HWREPORT", "drive", ""):
            with self.assertRaises(p.CommandError, msg=forbidden):
                p.encode_command(1, forbidden)

    def test_field_rules(self):
        bad = [
            dict(left=150),                          # missing
            dict(left=1, right=2, speed=3),          # unknown field
            dict(left=256, right=0),                 # out of range
            dict(left=-256, right=0),
            dict(left=1.5, right=0),                 # float
            dict(left=True, right=0),                # bool is not an int
            dict(left="150", right=0),
        ]
        for fields in bad:
            with self.assertRaises(p.CommandError, msg=fields):
                p.encode_command(1, "DRIVE", **fields)
        with self.assertRaises(p.CommandError):
            p.encode_command(1, "PING", x=1)

    def test_seq_rules(self):
        for seq in (0, 65536, -1, True, 1.0, None):
            with self.assertRaises(p.CommandError, msg=seq):
                p.encode_command(seq, "PING")
        p.encode_command(1, "PING")
        p.encode_command(65535, "PING")

    def test_longest_frame_is_within_the_esp32_limit(self):
        frame = p.encode_command(65535, "DRIVE", left=-255, right=-255)
        self.assertLessEqual(len(frame) - 1, p.COMMAND_LINE_MAX)

    def test_encoded_frames_decode_as_frames_would(self):
        payload = p.encode_command(9, "STOP")[:-6]
        self.assertEqual(p.crc16(payload), int(p.encode_command(9, "STOP")[-5:-1], 16))


class TestSequenceCounter(unittest.TestCase):
    def test_starts_at_one_and_wraps_to_one(self):
        c = p.SequenceCounter()
        self.assertEqual(c.next(), 1)
        c.resync(65534)
        self.assertEqual([c.next(), c.next(), c.next()], [65535, 1, 2])

    def test_resync_continues_from_the_esp32(self):
        c = p.SequenceCounter()
        c.resync(32768)
        self.assertEqual(c.next(), 32769)
        c.resync(None)           # after an ESP32 reboot: last_seq is null
        self.assertEqual(c.next(), 1)
        for invalid in (0, 70000, "5", True):
            c.resync(invalid)
            self.assertEqual(c.next(), 1)


class TestLineAssembler(unittest.TestCase):
    def test_partial_frames_across_reads(self):
        a = LineAssembler()
        line = fx.REAL_TELEMETRY
        self.assertEqual(a.feed(line[:17]), [])
        self.assertEqual(a.feed(line[17:300]), [])
        self.assertEqual(a.feed(line[300:]), [line])

    def test_several_frames_in_one_read(self):
        a = LineAssembler()
        data = fx.REAL_TELEMETRY + fx.REAL_DIAG_FRONT + fx.REAL_DIAG_REAR[:50]
        self.assertEqual(a.feed(data), [fx.REAL_TELEMETRY, fx.REAL_DIAG_FRONT])
        self.assertEqual(a.feed(fx.REAL_DIAG_REAR[50:]), [fx.REAL_DIAG_REAR])

    def test_overlong_line_is_discarded_whole_and_counted_once(self):
        a = LineAssembler(max_line=100)
        self.assertEqual(a.feed(b"x" * 80), [])
        self.assertEqual(a.feed(b"x" * 80), [])
        self.assertEqual(a.feed(b"y" * 500), [])
        self.assertEqual(a.overflows, 1)
        # Its tail up to the LF is dropped; the next line is intact.
        self.assertEqual(a.feed(b"tail\n" + fx.REAL_DIAG_FRONT), [fx.REAL_DIAG_FRONT])
        self.assertEqual(a.overflows, 1)


class TestAgainstReferenceImplementation(unittest.TestCase):
    """rover_link.py is the protocol's reference implementation; agree with it."""

    @classmethod
    def setUpClass(cls):
        cls.rl = load_reference()
        if cls.rl is None:
            raise unittest.SkipTest("rover_link.py reference not present")

    def test_crc_agrees(self):
        for data in (b"", b"123456789", fx.REAL_TELEMETRY, bytes(range(256))):
            self.assertEqual(p.crc16(data), self.rl.crc16(data))

    def test_command_encoding_agrees(self):
        for seq, cmd, fields in ((1, "PING", {}), (65535, "STOP", {}),
                                 (42, "DRIVE", {"left": 150, "right": 150}),
                                 (300, "DRIVE", {"left": -255, "right": 0})):
            self.assertEqual(p.encode_command(seq, cmd, **fields), self.rl.command(seq, cmd, **fields))

    def test_decoding_agrees_on_real_and_invalid_lines(self):
        lines = [fx.REAL_TELEMETRY, fx.REAL_DIAG_FRONT, fx.REAL_DIAG_REAR, fx.REAL_DIAG_SYSTEM,
                 fx.ack(3), fx.error("INVALID_CRC"), fx.ready(), fx.ROM_LINE,
                 fx.REAL_PARTIAL_ON_OPEN, fx.frame({"type": "ERROR", "seq": None, "reason": "X"}, crc=1)]
        for line in lines:
            mine = p.decode_line(line)
            ref = self.rl.decode(line)
            if ref is None:
                self.assertIsInstance(mine, Rejected, line[:40])
            else:
                self.assertIsInstance(mine, Frame, line[:40])
                self.assertEqual(dict(mine.data), ref)


if __name__ == "__main__":
    unittest.main()
