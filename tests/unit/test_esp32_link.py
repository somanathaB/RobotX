"""Esp32Link over a fake serial port: lifecycle, state, sequence, and safety.

No hardware. Most tests drive `poll_once()` directly with a fake clock, so
every transition is deterministic; the lifecycle tests run the real I/O thread.
"""

import logging
import time
import unittest

from robotx.control.motion import MotionIntent
from robotx.control.safety import SafetyVerdict
from robotx.esp32.link import Esp32Config, Esp32Link, to_drive_units
from robotx.esp32.state import Esp32LinkStatus as S
from tests.fixtures import esp32 as fx


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


def fresh(**cfg):
    port = fx.FakePort()
    link, clock = fx.make_link(port, **cfg)
    return link, port, clock


class TestConfig(unittest.TestCase):
    def test_serial_aliases_are_refused(self):
        for alias in ("/dev/serial0", "/dev/serial1"):
            with self.assertRaises(ValueError):
                Esp32Config(port=alias)

    def test_motion_requires_transmit(self):
        with self.assertRaises(ValueError):
            Esp32Config(transmit_enabled=False, motion_enabled=True)

    def test_timing_values_are_validated(self):
        from robotx.esp32.link import MAX_COMMAND_AGE_LIMIT_S

        Esp32Config(max_command_age_s=MAX_COMMAND_AGE_LIMIT_S)
        for bad in (dict(max_command_age_s=0), dict(max_command_age_s=1.01),
                    dict(max_command_age_s=-1), dict(stale_after_s=0), dict(ack_timeout_s=-0.1),
                    dict(reconnect_initial_s=5.0, reconnect_max_s=1.0), dict(max_pending=0)):
            with self.assertRaises(ValueError, msg=bad):
                Esp32Config(**bad)

    def test_from_settings_carries_every_timing_value(self):
        from robotx.config.settings import Settings

        s = Settings.from_env({"ROBOTX_ESP32_COMMAND_MAX_AGE_S": "0.25",
                               "ROBOTX_ESP32_ACK_TIMEOUT_S": "0.4",
                               "ROBOTX_ESP32_STALE_AFTER_S": "1.5"})
        cfg = Esp32Config.from_settings(s)
        self.assertEqual((cfg.max_command_age_s, cfg.ack_timeout_s, cfg.stale_after_s),
                         (0.25, 0.4, 1.5))

    def test_defaults_are_the_safe_ones(self):
        cfg = Esp32Config()
        self.assertEqual((cfg.port, cfg.baudrate), ("/dev/ttyAMA0", 115200))
        self.assertFalse(cfg.motion_enabled)


class TestOpenAndReconnect(unittest.TestCase):
    def test_open_failure_is_disconnected_with_bounded_backoff(self):
        link, clock = fx.make_link(OSError("no such device"), OSError("still no"), fx.FakePort())
        link.poll_once()
        st = link.status()
        self.assertIs(st.link, S.DISCONNECTED)
        self.assertIn("cannot open", st.detail)
        self.assertEqual(st.controller.counters.open_failures, 1)

        link.poll_once()          # inside the backoff: no second attempt
        self.assertEqual(link.status().controller.counters.open_failures, 1)
        clock.advance(1.0)
        link.poll_once()          # second failure; backoff doubles to 2 s
        self.assertEqual(link.status().controller.counters.open_failures, 2)
        clock.advance(1.5)
        link.poll_once()
        self.assertIs(link.status().link, S.DISCONNECTED)
        clock.advance(0.6)
        link.poll_once()
        self.assertIs(link.status().link, S.CONNECTING)

    def test_backoff_is_capped(self):
        link, clock = fx.make_link(*[OSError("x")] * 12, reconnect_max_s=4.0)
        for _ in range(12):
            link.poll_once()
            clock.advance(10.0)
        self.assertLessEqual(link._backoff, 4.0)

    def test_read_failure_disconnects_and_reconnects_without_replay(self):
        first, second = fx.FakePort(), fx.FakePort()
        link, clock = fx.make_link(first, second, motion_enabled=True)
        fx.bring_up(link, first, clock)
        link.submit(fx.decision(MotionIntent.forward(0.4)))   # pending, never flushed
        first.fail_read = OSError("device reports readiness to read but returned no data")
        link.poll_once()

        st = link.status()
        self.assertIs(st.link, S.DISCONNECTED)
        self.assertTrue(first.closed)
        self.assertEqual(st.controller.counters.disconnects, 1)
        self.assertFalse(st.controller.bidirectional)

        clock.advance(1.0)
        link.poll_once()
        self.assertIs(link.status().link, S.CONNECTING)
        self.assertEqual(second.writes, [b"\n"])       # resync only: nothing replayed
        second.feed(fx.telemetry(uptime_ms=3600000))
        for _ in range(3):
            link.poll_once()
        self.assertEqual(second.command_names(), ["PING"])
        self.assertIs(link.status().link, S.CONNECTING)  # not UP until PING is answered

    def test_write_failure_disconnects(self):
        link, port, clock = fresh()
        link.poll_once()
        port.feed(fx.REAL_TELEMETRY)
        port.fail_write = OSError("write failed")
        link.poll_once()
        self.assertIs(link.status().link, S.DISCONNECTED)
        self.assertTrue(port.closed)


class TestReceive(unittest.TestCase):
    def test_telemetry_maps_field_for_field_and_invents_nothing(self):
        link, port, clock = fresh(transmit_enabled=False)
        link.poll_once()
        port.feed(fx.REAL_TELEMETRY)
        link.poll_once()
        tel = link.status().controller.telemetry
        self.assertEqual(tel.uptime_ms, 3588443)
        self.assertEqual((tel.state, tel.block_reason), ("DRIVE_UNAVAILABLE", "MOTOR_PWM_UNAVAILABLE"))
        self.assertEqual(tel.last_seq, 32768)
        self.assertEqual((tel.left_cmd, tel.right_applied), (0, 0))
        self.assertEqual((tel.front_left_cm, tel.front_right_cm), (37.9, 38.0))
        self.assertEqual(tel.rear_mm, (None, None, None))
        self.assertIs(tel.motor_drive_available, False)
        self.assertIs(tel.front_obstacle, True)

    def test_optional_fields_absent_are_none_not_defaulted(self):
        data = dict(fx.REAL_TELEMETRY_FIELDS)
        for key in ("front_warning", "rear_available", "rear_obstacle", "rear_sensor_fault"):
            del data[key]
        link, port, _ = fresh(transmit_enabled=False)
        link.poll_once()
        port.feed(fx.frame(data))
        link.poll_once()
        tel = link.status().controller.telemetry
        self.assertIsNone(tel.front_warning)
        self.assertIsNone(tel.rear_available)
        self.assertIsNone(tel.rear_sensor_fault)

    def test_before_any_telemetry_nothing_is_reported(self):
        link, _, _ = fresh()
        link.poll_once()
        ctrl = link.status().controller
        self.assertIsNone(ctrl.telemetry)
        self.assertIsNone(ctrl.telemetry_age_s)
        self.assertIsNone(ctrl.proto)
        self.assertIsNone(ctrl.firmware)

    def test_diag_sections_are_kept_apart_from_telemetry(self):
        link, port, _ = fresh(transmit_enabled=False)
        link.poll_once()
        port.feed(fx.REAL_DIAG_SYSTEM + fx.REAL_DIAG_FRONT + fx.REAL_DIAG_REAR)
        link.poll_once()
        st = link.status()
        self.assertEqual(set(st.diag.sections), {"SYSTEM", "FRONT", "REAR"})
        self.assertEqual(st.diag.system["pca_status"], "ADDRESS_UNCONFIRMED")
        self.assertEqual(st.diag.system["link"]["rx_ok"], 13)
        self.assertEqual(st.controller.proto, 2)
        self.assertIsNone(st.controller.telemetry)

    def test_malformed_bad_crc_and_unknown_lines_are_counted_not_applied(self):
        link, port, _ = fresh(transmit_enabled=False)
        link.poll_once()
        port.feed(fx.REAL_TELEMETRY)                 # the first line after open
        link.poll_once()
        port.feed(
            fx.ROM_LINE
            + fx.REAL_TELEMETRY.replace(b'"left_cmd":0', b'"left_cmd":9')   # bad CRC
            + fx.frame('{"type":"TELEMETRY",}')                             # bad JSON
            + fx.frame({"type": "FOO"})                                      # unknown type
            + fx.frame({"type": "COMMAND", "seq": 1, "cmd": "DRIVE"})        # not for the Pi
            + b"\r\n"                                                         # resync: ignored
        )
        link.poll_once()
        c = link.status().controller.counters
        self.assertEqual(c.rejected_lines, 5)
        self.assertEqual(c.bad_crc, 1)
        self.assertEqual(c.frames_ok, 1)
        self.assertEqual(link.status().controller.telemetry.left_cmd, 0)

    def test_the_first_line_after_open_is_expected_garbage(self):
        link, port, _ = fresh(transmit_enabled=False)
        link.poll_once()
        port.feed(fx.REAL_PARTIAL_ON_OPEN + fx.REAL_TELEMETRY)
        link.poll_once()
        c = link.status().controller.counters
        self.assertEqual((c.partial_on_open, c.rejected_lines, c.frames_ok), (1, 0, 1))

    def test_partial_and_back_to_back_frames(self):
        link, port, _ = fresh(transmit_enabled=False)
        link.poll_once()
        line = fx.REAL_TELEMETRY
        port.feed(line[:100], line[100:] + fx.REAL_DIAG_FRONT[:20], fx.REAL_DIAG_FRONT[20:])
        for _ in range(3):
            link.poll_once()
        c = link.status().controller.counters
        self.assertEqual((c.telemetry, c.diag, c.rejected_lines), (1, 1, 0))

    def test_an_overlong_line_is_bounded(self):
        link, port, _ = fresh(transmit_enabled=False)
        link.poll_once()
        port.feed(fx.REAL_TELEMETRY, b"A" * 5000, b"A" * 5000, b"\n" + fx.REAL_DIAG_FRONT)
        for _ in range(4):
            link.poll_once()
        c = link.status().controller.counters
        self.assertEqual(c.too_long, 1)
        self.assertEqual(c.diag, 1)
        self.assertLess(len(link._assembler._buf), 5000)


class TestStatusTransitions(unittest.TestCase):
    def test_the_normal_path(self):
        link, port, clock = fresh()
        self.assertIs(link.status().link, S.DISCONNECTED)
        link.poll_once()
        self.assertIs(link.status().link, S.CONNECTING)
        self.assertEqual(port.writes, [b"\n"])                  # documented resync first
        port.feed(fx.REAL_TELEMETRY)
        link.poll_once()
        st = link.status()
        self.assertIs(st.link, S.CONNECTING)
        self.assertIn("PING", st.detail)
        ping = port.commands()[-1]
        self.assertEqual(ping, {"type": "COMMAND", "seq": 32769, "cmd": "PING"})  # last_seq + 1
        port.feed(fx.ack(32769))
        clock.advance(0.02)
        link.poll_once()
        st = link.status()
        self.assertIs(st.link, S.UP)
        self.assertTrue(st.controller.bidirectional)
        self.assertAlmostEqual(st.controller.ping_rtt_ms, 20.0, places=3)

    def test_receive_only_is_up_without_writing_a_byte(self):
        link, port, _ = fresh(transmit_enabled=False)
        link.poll_once()
        port.feed(fx.REAL_TELEMETRY)
        link.poll_once()
        self.assertIs(link.status().link, S.UP)
        self.assertIn("receive-only", link.status().detail)
        link.submit(fx.decision(MotionIntent.forward(0.5)))
        link.poll_once()
        self.assertEqual(port.writes, [])

    def test_stale_then_recovered(self):
        link, port, clock = fresh()
        fx.bring_up(link, port, clock)
        clock.advance(1.5)
        link.poll_once()
        st = link.status()
        self.assertIs(st.link, S.STALE)
        self.assertIn("no TELEMETRY", st.detail)
        port.feed(fx.telemetry(uptime_ms=3590000))
        link.poll_once()
        self.assertIs(link.status().link, S.UP)

    def test_waiting_through_the_boot_scan_is_connecting_not_stale(self):
        link, port, clock = fresh()
        link.poll_once()
        port.feed(fx.ready(uptime_ms=500))
        link.poll_once()
        clock.advance(60.0)
        link.poll_once()
        st = link.status()
        self.assertIs(st.link, S.CONNECTING)
        self.assertIn("boot", st.detail)
        self.assertTrue(st.controller.ready_seen)
        self.assertEqual(st.controller.firmware, "82f2a8a")
        self.assertEqual(st.controller.reboot_count, 0)       # a first READY is not a reboot
        self.assertEqual(port.command_names(), [])            # nothing sent into the blocked ESP32

    def test_stop_goes_through_stopping_to_disconnected(self):
        link, port, clock = fresh()
        fx.bring_up(link, port, clock)
        link.stop()
        self.assertIs(link.status().link, S.DISCONNECTED)
        self.assertTrue(port.closed)


class TestSequenceAndResponses(unittest.TestCase):
    def test_ack_timeouts_degrade_and_recover(self):
        link, port, clock = fresh(ping_retry_s=0.6)
        link.poll_once()
        port.feed(fx.REAL_TELEMETRY)
        link.poll_once()
        for i in range(3):                      # three unanswered PINGs
            clock.advance(0.6)
            port.feed(fx.telemetry(uptime_ms=3588443 + (i + 1) * 600))
            link.poll_once()
            link.poll_once()
        st = link.status()
        self.assertEqual(st.controller.counters.ack_timeouts, 3)
        self.assertIs(st.link, S.DEGRADED)
        self.assertIn("unanswered", st.detail)
        last_ping = port.commands()[-1]["seq"]
        port.feed(fx.ack(last_ping))
        link.poll_once()
        self.assertIs(link.status().link, S.UP)

    def test_stale_seq_resyncs_from_the_esp32(self):
        link, port, clock = fresh()
        link.poll_once()
        port.feed(fx.REAL_TELEMETRY)
        link.poll_once()
        ping = port.commands()[-1]["seq"]
        port.feed(fx.ack(ping, result="REJECTED", reason="STALE_SEQ", last_seq=40000))
        link.poll_once()
        link.poll_once()
        self.assertEqual(port.commands()[-1], {"type": "COMMAND", "seq": 40001, "cmd": "PING"})

    def test_error_frames_are_parsed_and_close_their_command(self):
        link, port, clock = fresh()
        link.poll_once()
        port.feed(fx.REAL_TELEMETRY)
        link.poll_once()
        ping = port.commands()[-1]["seq"]
        port.feed(fx.error("INVALID_MESSAGE_TYPE", seq=ping, field="type"), fx.error("INVALID_CRC"))
        link.poll_once()
        link.poll_once()
        st = link.status()
        self.assertEqual(st.controller.counters.esp32_errors, 2)
        self.assertEqual(st.controller.last_error, "INVALID_CRC")
        self.assertEqual(link._pending, {})
        self.assertEqual(st.controller.counters.ack_timeouts, 0)

    def test_a_late_ack_is_unmatched_not_applied(self):
        link, port, clock = fresh()
        fx.bring_up(link, port, clock)
        port.feed(fx.ack(12345))
        link.poll_once()
        self.assertEqual(link.status().controller.counters.unmatched_responses, 1)

    def test_protocol_version_mismatch_is_degraded(self):
        link, port, clock = fresh()
        link.poll_once()
        port.feed(fx.REAL_TELEMETRY)
        link.poll_once()
        port.feed(fx.ack(port.commands()[-1]["seq"], proto=3))
        link.poll_once()
        st = link.status()
        self.assertIs(st.link, S.DEGRADED)
        self.assertIn("protocol", st.detail)

    def test_pending_commands_are_bounded(self):
        link, port, clock = fresh(motion_enabled=True, max_pending=2)
        fx.bring_up(link, port, clock)
        for _ in range(5):
            link.submit(fx.decision(MotionIntent.stop("x")))
            link.poll_once()
        self.assertLessEqual(len(link._pending), 2)


class TestReboot(unittest.TestCase):
    def assert_latched(self, link):
        st = link.status()
        self.assertIs(st.link, S.DEGRADED)
        self.assertTrue(st.controller.reboot_latched)
        self.assertEqual(st.controller.reboot_count, 1)
        self.assertFalse(st.motion_ready)

    def test_uptime_going_backwards(self):
        link, port, clock = fresh(motion_enabled=True)
        fx.bring_up(link, port, clock)
        port.feed(fx.telemetry(uptime_ms=900, last_seq=None))
        link.poll_once()
        self.assert_latched(link)

    def test_ready_from_a_running_esp32(self):
        link, port, clock = fresh(motion_enabled=True)
        fx.bring_up(link, port, clock)
        port.feed(fx.ready())
        link.poll_once()
        self.assert_latched(link)

    def test_the_latch_holds_motion_until_acknowledged_then_requires_a_fresh_ping(self):
        link, port, clock = fresh(motion_enabled=True)
        fx.bring_up(link, port, clock)
        port.feed(fx.telemetry(uptime_ms=900, last_seq=None))
        link.poll_once()
        sent_before = len(port.writes)
        port.feed(fx.telemetry(uptime_ms=1100, last_seq=None))
        link.submit(fx.decision(MotionIntent.stop("x")))
        link.poll_once()
        self.assertEqual(len(port.writes), sent_before)          # nothing while latched

        self.assertTrue(link.acknowledge_reboot())
        port.feed(fx.telemetry(uptime_ms=1300, last_seq=None))
        link.poll_once()
        self.assertIs(link.status().link, S.CONNECTING)
        ping = port.commands()[-1]
        self.assertEqual(ping, {"type": "COMMAND", "seq": 1, "cmd": "PING"})  # seq restarts at 1
        port.feed(fx.ack(1))
        link.poll_once()
        self.assertIs(link.status().link, S.UP)
        self.assertFalse(link.acknowledge_reboot())


class TestMotionCommands(unittest.TestCase):
    """Only reachable with motion enabled; exercised against fakes only."""

    def up_with_drive(self, **cfg):
        link, port, clock = fresh(motion_enabled=True, **cfg)
        link.poll_once()
        port.feed(fx.telemetry(motor_drive_available=True, last_seq=None))
        link.poll_once()
        port.feed(fx.ack(1))
        link.poll_once()
        self.assertIs(link.status().link, S.UP)
        return link, port, clock

    def test_drive_units(self):
        self.assertEqual([to_drive_units(v) for v in (0.0, 1.0, -1.0, 0.5, -0.5, 0.75, 2.0, -3.0)],
                         [0, 255, -255, 128, -128, 191, 255, -255])

    def test_gated_forward_intent_becomes_one_drive(self):
        link, port, clock = self.up_with_drive()
        link.submit(fx.decision(MotionIntent.forward(0.5, steer=0.2)))
        link.poll_once()
        link.poll_once()
        drives = [c for c in port.commands() if c["cmd"] == "DRIVE"]
        self.assertEqual(drives, [{"type": "COMMAND", "seq": 2, "cmd": "DRIVE", "left": 128, "right": 102}])

    def test_turn_intents_map_to_opposed_sides(self):
        link, port, clock = self.up_with_drive()
        link.submit(fx.decision(MotionIntent.turn_left(0.4)))
        link.poll_once()
        cmd = port.commands()[-1]
        self.assertEqual((cmd["left"], cmd["right"]), (-102, 102))

    def test_stop_and_hold_intents_are_stop(self):
        link, port, clock = self.up_with_drive()
        for intent in (MotionIntent.stop("x"), MotionIntent.hold("idle")):
            link.submit(fx.decision(intent))
            link.poll_once()
        self.assertEqual(port.command_names()[-2:], ["STOP", "STOP"])

    def test_a_vetoed_decision_is_stop_whatever_its_intent(self):
        link, port, clock = self.up_with_drive()
        vetoed = fx.decision(MotionIntent.forward(0.6), SafetyVerdict.VETOED)
        link.submit(vetoed)
        link.poll_once()
        self.assertEqual(port.command_names()[-1], "STOP")

    def test_only_a_safety_decision_is_accepted(self):
        link, _, _ = self.up_with_drive()
        with self.assertRaises(TypeError):
            link.submit(MotionIntent.forward(0.5))

    def test_a_stale_intent_is_stop(self):
        link, port, clock = self.up_with_drive()
        old = MotionIntent(command=MotionIntent.forward(0.5).command, left=0.5, right=0.5,
                           timestamp=time.time() - 5.0)
        link.submit(fx.decision(old))
        link.poll_once()
        self.assertEqual(port.command_names()[-1], "STOP")

    def test_a_command_not_sent_in_time_is_dropped_not_sent_late(self):
        link, port, clock = self.up_with_drive()
        before = len(port.writes)
        link.submit(fx.decision(MotionIntent.forward(0.5)))
        clock.advance(0.5)
        port.feed(fx.telemetry(motor_drive_available=True, uptime_ms=3590000))
        link.poll_once()
        self.assertEqual(len(port.writes), before)
        self.assertEqual(link.status().controller.counters.stale_commands_dropped, 1)

    def test_a_command_is_sent_once_never_repeated(self):
        link, port, clock = self.up_with_drive()
        link.submit(fx.decision(MotionIntent.forward(0.5)))
        for _ in range(5):
            link.poll_once()
        self.assertEqual(port.command_names().count("DRIVE"), 1)

    def test_drive_unavailable_turns_drive_into_stop(self):
        link, port, clock = fresh(motion_enabled=True)
        fx.bring_up(link, port, clock)             # real telemetry: drive unavailable
        self.assertFalse(link.status().motion_ready)
        link.submit(fx.decision(MotionIntent.forward(0.5)))
        link.poll_once()
        self.assertEqual(port.command_names(), ["PING", "STOP"])

    def test_nothing_is_sent_while_the_link_is_not_up(self):
        link, port, clock = fresh(motion_enabled=True)
        link.poll_once()
        port.feed(fx.REAL_TELEMETRY)
        link.submit(fx.decision(MotionIntent.stop("x")))
        link.poll_once()                            # CONNECTING: only the PING goes out
        self.assertEqual(port.command_names(), ["PING"])

    def test_motion_disabled_sends_only_resync_and_ping(self):
        link, port, clock = fresh()
        fx.bring_up(link, port, clock)
        for intent in (MotionIntent.forward(0.5), MotionIntent.stop("x"), MotionIntent.turn_right(1.0)):
            link.submit(fx.decision(intent))
            link.poll_once()
        self.assertEqual(port.writes[0], b"\n")
        self.assertEqual(port.command_names(), ["PING"])


class TestPySerialAdapter(unittest.TestCase):
    """The adapter must never ask pyserial to fill a large buffer."""

    class FakeSerial:
        def __init__(self, waiting):
            self.in_waiting = waiting
            self.requested = []
            self.closed = False

        def read(self, n):
            self.requested.append(n)
            return b"x" * n

        def write(self, data):
            return len(data)

        def close(self):
            self.closed = True

    def test_waits_for_one_byte_when_nothing_is_buffered(self):
        from robotx.esp32.transport import PySerialPort

        s = self.FakeSerial(waiting=0)
        PySerialPort(s).read(4096)
        self.assertEqual(s.requested, [1])

    def test_takes_what_is_buffered_up_to_the_limit(self):
        from robotx.esp32.transport import PySerialPort

        s = self.FakeSerial(waiting=700)
        PySerialPort(s).read(4096)
        s.in_waiting = 9000
        port = PySerialPort(s)
        port.read(4096)
        port.close()
        self.assertEqual(s.requested, [700, 4096])
        self.assertTrue(s.closed)


class TestLifecycle(unittest.TestCase):
    """The real I/O thread, over a fake port that blocks like a serial read."""

    def test_the_io_thread_closes_the_port_and_never_reads_after(self):
        port = fx.FakePort(fx.REAL_TELEMETRY, block_s=0.01)
        link = Esp32Link(Esp32Config(transmit_enabled=False), port_factory=lambda: port)
        link.start()
        deadline = time.monotonic() + 2.0
        while link.status().link is not S.UP and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertIs(link.status().link, S.UP)
        thread = link._thread
        link.stop()
        self.assertFalse(thread.is_alive())
        self.assertTrue(port.closed)
        reads = port.reads
        time.sleep(0.05)
        self.assertEqual(port.reads, reads)       # FakePort raises on any read after close
        self.assertIs(link.status().link, S.DISCONNECTED)

    def test_starting_is_connecting_not_a_failure(self):
        import threading

        release = threading.Event()
        port = fx.FakePort(block_s=0.005)

        def slow_factory():
            release.wait(2.0)
            return port

        link = Esp32Link(Esp32Config(transmit_enabled=False), port_factory=slow_factory)
        link.start()
        try:
            st = link.status()
            self.assertIs(st.link, S.CONNECTING)
            self.assertIn("opening /dev/ttyAMA0", st.detail)
        finally:
            release.set()
            link.stop()

    def test_stop_is_idempotent_and_safe_before_start(self):
        link, port, _ = fresh()
        link.stop()
        link.poll_once()                           # a test may open a port without the thread
        link.stop()
        self.assertTrue(port.closed)
        link.stop()

    def test_a_final_stop_is_sent_only_after_motion_was_carried(self):
        link, port, clock = fresh(motion_enabled=True)
        fx.bring_up(link, port, clock)
        link.stop()
        self.assertNotIn("STOP", port.command_names())   # nothing was ever driven

        link, port, clock = fresh(motion_enabled=True)
        fx.bring_up(link, port, clock)
        link.submit(fx.decision(MotionIntent.stop("x")))
        link.poll_once()
        link.stop()
        self.assertEqual(port.command_names()[-2:], ["STOP", "STOP"])
        self.assertTrue(port.closed)

    def test_a_crashing_factory_does_not_kill_the_thread(self):
        attempts = []

        def factory():
            attempts.append(1)
            raise OSError("gone")

        link = Esp32Link(Esp32Config(reconnect_initial_s=0.01, reconnect_max_s=0.02),
                         port_factory=factory)
        link.start()
        time.sleep(0.2)
        link.stop()
        self.assertGreater(len(attempts), 2)


if __name__ == "__main__":
    unittest.main()
