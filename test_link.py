#!/usr/bin/env python3
"""Protocol v2 link tests -- against the host simulator OR a real ESP32.

    python tests/test_link.py --sim <rover_sim.exe> [--sim-env KEY=VALUE ...]
    python tests/test_link.py --port /dev/ttyAMA0          (from the Pi 5)

MOTOR SAFETY. The tests need no motor. On real hardware the suite first reads
telemetry, and if "motor_drive_available" is true it REFUSES to run unless
--allow-motion is given -- in which case the rover must be on blocks. While the
PCA9685 address is unconfirmed (the current configuration) every movement
command is gated to zero by the firmware itself.

Each test sends its own sequence numbers and checks the response by seq.
Exit code 0 = all passed, 1 = a test failed, 2 = refused to run.
"""

import argparse
import statistics
import sys
import time

import rover_link as rl
from rover_link import command, encode, crc16

MIN_EFFECTIVE = 40      # config.h MOTOR_MIN_EFFECTIVE_DUTY
TURN_INNER_PCT = 55     # config.h MOTOR_TURN_INNER_PCT
LINE_MAX = 160          # config.h COMM_LINE_MAX
ERR_PER_SEC = 10        # config.h COMM_ERROR_FRAMES_PER_SEC
TIMEOUT_MS = 2000       # config.h COMMAND_TIMEOUT_MS


class Fail(Exception):
    pass


class Skip(Exception):
    pass


def check(cond, msg):
    if not cond:
        raise Fail(msg)


class Ctx:
    def __init__(self, link, physical):
        self.link = link
        self.physical = physical
        self.seq = 100
        self.latencies = []
        self.stats = {}

    # -- sending -----------------------------------------------------------
    def next_seq(self):
        self.seq = self.seq + 1 if self.seq < rl.SEQ_MAX else rl.SEQ_MIN
        return self.seq

    def cmd(self, name, seq=None, timeout=1.5, **fields):
        """Send a command, return the ACK/ERROR frame answering its seq."""
        seq = self.next_seq() if seq is None else seq
        return self.frame_for(command(seq, name, **fields), seq, timeout)

    def frame_for(self, data, seq, timeout=1.5):
        start = self.link.mark()
        t = self.link.send(data)
        rec = self.link.response(seq, start, timeout)
        check(rec is not None, f"no ACK/ERROR for seq {seq}: {data!r}")
        self.latencies.append(rec.t - t)
        return rec.frame

    def unsolicited_errors(self, data, expect_count, window=0.6):
        """Send raw bytes; return the seq-null ERROR frames that follow."""
        start = self.link.mark()
        self.link.send(data)
        deadline = time.monotonic() + window
        while time.monotonic() < deadline:
            errs = [r.frame for r in self.link.frames(start, ftype="ERROR")
                    if r.frame.get("seq") is None]
            if len(errs) >= expect_count:
                time.sleep(0.1)         # catch any extra, unexpected ones
                break
            time.sleep(0.02)
        return [r.frame for r in self.link.frames(start, ftype="ERROR")
                if r.frame.get("seq") is None], start

    def telemetry(self, timeout=1.0):
        rec = self.link.wait(lambda f: f["type"] == "TELEMETRY", self.link.mark(), timeout)
        check(rec is not None, "no TELEMETRY frame")
        return rec.frame

    def diag(self, section, timeout=2.0):
        rec = self.link.wait(lambda f: f["type"] == "DIAG" and f.get("section") == section,
                             self.link.mark(), timeout)
        check(rec is not None, f"no DIAG {section} frame")
        return rec.frame

    @staticmethod
    def settle_error_limit():
        # seq-null ERROR frames are rate limited to ERR_PER_SEC per second;
        # start tests that produce several of them in a fresh window.
        time.sleep(1.1)

    def stop(self):
        f = self.cmd("STOP")
        check(f["type"] == "ACK" and f["result"] == "ACCEPTED", f"STOP not accepted: {f}")


# ---------------------------------------------------------------------------
# Expectations
# ---------------------------------------------------------------------------
def ack(f, cmd=None, result=None, reason=None, field=None):
    check(f["type"] == "ACK", f"expected ACK, got {f}")
    if cmd is not None:
        check(f.get("cmd") == cmd, f"cmd {f.get('cmd')!r} != {cmd!r}: {f}")
    if result is not None:
        check(f.get("result") == result, f"result {f.get('result')!r} != {result!r}: {f}")
    if reason is not None:
        check(f.get("reason") == reason, f"reason {f.get('reason')!r} != {reason!r}: {f}")
    if field is not None:
        check(f.get("field") == field, f"field {f.get('field')!r} != {field!r}: {f}")
    return f


def error(f, reason, seq="any", field=None):
    check(f["type"] == "ERROR", f"expected ERROR, got {f}")
    check(f.get("reason") == reason, f"reason {f.get('reason')!r} != {reason!r}: {f}")
    if seq != "any":
        check(f.get("seq") == seq, f"seq {f.get('seq')!r} != {seq!r}: {f}")
    if field is not None:
        check(f.get("field") == field, f"field {f.get('field')!r} != {field!r}: {f}")
    return f


def effective(v):
    v = max(-255, min(255, v))
    return 0 if abs(v) < MIN_EFFECTIVE else v


def expect_motion(ctx, f, cmd, req_l, req_r):
    """Check a DRIVE/MOVE ack against what the firmware must have done."""
    tel = ctx.last_tel
    ack(f, cmd=cmd)
    check((f["req_left"], f["req_right"]) == (req_l, req_r),
          f"requested values not echoed: {f}")

    if not tel["motor_drive_available"]:
        ack(f, result="GATED", reason="MOTOR_PWM_UNAVAILABLE")
        check((f["gated_left"], f["gated_right"]) == (0, 0), f"gated must be 0: {f}")
        check((f["applied_left"], f["applied_right"]) == (0, 0), f"applied must be 0: {f}")
        return

    gl, gr = req_l, req_r
    if tel["forward_blocked"]:
        gl, gr = min(gl, 0), min(gr, 0)
    if tel["reverse_blocked"]:
        gl, gr = max(gl, 0), max(gr, 0)

    if (gl, gr) != (req_l, req_r):
        check(f["result"] == "GATED" and f["reason"] != "NONE", f"expected GATED: {f}")
    else:
        ack(f, result="ACCEPTED", reason="NONE")
    check((f["gated_left"], f["gated_right"]) == (gl, gr), f"gated values wrong: {f}")
    check((f["applied_left"], f["applied_right"]) == (effective(gl), effective(gr)),
          f"applied values wrong: {f}")


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def t01_valid_drive(ctx):
    ctx.last_tel = ctx.telemetry()
    f = ctx.cmd("DRIVE", left=150, right=150)
    expect_motion(ctx, f, "DRIVE", 150, 150)
    tel = ctx.telemetry()
    check((tel["left_cmd"], tel["right_cmd"]) == (150, 150), f"telemetry left_cmd: {tel}")
    check((tel["left_applied"], tel["right_applied"]) ==
          (f["applied_left"], f["applied_right"]), "telemetry applied != ack applied")

    for l, r in ((-100, 100), (30, 30), (-255, 255)):
        expect_motion(ctx, ctx.cmd("DRIVE", left=l, right=r), "DRIVE", l, r)
    ctx.stop()


def t02_stop(ctx):
    f = ack(ctx.cmd("STOP"), cmd="STOP", result="ACCEPTED", reason="NONE")
    check((f["applied_left"], f["applied_right"]) == (0, 0), f"STOP applied: {f}")
    tel = ctx.telemetry()
    check((tel["left_cmd"], tel["right_cmd"]) == (0, 0), "STOP did not zero left_cmd")
    ack(ctx.cmd("STOP", now=True), result="REJECTED", reason="UNKNOWN_FIELD", field="now")


def t03_move(ctx):
    ctx.last_tel = ctx.telemetry()
    inner = 100 * TURN_INNER_PCT // 100
    for d, (l, r) in (("F", (100, 100)), ("L", (inner, 100)),
                      ("R", (100, inner)), ("B", (-80, -80))):
        speed = 80 if d == "B" else 100
        expect_motion(ctx, ctx.cmd("MOVE", dir=d, speed=speed), "MOVE", l, r)

    ack(ctx.cmd("MOVE", dir="S"), cmd="MOVE", result="ACCEPTED")
    ack(ctx.cmd("MOVE", dir="S", speed=10), result="ACCEPTED")

    ctx.cmd("MOVE", dir="F", speed=100)
    ack(ctx.cmd("MOVE", dir="X", speed=100), result="REJECTED",
        reason="INVALID_ARGUMENT", field="dir")
    tel = ctx.telemetry()
    check(tel["left_cmd"] == 0, "invalid MOVE dir must stop (baseline behaviour)")

    ack(ctx.cmd("MOVE", dir="F"), result="REJECTED", reason="MISSING_FIELD", field="speed")
    ack(ctx.cmd("MOVE", dir="F", speed=256), result="REJECTED", reason="OUT_OF_RANGE")
    ack(ctx.cmd("MOVE", dir="F", speed=-1), result="REJECTED", reason="OUT_OF_RANGE")
    ack(ctx.cmd("MOVE", dir=5, speed=10), result="REJECTED", reason="WRONG_TYPE", field="dir")
    ctx.stop()


def t04_unknown_command(ctx):
    ctx.stop()
    ack(ctx.cmd("FLY"), cmd="FLY", result="REJECTED", reason="UNKNOWN_COMMAND", field="cmd")
    ack(ctx.cmd("drive", left=100, right=100), result="REJECTED", reason="UNKNOWN_COMMAND")
    check(ctx.telemetry()["left_cmd"] == 0, "unknown command changed left_cmd")


def t05_missing_field(ctx):
    ctx.settle_error_limit()
    ack(ctx.cmd("DRIVE", left=100), result="REJECTED", reason="MISSING_FIELD", field="right")
    ack(ctx.cmd("DRIVE", right=100), result="REJECTED", reason="MISSING_FIELD", field="left")
    ack(ctx.cmd("DRIVE"), result="REJECTED", reason="MISSING_FIELD", field="left")
    ack(ctx.cmd("MOTORTEST", motor=0, power=100, ms=100), result="REJECTED",
        reason="MISSING_FIELD", field="onblocks")

    errs, _ = ctx.unsolicited_errors(encode({"type": "COMMAND", "cmd": "PING"}), 1)
    check(len(errs) == 1, f"missing seq: {errs}")
    error(errs[0], "MISSING_FIELD", seq=None, field="seq")

    s = ctx.next_seq()
    error(ctx.frame_for(encode({"seq": s, "cmd": "PING"}), s), "MISSING_FIELD", s, "type")
    s = ctx.next_seq()
    error(ctx.frame_for(encode({"type": "COMMAND", "seq": s}), s), "MISSING_FIELD", s, "cmd")


def t06_wrong_type(ctx):
    ctx.settle_error_limit()
    ack(ctx.cmd("DRIVE", left="150", right=0), result="REJECTED", reason="WRONG_TYPE", field="left")
    ack(ctx.cmd("DRIVE", left=True, right=0), result="REJECTED", reason="WRONG_TYPE", field="left")
    ack(ctx.cmd("DRIVE", left=None, right=0), result="REJECTED", reason="WRONG_TYPE", field="left")
    ack(ctx.cmd("MOTORTEST", motor=0, power=100, ms=100, onblocks=1),
        result="REJECTED", reason="WRONG_TYPE", field="onblocks")

    errs, _ = ctx.unsolicited_errors(encode({"type": "COMMAND", "seq": "7", "cmd": "PING"}), 1)
    check(len(errs) == 1, f"seq as string: {errs}")
    error(errs[0], "WRONG_TYPE", seq=None, field="seq")

    s = ctx.next_seq()
    error(ctx.frame_for(encode({"type": "COMMAND", "seq": s, "cmd": 5}), s), "WRONG_TYPE", s, "cmd")
    s = ctx.next_seq()
    error(ctx.frame_for(encode({"type": "ACK", "seq": s, "cmd": "PING"}), s),
          "INVALID_MESSAGE_TYPE", s, "type")
    s = ctx.next_seq()
    error(ctx.frame_for(encode({"type": 1, "seq": s, "cmd": "PING"}), s), "WRONG_TYPE", s, "type")


def t07_malformed_number(ctx):
    ctx.stop()
    ctx.settle_error_limit()
    cases = [("150abc", "MALFORMED_NUMBER"), ("1.5", "MALFORMED_NUMBER"),
             ("1e3", "MALFORMED_NUMBER"), ("01", "MALFORMED_NUMBER"),
             ("-", "MALFORMED_NUMBER"), ("0x10", "MALFORMED_NUMBER"),
             ("--1", "MALFORMED_NUMBER"), ("+5", "INVALID_MESSAGE")]
    sent = []
    data = b""
    for literal, _ in cases:
        s = ctx.next_seq()
        sent.append(s)
        data += encode('{"type":"COMMAND","seq":%d,"cmd":"DRIVE","left":%s,"right":0}'
                       % (s, literal))
    errs, start = ctx.unsolicited_errors(data, len(cases))
    check([e["reason"] for e in errs] == [r for _, r in cases],
          f"reasons {[e['reason'] for e in errs]}")
    acks = [r.frame for r in ctx.link.frames(start, ftype="ACK") if r.frame["seq"] in sent]
    check(not acks, f"malformed numbers produced ACKs: {acks}")
    check(ctx.telemetry()["left_cmd"] == 0, "malformed number reached the motor command")


def t08_out_of_range(ctx):
    ctx.stop()
    ctx.last_tel = ctx.telemetry()
    for l, r, fld in ((999999, 0, "left"), (256, 0, "left"), (-256, 0, "left"),
                      (0, 99999999999, "right"), (2147483648, 0, "left"),
                      (0, -2147483649, "right")):
        ack(ctx.cmd("DRIVE", left=l, right=r), result="REJECTED",
            reason="OUT_OF_RANGE", field=fld)
    check(ctx.telemetry()["left_cmd"] == 0, "out-of-range value reached the motor command")
    expect_motion(ctx, ctx.cmd("DRIVE", left=255, right=-255), "DRIVE", 255, -255)
    ctx.stop()


def t09_crc(ctx):
    ctx.stop()
    ctx.settle_error_limit()
    s = ctx.next_seq()
    payload = b'{"type":"COMMAND","seq":%d,"cmd":"DRIVE","left":150,"right":150}' % s
    good_crc = crc16(payload)

    errs, _ = ctx.unsolicited_errors(payload + b"*%04X\n" % (good_crc ^ 0x0001), 1)
    check(len(errs) == 1, f"bad CRC: {errs}")
    error(errs[0], "INVALID_CRC", seq=None)

    # One corrupted digit (a single-bit flip 1 -> 9) with the ORIGINAL CRC.
    corrupted = payload.replace(b'"left":150', b'"left":950')
    errs, _ = ctx.unsolicited_errors(corrupted + b"*%04X\n" % good_crc, 1)
    check(len(errs) == 1, f"corrupted payload: {errs}")
    error(errs[0], "INVALID_CRC", seq=None)
    check(ctx.telemetry()["left_cmd"] == 0, "corrupted frame reached the motor command")

    errs, _ = ctx.unsolicited_errors(payload + b"\n", 1)
    error(errs[0], "INVALID_FRAME", seq=None)
    errs, _ = ctx.unsolicited_errors(payload + b"*%03X\n" % (good_crc & 0xFFF), 1)
    error(errs[0], "INVALID_FRAME", seq=None)

    s = ctx.next_seq()
    p = b'{"type":"COMMAND","seq":%d,"cmd":"PING"}' % s
    ack(ctx.frame_for(p + b"*%04x\n" % crc16(p), s), result="ACCEPTED")    # lowercase hex


def t10_partial_frame(ctx):
    s = ctx.next_seq()
    data = command(s, "PING")
    start = ctx.link.mark()
    ctx.link.send(data[:17])
    time.sleep(0.3)
    check(ctx.link.response(s, start, 0.05) is None, "responded to a partial frame")
    check(not [r for r in ctx.link.frames(start, ftype="ERROR")], "partial frame produced ERROR")
    ctx.link.send(data[17:])
    rec = ctx.link.response(s, start, 1.0)
    check(rec is not None, "no ACK after the frame was completed")
    ack(rec.frame, result="ACCEPTED")

    s = ctx.next_seq()
    data = command(s, "PING")
    start = ctx.link.mark()
    for i in range(len(data)):
        ctx.link.send(data[i:i + 1])
        time.sleep(0.003)
    rec = ctx.link.response(s, start, 1.0)
    check(rec is not None, "byte-by-byte frame not accepted")

    a, b = ctx.next_seq(), ctx.next_seq()
    start = ctx.link.mark()
    ctx.link.send(command(a, "PING") + command(b, "PING"))
    check(ctx.link.response(a, start) and ctx.link.response(b, start),
          "two frames in one write")


def t11_oversized_frame(ctx):
    ctx.settle_error_limit()
    errs, _ = ctx.unsolicited_errors(b"A" * 400 + b"\n", 1)
    check(len(errs) == 1, f"expected exactly one FRAME_TOO_LONG, got {errs}")
    error(errs[0], "FRAME_TOO_LONG", seq=None)
    ack(ctx.cmd("PING"), result="ACCEPTED")

    # Boundary: a frame of exactly LINE_MAX bytes is accepted, LINE_MAX+1 is not.
    def padded(seq, total):
        base = '{"type":"COMMAND","seq":%d,"cmd":"PING"' % seq
        pad = total - len(base) - 1 - 5
        return encode(base + " " * pad + "}")

    s = ctx.next_seq()
    frame = padded(s, LINE_MAX)
    check(len(frame) == LINE_MAX + 1, "test construction")
    ack(ctx.frame_for(frame, s), result="ACCEPTED")

    s = ctx.next_seq()
    errs, start = ctx.unsolicited_errors(padded(s, LINE_MAX + 1), 1)
    error(errs[0], "FRAME_TOO_LONG", seq=None)
    check(ctx.link.response(s, start, 0.2) is None, "oversized frame was executed")


def t12_garbage(ctx):
    ctx.stop()
    ctx.settle_error_limit()
    s1, s2, s3 = ctx.next_seq(), ctx.next_seq(), ctx.next_seq()
    drive = lambda s: command(s, "DRIVE", left=200, right=200)
    lines = [
        b"hello world\n",
        b"\x00\x01\xfe\xff\n",
        b'{"type":"COMMAND","seq":%d,"cmd":"DRIVE","left":200,"right":200}\n' % s1,
        b"xx" + drive(s2),
        drive(s3)[:-1] + b"zz\n",
        b"rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)\r\n",
    ]
    errs, start = ctx.unsolicited_errors(b"".join(lines) + b"\r\n", len(lines))
    check(len(errs) == len(lines), f"expected {len(lines)} errors, got {len(errs)}: {errs}")
    check(all(e["reason"] == "INVALID_FRAME" for e in errs), f"reasons: {errs}")
    acks = [r.frame for r in ctx.link.frames(start, ftype="ACK")
            if r.frame["seq"] in (s1, s2, s3)]
    check(not acks, f"garbage produced an ACK: {acks}")
    check(ctx.telemetry()["left_cmd"] == 0, "embedded DRIVE in garbage was executed")


def t13_duplicate(ctx):
    ctx.last_tel = ctx.telemetry()
    s = ctx.next_seq()
    data = command(s, "DRIVE", left=120, right=120)
    first = ctx.frame_for(data, s)
    expect_motion(ctx, first, "DRIVE", 120, 120)

    dup = ack(ctx.frame_for(data, s), result="DUPLICATE", reason="DUPLICATE_SEQ")
    check(dup["original_result"] == first["result"] and dup["original_cmd"] == "DRIVE",
          f"duplicate did not report the original: {dup}")

    # Same seq, different content: still a duplicate, and NOT executed.
    dup2 = ack(ctx.frame_for(command(s, "STOP"), s), result="DUPLICATE")
    check(dup2["cmd"] == "STOP" and dup2["original_cmd"] == "DRIVE", f"{dup2}")
    check(ctx.telemetry()["left_cmd"] == 120, "a duplicate seq was executed")
    ctx.stop()

    # Replay of an OLDER frame after another command: stale, not executed.
    stale = ack(ctx.frame_for(data, s), result="REJECTED", reason="STALE_SEQ")
    check(stale["last_seq"] == ctx.seq, f"stale ack must report last_seq: {stale}")
    check(ctx.telemetry()["left_cmd"] == 0, "a stale replay was executed")

    s = ctx.next_seq()
    mt = command(s, "MOTORTEST", motor=0, power=120, ms=600, onblocks=True)
    if not ctx.last_tel["motor_drive_available"]:
        ack(ctx.frame_for(mt, s), result="REJECTED", reason="MOTOR_PWM_UNAVAILABLE")
        d = ack(ctx.frame_for(mt, s), result="DUPLICATE")
        check(d["original_result"] == "REJECTED", f"{d}")
        return

    # Drive available (simulator only): a duplicate must NOT restart the timer.
    start = ctx.link.mark()
    t0 = ctx.link.send(mt)
    ack(ctx.link.response(s, start).frame, result="ACCEPTED")
    time.sleep(0.3)
    ack(ctx.frame_for(mt, s), result="DUPLICATE")
    done = ctx.link.wait(lambda f: f["type"] == "EVENT" and f.get("event") == "MOTORTEST_DONE"
                         and f.get("seq") == s, start, 1.5)
    check(done is not None, "no MOTORTEST_DONE")
    elapsed = done.t - t0
    check(done.frame["reason"] == "EXPIRED", f"{done.frame}")
    check(0.5 <= elapsed <= 0.8, f"MOTORTEST ran {elapsed:.3f}s; a duplicate restarted it?")
    ctx.stats["motortest_elapsed_s"] = round(elapsed, 3)

    # And a replay AFTER another command must not start it again.
    ctx.cmd("PING")
    start = ctx.link.mark()
    ack(ctx.frame_for(mt, s), result="REJECTED", reason="STALE_SEQ")
    check(ctx.telemetry()["state"] != "MOTOR_TEST", "stale MOTORTEST replay started a test")


def t14_sequence_wrap(ctx):
    ctx.settle_error_limit()
    # Walk forward to 65535 in legal steps (each at most the 32767 window).
    cur = ctx.seq
    while cur != 65535:
        cur = min(cur + 30000, 65535)
        ack(ctx.cmd("PING", seq=cur), result="ACCEPTED")
    ack(ctx.cmd("PING", seq=1), result="ACCEPTED")                    # wraps to 1
    f = ack(ctx.cmd("PING", seq=65535), result="REJECTED", reason="STALE_SEQ")
    check(f["last_seq"] == 1, f"{f}")
    ack(ctx.cmd("PING", seq=1), result="DUPLICATE")
    ack(ctx.cmd("PING", seq=1 + 32768), result="REJECTED", reason="STALE_SEQ")   # too far
    ack(ctx.cmd("PING", seq=1 + 32767), result="ACCEPTED")                       # window edge
    ctx.seq = 1 + 32767

    for bad in (0, 65536, -1, 4294967296):
        errs, _ = ctx.unsolicited_errors(encode('{"type":"COMMAND","seq":%d,"cmd":"PING"}' % bad), 1)
        check(len(errs) == 1, f"seq {bad}: {errs}")
        error(errs[0], "INVALID_SEQUENCE", seq=None, field="seq")


def t15_ping(ctx):
    f = ack(ctx.cmd("PING"), cmd="PING", result="ACCEPTED", reason="NONE")
    check(isinstance(f.get("state"), str) and isinstance(f.get("uptime_ms"), int)
          and f.get("proto") == rl.PROTO_VERSION, f"PING fields: {f}")
    ack(ctx.cmd("PING", x=1), result="REJECTED", reason="UNKNOWN_FIELD", field="x")


def t16_reset(ctx):
    ack(ctx.cmd("RESET"), cmd="RESET", result="ACCEPTED")
    tel = ctx.telemetry()
    check(tel["safety_stop"] is False and tel["left_cmd"] == 0, f"after RESET: {tel}")
    ack(ctx.cmd("RESET", all=True), result="REJECTED", reason="UNKNOWN_FIELD")


def t17_watchdog(ctx):
    """Only a validated movement command refreshes the 2 s failsafe."""
    s0 = ctx.next_seq()
    stop_frame = command(s0, "STOP")
    start = ctx.link.mark()
    t0 = ctx.link.send(stop_frame)
    check(ctx.link.response(s0, start) is not None, "no ACK for STOP")

    s = ctx.next_seq()
    bad_crc = command(s, "STOP")[:-5] + b"0000\n"
    noise = [
        lambda: ctx.cmd("PING"),
        lambda: ctx.cmd("I2CSTATUS"),
        lambda: ctx.link.send(bad_crc),
        lambda: ctx.cmd("FLY"),
        lambda: ctx.cmd("DRIVE", left=999, right=0),
        lambda: ctx.cmd("DRIVE", left=100),
        lambda: ctx.link.send(b"garbage\n"),
        lambda: ctx.frame_for(stop_frame, s0),                 # stale STOP replay
        lambda: ctx.cmd("MOTORTEST", motor=0, power=100, ms=100, onblocks=False),
        lambda: ctx.cmd("TOFTEST"),
    ]
    # Every input type goes out between 1.0 s and ~1.9 s after the STOP. If ANY
    # of them refreshed the watchdog, the timeout would move to >= 3.0 s and
    # the 2.35 s bound below would fail.
    time.sleep(1.0)
    for fn in noise:
        fn()
        time.sleep(0.07)
    check(time.monotonic() - t0 < 1.95, "noise did not fit inside the window")

    ev = ctx.link.wait(lambda f: f["type"] == "EVENT" and f.get("event") == "COMMAND_TIMEOUT",
                       start, 1.5)
    check(ev is not None, "no COMMAND_TIMEOUT: something refreshed the watchdog")
    dt = ev.t - t0
    check(1.9 <= dt <= 2.35, f"COMMAND_TIMEOUT after {dt:.3f}s, expected ~2.0s")
    check(ev.frame["command_age_ms"] >= TIMEOUT_MS, f"{ev.frame}")
    check(ctx.telemetry()["command_timeout"] is True, "telemetry command_timeout")
    ctx.stats["watchdog_fired_after_s"] = round(dt, 3)

    # Control: a stream of valid DRIVE commands (gated or not) keeps it alive.
    start = ctx.link.mark()
    t1 = time.monotonic()
    while time.monotonic() - t1 < 3.0:
        ctx.cmd("DRIVE", left=0, right=0)
        time.sleep(0.5)
    ev = ctx.link.wait(lambda f: f["type"] == "EVENT" and f.get("event") == "COMMAND_TIMEOUT",
                       start, 0.01)
    check(ev is None, "COMMAND_TIMEOUT fired while valid DRIVE commands were arriving")
    ctx.stop()


def t18_telemetry_size(ctx):
    start = ctx.link.mark()
    time.sleep(4.3)
    end = ctx.link.mark()
    fast = ctx.link.frames(start, end, "TELEMETRY")
    diag = ctx.link.frames(start, end, "DIAG")
    check(len(fast) >= 18, f"only {len(fast)} TELEMETRY frames in 4.3 s")

    sizes = [len(r.raw) for r in fast]
    gaps = [(b.t - a.t) * 1000 for a, b in zip(fast, fast[1:])]
    by_section = {}
    for r in diag:
        by_section.setdefault(r.frame["section"], []).append(len(r.raw))
    check(set(by_section) == {"FRONT", "REAR", "SYSTEM"}, f"DIAG sections: {set(by_section)}")

    span = fast[-1].t - fast[0].t
    total_bytes = sum(len(r.raw) for r in ctx.link.frames(start, end))
    fast_max = max(sizes)
    fast_pct = rl.wire_ms(fast_max) / 200.0 * 100
    check(fast_pct < 50.0, f"fast telemetry uses {fast_pct:.1f}% of its 200 ms slot")

    required = {"uptime_ms", "state", "block_reason", "last_seq", "last_reject",
                "left_cmd", "right_cmd", "left_applied", "right_applied",
                "front_left_cm", "front_right_cm", "front_valid", "front_obstacle",
                "rear_mm", "forward_blocked", "reverse_blocked", "safety_stop",
                "command_age_ms", "command_timeout", "motor_drive_available"}
    check(required <= set(fast[-1].frame), f"missing: {required - set(fast[-1].frame)}")

    ctx.stats.update({
        "fast_bytes_min": min(sizes), "fast_bytes_max": fast_max,
        "fast_wire_ms_max": round(rl.wire_ms(fast_max), 1),
        "fast_pct_of_200ms": round(fast_pct, 1),
        "fast_interval_ms_mean": round(statistics.mean(gaps), 1),
        "diag_bytes": {k: max(v) for k, v in sorted(by_section.items())},
        "diag_wire_ms": {k: round(rl.wire_ms(max(v)), 1) for k, v in sorted(by_section.items())},
        "measured_link_util_pct": round(total_bytes * rl.BITS_PER_BYTE / rl.BAUD / span * 100, 1),
    })


def t19_burst(ctx):
    seqs = [ctx.next_seq() for _ in range(20)]
    kinds = ["PING", "DRIVE", "FLY", "I2CSTATUS"]
    data = b""
    for i, s in enumerate(seqs):
        k = kinds[i % len(kinds)]
        data += command(s, k, left=0, right=0) if k == "DRIVE" else command(s, k)
    start = ctx.link.mark()
    ctx.link.send(data)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        got = [r.frame["seq"] for r in ctx.link.frames(start, ftype="ACK") if r.frame["seq"] in seqs]
        if len(got) >= len(seqs):
            break
        time.sleep(0.05)
    got = [r.frame for r in ctx.link.frames(start, ftype="ACK") if r.frame["seq"] in seqs]
    check([f["seq"] for f in got] == seqs, f"ACK order/set wrong: {[f['seq'] for f in got]}")
    check(all(f["cmd"] == kinds[i % len(kinds)] for i, f in enumerate(got)), "cmd echo")
    ctx.stop()


def t20_ack_correlation(ctx):
    plan = []
    for i in range(40):
        s = ctx.next_seq()
        k = i % 5
        if k == 0:
            plan.append((s, "PING", command(s, "PING"), "ACCEPTED"))
        elif k == 1:
            plan.append((s, "DRIVE", command(s, "DRIVE", left=0, right=0), None))
        elif k == 2:
            plan.append((s, "DRIVE", command(s, "DRIVE", left=500, right=0), "REJECTED"))
        elif k == 3:
            plan.append((s, "NOPE", command(s, "NOPE"), "REJECTED"))
        else:
            plan.append((s, "STOP", command(s, "STOP"), "ACCEPTED"))

    start = ctx.link.mark()
    for i in range(0, len(plan), 2):             # two in flight at a time
        for s, _, data, _ in plan[i:i + 2]:
            ctx.link.send(data)
        time.sleep(0.03)
    time.sleep(1.0)

    sent = {s for s, _, _, _ in plan}
    responses = [r.frame for r in ctx.link.frames(start)
                 if r.frame["type"] in ("ACK", "ERROR")]
    check(all(f["seq"] in sent for f in responses),
          f"response for an unsent seq: {[f for f in responses if f['seq'] not in sent]}")
    for s, name, _, result in plan:
        mine = [f for f in responses if f["seq"] == s]
        check(len(mine) == 1, f"seq {s}: {len(mine)} responses")
        check(mine[0]["cmd"] == name, f"seq {s}: cmd {mine[0]['cmd']} != {name}")
        if result:
            check(mine[0]["result"] == result, f"seq {s}: {mine[0]}")
    interleaved = [r for r in ctx.link.frames(start) if r.frame["type"] in ("TELEMETRY", "DIAG")]
    ctx.stats["correlation_frames_interleaved"] = len(interleaved)


def t21_error_rate_limit(ctx):
    ctx.settle_error_limit()
    before = ctx.diag("SYSTEM")["link"]["errors_suppressed"]
    errs, _ = ctx.unsolicited_errors(b"noise\n" * 25, ERR_PER_SEC, window=0.8)
    check(len(errs) == ERR_PER_SEC, f"{len(errs)} ERROR frames for 25 noise lines")
    after = ctx.diag("SYSTEM")["link"]["errors_suppressed"]
    check(after - before == 25 - ERR_PER_SEC, f"suppressed count {after - before}")


def t22_ready_and_noise(ctx):
    items = ctx.link.items
    first = next((r for r in items if r.frame is not None), None)
    if first is None or first.frame.get("event") != "READY":
        raise Skip("READY not captured (link opened after boot)")
    f = first.frame
    check(f["type"] == "EVENT" and f["proto"] == rl.PROTO_VERSION and f["seq"] is None, f"{f}")
    noise = [r for r in items if r.frame is None and items.index(r) < items.index(first)]
    ctx.stats["pre_ready_noise_lines"] = len(noise)


def t23_motortest(ctx):
    tel = ctx.telemetry()
    if not tel["motor_drive_available"]:
        ack(ctx.cmd("MOTORTEST", motor=0, power=120, ms=300, onblocks=True),
            result="REJECTED", reason="MOTOR_PWM_UNAVAILABLE")
    ack(ctx.cmd("MOTORTEST", motor=0, power=120, ms=300, onblocks=False),
        result="REJECTED", reason="ONBLOCKS_REQUIRED", field="onblocks")
    ack(ctx.cmd("MOTORTEST", motor=4, power=120, ms=300, onblocks=True),
        result="REJECTED", reason="OUT_OF_RANGE", field="motor")
    ack(ctx.cmd("MOTORTEST", motor=0, power=120, ms=0, onblocks=True),
        result="REJECTED", reason="OUT_OF_RANGE", field="ms")
    ack(ctx.cmd("MOTORTEST", motor=0, power=120, ms=3001, onblocks=True),
        result="REJECTED", reason="OUT_OF_RANGE", field="ms")
    ack(ctx.cmd("MOTORTEST", motor=0, power=120, ms=300, onblocks=True, ch=1),
        result="REJECTED", reason="UNKNOWN_FIELD", field="ch")
    if not tel["motor_drive_available"]:
        return

    s = ctx.next_seq()
    start = ctx.link.mark()
    ack(ctx.cmd("MOTORTEST", seq=s, motor=1, power=120, ms=2000, onblocks=True), result="ACCEPTED")
    check(ctx.telemetry()["state"] == "MOTOR_TEST", "state during MOTORTEST")
    ctx.cmd("PING")
    done = ctx.link.wait(lambda f: f["type"] == "EVENT" and f.get("event") == "MOTORTEST_DONE"
                         and f.get("seq") == s, start, 1.0)
    check(done is not None and done.frame["reason"] == "CANCELLED",
          "another command must cancel a running MOTORTEST")


def t24_diagnostics(ctx):
    f = ack(ctx.cmd("I2CSCAN"), result="ACCEPTED")
    check(isinstance(f.get("devices"), list) and "count" in f, f"{f}")
    ack(ctx.cmd("I2CSTATUS"), result="ACCEPTED")
    ack(ctx.cmd("HWREPORT"), result="ACCEPTED")
    ack(ctx.cmd("TCATEST"), result="REJECTED", reason="NO_ADDR_GIVEN_AND_NONE_CONFIRMED")
    ack(ctx.cmd("TCATEST", addr=200), result="REJECTED", reason="OUT_OF_RANGE", field="addr")
    ack(ctx.cmd("TCATEST", addr="0x70"), result="REJECTED", reason="WRONG_TYPE", field="addr")
    ack(ctx.cmd("TOFTEST", sensor=3), result="REJECTED", reason="OUT_OF_RANGE", field="sensor")
    if not ctx.physical:
        f = ack(ctx.cmd("TCATEST", addr=0x70), result="ACCEPTED")
        check(f["looks_like_tca9548a"] is True, f"simulated TCA at 0x70: {f}")
        ack(ctx.cmd("PCATEST", addr=0x40), result="ACCEPTED")


TESTS = [
    ("01 valid DRIVE", t01_valid_drive),
    ("02 valid STOP", t02_stop),
    ("03 valid MOVE", t03_move),
    ("04 unknown command", t04_unknown_command),
    ("05 missing field", t05_missing_field),
    ("06 wrong field type", t06_wrong_type),
    ("07 malformed number", t07_malformed_number),
    ("08 out-of-range number", t08_out_of_range),
    ("09 invalid CRC", t09_crc),
    ("10 partial frame", t10_partial_frame),
    ("11 oversized frame", t11_oversized_frame),
    ("12 garbage input", t12_garbage),
    ("13 duplicate sequence", t13_duplicate),
    ("14 sequence wrap", t14_sequence_wrap),
    ("15 PING", t15_ping),
    ("16 RESET", t16_reset),
    ("17 watchdog refresh rules", t17_watchdog),
    ("18 telemetry size", t18_telemetry_size),
    ("19 burst of commands", t19_burst),
    ("20 ACK correlation", t20_ack_correlation),
    ("21 ERROR rate limit", t21_error_rate_limit),
    ("22 READY / boot noise", t22_ready_and_noise),
    ("23 MOTORTEST validation", t23_motortest),
    ("24 diagnostics", t24_diagnostics),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--sim", help="path to the host simulator executable")
    g.add_argument("--port", help="serial port of the real ESP32 UART")
    ap.add_argument("--sim-env", action="append", default=[], help="KEY=VALUE for the simulator")
    ap.add_argument("--allow-motion", action="store_true",
                    help="run even if motor_drive_available is true (ROVER ON BLOCKS)")
    ap.add_argument("--only", help="comma-separated test number prefixes, e.g. 01,17")
    args = ap.parse_args()

    if args.sim:
        env = dict(kv.split("=", 1) for kv in args.sim_env)
        transport = rl.SimTransport(args.sim, env)
        label = "SIM " + (" ".join(args.sim_env) or "(default)")
    else:
        transport = rl.SerialTransport(args.port)
        label = "UART " + args.port
    link = rl.Link(transport)
    ctx = Ctx(link, physical=bool(args.port))

    try:
        # Let boot output settle and front sensors fill their windows.
        time.sleep(1.5)
        try:
            tel = ctx.telemetry(timeout=2.0)
        except Fail:
            print("No TELEMETRY frame received -- is the ESP32 running protocol v2?")
            return 2
        if ctx.physical and tel["motor_drive_available"] and not args.allow_motion:
            print("REFUSING: motor_drive_available is true. Put the rover on blocks "
                  "and pass --allow-motion.")
            return 2
        ctx.last_tel = tel
        # Resynchronise: continue from the last seq the ESP32 acknowledged, so a
        # rerun against an ESP32 that is still up is not rejected as stale.
        if tel.get("last_seq") is not None:
            ctx.seq = tel["last_seq"]

        only = args.only.split(",") if args.only else None
        results = []
        print(f"=== {label} | drive_available={tel['motor_drive_available']} "
              f"forward_blocked={tel['forward_blocked']} ===")
        for name, fn in TESTS:
            if only and not any(name.startswith(o) for o in only):
                continue
            t = time.monotonic()
            try:
                fn(ctx)
                status, detail = "PASS", ""
            except Skip as e:
                status, detail = "SKIP", str(e)
            except Fail as e:
                status, detail = "FAIL", str(e)
            results.append((name, status))
            print(f"  {status}  {name:28s} {time.monotonic() - t:5.2f}s  {detail}")

        # Every line received after READY must be a valid frame.
        items = link.items
        ready = next((i for i, r in enumerate(items) if r.frame and r.frame.get("event") == "READY"), 0)
        invalid = [r.raw for r in items[ready:] if r.frame is None]
        print(f"  lines received: {len(items)}, invalid after READY: {len(invalid)}")
        if invalid:
            results.append(("frame validity", "FAIL"))
            print("  FAIL  invalid lines:", invalid[:3])

        if ctx.latencies:
            lat = sorted(x * 1000 for x in ctx.latencies)
            print(f"  ACK round trip over this transport (ms): median {statistics.median(lat):.1f}, "
                  f"p95 {lat[int(len(lat) * 0.95)]:.1f}, max {lat[-1]:.1f}  (n={len(lat)})")
        for k, v in ctx.stats.items():
            print(f"  {k}: {v}")

        failed = [n for n, s in results if s == "FAIL"]
        print(f"=== {sum(s == 'PASS' for _, s in results)} passed, {len(failed)} failed, "
              f"{sum(s == 'SKIP' for _, s in results)} skipped ===")
        return 1 if failed else 0
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
