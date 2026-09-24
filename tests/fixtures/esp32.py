"""ESP32 link test fixtures: real frames, a fake serial port and a fake clock.

The REAL_* lines were captured from the physical ESP32 (firmware 82f2a8a,
protocol v2) on /dev/ttyAMA0 on 2026-09-25, byte for byte, CRC included. They
are the ground truth the decoder is tested against; everything else here is
built with the protocol's own CRC.
"""

from __future__ import annotations

import json
import time
from collections import deque
from typing import Any, Callable, Dict, List, Optional, Sequence, Union

from robotx.control.motion import MotionIntent
from robotx.control.safety import SafetyDecision, SafetyVerdict
from robotx.esp32.link import Esp32Config, Esp32Link
from robotx.esp32.protocol import crc16

REAL_TELEMETRY = (
    b'{"type":"TELEMETRY","uptime_ms":3588443,"state":"DRIVE_UNAVAILABLE","block_reason":'
    b'"MOTOR_PWM_UNAVAILABLE","last_seq":32768,"last_reject":"INVALID_FRAME","left_cmd":0,'
    b'"right_cmd":0,"left_applied":0,"right_applied":0,"front_left_cm":37.9,"front_right_cm"'
    b':38.0,"front_valid":true,"front_obstacle":true,"front_warning":true,"rear_mm":[null,null'
    b',null],"rear_available":false,"rear_obstacle":false,"rear_sensor_fault":false,'
    b'"forward_blocked":true,"reverse_blocked":false,"safety_stop":false,"command_age_ms":'
    b'3588419,"command_timeout":false,"motor_drive_available":false}*1501\n'
)
REAL_DIAG_REAR = (
    b'{"type":"DIAG","section":"REAR","uptime_ms":3588743,"rear_backend":"UNCONFIGURED",'
    b'"rear_orientation_verified":false,"rear_sensor_valid":[false,false,false],'
    b'"rear_sensor_status":["UNINITIALISED","UNINITIALISED","UNINITIALISED"],'
    b'"rear_sensor_health":["FAULT","FAULT","FAULT"],"rear_sensor_obstacle":[false,false,false],'
    b'"rear_health":"FAULT","rear_closest_mm":null,"rear_warning":false,"rear_sensor_raw_mm":'
    b'[null,null,null],"rear_fail_streak":[0,0,0],"rear_tca_channels":[0,1,2]}*D907\n'
)
REAL_DIAG_SYSTEM = (
    b'{"type":"DIAG","section":"SYSTEM","uptime_ms":3589143,"proto":2,"i2c_ready":true,'
    b'"tca_status":"ADDRESS_UNCONFIRMED","pca_status":"ADDRESS_UNCONFIRMED",'
    b'"tca_address_confirmed":false,"pca_address_confirmed":false,"motor_drive_status":'
    b'"ADDRESS_UNCONFIRMED","motor_map_verified":false,"command_ever_received":false,'
    b'"left_gated":0,"right_gated":0,"link":{"rx_ok":13,"rx_empty":0,"rx_bad_frame":25,'
    b'"rx_bad_crc":0,"rx_bad_message":4,"rx_too_long":2,"rx_rejected":1,"rx_duplicates":1,'
    b'"rx_stale":2,"errors_suppressed":15,"tx_overflows":0}}*E00A\n'
)
REAL_DIAG_FRONT = (
    b'{"type":"DIAG","section":"FRONT","uptime_ms":3589543,"front_left_valid":true,'
    b'"front_right_valid":true,"front_left_health":"OK","front_right_health":"OK",'
    b'"front_health":"OK","front_closest_cm":37.0,"sensor_map_verified":false,'
    b'"front_left_raw_cm":37.9,"front_right_raw_cm":37.7,"front_left_samples":5,'
    b'"front_right_samples":5,"front_left_timeout_streak":0,"front_right_timeout_streak":0}*63A6\n'
)
# The first line actually read after opening mid-stream (bit-misaligned).
REAL_PARTIAL_ON_OPEN = bytes.fromhex("9d89e9d1c9d595b189c99585c97db5b589e96db9d5b1b1b1") + b"\n"
# ESP32 boot ROM text, as PROTOCOL.md section 12 quotes it.
ROM_LINE = b"rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)\r\n"


def payload_of(line: bytes) -> Dict[str, Any]:
    return json.loads(line[: line.rindex(b"*")])


REAL_TELEMETRY_FIELDS = payload_of(REAL_TELEMETRY)


def frame(obj: Union[Dict[str, Any], str, bytes], *, crc: Optional[int] = None) -> bytes:
    """Frame an arbitrary payload with a valid (or the given) CRC."""

    if isinstance(obj, dict):
        obj = json.dumps(obj, separators=(",", ":"))
    if isinstance(obj, str):
        obj = obj.encode("ascii")
    return obj + b"*%04X\n" % (crc16(obj) if crc is None else crc)


def telemetry(**overrides: Any) -> bytes:
    data = dict(REAL_TELEMETRY_FIELDS)
    data.update(overrides)
    return frame(data)


def ack(seq: int, cmd: str = "PING", result: str = "ACCEPTED", reason: str = "NONE",
        **extra: Any) -> bytes:
    data = {"type": "ACK", "seq": seq, "cmd": cmd, "result": result, "reason": reason}
    if cmd == "PING" and result == "ACCEPTED":
        data.update({"state": "DRIVE_UNAVAILABLE", "uptime_ms": 1000, "proto": 2})
    data.update(extra)
    return frame(data)


def error(reason: str, seq: Optional[int] = None, **extra: Any) -> bytes:
    data = {"type": "ERROR", "seq": seq, "reason": reason}
    data.update(extra)
    return frame(data)


def event(name: str, uptime_ms: int = 100, **extra: Any) -> bytes:
    data = {"type": "EVENT", "event": name, "seq": None, "uptime_ms": uptime_ms}
    data.update(extra)
    return frame(data)


def ready(uptime_ms: int = 120) -> bytes:
    return event("READY", uptime_ms, proto=2, fw="82f2a8a", core=1, seq_min=1,
                 seq_max=65535, line_max=160)


class FakeClock:
    def __init__(self, start: float = 1000.0) -> None:
        self.t = start

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class FakePort:
    """An in-memory serial port. Fails loudly if used after close."""

    def __init__(self, *chunks: bytes, block_s: float = 0.0) -> None:
        self.inbox = deque(chunks)
        self.writes: List[bytes] = []
        self.closed = False
        self.block_s = block_s
        self.fail_read: Optional[BaseException] = None
        self.fail_write: Optional[BaseException] = None
        self.reads = 0

    def feed(self, *chunks: bytes) -> None:
        self.inbox.extend(chunks)

    def read(self, size: int) -> bytes:
        if self.closed:
            raise AssertionError("read() after close()")
        self.reads += 1
        if self.fail_read is not None:
            raise self.fail_read
        if self.inbox:
            return self.inbox.popleft()
        if self.block_s:
            time.sleep(self.block_s)
        return b""

    def write(self, data: bytes) -> int:
        if self.closed:
            raise AssertionError("write() after close()")
        if self.fail_write is not None:
            raise self.fail_write
        self.writes.append(bytes(data))
        return len(data)

    def close(self) -> None:
        self.closed = True

    def commands(self) -> List[Dict[str, Any]]:
        """Every COMMAND frame written, decoded. The resync LF is skipped."""

        out = []
        for w in self.writes:
            if w == b"\n":
                continue
            out.append(payload_of(w))
        return out

    def command_names(self) -> List[str]:
        return [c["cmd"] for c in self.commands()]


def make_link(*ports: Union[FakePort, BaseException], clock: Optional[FakeClock] = None,
              wall: Callable[[], float] = time.time, **cfg: Any):
    """An Esp32Link over fake ports, returned by the factory in order."""

    clock = clock or FakeClock()
    queue = deque(ports or (FakePort(),))

    def factory():
        item = queue.popleft() if queue else OSError("no more fake ports")
        if isinstance(item, BaseException):
            raise item
        return item

    link = Esp32Link(Esp32Config(**cfg), port_factory=factory, clock=clock, wall=wall)
    return link, clock


def bring_up(link: Esp32Link, port: FakePort, clock: FakeClock) -> int:
    """Drive a fresh link to UP: TELEMETRY, then answer its PING. Returns the PING seq."""

    link.poll_once()                   # open + resync LF
    port.feed(REAL_TELEMETRY)
    link.poll_once()                   # telemetry -> PING
    ping = [c for c in port.commands() if c["cmd"] == "PING"][-1]
    port.feed(ack(ping["seq"]))
    link.poll_once()
    return ping["seq"]


def decision(intent: MotionIntent, verdict: SafetyVerdict = SafetyVerdict.ALLOWED) -> SafetyDecision:
    return SafetyDecision(intent=intent, verdict=verdict, rule="test", reason="test")
