"""Pi <-> ESP32 link protocol v2 -- reference helpers for the test suite.

Framing, CRC and parsing exactly as PROTOCOL.md defines them, plus two
transports with the same interface:

    SimTransport     the host simulator (tests/host), over stdin/stdout
    SerialTransport  a real UART, via pyserial

This module is also a usable starting point for the Pi-side implementation.
"""

import json
import os
import re
import subprocess
import threading
import time

PROTO_VERSION = 2
SEQ_MIN = 1
SEQ_MAX = 65535
BAUD = 115200
BITS_PER_BYTE = 10          # 8N1: start + 8 data + stop


# ---------------------------------------------------------------------------
# CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, not reflected, xorout 0.
# ---------------------------------------------------------------------------
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


assert crc16(b"123456789") == 0x29B1


def encode(payload) -> bytes:
    """Frame a payload (dict, str or bytes): PAYLOAD*XXXX\\n."""
    if isinstance(payload, dict):
        payload = json.dumps(payload, separators=(",", ":"))
    if isinstance(payload, str):
        payload = payload.encode("ascii")
    return payload + b"*%04X\n" % crc16(payload)


def command(seq, cmd, **fields) -> bytes:
    msg = {"type": "COMMAND", "seq": seq, "cmd": cmd}
    msg.update(fields)
    return encode(msg)


_FRAME_RE = re.compile(rb"^(\{.*\})\*([0-9A-Fa-f]{4})$")


def decode(line: bytes):
    """A received line -> dict, or None if it is not a valid v2 frame.

    Boot ROM text, partial lines and corrupted frames all return None. The Pi
    must discard those, never interpret them.
    """
    line = line.rstrip(b"\n")
    if line.endswith(b"\r"):
        line = line[:-1]
    m = _FRAME_RE.match(line)
    if not m or int(m.group(2), 16) != crc16(m.group(1)):
        return None
    try:
        obj = json.loads(m.group(1))
    except ValueError:
        return None
    if not isinstance(obj, dict) or not isinstance(obj.get("type"), str):
        return None
    return obj


def wire_ms(nbytes: int) -> float:
    return nbytes * BITS_PER_BYTE * 1000.0 / BAUD


# ---------------------------------------------------------------------------
# Transports
# ---------------------------------------------------------------------------
class SimTransport:
    """The host simulator as a subprocess. env: SIM_* overrides."""

    def __init__(self, exe, env=None):
        full_env = dict(os.environ)
        full_env.update(env or {})
        self.p = subprocess.Popen([exe], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, env=full_env, bufsize=0)

    def write(self, data: bytes):
        self.p.stdin.write(data)
        self.p.stdin.flush()

    def readline(self) -> bytes:
        return self.p.stdout.readline()

    def close(self):
        try:
            self.p.stdin.close()
            self.p.wait(timeout=3)
        except Exception:
            self.p.kill()


class SerialTransport:
    """A real UART. On the Pi 5 this is typically /dev/ttyAMA0 (GPIO14/15)."""

    def __init__(self, port, baud=BAUD):
        import serial  # pyserial
        self.s = serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1,
                               timeout=0.2, xonxoff=False, rtscts=False)
        self._buf = b""
        self._open = True

    def write(self, data: bytes):
        self.s.write(data)
        self.s.flush()

    def readline(self) -> bytes:
        while self._open:
            chunk = self.s.read_until(b"\n")
            if not chunk:
                continue
            self._buf += chunk
            if self._buf.endswith(b"\n"):
                line, self._buf = self._buf, b""
                return line
        return b""

    def close(self):
        self._open = False
        self.s.close()


# ---------------------------------------------------------------------------
# Link: a background reader that timestamps every received line
# ---------------------------------------------------------------------------
class Received:
    __slots__ = ("t", "raw", "frame")

    def __init__(self, t, raw, frame):
        self.t = t
        self.raw = raw
        self.frame = frame


class Link:
    def __init__(self, transport):
        self.transport = transport
        self.items = []
        self.cv = threading.Condition()
        self.bytes_sent = 0
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        while True:
            line = self.transport.readline()
            if not line:
                return
            rec = Received(time.monotonic(), line, decode(line))
            with self.cv:
                self.items.append(rec)
                self.cv.notify_all()

    def send(self, data: bytes) -> float:
        t = time.monotonic()
        self.transport.write(data)
        self.bytes_sent += len(data)
        return t

    def mark(self) -> int:
        with self.cv:
            return len(self.items)

    def wait(self, pred, start=0, timeout=1.0):
        """First received item at index >= start whose frame satisfies pred."""
        deadline = time.monotonic() + timeout
        idx = start
        with self.cv:
            while True:
                while idx < len(self.items):
                    rec = self.items[idx]
                    idx += 1
                    if rec.frame is not None and pred(rec.frame):
                        return rec
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.cv.wait(remaining)

    def response(self, seq, start, timeout=1.0):
        """The ACK or ERROR answering `seq`."""
        return self.wait(lambda f: f["type"] in ("ACK", "ERROR") and f.get("seq") == seq,
                         start, timeout)

    def frames(self, start=0, end=None, ftype=None):
        with self.cv:
            items = self.items[start:end]
        return [r for r in items if r.frame is not None and
                (ftype is None or r.frame["type"] == ftype)]

    def close(self):
        self.transport.close()
