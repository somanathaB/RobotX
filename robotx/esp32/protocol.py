"""ESP32 link protocol v2: framing, CRC, strict decoding and command encoding.

Pure functions and values only: no serial port, no threads, no clock. The wire
contract is PROTOCOL.md; `rover_link.py` beside it is the reference
implementation, and `tests/unit/test_esp32_protocol.py` cross-checks this
module against it.

Why this module exists rather than importing `rover_link`:

- `rover_link` is a test helper at the repository root, pinned by hash as
  reference material, not a package module;
- its `decode()` collapses every failure into `None`, so a CRC error and boot
  ROM text are indistinguishable, and it accepts duplicate keys, NaN and lines
  of any length;
- its `encode()` will frame floats and escaped strings, which the ESP32 rejects.

Inbound and outbound rules differ, and that is the protocol, not a shortcut.
The strict flat grammar of PROTOCOL.md section 4 and the 160-byte line limit
apply to *commands*. Frames from the ESP32 carry floats (`front_left_cm`),
arrays (`rear_mm`) and one nested object (DIAG SYSTEM `link`), and run to
~1.4 KB, so the inbound decoder checks framing, CRC and each type's documented
fields instead.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from enum import Enum
from types import MappingProxyType
from typing import Any, Dict, List, Mapping, Optional, Tuple, Union

PROTO_VERSION = 2
BAUD = 115200
SEQ_MIN = 1
SEQ_MAX = 65535

# PROTOCOL.md section 2: a command line longer than this (before LF) is
# discarded by the ESP32. Nothing this module encodes may exceed it.
COMMAND_LINE_MAX = 160

# The Pi's own bound on an inbound line. The ESP32 cannot emit a frame larger
# than its 2560-byte TX frame buffer (section 10); anything longer is not a
# frame, and buffering it would only let line noise grow memory.
INBOUND_LINE_MAX = 4096

# Section 12: a lone LF clears any partial line in the ESP32's buffer.
RESYNC = b"\n"

MESSAGE_TYPES = frozenset({"ACK", "ERROR", "EVENT", "TELEMETRY", "DIAG"})
ACK_RESULTS = frozenset({"ACCEPTED", "GATED", "REJECTED", "DUPLICATE"})
DIAG_SECTIONS = frozenset({"FRONT", "REAR", "SYSTEM"})


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


# ---------------------------------------------------------------------------
# Decoding
# ---------------------------------------------------------------------------
class FrameError(str, Enum):
    """Why a received line is not a usable frame."""

    EMPTY = "EMPTY"                # blank line or CR only: a resync, ignored
    TOO_LONG = "TOO_LONG"          # longer than INBOUND_LINE_MAX
    NOT_ASCII = "NOT_ASCII"        # a byte outside 0x20-0x7E
    NOT_A_FRAME = "NOT_A_FRAME"    # no "{...}*XXXX" shape (boot ROM text, noise)
    BAD_CRC = "BAD_CRC"
    BAD_JSON = "BAD_JSON"          # not JSON, not an object, duplicate key, NaN
    UNKNOWN_TYPE = "UNKNOWN_TYPE"  # "type" missing or not one the ESP32 sends
    BAD_FIELDS = "BAD_FIELDS"      # a documented field missing or mistyped


@dataclass(frozen=True)
class Frame:
    """One valid frame from the ESP32. `data` is read-only."""

    type: str
    data: Mapping[str, Any]
    size: int


@dataclass(frozen=True)
class Rejected:
    """A received line that is not a valid frame, and why."""

    reason: FrameError
    detail: str = ""
    size: int = 0


_TRAILER = re.compile(rb"^(\{.*\})\*([0-9A-Fa-f]{4})$")
_PRINTABLE = bytes(range(0x20, 0x7F))


class _DuplicateKey(ValueError):
    pass


def _no_duplicates(pairs: List[Tuple[str, Any]]) -> Dict[str, Any]:
    obj: Dict[str, Any] = {}
    for key, value in pairs:
        if key in obj:
            raise _DuplicateKey(f"duplicate key {key!r}")
        obj[key] = value
    return obj


def _no_constants(name: str) -> Any:
    raise ValueError(f"non-finite number {name}")


# Documented fields per type: (key, kind). Only fields PROTOCOL.md (and the
# reference suite's required-field list) says are always present are required;
# optional documented fields are type-checked when present.
_INT, _OPT_INT, _STR, _BOOL, _OPT_NUM, _LIST = "int", "int?", "str", "bool", "num?", "list"

_REQUIRED: Dict[str, Tuple[Tuple[str, str], ...]] = {
    "ACK": (("seq", _INT), ("cmd", _STR), ("result", _STR), ("reason", _STR)),
    "ERROR": (("seq", _OPT_INT), ("reason", _STR)),
    "EVENT": (("event", _STR), ("seq", _OPT_INT), ("uptime_ms", _INT)),
    # The same twenty keys test_link.py t18 requires of real telemetry.
    "TELEMETRY": (
        ("uptime_ms", _INT), ("state", _STR), ("block_reason", _STR),
        ("last_seq", _OPT_INT), ("last_reject", _STR),
        ("left_cmd", _INT), ("right_cmd", _INT),
        ("left_applied", _INT), ("right_applied", _INT),
        ("front_left_cm", _OPT_NUM), ("front_right_cm", _OPT_NUM),
        ("front_valid", _BOOL), ("front_obstacle", _BOOL), ("rear_mm", _LIST),
        ("forward_blocked", _BOOL), ("reverse_blocked", _BOOL), ("safety_stop", _BOOL),
        ("command_age_ms", _INT), ("command_timeout", _BOOL),
        ("motor_drive_available", _BOOL),
    ),
    "DIAG": (("section", _STR), ("uptime_ms", _INT)),
}

_OPTIONAL: Dict[str, Tuple[Tuple[str, str], ...]] = {
    "TELEMETRY": (
        ("front_warning", _BOOL), ("rear_available", _BOOL),
        ("rear_obstacle", _BOOL), ("rear_sensor_fault", _BOOL),
    ),
}


def _is_int(v: Any) -> bool:
    # bool is an int subclass in Python; JSON true is not an integer.
    return isinstance(v, int) and not isinstance(v, bool)


def _kind_ok(kind: str, v: Any) -> bool:
    if kind == _INT:
        return _is_int(v)
    if kind == _OPT_INT:
        return v is None or _is_int(v)
    if kind == _STR:
        return isinstance(v, str)
    if kind == _BOOL:
        return isinstance(v, bool)
    if kind == _OPT_NUM:
        return v is None or (isinstance(v, (int, float)) and not isinstance(v, bool))
    if kind == _LIST:
        return isinstance(v, list)
    raise AssertionError(kind)


def _schema_problem(ftype: str, obj: Mapping[str, Any]) -> Optional[str]:
    for key, kind in _REQUIRED[ftype]:
        if key not in obj:
            return f"{ftype} missing {key}"
        if not _kind_ok(kind, obj[key]):
            return f"{ftype} {key} is not {kind}: {obj[key]!r}"
    for key, kind in _OPTIONAL.get(ftype, ()):
        if key in obj and not _kind_ok(kind, obj[key]):
            return f"{ftype} {key} is not {kind}: {obj[key]!r}"
    if ftype == "ACK" and obj["result"] not in ACK_RESULTS:
        return f"ACK result {obj['result']!r} is not documented"
    if ftype == "DIAG" and obj["section"] not in DIAG_SECTIONS:
        return f"DIAG section {obj['section']!r} is not documented"
    return None


def decode_line(line: bytes) -> Union[Frame, Rejected]:
    """One received line (with or without its LF) -> Frame, or Rejected.

    Never raises for bad input: every way a line can be wrong is a `Rejected`
    with a reason, so the caller can count them separately. Boot ROM text is
    `NOT_A_FRAME`, exactly as it should be.
    """

    size = len(line)
    body = line[:-1] if line.endswith(b"\n") else line
    if body.endswith(b"\r"):
        body = body[:-1]
    if not body:
        return Rejected(FrameError.EMPTY, size=size)
    if len(body) > INBOUND_LINE_MAX:
        return Rejected(FrameError.TOO_LONG, f"{len(body)} bytes", size)
    if body.translate(None, _PRINTABLE):
        return Rejected(FrameError.NOT_ASCII, size=size)

    m = _TRAILER.match(body)
    if m is None:
        return Rejected(FrameError.NOT_A_FRAME, size=size)
    payload, crc_text = m.group(1), m.group(2)
    if int(crc_text, 16) != crc16(payload):
        return Rejected(FrameError.BAD_CRC, crc_text.decode("ascii"), size)

    try:
        obj = json.loads(
            payload.decode("ascii"),
            object_pairs_hook=_no_duplicates,
            parse_constant=_no_constants,
        )
    except ValueError as e:
        return Rejected(FrameError.BAD_JSON, str(e), size)
    if not isinstance(obj, dict):
        return Rejected(FrameError.BAD_JSON, "not an object", size)

    ftype = obj.get("type")
    if not isinstance(ftype, str) or ftype not in MESSAGE_TYPES:
        return Rejected(FrameError.UNKNOWN_TYPE, repr(ftype), size)

    problem = _schema_problem(ftype, obj)
    if problem is not None:
        return Rejected(FrameError.BAD_FIELDS, problem, size)

    return Frame(ftype, MappingProxyType(obj), size)


# ---------------------------------------------------------------------------
# Encoding
# ---------------------------------------------------------------------------
class CommandError(ValueError):
    """A command this link refuses to encode."""


# The only commands the Pi sends, with their fields in PROTOCOL.md section 6
# order and inclusive ranges. MOVE, MOTORTEST, RESET and the diagnostics are
# deliberately absent: RobotX expresses motion as left/right (DRIVE), MOTORTEST
# bypasses the ESP32 safety gate, and RESET clears safety latches -- an
# operator decision, not something the link does on its own.
ALLOWED_COMMANDS: Mapping[str, Tuple[Tuple[str, int, int], ...]] = MappingProxyType({
    "PING": (),
    "STOP": (),
    "DRIVE": (("left", -255, 255), ("right", -255, 255)),
})


def encode_command(seq: int, cmd: str, **fields: int) -> bytes:
    """Encode one COMMAND frame, refusing anything the protocol would reject.

    The payload is built by hand rather than with `json.dumps` so that it can
    only ever contain the envelope, the command name and range-checked
    integers: no floats, no escapes, no key order surprises.
    """

    if not _is_int(seq) or not SEQ_MIN <= seq <= SEQ_MAX:
        raise CommandError(f"seq must be an integer {SEQ_MIN}..{SEQ_MAX}, got {seq!r}")
    spec = ALLOWED_COMMANDS.get(cmd)
    if spec is None:
        raise CommandError(f"{cmd!r} is not a command this link sends")
    expected = [name for name, _, _ in spec]
    if sorted(fields) != sorted(expected):
        raise CommandError(f"{cmd} takes fields {expected}, got {sorted(fields)}")

    parts = [f'{{"type":"COMMAND","seq":{seq},"cmd":"{cmd}"']
    for name, low, high in spec:
        value = fields[name]
        if not _is_int(value) or not low <= value <= high:
            raise CommandError(f"{cmd} {name} must be an integer {low}..{high}, got {value!r}")
        parts.append(f',"{name}":{value}')
    parts.append("}")

    payload = "".join(parts).encode("ascii")
    frame = payload + b"*%04X\n" % crc16(payload)
    if len(frame) - 1 > COMMAND_LINE_MAX:
        raise CommandError(f"{cmd} frame is {len(frame) - 1} bytes (limit {COMMAND_LINE_MAX})")
    return frame


class SequenceCounter:
    """The Pi's command sequence (section 8): 1..65535, then wraps to 1.

    Starts from 1. `resync()` continues from the ESP32's own `last_seq`, which
    is how a Pi restart or an ESP32 reboot is survived without STALE_SEQ.
    """

    def __init__(self) -> None:
        self._last: Optional[int] = None

    @property
    def last(self) -> Optional[int]:
        return self._last

    def next(self) -> int:
        if self._last is None or self._last >= SEQ_MAX:
            self._last = SEQ_MIN
        else:
            self._last += 1
        return self._last

    def resync(self, last_acked: Optional[int]) -> None:
        """Continue after `last_acked`; from 1 if it is None or invalid."""

        if (
            isinstance(last_acked, int)
            and not isinstance(last_acked, bool)
            and SEQ_MIN <= last_acked <= SEQ_MAX
        ):
            self._last = last_acked
        else:
            self._last = None
