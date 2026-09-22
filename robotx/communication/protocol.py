"""The robot <-> backend wire contract, in one place.

Scope and honesty
-----------------
This module defines what the Pi puts on the wire and what it will accept off
it. It is deliberately the *only* place that knows event names and payload
shapes -- nothing else in the agent may emit or parse a backend message.

Two very different classes of fact live here, and they are kept apart:

1. **Derived from the backend data model.** Field names and enum values
   (`robotId`, `lat`, `lon`, `speed`, `battery`, `STOP`/`PAUSE`/`RETURN`/
   `RESUME`, `SENT`/`ACK`/`FAILED`, `INFO`/`WARNING`/`CRITICAL`) come from the
   FalconAut Prisma models. These are used verbatim, including camelCase.

2. **NOT derivable: the Socket.IO envelope.** Event names, the namespace, the
   handshake `auth` shape and whether the server uses Socket.IO callback acks
   are transport decisions that live in backend source. That source is **not
   present in this repository or on this machine** (verified: no backend tree,
   no simulator, no `schema.prisma` file, nothing serving Socket.IO locally).

Because of (2), every transport-level name is a `ProtocolBinding` value rather
than a literal, and the built-in binding is marked `PROVISIONAL`. Its names are
inherited from the client that already existed in this repository; they were
never verified against a server. Drop the real contract in with
`ROBOTX_PROTOCOL_FILE=/path/binding.json` and no Python changes at all.

A `PROVISIONAL` binding is reported as such in state, telemetry and logs, and
the agent must never describe itself as integrated while one is in use.

Data-model fields the Pi deliberately does NOT send
---------------------------------------------------
`Robot.id`, `socketId`, `isOnline`, `lastSeenAt`, `simulated`, `locationId`,
`campusId`, `zoneId`, `currentTaskId`, and every `createdAt`/`issuedAt`. These
are **backend-owned**: they are assigned by the server from the connection it
can see and the clock it trusts. A robot asserting its own `isOnline` is a
robot that can lie about being alive, which is precisely the failure mode this
integration exists to prevent. The Pi states what it measures; the backend
decides what that means.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple

from robotx.diagnostics.health import HealthStatus
from robotx.state.robot_state import OperatingMode, RobotSnapshot


# Bumped only on a breaking change to what this module puts on the wire. Sent
# so a backend can tell which Pi build produced a payload.
WIRE_SCHEMA_VERSION = 1

# Refuse to parse anything larger than this. A Socket.IO peer can send an
# arbitrarily large payload; the agent loop must not be asked to walk it.
MAX_INBOUND_PAYLOAD_BYTES = 64 * 1024


# --- enums taken from the backend data model ----------------------------------


class CommandType(str, Enum):
    """`Command.type`. These four exist in the backend enum; there are no others.

    Adding a fifth here would be inventing backend protocol.
    """

    STOP = "STOP"
    PAUSE = "PAUSE"
    RETURN = "RETURN"
    RESUME = "RESUME"


class CommandStatus(str, Enum):
    """`Command.status`.

    The backend enum has exactly three values, and notably **no terminal
    "executed" state distinct from `ACK`**. So `ACK` has to carry the meaning
    "this robot has applied the command", not merely "received the bytes" --
    reporting `ACK` on receipt would leave no way to ever say it was done. The
    Pi therefore acks *after* applying the command, and `executedAt` is the
    instant it was applied. See `docs/communication/ROBOT_BACKEND_PROTOCOL.md`.
    """

    SENT = "SENT"      # set by the backend when it issues; never sent by the Pi
    ACK = "ACK"        # applied by the robot
    FAILED = "FAILED"  # rejected or could not be applied


class EventLevel(str, Enum):
    """`Event.type`."""

    INFO = "INFO"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"


# --- the transport binding ----------------------------------------------------


class BindingSource(str, Enum):
    """Where a binding's event names came from, i.e. how much to trust them."""

    PROVISIONAL = "PROVISIONAL"  # built-in guess; inherited from legacy client
    FILE = "FILE"                # operator supplied via ROBOTX_PROTOCOL_FILE
    EXPLICIT = "EXPLICIT"        # constructed in code (tests, harnesses)


@dataclass(frozen=True)
class ProtocolBinding:
    """Event names and namespace: the part that is backend source, not ours.

    Every field is a name on the wire. None of them can be checked from this
    repository, which is why they are data rather than literals scattered
    through the client.
    """

    source: BindingSource = BindingSource.PROVISIONAL
    namespace: str = "/robot"

    # Pi -> backend
    register: str = "robot_hello"
    telemetry: str = "telemetry"
    status: str = "status"
    event: str = "event"
    command_result: str = "command_ack"

    # backend -> Pi
    command: str = "command"
    registered: str = "registered"  # optional server confirmation of register

    @property
    def is_provisional(self) -> bool:
        """True when the event names were never sourced from the backend.

        Anything that reports integration status must consult this. A link that
        connects over a provisional binding has proved that a Socket.IO server
        accepted a TCP connection -- nothing more.
        """

        return self.source is BindingSource.PROVISIONAL

    def outbound_events(self) -> Tuple[str, ...]:
        return (self.register, self.telemetry, self.status, self.event, self.command_result)

    def inbound_events(self) -> Tuple[str, ...]:
        return (self.command, self.registered)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "source": self.source.value,
            "provisional": self.is_provisional,
            "namespace": self.namespace,
            "outbound": {
                "register": self.register,
                "telemetry": self.telemetry,
                "status": self.status,
                "event": self.event,
                "command_result": self.command_result,
            },
            "inbound": {"command": self.command, "registered": self.registered},
        }

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any], *, source: BindingSource) -> "ProtocolBinding":
        """Build from a flat or nested mapping, ignoring unknown keys.

        Accepts both the nested shape `to_dict()` produces and a flat one, so a
        hand-written binding file does not have to guess which we wanted.
        """

        flat: Dict[str, Any] = {}
        for key, value in data.items():
            if key in ("outbound", "inbound") and isinstance(value, Mapping):
                flat.update({k: v for k, v in value.items()})
            else:
                flat[key] = value

        known = {"namespace", "register", "telemetry", "status", "event",
                 "command_result", "command", "registered"}
        kwargs = {k: str(v) for k, v in flat.items() if k in known and v is not None}
        return replace(cls(source=source), source=source, **kwargs)

    @classmethod
    def load(cls, path: Optional[str]) -> "ProtocolBinding":
        """Load an operator-supplied binding, or return the provisional one.

        A malformed or missing file is a configuration error worth failing on:
        silently falling back to guessed event names would be the worst of both
        worlds -- the operator believes the real contract is in force while the
        Pi talks to nobody.
        """

        if not path:
            return cls()

        raw = Path(path).read_text()
        data = json.loads(raw)
        if not isinstance(data, dict):
            raise ValueError(f"protocol binding {path!r} must contain a JSON object")
        return cls.from_mapping(data, source=BindingSource.FILE)


# --- outbound payloads --------------------------------------------------------


@dataclass(frozen=True)
class TelemetryFrame:
    """A telemetry payload, or an explicit refusal to produce one.

    Refusing is a first-class outcome. When the robot has no trustworthy
    position there is no honest `lat`/`lon` to send, and the correct behaviour
    is to send nothing and say why -- not to resend the last fix, which would
    tell a dashboard the robot is parked where it was ten minutes ago.
    """

    payload: Optional[Dict[str, Any]]
    skipped_reason: Optional[str] = None

    @property
    def sendable(self) -> bool:
        return self.payload is not None


def _position_age_s(snapshot: RobotSnapshot, now: float) -> Optional[float]:
    if snapshot.position is None:
        return None
    return max(0.0, now - snapshot.position.timestamp)


def build_telemetry_payload(
    snapshot: RobotSnapshot,
    *,
    robot_id: str,
    max_position_age_s: float,
    now: Optional[float] = None,
) -> TelemetryFrame:
    """Core high-frequency telemetry: `Telemetry` model fields only.

    Maps 1:1 onto the backend `Telemetry` row (`robotId`, `lat`, `lon`,
    `speed`, `battery`) plus the Pi's own capture timestamp. Health,
    navigation, perception and diagnostics are deliberately absent -- they go
    on the low-rate status channel, because a row per detection at camera rate
    is how you fill a database with data nobody reads.

    `battery` is `null`. This robot has no battery-sensing hardware at all (see
    `robotx.hardware.battery`), and a plausible-looking number is worse than a
    null: an operator cannot tell an invented 76% from a measured one. Whether
    the backend's `battery` column tolerates null is an open question recorded
    as a blocker; the Pi will not resolve it by making a value up.
    """

    now = time.time() if now is None else now

    # `has_fix` rather than a status comparison, so this module needs nothing
    # from the GPS hardware package -- the reading carries its own verdict.
    if not snapshot.gps.has_fix or snapshot.position is None:
        return TelemetryFrame(None, f"no usable GPS fix (status={snapshot.gps.status.value})")

    age = _position_age_s(snapshot, now)
    if age is not None and age > max_position_age_s:
        return TelemetryFrame(None, f"position is {age:.1f}s old (limit {max_position_age_s:.1f}s)")

    position = snapshot.position
    return TelemetryFrame(
        {
            "schemaVersion": WIRE_SCHEMA_VERSION,
            "robotId": robot_id,
            "lat": round(position.latitude, 7),
            "lon": round(position.longitude, 7),
            # Speed over ground from the receiver. Null rather than 0.0 when the
            # receiver did not report it: "not measured" is not "stationary".
            "speed": None if position.speed_mps is None else round(position.speed_mps, 3),
            "battery": None,
            # The Pi's capture time. The backend owns `createdAt`; this exists
            # so it can detect a delayed or replayed frame.
            "capturedAt": round(position.timestamp, 3),
            "positionAgeS": None if age is None else round(age, 3),
        }
    )


def build_status_payload(
    snapshot: RobotSnapshot,
    *,
    robot_id: str,
    binding: ProtocolBinding,
    now: Optional[float] = None,
) -> Dict[str, Any]:
    """Low-rate robot state: what `Robot.status`/`battery` should reflect.

    Carries no `isOnline`, `socketId` or `lastSeenAt`: the backend derives
    those from the connection it is holding, which is the only party that can
    observe them truthfully.

    Every subsystem block states its own freshness, so a consumer can tell
    "perception says the path is clear" from "perception has not run in a
    minute" without having to know how the Pi is built.
    """

    now = time.time() if now is None else now
    health = snapshot.health

    return {
        "schemaVersion": WIRE_SCHEMA_VERSION,
        "robotId": robot_id,
        # `Robot.status`: the agent's operating mode, the one authoritative
        # answer to "what is this robot doing".
        "status": snapshot.mode.value,
        "battery": None,
        "reportedAt": round(now, 3),
        "uptimeS": round(snapshot.uptime_s, 1),
        "health": {
            "status": health.status.value,
            "components": {
                name: component.status.value for name, component in health.components.items()
            },
        },
        "subsystems": {
            "gps": snapshot.gps.status.value,
            "perception": snapshot.perception.status.value,
            "navigation": snapshot.navigation.status.value,
            "camera": health.components["camera"].status.value
            if "camera" in health.components
            else HealthStatus.UNKNOWN.value,
        },
        "position": _status_position_block(snapshot, now),
        "motionIntent": {
            "command": snapshot.motion_intent.command.value,
            "reason": snapshot.motion_intent.reason,
        },
        "navigation": {
            "status": snapshot.navigation.status.value,
            "waypointIndex": snapshot.navigation.waypoint_index,
            "waypointsTotal": snapshot.navigation.waypoints_total,
            "progress": round(snapshot.navigation.progress, 3),
        },
        "lastError": snapshot.last_error,
        "protocol": {"provisional": binding.is_provisional, "source": binding.source.value},
    }


def _status_position_block(snapshot: RobotSnapshot, now: float) -> Optional[Dict[str, Any]]:
    """Last known position, explicitly labelled with its age.

    The status channel *may* carry a stale position where telemetry may not,
    because here it is tagged with `ageS` and `isFresh` and cannot be mistaken
    for a live one. A dashboard can show a last-known marker greyed out; it
    could not do that from a bare lat/lon.
    """

    if snapshot.position is None:
        return None
    age = max(0.0, now - snapshot.position.timestamp)
    return {
        "lat": round(snapshot.position.latitude, 7),
        "lon": round(snapshot.position.longitude, 7),
        "ageS": round(age, 2),
        "isFresh": snapshot.gps.has_fix,
        "headingDeg": snapshot.position.heading_deg,
        "headingSource": snapshot.position.heading_source.value,
        "satellites": snapshot.position.satellites,
    }


def build_event_payload(
    *,
    robot_id: str,
    level: EventLevel,
    message: str,
    task_id: Optional[str] = None,
    now: Optional[float] = None,
) -> Dict[str, Any]:
    """An `Event` row: robotId, optional taskId, type, message.

    `taskId` is included only when the Pi actually holds one. The Pi never
    invents a task association, and never sends `createdAt` -- the backend's
    clock is the one that orders events across robots.
    """

    payload: Dict[str, Any] = {
        "schemaVersion": WIRE_SCHEMA_VERSION,
        "robotId": robot_id,
        "type": level.value,
        "message": message[:500],
        "reportedAt": round(time.time() if now is None else now, 3),
    }
    if task_id:
        payload["taskId"] = task_id
    return payload


def build_command_result_payload(
    *,
    robot_id: str,
    command_id: str,
    status: CommandStatus,
    reason: str = "",
    executed_at: Optional[float] = None,
) -> Dict[str, Any]:
    """A `Command` status update: ACK or FAILED, with the instant it happened.

    `executedAt` is set for both outcomes. For `ACK` it is when the intent was
    applied; for `FAILED` it is when the attempt was concluded. A `FAILED`
    always carries a `reason` -- a rejection an operator cannot explain is one
    they will retry blindly.
    """

    if status is CommandStatus.SENT:
        raise ValueError("SENT is assigned by the backend; a robot never reports it")

    return {
        "schemaVersion": WIRE_SCHEMA_VERSION,
        "robotId": robot_id,
        "commandId": command_id,
        "status": status.value,
        "executedAt": round(time.time() if executed_at is None else executed_at, 3),
        "reason": reason,
    }


def build_register_payload(
    *,
    robot_id: str,
    binding: ProtocolBinding,
    agent_version: str,
    capabilities: Mapping[str, Any],
) -> Dict[str, Any]:
    """Identity announced after connect.

    `simulated: false` is the one `Robot` column the Pi is entitled to assert:
    it is a fact about what kind of agent this process is, known here and
    nowhere else. Everything else about the robot's record -- where it is
    assigned, what task it holds -- belongs to the backend.

    No token appears here. The credential travels in the connect handshake and
    must never be repeated in an event payload, where it would be logged by any
    server with event logging switched on.
    """

    return {
        "schemaVersion": WIRE_SCHEMA_VERSION,
        "robotId": robot_id,
        "simulated": False,
        "agent": {"name": "robotx-pi", "version": agent_version},
        "capabilities": dict(capabilities),
        "protocolSource": binding.source.value,
    }


def agent_capabilities() -> Dict[str, Any]:
    """What this physical robot can and cannot do, stated once.

    Sent at registration so a dashboard does not have to infer from silence
    why a field is always null. Everything here is a property of the hardware
    that exists today, not of what is planned.
    """

    return {
        "telemetry": True,
        "commands": [c.value for c in CommandType],
        # No fuel gauge, ADC or divider on this robot.
        "battery": False,
        "gps": True,
        "camera": True,
        # Motor authority belongs to the ESP32; that link does not exist yet,
        # so this robot cannot move at all, whatever it is commanded.
        "motion": False,
    }


# --- inbound parsing ----------------------------------------------------------


class RejectionReason(str, Enum):
    """Why an inbound command was refused. Reported back as `FAILED.reason`."""

    MALFORMED = "MALFORMED"            # not an object, or unreadable fields
    OVERSIZED = "OVERSIZED"            # larger than MAX_INBOUND_PAYLOAD_BYTES
    UNKNOWN_TYPE = "UNKNOWN_TYPE"      # not one of the four backend commands
    MISSING_ID = "MISSING_ID"          # no commandId to acknowledge against
    WRONG_ROBOT = "WRONG_ROBOT"        # addressed to a different robotId
    STALE = "STALE"                    # issued too long ago to act on


@dataclass(frozen=True)
class CommandRejection:
    """A command that will not be executed, and what to tell the backend."""

    reason: RejectionReason
    detail: str
    command_id: Optional[str] = None

    @property
    def is_ackable(self) -> bool:
        """Whether a `FAILED` can be reported for this.

        Without a `commandId` there is no row to update, so an ack would be
        undeliverable. Those are logged locally instead of emitted into the
        void.
        """

        return bool(self.command_id)


@dataclass(frozen=True)
class InboundCommand:
    """A validated backend command, ready to be turned into an agent intent."""

    command_id: str
    type: CommandType
    issued_at: Optional[float]
    received_at: float
    raw: Dict[str, Any]

    def age_s(self, now: Optional[float] = None) -> Optional[float]:
        if self.issued_at is None:
            return None
        return max(0.0, (time.time() if now is None else now) - self.issued_at)


def parse_timestamp(value: Any) -> Optional[float]:
    """Best-effort unix seconds from a JSON timestamp, or None.

    Prisma `DateTime` reaches JSON as ISO-8601, but a Socket.IO layer may
    serialize it as epoch milliseconds or seconds instead. Which one this
    backend does is unverified, so all three are accepted and an unparseable
    value yields None -- treated as "no issue time", never as "now", which
    would silently make every stale command look fresh.
    """

    if value is None:
        return None
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        number = float(value)
        # Heuristic split: epoch seconds passed 1e11 in the year 5138, so
        # anything above it is milliseconds.
        return number / 1000.0 if number > 1e11 else number
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return None
        try:
            from datetime import datetime

            return datetime.fromisoformat(text.replace("Z", "+00:00")).timestamp()
        except ValueError:
            try:
                return parse_timestamp(float(text))
            except ValueError:
                return None
    return None


def parse_command(
    data: Any,
    *,
    expected_robot_id: str,
    max_age_s: Optional[float] = None,
    now: Optional[float] = None,
) -> Any:
    """Validate one inbound command payload.

    Returns an `InboundCommand` or a `CommandRejection`. Never raises: this
    runs on a socket callback fed by a remote party, and an exception there
    would take out the event loop's handler for every subsequent command.

    A command naming a different `robotId` is rejected outright. Routing is the
    server's job, but a robot that executes anything it happens to receive is
    one misconfigured room away from obeying another robot's stop.
    """

    now = time.time() if now is None else now

    if not isinstance(data, dict):
        return CommandRejection(RejectionReason.MALFORMED, f"expected an object, got {type(data).__name__}")

    try:
        encoded = len(json.dumps(data, default=str))
    except (TypeError, ValueError):
        return CommandRejection(RejectionReason.MALFORMED, "payload is not JSON-serializable")
    if encoded > MAX_INBOUND_PAYLOAD_BYTES:
        return CommandRejection(RejectionReason.OVERSIZED, f"{encoded} bytes exceeds {MAX_INBOUND_PAYLOAD_BYTES}")

    command_id = data.get("commandId") or data.get("id")
    command_id = str(command_id).strip() if command_id is not None else ""

    raw_type = data.get("type")
    if raw_type is None:
        return CommandRejection(RejectionReason.MALFORMED, "no `type` field", command_id or None)
    try:
        command_type = CommandType(str(raw_type).strip().upper())
    except ValueError:
        return CommandRejection(
            RejectionReason.UNKNOWN_TYPE,
            f"{str(raw_type)[:40]!r} is not one of {[c.value for c in CommandType]}",
            command_id or None,
        )

    if not command_id:
        return CommandRejection(
            RejectionReason.MISSING_ID,
            f"{command_type.value} carries no commandId; nothing to acknowledge against",
        )

    target = data.get("robotId")
    if target is not None and str(target).strip() != expected_robot_id:
        return CommandRejection(
            RejectionReason.WRONG_ROBOT,
            f"addressed to {str(target)[:40]!r}, this robot is {expected_robot_id!r}",
            command_id,
        )

    issued_at = parse_timestamp(data.get("issuedAt") or data.get("createdAt"))
    if max_age_s is not None and issued_at is not None:
        age = now - issued_at
        if age > max_age_s:
            return CommandRejection(
                RejectionReason.STALE,
                f"issued {age:.1f}s ago, limit is {max_age_s:.1f}s",
                command_id,
            )

    return InboundCommand(
        command_id=command_id,
        type=command_type,
        issued_at=issued_at,
        received_at=now,
        raw=data,
    )


# --- logging safety -----------------------------------------------------------


_SECRET_KEYS = {"token", "auth", "authorization", "password", "secret", "apikey", "api_key"}


def redact(payload: Any, *, _depth: int = 0) -> Any:
    """Copy a payload with credential-shaped values replaced.

    Used on every payload that reaches a log line. A token in a log file is a
    token in whatever ships that log file, and the one thing worse than an
    unauthenticated channel is one whose credential is in plain text on disk.
    """

    if _depth > 6:
        return "<truncated>"
    if isinstance(payload, dict):
        return {
            key: ("<redacted>" if str(key).lower() in _SECRET_KEYS else redact(value, _depth=_depth + 1))
            for key, value in payload.items()
        }
    if isinstance(payload, (list, tuple)):
        return [redact(item, _depth=_depth + 1) for item in payload[:20]]
    return payload


def mode_for_command(command_type: CommandType) -> Optional[OperatingMode]:
    """The operating mode a command is asking for, where it maps to one.

    `RETURN` is absent on purpose: it asks for a new route, not a new mode, and
    which mode it ends in depends on whether a home position is known.
    """

    return {
        CommandType.STOP: OperatingMode.STOPPED,
        CommandType.PAUSE: OperatingMode.PAUSED,
        CommandType.RESUME: OperatingMode.AUTO,
    }.get(command_type)
