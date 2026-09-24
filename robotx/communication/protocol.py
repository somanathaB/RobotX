"""The robot <-> backend wire contract, in one place.

Source of truth
---------------
`ROBOTX_PI_P2B1_HANDOFF.md` (Dashboard repository, 2026-09-24), itself derived
from `docs/contracts/PHYSICAL_ROBOTX_BACKEND_CONTRACT.md`. Every event name and
field here is taken from it; nothing is added that it does not define.

This module and `robotx.communication.engine` (the signed Assignment Engine
envelopes) are the only places that know event names and payload shapes --
nothing else in the agent may emit or parse a backend message.

Summary of the wire
-------------------
- Socket.IO v4, namespace `/`, anonymous connect, then `AUTH {robotId, token |
  pairingCode}` -> `AUTH_SUCCESS` and `AUTH_OK` (both arrive; one is handled).
  Refusal is a silent disconnect.
- `HEARTBEAT {}` every ~2 s, or `{commitmentId, fence}` while carrying out an
  accepted mission.
- `TELEMETRY {timestamp, sequence, status, lat?, lon?, speed?}`. Anything not
  measured is **omitted**, never nulled or zeroed.
- Operator `COMMAND {commandId, type, timestamp}` -> `COMMAND_ACK {commandId}`
  once applied. A bare `STOP` (task cancellation) expects no ack.
- Engine `command` envelopes (OFFER, WITHDRAW, ...) -> `COMMAND_ACK {outboxId,
  fence, authorityEpoch}`, then exactly one of `OFFER_ACCEPT` / `OFFER_REJECT`
  / `OFFER_DEFER`; `CUSTODY_EVENT`; `TASK_COMPLETE {taskId, lat, lon}`.
- `TASK_ASSIGN` is a recovery re-send, never a new assignment.

Fields the Pi deliberately does NOT send
----------------------------------------
`robotId` outside AUTH (identity is the authenticated socket), `isOnline`,
`lastSeenAt`, battery, e-stop and every other quantity this Rover cannot
measure. A key named like `capability`/`capabilities` would make the backend
drop the whole telemetry frame, and none is ever put in one.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple

from robotx.diagnostics.health import HealthStatus
from robotx.mission.mission import (
    MAX_PATH_POINTS,
    Mission,
    MissionRejected,
    MissionRejectReason,
)
from robotx.state.robot_state import OperatingMode, RobotSnapshot


# Bumped only on a breaking change to what this module puts on the wire. Sent
# so a backend can tell which Pi build produced a payload.
WIRE_SCHEMA_VERSION = 1

# Refuse to parse anything larger than this. A Socket.IO peer can send an
# arbitrarily large payload; the agent loop must not be asked to walk it.
MAX_INBOUND_PAYLOAD_BYTES = 64 * 1024


def now_ms(now: Optional[float] = None) -> int:
    """Epoch milliseconds, as an int.

    FalconAut treats an observation's `timestamp` as load-bearing: it is the
    instant the *robot* measured something, and the backend orders and ages
    observations by it. So it must be taken at the observation boundary, not at
    startup and not on the server.

    Milliseconds, not seconds, and an `int` rather than a float -- a fractional
    millisecond would be a different type on the wire than the backend
    declares, and JSON gives no way for the receiver to tell the difference
    between "1.5 ms of precision" and "someone sent seconds by mistake".
    """

    return int(round((time.time() if now is None else now) * 1000.0))


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

    FALCONAUT = "FALCONAUT"  # the FalconAut robot contract (built-in default)
    FILE = "FILE"            # operator supplied via ROBOTX_PROTOCOL_FILE
    EXPLICIT = "EXPLICIT"    # constructed in code (tests, harnesses)


# Binding fields whose names are not attested by the backend contract.
# Everything NOT in this set is. Surfaced by `describe()` and in the startup
# warning, so nobody has to read this file to learn which names are worth
# double-checking.
#
# `auth_failed` is the only one left: the contract describes an authentication
# failure as a **silent disconnect**, and never mentions an error event. The
# binding keeps one anyway, because a backend that does send a reason is
# strictly easier to debug than one that does not -- but nothing depends on it
# existing, and the silent-disconnect path is what is actually relied upon.
#
# Note this set is about *names*. The `TELEMETRY` and `HEARTBEAT` payload
# **field lists** are still unverified; see `build_heartbeat_payload`.
UNCONFIRMED_NAMES = ("auth_failed",)


@dataclass(frozen=True)
class ProtocolBinding:
    """FalconAut event names and namespace.

    Names are data rather than literals scattered through the client so that a
    single JSON file can correct any of them without a code change.
    """

    source: BindingSource = BindingSource.FALCONAUT
    # FalconAut's robot handler is on the default namespace. The previous
    # `/robot` namespace in this repository was never attested by anything.
    namespace: str = "/"

    # Pi -> backend (ROBOTX_PI_P2B1_HANDOFF.md)
    auth: str = "AUTH"
    telemetry: str = "TELEMETRY"
    task_complete: str = "TASK_COMPLETE"
    # Liveness, ~every 2 s, independent of whether the robot has a position.
    heartbeat: str = "HEARTBEAT"
    # A separate emitted event, never a Socket.IO callback acknowledgement.
    # Serves both the operator path ({commandId}) and the engine path
    # ({outboxId, fence, authorityEpoch}).
    command_ack: str = "COMMAND_ACK"
    offer_accept: str = "OFFER_ACCEPT"
    offer_reject: str = "OFFER_REJECT"
    offer_defer: str = "OFFER_DEFER"
    custody_event: str = "CUSTODY_EVENT"

    # Not part of the robot contract. Empty means "never emitted"; an operator
    # may bind them if the backend turns out to accept them.
    register: str = ""
    status: str = ""
    event: str = ""

    # backend -> Pi
    auth_success: str = "AUTH_SUCCESS"
    # The contract names two success events. Both are bound, because binding
    # only one means that if the backend picks the other the robot never
    # authenticates at all -- it waits out the auth timeout and reports a
    # failure that looks exactly like a refused credential.
    auth_ok: str = "AUTH_OK"
    auth_failed: str = "AUTH_FAILED"
    # Operator commands: {commandId, type, timestamp}.
    command: str = "COMMAND"
    # Signed Assignment Engine envelopes (OFFER, WITHDRAW, ...). Lower-case,
    # and a different event from the operator `COMMAND` above.
    engine_command: str = "command"
    # A bare STOP, distinct from COMMAND{type:STOP}: task cancellation. It
    # expects no acknowledgement.
    stop: str = "STOP"
    # Sent only by the backend's post-restart recovery sweep. Never starts a
    # mission on this Pi: see `BackendLink._on_task_assign`.
    task_assign: str = "TASK_ASSIGN"
    # The backend's verdict on a TASK_COMPLETE claim.
    task_complete_ack: str = "TASK_COMPLETE_ACK"

    @property
    def unconfirmed(self) -> Tuple[str, ...]:
        """Which of this binding's names are not literally attested.

        A `FILE` binding is an operator asserting the real names, so nothing in
        it is reported as unconfirmed.
        """

        if self.source is not BindingSource.FALCONAUT:
            return ()
        return UNCONFIRMED_NAMES

    def outbound_events(self) -> Tuple[str, ...]:
        names = (self.auth, self.telemetry, self.heartbeat, self.command_ack,
                 self.offer_accept, self.offer_reject, self.offer_defer,
                 self.custody_event, self.task_complete, self.register,
                 self.status, self.event)
        return tuple(name for name in names if name)

    def inbound_events(self) -> Tuple[str, ...]:
        names = (self.auth_success, self.auth_ok, self.auth_failed,
                 self.command, self.engine_command, self.stop, self.task_assign,
                 self.task_complete_ack)
        return tuple(name for name in names if name)

    def auth_success_events(self) -> Tuple[str, ...]:
        """Every event that means "you are authenticated", deduplicated.

        Deduplicated because binding both names to the same string -- which an
        operator correcting one of them could easily do -- would otherwise
        register two handlers for one event.
        """

        seen = []
        for name in (self.auth_success, self.auth_ok):
            if name and name not in seen:
                seen.append(name)
        return tuple(seen)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "source": self.source.value,
            "namespace": self.namespace,
            "unconfirmed": list(self.unconfirmed),
            "outbound": {
                "auth": self.auth,
                "telemetry": self.telemetry,
                "heartbeat": self.heartbeat,
                "command_ack": self.command_ack,
                "offer_accept": self.offer_accept,
                "offer_reject": self.offer_reject,
                "offer_defer": self.offer_defer,
                "custody_event": self.custody_event,
                "task_complete": self.task_complete,
                "register": self.register,
                "status": self.status,
                "event": self.event,
            },
            "inbound": {
                "auth_success": self.auth_success,
                "auth_ok": self.auth_ok,
                "auth_failed": self.auth_failed,
                "command": self.command,
                "engine_command": self.engine_command,
                "stop": self.stop,
                "task_assign": self.task_assign,
                "task_complete_ack": self.task_complete_ack,
            },
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

        known = {"namespace", "auth", "telemetry", "heartbeat", "command_ack",
                 "offer_accept", "offer_reject", "offer_defer", "custody_event",
                 "task_complete", "register", "status", "event", "auth_success",
                 "auth_ok", "auth_failed", "command", "engine_command", "stop",
                 "task_assign", "task_complete_ack"}
        kwargs = {k: str(v) for k, v in flat.items() if k in known and v is not None}
        return replace(cls(source=source), source=source, **kwargs)

    @classmethod
    def load(cls, path: Optional[str]) -> "ProtocolBinding":
        """Load an operator-supplied binding, or return the FalconAut default.

        A malformed or missing file raises rather than falling back: an
        operator who pointed at a binding file believes it is in force, and
        silently reverting to defaults would hide exactly the mismatch the file
        was written to fix.
        """

        if not path:
            return cls()

        raw = Path(path).read_text()
        data = json.loads(raw)
        if not isinstance(data, dict):
            raise ValueError(f"protocol binding {path!r} must contain a JSON object")
        return cls.from_mapping(data, source=BindingSource.FILE)


# --- outbound payloads --------------------------------------------------------


# `status` values the backend accepts (telemetry.handler.js:251). `OFFLINE` is
# the server's to assign and is never sent.
BACKEND_STATUS_FOR_MODE = {
    OperatingMode.IDLE: "IDLE",
    OperatingMode.AUTO: "ACTIVE",
    OperatingMode.PAUSED: "PAUSED",
    # An operator STOP ends the run and clears the route: the Rover is idle.
    OperatingMode.STOPPED: "IDLE",
    OperatingMode.ERROR: "ERROR",
}


@dataclass(frozen=True)
class TelemetryFrame:
    """One `TELEMETRY` payload, and whether it carries a position fix.

    A frame is always produced: status is worth reporting with or without a
    fix. What changes is which fields are present. `position_omitted` says why
    `lat`/`lon` are absent, for logs and tests; it never goes on the wire.
    """

    payload: Dict[str, Any]
    position_timestamp: Optional[float] = None
    position_omitted: Optional[str] = None

    @property
    def has_position(self) -> bool:
        return "lat" in self.payload


def _position_age_s(snapshot: RobotSnapshot, now: float) -> Optional[float]:
    if snapshot.position is None:
        return None
    return max(0.0, now - snapshot.position.timestamp)


def build_telemetry_payload(
    snapshot: RobotSnapshot,
    *,
    sequence: int,
    max_position_age_s: float,
    last_position_timestamp: Optional[float] = None,
    now: Optional[float] = None,
) -> TelemetryFrame:
    """`TELEMETRY` to the handoff (§5, §16): measured fields only, the rest omitted.

    Absence is expressed by **omitting** the key. The backend treats any number
    as a measurement, so there is no `null`, `0`, `-1` or placeholder anywhere:

    - `lat`/`lon` (and `speed`) only for a fresh, measured GPS fix, and only
      once per fix. Every `lat`/`lon` the backend receives is stored as a real,
      non-dead-reckoned Observation and counts as completion evidence; a
      dead-reckoned pose is commanded motion around a configured origin, so it
      is never sent, and there is no switch that could send it.
    - `battery` is never sent: this Rover has no battery sensing.
    - no `heading`, `distanceTravelled`, `safety`, `faults`, `localisation` or
      `energy`: nothing on this Pi measures them (the ESP32 link does not
      exist), so they are omitted rather than asserted.

    `timestamp` (epoch ms) is the instant of measurement: the fix's own time
    when the frame carries one, otherwise the instant the status was read. It
    is never re-stamped: a fix too old to send is omitted, not refreshed, and a
    fix already sent is not sent again as though it were a new observation.

    `sequence` is supplied by the caller, which owns its strict increase.
    """

    now = time.time() if now is None else now
    payload: Dict[str, Any] = {
        "sequence": int(sequence),
        "status": BACKEND_STATUS_FOR_MODE.get(snapshot.mode, "ERROR"),
    }

    position = snapshot.position
    omitted: Optional[str] = None
    # `has_fix` rather than a status comparison, so this module needs nothing
    # from the GPS hardware package -- the reading carries its own verdict.
    if position is None or not snapshot.gps.has_fix:
        omitted = f"no usable GPS fix (status={snapshot.gps.status.value})"
    elif not position.is_measured:
        omitted = f"position is {position.source.value}, not measured"
    else:
        age = _position_age_s(snapshot, now)
        if age is not None and age > max_position_age_s:
            omitted = f"position is {age:.1f}s old (limit {max_position_age_s:.1f}s)"
        elif last_position_timestamp is not None and position.timestamp <= last_position_timestamp:
            omitted = "this fix was already sent"

    if omitted is None:
        payload["timestamp"] = now_ms(position.timestamp)
        payload["lat"] = round(position.latitude, 7)
        payload["lon"] = round(position.longitude, 7)
        # Speed over ground from the receiver, when it reported one.
        if position.speed_mps is not None:
            payload["speed"] = round(position.speed_mps, 3)
        return TelemetryFrame(payload, position_timestamp=position.timestamp)

    payload["timestamp"] = now_ms(now)
    return TelemetryFrame(payload, position_omitted=omitted)


def build_heartbeat_payload(
    *,
    commitment_id: Optional[str] = None,
    fence: Any = None,
) -> Dict[str, Any]:
    """`HEARTBEAT` (handoff §4): `{}` when idle, `{commitmentId, fence}` on a mission.

    No `robotId`: identity comes from the authenticated socket. No timestamp:
    the backend stamps its own time on every beat. The commitment form renews
    the mission lease, so it is sent only while this Rover is genuinely
    carrying out that commitment -- the caller decides that.
    """

    if commitment_id is None:
        return {}
    return {"commitmentId": commitment_id, "fence": fence}


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
        # Progress through the assigned task, if there is one. The route itself
        # is not echoed back: RobotX generated it and already has it, and
        # re-sending both waypoint lists on every status frame would spend the
        # link telling the backend what it just said.
        "mission": _status_mission_block(snapshot),
        "lastError": snapshot.last_error,
        "protocol": {"source": binding.source.value, "unconfirmed": list(binding.unconfirmed)},
    }


def _status_mission_block(snapshot: RobotSnapshot) -> Optional[Dict[str, Any]]:
    """The assigned task and how far through it the Rover is, or None."""

    mission = snapshot.mission
    if mission is None:
        return None
    return {
        "taskId": mission.task_id,
        "status": mission.status.value,
        "segment": mission.segment.value,
        "waypointIndex": mission.waypoint_index,
        "waypointsTotal": mission.waypoints_total,
        "pickupReachedAt": None
        if mission.pickup_reached_at is None
        else now_ms(mission.pickup_reached_at),
        "completedAt": None
        if mission.completed_at is None
        else now_ms(mission.completed_at),
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


def build_command_ack_payload(*, command_id: str) -> Dict[str, Any]:
    """Operator `COMMAND_ACK` (handoff §7, §13): exactly `{commandId}`.

    Sent only for a command this Rover **applied**. There is no FAILED on
    this wire: a command the Rover refuses is simply not acknowledged, and the
    backend marks it FAILED itself after its 5/10/15 s redeliveries -- which is
    the truthful outcome. No `robotId` (identity is the socket), no time.
    """

    return {"commandId": command_id}


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


# --- authentication -----------------------------------------------------------


class AuthMethod(str, Enum):
    """Which credential this robot is presenting."""

    PAIRING_CODE = "PAIRING_CODE"  # first commissioning, 6 digits, 300 s TTL
    TOKEN = "TOKEN"                # a session token kept from a previous AUTH


def build_auth_payload(
    *,
    robot_id: str,
    token: Optional[str] = None,
    pairing_code: Optional[str] = None,
) -> Tuple[Dict[str, Any], AuthMethod]:
    """The `AUTH` payload, and which credential it used.

    A persisted session token is preferred over a pairing code: the code is
    single-use with a 300 s TTL, so burning one on every reconnect would mean a
    human re-commissioning the robot every time the Wi-Fi blinked.

    The credential is carried here rather than in the Engine.IO handshake
    because FalconAut's socket connection is **anonymous** -- the server has no
    `handshake.auth` to read, and putting one there would authenticate nothing
    while still putting a secret somewhere it was not expected.

    Raises when neither credential is available: emitting an `AUTH` the backend
    must refuse just produces a silent disconnect and a confusing log.
    """

    if token:
        return {"robotId": robot_id, "token": token}, AuthMethod.TOKEN
    if pairing_code:
        return {"robotId": robot_id, "pairingCode": pairing_code}, AuthMethod.PAIRING_CODE
    raise ValueError(
        "no credential to authenticate with: set ROBOTX_PAIRING_CODE from "
        "POST /api/robots/commission, or restore a persisted session token"
    )


@dataclass(frozen=True)
class AuthResult:
    """What came back in `AUTH_SUCCESS`."""

    token: Optional[str]
    raw: Dict[str, Any]

    @property
    def has_token(self) -> bool:
        return bool(self.token)


def parse_auth_success(data: Any) -> AuthResult:
    """Pull the session token out of an `AUTH_SUCCESS` payload.

    Several spellings are accepted because this is the one field whose exact
    name decides whether the robot can ever reconnect without a human issuing a
    fresh pairing code. Accepting `token`/`sessionToken`/`robotToken` costs
    nothing and removes the most expensive way to be wrong.

    A payload with no recognizable token is **not** an error: the backend may
    consider the socket authenticated without issuing one. The robot records
    that it has no token and will need a pairing code again next time, which is
    visible in `describe()` rather than discovered at the next reconnect.
    """

    if not isinstance(data, dict):
        return AuthResult(None, {"raw": str(data)[:200]})

    for key in ("token", "sessionToken", "robotToken", "accessToken"):
        value = data.get(key)
        if isinstance(value, str) and value.strip():
            return AuthResult(value.strip(), data)

    # Some APIs nest the payload one level down.
    for container in ("robot", "data", "session"):
        nested = data.get(container)
        if isinstance(nested, dict):
            inner = parse_auth_success(nested)
            if inner.has_token:
                return AuthResult(inner.token, data)

    return AuthResult(None, data)


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

        A command addressed to **another robot** is never acknowledged either,
        not even as `FAILED`. Its `commandId` belongs to that robot's `Command`
        row, and a `COMMAND_ACK` from this one would let a misrouted delivery
        mark someone else's command as failed.
        """

        return bool(self.command_id) and self.reason is not RejectionReason.WRONG_ROBOT


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

    # The backend sends `timestamp` (epoch ms); the older spellings are kept.
    issued_at = parse_timestamp(
        data.get("timestamp") or data.get("issuedAt") or data.get("createdAt")
    )
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


def _wire_point(value: Any, *, where: str) -> Tuple[Any, Any]:
    """Pull `(lat, lon)` out of one `{lat, lon}` object from the wire.

    Only `lat` and `lon` are accepted. Tolerating `lng`, `latitude` or a bare
    `[lat, lon]` array would be inventing a second route format for the Rover
    to understand, which is exactly what the contract exists to prevent: if
    RobotX ever sends a different spelling, that is a mismatch to fix once at
    the boundary, not a variant to silently absorb forever.

    Nothing is validated here beyond the shape. Whether the values are real
    coordinates is the domain's rule, enforced in `Mission.create`.
    """

    if not isinstance(value, dict):
        raise MissionRejected(
            MissionRejectReason.MALFORMED,
            f"{where} is {type(value).__name__}, expected an object with lat and lon",
        )
    if "lat" not in value or "lon" not in value:
        raise MissionRejected(
            MissionRejectReason.MALFORMED,
            f"{where} has keys {sorted(str(k) for k in value)[:6]}; expected lat and lon",
        )
    return (value["lat"], value["lon"])


def _wire_path(value: Any, *, where: str) -> list:
    """One waypoint array from the wire, as `(lat, lon)` pairs."""

    if not isinstance(value, (list, tuple)):
        raise MissionRejected(
            MissionRejectReason.MALFORMED,
            f"{where} is {type(value).__name__}, expected an array of points",
        )
    # Length is checked before walking: a hostile or broken payload must not
    # be able to make the socket callback iterate a million objects.
    if len(value) > MAX_PATH_POINTS:
        raise MissionRejected(
            MissionRejectReason.ROUTE_TOO_LONG,
            f"{where} has {len(value)} points; the limit is {MAX_PATH_POINTS}",
        )
    return [_wire_point(point, where=f"{where}[{i}]") for i, point in enumerate(value)]


def parse_task_assign(
    data: Any,
    *,
    expected_robot_id: Optional[str] = None,
) -> Mission:
    """Validate one `TASK_ASSIGN` payload into a domain `Mission`.

    The contract, verified against RobotX::

        {
          taskId,
          pickup:       {lat, lon},
          drop:         {lat, lon},
          pathToPickup: [{lat, lon}, ...],
          pathToDrop:   [{lat, lon}, ...],
          timestamp
        }

    Both paths are WGS84 waypoint arrays that **RobotX** derived from Mapbox
    Directions. The Pi receives no tiles, no map data and no Mapbox
    credentials, and nothing downstream of this function computes a route -- it
    follows the one that arrived or it does not drive.

    Raises `MissionRejected` rather than returning a partial mission. There is
    no lenient mode and no defaulting: an assignment that cannot be fully
    accounted for is refused loudly at the boundary, which is the only place
    the refusal is still cheap.
    """

    if not isinstance(data, dict):
        raise MissionRejected(
            MissionRejectReason.MALFORMED,
            f"expected an object, got {type(data).__name__}",
        )

    try:
        encoded = len(json.dumps(data, default=str))
    except (TypeError, ValueError):
        raise MissionRejected(
            MissionRejectReason.MALFORMED, "payload is not JSON-serializable"
        ) from None
    if encoded > MAX_INBOUND_PAYLOAD_BYTES:
        raise MissionRejected(
            MissionRejectReason.MALFORMED,
            f"{encoded} bytes exceeds {MAX_INBOUND_PAYLOAD_BYTES}",
        )

    task_id = data.get("taskId")
    identifier = task_id.strip() if isinstance(task_id, str) else None

    # Routing is the server's job, but a Rover that drives any assignment it
    # happens to receive is one misconfigured room away from delivering
    # another robot's parcel. Checked only when the payload names a robot:
    # the verified schema does not carry `robotId`.
    target = data.get("robotId")
    if expected_robot_id and target is not None and str(target).strip() != expected_robot_id:
        raise MissionRejected(
            MissionRejectReason.WRONG_ROBOT,
            f"addressed to {str(target)[:40]!r}, this robot is {expected_robot_id!r}",
            task_id=identifier,
        )

    if "timestamp" not in data:
        raise MissionRejected(
            MissionRejectReason.INVALID_TIMESTAMP,
            "no `timestamp` field",
            task_id=identifier,
        )
    # Epoch milliseconds on the wire, by the same convention every other
    # RobotX observation uses; `parse_timestamp` also accepts seconds and
    # ISO-8601 so a serializer change does not break assignment outright.
    timestamp = parse_timestamp(data.get("timestamp"))
    if timestamp is None:
        raise MissionRejected(
            MissionRejectReason.INVALID_TIMESTAMP,
            f"timestamp {str(data.get('timestamp'))[:40]!r} is unreadable",
            task_id=identifier,
        )

    return Mission.create(
        task_id=task_id,
        pickup=_wire_point(data.get("pickup"), where="pickup"),
        drop=_wire_point(data.get("drop"), where="drop"),
        path_to_pickup=_wire_path(data.get("pathToPickup"), where="pathToPickup"),
        path_to_drop=_wire_path(data.get("pathToDrop"), where="pathToDrop"),
        timestamp=timestamp,
    )


def build_task_complete_payload(*, task_id: str, lat: float, lon: float) -> Dict[str, Any]:
    """`TASK_COMPLETE` (handoff §12): `{taskId, lat, lon}`, lat/lon as JSON numbers.

    `lat`/`lon` must be a real measured fix: the backend grades the claim
    against the measured track, and a claim from a kinematically unreachable
    position raises a security event. No `robotId` (the socket is the
    identity) and no `timestamp` (not read by the backend).
    """

    if isinstance(lat, bool) or isinstance(lon, bool) or not isinstance(lat, (int, float)) \
            or not isinstance(lon, (int, float)):
        raise ValueError("TASK_COMPLETE lat/lon must be numbers")
    return {"taskId": task_id, "lat": round(float(lat), 7), "lon": round(float(lon), 7)}


def parse_stop_event(
    data: Any,
    *,
    expected_robot_id: Optional[str] = None,
    now: Optional[float] = None,
) -> Optional[InboundCommand]:
    """The bare `STOP` event, which cancels a task rather than carrying one.

    Deliberately far more permissive than `parse_command`, and it never
    rejects. A stop is the one instruction where refusing to act because the
    payload was not shaped as expected is worse than acting: every failure mode
    of obeying it is "the robot stopped when it need not have".

    It is still routed through the same idempotency cache, keyed on whatever
    identity the payload offers -- a `commandId`, else a `taskId`. With neither,
    the id is synthesized per delivery, which is safe precisely because
    repeating a stop is a no-op.

    The one case it refuses is a payload that **names a different robot**:
    returns None, and the caller neither executes nor acknowledges it. Nothing
    addressed to another robot is acted on or answered by this one, a stop
    included. A payload with no `robotId` is still obeyed.
    """

    now = time.time() if now is None else now
    payload: Dict[str, Any] = data if isinstance(data, dict) else {}

    target = payload.get("robotId")
    if expected_robot_id and target is not None and str(target).strip() != expected_robot_id:
        return None

    identity = payload.get("commandId") or payload.get("id") or payload.get("taskId")
    if identity is not None and str(identity).strip():
        command_id = f"stop:{str(identity).strip()}"
    else:
        command_id = f"stop:anonymous:{now_ms(now)}"

    return InboundCommand(
        command_id=command_id,
        type=CommandType.STOP,
        issued_at=parse_timestamp(payload.get("issuedAt") or payload.get("timestamp")),
        received_at=now,
        raw=payload,
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
