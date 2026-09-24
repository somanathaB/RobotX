"""Assignment Engine envelopes: the signed `command` channel.

Everything the Pi reads or writes on the engine path lives here, to the letter
of `ROBOTX_PI_P2B1_HANDOFF.md` (Dashboard repository) §6-§11. This module is
pure -- no socket, no clock it does not take as an argument except as a
default, no state -- so every admission rule is testable on its own.

Admission, in the handoff's order
---------------------------------
1. `agentId == robotId`                 else ADDRESSED_TO_ANOTHER_AGENT
2. `notValidAfter` in the future        else NOT_VALID_AFTER_PASSED
3. HMAC-SHA256 signature                else BAD_SIGNATURE / SIGNATURE_UNVERIFIABLE
4. fence > fenceFloor and > the commitment's highest applied fence
5. sequence: first 0, then contiguous; at/below highest = DUPLICATE, gap = hold
6. persist the high-water marks **before** acting

Steps 1-3 are here. Steps 4-6 need the persisted per-commitment record, so
they live with it in `BackendLink._admit`, using `check_fence` / `check_sequence`
from this module.

An envelope that fails admission is **not admitted**: no `COMMAND_ACK`, no
`OFFER_*` response, nothing applied. That is the fail-closed reading of
"reject" at the admission stage -- the robot does not answer what it has not
admitted. A `REJECT` *response* is something else: the reply to an admitted
OFFER the Rover cannot carry out.

The signature
-------------
The key is the backend's `COMMAND_SIGNING_KEY`. The backend has **no mechanism
to provision it to a robot**; here it can only be supplied by configuration
(`ROBOTX_COMMAND_SIGNING_KEY`). Without it the Pi does not skip verification --
it cannot admit an OFFER at all. See `canonical_string` for the three points of
the canonical form that must be confirmed against a backend-generated vector.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import math
import re
import time
from dataclasses import dataclass
from datetime import datetime
from decimal import Decimal
from enum import Enum
from typing import Any, Dict, Mapping, Optional, Tuple, Union

from robotx.mission.mission import Mission, MissionRejected, MissionRejectReason


# --- vocabulary (handoff §6) --------------------------------------------------

OFFER = "OFFER"
# Commands that end the commitment: stop that mission, tombstone, ACK.
TERMINATING_COMMANDS = frozenset({"WITHDRAW", "RECALL", "ABORT_MISSION"})
# Commands with no producer today: admitted and ACKed, with no invented effect.
NO_EFFECT_COMMANDS = frozenset({
    "REROUTE", "RESEQUENCE", "RESUME", "TRANSFER_CUSTODY", "STAND_DOWN_ALL",
    "QUARANTINE", "RELEASE_QUARANTINE", "ESTOP_CLEAR", "SHARD_MIGRATE",
    "PARAMETER_PUSH",
})
ENGINE_COMMANDS = frozenset({OFFER}) | TERMINATING_COMMANDS | NO_EFFECT_COMMANDS

# Signed fields, in signing order (handoff §6 step 3).
SIGNED_FIELDS = (
    "agentId", "command", "commandClass", "fenceScope", "commitmentId", "fence",
    "authorityEpoch", "fenceFloor", "sequence", "notValidAfter", "payload",
)
_INTEGER_FIELDS = frozenset({"fence", "authorityEpoch", "fenceFloor"})
FIELD_SEPARATOR = "\u001f"
MIN_SIGNING_KEY_BYTES = 32

CUSTODY_KINDS = ("ACQUIRED", "RELEASED")

_DIGITS = re.compile(r"^[0-9]+$")
_NUMERIC_STRING = re.compile(r"^\s*[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?\s*$")


class EnvelopeRejectReason(str, Enum):
    """Why an envelope was not admitted. Logged and counted, never sent."""

    MALFORMED = "MALFORMED"
    ADDRESSED_TO_ANOTHER_AGENT = "ADDRESSED_TO_ANOTHER_AGENT"
    NOT_VALID_AFTER_PASSED = "NOT_VALID_AFTER_PASSED"
    UNKNOWN_COMMAND = "UNKNOWN_COMMAND"
    SIGNATURE_UNVERIFIABLE = "SIGNATURE_UNVERIFIABLE"  # no signing key configured
    BAD_SIGNATURE = "BAD_SIGNATURE"
    FENCE_AT_OR_BELOW_FLOOR = "FENCE_AT_OR_BELOW_FLOOR"
    STALE_FENCE = "STALE_FENCE"
    TOMBSTONED = "TOMBSTONED"
    DUPLICATE = "DUPLICATE"
    OUT_OF_ORDER = "OUT_OF_ORDER"  # a sequence gap: held, not applied
    NOT_PERSISTED = "NOT_PERSISTED"  # high-water marks could not reach disk


@dataclass(frozen=True)
class EnvelopeRejection:
    reason: EnvelopeRejectReason
    detail: str
    commitment_id: Optional[str] = None


@dataclass(frozen=True)
class Envelope:
    """An engine envelope that passed steps 1-2 and is structurally complete."""

    outbox_id: str
    command: str
    agent_id: str
    commitment_id: str
    fence: int
    fence_floor: Optional[int]
    sequence: int
    not_valid_after: float
    signature: str
    payload: Optional[Dict[str, Any]]
    raw: Dict[str, Any]

    @property
    def fence_wire(self) -> Any:
        """The fence exactly as it arrived, for echoing back."""

        return self.raw["fence"]


# --- helpers ------------------------------------------------------------------


def parse_fence(value: Any) -> Optional[int]:
    """A fence as an integer: decimal string or non-negative JSON integer."""

    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value if value >= 0 else None
    if isinstance(value, str) and _DIGITS.match(value.strip()):
        return int(value.strip())
    return None


def parse_iso(value: Any) -> Optional[float]:
    """Unix seconds from an ISO-8601 string, or None."""

    if not isinstance(value, str) or not value.strip():
        return None
    try:
        parsed = datetime.fromisoformat(value.strip().replace("Z", "+00:00"))
    except ValueError:
        return None
    if parsed.tzinfo is None:
        return None  # a time with no zone cannot be compared to the Pi's clock
    return parsed.timestamp()


def _js_number(value: float) -> str:
    """`Number.prototype.toString` for a finite double, as JSON.stringify renders it."""

    if value == 0:
        return "0"
    sign = "-" if value < 0 else ""
    # repr() is the shortest round-trip form, the same digit string ECMAScript
    # chooses; only the layout differs.
    digits_tuple, exponent = Decimal(repr(abs(value))).normalize().as_tuple()[1:]
    digits = "".join(str(d) for d in digits_tuple)
    k = len(digits)
    n = k + exponent
    if k <= n <= 21:
        body = digits + "0" * (n - k)
    elif 0 < n <= 21:
        body = digits[:n] + "." + digits[n:]
    elif -6 < n <= 0:
        body = "0." + "0" * (-n) + digits
    else:
        mantissa = digits[0] + ("." + digits[1:] if k > 1 else "")
        e = n - 1
        body = f"{mantissa}e{'+' if e >= 0 else '-'}{abs(e)}"
    return sign + body


def js_stringify(value: Any) -> str:
    """JavaScript `JSON.stringify`, with object keys sorted (recursively).

    Python's `json.dumps` differs from JavaScript in exactly the places that
    break a signature: spacing, non-ASCII escaping, and floats (`1.0` vs `1`,
    `1e-07` vs `1e-7`). Each is handled here.
    """

    if value is None:
        return "null"
    if value is True:
        return "true"
    if value is False:
        return "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return _js_number(value) if math.isfinite(value) else "null"
    if isinstance(value, str):
        return json.dumps(value, ensure_ascii=False)
    if isinstance(value, Mapping):
        items = sorted((str(k), v) for k, v in value.items())
        return "{" + ",".join(
            f"{json.dumps(k, ensure_ascii=False)}:{js_stringify(v)}" for k, v in items
        ) + "}"
    if isinstance(value, (list, tuple)):
        return "[" + ",".join(js_stringify(v) for v in value) + "]"
    raise TypeError(f"cannot canonicalize {type(value).__name__}")


def _render(field: str, raw: Mapping[str, Any]) -> str:
    value = raw.get(field)
    if value is None:
        return "null"
    if field in _INTEGER_FIELDS:
        as_int = parse_fence(value) if not isinstance(value, int) else value
        if as_int is None:
            raise ValueError(f"{field} {value!r} is not an integer")
        return str(as_int)
    if field == "notValidAfter" and isinstance(value, str):
        return value
    return js_stringify(value)


def canonical_string(raw: Mapping[str, Any]) -> str:
    """The exact string the backend signs (`commandSigning.js`, handoff §6).

    Three renderings follow the handoff's words but should be pinned by a
    test vector generated on the backend before this is trusted in the field:

    - `notValidAfter` is rendered as the bare ISO string, without the quotes
      `JSON.stringify` would add (the handoff lists it separately from
      "everything else -> JSON.stringify").
    - "objects -> sorted keys" is applied recursively.
    - the key is used as the UTF-8 bytes of the configured string, which is
      what Node's `createHmac` does with a string key.
    """

    return FIELD_SEPARATOR.join(f"{field}={_render(field, raw)}" for field in SIGNED_FIELDS)


def sign(raw: Mapping[str, Any], key: bytes) -> str:
    return hmac.new(key, canonical_string(raw).encode("utf-8"), hashlib.sha256).hexdigest()


def verify_signature(raw: Mapping[str, Any], key: Optional[bytes]) -> Optional[EnvelopeRejection]:
    """None when the signature verifies; otherwise why not.

    With no key this returns SIGNATURE_UNVERIFIABLE rather than passing: an
    envelope that cannot be verified is not admitted. There is deliberately
    no "skip verification" mode.
    """

    commitment = raw.get("commitmentId") if isinstance(raw.get("commitmentId"), str) else None
    if not key or len(key) < MIN_SIGNING_KEY_BYTES:
        return EnvelopeRejection(
            EnvelopeRejectReason.SIGNATURE_UNVERIFIABLE,
            "no command signing key of at least 32 bytes is configured "
            "(ROBOTX_COMMAND_SIGNING_KEY); the envelope cannot be verified",
            commitment,
        )
    signature = raw.get("signature")
    if not isinstance(signature, str) or not signature:
        return EnvelopeRejection(EnvelopeRejectReason.BAD_SIGNATURE, "no signature", commitment)
    try:
        expected = sign(raw, key)
    except (TypeError, ValueError) as e:
        return EnvelopeRejection(EnvelopeRejectReason.MALFORMED, f"cannot canonicalize: {e}", commitment)
    if not hmac.compare_digest(expected, signature.strip().lower()):
        return EnvelopeRejection(EnvelopeRejectReason.BAD_SIGNATURE, "signature does not verify", commitment)
    return None


# --- admission steps 1-2 and structure ----------------------------------------


def parse_envelope(
    data: Any,
    *,
    expected_agent_id: str,
    now: Optional[float] = None,
) -> Union[Envelope, EnvelopeRejection]:
    """Structure, addressee and validity window. Never raises."""

    now = time.time() if now is None else now
    if not isinstance(data, dict):
        return EnvelopeRejection(EnvelopeRejectReason.MALFORMED, f"expected an object, got {type(data).__name__}")

    commitment = data.get("commitmentId")
    commitment_id = commitment.strip() if isinstance(commitment, str) and commitment.strip() else None

    # Step 1 first: something addressed to another agent is not ours to
    # inspect further, answer, or acknowledge.
    agent = data.get("agentId")
    if not isinstance(agent, str) or agent != expected_agent_id:
        return EnvelopeRejection(
            EnvelopeRejectReason.ADDRESSED_TO_ANOTHER_AGENT,
            f"agentId {str(agent)[:40]!r} is not {expected_agent_id!r}",
            commitment_id,
        )

    command = data.get("command")
    if not isinstance(command, str) or command not in ENGINE_COMMANDS:
        return EnvelopeRejection(
            EnvelopeRejectReason.UNKNOWN_COMMAND, f"command {str(command)[:40]!r}", commitment_id
        )

    outbox = data.get("outboxId")
    if not isinstance(outbox, str) or not outbox.strip():
        return EnvelopeRejection(EnvelopeRejectReason.MALFORMED, "no outboxId", commitment_id)
    if commitment_id is None:
        return EnvelopeRejection(EnvelopeRejectReason.MALFORMED, "no commitmentId")

    fence = parse_fence(data.get("fence"))
    if fence is None:
        return EnvelopeRejection(EnvelopeRejectReason.MALFORMED, f"fence {data.get('fence')!r}", commitment_id)
    floor_raw = data.get("fenceFloor")
    fence_floor = None if floor_raw is None else parse_fence(floor_raw)
    if floor_raw is not None and fence_floor is None:
        return EnvelopeRejection(EnvelopeRejectReason.MALFORMED, f"fenceFloor {floor_raw!r}", commitment_id)

    sequence = data.get("sequence")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        return EnvelopeRejection(EnvelopeRejectReason.MALFORMED, f"sequence {sequence!r}", commitment_id)

    payload = data.get("payload")
    if payload is not None and not isinstance(payload, dict):
        return EnvelopeRejection(EnvelopeRejectReason.MALFORMED, "payload is not an object", commitment_id)

    not_valid_after = parse_iso(data.get("notValidAfter"))
    if not_valid_after is None:
        return EnvelopeRejection(
            EnvelopeRejectReason.MALFORMED, f"notValidAfter {data.get('notValidAfter')!r}", commitment_id
        )
    # Step 2.
    if not_valid_after <= now:
        return EnvelopeRejection(
            EnvelopeRejectReason.NOT_VALID_AFTER_PASSED,
            f"notValidAfter was {now - not_valid_after:.1f}s ago",
            commitment_id,
        )

    return Envelope(
        outbox_id=outbox.strip(),
        command=command,
        agent_id=agent,
        commitment_id=commitment_id,
        fence=fence,
        fence_floor=fence_floor,
        sequence=sequence,
        not_valid_after=not_valid_after,
        signature=str(data.get("signature") or ""),
        payload=payload,
        raw=data,
    )


# --- admission steps 4-5 ------------------------------------------------------


def check_fence(envelope: Envelope, highest_applied: Optional[int]) -> Optional[EnvelopeRejection]:
    """Step 4: strictly above the floor and above anything already applied."""

    if envelope.fence_floor is not None and envelope.fence <= envelope.fence_floor:
        return EnvelopeRejection(
            EnvelopeRejectReason.FENCE_AT_OR_BELOW_FLOOR,
            f"fence {envelope.fence} <= fenceFloor {envelope.fence_floor}",
            envelope.commitment_id,
        )
    if highest_applied is not None and envelope.fence <= highest_applied:
        return EnvelopeRejection(
            EnvelopeRejectReason.STALE_FENCE,
            f"fence {envelope.fence} <= highest applied {highest_applied}",
            envelope.commitment_id,
        )
    return None


def check_sequence(envelope: Envelope, highest_applied: Optional[int]) -> Optional[EnvelopeRejection]:
    """Step 5: first is 0, then contiguous."""

    expected = 0 if highest_applied is None else highest_applied + 1
    if envelope.sequence < expected:
        return EnvelopeRejection(
            EnvelopeRejectReason.DUPLICATE,
            f"sequence {envelope.sequence} already applied (next is {expected})",
            envelope.commitment_id,
        )
    if envelope.sequence > expected:
        return EnvelopeRejection(
            EnvelopeRejectReason.OUT_OF_ORDER,
            f"sequence {envelope.sequence} arrived before {expected}; holding it",
            envelope.commitment_id,
        )
    return None


# --- the OFFER payload --------------------------------------------------------


@dataclass(frozen=True)
class Stop:
    sequence: int
    lat: float
    lon: float
    # Empty when the stop carried no path: not executable (NO_EXECUTABLE_PATH).
    path: Tuple[Tuple[float, float], ...]
    stop_type: Any = None


@dataclass(frozen=True)
class Offer:
    commitment_id: str
    fence: int
    fence_wire: Any
    leg_id: Any
    task_id: Optional[str]
    stops: Tuple[Stop, ...]
    offer_expiry: Optional[float]
    envelope: Envelope

    def commanded_path(self) -> Tuple[Tuple[float, float], ...]:
        """Every stop's path, in stop order: the corridor TASK_COMPLETE is graded on."""

        return tuple(point for stop in self.stops for point in stop.path)


def _number(value: Any) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def parse_offer(envelope: Envelope) -> Union[Offer, EnvelopeRejection]:
    """The OFFER payload, cross-checked against its envelope.

    A payload whose `commitmentId` or `fence` disagrees with the signed
    envelope around it is malformed and not admitted.
    """

    payload = envelope.payload
    cid = envelope.commitment_id

    def bad(detail: str) -> EnvelopeRejection:
        return EnvelopeRejection(EnvelopeRejectReason.MALFORMED, detail, cid)

    if not isinstance(payload, dict):
        return bad("OFFER carries no payload object")
    if payload.get("commitmentId") != cid:
        return bad(f"payload commitmentId {str(payload.get('commitmentId'))[:40]!r} != envelope {cid!r}")
    if parse_fence(payload.get("fence")) != envelope.fence:
        return bad(f"payload fence {payload.get('fence')!r} != envelope fence {envelope.fence_wire!r}")

    task = payload.get("taskId")
    if task is not None and not (isinstance(task, str) and task.strip()):
        return bad(f"taskId {task!r}")

    raw_stops = payload.get("stopSequence")
    if not isinstance(raw_stops, list):
        return bad("stopSequence is not an array")
    stops = []
    for i, raw in enumerate(raw_stops):
        if not isinstance(raw, dict):
            return bad(f"stopSequence[{i}] is not an object")
        seq, lat, lon = raw.get("sequence"), _number(raw.get("lat")), _number(raw.get("lon"))
        if isinstance(seq, bool) or not isinstance(seq, int) or lat is None or lon is None:
            return bad(f"stopSequence[{i}] lacks an integer sequence and numeric lat/lon")
        path = []
        raw_path = raw.get("path")
        if isinstance(raw_path, list):
            for j, point in enumerate(raw_path):
                plat = _number(point.get("lat")) if isinstance(point, dict) else None
                plon = _number(point.get("lon")) if isinstance(point, dict) else None
                if plat is None or plon is None:
                    return bad(f"stopSequence[{i}].path[{j}] is not {{lat, lon}}")
                path.append((plat, plon))
        elif raw_path is not None:
            return bad(f"stopSequence[{i}].path is not an array")
        stops.append(Stop(seq, lat, lon, tuple(path), raw.get("stopType")))

    return Offer(
        commitment_id=cid,
        fence=envelope.fence,
        fence_wire=envelope.fence_wire,
        leg_id=payload.get("legId"),
        task_id=task.strip() if isinstance(task, str) else None,
        stops=tuple(sorted(stops, key=lambda s: s.sequence)),
        offer_expiry=parse_iso(payload.get("offerExpiry")),
        envelope=envelope,
    )


def offer_to_mission(offer: Offer, *, now: Optional[float] = None) -> Mission:
    """The Rover's mission for an offer, or `MissionRejected`.

    The Rover can execute exactly one shape: a task leg with two stops, the
    first where custody is acquired and the second where it is released, each
    with the path that leads to it. `stopType` is not interpreted -- its
    vocabulary is not in the handoff -- so the order of `sequence` is what
    decides which stop is which, and anything else is refused rather than
    guessed at.
    """

    if len(offer.stops) != 2 or offer.task_id is None:
        raise MissionRejected(
            MissionRejectReason.MALFORMED,
            f"{len(offer.stops)} stop(s), taskId={offer.task_id!r}; this Rover executes "
            "only a two-stop task leg",
            task_id=offer.task_id,
        )
    first, last = offer.stops
    return Mission.create(
        task_id=offer.task_id,
        pickup=(first.lat, first.lon),
        drop=(last.lat, last.lon),
        path_to_pickup=list(first.path),
        path_to_drop=list(last.path),
        timestamp=time.time() if now is None else now,
    )


# --- decisions and responses (handoff §7-§11) ---------------------------------


class OfferVerdict(str, Enum):
    ACCEPT = "ACCEPT"
    REJECT = "REJECT"
    DEFER = "DEFER"


@dataclass(frozen=True)
class OfferDecision:
    """The Rover's own answer to "can I carry this out?". Never a ranking."""

    verdict: OfferVerdict
    reason: Optional[str] = None
    until: Any = None
    mission: Optional[Mission] = None

    @classmethod
    def accept(cls, mission: Mission) -> "OfferDecision":
        return cls(OfferVerdict.ACCEPT, mission=mission)

    @classmethod
    def reject(cls, reason: str) -> "OfferDecision":
        return cls(OfferVerdict.REJECT, reason=reason)

    @classmethod
    def defer(cls, until: Any, reason: str) -> "OfferDecision":
        return cls(OfferVerdict.DEFER, reason=reason, until=until)


def validate_defer_until(until: Any, *, now: Optional[float] = None) -> Any:
    """`until` as the handoff allows it, or ValueError.

    A number of epoch milliseconds, or an ISO-8601 string, strictly in the
    future. A *numeric string* is invalid on the backend, so it is refused
    here rather than sent to be silently refused there.
    """

    now_ms = (time.time() if now is None else now) * 1000.0
    if isinstance(until, bool):
        raise ValueError("until is a boolean")
    if isinstance(until, (int, float)):
        if not math.isfinite(float(until)) or until <= now_ms:
            raise ValueError(f"until {until!r} is not a future epoch-ms time")
        return until
    if isinstance(until, str):
        if _NUMERIC_STRING.match(until):
            raise ValueError(f"until {until!r} is a numeric string; send a number or ISO-8601")
        at = parse_iso(until)
        if at is None or at * 1000.0 <= now_ms:
            raise ValueError(f"until {until!r} is not a future ISO-8601 time")
        return until
    raise ValueError(f"until is {type(until).__name__}")


def build_engine_ack(envelope: Envelope) -> Dict[str, Any]:
    """`COMMAND_ACK` for an engine envelope: its outboxId, fence and epoch, echoed."""

    payload: Dict[str, Any] = {"outboxId": envelope.outbox_id, "fence": envelope.fence_wire}
    if "authorityEpoch" in envelope.raw:
        payload["authorityEpoch"] = envelope.raw["authorityEpoch"]
    return payload


def build_offer_response(offer: Offer, decision: OfferDecision, *, now: Optional[float] = None) -> Dict[str, Any]:
    """The `OFFER_ACCEPT` / `OFFER_REJECT` / `OFFER_DEFER` payload.

    `fence` echoes the OFFER's fence exactly as it arrived. Raises ValueError
    for a DEFER whose `until` the backend would refuse.
    """

    payload: Dict[str, Any] = {"commitmentId": offer.commitment_id, "fence": offer.fence_wire}
    if decision.verdict is OfferVerdict.REJECT:
        payload["reason"] = decision.reason
    elif decision.verdict is OfferVerdict.DEFER:
        payload["until"] = validate_defer_until(decision.until, now=now)
        payload["reason"] = decision.reason
    return payload


def build_custody_event(*, commitment_id: str, fence: Any, kind: str) -> Dict[str, Any]:
    """`CUSTODY_EVENT`: exactly the three fields the backend reads."""

    if kind not in CUSTODY_KINDS:
        raise ValueError(f"custody kind {kind!r}")
    return {"commitmentId": commitment_id, "fence": fence, "kind": kind}
