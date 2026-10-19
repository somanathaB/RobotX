"""Opt-in, test-only mission ingress: a locally supplied task, decided by the agent.

Why this exists
---------------
The RobotX Assignment Engine does not assign tasks to physical robots, so a
physical integration test has no way to give this Rover a mission while its
backend link is up (engine mode refuses `/mission/start`, and an OFFER must be
signed by the backend). This module is the one exception: a loopback-only,
token-authorized route that hands a locally supplied two-stop task to the
running agent -- and nothing else.

What it does
------------
`InjectionGate.inject` runs, in the agent's own process and event loop,
exactly what `BackendLink._respond_to_offer` runs for an admitted OFFER:

    agent.assess_offer(assignment)                      # every readiness rule
    agent.assign_mission(mission, custody_required=True)

so a test task meets every gate an engine task does, holds at each stop for a
real custody confirmation, and is driven, stopped and reported by the agent
loop like any other. Nothing here moves a wheel, enables motion, or talks to
the ESP32.

What it never does
------------------
- Signs, forges or imitates an OFFER. There is no commitment, fence, envelope
  or signature anywhere in this path, and `engine.verify_signature` is not
  involved: the backend's admission path is untouched.
- Claims RobotX assigned the task. Task ids must start with `TEST-INJECTED-`;
  every record carries `"provenance": {"source": "test_injected",
  "backend_assigned": false}`. With no commitment held, the backend link sends
  no OFFER_*, CUSTODY_EVENT or TASK_COMPLETE for it (`BackendLink._held_mission`
  is None), and HEARTBEAT stays `{}`. TELEMETRY is unchanged and genuine.
- Runs unless asked. Disabled by default (`ROBOTX_TEST_INJECTION_ENABLED`),
  and even when enabled it refuses every request -- fails closed -- unless the
  API is bound to a loopback IP address, the injection token digest is
  configured, the separate operator token digest
  (`ROBOTX_OPERATOR_TOKEN_SHA256`, which guards custody confirmation and
  e-stop clear) is configured and differs from it, and the provenance log is
  readable and writable.

Authorization
-------------
`Authorization: Bearer <token>`, compared in constant time against the
SHA-256 digest in `ROBOTX_TEST_INJECTION_TOKEN_SHA256`. The token itself is
never configured, stored or logged by the agent; the operator holds it. Five
consecutive failures lock the route for five minutes. Requests must also come
from a loopback peer, name a loopback `Host` (no DNS rebinding) and carry no
`Origin` (no browser page can post here).
"""

from __future__ import annotations

import hashlib
import hmac
import ipaddress
import json
import logging
import math
import os
import re
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Optional, Tuple

from robotx.communication.engine import OfferVerdict, Stop
from robotx.application.operator_access import valid_digest
from robotx.config.logging_setup import log_event
from robotx.mission.mission import Mission, MissionRejected, validate_coordinate


logger = logging.getLogger(__name__)

LatLon = Tuple[float, float]

SCHEMA_VERSION = 1
TASK_ID_PREFIX = "TEST-INJECTED-"
PROVENANCE_SOURCE = "test_injected"
PROVENANCE_REASON = (
    "the RobotX backend Assignment Engine cannot assign tasks to physical robots; "
    "this assignment was supplied locally by a test operator, not by RobotX"
)
MAX_REGION_RADIUS_M = 500.0
MIN_TOKEN_CHARS = 32
MAX_AUTH_FAILURES = 5
AUTH_LOCKOUT_S = 300.0
# Two legs of 2000 points at ~50 bytes each, with room to spare.
MAX_BODY_BYTES = 512 * 1024

REQUIRED_FIELDS = (
    "schema_version", "test_run_id", "task_id", "robot_id", "region",
    "pickup", "drop", "path_to_pickup", "path_to_drop",
)

_IDENTIFIER = re.compile(r"^[A-Za-z0-9._-]{1,64}$")
_SHA256_HEX = re.compile(r"^[0-9a-f]{64}$")
_EARTH_RADIUS_M = 6_371_000.0


def provenance() -> Dict[str, Any]:
    return {"source": PROVENANCE_SOURCE, "backend_assigned": False, "reason": PROVENANCE_REASON}


class InjectionRejected(Exception):
    """A request refused before or instead of reaching the agent. Never partial."""

    def __init__(self, code: str, detail: str, status: int = 422) -> None:
        super().__init__(f"{code}: {detail}")
        self.code = code
        self.detail = detail
        self.status = status

    def to_dict(self) -> Dict[str, Any]:
        return {"outcome": "REFUSED", "code": self.code, "detail": self.detail}


# --- the request --------------------------------------------------------------


def distance_m(a: LatLon, b: LatLon) -> float:
    lat1, lon1, lat2, lon2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = math.sin((lat2 - lat1) / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2) ** 2
    return 2 * _EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


@dataclass(frozen=True)
class InjectedAssignment:
    """The two stops `RobotAgent.assess_offer` reads, and only those.

    Deliberately not an `engine.Offer`: no commitment, fence, envelope or
    signature, so nothing downstream can take it for one RobotX holds.
    """

    task_id: str
    stops: Tuple[Stop, ...]
    provenance: str = PROVENANCE_SOURCE


@dataclass(frozen=True)
class InjectionRequest:
    test_run_id: str
    task_id: str
    robot_id: str
    region_center: LatLon
    region_radius_m: float
    mission: Mission
    raw: Dict[str, Any] = field(repr=False, compare=False)

    def assignment(self) -> InjectedAssignment:
        m = self.mission
        return InjectedAssignment(
            task_id=m.task_id,
            stops=(
                Stop(sequence=0, lat=m.pickup[0], lon=m.pickup[1], path=m.path_to_pickup, stop_type="PICKUP"),
                Stop(sequence=1, lat=m.drop[0], lon=m.drop[1], path=m.path_to_drop, stop_type="DROP"),
            ),
        )

    def in_region(self, point: LatLon) -> bool:
        return distance_m(self.region_center, point) <= self.region_radius_m

    def summary(self) -> Dict[str, Any]:
        def wire(p: LatLon) -> Dict[str, float]:
            return {"lat": p[0], "lon": p[1]}

        return {
            "test_run_id": self.test_run_id,
            "task_id": self.task_id,
            "robot_id": self.robot_id,
            "region": {"center": wire(self.region_center), "radius_m": self.region_radius_m},
            "pickup": wire(self.mission.pickup),
            "drop": wire(self.mission.drop),
            "path_to_pickup_points": len(self.mission.path_to_pickup),
            "path_to_drop_points": len(self.mission.path_to_drop),
        }


def _point(value: Any, where: str) -> Tuple[Any, Any]:
    """`{lat, lon}` -> a pair, unchecked: `Mission.create` checks the numbers."""

    if not isinstance(value, Mapping) or set(value) != {"lat", "lon"}:
        raise InjectionRejected("MALFORMED", f"{where} must be an object with exactly lat and lon")
    return (value["lat"], value["lon"])


def _path(value: Any, where: str) -> Any:
    if not isinstance(value, list):
        raise InjectionRejected("MALFORMED", f"{where} must be an array of {{lat, lon}}")
    return [_point(p, f"{where}[{i}]") for i, p in enumerate(value)]


def _identifier(raw: Mapping[str, Any], key: str) -> str:
    value = raw[key]
    if not isinstance(value, str) or not _IDENTIFIER.match(value):
        raise InjectionRejected("MALFORMED", f"{key} must be 1-64 characters of [A-Za-z0-9._-]")
    return value


def validate_request(raw: Any, *, expected_robot_id: Optional[str] = None,
                     now: Optional[float] = None) -> InjectionRequest:
    """A validated request, or `InjectionRejected`.

    The Rover's own rules (`Mission.create`: finite in-range coordinates,
    2..2000 points per leg, a task id), plus stricter ones that exist only for
    a hand-written test task: the provenance prefix, this Rover's identity,
    and every point inside a declared region of at most 500 m.
    """

    if not isinstance(raw, dict):
        raise InjectionRejected("MALFORMED", "the request must be a JSON object")
    missing = [k for k in REQUIRED_FIELDS if k not in raw]
    if missing:
        raise InjectionRejected("MISSING_FIELD", f"missing: {', '.join(missing)}")
    unknown = sorted(set(raw) - set(REQUIRED_FIELDS))
    if unknown:
        raise InjectionRejected("UNKNOWN_FIELD", f"not part of the schema: {', '.join(unknown)}")
    if isinstance(raw["schema_version"], bool) or raw["schema_version"] != SCHEMA_VERSION:
        raise InjectionRejected("SCHEMA_VERSION", f"schema_version must be {SCHEMA_VERSION}")

    test_run_id = _identifier(raw, "test_run_id")
    task_id = _identifier(raw, "task_id")
    if not task_id.startswith(TASK_ID_PREFIX) or len(task_id) == len(TASK_ID_PREFIX):
        raise InjectionRejected("TASK_ID_PROVENANCE", f"task_id must start with {TASK_ID_PREFIX!r}")
    robot_id = _identifier(raw, "robot_id")
    if expected_robot_id is not None and robot_id != expected_robot_id:
        raise InjectionRejected("WRONG_ROBOT", f"request is for {robot_id!r}; this Rover is {expected_robot_id!r}")

    region = raw["region"]
    if not isinstance(region, Mapping) or set(region) != {"center", "radius_m"}:
        raise InjectionRejected("MALFORMED", "region must be {center: {lat, lon}, radius_m}")
    radius = region["radius_m"]
    try:
        radius_ok = (not isinstance(radius, bool) and isinstance(radius, (int, float))
                     and math.isfinite(radius) and 0 < radius <= MAX_REGION_RADIUS_M)
    except OverflowError:  # an integer too large for a float
        radius_ok = False
    if not radius_ok:
        raise InjectionRejected("REGION", f"region.radius_m must be in (0, {MAX_REGION_RADIUS_M:.0f}]")

    try:
        center = validate_coordinate(*_point(region["center"], "region.center"), where="region.center")
        mission = Mission.create(
            task_id=task_id,
            pickup=_point(raw["pickup"], "pickup"),
            drop=_point(raw["drop"], "drop"),
            path_to_pickup=_path(raw["path_to_pickup"], "path_to_pickup"),
            path_to_drop=_path(raw["path_to_drop"], "path_to_drop"),
            timestamp=time.time() if now is None else now,
        )
    except MissionRejected as e:
        raise InjectionRejected(e.reason.value, e.detail) from None
    except OverflowError:
        # JSON integers are unbounded; float() of one past ~1e308 raises.
        raise InjectionRejected("INVALID_COORDINATE", "a coordinate is too large to be a number") from None

    request = InjectionRequest(test_run_id=test_run_id, task_id=task_id, robot_id=robot_id,
                               region_center=center, region_radius_m=float(radius),
                               mission=mission, raw=dict(raw))
    points = (
        [("pickup", mission.pickup), ("drop", mission.drop)]
        + [(f"path_to_pickup[{i}]", p) for i, p in enumerate(mission.path_to_pickup)]
        + [(f"path_to_drop[{i}]", p) for i, p in enumerate(mission.path_to_drop)]
    )
    for where, point in points:
        if not request.in_region(point):
            raise InjectionRejected(
                "OUT_OF_REGION",
                f"{where} is {distance_m(center, point):.0f} m from the region center (radius {radius:.0f} m)",
            )
    return request


# --- configuration --------------------------------------------------------------


def is_loopback_address(host: Optional[str]) -> bool:
    """An IP literal in 127.0.0.0/8 or ::1. A name ("localhost") is not enough."""

    if not host:
        return False
    try:
        return ipaddress.ip_address(host.strip().strip("[]")).is_loopback
    except ValueError:
        return False


def token_digest(token: str) -> str:
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class InjectionConfig:
    enabled: bool = False
    token_sha256: Optional[str] = field(default=None, repr=False)
    log_path: Optional[str] = None
    api_host: str = ""
    robot_id: str = ""
    operator_token_sha256: Optional[str] = field(default=None, repr=False)

    @classmethod
    def from_settings(cls, settings: Any) -> "InjectionConfig":
        return cls(
            enabled=bool(settings.test_injection_enabled),
            token_sha256=settings.test_injection_token_sha256,
            log_path=settings.test_injection_log_path,
            api_host=settings.api_host,
            robot_id=settings.robot_id,
            operator_token_sha256=settings.operator_token_sha256,
        )

    def problem(self) -> Optional[str]:
        """Why an enabled gate must refuse everything, or None."""

        if not is_loopback_address(self.api_host):
            return (f"ROBOTX_API_HOST={self.api_host!r} is not a loopback IP address; test injection "
                    "is served only on loopback")
        digest = (self.token_sha256 or "").strip()
        if not _SHA256_HEX.match(digest):
            return "ROBOTX_TEST_INJECTION_TOKEN_SHA256 is not set to a 64-character lowercase hex SHA-256 digest"
        if not self.log_path or not Path(self.log_path).is_absolute():
            return "ROBOTX_TEST_INJECTION_LOG_PATH is not set to an absolute path"
        if not self.robot_id:
            return "ROBOTX_ROBOT_ID is not set"
        if not valid_digest(self.operator_token_sha256):
            # A test mission advances on /mission/custody and is released by
            # /safety/clear; neither may be open to every local process.
            return ("ROBOTX_OPERATOR_TOKEN_SHA256 must be set when test injection is enabled, so custody "
                    "confirmation and e-stop clear are not open to every local process or web page")
        if (self.operator_token_sha256 or "").strip() == digest:
            return "the operator and injection tokens must differ"
        return None


# --- the gate -------------------------------------------------------------------


class InjectionGate:
    """The route's whole policy. One per process; every inject is serialized."""

    def __init__(self, config: InjectionConfig, *, clock: Callable[[], float] = time.monotonic) -> None:
        self.config = config
        self.clock = clock
        self._lock = threading.Lock()
        self._failures = 0
        self._locked_until = 0.0
        self._task_ids: Dict[str, str] = {}   # task id -> last event recorded for it
        self._problem: Optional[str] = config.problem() if config.enabled else None
        if config.enabled and self._problem is None:
            self._problem = self._load_log() or self._probe_writable()
        if config.enabled:
            if self._problem is None:
                log_event(logger, "test_injection.enabled",
                          "TEST-ONLY mission injection is ENABLED on loopback", level=logging.WARNING,
                          log_path=config.log_path, injected_before=len(self._task_ids))
            else:
                log_event(logger, "test_injection.unavailable",
                          "test injection is enabled but refuses every request", level=logging.ERROR,
                          reason=self._problem)

    @classmethod
    def from_settings(cls, settings: Any) -> "InjectionGate":
        return cls(InjectionConfig.from_settings(settings))

    @property
    def enabled(self) -> bool:
        return self.config.enabled

    @property
    def available(self) -> bool:
        return self.config.enabled and self._problem is None

    # --- the provenance log -------------------------------------------------------

    def _load_log(self) -> Optional[str]:
        """Why the log cannot be trusted, or None. Never raises: called while the agent starts.

        Every complete line must be a JSON object, or the gate refuses: a
        corrupt history cannot say which task ids were used. One case is
        recovered: a torn *final* fragment with no newline, which only an
        interrupted last write leaves. Each record is fsync'ed before the
        agent is consulted, so a torn ATTEMPTED means the agent never saw that
        request, and a torn outcome follows its own complete ATTEMPTED -- the
        task id stays spent either way. The fragment is moved to a
        `.torn-<time>` file beside the log (kept, not discarded) and the log is
        cut back to its last complete line, so the next record starts clean.
        """

        path = Path(self.config.log_path or "")
        try:
            data = path.read_bytes()
        except FileNotFoundError:
            return None
        except Exception as e:  # noqa: BLE001 - permissions, sandbox, I/O: all mean "unknown history"
            return self._unreadable(path, e)

        cut = data.rfind(b"\n") + 1
        complete, tail = data[:cut], data[cut:]
        try:
            for number, line in enumerate(complete.decode("utf-8").splitlines(), start=1):
                if not line.strip():
                    continue
                record = json.loads(line)
                if not isinstance(record, dict):
                    raise ValueError(f"line {number} is not an object")
                if isinstance(record.get("task_id"), str):
                    self._task_ids[record["task_id"]] = str(record.get("event"))
        except Exception as e:  # noqa: BLE001 - bad JSON, bad encoding
            return self._unreadable(path, e)

        if tail:
            return self._quarantine_tail(path, tail, len(complete))
        return None

    def _unreadable(self, path: Path, error: BaseException) -> str:
        self._task_ids.clear()
        return (f"the provenance log {path} cannot be read ({type(error).__name__}); refusing rather than "
                "forgetting which task ids were injected")

    def _quarantine_tail(self, path: Path, tail: bytes, keep: int) -> Optional[str]:
        try:
            # Conservative: a task id still legible in the fragment counts as spent.
            fragment = json.loads(tail.decode("utf-8"))
            if isinstance(fragment, dict) and isinstance(fragment.get("task_id"), str):
                self._task_ids.setdefault(fragment["task_id"], "TORN")
        except Exception:  # noqa: BLE001 - usually not parseable at all; nothing to keep
            pass
        side = path.with_name(f"{path.name}.torn-{int(time.time() * 1000)}")
        try:
            fd = os.open(side, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, "wb") as handle:
                handle.write(tail)
                handle.flush()
                os.fsync(handle.fileno())
            fd = os.open(path, os.O_WRONLY)
            try:
                os.ftruncate(fd, keep)
                os.fsync(fd)
            finally:
                os.close(fd)
        except Exception as e:  # noqa: BLE001
            self._task_ids.clear()
            return (f"the provenance log {path} ends in a torn record that could not be set aside "
                    f"({type(e).__name__}); refusing until it is repaired")
        log_event(logger, "test_injection.torn_record_quarantined",
                  "an interrupted final record was moved aside; history before it is intact",
                  level=logging.WARNING, log_path=str(path), quarantine=str(side), bytes=len(tail))
        return None

    def _probe_writable(self) -> Optional[str]:
        """Why the log cannot be appended to, or None: found at start, not on the first request."""

        path = Path(self.config.log_path or "")
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            os.close(os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600))
        except Exception as e:  # noqa: BLE001 - read-only filesystem, permissions, sandbox: all refuse
            return f"the provenance log {path} cannot be written ({type(e).__name__}); refusing every injection"
        return None

    def _append(self, record: Dict[str, Any]) -> bool:
        path = Path(self.config.log_path or "")
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
            with os.fdopen(fd, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, sort_keys=True) + "\n")
                handle.flush()
                os.fsync(handle.fileno())
        except OSError as e:
            log_event(logger, "test_injection.log_failed", "provenance record not written", level=logging.ERROR,
                      error=type(e).__name__, task_id=record.get("task_id"))
            return False
        self._task_ids[record["task_id"]] = record["event"]
        return True

    @staticmethod
    def _record(event: str, request: InjectionRequest, **fields: Any) -> Dict[str, Any]:
        now = time.time()
        return {
            "at": datetime.fromtimestamp(now, timezone.utc).isoformat(timespec="milliseconds"),
            "at_unix": round(now, 3),
            "event": event,
            "test_run_id": request.test_run_id,
            "task_id": request.task_id,
            "robot_id": request.robot_id,
            "provenance": provenance(),
            **fields,
        }

    # --- request checks -------------------------------------------------------------

    def check_origin(self, *, client_host: Optional[str], host_header: Optional[str],
                     origin: Optional[str], server_host: Optional[str]) -> None:
        # `server_host` is the local address of the accepted connection, so this
        # holds whatever the process was told to bind -- not only what
        # ROBOTX_API_HOST claims.
        if not is_loopback_address(client_host) or not is_loopback_address(server_host):
            raise InjectionRejected("NOT_LOOPBACK", "test injection is accepted from loopback only", 403)
        host = (host_header or "").strip()
        bare = host[1:host.find("]")] if host.startswith("[") else host.rsplit(":", 1)[0]
        if not is_loopback_address(bare):
            raise InjectionRejected("HOST_NOT_LOOPBACK", "the Host header must be a loopback IP address", 403)
        if origin is not None:
            raise InjectionRejected("ORIGIN_PRESENT", "browser requests are refused", 403)

    def check_available(self) -> None:
        if self._problem is not None:
            raise InjectionRejected("NOT_CONFIGURED", self._problem, 503)

    def authorize(self, authorization: Optional[str]) -> None:
        now = self.clock()
        with self._lock:
            if now < self._locked_until:
                raise InjectionRejected("LOCKED_OUT", "too many failed authorizations; try again later", 429)
            scheme, _, token = (authorization or "").partition(" ")
            token = token.strip()
            expected = (self.config.token_sha256 or "").strip()
            ok = (scheme.lower() == "bearer" and len(token) >= MIN_TOKEN_CHARS and bool(expected)
                  and hmac.compare_digest(token_digest(token), expected))
            if ok:
                self._failures = 0
                return
            self._failures += 1
            if self._failures >= MAX_AUTH_FAILURES:
                self._failures = 0
                self._locked_until = now + AUTH_LOCKOUT_S
            log_event(logger, "test_injection.unauthorized", "authorization refused", level=logging.WARNING,
                      locked=now < self._locked_until)
        raise InjectionRejected("UNAUTHORIZED", "a valid bearer token is required", 401)

    # --- injection --------------------------------------------------------------------

    def inject(self, agent: Any, raw: Any) -> Dict[str, Any]:
        """Hand one request to `agent`. Returns the agent's verdict; raises when refused before it.

        Synchronous on purpose: called from the event loop the agent ticks on,
        so no tick can run between the checks and the assignment.
        """

        with self._lock:
            self.check_available()
            request = validate_request(raw, expected_robot_id=self.config.robot_id)
            if request.task_id in self._task_ids:
                raise InjectionRejected("DUPLICATE_TASK", f"{request.task_id} was injected before; "
                                        "a task id is injected once", 409)
            self._check_conflicts(agent, request)

            if not self._append(self._record("ATTEMPTED", request, request_summary=request.summary())):
                raise InjectionRejected("PROVENANCE_NOT_RECORDED",
                                        "the provenance log could not be written; nothing was injected", 503)

            accepted, reason, mission = self._decide(agent, request)
            event = "ACCEPTED" if accepted else "REJECTED"
            if not self._append(self._record(event, request, reason=reason, mission=mission)):
                if accepted:
                    # Unrecorded provenance is not allowed to drive.
                    _stand_down(agent, "test injection provenance could not be recorded")
                raise InjectionRejected("PROVENANCE_NOT_RECORDED",
                                        "the outcome could not be recorded; any accepted mission was stopped", 503)

            log_event(logger, f"test_injection.{event.lower()}", level=logging.WARNING if accepted else logging.INFO,
                      task_id=request.task_id, test_run_id=request.test_run_id, reason=reason,
                      provenance=PROVENANCE_SOURCE, backend_assigned=False)
            return {
                "outcome": event,
                "reason": reason,
                "task_id": request.task_id,
                "test_run_id": request.test_run_id,
                "provenance": provenance(),
                "mission": mission,
            }

    def _check_conflicts(self, agent: Any, request: InjectionRequest) -> None:
        backend = getattr(agent, "backend", None)
        held = backend.commitments.held() if backend is not None else None
        if held is not None:
            raise InjectionRejected("BACKEND_COMMITMENT_HELD",
                                    "this Rover holds a RobotX commitment; a test task may not replace it", 409)
        snapshot = agent.state.snapshot()
        if (snapshot.mission is not None and snapshot.mission.is_active) or \
                snapshot.mode.value in ("AUTO", "PAUSED"):
            raise InjectionRejected("MISSION_IN_PROGRESS",
                                    f"the Rover is {snapshot.mode.value} with a mission held", 409)
        position = snapshot.position
        if position is not None and position.is_measured and not request.in_region(position.lat_lon):
            raise InjectionRejected(
                "ROVER_OUT_OF_REGION",
                f"the Rover is {distance_m(request.region_center, position.lat_lon):.0f} m from the region "
                f"center (radius {request.region_radius_m:.0f} m)",
                409,
            )

    @staticmethod
    def _decide(agent: Any, request: InjectionRequest) -> Tuple[bool, Optional[str], Optional[Dict[str, Any]]]:
        """The agent's own verdict, exactly as `BackendLink._respond_to_offer` takes it."""

        try:
            decision = agent.assess_offer(request.assignment())
        except Exception:  # noqa: BLE001 - a bug here must decline, not accept
            logger.exception("assess_offer failed for a test injection")
            return False, "AGENT_ERROR", None
        if decision.verdict is not OfferVerdict.ACCEPT:
            return False, decision.reason, None
        try:
            assignment = agent.assign_mission(decision.mission, custody_required=True)
        except MissionRejected as e:
            return False, f"MISSION_REFUSED: {e.reason.value}", None
        except Exception:  # noqa: BLE001
            logger.exception("assign_mission failed for a test injection")
            # It may have got as far as holding the mission; recorded as
            # rejected, it must not be held.
            _stand_down(agent, "test injection assignment failed part-way")
            return False, "AGENT_ERROR", None
        return True, None, assignment.active.to_dict()

    def status(self) -> Dict[str, Any]:
        locked = self.clock() < self._locked_until
        return {
            "enabled": self.config.enabled,
            "available": self.available and not locked,
            "problem": self._problem,
            "locked_out": locked,
            "task_id_prefix": TASK_ID_PREFIX,
            "provenance": provenance(),
            "injected_task_ids": len(self._task_ids),
        }


def _stand_down(agent: Any, reason: str) -> None:
    """Stop a mission that must not run; if even that fails, latch the e-stop."""

    try:
        agent.stop_mission(reason)
    except Exception:  # noqa: BLE001
        logger.exception("stop_mission failed; engaging the emergency stop")
        agent.emergency_stop(reason)
