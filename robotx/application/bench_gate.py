"""The guarded route for the bench motor pulse (`robotx.control.bench_motion`).

Off unless `ROBOTX_BENCH_MOTION_ENABLED=1`; the routes then answer 404 like an
unknown path. Enabled, every request is refused (503) unless all of these hold:

- the API is bound to a loopback IP address;
- `ROBOTX_BENCH_MOTION_TOKEN_SHA256` is a SHA-256 hex digest, different from
  the operator and injection token digests;
- `ROBOTX_OPERATOR_TOKEN_SHA256` is set (an unconfirmed stop latches the
  e-stop, and clearing it must not be open to every local process);
- `ROBOTX_BENCH_MOTOR_CUTOFF_CONFIRMED=1`: whoever configured this robot
  states that an independent way to remove motor power exists and is fitted.
  Software cannot verify that; this is the deployment's recorded statement,
  and without it the route refuses.

A run request must come from loopback, with no `Origin` (no browser page can
post here), a valid bearer token, and a JSON body that re-states, for this run,
that the wheels are off the ground and the motor cutoff is attended:

    {"wheels_off_ground": true, "motor_cutoff_attended": true,
     "duration_s": <optional, <= configured>, "speed": <optional, <= configured>,
     "task_id": <required with ROBOTX_BENCH_WHEEL_DEMO=1, refused otherwise>}

Nothing here moves anything: the agent decides (`RobotAgent.start_bench_motion`)
and every gate it has still applies. A run is never a mission and is never
reported to the backend as one.
"""

from __future__ import annotations

import hashlib
import hmac
import ipaddress
import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional, Tuple

from robotx.application.operator_access import valid_digest
from robotx.config.logging_setup import log_event

logger = logging.getLogger(__name__)

MAX_BODY_BYTES = 4096
MIN_TOKEN_CHARS = 32
MAX_AUTH_FAILURES = 5
AUTH_LOCKOUT_S = 300.0
_ALLOWED_KEYS = {"wheels_off_ground", "motor_cutoff_attended", "duration_s", "speed", "task_id"}
MAX_TASK_ID_CHARS = 128


class BenchRejected(Exception):
    """A bench request refused before (or by) the agent, with an HTTP status."""

    def __init__(self, code: str, detail: str, status: int = 422) -> None:
        super().__init__(f"{code}: {detail}")
        self.code = code
        self.detail = detail
        self.status = status

    def to_dict(self) -> Dict[str, Any]:
        return {"outcome": "REFUSED", "code": self.code, "detail": self.detail}


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
class BenchGateConfig:
    enabled: bool = False
    token_sha256: Optional[str] = field(default=None, repr=False)
    operator_token_sha256: Optional[str] = field(default=None, repr=False)
    # Digests of the other local tokens, which the bench token must not reuse.
    other_token_digests: Tuple[str, ...] = field(default=(), repr=False)
    motor_cutoff_confirmed: bool = False
    api_host: str = ""

    @classmethod
    def from_settings(cls, settings: Any, *, other_token_digests: Tuple[Optional[str], ...] = ()) -> "BenchGateConfig":
        return cls(
            enabled=bool(settings.bench_motion_enabled),
            token_sha256=settings.bench_motion_token_sha256,
            operator_token_sha256=settings.operator_token_sha256,
            other_token_digests=tuple(d.strip() for d in other_token_digests if d),
            motor_cutoff_confirmed=bool(settings.bench_motor_cutoff_confirmed),
            api_host=settings.api_host,
        )

    def problem(self) -> Optional[str]:
        """Why an enabled gate must refuse everything, or None."""

        if not is_loopback_address(self.api_host):
            return f"ROBOTX_API_HOST={self.api_host!r} is not a loopback IP address; the bench route is loopback-only"
        digest = (self.token_sha256 or "").strip()
        if not valid_digest(digest):
            return "ROBOTX_BENCH_MOTION_TOKEN_SHA256 is not a 64-character lowercase hex SHA-256 digest"
        if not valid_digest(self.operator_token_sha256):
            return "ROBOTX_OPERATOR_TOKEN_SHA256 must be set when the bench route is enabled"
        if digest == (self.operator_token_sha256 or "").strip() or digest in self.other_token_digests:
            return "the bench token must differ from every other local token (operator, injection)"
        if not self.motor_cutoff_confirmed:
            return ("ROBOTX_BENCH_MOTOR_CUTOFF_CONFIRMED is not 1: no independent motor-power cutoff has been "
                    "declared for this robot, so no bench pulse may run")
        return None


class BenchGate:
    def __init__(self, config: BenchGateConfig, *, clock: Callable[[], float] = time.monotonic) -> None:
        self.config = config
        self.clock = clock
        self._lock = threading.Lock()
        self._failures = 0
        self._locked_until = 0.0
        self._problem = config.problem() if config.enabled else None
        if config.enabled:
            if self._problem is None:
                log_event(logger, "bench.route_enabled", "BENCH motor-pulse route is ENABLED on loopback",
                          level=logging.WARNING)
            else:
                log_event(logger, "bench.route_unavailable", "bench route enabled but refuses every request",
                          level=logging.ERROR, reason=self._problem)

    @classmethod
    def from_settings(cls, settings: Any, *, other_token_digests: Tuple[Optional[str], ...] = ()) -> "BenchGate":
        return cls(BenchGateConfig.from_settings(settings, other_token_digests=other_token_digests))

    @property
    def enabled(self) -> bool:
        return self.config.enabled

    def check_origin(self, *, client_host: Optional[str], host_header: Optional[str],
                     origin: Optional[str], server_host: Optional[str]) -> None:
        if not is_loopback_address(client_host) or not is_loopback_address(server_host):
            raise BenchRejected("NOT_LOOPBACK", "the bench route is accepted from loopback only", 403)
        host = (host_header or "").strip()
        bare = host[1:host.find("]")] if host.startswith("[") else host.rsplit(":", 1)[0]
        if not is_loopback_address(bare):
            raise BenchRejected("HOST_NOT_LOOPBACK", "the Host header must be a loopback IP address", 403)
        if origin is not None:
            raise BenchRejected("ORIGIN_PRESENT", "browser requests are refused", 403)

    def check_available(self) -> None:
        if self._problem is not None:
            raise BenchRejected("NOT_CONFIGURED", self._problem, 503)

    def authorize(self, authorization: Optional[str]) -> None:
        now = self.clock()
        with self._lock:
            if now < self._locked_until:
                raise BenchRejected("LOCKED_OUT", "too many failed authorizations; try again later", 429)
            scheme, _, token = (authorization or "").partition(" ")
            token = token.strip()
            expected = (self.config.token_sha256 or "").strip()
            if scheme.lower() == "bearer" and len(token) >= MIN_TOKEN_CHARS and expected \
                    and hmac.compare_digest(token_digest(token), expected):
                self._failures = 0
                return
            self._failures += 1
            if self._failures >= MAX_AUTH_FAILURES:
                self._failures = 0
                self._locked_until = now + AUTH_LOCKOUT_S
            log_event(logger, "bench.unauthorized", "authorization refused", level=logging.WARNING)
        raise BenchRejected("UNAUTHORIZED", "a valid bearer token is required", 401)

    @staticmethod
    def parse(raw: Any, limits: Any) -> Dict[str, Any]:
        """The run parameters from a request body, or BenchRejected (422)."""

        if not isinstance(raw, dict):
            raise BenchRejected("MALFORMED", "the body must be a JSON object")
        unknown = set(raw) - _ALLOWED_KEYS
        if unknown:
            raise BenchRejected("UNKNOWN_FIELD", f"unknown field(s): {', '.join(sorted(unknown))}")
        for key in ("wheels_off_ground", "motor_cutoff_attended"):
            if raw.get(key) is not True:
                raise BenchRejected("ATTESTATION_MISSING",
                                    f"{key} must be true: the wheels off the ground and a person at the motor cutoff")
        out = {"speed": limits.speed, "duration_s": limits.duration_s}
        for key in ("speed", "duration_s"):
            if key in raw:
                value = raw[key]
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise BenchRejected("MALFORMED", f"{key} must be a number")
                out[key] = float(value)
        if "task_id" in raw:
            task_id = raw["task_id"]
            if not isinstance(task_id, str) or not task_id.strip() or len(task_id) > MAX_TASK_ID_CHARS:
                raise BenchRejected("MALFORMED", f"task_id must be a non-empty string of at most {MAX_TASK_ID_CHARS} characters")
            out["task_id"] = task_id
        return out

    def run(self, agent: Any, raw: Any) -> Dict[str, Any]:
        params = self.parse(raw, agent.bench.cfg)
        # Only a request naming a task passes one, so an agent without the
        # task binding is called exactly as before.
        bound = {"task_id": params["task_id"]} if "task_id" in params else {}
        return agent.start_bench_motion(speed=params["speed"], duration_s=params["duration_s"],
                                        source="bench_route", **bound)

    def status(self, agent: Any) -> Dict[str, Any]:
        wheel_demo = getattr(agent, "bench_wheel_status", None)
        return {"enabled": self.enabled, "available": self._problem is None, "problem": self._problem,
                "locked_out": self.clock() < self._locked_until, **agent.bench.status(),
                **({"wheel_demo": wheel_demo()} if callable(wheel_demo) else {})}
