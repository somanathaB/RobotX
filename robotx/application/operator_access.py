"""Opt-in access control for the local routes that release a stop or advance a mission.

`POST /safety/clear`, `/controller/reset`, `/mission/idle` and
`/mission/custody` undo a stop or move a mission forward. The API has no
authentication (H3 is V2 work), so on loopback any local process can call
them -- and so can any web page open in a browser on the Pi: a cross-site
"simple" POST needs no CORS preflight, and a DNS-rebinding page is same-origin.

Whatever is configured, `main.require_operator` refuses any request to those
four routes that carries an `Origin` header (every browser POST does; curl
never does). With `ROBOTX_OPERATOR_TOKEN_SHA256` set, they also require:

- no `Origin` header (checked here too, for callers outside the route guard);
- a loopback `Host` (`127.x`, `::1` or `localhost`, as the runbook's
  `S=http://127.0.0.1:8000` sends);
- `Authorization: Bearer <token>` whose SHA-256 matches, constant time.

Unset (the default), a non-browser caller is served exactly as before.
Routes that *stop* the Rover -- `/safety/estop`, `/mission/stop`,
`/mission/pause` -- are never guarded: stopping must not depend on a token.

A digest that is set but malformed fails closed: the guarded routes refuse
(503) until it is fixed. That leaves the Rover stopped, never moving.
"""

from __future__ import annotations

import hashlib
import hmac
import ipaddress
import re
from dataclasses import dataclass, field
from typing import Any, Optional, Tuple


MIN_TOKEN_CHARS = 32
_SHA256_HEX = re.compile(r"^[0-9a-f]{64}$")


def valid_digest(value: Optional[str]) -> bool:
    return bool(value) and _SHA256_HEX.match(value.strip()) is not None  # type: ignore[union-attr]


def _loopback_host(host_header: Optional[str]) -> bool:
    host = (host_header or "").strip()
    if not host:
        return False
    bare = host[1:host.find("]")] if host.startswith("[") else host.rsplit(":", 1)[0]
    if bare.lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(bare).is_loopback
    except ValueError:
        return False


@dataclass(frozen=True)
class OperatorAccess:
    token_sha256: Optional[str] = field(default=None, repr=False)

    @classmethod
    def from_settings(cls, settings: Any) -> "OperatorAccess":
        value = settings.operator_token_sha256
        return cls(token_sha256=value if value else None)

    @property
    def required(self) -> bool:
        return self.token_sha256 is not None

    @property
    def problem(self) -> Optional[str]:
        if self.required and not valid_digest(self.token_sha256):
            return "ROBOTX_OPERATOR_TOKEN_SHA256 is not a 64-character lowercase hex SHA-256 digest"
        return None

    def refusal(self, *, authorization: Optional[str], origin: Optional[str],
                host_header: Optional[str]) -> Optional[Tuple[int, str]]:
        """(status, detail) when this request may not use a guarded route, else None."""

        if not self.required:
            return None
        if self.problem is not None:
            return 503, "operator access is misconfigured; guarded routes are refused until it is fixed"
        if origin is not None:
            return 403, "browser requests are refused on this route"
        if not _loopback_host(host_header):
            return 403, "the Host header must name this Pi's loopback address"
        scheme, _, token = (authorization or "").partition(" ")
        token = token.strip()
        expected = (self.token_sha256 or "").strip()
        if scheme.lower() != "bearer" or len(token) < MIN_TOKEN_CHARS or not hmac.compare_digest(
                hashlib.sha256(token.encode("utf-8")).hexdigest(), expected):
            return 401, "a valid operator bearer token is required"
        return None
