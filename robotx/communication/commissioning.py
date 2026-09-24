"""The one-time pairing step that turns a blank Pi into a known FalconAut robot.

The flow
--------
::

    POST /api/robots/commission      -> 6-digit pairing code, 300 s TTL
              |
    (code goes into the Pi's environment or straight into AUTH)
              |
    socket connects anonymously -> AUTH{robotId, pairingCode} -> AUTH_SUCCESS{token}
              |
    token persisted (robotx.communication.token_store) and used from then on

Why this is a separate command and not something the agent does at boot
-----------------------------------------------------------------------
Commissioning mints a credential. Doing that automatically on every start
would mean any Pi that could reach the backend could enrol itself as a robot,
which is the opposite of what pairing is for. It is a deliberate, operator-run
action, and the 300 s TTL says the backend expects a human to be present.

Run it as::

    python -m robotx.communication.commissioning --url http://backend:3000 \\
        --robot-id robotx-pi

It prints the pairing code and the exact export line to use. If the endpoint
needs an operator credential your deployment issues, pass `--bearer`.

Not reachable from here
-----------------------
This code has **never been run against a live FalconAut instance** -- none is
reachable from this environment. The request shape below follows the endpoint
as specified; the response parser deliberately accepts several shapes so that a
minor difference in envelope does not require a code change to get a code out.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from typing import Any, Dict, Optional


DEFAULT_TIMEOUT_S = 15.0
COMMISSION_PATH = "/api/robots/commission"

# The contract states a 6-digit code with a 300 second lifetime.
PAIRING_CODE_RE = re.compile(r"^\d{6}$")
PAIRING_CODE_TTL_S = 300


class CommissioningError(RuntimeError):
    """Commissioning did not produce a usable pairing code."""


@dataclass(frozen=True)
class PairingCode:
    """A pairing code and what is known about its lifetime."""

    code: str
    expires_in_s: int = PAIRING_CODE_TTL_S
    raw: Optional[Dict[str, Any]] = None

    @property
    def is_well_formed(self) -> bool:
        return bool(PAIRING_CODE_RE.match(self.code))


def extract_pairing_code(data: Any) -> PairingCode:
    """Find the pairing code in a commission response.

    Tolerant by design. The code is a 6-digit string whose *name* may be
    `pairingCode`, `code` or `pairing_code`, and which may sit at the top level
    or one container down. Being strict here would turn a trivial envelope
    difference into a failed bring-up at exactly the moment someone is standing
    at the robot with a 300-second timer running.
    """

    if not isinstance(data, dict):
        raise CommissioningError(f"expected a JSON object, got {type(data).__name__}")

    for key in ("pairingCode", "code", "pairing_code"):
        value = data.get(key)
        if value is not None and str(value).strip():
            ttl = data.get("expiresIn") or data.get("ttl") or PAIRING_CODE_TTL_S
            return PairingCode(
                code=str(value).strip(),
                expires_in_s=int(ttl) if isinstance(ttl, (int, float)) else PAIRING_CODE_TTL_S,
                raw=data,
            )

    for container in ("robot", "data", "result"):
        nested = data.get(container)
        if isinstance(nested, dict):
            try:
                return extract_pairing_code(nested)
            except CommissioningError:
                continue

    raise CommissioningError(
        f"no pairing code in the response; keys were {sorted(data)[:12]}"
    )


def commission(
    *,
    base_url: str,
    robot_id: str,
    name: Optional[str] = None,
    bearer: Optional[str] = None,
    timeout_s: float = DEFAULT_TIMEOUT_S,
) -> PairingCode:
    """Ask the backend to commission this robot and return the pairing code.

    `simulated: false` is asserted because this is a physical robot, and the
    backend's simulator path is explicitly guarded against physical robots --
    presenting as a simulator would take a route that is meant to refuse us.
    """

    import requests  # imported here so the agent never pays for it at runtime

    url = base_url.rstrip("/") + COMMISSION_PATH
    body: Dict[str, Any] = {"robotId": robot_id, "simulated": False}
    if name:
        body["name"] = name

    headers = {
        "Content-Type": "application/json",
        # A native robot client, not a browser. The backend keys some
        # behaviour off client type, and no Origin or Mozilla-style
        # User-Agent is sent for that reason.
        "User-Agent": "robotx-pi",
        "Accept": "application/json",
    }
    if bearer:
        headers["Authorization"] = f"Bearer {bearer}"

    try:
        response = requests.post(url, json=body, headers=headers, timeout=timeout_s)
    except Exception as e:  # noqa: BLE001 - surfaced to an operator, not a loop
        raise CommissioningError(f"could not reach {url}: {e}") from e

    if response.status_code >= 400:
        raise CommissioningError(
            f"{url} returned HTTP {response.status_code}: {response.text[:300]}"
        )

    try:
        payload = response.json()
    except ValueError as e:
        raise CommissioningError(
            f"{url} did not return JSON: {response.text[:200]}"
        ) from e

    return extract_pairing_code(payload)


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m robotx.communication.commissioning",
        description="Commission this Pi as a FalconAut robot and print its pairing code.",
    )
    parser.add_argument("--url", required=True, help="FalconAut base URL, e.g. http://host:3000")
    parser.add_argument("--robot-id", required=True, help="the robot's id (ROBOTX_ROBOT_ID)")
    parser.add_argument("--name", default=None, help="optional human-readable robot name")
    parser.add_argument("--bearer", default=None, help="operator bearer token, if the endpoint needs one")
    parser.add_argument("--json", action="store_true", help="print the raw response as JSON")
    args = parser.parse_args(argv)

    try:
        pairing = commission(
            base_url=args.url, robot_id=args.robot_id, name=args.name, bearer=args.bearer
        )
    except CommissioningError as e:
        print(f"commissioning failed: {e}", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(pairing.raw, indent=2))
        return 0

    if not pairing.is_well_formed:
        print(
            f"warning: {pairing.code!r} is not the expected 6-digit code; "
            f"using it anyway",
            file=sys.stderr,
        )

    print(f"pairing code: {pairing.code}")
    print(f"expires in:   {pairing.expires_in_s}s")
    print()
    print("Start the agent within that window:")
    print(f"  export ROBOTX_PAIRING_CODE={pairing.code}")
    print(f"  export ROBOTX_ROBOT_ID={args.robot_id}")
    print(f"  export ROBOTX_SOCKET_SERVER_URL={args.url}")
    print("  export ROBOTX_SOCKET_ENABLED=1")
    print()
    print("After the first AUTH_SUCCESS the session token is persisted and the")
    print("pairing code is no longer needed; unset it once the robot is online.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
