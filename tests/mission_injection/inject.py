"""Physical-Rover mission injection through the agent's test-only ingress.

    /opt/robotx/RobotX-Pi/venv/bin/python -m tests.mission_injection.inject \
        --fixture FILE --authorize-hardware-test --token-file FILE \
        [--watch-s SECONDS] [--ledger FILE]

Steps, each written to the local test ledger with test provenance:

1. Validate the fixture against the Rover's identity (the agent's own schema).
2. Run the non-motion preflight; stop unless it is READY. That includes the
   agent reporting `GET /test/injection` available.
3. With `--authorize-hardware-test` only: one `POST /test/mission/inject`
   carrying the fixture and the operator's bearer token. The agent decides
   (`assess_offer` -> `assign_mission(custody_required=True)`); this script
   only records the answer.
4. With `--watch-s`: read `GET /state` once a second and record the
   transitions the agent reports (`lifecycle.LifecycleRecorder`).

This script sends exactly one request that changes anything -- step 3 -- and
never stops, resumes, confirms custody or moves the Rover. Stopping is the
operator's: `POST /safety/estop` or `POST /mission/stop` on the agent.

The token file must be readable by its owner only (mode 0600). The token is
never printed or written to the ledger.
"""

from __future__ import annotations

import argparse
import json
import os
import stat
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from tests.mission_injection.fixture import FixtureError, MissionFixture, load_fixture
from tests.mission_injection.ledger import (
    ACCEPTED,
    OUTCOME_UNKNOWN,
    DEFAULT_LEDGER_PATH,
    FIXTURE_REJECTED,
    FIXTURE_VALIDATED,
    INJECTION_BLOCKED,
    PREFLIGHT,
    REJECTED,
    DuplicateInjection,
    Ledger,
)
from tests.mission_injection.lifecycle import LifecycleRecorder
from tests.mission_injection.preflight import API_BASE, LoopbackClient, collect, evaluate, format_report


INJECT_PATH = "/test/mission/inject"
INGRESS = f"POST {INJECT_PATH} (loopback, bearer token) -> RobotAgent.assess_offer -> assign_mission"


class TokenFileError(Exception):
    pass


def read_token(path: str) -> str:
    """The bearer token from an owner-only file. Never echoed."""

    p = Path(path)
    try:
        info = p.stat()
    except OSError as e:
        raise TokenFileError(f"{path}: {e.strerror}") from None
    if not stat.S_ISREG(info.st_mode) or info.st_mode & (stat.S_IRWXG | stat.S_IRWXO):
        raise TokenFileError(f"{path} must be a regular file readable by its owner only (chmod 600)")
    if info.st_uid != os.getuid():
        raise TokenFileError(f"{path} is not owned by the user running the test")
    token = p.read_text(encoding="utf-8").strip()
    if not token:
        raise TokenFileError(f"{path} is empty")
    return token


class InjectionClient:
    """The one write this harness makes: POST the fixture to the loopback ingress."""

    def __init__(self, base: str = API_BASE, timeout_s: float = 10.0) -> None:
        if not base.startswith("http://127.0.0.1:"):
            raise ValueError("test injection is sent to the local agent on 127.0.0.1 only")
        self.base = base.rstrip("/")
        self.timeout_s = timeout_s

    def inject(self, fixture: MissionFixture, token: str) -> Tuple[int, Dict[str, Any]]:
        request = urllib.request.Request(
            self.base + INJECT_PATH,
            data=json.dumps(fixture.raw).encode("utf-8"),
            headers={"Content-Type": "application/json", "Authorization": f"Bearer {token}"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                return response.status, json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            try:
                body = json.loads(e.read().decode("utf-8"))
            except ValueError:
                body = {"detail": "non-JSON error body"}
            return e.code, body


def watch(fixture: MissionFixture, ledger: Ledger, seconds: float,
          client: Optional[LoopbackClient] = None) -> LifecycleRecorder:
    client = client or LoopbackClient()
    recorder = LifecycleRecorder(fixture.task_id, ledger, test_run_id=fixture.test_run_id)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            recorder.observe(client.get("/state"))
        except (OSError, ValueError) as e:
            recorder.events.append({"stage": "STATE_UNREADABLE", "error": type(e).__name__})
        time.sleep(1.0)
    return recorder


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Inject a TEST-INJECTED- mission into the physical Rover.")
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--ledger", default=str(DEFAULT_LEDGER_PATH))
    parser.add_argument("--authorize-hardware-test", action="store_true",
                        help="the operator has reviewed the preflight and authorizes a physical test")
    parser.add_argument("--token-file", help="owner-only file holding the injection bearer token")
    parser.add_argument("--watch-s", type=float, default=0.0, help="record agent transitions for this long")
    args = parser.parse_args(argv)
    ledger = Ledger(args.ledger)

    observed = collect()
    robot_id = (observed.get("config") or {}).get("robot_id")
    try:
        fixture = load_fixture(args.fixture, expected_robot_id=robot_id)
    except FixtureError as e:
        ledger.append(FIXTURE_REJECTED, fixture_path=args.fixture, code=e.code, detail=e.detail)
        print(f"FIXTURE REJECTED {e.code}: {e.detail}")
        return 2
    ledger.append(FIXTURE_VALIDATED, fixture=fixture, fixture_path=args.fixture)

    previous = ledger.previous_attempt(fixture.task_id)
    if previous is not None:
        print(f"DUPLICATE: {fixture.task_id} was already injected at {previous.get('at')}; use a new task_id")
        return 2

    report = evaluate(observed, fixture=fixture)
    ledger.append(PREFLIGHT, fixture=fixture, report=report.to_dict())
    print(format_report(report))
    print()

    if not args.authorize_hardware_test:
        print("NOT_TESTED: no --authorize-hardware-test; fixture validated and preflight run, nothing injected.")
        return 1
    if report.verdict != "READY":
        ledger.append(INJECTION_BLOCKED, fixture=fixture, stage="PREFLIGHT",
                      blockers=[c.name for c in report.blockers()])
        print("BLOCKED: preflight is not READY; injection not attempted. Blocking: "
              + ", ".join(c.name for c in report.blockers()))
        return 3
    if not args.token_file:
        print("BLOCKED: --token-file is required to inject.")
        return 3
    try:
        token = read_token(args.token_file)
    except TokenFileError as e:
        ledger.append(INJECTION_BLOCKED, fixture=fixture, stage="TOKEN", reason=str(e))
        print(f"BLOCKED: {e}")
        return 3

    try:
        ledger.claim_attempt(fixture, ingress=INGRESS)
    except DuplicateInjection as e:
        print(f"DUPLICATE: {e}")
        return 2
    try:
        status, body = InjectionClient().inject(fixture, token)
    except (OSError, ValueError) as e:
        # The agent may have decided and the answer been lost: never guess which.
        ledger.append(OUTCOME_UNKNOWN, fixture=fixture, error=type(e).__name__)
        print(f"OUTCOME UNKNOWN ({type(e).__name__}): the request may have reached the agent. Check "
              "GET /state (mission.task_id) and the agent's log before doing anything else.")
        return 5
    accepted = status == 200 and body.get("outcome") == "ACCEPTED"
    # REJECTED is the agent's own verdict; REFUSED never reached assess_offer.
    decided_by = ("the running agent (assess_offer/assign_mission)" if body.get("outcome") in ("ACCEPTED", "REJECTED")
                  else "the injection gate, before the agent")
    ledger.append(ACCEPTED if accepted else REJECTED, fixture=fixture, http_status=status, response=body,
                  decided_by=decided_by)
    print(f"{'ACCEPTED' if accepted else 'NOT ACCEPTED'} (HTTP {status}): "
          f"{body.get('reason') or body.get('code') or ''} {body.get('detail') or ''}".rstrip())
    if not accepted:
        return 4

    if args.watch_s > 0:
        recorder = watch(fixture, ledger, args.watch_s)
        print("Observed stages: " + ", ".join(recorder.stages()))
        print(f"Completed (agent-reported): {recorder.completed}; "
              f"backend link: {recorder.backend_telemetry()} (the Pi's own link status)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
