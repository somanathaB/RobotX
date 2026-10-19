"""Run a TEST-INJECTED mission end to end against SIMULATED hardware.

    venv/bin/python -m scripts.simulated_mission                 # every scenario
    venv/bin/python -m scripts.simulated_mission --scenario estop [--verbose]

What is real: the agent and its own loop, mission admission (assess_offer and
route admission), assignment, navigation, decision, safety gate, obstacle /
progress / applied-motion monitors, the ESP32 link protocol, and the local
HTTP routes -- `POST /test/mission/inject` (token, loopback and provenance
checks), `/mission/custody`, `/safety/estop`, `/safety/clear`,
`/mission/resume` -- called in-process through their FastAPI handlers.

What is simulated: the ESP32 firmware, wheels, GPS and camera
(`tests.fixtures.sim_rover`), the clock-driven world, and the operator.

What it never does: open a serial port or any device, bind a socket, contact a
backend, read /etc/robotx, or touch the running service. The settings are an
explicit dictionary, not the host environment. The injection log goes to a
temporary directory that is removed afterwards.

A PASS here is not hardware evidence: physical movement and a real delivery
are reported NOT TESTED, always.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import secrets
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import httpx

import robotx.application.main as api
from robotx.application.injection import InjectionConfig, InjectionGate, token_digest
from robotx.application.operator_access import OperatorAccess
from robotx.mission.mission import MissionStatus
from robotx.state.robot_state import OperatingMode
from tests.fixtures import esp32 as fx
from tests.fixtures.sim_rover import SimulatedRover, short_route
from tests.fixtures.task_assign import offset

ROBOT = "RBT-SIM-0001"
START = (12.9700000, 77.5900000)
SCENARIOS = ("nominal", "estop", "reboot", "obstacle", "applied-zero", "gated", "route-far")


def pt(p: Tuple[float, float]) -> Dict[str, float]:
    return {"lat": p[0], "lon": p[1]}


def injection_body(task_id: str, start: Tuple[float, float]) -> Dict[str, Any]:
    pickup, drop, to_pickup, to_drop = short_route(start)
    return {
        "schema_version": 1, "test_run_id": "simulated-mission", "task_id": task_id, "robot_id": ROBOT,
        "region": {"center": pt(START), "radius_m": 300},
        "pickup": pt(pickup), "drop": pt(drop),
        "path_to_pickup": [pt(p) for p in to_pickup], "path_to_drop": [pt(p) for p in to_drop],
    }


class Run:
    """One scenario: the rover, an in-process API client, a timeline and expectations."""

    def __init__(self, name: str, tmp: Path, verbose: bool) -> None:
        self.name = name
        self.verbose = verbose
        self.rover = SimulatedRover(START, robot_id=ROBOT)
        assert isinstance(self.rover.port, fx.FakePort), "the simulated ESP32 must be an in-memory port"
        self.token, self.operator = secrets.token_hex(16), secrets.token_hex(16)
        self.gate = InjectionGate(InjectionConfig(
            enabled=True, token_sha256=token_digest(self.token), log_path=str(tmp / f"{name}.jsonl"),
            api_host="127.0.0.1", robot_id=ROBOT, operator_token_sha256=token_digest(self.operator)))
        self.t0 = time.monotonic()
        self.timeline: List[str] = []
        self.expectations: List[Tuple[str, bool, str]] = []
        self._last: Tuple[Any, ...] = ()

    # --- plumbing --------------------------------------------------------------

    def note(self, text: str) -> None:
        line = f"  t+{time.monotonic() - self.t0:5.1f}s  {text}"
        self.timeline.append(line)
        if self.verbose:
            print(line)

    def expect(self, what: str, ok: bool, detail: str = "") -> bool:
        self.expectations.append((what, bool(ok), detail))
        self.note(f"{'PASS' if ok else 'FAIL'}: {what}" + (f" ({detail})" if detail and not ok else ""))
        return bool(ok)

    def watch(self) -> None:
        snap = self.rover.agent.state.snapshot()
        m = snap.mission
        key = (snap.mode, m and m.status, snap.last_error)
        if key != self._last:
            self._last = key
            self.note(f"mode={snap.mode.value:<7} mission={(m.status.value if m else None)!s:<9} "
                      f"error={snap.last_error!r}")

    async def run(self, seconds: float, until: Optional[Callable[[], bool]] = None) -> bool:
        def check() -> bool:
            self.watch()
            return bool(until and until())
        reached = await self.rover.run(seconds, until=check)
        return reached if until else True

    async def call(self, method: str, path: str, payload: Any = None, *, token: Optional[str] = None) -> Tuple[int, Any]:
        api.state.agent, api.state.injection = self.rover.agent, self.gate
        api.state.operator_access = OperatorAccess(token_sha256=token_digest(self.operator))
        transport = httpx.ASGITransport(app=api.app, client=("127.0.0.1", 50000))
        headers = {"Authorization": f"Bearer {token}"} if token else {}
        async with httpx.AsyncClient(transport=transport, base_url="http://127.0.0.1:8000") as client:
            r = await client.request(method, path, json=payload, headers=headers)
        body = r.json() if r.content else None
        self.note(f"{method} {path} -> {r.status_code}")
        return r.status_code, body

    def mission(self):
        return self.rover.agent.state.snapshot().mission

    def status_is(self, status: MissionStatus) -> Callable[[], bool]:
        return lambda: self.mission() is not None and self.mission().status is status

    def mode_is(self, mode: OperatingMode) -> Callable[[], bool]:
        return lambda: self.rover.agent.mode is mode

    async def no_drive_for(self, seconds: float, why: str) -> None:
        mark = self.rover.mark()
        await self.run(seconds)
        self.expect(f"no DRIVE for {seconds:g}s {why}", not self.rover.drives(mark),
                    f"{len(self.rover.drives(mark))} DRIVE frames")

    # --- the mission, start to finish --------------------------------------------

    async def inject(self, start=START) -> Tuple[int, Any]:
        return await self.call("POST", "/test/mission/inject", injection_body(f"TEST-INJECTED-SIM-{self.name}", start),
                               token=self.token)

    async def custody(self, kind: str) -> bool:
        status, _ = await self.call("POST", "/mission/custody", {"kind": kind}, token=self.operator)
        return self.expect(f"operator custody {kind} recorded", status == 200)

    async def to_completion(self) -> None:
        if not self.expect("drop leg reached", await self.run(30.0, self.status_is(MissionStatus.AT_DROP))):
            return
        await self.custody("RELEASED")
        self.expect("mission COMPLETE (simulated)", await self.run(3.0, self.status_is(MissionStatus.COMPLETE)))
        self.expect("no reverse or pivot DRIVE was ever written", self.rover.min_drive_units >= 0)


async def scenario(name: str, run: Run) -> None:
    rover = run.rover
    await rover.start()
    ready = await run.run(5.0, lambda: rover.agent.readiness_blockers() == [])
    if not run.expect("agent reports itself ready (simulated hardware)", ready, str(rover.agent.readiness_blockers())):
        return

    if name == "route-far":
        status, body = await run.inject(start=offset(START, north_m=-150.0))
        run.expect("a route that does not start at the rover is refused (409)", status == 409, str(body))
        run.expect("refusal names ROUTE_START_TOO_FAR", "ROUTE_START_TOO_FAR" in json.dumps(body), str(body))
        run.expect("no mission was assigned", run.mission() is None)
        return

    if name == "applied-zero":
        rover.firmware.applied_zero = True
    if name == "gated":
        rover.firmware.gate_reason = "REAR_SENSOR_FAULT"

    status, body = await run.inject()
    if not run.expect("injected test mission ACCEPTED by the agent", status == 200, str(body)):
        return
    run.expect("mode AUTO after admission", rover.agent.mode is OperatingMode.AUTO)

    if name in ("applied-zero", "gated"):
        paused = await run.run(5.0, run.mode_is(OperatingMode.PAUSED))
        error = rover.agent.state.snapshot().last_error or ""
        run.expect("mission PAUSED by the applied-motion monitor", paused and error.startswith("controller: "), error)
        await run.no_drive_for(2.0, "after the controller pause")
        return

    if name == "obstacle":
        await run.run(1.0)
        rover.obstacle = True
        run.note("SIMULATED obstacle in the camera corridor")
        await run.no_drive_for(3.0, "while the obstacle is in view")
        rover.obstacle = False
        run.note("obstacle cleared")

    if not run.expect("pickup reached", await run.run(30.0, run.status_is(MissionStatus.AT_PICKUP))):
        return
    await run.no_drive_for(1.0, "while holding at the pickup for custody")
    await run.custody("ACQUIRED")
    await run.run(3.0, run.status_is(MissionStatus.TO_DROP))
    await run.run(1.5)

    if name in ("estop", "reboot"):
        if name == "estop":
            await run.call("POST", "/safety/estop")
        else:
            rover.uptime = 400
            run.note("SIMULATED ESP32 reboot (TELEMETRY uptime went backwards)")
        run.expect("e-stop latched, mode STOPPED", await run.run(2.0, lambda: rover.agent.emergency_stopped
                                                                and rover.agent.mode is OperatingMode.STOPPED))
        run.expect("mission held and marked interrupted", bool(run.mission() and run.mission().interrupted),
                   str(run.mission() and run.mission().interrupted_by))
        run.expect("lease renewal withheld while latched", not rover.agent.mission_lease_renewable())
        await run.no_drive_for(2.0, "while latched")
        status, _ = await run.call("POST", "/mission/resume")
        run.expect("RESUME refused while latched (409)", status == 409)
        status, _ = await run.call("POST", "/safety/clear", token=run.operator)
        run.expect("operator clear accepted", status == 200)
        run.expect("cleared -> PAUSED, never AUTO", rover.agent.mode is OperatingMode.PAUSED)
        await run.no_drive_for(2.0, "after clearing, before RESUME")
        status, _ = await run.call("POST", "/mission/resume")
        run.expect("explicit RESUME -> AUTO", status == 200 and rover.agent.mode is OperatingMode.AUTO)

    await run.to_completion()


async def run_one(name: str, tmp: Path, verbose: bool) -> Run:
    run = Run(name, tmp, verbose)
    try:
        await scenario(name, run)
    finally:
        await run.rover.stop()
        api.state.agent = api.state.injection = api.state.operator_access = None
    return run


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Simulated end-to-end TEST-INJECTED mission (no hardware).")
    parser.add_argument("--scenario", choices=SCENARIOS + ("all",), default="all")
    parser.add_argument("--verbose", action="store_true", help="print the timeline as it happens")
    parser.add_argument("--json", action="store_true", help="print the results as JSON")
    args = parser.parse_args(argv)

    import logging

    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)
    names = SCENARIOS if args.scenario == "all" else (args.scenario,)
    results = []
    with tempfile.TemporaryDirectory(prefix="robotx-sim-") as tmp:
        for name in names:
            if not args.json:
                print(f"\n=== scenario: {name} (SIMULATED hardware) ===")
            run = asyncio.run(run_one(name, Path(tmp), args.verbose and not args.json))
            ok = bool(run.expectations) and all(passed for _, passed, _ in run.expectations)
            results.append({"scenario": name, "result": "PASS" if ok else "FAIL",
                            "expectations": [{"what": w, "pass": p, "detail": d} for w, p, d in run.expectations],
                            "timeline": run.timeline})
            if not args.json:
                if not args.verbose:
                    print("\n".join(run.timeline))
                print(f"--- {name}: {'PASS' if ok else 'FAIL'} "
                      f"({sum(p for _, p, _ in run.expectations)}/{len(run.expectations)} expectations)")

    failed = [r["scenario"] for r in results if r["result"] != "PASS"]
    summary = {
        "simulated_scenarios": {r["scenario"]: r["result"] for r in results},
        "physical_movement": "NOT TESTED",
        "real_backend_assigned_delivery": "NOT TESTED",
        "note": "simulated ESP32, wheels, GPS and camera; no serial port, socket or backend was used",
    }
    if args.json:
        print(json.dumps({"results": results, "summary": summary}, indent=2))
    else:
        print("\nSUMMARY")
        for r in results:
            print(f"  {r['result']:<10} simulated: {r['scenario']}")
        print("  NOT TESTED physical wheel movement (no motor was driven)")
        print("  NOT TESTED a real backend-assigned delivery")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
