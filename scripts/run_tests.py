"""RobotX-Pi test runner: named suites, PASS / FAIL / SKIP / NOT TESTED, never overclaiming.

    venv/bin/python -m scripts.run_tests                  # every offline suite
    venv/bin/python -m scripts.run_tests unit safety      # chosen suites
    venv/bin/python -m scripts.run_tests --live preflight # + the read-only live preflight
    venv/bin/python -m scripts.run_tests --list

Suites (offline ones touch no hardware, no serial port, no backend; loopback only):

  unit        every unit test (tests/unit)
  mission     OFFER -> assessment -> assignment -> navigation -> custody -> completion,
              including a real Socket.IO link to a SIMULATED backend
  navigation  localization (GPS heading), navigator, decision layer, progress and
              obstacle monitors: position + route -> motion intent
  telemetry   Pi -> backend wire: TELEMETRY status/safety/readiness/faults, transition
              frames, HEARTBEAT lease, custody/completion reports, the signature vector
  delivery    a genuine signed OFFER over a real Socket.IO link to a SIMULATED backend
              and SIMULATED ESP32: admission, delivery, WITHDRAW, PAUSE/RESUME, task
              cancellation, backend loss, ESP32 reboot
  safety      safety gate, e-stop latch, interrupted-mission recovery, obstacle and
              navigation stops, operator-route guards
  esp32       ESP32 protocol, link, ACK/timeout/re-proof, applied-motion monitoring,
              over fake transports and the SIMULATED firmware model
  injection   guarded test-mission injection (unit + real agent process) and the
              end-to-end simulated mission script, every scenario
  integration every integration test (tests/integration)
  static      byte-compile everything; mypy on the modules this work added
  preflight   READ-ONLY report on the RUNNING agent (needs --live): GET-only loopback,
              no POST, no serial port, no service change

Statuses:
  PASS        ran, nothing failed (skips are counted and shown)
  FAIL        a test failed or errored, the suite could not run, or (preflight) the
              live rover is not ready for a motion test -- that last one is a
              readiness verdict, not a software failure, and is labelled as such
  SKIP        nothing ran (every test skipped, or a prerequisite is missing)
  NOT TESTED  not exercised by this runner at all -- never a pass

Simulated suites prove software behaviour against simulated feedback. They never
verify physical movement or a delivery; the closing NOT TESTED list says so on
every run.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import unittest
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

REPO = Path(__file__).resolve().parent.parent

SUITES: Dict[str, Dict[str, Any]] = {
    "unit": {"discover": "tests/unit"},
    "mission": {"modules": [
        "tests.unit.test_mission", "tests.unit.test_mission_admission", "tests.unit.test_engine_offer",
        "tests.unit.test_resume_commitment_binding", "tests.unit.test_readiness_telemetry",
        "tests.unit.test_local_api_admission", "tests.unit.test_agent_backend_commands",
        "tests.unit.test_commands", "tests.unit.test_navigation", "tests.unit.test_navigation_synthetic",
        "tests.integration.test_simulated_delivery", "tests.integration.test_injection_backend_process",
        "tests.unit.test_delivery_telemetry",
    ]},
    "navigation": {"modules": [
        "tests.unit.test_gps_and_position", "tests.unit.test_navigation", "tests.unit.test_navigation_synthetic",
        "tests.unit.test_navigation_progress", "tests.unit.test_motion_and_decision", "tests.unit.test_local_frame",
        "tests.unit.test_obstacle_recovery", "tests.unit.test_esp32_gps_path",
    ]},
    "telemetry": {"modules": [
        "tests.unit.test_protocol", "tests.unit.test_readiness_telemetry", "tests.unit.test_delivery_telemetry",
        "tests.unit.test_gate3_contract", "tests.unit.test_pi_backend_boundary", "tests.unit.test_backend_link",
        "tests.unit.test_state_telemetry_health", "tests.unit.test_demo_battery",
    ]},
    "delivery": {"modules": ["tests.integration.test_simulated_delivery"]},
    "safety": {"modules": [
        "tests.unit.test_safety_gate", "tests.unit.test_mission_recovery", "tests.unit.test_obstacle_recovery",
        "tests.unit.test_navigation_progress", "tests.unit.test_motion_and_decision",
        "tests.unit.test_audit_regressions", "tests.unit.test_operator_access", "tests.unit.test_perception_people",
    ]},
    "esp32": {"modules": [
        "tests.unit.test_esp32_protocol", "tests.unit.test_esp32_link", "tests.unit.test_esp32_agent",
        "tests.unit.test_esp32_boundary", "tests.unit.test_front_range", "tests.unit.test_applied_motion",
        "tests.unit.test_esp32_gps_path", "tests.integration.test_esp32_sim_link",
    ]},
    "injection": {"modules": [
        "tests.unit.test_injection_endpoint", "tests.unit.test_injection_hardening",
        "tests.unit.test_mission_injection", "tests.unit.test_live_preflight",
        "tests.integration.test_injection_process", "tests.integration.test_injection_sandbox",
    ], "scripts": [["-m", "scripts.simulated_mission", "--json"]]},
    "integration": {"discover": "tests/integration"},
    "static": {"static": True},
    "preflight": {"live": True},
}
DEFAULT = ("unit", "mission", "navigation", "telemetry", "delivery", "safety", "esp32", "injection",
           "integration", "static")

# Modules added by this work, held to a clean mypy run. The rest of the
# package has pre-existing mypy findings, reported as a count only.
MYPY_CLEAN = ("robotx/control/applied_motion.py", "robotx/mission/admission.py", "robotx/esp32/state.py",
              "robotx/diagnostics/faults.py")

NOT_TESTED = (
    "physical wheel actuation: no motor was driven; `applied` values are the firmware model's, not the rover's",
    "the real ESP32 (f10bb8f7208e) answering DRIVE with ACCEPTED/GATED and non-zero applied output",
    "a real RobotX backend OFFER to this robot, and its lease / RESUME behaviour",
    "GPS outdoors on the rover, and arrival judged on it",
    "a physical emergency stop (none exists on this rover; only the software latch)",
)

RESULT_TAG = "RUN_TESTS_RESULT_JSON:"


# --- child: run unittest in a fresh interpreter, report counts as JSON --------------


class _Result(unittest.TextTestResult):
    def __init__(self, *a: Any, **k: Any) -> None:
        super().__init__(*a, **k)
        self.passed = 0

    def addSuccess(self, test: unittest.TestCase) -> None:
        super().addSuccess(test)
        self.passed += 1


def _child(spec: Dict[str, Any]) -> int:
    loader = unittest.TestLoader()
    if "discover" in spec:
        suite = loader.discover(spec["discover"], top_level_dir=str(REPO))
    else:
        suite = unittest.TestSuite(loader.loadTestsFromName(m) for m in spec["modules"])
    with open(os.devnull, "w") as sink:
        runner = unittest.TextTestRunner(stream=sink, resultclass=_Result, verbosity=0)
        result = runner.run(suite)
    def ids(items: List[Any]) -> List[str]:
        return [t.id() if hasattr(t, "id") else str(t) for t, _ in items]
    print(RESULT_TAG + json.dumps({
        "ran": result.testsRun, "passed": result.passed, "failures": ids(result.failures),
        "errors": ids(result.errors), "skipped": len(result.skipped),
        "skip_reasons": sorted({reason for _, reason in result.skipped})[:5],
        "details": [tb for _, tb in (result.failures + result.errors)][:3],
    }))
    return 0


# --- parent ---------------------------------------------------------------------------


def _run_unittest(name: str, spec: Dict[str, Any]) -> Dict[str, Any]:
    started = time.monotonic()
    proc = subprocess.run([sys.executable, "-m", "scripts.run_tests", "--child", json.dumps(spec)],
                          cwd=REPO, capture_output=True, text=True, timeout=1800)
    lines = [line for line in proc.stdout.splitlines() if line.startswith(RESULT_TAG)]
    out: Dict[str, Any] = {"suite": name, "seconds": round(time.monotonic() - started, 1)}
    if not lines:
        out.update(status="FAIL", summary="the suite did not run", detail=(proc.stderr or proc.stdout)[-2000:])
        return out
    r = json.loads(lines[-1][len(RESULT_TAG):])
    bad = len(r["failures"]) + len(r["errors"])
    if bad:
        status = "FAIL"
    elif r["ran"] == 0 or r["skipped"] == r["ran"]:
        status = "SKIP"
    else:
        status = "PASS"
    out.update(status=status, counts={k: r[k] if isinstance(r[k], int) else len(r[k])
                                      for k in ("ran", "passed", "failures", "errors", "skipped")},
               failed=r["failures"] + r["errors"], skip_reasons=r["skip_reasons"], detail="\n".join(r["details"]),
               summary=f"{r['ran']} ran, {r['passed']} passed, {len(r['failures'])} failed, "
                       f"{len(r['errors'])} errors, {r['skipped']} skipped")
    return out


def _run_scripts(spec: Dict[str, Any]) -> List[Dict[str, Any]]:
    results = []
    for argv in spec.get("scripts", []):
        started = time.monotonic()
        proc = subprocess.run([sys.executable, *argv], cwd=REPO, capture_output=True, text=True, timeout=900)
        name = " ".join(argv[1:2])
        try:
            report = json.loads(proc.stdout)
            scenarios = report["summary"]["simulated_scenarios"]
            summary = ", ".join(f"{k}={v}" for k, v in scenarios.items())
        except (ValueError, KeyError):
            scenarios, summary = {}, (proc.stderr or proc.stdout)[-500:]
        results.append({"suite": f"  script {name}", "seconds": round(time.monotonic() - started, 1),
                        "status": "PASS" if proc.returncode == 0 and scenarios else "FAIL",
                        "summary": f"SIMULATED scenarios: {summary}"})
    return results


def _run_static() -> Dict[str, Any]:
    started = time.monotonic()
    problems = []
    compiled = subprocess.run([sys.executable, "-m", "compileall", "-q", "robotx", "tests", "scripts"],
                              cwd=REPO, capture_output=True, text=True)
    if compiled.returncode != 0:
        problems.append("compileall: " + (compiled.stdout + compiled.stderr)[-1000:])
    mypy = subprocess.run([sys.executable, "-m", "mypy", "--ignore-missing-imports", "--follow-imports=silent",
                           *MYPY_CLEAN], cwd=REPO, capture_output=True, text=True)
    if "No module named mypy" in mypy.stderr:
        mypy_note = "mypy not installed: NOT TESTED"
    elif mypy.returncode != 0:
        problems.append("mypy: " + mypy.stdout[-1500:])
        mypy_note = "mypy found errors in the new modules"
    else:
        mypy_note = f"mypy clean on {len(MYPY_CLEAN)} new modules"
    whole = subprocess.run([sys.executable, "-m", "mypy", "--ignore-missing-imports", "robotx"],
                           cwd=REPO, capture_output=True, text=True)
    tail = (whole.stdout.strip().splitlines() or ["?"])[-1]
    return {"suite": "static", "seconds": round(time.monotonic() - started, 1),
            "status": "FAIL" if problems else "PASS", "detail": "\n".join(problems),
            "summary": f"compileall ok; {mypy_note}; whole package (pre-existing, info only): {tail}"}


def _run_preflight(live: bool) -> Dict[str, Any]:
    if not live:
        return {"suite": "preflight", "status": "NOT TESTED",
                "summary": "live preflight not requested (add --live; it only reads the running agent)"}
    started = time.monotonic()
    proc = subprocess.run([sys.executable, "-m", "tests.mission_injection.preflight", "--json"],
                          cwd=REPO, capture_output=True, text=True, timeout=60)
    out: Dict[str, Any] = {"suite": "preflight", "seconds": round(time.monotonic() - started, 1)}
    try:
        report = json.loads(proc.stdout)
    except ValueError:
        out.update(status="FAIL", summary="the preflight did not produce a report",
                   detail=(proc.stderr or proc.stdout)[-1500:])
        return out
    checks = report.get("checks", [])
    unreachable = [c for c in checks if c["name"] == "local_api_read" and c["result"] != "PASS"]
    if unreachable and "Connection refused" in unreachable[0]["detail"]:
        out.update(status="SKIP", summary="no agent is answering on 127.0.0.1:8000", checks=checks)
        return out
    verdict = report.get("verdict")
    out.update(status="PASS" if verdict == "READY" else "FAIL", checks=checks,
               summary=f"live rover {verdict}" + ("" if verdict == "READY" else
                        " for a motion test (a readiness verdict, not a software test failure)"))
    return out


def _print(results: List[Dict[str, Any]], verbose: bool) -> None:
    print("\nRobotX-Pi test results")
    print("=" * 78)
    for r in results:
        print(f"  {r['status']:<11} {r['suite']:<22} {r.get('summary', '')}"
              + (f"  [{r['seconds']}s]" if "seconds" in r else ""))
        for name in r.get("failed", [])[:10]:
            print(f"{'':16}FAILED: {name}")
        if r["status"] == "FAIL" and r.get("detail") and (verbose or not r.get("failed")):
            print("\n".join(f"{'':16}{line}" for line in r["detail"].splitlines()[-25:]))
        for c in r.get("checks", []):
            if verbose or c["result"] != "PASS":
                tag = c["result"] + ("" if c.get("required", True) else " (info)")
                print(f"{'':16}{tag:<18} {c['name']}: {c['detail'][:150]}")
    print("-" * 78)
    for item in NOT_TESTED:
        print(f"  {'NOT TESTED':<11} {item}")
    print("=" * 78)
    print("Simulated tests verify software against SIMULATED hardware only. They do not show that")
    print("the rover moved or that any delivery happened.")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("suites", nargs="*", help=f"default: {' '.join(DEFAULT)}")
    parser.add_argument("--live", action="store_true", help="allow the read-only live preflight")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--verbose", "-v", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--child", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)

    if args.child:
        return _child(json.loads(args.child))
    if args.list:
        for name, spec in SUITES.items():
            print(f"{name:<12} {spec}")
        return 0
    names = args.suites or list(DEFAULT) + (["preflight"] if args.live else [])
    unknown = [n for n in names if n not in SUITES]
    if unknown:
        parser.error(f"unknown suite(s): {', '.join(unknown)}")

    results: List[Dict[str, Any]] = []
    for name in names:
        spec = SUITES[name]
        if not args.json:
            print(f"running {name} ...", flush=True)
        if spec.get("live"):
            results.append(_run_preflight(args.live))
        elif spec.get("static"):
            results.append(_run_static())
        else:
            results.append(_run_unittest(name, spec))
            results.extend(_run_scripts(spec))
    if "preflight" not in names:
        results.append(_run_preflight(False))

    if args.json:
        print(json.dumps({"results": results, "not_tested": NOT_TESTED}, indent=2))
    else:
        _print(results, args.verbose)
    return 1 if any(r["status"] == "FAIL" for r in results) else 0


if __name__ == "__main__":
    sys.exit(main())
