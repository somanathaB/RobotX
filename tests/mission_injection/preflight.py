"""Non-motion preflight of the running RobotX agent. Reads; never writes.

    /opt/robotx/RobotX-Pi/venv/bin/python -m tests.mission_injection.preflight \
        [--fixture FILE] [--json] [--ledger FILE]

Everything comes from the agent that is already running, so nothing here
competes for the ESP32 UART, the camera or the backend session:

- `GET /health /state /config /backend /test/injection` on 127.0.0.1 -- an
  allowlist; any other path or method is refused by `LoopbackClient` itself
  (no POST, no /camera). `/test/injection` answers 404 while injection is off.
- `systemctl is-active robotx-agent` and `timedatectl show` (read-only).

No serial port is opened, no command reaches the ESP32, no service is touched.

Each check is PASS, FAIL, BLOCKED or NOT_TESTED. BLOCKED means a condition
that by itself forbids the test (configuration, an obstacle, an ingress that
does not exist); FAIL means something measured is wrong; NOT_TESTED means this
preflight cannot observe it, which is never a pass. The verdict is READY only
when every required check is PASS.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

from tests.mission_injection.fixture import FixtureError, MissionFixture, load_fixture


EXPECTED_BACKEND_URL = "https://robotx-admin-dashboard.onrender.com"
EXPECTED_SERVICE = "robotx-agent"
API_BASE = "http://127.0.0.1:8000"
READ_PATHS = ("/health", "/state", "/config", "/backend", "/test/injection")
# Absent (404) when the agent's test injection is disabled: an answer, not an error.
OPTIONAL_PATHS = ("/test/injection",)


class Result(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    BLOCKED = "BLOCKED"
    NOT_TESTED = "NOT_TESTED"


@dataclass(frozen=True)
class Check:
    name: str
    result: Result
    detail: str
    # False only for checks that cannot hold back a test: informational, and
    # stated as NOT_TESTED rather than passed.
    required: bool = True

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["result"] = self.result.value
        return d


@dataclass(frozen=True)
class PreflightReport:
    checks: List[Check]
    observed_at: float

    @property
    def verdict(self) -> str:
        return "READY" if all(c.result is Result.PASS for c in self.checks if c.required) else "BLOCKED"

    def blockers(self) -> List[Check]:
        return [c for c in self.checks if c.required and c.result is not Result.PASS]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "verdict": self.verdict,
            "observed_at": self.observed_at,
            "checks": [c.to_dict() for c in self.checks],
            "blockers": [c.name for c in self.blockers()],
        }


# --- collection ---------------------------------------------------------------


class LoopbackClient:
    """GET on the agent's loopback API, for the allowlisted paths only."""

    def __init__(self, base: str = API_BASE, timeout_s: float = 5.0) -> None:
        if not base.startswith(("http://127.0.0.1:", "http://localhost:")):
            raise ValueError("the preflight reads the local agent only")
        self.base = base.rstrip("/")
        self.timeout_s = timeout_s

    def get(self, path: str) -> Dict[str, Any]:
        if path not in READ_PATHS:
            raise ValueError(f"{path} is not a read-only preflight path")
        request = urllib.request.Request(self.base + path, method="GET")
        with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
            return json.loads(response.read().decode("utf-8"))


def _run(argv: List[str]) -> Optional[str]:
    try:
        return subprocess.run(argv, capture_output=True, text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def collect(client: Optional[LoopbackClient] = None,
            run: Callable[[List[str]], Optional[str]] = _run) -> Dict[str, Any]:
    client = client or LoopbackClient()
    observed: Dict[str, Any] = {"errors": {}}
    for path in READ_PATHS:
        key = path.strip("/").replace("/", "_")
        try:
            observed[key] = client.get(path)
        except urllib.error.HTTPError as e:
            observed[key] = None
            observed[f"{key}_http_status"] = e.code
            if not (path in OPTIONAL_PATHS and e.code == 404):
                observed["errors"][path] = f"HTTP {e.code}"
        except (OSError, ValueError, urllib.error.URLError) as e:
            observed[key] = None
            observed["errors"][path] = f"{type(e).__name__}: {e}"
    observed["service_active"] = run(["systemctl", "is-active", EXPECTED_SERVICE])
    ntp = run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"])
    observed["clock_synchronized"] = None if ntp is None else ntp == "yes"
    observed["collected_at"] = time.time()
    return observed


# --- evaluation (pure) --------------------------------------------------------


def _get(data: Any, *path: str) -> Any:
    for key in path:
        if not isinstance(data, dict):
            return None
        data = data.get(key)
    return data


def evaluate(observed: Dict[str, Any], *, fixture: Optional[MissionFixture] = None,
             expected_backend: str = EXPECTED_BACKEND_URL) -> PreflightReport:
    checks: List[Check] = []

    def add(name: str, result: Result, detail: str, required: bool = True) -> None:
        checks.append(Check(name, result, detail, required))

    health, state, config, backend = (observed.get(k) for k in ("health", "state", "config", "backend"))

    # Agent and API.
    service = observed.get("service_active")
    add("agent_service", Result.PASS if service == "active" else Result.FAIL,
        f"systemctl is-active {EXPECTED_SERVICE}: {service!r}")
    errors = observed.get("errors") or {}
    add("local_api_read", Result.FAIL if errors else Result.PASS,
        "; ".join(f"{p}: {e}" for p, e in errors.items()) or "GET /health /state /config /backend answered")
    if not (state and config and backend and health):
        add("observations", Result.FAIL, "the agent's state could not be read; nothing below is known")
        return PreflightReport(checks, observed.get("collected_at") or time.time())

    # Mission ingress: the agent's own test-injection status, read without a POST.
    injection = observed.get("test_injection")
    if not config.get("socket_enabled"):
        add("mission_ingress", Result.BLOCKED,
            "bench mode (ROBOTX_SOCKET_ENABLED=0): the backend link is off -- no RobotX telemetry, which "
            "this test requires.")
    elif not isinstance(injection, dict):
        status = observed.get("test_injection_http_status")
        add("mission_ingress", Result.BLOCKED,
            "engine mode: a mission is admitted only as a backend-signed OFFER, and the test-only ingress "
            + ("is disabled on this agent (GET /test/injection: 404; ROBOTX_TEST_INJECTION_ENABLED is off "
               "or the running code predates it)." if status == 404 else f"could not be read ({status!r})."))
    elif not injection.get("available"):
        add("mission_ingress", Result.BLOCKED,
            f"test injection is enabled but unavailable: problem={injection.get('problem')!r} "
            f"locked_out={injection.get('locked_out')!r}")
    else:
        add("mission_ingress", Result.PASS,
            f"POST /test/mission/inject available on loopback; {injection.get('injected_task_ids')!r} task id(s) "
            "injected before; provenance test_injected, backend_assigned=false")

    # Backend.
    server = backend.get("server")
    add("backend_destination",
        Result.PASS if server == expected_backend and backend.get("tls") else Result.FAIL,
        f"server={server!r} tls={backend.get('tls')!r} (expected {expected_backend!r} over TLS)")
    streaming = bool(backend.get("authenticated") and backend.get("streaming"))
    add("backend_auth_and_telemetry", Result.PASS if streaming else Result.FAIL,
        f"status={backend.get('status')!r} authenticated={backend.get('authenticated')!r} "
        f"auth_method={backend.get('auth_method')!r} telemetry_sent={_get(backend, 'stats', 'telemetry_sent')!r} "
        f"emit_failures={_get(backend, 'stats', 'emit_failures')!r} -- the Pi's own link status, not a backend "
        "confirmation of storage")

    # Identity.
    ids = {"health": health.get("robot_id"), "state": state.get("robot_id"),
           "config": config.get("robot_id"), "backend": backend.get("robot_id")}
    if fixture is not None:
        ids["fixture"] = fixture.robot_id
    add("robot_identity", Result.PASS if len(set(ids.values())) == 1 and None not in ids.values() else Result.FAIL,
        ", ".join(f"{k}={v!r}" for k, v in ids.items()))

    # ESP32 link and UART.
    comm = state.get("communication") or {}
    controller = state.get("controller") or {}
    tel = controller.get("telemetry") or {}
    age = controller.get("telemetry_age_s")
    stale_after = config.get("esp32_stale_after_s")
    link_up = comm.get("esp32") == "UP" and controller.get("bidirectional") is True
    fresh = isinstance(age, (int, float)) and isinstance(stale_after, (int, float)) and age <= stale_after
    add("esp32_link", Result.PASS if link_up and fresh else Result.FAIL,
        f"link={comm.get('esp32')!r} ({comm.get('esp32_detail')!r}) telemetry_age_s={age!r} "
        f"(stale after {stale_after!r}) port={config.get('esp32_port')!r}")
    add("uart_ownership", Result.NOT_TESTED,
        f"the running agent is the configured owner of {config.get('esp32_port')!r} (ESP32 link and "
        f"gps_source={config.get('gps_source')!r}); this preflight opened no port. Exclusive ownership on the "
        "Pi needs root to verify, and the ESP32's UART0 is also wired to its USB bridge on the laptop -- a "
        "laptop serial session must be idle (no TX) before any hardware test.")
    diag_system = _get(state, "controller_diag", "SYSTEM") or {}
    add("controller_firmware", Result.PASS if diag_system.get("fw_build") else Result.NOT_TESTED,
        f"fw_build={diag_system.get('fw_build')!r} built {diag_system.get('fw_build_utc')!r}; "
        f"stall_count={diag_system.get('stall_count')!r} last_stall={diag_system.get('last_stall_ms')!r} ms "
        f"({diag_system.get('last_stall_source')!r})", required=False)

    # Motor controller.
    motor = tel.get("motor") or {}
    drive_ok = motor.get("drive_available") is True and tel.get("state") != "DRIVE_UNAVAILABLE"
    add("motor_controller", Result.PASS if drive_ok else Result.FAIL,
        f"state={tel.get('state')!r} block_reason={tel.get('block_reason')!r} "
        f"drive_available={motor.get('drive_available')!r} pca_status={diag_system.get('pca_status')!r} "
        f"motor_drive_status={diag_system.get('motor_drive_status')!r}")
    add("motion_enabled", Result.PASS if config.get("esp32_motion_enabled") else Result.BLOCKED,
        f"ROBOTX_ESP32_MOTION_ENABLED={config.get('esp32_motion_enabled')!r}; the link writes no DRIVE while "
        "false. Not changed by this harness.")
    add("motion_ready", Result.PASS if controller.get("motion_ready") is True else Result.FAIL,
        f"Esp32Link motion_ready={controller.get('motion_ready')!r} (the agent's own gate: motion enabled, "
        "link UP, fresh TELEMETRY with drive available and no controller fault)")

    # Front sensing.
    front = tel.get("front") or {}
    diag_front = _get(state, "controller_diag", "FRONT") or {}
    add("front_sensors", Result.PASS if front.get("valid") is True and fresh else Result.FAIL,
        f"valid={front.get('valid')!r} left_cm={front.get('left_cm')!r} right_cm={front.get('right_cm')!r} "
        f"health={diag_front.get('front_health')!r} fresh={fresh}")
    add("front_sensor_map", Result.PASS if diag_front.get("sensor_map_verified") is True else Result.NOT_TESTED,
        f"sensor_map_verified={diag_front.get('sensor_map_verified')!r}: which physical sensor is 'left' and "
        "which is 'right' has not been confirmed on the rover")
    stop_cm = config.get("safety_stop_distance_cm")
    readings = [v for v in (front.get("left_cm"), front.get("right_cm")) if isinstance(v, (int, float))]
    closest = min(readings) if readings else None
    clear = (not front.get("obstacle") and closest is not None and isinstance(stop_cm, (int, float))
             and closest > stop_cm)
    add("front_path_clear", Result.PASS if clear else Result.BLOCKED,
        f"closest={closest!r} cm, obstacle={front.get('obstacle')!r}, stop distance={stop_cm!r} cm")

    # Rear sensing.
    rear = tel.get("rear") or {}
    diag_rear = _get(state, "controller_diag", "REAR") or {}
    rear_ok = rear.get("available") is True and not rear.get("sensor_fault")
    add("rear_sensors", Result.PASS if rear_ok else Result.FAIL,
        f"available={rear.get('available')!r} sensor_fault={rear.get('sensor_fault')!r} "
        f"backend={diag_rear.get('rear_backend')!r} status={diag_rear.get('rear_sensor_status')!r}; reverse "
        "stays vetoed by the safety gate (negative_wheel) regardless")

    # Localization.
    gps = state.get("gps") or {}
    position = state.get("position") or {}
    h_acc = position.get("h_acc_m")
    max_h_acc = config.get("nav_max_h_acc_m")
    gps_ok = (gps.get("status") == "FIX" and isinstance(gps.get("age_s"), (int, float))
              and gps["age_s"] <= (config.get("gps_stale_after_s") or 0)
              and position.get("source") == "GPS"
              and isinstance(h_acc, (int, float)) and isinstance(max_h_acc, (int, float)) and h_acc <= max_h_acc)
    add("localization", Result.PASS if gps_ok else Result.FAIL,
        f"gps={gps.get('status')!r} age_s={gps.get('age_s')!r} fix_type={position.get('fix_type')!r} "
        f"sats={position.get('satellites')!r} h_acc_m={h_acc!r} (limit {max_h_acc!r}) source={position.get('source')!r}")
    if fixture is not None:
        lat, lon = position.get("latitude"), position.get("longitude")
        here = (lat, lon)
        inside = (gps_ok and isinstance(lat, (int, float)) and isinstance(lon, (int, float))
                  and fixture.in_region((float(lat), float(lon))))
        add("rover_in_fixture_region", Result.PASS if inside else Result.FAIL,
            f"rover at {here}, fixture region radius {fixture.region_radius_m:.0f} m")
    add("clock_synchronized", Result.PASS if observed.get("clock_synchronized") else Result.FAIL,
        f"NTPSynchronized={observed.get('clock_synchronized')!r}")

    # Safety and mission.
    safety = state.get("safety") or {}
    estop = safety.get("rule") == "estop"
    gates = tel.get("gates") or {}
    add("safety_state",
        Result.FAIL if estop or state.get("mode") == "ERROR" or gates.get("safety_stop") else Result.PASS,
        f"mode={state.get('mode')!r} verdict={safety.get('verdict')!r} rule={safety.get('rule')!r} "
        f"controller safety_stop={gates.get('safety_stop')!r} forward_blocked={gates.get('forward_blocked')!r} "
        f"reverse_blocked={gates.get('reverse_blocked')!r} last_error={state.get('last_error')!r}")
    add("safety_config",
        Result.PASS if stop_cm == 90.0 else Result.FAIL,
        f"safety_stop_distance_cm={stop_cm!r} (expected 90), require_range_sensor="
        f"{config.get('safety_require_range_sensor')!r}, max_speed={config.get('safety_max_speed')!r}")
    add("custody_confirmation",
        Result.PASS if config.get("custody_confirmation") == "operator" else Result.BLOCKED,
        f"ROBOTX_CUSTODY_CONFIRMATION={config.get('custody_confirmation')!r}; the agent refuses any custody "
        "mission (NO_CUSTODY_SENSING) unless it is 'operator'")
    mission = state.get("mission")
    active = isinstance(mission, dict) and mission.get("status") not in (None, "COMPLETE", "ABORTED")
    add("mission_state", Result.BLOCKED if active else Result.PASS,
        f"mission={None if mission is None else (mission.get('task_id'), mission.get('status'))!r}")

    # Not observable here.
    add("physical_estop", Result.NOT_TESTED,
        "no physical emergency-stop is in the hardware inventory; only the software latch (/safety/estop)",
        required=False)
    power = state.get("power") or {}
    add("battery", Result.NOT_TESTED,
        f"status={power.get('status')!r}: {power.get('source')!r}", required=False)

    return PreflightReport(checks, observed.get("collected_at") or time.time())


def format_report(report: PreflightReport) -> str:
    lines = [f"PREFLIGHT {report.verdict}  (non-motion; nothing was written to the agent or the ESP32)", ""]
    width = max(len(c.name) for c in report.checks)
    for c in report.checks:
        tag = c.result.value + ("" if c.required else " (info)")
        lines.append(f"  {tag:<18} {c.name:<{width}}  {c.detail}")
    if report.blockers():
        lines += ["", "Blocking: " + ", ".join(c.name for c in report.blockers())]
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Non-motion preflight of the running RobotX agent.")
    parser.add_argument("--fixture", help="also check a mission fixture against the live Rover")
    parser.add_argument("--json", action="store_true", help="print the report as JSON")
    parser.add_argument("--ledger", help="append the report to this test ledger")
    args = parser.parse_args(argv)

    observed = collect()
    fixture = None
    if args.fixture:
        try:
            fixture = load_fixture(args.fixture, expected_robot_id=_get(observed, "config", "robot_id"))
        except FixtureError as e:
            print(f"FIXTURE REJECTED {e.code}: {e.detail}", file=sys.stderr)
            return 2
    report = evaluate(observed, fixture=fixture)
    if args.ledger:
        from tests.mission_injection.ledger import PREFLIGHT, Ledger

        Ledger(args.ledger).append(PREFLIGHT, fixture=fixture, report=report.to_dict())
    print(json.dumps(report.to_dict(), indent=2) if args.json else format_report(report))
    return 0 if report.verdict == "READY" else 1


if __name__ == "__main__":
    sys.exit(main())
