"""The offer-acceptance demonstration, end to end: the REAL Pi agent against a REAL backend.

    venv/bin/python -m tests.integration.offer_acceptance_demo_harness
                     (driven by the backend's tools/verify/gate3PiSimOffer.js --offer-acceptance-demo)

What runs, unmodified: `RobotAgent` with ROBOTX_OFFER_ACCEPTANCE_DEMO=1 and
ROBOTX_ESP32_MOTION_ENABLED=0, `BackendLink` (AUTH, OFFER admission: addressee,
expiry, HMAC, fence, sequence, persist-before-effect; COMMAND_ACK; OFFER_ACCEPT;
HEARTBEAT; TELEMETRY with readiness; PROBE_RESULT), `Esp32Link`, and the ESP32
firmware compiled into the host simulator. No serial device is opened, no
camera stand-in is installed (the demonstration must not need one), no motor
exists.

TEST INPUT, labelled: the CRC-valid ESP32 `GPS` frames of gate3_harness (fixed
position, hAcc 1.5 m), because this machine has no receiver.

Checks: authenticated; ESP32 link UP with motion NOT ready (motors disabled);
TELEMETRY readiness `{"blockers": []}`; PROBE answered; the OFFER admitted,
COMMAND_ACK and OFFER_ACCEPT sent; then, for WATCH_S after the acceptance, the
mode stays IDLE with no mission, no DRIVE or STOP is ever written to the ESP32,
and every HEARTBEAT is the bare form (no lease renewal). Writes GATE3_REPORT and
exits 0 only if every check holds.
"""

from __future__ import annotations

import asyncio
import json
import os
import sys
import time
from typing import Any, Dict

import robotx.application.agent as agent_module
from robotx.application.agent import RobotAgent
from robotx.config.settings import Settings
from robotx.state.robot_state import OperatingMode
from tests.integration.gate3_harness import (
    FRONT_FILE,
    REPORT,
    TIMEOUT_S,
    Gate3SimulatorPort,
    _no_serial,
    drive_frames,
    evidence,
    lock,
    wait_for,
)

WATCH_S = float(os.environ.get("OFFER_DEMO_WATCH_S", "20"))


async def main() -> int:
    with open(FRONT_FILE, "w") as f:
        f.write("150\n")
    agent_module.HostSimulatorPort = Gate3SimulatorPort
    import robotx.esp32.link as link_module
    import robotx.esp32.transport as transport_module

    transport_module.open_serial = _no_serial
    link_module.open_serial = _no_serial

    settings = Settings.from_env(os.environ)
    if not settings.esp32_simulator_exe:
        raise SystemExit("ROBOTX_ESP32_SIMULATOR_EXE is required: this harness never uses a UART")
    if not settings.offer_acceptance_demo or settings.esp32_motion_enabled:
        raise SystemExit("run with ROBOTX_OFFER_ACCEPTANCE_DEMO=1 and ROBOTX_ESP32_MOTION_ENABLED=0")

    agent = RobotAgent(settings)
    await agent.start()
    backend = agent.backend
    original_emit = backend._emit

    async def recording_emit(event, payload, **kwargs):
        sent = await original_emit(event, payload, **kwargs)
        if sent:
            with lock:
                evidence["backend_emits"].append({"t": time.time(), "event": event, "payload": payload})
                del evidence["backend_emits"][:-600]
        return sent

    backend._emit = recording_emit

    def emitted(event):
        with lock:
            return [e for e in evidence["backend_emits"] if e["event"] == event]

    checks: Dict[str, Any] = {}
    after: Dict[str, Any] = {}
    try:
        checks["authenticated"] = await wait_for(lambda: backend._authenticated.is_set(), 30)
        checks["esp32_link_up"] = await wait_for(lambda: agent.state.snapshot().communication.esp32.is_up, 30)
        checks["esp32_motion_not_ready"] = agent.esp32 is not None and agent.esp32.status().motion_ready is False
        checks["readiness_reported_clear"] = await wait_for(
            lambda: any(e["payload"].get("readiness") == {"blockers": []} for e in emitted("TELEMETRY")), 30)
        print("[gate3] ready: auth=%s esp32_up=%s readiness_clear=%s" % (
            checks["authenticated"], checks["esp32_link_up"], checks["readiness_reported_clear"]), flush=True)

        checks["offer_admitted"] = await wait_for(lambda: backend.stats.get("engine_commands_admitted", 0) >= 1, TIMEOUT_S, 0.25)
        checks["offer_accept_sent"] = await wait_for(lambda: bool(emitted("OFFER_ACCEPT")), 10)
        checks["command_ack_sent"] = any("outboxId" in e["payload"] for e in emitted("COMMAND_ACK"))
        accepted_at = emitted("OFFER_ACCEPT")[0]["t"] if checks["offer_accept_sent"] else time.time()
        if checks["offer_accept_sent"]:
            print("[gate3] OFFER_ACCEPT sent for %s" % emitted("OFFER_ACCEPT")[0]["payload"].get("commitmentId"), flush=True)

        # The watch: nothing starts, nothing moves, nothing renews the lease.
        modes, missions = set(), set()
        deadline = time.monotonic() + WATCH_S
        while time.monotonic() < deadline:
            snap = agent.state.snapshot()
            modes.add(snap.mode.value)
            missions.add(None if snap.mission is None else snap.mission.task_id)
            await asyncio.sleep(0.25)
        beats_after = [e["payload"] for e in emitted("HEARTBEAT") if e["t"] > accepted_at]
        motion = [d for d in drive_frames() if d.get("cmd") in ("DRIVE", "STOP")]
        after = {"modes_seen": sorted(modes), "missions_seen": sorted(str(m) for m in missions),
                 "heartbeats_after_accept": len(beats_after), "motion_frames_written": motion[:5]}
        checks["mode_stayed_idle"] = modes == {OperatingMode.IDLE.value}
        checks["no_mission_started"] = missions == {None}
        checks["no_drive_or_stop_written"] = not motion
        checks["heartbeats_after_accept_are_bare"] = bool(beats_after) and all(b == {} for b in beats_after)
        checks["probe_answered"] = backend.stats.get("probes_answered", 0) >= 1
        checks["telemetry_has_fix_quality"] = any(e["payload"].get("position", {}).get("fixType") == "3D" for e in emitted("TELEMETRY"))
        checks["telemetry_has_released_latch"] = any(
            e["payload"].get("safety", {}).get("stopLatch", {}).get("engaged") is False for e in emitted("TELEMETRY"))
    finally:
        await agent.stop()

    checks["serial_devices_opened"] = evidence["serial_devices_opened"]
    report = {
        "checks": checks,
        "after_accept": after,
        "wire_commands": sorted({d.get("cmd") for d in drive_frames()}),
        "gps_frames_injected": evidence["gps_frames_injected"],
        "backend_events": sorted({e["event"] for e in evidence["backend_emits"]}),
        "backend_stats": dict(backend.stats),
    }
    with open(REPORT, "w") as f:
        json.dump(report, f, indent=2, default=str)
    required = [
        "authenticated", "esp32_link_up", "esp32_motion_not_ready", "readiness_reported_clear", "offer_admitted",
        "offer_accept_sent", "command_ack_sent", "mode_stayed_idle", "no_mission_started", "no_drive_or_stop_written",
        "heartbeats_after_accept_are_bare", "probe_answered", "telemetry_has_fix_quality", "telemetry_has_released_latch",
    ]
    failed = [name for name in required if checks.get(name) is not True]
    ok = not failed and evidence["serial_devices_opened"] == 0
    print("[gate3] " + ("PASS" if ok else f"FAIL: {failed}"), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
