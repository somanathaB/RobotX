"""GATE 3 — the real Pi agent against a real RobotX backend and the ESP32 host simulator.

    venv/bin/python -m tests.integration.gate3_harness        (driven by the backend's
                                                              tools/verify/gate3PiSimOffer.js)

What runs, unmodified: `RobotAgent` (mission manager, navigator, DecisionMaker,
safety gate, telemetry), `BackendLink` (AUTH, OFFER admission: HMAC, addressee,
expiry, fence, sequence, persist-before-effect; OFFER_ACCEPT; COMMAND_ACK;
HEARTBEAT; TELEMETRY; PROBE_RESULT), `Esp32Link` (framing, CRC, sequence, PING,
DRIVE/STOP, ACK matching, telemetry), and the ESP32 firmware itself, compiled
into the host simulator. No serial device is opened and no motor exists.

TEST INPUTS — the two things this machine has no hardware for, labelled as such:

- GPS: the simulator's u-blox stub sits in BACKOFF, so the harness writes
  CRC-valid ESP32 `GPS` frames (PROTOCOL.md section 11) into the stream the link
  reads, at a fixed position, once a second, between whole lines. They go
  through the Pi's real decoder and ESP32 GPS adapter.
- Perception: no camera here, so a clear-scene result (status OK, frame
  metadata, zero detections) stands in for the camera pipeline.

Two phases:
  A. path clear (front range 150 cm): the OFFER is admitted and accepted, the
     mission starts, the navigator and DecisionMaker produce forward motion, the
     safety gate passes it, a DRIVE goes to the simulated ESP32 and is ACKed
     ACCEPTED, and the ESP32's TELEMETRY shows the applied output.
  B. obstacle (front range set to 20 cm through SIM_FRONT_CM_FILE): the ESP32's
     own obstacle gate must hold the output at 0 whatever the Pi asks, latch its
     safety stop, and the Pi must report the latch to the backend.

Writes a JSON report to GATE3_REPORT and exits 0 only if every check holds.
"""

from __future__ import annotations

import asyncio
import json
import os
import sys
import threading
import time
from typing import Any, Dict, List

import robotx.application.agent as agent_module
from robotx.application.agent import RobotAgent
from robotx.config.settings import Settings
from robotx.esp32.protocol import crc16, decode_line
from robotx.esp32.sim_transport import HostSimulatorPort
from robotx.perception.types import FrameMetadata, PerceptionResult, PerceptionStatus

REPORT = os.environ.get("GATE3_REPORT", "gate3-report.json")
LAT = float(os.environ["GATE3_LAT"])
LON = float(os.environ["GATE3_LON"])
FRONT_FILE = os.environ["SIM_FRONT_CM_FILE"]
TIMEOUT_S = float(os.environ.get("GATE3_TIMEOUT_S", "180"))

evidence: Dict[str, Any] = {
    "commands_written": [],
    "acks": [],
    "esp32_telemetry_samples": [],
    "backend_emits": [],
    "gps_frames_injected": 0,
    "serial_devices_opened": 0,
}
lock = threading.Lock()


class Gate3SimulatorPort(HostSimulatorPort):
    """The host simulator, plus the harness's GPS frames, plus a record of the wire."""

    def __init__(self, exe: str, **kwargs: Any) -> None:
        super().__init__(exe, **kwargs)
        self._next_gps_at = 0.0
        self._pending_line = b""

    def _gps_frame(self) -> bytes:
        payload = (
            '{"type":"GPS","uptime_ms":%d,"gps_status":"OK","fix_type":3,"fix_ok":true,"siv":14,'
            '"lat_e7":%d,"lon_e7":%d,"hacc_mm":1500,"speed_mm_s":0,"head_mot_e5":null,"age_ms":100}'
            % (int(time.monotonic() * 1000) % 2_000_000_000, round(LAT * 1e7), round(LON * 1e7))
        ).encode("ascii")
        return payload + b"*%04X\n" % crc16(payload)

    def read(self, size: int) -> bytes:
        # Whole lines only: a partial line is held until its LF arrives, so an
        # injected frame can never land inside one the simulator is mid-way through.
        self._hold = getattr(self, "_hold", b"") + super().read(size)
        cut = self._hold.rfind(b"\n")
        if cut < 0:
            complete = b""
        else:
            complete, self._hold = self._hold[: cut + 1], self._hold[cut + 1:]
        # The simulator's u-blox stub never locks (it reports BACKOFF every second).
        # The harness's GPS input REPLACES that stub output rather than racing it:
        # the stub's GPS lines are dropped, everything else passes through untouched.
        kept = [line for line in complete.splitlines(keepends=True) if not line.startswith(b'{"type":"GPS"')]
        with lock:
            evidence["sim_gps_frames_replaced"] = evidence.get("sim_gps_frames_replaced", 0) + (
                len(complete.splitlines()) - len(kept))
        data = b"".join(kept)
        if data:
            self._record_inbound(data)
        now = time.monotonic()
        if now >= self._next_gps_at:
            self._next_gps_at = now + 1.0
            with lock:
                evidence["gps_frames_injected"] += 1
            return data + self._gps_frame()
        return data

    def _record_inbound(self, data: bytes) -> None:
        buf = self._pending_line + data
        *lines, self._pending_line = buf.split(b"\n")
        for raw in lines:
            frame = decode_line(raw + b"\n")
            if getattr(frame, "type", None) == "ACK":
                with lock:
                    evidence["acks"].append(dict(frame.data))
            elif getattr(frame, "type", None) == "TELEMETRY":
                with lock:
                    samples = evidence["esp32_telemetry_samples"]
                    sample = {k: frame.data[k] for k in ("state", "block_reason", "left_applied", "right_applied",
                                                          "front_obstacle", "forward_blocked", "safety_stop",
                                                          "motor_drive_available", "last_seq")}
                    if not samples or samples[-1] != sample:
                        samples.append(sample)  # deduplicated, uncapped: indices stay stable

    def write(self, data: bytes) -> int:
        frame_text = data.decode("ascii", "replace").strip()
        if frame_text:
            with lock:
                evidence["commands_written"].append({"t": time.time(), "frame": frame_text})
        return super().write(data)


class ClearScenePerception:
    """TEST INPUT: what a camera pipeline reports for an empty scene."""

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def latest(self) -> PerceptionResult:
        return PerceptionResult(
            timestamp=time.time(),
            status=PerceptionStatus.OK,
            detections=(),
            frame=FrameMetadata(width=640, height=480, source="gate3-test-input", age_s=0.0),
            backend="gate3-clear-scene",
        )


def _no_serial(*_args: Any, **_kwargs: Any):
    with lock:
        evidence["serial_devices_opened"] += 1
    raise AssertionError("Gate 3 must never open a serial device")


async def wait_for(predicate, timeout_s: float, step_s: float = 0.1) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        await asyncio.sleep(step_s)
    return predicate()


def drive_frames() -> List[Dict[str, Any]]:
    out = []
    for row in evidence["commands_written"]:
        frame = decode_line((row["frame"] + "\n").encode("ascii"))
        # Commands are not a type the Pi decodes; parse the JSON part directly.
        try:
            body = json.loads(row["frame"][: row["frame"].rindex("*")])
        except ValueError:
            continue
        if body.get("cmd") in ("DRIVE", "STOP", "PING"):
            out.append(body)
    return out


async def main() -> int:
    with open(FRONT_FILE, "w") as f:
        f.write("150\n")
    agent_module.HostSimulatorPort = Gate3SimulatorPort
    import robotx.esp32.transport as transport_module
    import robotx.esp32.link as link_module
    transport_module.open_serial = _no_serial
    link_module.open_serial = _no_serial

    settings = Settings.from_env(os.environ)
    if not settings.esp32_simulator_exe:
        raise SystemExit("ROBOTX_ESP32_SIMULATOR_EXE is required: Gate 3 never uses a UART")

    agent = RobotAgent(settings)
    await agent.start()
    agent.perception = ClearScenePerception()

    # Record what the Pi tells the backend.
    backend = agent.backend
    original_emit = backend._emit

    async def recording_emit(event, payload, **kwargs):
        sent = await original_emit(event, payload, **kwargs)
        if sent:
            with lock:
                evidence["backend_emits"].append({"t": time.time(), "event": event, "payload": payload})
                del evidence["backend_emits"][:-400]
        return sent

    backend._emit = recording_emit

    checks: Dict[str, Any] = {}
    try:
        checks["authenticated"] = await wait_for(lambda: backend._authenticated.is_set(), 30)
        checks["esp32_link_up_and_motion_ready"] = await wait_for(
            lambda: agent.esp32 is not None and agent.esp32.status().motion_ready, 30)
        print("[gate3] ready: auth=%s esp32=%s" % (checks["authenticated"], checks["esp32_link_up_and_motion_ready"]), flush=True)

        checks["offer_admitted"] = await wait_for(lambda: backend.stats.get("engine_commands_admitted", 0) >= 1, TIMEOUT_S, 0.25)
        accepted = lambda: any(e["event"] == "OFFER_ACCEPT" for e in evidence["backend_emits"])
        checks["offer_accept_sent"] = await wait_for(accepted, 10)
        checks["command_ack_sent"] = any(e["event"] == "COMMAND_ACK" and "outboxId" in e["payload"] for e in evidence["backend_emits"])
        checks["mission_active"] = await wait_for(
            lambda: agent.state.snapshot().mission is not None and agent.state.snapshot().mission.is_active, 10)

        def driving() -> bool:
            snap = agent.state.snapshot()
            return any(d.get("cmd") == "DRIVE" and (d.get("left", 0) > 0 or d.get("right", 0) > 0) for d in drive_frames()) and \
                snap.navigation is not None
        checks["drive_written"] = await wait_for(driving, 20)
        snap = agent.state.snapshot()
        checks["phase_a_state"] = {
            "mode": snap.mode.value,
            "mission_task": snap.mission.task_id if snap.mission else None,
            "mission_segment": snap.mission.segment.value if snap.mission else None,
            "navigation_status": snap.navigation.status.value,
            "motion_intent": {"reason": snap.motion_intent.reason, "left": snap.motion_intent.left, "right": snap.motion_intent.right},
            "safety": {"verdict": snap.safety.verdict.value, "rule": snap.safety.rule},
        }
        drives = [d for d in drive_frames() if d.get("cmd") == "DRIVE" and (d["left"] > 0 or d["right"] > 0)]
        first_drive = drives[0] if drives else None
        checks["first_forward_drive"] = first_drive
        checks["drive_acked_accepted"] = await wait_for(
            lambda: first_drive is not None and any(a.get("seq") == first_drive["seq"] and a.get("result") == "ACCEPTED" for a in evidence["acks"]), 5)
        checks["esp32_applied_output"] = await wait_for(
            lambda: any(s["left_applied"] > 0 and s["right_applied"] > 0 for s in evidence["esp32_telemetry_samples"]), 5)
        checks["pi_saw_applied_output"] = await wait_for(
            lambda: (lambda c: c is not None and c.telemetry is not None and c.telemetry.left_applied > 0)(agent.state.snapshot().controller), 5)
        tel = [e["payload"] for e in evidence["backend_emits"] if e["event"] == "TELEMETRY"]
        checks["backend_telemetry_has_fix_quality"] = any(p.get("position") == {"fixType": "3D", "hAccM": 1.5} for p in tel)
        checks["backend_telemetry_has_released_latch"] = any(
            p.get("safety", {}).get("stopLatch", {}).get("engaged") is False for p in tel)
        checks["probe_answered"] = await wait_for(lambda: backend.stats.get("probes_answered", 0) >= 1, 10)

        # ── Phase B: an obstacle the ESP32 must enforce ───────────────────────────
        mark_acks = len(evidence["acks"])
        mark_tel = len(evidence["esp32_telemetry_samples"])
        with open(FRONT_FILE, "w") as f:
            f.write("20\n")
        checks["esp32_blocked_forward"] = await wait_for(
            lambda: any(s["front_obstacle"] and s["forward_blocked"] and s["left_applied"] == 0 and s["right_applied"] == 0
                        for s in evidence["esp32_telemetry_samples"][mark_tel:]), 10)
        # The firmware's own gate answers a forward DRIVE into an obstacle with ACK GATED
        # FRONT_OBSTACLE and applies 0 (PROTOCOL.md section 9). It does not set the
        # safety_stop latch for an obstacle; it holds STOPPED (no auto-resume).
        checks["drive_gated_front_obstacle"] = await wait_for(
            lambda: any(a.get("cmd") == "DRIVE" and a.get("result") == "GATED" and a.get("reason") == "FRONT_OBSTACLE"
                        and a.get("applied_left") == 0 and a.get("applied_right") == 0
                        for a in evidence["acks"][mark_acks:]), 10)
        checks["no_drive_applied_while_blocked"] = all(
            not (s["front_obstacle"] and (s["left_applied"] != 0 or s["right_applied"] != 0))
            for s in evidence["esp32_telemetry_samples"][mark_tel:])
        # A DRIVE already in flight before the simulator re-read the range (it polls
        # every 20 ms) is legitimately ACCEPTED. From the first gated DRIVE on, none may be.
        after = evidence["acks"][mark_acks:]
        first_gated = next((i for i, a in enumerate(after) if a.get("result") == "GATED"), None)
        checks["drive_after_obstacle_not_accepted"] = first_gated is not None and all(
            not (a.get("cmd") == "DRIVE" and a.get("result") == "ACCEPTED") for a in after[first_gated:])
        checks["pi_saw_forward_blocked"] = await wait_for(
            lambda: (lambda c: c is not None and c.telemetry is not None and c.telemetry.forward_blocked)(agent.state.snapshot().controller), 10)
        # Truthful reporting: when the ESP32 latched its safety stop, the Pi reports the
        # software stop latch engaged, attributed to the ESP32 component -- and before
        # the obstacle it reported it released.
        esp32_latched = any(s["safety_stop"] for s in evidence["esp32_telemetry_samples"][mark_tel:])
        engaged_reports = [e["payload"]["safety"]["stopLatch"] for e in evidence["backend_emits"]
                           if e["event"] == "TELEMETRY" and e["payload"].get("safety", {}).get("stopLatch", {}).get("engaged") is True]
        if esp32_latched:
            checks["pi_latch_report_matches_esp32"] = await wait_for(lambda: any(
                e["event"] == "TELEMETRY" and e["payload"].get("safety", {}).get("stopLatch", {}).get("components", {}).get("esp32SafetyStop") is True
                for e in evidence["backend_emits"]), 10)
        else:
            checks["pi_latch_report_matches_esp32"] = not engaged_reports
        checks["esp32_safety_stop_latched_by_obstacle"] = esp32_latched
    finally:
        await agent.stop()

    checks["serial_devices_opened"] = evidence["serial_devices_opened"]
    status = agent.esp32.status() if agent.esp32 is not None else None
    report = {
        "checks": checks,
        "gps_frames_injected": evidence["gps_frames_injected"],
        "sim_gps_frames_replaced": evidence.get("sim_gps_frames_replaced", 0),
        "esp32_counters": None if status is None else dict(status.controller.counters.__dict__),
        "acks": evidence["acks"][-12:],
        "drive_frames": drive_frames()[-12:],
        "esp32_telemetry_samples": evidence["esp32_telemetry_samples"][-12:],
        "backend_events": sorted({e["event"] for e in evidence["backend_emits"]}),
        "backend_stats": dict(backend.stats),
    }
    with open(REPORT, "w") as f:
        json.dump(report, f, indent=2, default=str)
    required = [
        "authenticated", "esp32_link_up_and_motion_ready", "offer_admitted", "offer_accept_sent", "command_ack_sent",
        "mission_active", "drive_written", "drive_acked_accepted", "esp32_applied_output", "pi_saw_applied_output",
        "backend_telemetry_has_fix_quality", "backend_telemetry_has_released_latch", "probe_answered",
        "esp32_blocked_forward", "drive_gated_front_obstacle", "no_drive_applied_while_blocked",
        "drive_after_obstacle_not_accepted", "pi_saw_forward_blocked", "pi_latch_report_matches_esp32",
    ]
    failed = [name for name in required if checks.get(name) is not True]
    ok = not failed and evidence["serial_devices_opened"] == 0
    print("[gate3] " + ("PASS" if ok else f"FAIL: {failed}"), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
