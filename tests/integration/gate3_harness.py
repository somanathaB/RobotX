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

GATE 3b (GATE3B=1) replaces phase B with the completion half:
  C. the stand-in GPS moves along the OFFER's OWN `stopSequence[].path` -- read from
     the signed OFFER as the Pi received it, never a synthetic line -- and only while
     the simulated ESP32 reports forward output applied on both sides. Its speed is
     GATE3B_FULL_SPEED_MPS (the unit's commissioned maximum, a harness declaration)
     scaled by the applied output (-255..255). Applied output that is zero, or that
     turns in place (opposite signs), moves nothing: a GPS receiver cannot observe a
     rotation, so the stand-in does not either. At each stop the harness invokes the
     existing operator custody confirmation -- the `POST /mission/custody` handler
     function itself, awaited in-process (no HTTP server is started, nothing listens
     on any port) -- and then checks what the Pi tells the backend: CUSTODY_EVENT,
     TASK_COMPLETE, and the backend's TASK_COMPLETE_ACK.

Writes a JSON report to GATE3_REPORT and exits 0 only if every check holds.
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import robotx.application.agent as agent_module
from robotx.application.agent import RobotAgent
from robotx.config.settings import Settings
from robotx.esp32.protocol import crc16, decode_line
from robotx.esp32.sim_transport import HostSimulatorPort
from robotx.mission.evidence import distance_to_path_m
from robotx.perception.types import FrameMetadata, PerceptionResult, PerceptionStatus

REPORT = os.environ.get("GATE3_REPORT", "gate3-report.json")
LAT = float(os.environ["GATE3_LAT"])
LON = float(os.environ["GATE3_LON"])
FRONT_FILE = os.environ["SIM_FRONT_CM_FILE"]
TIMEOUT_S = float(os.environ.get("GATE3_TIMEOUT_S", "180"))
GATE3B = os.environ.get("GATE3B") == "1"
FULL_SPEED_MPS = float(os.environ.get("GATE3B_FULL_SPEED_MPS", "0") or 0)
# No forward progress for this long, away from a stop, is a stall: the run ends
# with the Pi's own navigation state rather than waiting out a timeout.
STALL_S = float(os.environ.get("GATE3B_STALL_S", "45"))
APPLIED_FULL_SCALE = 255.0  # PROTOCOL.md section 6: DRIVE left/right -255..255

evidence: Dict[str, Any] = {
    "commands_written": [],
    "acks": [],
    "esp32_telemetry_samples": [],
    "backend_emits": [],
    "gps_frames_injected": 0,
    "serial_devices_opened": 0,
}
lock = threading.Lock()

_EARTH_RADIUS_M = 6_371_000.0
LatLon = Tuple[float, float]


def _haversine_m(a: LatLon, b: LatLon) -> float:
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    dp, dl = p2 - p1, math.radians(b[1] - a[1])
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * _EARTH_RADIUS_M * math.asin(math.sqrt(h))


def _bearing_deg(a: LatLon, b: LatLon) -> float:
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    dl = math.radians(b[1] - a[1])
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


class PathTrack:
    """TEST INPUT (Gate 3b): where the stand-in GPS says the rover is.

    Constrained to the OFFER's own paths: every position it reports lies on
    `stopSequence[leg].path`, at a distance along it that grows only by what the
    simulated ESP32 reports it applied. Nothing here steers, turns or chooses a
    route -- it is the receiver, not the robot.
    """

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.legs: Optional[List[List[LatLon]]] = None
        self.leg = 0
        self.seg = 0          # index of the vertex the current segment starts at
        self.along_m = 0.0    # metres along the current segment
        self.travelled_m = 0.0
        self.speed_mps = 0.0
        self.heading_deg: Optional[float] = None
        self.leg_join_gap_m: Optional[float] = None
        self.positions: List[Dict[str, Any]] = []
        self._last_tel_t: Optional[float] = None

    @property
    def active(self) -> bool:
        return self.legs is not None

    def load(self, legs: Sequence[Sequence[LatLon]]) -> None:
        with self.lock:
            self.legs = [list(path) for path in legs]
            self.leg, self.seg, self.along_m = 0, 0, 0.0
            self._update_heading()

    def switch_to_leg(self, leg: int) -> None:
        with self.lock:
            if self.legs is None or leg == self.leg or leg >= len(self.legs):
                return
            end = self._position()
            self.leg, self.seg, self.along_m = leg, 0, 0.0
            self.leg_join_gap_m = _haversine_m(end, self._position())
            self._update_heading()

    def _path(self) -> List[LatLon]:
        return self.legs[self.leg]

    def _position(self) -> LatLon:
        path = self._path()
        if self.seg >= len(path) - 1:
            return path[-1]
        a, b = path[self.seg], path[self.seg + 1]
        length = _haversine_m(a, b)
        f = 0.0 if length <= 0 else min(1.0, self.along_m / length)
        return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)

    def _update_heading(self) -> None:
        path = self._path()
        for i in range(self.seg, len(path) - 1):
            if _haversine_m(path[i], path[i + 1]) > 0.5:
                self.heading_deg = _bearing_deg(path[i], path[i + 1])
                return

    def position(self) -> Optional[LatLon]:
        with self.lock:
            return None if self.legs is None else self._position()

    def on_applied_output(self, left: int, right: int, now: float) -> None:
        """One ESP32 TELEMETRY frame: move by what was applied since the last one."""

        with self.lock:
            last, self._last_tel_t = self._last_tel_t, now
            if self.legs is None or last is None:
                return
            dt = min(max(0.0, now - last), 0.5)
            forward = left >= 0 and right >= 0 and (left + right) > 0
            self.speed_mps = FULL_SPEED_MPS * ((left + right) / 2.0) / APPLIED_FULL_SCALE if forward else 0.0
            remaining = self.speed_mps * dt
            path = self._path()
            while remaining > 0 and self.seg < len(path) - 1:
                length = _haversine_m(path[self.seg], path[self.seg + 1])
                left_on_seg = length - self.along_m
                if remaining < left_on_seg:
                    self.along_m += remaining
                    self.travelled_m += remaining
                    remaining = 0.0
                else:
                    self.travelled_m += left_on_seg
                    remaining -= left_on_seg
                    self.seg += 1
                    self.along_m = 0.0
                    self._update_heading()

    def frame_fields(self) -> Tuple[LatLon, int, Optional[int]]:
        """(position, speed mm/s, heading-of-motion 1e-5 deg) for the next GPS frame."""

        with self.lock:
            here = self._position()
            heading = None if self.heading_deg is None else round(self.heading_deg * 1e5)
            self.positions.append({"t": time.time(), "leg": self.leg, "seg": self.seg,
                                   "lat": here[0], "lon": here[1], "travelled_m": round(self.travelled_m, 2)})
            return here, round(self.speed_mps * 1000), heading


TRACK = PathTrack()


class Gate3SimulatorPort(HostSimulatorPort):
    """The host simulator, plus the harness's GPS frames, plus a record of the wire."""

    def __init__(self, exe: str, **kwargs: Any) -> None:
        super().__init__(exe, **kwargs)
        self._next_gps_at = 0.0
        self._pending_line = b""

    def _gps_frame(self) -> bytes:
        lat, lon, speed_mm_s, head = LAT, LON, 0, None
        if TRACK.active:
            (lat, lon), speed_mm_s, head = TRACK.frame_fields()
        payload = (
            '{"type":"GPS","uptime_ms":%d,"gps_status":"OK","fix_type":3,"fix_ok":true,"siv":14,'
            '"lat_e7":%d,"lon_e7":%d,"hacc_mm":1500,"speed_mm_s":%d,"head_mot_e5":%s,"age_ms":100}'
            % (int(time.monotonic() * 1000) % 2_000_000_000, round(lat * 1e7), round(lon * 1e7),
               speed_mm_s, "null" if head is None else str(head))
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
                TRACK.on_applied_output(int(frame.data["left_applied"]), int(frame.data["right_applied"]), time.monotonic())
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


def _pi_state(agent: RobotAgent) -> Dict[str, Any]:
    """The Pi's own view at this instant -- what a stall is reported with."""

    snap = agent.state.snapshot()
    nav = snap.navigation
    mission = snap.mission
    planner = agent.navigator.planner
    return {
        "mode": snap.mode.value,
        "mission_status": mission.status.value if mission else None,
        "navigation_status": nav.status.value if nav else None,
        "waypoint": None if nav is None else f"{nav.waypoint_index + 1}/{nav.waypoints_total}",
        "distance_to_target_m": None if nav is None or nav.distance_to_target_m is None else round(nav.distance_to_target_m, 1),
        "desired_heading_deg": None if nav is None or nav.desired_heading_deg is None else round(nav.desired_heading_deg, 1),
        "current_heading_deg": None if nav is None or nav.current_heading_deg is None else round(nav.current_heading_deg, 1),
        "heading_error_deg": None if nav is None or nav.heading_error_deg is None else round(nav.heading_error_deg, 1),
        "planner_offroute_count": getattr(planner, "_offroute_count", None),
        "planner_off_route_m": planner.cfg.off_route_m,
        "motion_intent": {"reason": snap.motion_intent.reason, "left": snap.motion_intent.left, "right": snap.motion_intent.right},
        "safety": {"verdict": snap.safety.verdict.value, "rule": snap.safety.rule},
        "esp32_applied": None if snap.controller is None or snap.controller.telemetry is None else
        [snap.controller.telemetry.left_applied, snap.controller.telemetry.right_applied],
        "track": {"leg": TRACK.leg, "seg": TRACK.seg, "travelled_m": round(TRACK.travelled_m, 1),
                  "position": TRACK.position()},
    }


async def drive_leg(agent: RobotAgent, until_status: str, path: Sequence[LatLon], log: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Let the Pi drive one leg until its mission reaches `until_status`, or stall."""

    length = sum(_haversine_m(path[i], path[i + 1]) for i in range(len(path) - 1))
    deadline = time.monotonic() + 120.0 + length / 0.1
    progress_at, progress_t = TRACK.travelled_m, time.monotonic()
    last_reason = None
    while time.monotonic() < deadline:
        state = _pi_state(agent)
        reason = state["motion_intent"]["reason"]
        if reason != last_reason:
            log.append({"t": round(time.time(), 1), "status": state["mission_status"], "nav": state["navigation_status"],
                        "waypoint": state["waypoint"], "intent": reason, "travelled_m": state["track"]["travelled_m"]})
            last_reason = reason
        if state["mission_status"] == until_status:
            return {"reached": True, "leg_length_m": round(length, 1), "state": state}
        if state["navigation_status"] == "REROUTE_NEEDED":
            return {"reached": False, "stall": "REROUTE_NEEDED", "leg_length_m": round(length, 1), "state": state}
        if state["mission_status"] in ("ABORTED", "COMPLETE") or state["mode"] == "ERROR":
            return {"reached": False, "stall": f"mission {state['mission_status']} / mode {state['mode']}", "state": state}
        if TRACK.travelled_m > progress_at + 0.5:
            progress_at, progress_t = TRACK.travelled_m, time.monotonic()
        elif time.monotonic() - progress_t > STALL_S:
            return {"reached": False, "stall": f"NO_PROGRESS_{STALL_S:.0f}S", "leg_length_m": round(length, 1), "state": state}
        await asyncio.sleep(0.1)
    return {"reached": False, "stall": "TIMEOUT", "leg_length_m": round(length, 1), "state": _pi_state(agent)}


async def confirm_custody_in_process(kind: str) -> Dict[str, Any]:
    """The existing operator custody confirmation, invoked inside this test process.

    This awaits the `POST /mission/custody` handler function from
    `robotx.application.main` directly. No HTTP server is started and nothing
    listens on any port: the confirmation path is exercised without being
    reachable from any network.
    """

    import robotx.application.main as http_surface
    from fastapi import HTTPException

    try:
        body = await http_surface.confirm_custody({"kind": kind})
        return {"ok": True, "kind": kind, "mission_status": (body.get("mission") or {}).get("status")}
    except HTTPException as e:
        return {"ok": False, "kind": kind, "http_status": e.status_code, "detail": e.detail}


def _emits(event: str) -> List[Dict[str, Any]]:
    with lock:
        return [e for e in evidence["backend_emits"] if e["event"] == event]


async def completion_phase(agent: RobotAgent, backend: Any, offer_payload: Dict[str, Any]) -> Dict[str, Any]:
    """Gate 3b: pickup arrival -> ACQUIRED -> drive -> drop arrival -> RELEASED -> TASK_COMPLETE."""

    import robotx.application.main as http_surface

    out: Dict[str, Any] = {"timeline": []}
    http_surface.state.agent = agent  # the handler's own lookup; in-process only

    stops = sorted(offer_payload["stopSequence"], key=lambda s: s["sequence"])
    legs = [[(p["lat"], p["lon"]) for p in stop["path"]] for stop in stops]
    mission = agent.state.snapshot().mission.mission
    out["offer_paths_are_mission_paths"] = (
        len(legs) == 2 and legs[0] == [tuple(p) for p in mission.path_to_pickup]
        and legs[1] == [tuple(p) for p in mission.path_to_drop])
    out["offer_task_id"] = offer_payload.get("taskId")

    # Negative controls: the confirmation path still refuses a handover the
    # mission has not reached.
    out["custody_refused_before_pickup"] = await confirm_custody_in_process("ACQUIRED")
    out["custody_release_refused_before_pickup"] = await confirm_custody_in_process("RELEASED")

    pickup = await drive_leg(agent, "AT_PICKUP", legs[0], out["timeline"])
    out["pickup_leg"] = pickup
    if not pickup["reached"]:
        return out
    out["pickup_arrival_distance_to_stop_m"] = round(_haversine_m(TRACK.position(), (stops[0]["lat"], stops[0]["lon"])), 2)
    out["release_refused_at_pickup"] = await confirm_custody_in_process("RELEASED")
    out["custody_acquired"] = await confirm_custody_in_process("ACQUIRED")
    out["custody_event_acquired"] = await wait_for(
        lambda: any(e["payload"].get("kind") == "ACQUIRED" for e in _emits("CUSTODY_EVENT")), 10)

    out["drop_leg_started"] = await wait_for(lambda: agent.state.snapshot().mission.status.value == "TO_DROP", 10)
    TRACK.switch_to_leg(1)
    out["leg_join_gap_m"] = None if TRACK.leg_join_gap_m is None else round(TRACK.leg_join_gap_m, 2)
    drop = await drive_leg(agent, "AT_DROP", legs[1], out["timeline"])
    out["drop_leg"] = drop
    if not drop["reached"]:
        return out
    out["drop_arrival_distance_to_stop_m"] = round(_haversine_m(TRACK.position(), (stops[1]["lat"], stops[1]["lon"])), 2)
    out["custody_released"] = await confirm_custody_in_process("RELEASED")
    out["custody_event_released"] = await wait_for(
        lambda: any(e["payload"].get("kind") == "RELEASED" for e in _emits("CUSTODY_EVENT")), 10)
    out["mission_complete"] = await wait_for(lambda: agent.state.snapshot().mission.status.value == "COMPLETE", 10)
    out["task_complete_sent"] = await wait_for(lambda: bool(_emits("TASK_COMPLETE")), 20)
    out["task_complete_payload"] = (_emits("TASK_COMPLETE") or [{}])[-1].get("payload")
    out["task_complete_acked"] = await wait_for(
        lambda: backend.stats.get("task_completes_acked", 0) >= 1 or backend.stats.get("task_completes_verifying", 0) >= 1, 20)
    out["task_complete_backend_verdict"] = (
        "ACKED" if backend.stats.get("task_completes_acked", 0) >= 1 else
        "VERIFYING" if backend.stats.get("task_completes_verifying", 0) >= 1 else None)
    out["task_complete_withheld"] = backend.stats.get("task_completes_withheld", 0)
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

    # Record every OFFER exactly as the Pi received it (Gate 3b reads its paths).
    offers_received: List[Dict[str, Any]] = []
    original_engine = backend._on_engine_command

    async def recording_engine_command(data):
        if isinstance(data, dict) and data.get("command") == "OFFER":
            offers_received.append(json.loads(json.dumps(data)))
        return await original_engine(data)

    backend._on_engine_command = recording_engine_command

    checks: Dict[str, Any] = {}
    completion: Optional[Dict[str, Any]] = None
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

        accepted_offer: Optional[Dict[str, Any]] = None
        if GATE3B and checks["offer_accept_sent"] and checks["mission_active"]:
            accepted_cid = next(e["payload"].get("commitmentId") for e in evidence["backend_emits"] if e["event"] == "OFFER_ACCEPT")
            accepted_offer = next((o for o in offers_received if o.get("commitmentId") == accepted_cid), None)
            if accepted_offer is not None:
                payload = accepted_offer["payload"]
                stops = sorted(payload["stopSequence"], key=lambda s: s["sequence"])
                legs = [[(p["lat"], p["lon"]) for p in stop["path"]] for stop in stops]
                TRACK.load(legs)
                checks["track_start_offset_m"] = round(_haversine_m((LAT, LON), legs[0][0]), 2)

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

        if GATE3B:
            # ── Phase C (Gate 3b): the completion half, instead of the obstacle ──
            completion = (await completion_phase(agent, backend, accepted_offer["payload"])
                          if accepted_offer is not None else {"error": "no accepted OFFER was captured"})
        else:
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
    common = [
        "authenticated", "esp32_link_up_and_motion_ready", "offer_admitted", "offer_accept_sent", "command_ack_sent",
        "mission_active", "drive_written", "drive_acked_accepted", "esp32_applied_output", "pi_saw_applied_output",
        "backend_telemetry_has_fix_quality", "backend_telemetry_has_released_latch", "probe_answered",
    ]
    if GATE3B:
        c = completion or {}
        # Every position the stand-in reported lies on the OFFER path of its leg.
        off_path = [distance_to_path_m((p["lat"], p["lon"]), TRACK.legs[p["leg"]]) for p in TRACK.positions] if TRACK.legs else []
        checks["gps_on_offer_path"] = bool(off_path) and max(off_path) <= 0.5
        checks["gps_max_distance_from_offer_path_m"] = round(max(off_path), 3) if off_path else None
        checks["gps_advanced_m"] = round(TRACK.travelled_m, 1)
        checks["gps_advanced"] = TRACK.travelled_m > 1.0
        checks["offer_paths_are_mission_paths"] = c.get("offer_paths_are_mission_paths") is True
        checks["custody_guard_holds"] = (
            (c.get("custody_refused_before_pickup") or {}).get("http_status") == 409
            and (c.get("custody_release_refused_before_pickup") or {}).get("http_status") == 409
            and (c.get("release_refused_at_pickup") or {}).get("http_status") == 409)
        checks["pickup_arrival"] = (c.get("pickup_leg") or {}).get("reached") is True
        checks["custody_acquired"] = (c.get("custody_acquired") or {}).get("ok") is True
        checks["custody_event_acquired"] = c.get("custody_event_acquired") is True
        checks["drive_to_drop"] = c.get("drop_leg_started") is True
        checks["drop_arrival"] = (c.get("drop_leg") or {}).get("reached") is True
        checks["custody_released"] = (c.get("custody_released") or {}).get("ok") is True
        checks["custody_event_released"] = c.get("custody_event_released") is True
        checks["task_complete_sent"] = c.get("task_complete_sent") is True
        checks["task_complete_acked"] = c.get("task_complete_backend_verdict") == "ACKED"
        report["completion"] = c
        report["gps_track_samples"] = TRACK.positions[:: max(1, len(TRACK.positions) // 40)]
        required = common + [
            "gps_advanced", "gps_on_offer_path", "offer_paths_are_mission_paths", "custody_guard_holds",
            "pickup_arrival", "custody_acquired", "custody_event_acquired", "drive_to_drop", "drop_arrival",
            "custody_released", "custody_event_released", "task_complete_sent", "task_complete_acked",
        ]
    else:
        required = common + [
            "esp32_blocked_forward", "drive_gated_front_obstacle", "no_drive_applied_while_blocked",
            "drive_after_obstacle_not_accepted", "pi_saw_forward_blocked", "pi_latch_report_matches_esp32",
        ]
    with open(REPORT, "w") as f:
        json.dump(report, f, indent=2, default=str)
    failed = [name for name in required if checks.get(name) is not True]
    ok = not failed and evidence["serial_devices_opened"] == 0
    print("[gate3] " + ("PASS" if ok else f"FAIL: {failed}"), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
