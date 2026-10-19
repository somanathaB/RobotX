"""Entry point: a local HTTP surface over the Pi robot agent.

    venv/bin/python -m robotx.application

That serves on `ROBOTX_API_HOST`:`ROBOTX_API_PORT` (default 127.0.0.1:8000, this
Pi only; see `run`). Reaching it from another machine is an explicit choice:
set `ROBOTX_API_HOST` to the interface to expose.

The HTTP layer is a thin window onto `RobotAgent`: it reads state and starts or
stops a mission. All the logic lives in the agent. Nothing here drives motors,
talks to an ESP32, or connects to a backend.

Endpoints are unauthenticated and intended for a trusted local network only.
With the backend link enabled RobotX admits missions, so the two routes that
put the robot into AUTO -- `/mission/start` and `/mission/resume` -- refuse
(409) and a RESUME comes only through the backend (see
`refuse_local_admission_in_engine_mode`).

One test-only exception, off by default: with `ROBOTX_TEST_INJECTION_ENABLED`
`POST /test/mission/inject` accepts a locally supplied `TEST-INJECTED-` task,
on loopback only and behind a bearer token, and hands it to the agent's own
`assess_offer` / `assign_mission` (see `robotx.application.injection`).
"""

from __future__ import annotations

import asyncio
import ipaddress
import json
import logging
from contextlib import asynccontextmanager
from typing import Any, AsyncGenerator, Dict, List, Optional

from fastapi import Depends, FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse, StreamingResponse
from pydantic import BaseModel, Field

from robotx.application.agent import RobotAgent
from robotx.application.bench_gate import MAX_BODY_BYTES as BENCH_MAX_BODY_BYTES, BenchGate, BenchRejected
from robotx.application.injection import MAX_BODY_BYTES, InjectionGate, InjectionRejected
from robotx.control.bench_motion import BenchRefused
from robotx.application.operator_access import OperatorAccess
from robotx.config.logging_setup import log_event, setup_logging
from robotx.config.settings import SETTINGS, Settings
from robotx.diagnostics.health import HealthStatus
from robotx.state.robot_state import MissionRefused
from robotx.state.telemetry import build_telemetry


logger = logging.getLogger(__name__)

MJPEG_BOUNDARY = "frame"
# ~5 s of empty polls at 0.05 s before giving up on a silent camera.
MJPEG_MAX_MISSED_FRAMES = 100


class AppState:
    """Holds the single agent instance for the lifetime of the process."""

    agent: Optional[RobotAgent] = None
    injection: Optional[InjectionGate] = None
    bench: Optional[BenchGate] = None
    operator_access: Optional[OperatorAccess] = None


state = AppState()


def get_agent() -> RobotAgent:
    if state.agent is None:
        raise HTTPException(status_code=503, detail="agent is not running")
    return state.agent


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    setup_logging(SETTINGS.log_level)
    state.operator_access = OperatorAccess.from_settings(SETTINGS)
    if state.operator_access.problem is not None:
        log_event(logger, "api.operator_access_misconfigured", state.operator_access.problem,
                  level=logging.ERROR)
    state.injection = InjectionGate.from_settings(SETTINGS)
    state.bench = BenchGate.from_settings(SETTINGS, other_token_digests=(SETTINGS.test_injection_token_sha256,))

    agent = RobotAgent(SETTINGS)
    try:
        await agent.start()
    except Exception:
        log_event(
            logger,
            "agent.start_failed",
            "agent failed to start",
            level=logging.CRITICAL,
            exc_info=True,
        )
        # Release whatever did come up before re-raising.
        await agent.stop()
        raise

    state.agent = agent
    try:
        yield
    finally:
        state.agent = None
        state.injection = None
        state.bench = None
        state.operator_access = None
        await agent.stop()


app = FastAPI(title="RobotX Pi Agent", version="2.0", lifespan=lifespan)


# --- read-only views ----------------------------------------------------------


@app.get("/health")
async def health() -> Dict[str, Any]:
    """Overall agent health, including per-subsystem status and host metrics."""

    agent = get_agent()
    snapshot = agent.state.snapshot()
    return {
        "robot_id": snapshot.robot_id,
        "mode": snapshot.mode.value,
        "uptime_s": round(snapshot.uptime_s, 1),
        "health": snapshot.health.to_dict(),
        "ok": snapshot.health.status is not HealthStatus.FAILED,
    }


@app.get("/state")
async def robot_state() -> Dict[str, Any]:
    """Full authoritative robot state."""

    return get_agent().state.snapshot().to_dict()


@app.get("/telemetry")
async def telemetry() -> Dict[str, Any]:
    """Current local telemetry payload (the same schema a backend would get)."""

    return build_telemetry(get_agent().state.snapshot())


@app.get("/config")
async def config() -> Dict[str, Any]:
    """Effective configuration. Secrets are reported as SET/UNSET only."""

    return SETTINGS.public_summary()


@app.get("/readiness")
async def readiness() -> Dict[str, Any]:
    """Every reason this Rover would refuse an offer right now, and whether a held lease is renewed.

    Read-only: the same `readiness_blockers` and `faults` (the active fault set,
    `robotx.diagnostics.faults`) the agent sends in TELEMETRY and applies in
    `assess_offer`, plus the e-stop latch, the lease predicate and perception
    (an offer-level refusal the readiness vocabulary has no code for).
    """

    agent = get_agent()
    snapshot = agent.state.snapshot()
    mission = snapshot.mission
    return {
        "blockers": agent.readiness_blockers(snapshot),
        "faults": agent.fault_report(snapshot),
        "perception_usable": snapshot.perception.is_usable,
        "estop_engaged": agent.emergency_stopped,
        "estop_reason": agent.safety.estop_reason,
        # Only meaningful for a held mission: null, not false, when there is none.
        "mission_lease_renewable": agent.mission_lease_renewable()
        if mission is not None and mission.is_active else None,
        "mission": None if mission is None else {
            "task_id": mission.task_id, "status": mission.status.value, "interrupted_by": mission.interrupted_by},
    }


@app.get("/backend")
async def backend_link() -> Dict[str, Any]:
    """Backend link state: connection, authentication, binding and counters.

    The place to look when a dashboard shows a robot as offline. `streaming`
    is the honest summary: the socket is open, FalconAut has authenticated this
    robot, and telemetry is flowing. A socket that is merely connected is not
    an integration, because FalconAut's connection is anonymous until AUTH
    succeeds.
    """

    agent = get_agent()
    if agent.backend is None:
        return {
            "enabled": SETTINGS.socket_enabled,
            "status": "DISABLED",
            "streaming": False,
            "authenticated": False,
            "detail": "no backend link is configured (ROBOTX_SOCKET_ENABLED=0)",
        }
    return agent.backend.describe()


# --- mission control ----------------------------------------------------------


BROWSER_REFUSAL = "browser requests are refused on this route"


def require_operator(request: Request) -> None:
    """Guard for the routes that release a stop or advance a mission. Never on a stop route.

    Always: a request carrying `Origin` is refused. Browsers send it on every
    POST -- a cross-site page's no-preflight "simple" request and a DNS-rebinding
    page alike -- while the documented operator procedure (curl over SSH) never
    does. Without this, any web page open on the Pi could clear the e-stop.

    Opt-in on top (`ROBOTX_OPERATOR_TOKEN_SHA256`, see
    `robotx.application.operator_access`): a bearer token and a loopback Host.
    """

    if request.headers.get("origin") is not None:
        log_event(logger, "api.operator_refused", BROWSER_REFUSAL, level=logging.WARNING,
                  route=request.url.path, status=403)
        raise HTTPException(status_code=403, detail=BROWSER_REFUSAL)
    access = state.operator_access
    if access is None:
        return
    refusal = access.refusal(
        authorization=request.headers.get("authorization"),
        origin=request.headers.get("origin"),
        host_header=request.headers.get("host"),
    )
    if refusal is not None:
        log_event(logger, "api.operator_refused", refusal[1], level=logging.WARNING,
                  route=request.url.path, status=refusal[0])
        raise HTTPException(status_code=refusal[0], detail=refusal[1])


OPERATOR = [Depends(require_operator)]


class Waypoint(BaseModel):
    lat: float = Field(ge=-90.0, le=90.0)
    lon: float = Field(ge=-180.0, le=180.0)


class MissionRequest(BaseModel):
    """A mission is an ordered list of waypoints, ending at the destination."""

    waypoints: List[Waypoint] = Field(min_length=1)


ENGINE_MODE_REFUSAL = "engine mode: missions come from RobotX; resume from the dashboard"


def refuse_local_admission_in_engine_mode(agent: RobotAgent, route: str) -> None:
    """409 before anything changes, whenever the backend link is enabled.

    With `ROBOTX_SOCKET_ENABLED` RobotX is the authority on what this robot
    drives: a mission arrives as a signed, fenced OFFER and a RESUME as an
    authenticated backend command, which also proves the link is up. A local
    start would replace the route of the engine mission it holds (whose lease
    the heartbeat keeps renewing), and a local resume would undo the pause the
    backend-loss policy took while RobotX may be reassigning the Leg. Decided
    on configuration, not on the link's current state: a link that is down or
    never started is exactly when a local AUTO must not happen. Bench mode
    (link disabled) is unaffected.
    """

    if agent.settings.socket_enabled:
        log_event(logger, "api.admission_refused", ENGINE_MODE_REFUSAL, level=logging.WARNING,
                  route=route, mode=agent.state.mode.value)
        raise HTTPException(status_code=409, detail=ENGINE_MODE_REFUSAL)


@app.post("/mission/start")
async def start_mission(request: MissionRequest) -> Dict[str, Any]:
    """Load a local waypoint route and switch the agent to AUTO. Bench mode only.

    The agent publishes motion intent only -- nothing moves until an ESP32 is
    connected and chooses to honour that intent.
    """

    agent = get_agent()
    refuse_local_admission_in_engine_mode(agent, "/mission/start")
    agent.start_mission([(w.lat, w.lon) for w in request.waypoints])
    return {"mode": agent.state.mode.value, "waypoints": len(request.waypoints)}


@app.post("/mission/stop")
async def stop_mission() -> Dict[str, Any]:
    agent = get_agent()
    agent.stop_mission("stopped via API")
    return {"mode": agent.state.mode.value}


@app.post("/mission/pause")
async def pause_mission() -> Dict[str, Any]:
    """Suspend the mission, keeping the route. The same path a backend PAUSE takes."""

    agent = get_agent()
    agent.pause_mission("paused via API")
    return {"mode": agent.state.mode.value}


@app.post("/mission/resume")
async def resume_mission() -> Dict[str, Any]:
    """Return a paused mission to AUTO, or 409 if there is nothing to resume. Bench mode only."""

    agent = get_agent()
    refuse_local_admission_in_engine_mode(agent, "/mission/resume")
    try:
        agent.resume_mission("resumed via API")
    except MissionRefused as e:
        raise HTTPException(status_code=409, detail=str(e)) from None
    return {"mode": agent.state.mode.value}


@app.post("/safety/estop")
async def emergency_stop() -> Dict[str, Any]:
    """Latch the safety gate shut. No motion intent leaves the Pi until cleared.

    Unconditional by design: an emergency stop must work from every state,
    including ERROR, and must never depend on the agent agreeing it was needed.
    """

    agent = get_agent()
    agent.emergency_stop("emergency stop via API")
    return {"mode": agent.state.mode.value, "emergency_stopped": True}


@app.post("/safety/clear", dependencies=OPERATOR)
async def clear_emergency_stop() -> Dict[str, Any]:
    """Release the emergency stop latch. Does not resume the mission."""

    agent = get_agent()
    released = agent.clear_emergency_stop("cleared via API")
    return {
        "mode": agent.state.mode.value,
        "emergency_stopped": agent.emergency_stopped,
        "was_engaged": released,
    }


@app.post("/controller/reset", dependencies=OPERATOR)
async def reset_controller() -> Dict[str, Any]:
    """Ask the ESP32 to clear its safety-stop and command-timeout latches, once.

    An operator safety action, like /safety/estop and /safety/clear, so not
    behind the engine-mode admission lock. 409 when refused (mission AUTO, link
    not ready, TELEMETRY stale, reboot latched, a RESET already pending), and
    then nothing is sent. 200 means the RESET will be written once -- not that
    the controller is clear: watch `controller.reset_status` and the ESP32's
    TELEMETRY in /state. Does not resume the mission.
    """

    agent = get_agent()
    try:
        agent.reset_controller("reset via API")
    except MissionRefused as e:
        raise HTTPException(status_code=409, detail=str(e)) from None
    return {
        "mode": agent.state.mode.value,
        "reset_status": agent.esp32.status().controller.reset_status,
    }


@app.post("/mission/custody", dependencies=OPERATOR)
async def confirm_custody(body: Dict[str, Any]) -> Dict[str, Any]:
    """An operator at the stop confirms a parcel handover: {"kind": ACQUIRED|RELEASED}.

    Only with ROBOTX_CUSTODY_CONFIRMATION=operator. Recorded as
    OPERATOR_CONFIRMED -- a person's statement, not a sensor reading -- and
    refused (409) unless the Rover is at the stop that handover belongs to.
    """

    agent = get_agent()
    if not agent.custody_sensing_available():
        raise HTTPException(status_code=409, detail="operator custody confirmation is not enabled on this Rover")
    kind = str((body or {}).get("kind", "")).upper()
    if kind not in ("ACQUIRED", "RELEASED"):
        raise HTTPException(status_code=422, detail='kind must be "ACQUIRED" or "RELEASED"')
    try:
        active = agent.record_custody(kind, source="OPERATOR_CONFIRMED")
    except MissionRefused as e:
        raise HTTPException(status_code=409, detail=str(e)) from None
    return {"kind": kind, "mission": None if active is None else active.to_dict()}


@app.post("/mission/idle", dependencies=OPERATOR)
async def resume_idle() -> Dict[str, Any]:
    """Leave STOPPED/ERROR for IDLE; 409 while a mission is still held (stop or resume it)."""

    agent = get_agent()
    try:
        agent.resume_idle()
    except MissionRefused as e:
        raise HTTPException(status_code=409, detail=str(e)) from None
    return {"mode": agent.state.mode.value}


# --- test-only mission injection (off by default) -----------------------------


def _enabled_injection_gate() -> InjectionGate:
    """The gate, or the same 404 an unknown route gets: disabled is absent."""

    gate = state.injection
    if gate is None or not gate.enabled:
        raise HTTPException(status_code=404, detail="Not Found")
    return gate


def _check_injection_request(gate: InjectionGate, request: Request, method: str) -> None:
    if request.method != method:
        raise HTTPException(status_code=405, detail="Method Not Allowed")
    server = request.scope.get("server")
    gate.check_origin(
        client_host=request.client.host if request.client else None,
        host_header=request.headers.get("host"),
        origin=request.headers.get("origin"),
        server_host=server[0] if server else None,
    )


# Every method is routed here so that, while disabled, any request answers the
# same 404 an unknown path does -- a 405 would say the route exists.
_ALL_METHODS = ["GET", "HEAD", "POST", "PUT", "PATCH", "DELETE", "OPTIONS"]


@app.api_route("/test/injection", methods=_ALL_METHODS)
async def test_injection_status(request: Request) -> Any:
    """Whether the test-only ingress is available. Read-only; no task details."""

    gate = _enabled_injection_gate()
    try:
        _check_injection_request(gate, request, "GET")
    except InjectionRejected as e:
        return JSONResponse(status_code=e.status, content=e.to_dict())
    return gate.status()


@app.api_route("/test/mission/inject", methods=_ALL_METHODS)
async def inject_test_mission(request: Request) -> Any:
    """Offer a locally supplied TEST-INJECTED- task to the agent. Test-only, off by default.

    200 when the agent accepted it, 409 when the agent (or a conflict check)
    refused it, 4xx/503 when the request never reached the agent. Nothing here
    moves the Rover: an accepted task is driven, or not, by the agent loop and
    every gate it already has.
    """

    gate = _enabled_injection_gate()
    try:
        _check_injection_request(gate, request, "POST")
        gate.check_available()
        gate.authorize(request.headers.get("authorization"))
        body = await request.body()
        if len(body) > MAX_BODY_BYTES:
            raise InjectionRejected("TOO_LARGE", f"request body exceeds {MAX_BODY_BYTES} bytes", 413)
        try:
            raw = json.loads(body)
        except ValueError:
            raise InjectionRejected("MALFORMED", "the request body is not JSON") from None
        # No await from here to the verdict: the agent loop cannot tick in between.
        result = gate.inject(get_agent(), raw)
    except InjectionRejected as e:
        return JSONResponse(status_code=e.status, content=e.to_dict())
    return JSONResponse(status_code=200 if result["outcome"] == "ACCEPTED" else 409, content=result)


# --- bench motor pulse (off by default) -----------------------------------------


def _enabled_bench_gate() -> BenchGate:
    gate = state.bench
    if gate is None or not gate.enabled:
        raise HTTPException(status_code=404, detail="Not Found")
    return gate


def _check_bench_request(gate: BenchGate, request: Request, method: str) -> None:
    if request.method != method:
        raise HTTPException(status_code=405, detail="Method Not Allowed")
    server = request.scope.get("server")
    gate.check_origin(
        client_host=request.client.host if request.client else None,
        host_header=request.headers.get("host"),
        origin=request.headers.get("origin"),
        server_host=server[0] if server else None,
    )


@app.api_route("/test/bench-motion", methods=_ALL_METHODS)
async def bench_motion_status(request: Request) -> Any:
    """The bench route's availability, limits and the last pulse's result. Read-only."""

    gate = _enabled_bench_gate()
    try:
        _check_bench_request(gate, request, "GET")
    except BenchRejected as e:
        return JSONResponse(status_code=e.status, content=e.to_dict())
    return gate.status(get_agent())


@app.api_route("/test/bench-motion/run", methods=_ALL_METHODS)
async def bench_motion_run(request: Request) -> Any:
    """Ask the agent for ONE bounded forward pulse (robotx.control.bench_motion). Off by default.

    200: the pulse started (it runs on the agent's tick and stops on its own;
    poll GET /test/bench-motion for the result). 409: the agent refused it, with
    the precondition that failed. Stop it any time with POST /safety/estop or
    POST /mission/stop.
    """

    gate = _enabled_bench_gate()
    try:
        _check_bench_request(gate, request, "POST")
        gate.check_available()
        gate.authorize(request.headers.get("authorization"))
        body = await request.body()
        if len(body) > BENCH_MAX_BODY_BYTES:
            raise BenchRejected("TOO_LARGE", f"request body exceeds {BENCH_MAX_BODY_BYTES} bytes", 413)
        try:
            raw = json.loads(body)
        except ValueError:
            raise BenchRejected("MALFORMED", "the request body is not JSON") from None
        result = gate.run(get_agent(), raw)
    except BenchRejected as e:
        return JSONResponse(status_code=e.status, content=e.to_dict())
    except BenchRefused as e:
        return JSONResponse(status_code=409, content={"code": e.code, "detail": e.detail})
    return JSONResponse(status_code=200, content=result)


# --- camera stream ------------------------------------------------------------


async def _mjpeg_frames(agent: RobotAgent) -> AsyncGenerator[bytes, None]:
    # End the response if the camera stops producing rather than holding the
    # connection open forever: a client watching a dead camera should see the
    # stream close, not an indefinite silence it cannot distinguish from a
    # still scene.
    missed = 0
    while True:
        camera = agent.camera
        if camera is None:
            return

        jpeg = await asyncio.to_thread(camera.get_jpeg)
        if jpeg is None:
            missed += 1
            if missed > MJPEG_MAX_MISSED_FRAMES:
                log_event(
                    logger,
                    "camera.stream_ended",
                    "no frames available; closing MJPEG stream",
                    level=logging.WARNING,
                    waited_s=round(missed * 0.05, 1),
                )
                return
            await asyncio.sleep(0.05)
            continue

        missed = 0

        yield (
            f"--{MJPEG_BOUNDARY}\r\n"
            f"Content-Type: image/jpeg\r\n"
            f"Content-Length: {len(jpeg)}\r\n\r\n"
        ).encode()
        yield jpeg + b"\r\n"
        await asyncio.sleep(0.05)


@app.get("/camera")
async def camera_stream() -> StreamingResponse:
    """MJPEG preview of the live camera, for local verification."""

    agent = get_agent()
    if agent.camera is None:
        raise HTTPException(status_code=503, detail="camera is not available")

    return StreamingResponse(
        _mjpeg_frames(agent),
        media_type=f"multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY}",
    )


# --- launch -------------------------------------------------------------------


def _is_loopback(host: str) -> bool:
    if host.strip().lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(host.strip()).is_loopback
    except ValueError:
        return False


def run(settings: Settings = SETTINGS) -> None:
    """Serve the API on `ROBOTX_API_HOST`:`ROBOTX_API_PORT`, the one launch path.

    The default host is loopback: the routes are unauthenticated, so being
    reachable from the network is something the deployment states (for
    example the interface an operator's custody confirmation arrives on), never
    something it gets by default. Anything else is served as configured, and
    said so in the log.
    """

    import uvicorn

    setup_logging(settings.log_level)
    if not _is_loopback(settings.api_host):
        log_event(
            logger,
            "api.network_exposed",
            "local API is reachable from other machines; its routes are unauthenticated",
            level=logging.WARNING,
            host=settings.api_host,
            port=settings.api_port,
        )
    uvicorn.run(app, host=settings.api_host, port=settings.api_port)
