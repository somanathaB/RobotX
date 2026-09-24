"""Entry point: a local HTTP surface over the Pi robot agent.

    venv/bin/python -m uvicorn robotx.application.main:app --host 0.0.0.0 --port 8000

The HTTP layer is a thin window onto `RobotAgent`: it reads state and starts or
stops a mission. All the logic lives in the agent. Nothing here drives motors,
talks to an ESP32, or connects to a backend.

Endpoints are unauthenticated and intended for a trusted local network only.
"""

from __future__ import annotations

import asyncio
import logging
from contextlib import asynccontextmanager
from typing import Any, AsyncGenerator, Dict, List, Optional

from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field

from robotx.application.agent import RobotAgent
from robotx.config.logging_setup import log_event, setup_logging
from robotx.config.settings import SETTINGS
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


state = AppState()


def get_agent() -> RobotAgent:
    if state.agent is None:
        raise HTTPException(status_code=503, detail="agent is not running")
    return state.agent


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    setup_logging(SETTINGS.log_level)

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


class Waypoint(BaseModel):
    lat: float = Field(ge=-90.0, le=90.0)
    lon: float = Field(ge=-180.0, le=180.0)


class MissionRequest(BaseModel):
    """A mission is an ordered list of waypoints, ending at the destination."""

    waypoints: List[Waypoint] = Field(min_length=1)


@app.post("/mission/start")
async def start_mission(request: MissionRequest) -> Dict[str, Any]:
    """Load a local waypoint route and switch the agent to AUTO.

    The agent publishes motion intent only -- nothing moves until an ESP32 is
    connected and chooses to honour that intent.
    """

    agent = get_agent()
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
    """Return a paused mission to AUTO, or 409 if there is nothing to resume."""

    agent = get_agent()
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


@app.post("/safety/clear")
async def clear_emergency_stop() -> Dict[str, Any]:
    """Release the emergency stop latch. Does not resume the mission."""

    agent = get_agent()
    released = agent.clear_emergency_stop("cleared via API")
    return {
        "mode": agent.state.mode.value,
        "emergency_stopped": agent.emergency_stopped,
        "was_engaged": released,
    }


@app.post("/mission/idle")
async def resume_idle() -> Dict[str, Any]:
    agent = get_agent()
    agent.resume_idle()
    return {"mode": agent.state.mode.value}


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
