"""Measurement harness: what the backend link costs the rest of the agent.

Not a unit test and not auto-discovered -- it takes minutes, uses the real
camera, and reports numbers rather than asserting. Run it on the Pi itself;
figures from any other machine are meaningless.

    venv/bin/python tests/integration/soak_backend_link.py --seconds 120

It runs the full agent (camera -> perception -> localization -> navigation ->
decision) twice: once standalone, once with the backend link connected to a
local Socket.IO server, and prints the difference. The question it answers is
whether adding the link degrades perception or the agent loop.

SAFETY: no motors are touched. The agent publishes MotionIntent only, and
there is no motor driver in the process.
"""

from __future__ import annotations

import argparse
import asyncio
import dataclasses
import json
import os
import resource
import statistics
import time

import socketio
from aiohttp import web

from robotx.application.agent import RobotAgent
from robotx.communication.backend_link import BackendConfig, BackendLink
from robotx.communication.protocol import (
    ProtocolBinding,
    build_status_payload,
    build_telemetry_payload,
)
from robotx.config.settings import Settings
from robotx.diagnostics.health import read_cpu_temp_c
from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus
from robotx.localization.position import Position


NAMESPACE = "/robot"


class CountingServer:
    """A local Socket.IO server that only counts what arrives."""

    def __init__(self):
        self.sio = socketio.AsyncServer(async_mode="aiohttp")
        self.app = web.Application()
        self.sio.attach(self.app)
        self.counts = {"telemetry": 0, "status": 0, "robot_hello": 0, "command_ack": 0}
        self.first_connect_at = None
        self.runner = None
        self.port = None

        @self.sio.event(namespace=NAMESPACE)
        async def connect(sid, environ, auth=None):
            if self.first_connect_at is None:
                self.first_connect_at = time.monotonic()

        for name in self.counts:
            self._count(name)

    def _count(self, name):
        @self.sio.on(name, namespace=NAMESPACE)
        async def handler(sid, data, _name=name):
            self.counts[_name] += 1

    async def start(self):
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        site = web.TCPSite(self.runner, "127.0.0.1", 0)
        await site.start()
        self.port = site._server.sockets[0].getsockname()[1]

    async def stop(self):
        if self.runner is not None:
            await self.runner.cleanup()

    @property
    def url(self):
        return f"http://127.0.0.1:{self.port}"


def process_cpu_seconds() -> float:
    usage = resource.getrusage(resource.RUSAGE_SELF)
    return usage.ru_utime + usage.ru_stime


def max_rss_mb() -> float:
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


async def sample_agent(agent: RobotAgent, seconds: float, label: str) -> dict:
    """Watch a running agent for `seconds`, measuring loop health and cost."""

    tick_gaps = []
    last_update = None
    cpu_before = process_cpu_seconds()
    started = time.monotonic()

    while time.monotonic() - started < seconds:
        snapshot = agent.state.snapshot()
        if last_update is not None and snapshot.updated_at != last_update:
            tick_gaps.append(snapshot.updated_at - last_update)
        last_update = snapshot.updated_at
        await asyncio.sleep(0.05)

    elapsed = time.monotonic() - started
    cpu_used = process_cpu_seconds() - cpu_before
    snapshot = agent.state.snapshot()

    return {
        "label": label,
        "seconds": round(elapsed, 1),
        "cpu_percent_of_one_core": round(100.0 * cpu_used / elapsed, 1),
        "max_rss_mb": round(max_rss_mb(), 1),
        "cpu_temp_c": read_cpu_temp_c(),
        "state_updates_observed": len(tick_gaps),
        "median_update_gap_ms": round(1000 * statistics.median(tick_gaps), 2) if tick_gaps else None,
        "p95_update_gap_ms": (
            round(1000 * sorted(tick_gaps)[int(0.95 * len(tick_gaps))], 2)
            if len(tick_gaps) > 20
            else None
        ),
        "camera": snapshot.health.components.get("camera").status.value
        if "camera" in snapshot.health.components
        else "n/a",
        "perception_status": snapshot.perception.status.value,
        "perception_processing_ms": round(snapshot.perception.processing_ms, 2),
        "gps_status": snapshot.gps.status.value,
        "health": snapshot.health.status.value,
        "mode": snapshot.mode.value,
    }


def measure_serialization(agent: RobotAgent, iterations: int = 2000) -> dict:
    """Cost of building and encoding one telemetry payload.

    A fix is injected if the receiver does not have one, because a real
    measurement needs a *sendable* payload -- and the Pi correctly refuses to
    build one from a position it does not have. The injected fix never leaves
    this function: it is a copy of the snapshot, not the agent's state.
    """

    snapshot = agent.state.snapshot()

    if not snapshot.gps.has_fix:
        now = time.time()
        snapshot = dataclasses.replace(
            snapshot,
            gps=GpsReading(
                status=GPSStatus.FIX,
                fix=GpsFix(latitude=12.9716, longitude=77.5946, timestamp=now,
                           speed_mps=0.8, satellites=9),
                age_s=0.1,
            ),
            position=Position(latitude=12.9716, longitude=77.5946, timestamp=now,
                              speed_mps=0.8, satellites=9),
        )

    started = time.perf_counter()
    for _ in range(iterations):
        build_telemetry_payload(snapshot, sequence=1, max_position_age_s=5.0)
    build_us = 1e6 * (time.perf_counter() - started) / iterations

    frame = build_telemetry_payload(snapshot, sequence=1, max_position_age_s=5.0)
    payload = frame.payload
    status_payload = build_status_payload(
        snapshot, robot_id="soak", binding=ProtocolBinding()
    )

    started = time.perf_counter()
    for _ in range(iterations):
        json.dumps(payload)
    encode_us = 1e6 * (time.perf_counter() - started) / iterations

    return {
        "build_payload_us": round(build_us, 2),
        "json_encode_us": round(encode_us, 2),
        "telemetry_bytes": len(json.dumps(payload)),
        "status_bytes": len(json.dumps(status_payload)),
        "has_position": frame.has_position,
        "used_injected_fix": not agent.state.snapshot().gps.has_fix,
    }


async def run(seconds: float) -> None:
    settings = Settings.from_env(
        {
            **os.environ,
            "ROBOTX_ROBOT_ID": "robotx-pi-soak",
            "ROBOTX_LOG_LEVEL": "WARNING",
            "ROBOTX_SOCKET_ENABLED": "0",
        }
    )

    results = {}

    # --- phase 1: agent alone -------------------------------------------------
    agent = RobotAgent(settings)
    await agent.start()
    try:
        await asyncio.sleep(5.0)  # let the camera and detector settle
        results["without_backend"] = await sample_agent(agent, seconds, "agent only")
        results["serialization"] = measure_serialization(agent)
    finally:
        await agent.stop()

    # --- phase 2: agent + backend link ---------------------------------------
    server = CountingServer()
    await server.start()

    agent = RobotAgent(settings)
    await agent.start()
    cfg = BackendConfig(
        enabled=True,
        server_url=server.url,
        robot_id=settings.robot_id,
        robot_token="soak-token",
        telemetry_interval_s=settings.backend_telemetry_interval_s,
        status_interval_s=settings.backend_status_interval_s,
    )
    link = BackendLink(cfg, agent.state, agent)

    try:
        connect_started = time.monotonic()
        await link.start()
        connected = await link.wait_connected(10.0)
        connect_latency_ms = round(1000 * (time.monotonic() - connect_started), 1)

        await asyncio.sleep(5.0)
        results["with_backend"] = await sample_agent(agent, seconds, "agent + backend link")
        results["link"] = {
            "connected": connected,
            "connect_latency_ms": connect_latency_ms,
            "server_received": dict(server.counts),
            "stats": dict(link.stats),
            "integrated": link.describe()["integrated"],
        }
    finally:
        await link.stop()
        await agent.stop()
        await server.stop()

    # --- delta ----------------------------------------------------------------
    before = results["without_backend"]
    after = results["with_backend"]
    results["delta"] = {
        "cpu_percent_points": round(
            after["cpu_percent_of_one_core"] - before["cpu_percent_of_one_core"], 1
        ),
        "median_update_gap_ms": (
            round(after["median_update_gap_ms"] - before["median_update_gap_ms"], 2)
            if before["median_update_gap_ms"] and after["median_update_gap_ms"]
            else None
        ),
        "perception_processing_ms": round(
            after["perception_processing_ms"] - before["perception_processing_ms"], 2
        ),
        "max_rss_mb": round(after["max_rss_mb"] - before["max_rss_mb"], 1),
    }

    print(json.dumps(results, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=60.0,
                        help="measurement window per phase")
    args = parser.parse_args()
    asyncio.run(run(args.seconds))


if __name__ == "__main__":
    main()
