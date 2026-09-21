import asyncio
import logging
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Dict, Optional

import socketio  # type: ignore


logger = logging.getLogger(__name__)


CommandHandler = Callable[[Dict[str, Any]], Awaitable[None]]


@dataclass
class SocketConfig:
    server_url: str
    namespace: str
    robot_id: str
    reconnect: bool = True
    # Bearer credential for the connect handshake. See RobotSocketClient.connect()
    # and ROBOTX_PI_REMEDIATION_PLAN.md (R-02) -- the server-side verification of
    # this token is a backend integration requirement, not implemented in this repo.
    robot_token: Optional[str] = None


class RobotSocketClient:
    """Async Socket.IO client for robot <-> server communication."""

    def __init__(self, cfg: SocketConfig) -> None:
        self.cfg = cfg
        self.sio = socketio.AsyncClient(reconnection=cfg.reconnect, logger=False, engineio_logger=False)
        self._connected = asyncio.Event()

        self._command_queue: asyncio.Queue[Dict[str, Any]] = asyncio.Queue()
        self._telemetry_queue: asyncio.Queue[Dict[str, Any]] = asyncio.Queue(maxsize=5)

        self._telemetry_task: Optional[asyncio.Task] = None
        self._cmd_task: Optional[asyncio.Task] = None

        self._setup_handlers()

    def _setup_handlers(self) -> None:
        @self.sio.event(namespace=self.cfg.namespace)
        async def connect():
            logger.info("Socket connected")
            self._connected.set()
            await self.sio.emit("robot_hello", {"robot_id": self.cfg.robot_id}, namespace=self.cfg.namespace)

        @self.sio.event(namespace=self.cfg.namespace)
        async def disconnect():
            logger.warning("Socket disconnected")
            self._connected.clear()

        @self.sio.on("command", namespace=self.cfg.namespace)
        async def on_command(data: Any):
            # data expected: {type: START|STOP|RETURN|MANUAL, payload: {...}}
            if not isinstance(data, dict):
                return
            await self._command_queue.put(data)

        @self.sio.on("manual", namespace=self.cfg.namespace)
        async def on_manual(data: Any):
            if not isinstance(data, dict):
                data = {"type": "MANUAL", "payload": {"raw": data}}
            else:
                data = {"type": "MANUAL", "payload": data}
            await self._command_queue.put(data)

    async def connect(self) -> None:
        """Connect and authenticate via python-socketio's handshake-level `auth`
        payload (delivered before any event is processed -- this is the
        standard, non-invented mechanism this library provides for exactly
        this purpose).

        This repo can only implement the client side: it sends the robot's
        identity and token. Verifying that token and rejecting an unknown or
        mismatched one is a REQUIRED BACKEND INTEGRATION -- no such server
        exists in this repository (confirmed absent by the audit). Until a
        verifying backend is deployed, this only labels the gap; it does not
        close it by itself.
        """
        auth: Dict[str, Any] = {"robot_id": self.cfg.robot_id}
        if self.cfg.robot_token:
            auth["token"] = self.cfg.robot_token
        else:
            logger.warning(
                "ROBOTX_ROBOT_TOKEN is not set -- connecting without an auth token. "
                "Any party able to reach this socket namespace can currently issue "
                "commands to this robot. Do not use this configuration on an "
                "untrusted network."
            )
        await self.sio.connect(self.cfg.server_url, namespaces=[self.cfg.namespace], auth=auth)

    async def close(self) -> None:
        try:
            if self._telemetry_task:
                self._telemetry_task.cancel()
            if self._cmd_task:
                self._cmd_task.cancel()
        finally:
            try:
                await self.sio.disconnect()
            except Exception:
                pass

    async def wait_connected(self, timeout_s: float = 10.0) -> bool:
        try:
            await asyncio.wait_for(self._connected.wait(), timeout=timeout_s)
            return True
        except asyncio.TimeoutError:
            return False

    async def emit_status(self, status: Dict[str, Any]) -> None:
        await self.sio.emit("status", status, namespace=self.cfg.namespace)

    async def enqueue_telemetry(self, telemetry: Dict[str, Any]) -> None:
        # Drop old telemetry if queue is full.
        if self._telemetry_queue.full():
            try:
                _ = self._telemetry_queue.get_nowait()
            except Exception:
                pass
        await self._telemetry_queue.put(telemetry)

    async def telemetry_loop(self, interval_s: float = 1.5) -> None:
        while True:
            telemetry = await self._telemetry_queue.get()
            try:
                await self.sio.emit("telemetry", telemetry, namespace=self.cfg.namespace)
            except Exception as e:
                logger.warning("Telemetry emit failed: %s", e)
            # Controller already controls enqueue cadence; don't double-throttle here.
            await asyncio.sleep(0)

    async def command_loop(self, handler: CommandHandler) -> None:
        while True:
            cmd = await self._command_queue.get()
            try:
                await handler(cmd)
            except Exception as e:
                logger.exception("Command handler failed: %s", e)

    def start_background_tasks(self, handler: CommandHandler, telemetry_interval_s: float) -> None:
        if self._telemetry_task is None or self._telemetry_task.done():
            self._telemetry_task = asyncio.create_task(self.telemetry_loop(interval_s=telemetry_interval_s))
        if self._cmd_task is None or self._cmd_task.done():
            self._cmd_task = asyncio.create_task(self.command_loop(handler))
