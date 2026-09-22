"""The Pi's single link to the FalconAut backend.

One object owns the whole boundary: the socket, its lifecycle, the outbound
rates and the inbound dispatch. There is exactly one Socket.IO client in this
repository and it is here.

What this deliberately does not import
--------------------------------------
`RPi.GPIO`, any motor driver, the camera, the GPS reader, the navigator, the
decision layer. It reads a `RobotSnapshot` and calls four mission methods.
That is the entire coupling, which is what keeps the future ESP32 link
independent of this one: swapping either side out touches no code in the other.

Reconnection is ours, not the library's
---------------------------------------
`python-socketio` can reconnect by itself, but its retries are invisible to the
rest of the agent -- state would sit at CONNECTED while the library quietly
failed in a loop. Since a dashboard's "online" indicator is downstream of that
state, the link drives its own connect loop with `reconnection=False`, so every
attempt, failure and backoff is observable and reportable.

Behaviour when the backend is unavailable
-----------------------------------------
The Pi keeps running. Perception, localization, navigation and the decision
layer are unaffected: the backend is a supervisor, not a controller, and losing
it must never be able to *start* motion. What losing it can do is stop motion,
after `loss_grace_s`, under the `pause` policy -- an autonomous robot driving
across a campus with no operator watching is a supervision gap, and a brief
Wi-Fi drop should not strand it mid-route either. Set the policy to `continue`
where the mission genuinely must outlive the link.
"""

from __future__ import annotations

import asyncio
import logging
import random
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, Optional
from urllib.parse import urlparse

import socketio  # type: ignore

from robotx.communication.commands import CommandExecutor, CommandTarget, rejection_outcome
from robotx.communication.protocol import (
    CommandRejection,
    CommandStatus,
    EventLevel,
    InboundCommand,
    ProtocolBinding,
    agent_capabilities,
    build_command_result_payload,
    build_event_payload,
    build_register_payload,
    build_status_payload,
    build_telemetry_payload,
    parse_command,
    redact,
)
from robotx.config.logging_setup import log_event
from robotx.state.robot_state import LinkStatus, OperatingMode, RobotState


logger = logging.getLogger(__name__)

AGENT_VERSION = "2.1"

# How often the publish loop wakes. Each channel decides from its own elapsed
# time whether it is due, so this only bounds scheduling jitter.
_PUBLISH_TICK_S = 0.25

# Identical event messages inside this window are emitted once. Stops a
# flapping subsystem from writing thousands of Event rows.
_EVENT_DEDUPE_S = 30.0
# Hard ceiling on event emissions, whatever their content.
_EVENT_MIN_INTERVAL_S = 1.0


class BackendLossPolicy(str, Enum):
    """What an active mission does when the backend link is lost."""

    PAUSE = "pause"        # suspend the mission after the grace period
    CONTINUE = "continue"  # keep driving; the link is not a safety interlock


@dataclass(frozen=True)
class BackendConfig:
    """Everything the link needs, resolved once at startup."""

    enabled: bool = False
    server_url: str = "http://localhost:3000"
    robot_id: str = "robotx-pi"
    robot_token: Optional[str] = None
    binding: ProtocolBinding = ProtocolBinding()

    # Outbound rates. Telemetry is the only high-frequency channel and it is
    # still an order of magnitude below the camera's 20 FPS: one database row
    # per second per robot is a dashboard, twenty is a landfill.
    telemetry_interval_s: float = 1.0
    status_interval_s: float = 5.0
    # A position older than this is not sent as telemetry at all.
    max_position_age_s: float = 5.0

    # Inbound guards.
    command_max_age_s: Optional[float] = 120.0

    # Connect backoff.
    backoff_initial_s: float = 1.0
    backoff_max_s: float = 60.0
    # Auth failures do not resolve themselves by retrying quickly.
    backoff_rejected_s: float = 60.0

    # Transport security.
    tls_verify: bool = True

    loss_policy: BackendLossPolicy = BackendLossPolicy.PAUSE
    loss_grace_s: float = 30.0

    @classmethod
    def from_settings(cls, settings: Any, *, binding: Optional[ProtocolBinding] = None) -> "BackendConfig":
        if binding is None:
            binding = ProtocolBinding.load(getattr(settings, "protocol_file", None))
        try:
            policy = BackendLossPolicy(str(settings.backend_loss_policy).strip().lower())
        except ValueError:
            policy = BackendLossPolicy.PAUSE
        return cls(
            enabled=settings.socket_enabled,
            server_url=settings.socket_server_url,
            robot_id=settings.robot_id,
            robot_token=settings.robot_token,
            binding=binding,
            telemetry_interval_s=settings.backend_telemetry_interval_s,
            status_interval_s=settings.backend_status_interval_s,
            max_position_age_s=settings.backend_max_position_age_s,
            command_max_age_s=settings.backend_command_max_age_s,
            backoff_initial_s=settings.backend_backoff_initial_s,
            backoff_max_s=settings.backend_backoff_max_s,
            tls_verify=settings.backend_tls_verify,
            loss_policy=policy,
            loss_grace_s=settings.backend_loss_grace_s,
        )

    @property
    def uses_tls(self) -> bool:
        return urlparse(self.server_url).scheme in ("https", "wss")


ClientFactory = Callable[["BackendConfig"], Any]


def _default_client_factory(cfg: BackendConfig) -> Any:
    # reconnection=False: see the module docstring. `logger`/`engineio_logger`
    # stay off because the library logs full payloads, which would put the
    # handshake credential into the Pi's log files.
    return socketio.AsyncClient(
        reconnection=False,
        logger=False,
        engineio_logger=False,
        ssl_verify=cfg.tls_verify,
    )


class BackendLink:
    """Connects, publishes, receives commands, and reports its own truth."""

    def __init__(
        self,
        cfg: BackendConfig,
        state: RobotState,
        target: CommandTarget,
        *,
        client_factory: ClientFactory = _default_client_factory,
    ) -> None:
        self.cfg = cfg
        self.state = state
        self.executor = CommandExecutor(target)
        self._target = target
        self._client_factory = client_factory

        self._sio: Optional[Any] = None
        self._task: Optional[asyncio.Task] = None
        self._running = False
        self._connected = asyncio.Event()

        self._status = LinkStatus.DISABLED
        self._detail = ""
        self._attempt = 0
        self._last_connect_error: str = ""
        self._registered_ack: Optional[Dict[str, Any]] = None

        self._last_telemetry_t = 0.0
        self._last_status_t = 0.0
        self._last_event_t = 0.0
        self._recent_events: Dict[str, float] = {}
        self._disconnected_since: Optional[float] = None
        self._loss_action_taken = False

        # Counters, reported over the local HTTP API and used by tests.
        self.stats: Dict[str, int] = {
            "connects": 0,
            "disconnects": 0,
            "connect_failures": 0,
            "telemetry_sent": 0,
            "telemetry_skipped": 0,
            "status_sent": 0,
            "events_sent": 0,
            "events_suppressed": 0,
            "commands_received": 0,
            "commands_rejected": 0,
            "commands_duplicate": 0,
            "acks_sent": 0,
            "emit_failures": 0,
            "unexpected_events": 0,
        }

    # --- lifecycle ------------------------------------------------------------

    async def start(self) -> None:
        """Begin connecting. Returns immediately; the link runs in the background.

        Startup never blocks on the backend and never fails because of it: a
        robot that will not boot without a dashboard is a robot that cannot be
        recovered when the dashboard is what broke.
        """

        if self._task is not None:
            return

        if not self.cfg.enabled:
            self._set_status(LinkStatus.DISABLED, "backend link disabled by configuration")
            return

        self._warn_about_configuration()
        self._running = True
        self._set_status(LinkStatus.CONNECTING, "starting")
        self._task = asyncio.create_task(self._run(), name="backend-link")

    async def stop(self) -> None:
        """Disconnect and release the socket. Safe to call when never started."""

        self._running = False
        task, self._task = self._task, None
        if task is not None:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            except Exception:
                logger.exception("Backend link raised during shutdown")

        sio = self._sio
        self._sio = None
        if sio is not None:
            try:
                await sio.disconnect()
            except Exception:
                logger.debug("Ignoring error while disconnecting backend socket", exc_info=True)

        self._connected.clear()
        self._set_status(
            LinkStatus.DISABLED if not self.cfg.enabled else LinkStatus.DISCONNECTED,
            "link stopped",
        )

    def _warn_about_configuration(self) -> None:
        """Say out loud, once, what this link does and does not guarantee."""

        if self.cfg.binding.is_provisional:
            log_event(
                logger,
                "backend.protocol_provisional",
                "backend event names are PROVISIONAL and were never verified against a "
                "server; set ROBOTX_PROTOCOL_FILE to the real contract before treating "
                "this link as integrated",
                level=logging.WARNING,
                namespace=self.cfg.binding.namespace,
            )
        if not self.cfg.robot_token:
            log_event(
                logger,
                "backend.no_token",
                "ROBOTX_ROBOT_TOKEN is not set; connecting with no credential. Anyone "
                "who can reach this namespace can command this robot.",
                level=logging.WARNING,
            )
        if not self.cfg.uses_tls:
            log_event(
                logger,
                "backend.no_tls",
                "backend URL is not https/wss: the credential and all telemetry travel "
                "in clear text",
                level=logging.WARNING,
                scheme=urlparse(self.cfg.server_url).scheme,
            )

    # --- connection state -----------------------------------------------------

    @property
    def status(self) -> LinkStatus:
        return self._status

    @property
    def connected(self) -> bool:
        return self._status is LinkStatus.CONNECTED

    def _set_status(self, status: LinkStatus, detail: str = "") -> None:
        """Record link state in exactly one place: the authoritative RobotState.

        Called from the socket's own callbacks, so what the agent reports and
        what the transport is doing cannot diverge.
        """

        changed = status is not self._status
        self._status = status
        self._detail = detail
        self.state.update_communication(
            backend=status,
            backend_detail=detail,
            backend_since=time.time(),
            backend_protocol_provisional=self.cfg.binding.is_provisional,
        )
        if changed:
            log_event(
                logger,
                "backend.link_status",
                detail,
                level=logging.WARNING if status is not LinkStatus.CONNECTED else logging.INFO,
                status=status.value,
            )

    async def wait_connected(self, timeout_s: float = 10.0) -> bool:
        try:
            await asyncio.wait_for(self._connected.wait(), timeout=timeout_s)
            return True
        except asyncio.TimeoutError:
            return False

    def next_backoff_s(self, attempt: int) -> float:
        """Exponential backoff with jitter, capped.

        Jitter is not decoration: a fleet of robots that lost the same backend
        would otherwise reconnect in lockstep and knock it over again the
        moment it came back.
        """

        base = min(self.cfg.backoff_max_s, self.cfg.backoff_initial_s * (2 ** max(0, attempt)))
        return base * (0.5 + random.random() * 0.5)

    # --- the connect/publish loop --------------------------------------------

    async def _run(self) -> None:
        while self._running:
            try:
                await self._connect_once()
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001 - the link must never die
                self.stats["connect_failures"] += 1
                self._last_connect_error = repr(e)[:200]
                rejected = _looks_like_rejection(e)
                self._set_status(
                    LinkStatus.REJECTED if rejected else LinkStatus.DISCONNECTED,
                    f"connect failed: {_safe_error(e)}",
                )
                delay = (
                    self.cfg.backoff_rejected_s if rejected else self.next_backoff_s(self._attempt)
                )
                self._attempt += 1
                await self._sleep_while_running(delay)
                continue

            # Connected. Publish until the link drops.
            self._attempt = 0
            try:
                await self._publish_loop()
            except asyncio.CancelledError:
                raise
            except Exception:
                logger.exception("Backend publish loop failed; reconnecting")

            if self._running:
                await self._sleep_while_running(self.next_backoff_s(self._attempt))
                self._attempt += 1

    async def _sleep_while_running(self, delay: float) -> None:
        """Sleep, but also run the link-loss policy while disconnected."""

        deadline = time.monotonic() + max(0.0, delay)
        while self._running and time.monotonic() < deadline:
            self._apply_loss_policy()
            await asyncio.sleep(min(0.5, max(0.05, deadline - time.monotonic())))

    async def _connect_once(self) -> None:
        self._set_status(LinkStatus.CONNECTING, f"attempt {self._attempt + 1}")

        sio = self._client_factory(self.cfg)
        self._sio = sio
        self._register_handlers(sio)

        # The credential goes in the Engine.IO handshake, which the server sees
        # before it processes a single event -- the only point at which it can
        # refuse a connection outright. It is never repeated in an event
        # payload, where a server's event log would capture it.
        auth: Dict[str, Any] = {"robotId": self.cfg.robot_id}
        if self.cfg.robot_token:
            auth["token"] = self.cfg.robot_token

        await sio.connect(
            self.cfg.server_url,
            namespaces=[self.cfg.binding.namespace],
            auth=auth,
        )

    def _register_handlers(self, sio: Any) -> None:
        ns = self.cfg.binding.namespace

        @sio.event(namespace=ns)
        async def connect() -> None:  # noqa: D401
            self.stats["connects"] += 1
            self._connected.set()
            self._disconnected_since = None
            self._loss_action_taken = False
            self._set_status(LinkStatus.CONNECTED, f"connected to {_safe_url(self.cfg.server_url)}")
            await self._send_register()

        @sio.event(namespace=ns)
        async def disconnect() -> None:  # noqa: D401
            self.stats["disconnects"] += 1
            self._connected.clear()
            self._disconnected_since = time.monotonic()
            self._set_status(LinkStatus.DISCONNECTED, "server closed the connection")

        @sio.event(namespace=ns)
        async def connect_error(data: Any) -> None:  # noqa: D401
            # Carries the server's rejection message when a connect handler
            # refused the handshake. Redacted: a server may echo the auth back.
            self._last_connect_error = str(redact(data))[:200]
            log_event(
                logger,
                "backend.connect_error",
                self._last_connect_error,
                level=logging.WARNING,
            )

        @sio.on(self.cfg.binding.command, namespace=ns)
        async def on_command(data: Any) -> None:
            await self._on_command(data)

        @sio.on(self.cfg.binding.registered, namespace=ns)
        async def on_registered(data: Any) -> None:
            self._registered_ack = data if isinstance(data, dict) else {"raw": str(data)[:200]}
            self._note_recv()
            log_event(logger, "backend.registered", payload=redact(self._registered_ack))

        @sio.on("*", namespace=ns)
        async def on_unexpected(event: str, *args: Any) -> None:
            # An event this Pi has no binding for. Counted and logged rather
            # than guessed at -- an unrecognized event is evidence the
            # provisional binding is wrong, which is exactly what needs
            # surfacing.
            self.stats["unexpected_events"] += 1
            self._note_recv()
            log_event(
                logger,
                "backend.unexpected_event",
                "received an event this agent has no binding for",
                level=logging.WARNING,
                socket_event=str(event)[:60],
            )

    # --- outbound -------------------------------------------------------------

    async def _emit(self, event: str, payload: Dict[str, Any]) -> bool:
        """Emit one payload. Returns whether it left the process.

        A failure here is the link's own business: it is counted and reported,
        never raised at the agent, which has a robot to keep running.
        """

        sio = self._sio
        if sio is None or not self.connected:
            return False
        try:
            await sio.emit(event, payload, namespace=self.cfg.binding.namespace)
        except Exception as e:  # noqa: BLE001
            self.stats["emit_failures"] += 1
            log_event(
                logger,
                "backend.emit_failed",
                _safe_error(e),
                level=logging.WARNING,
                socket_event=event,
            )
            return False
        self.state.update_communication(backend_last_send_at=time.time())
        return True

    def _note_recv(self) -> None:
        self.state.update_communication(backend_last_recv_at=time.time())

    async def _send_register(self) -> None:
        payload = build_register_payload(
            robot_id=self.cfg.robot_id,
            binding=self.cfg.binding,
            agent_version=AGENT_VERSION,
            capabilities=agent_capabilities(),
        )
        await self._emit(self.cfg.binding.register, payload)
        # Send status immediately so the backend has a mode to show before the
        # first telemetry frame -- which may never arrive if there is no fix.
        await self._publish_status(force=True)

    async def _publish_loop(self) -> None:
        while self._running and self.connected:
            now = time.monotonic()
            if now - self._last_telemetry_t >= self.cfg.telemetry_interval_s:
                await self._publish_telemetry()
                self._last_telemetry_t = now
            if now - self._last_status_t >= self.cfg.status_interval_s:
                await self._publish_status()
                self._last_status_t = now
            await asyncio.sleep(_PUBLISH_TICK_S)

    async def _publish_telemetry(self) -> None:
        frame = build_telemetry_payload(
            self.state.snapshot(),
            robot_id=self.cfg.robot_id,
            max_position_age_s=self.cfg.max_position_age_s,
        )
        if not frame.sendable:
            # Not an error, and not a reason to send something else instead.
            self.stats["telemetry_skipped"] += 1
            logger.debug("Telemetry skipped: %s", frame.skipped_reason)
            return
        if await self._emit(self.cfg.binding.telemetry, frame.payload or {}):
            self.stats["telemetry_sent"] += 1

    async def _publish_status(self, *, force: bool = False) -> None:
        payload = build_status_payload(
            self.state.snapshot(),
            robot_id=self.cfg.robot_id,
            binding=self.cfg.binding,
        )
        if await self._emit(self.cfg.binding.status, payload):
            self.stats["status_sent"] += 1
            if force:
                self._last_status_t = time.monotonic()

    async def emit_event(self, level: EventLevel, message: str, *, task_id: Optional[str] = None) -> bool:
        """Publish an `Event` row: something an operator should see.

        Rate limited and deduplicated. Events are meant to be sparse and
        meaningful; a subsystem flapping once a tick must not be able to turn
        this channel into a second telemetry stream.
        """

        now = time.monotonic()
        last_same = self._recent_events.get(message)
        if last_same is not None and now - last_same < _EVENT_DEDUPE_S:
            self.stats["events_suppressed"] += 1
            return False
        if now - self._last_event_t < _EVENT_MIN_INTERVAL_S:
            self.stats["events_suppressed"] += 1
            return False

        payload = build_event_payload(
            robot_id=self.cfg.robot_id, level=level, message=message, task_id=task_id
        )
        sent = await self._emit(self.cfg.binding.event, payload)
        if sent:
            self.stats["events_sent"] += 1
            self._last_event_t = now
            self._recent_events[message] = now
            # Bound the dedupe table.
            if len(self._recent_events) > 64:
                for key, seen in sorted(self._recent_events.items(), key=lambda kv: kv[1])[:32]:
                    self._recent_events.pop(key, None)
        return sent

    # --- inbound --------------------------------------------------------------

    async def _on_command(self, data: Any) -> None:
        """Validate, apply, then acknowledge. In that order, always."""

        self.stats["commands_received"] += 1
        self._note_recv()

        parsed = parse_command(
            data,
            expected_robot_id=self.cfg.robot_id,
            max_age_s=self.cfg.command_max_age_s,
        )

        if isinstance(parsed, CommandRejection):
            self.stats["commands_rejected"] += 1
            log_event(
                logger,
                "command.rejected",
                parsed.detail,
                level=logging.WARNING,
                reason=parsed.reason.value,
                command_id=parsed.command_id,
            )
            outcome = rejection_outcome(parsed)
            if outcome is not None:
                await self._send_command_result(outcome)
            return

        command: InboundCommand = parsed
        # Applied first; acknowledged second. An ACK emitted before the intent
        # was applied would be a claim the Pi cannot back up.
        outcome = self.executor.execute(command)
        if outcome.duplicate:
            self.stats["commands_duplicate"] += 1
        await self._send_command_result(outcome)

    async def _send_command_result(self, outcome: Any) -> None:
        payload = build_command_result_payload(
            robot_id=self.cfg.robot_id,
            command_id=outcome.command_id,
            status=outcome.status,
            reason=outcome.reason,
            executed_at=outcome.executed_at,
        )
        if await self._emit(self.cfg.binding.command_result, payload):
            self.stats["acks_sent"] += 1

    # --- link-loss policy -----------------------------------------------------

    def _apply_loss_policy(self) -> None:
        """Suspend an active mission once the link has been down long enough.

        Only ever reduces motion. There is no path here that resumes, speeds up
        or starts anything, so a backend that disappears can never cause the
        robot to do more than it was already doing.
        """

        if self.cfg.loss_policy is not BackendLossPolicy.PAUSE:
            return
        if self._loss_action_taken or self._disconnected_since is None:
            return
        if time.monotonic() - self._disconnected_since < self.cfg.loss_grace_s:
            return
        if self._target.mode is not OperatingMode.AUTO:
            return

        self._loss_action_taken = True
        log_event(
            logger,
            "backend.loss_pause",
            "pausing the mission: backend unreachable beyond the grace period",
            level=logging.WARNING,
            grace_s=self.cfg.loss_grace_s,
        )
        try:
            self._target.pause_mission("backend link lost")
        except Exception:
            logger.exception("Failed to pause mission after backend loss")

    # --- introspection --------------------------------------------------------

    def describe(self) -> Dict[str, Any]:
        """Current link state, for the local HTTP API and the test harness.

        `integrated` is the honest bottom line and is false while the binding
        is provisional, however healthy the socket looks. A connected socket
        speaking guessed event names is a connection, not an integration.
        """

        return {
            "enabled": self.cfg.enabled,
            "status": self._status.value,
            "detail": self._detail,
            "server": _safe_url(self.cfg.server_url),
            "tls": self.cfg.uses_tls,
            "authenticated_client_side": bool(self.cfg.robot_token),
            "protocol": self.cfg.binding.to_dict(),
            "integrated": self.connected and not self.cfg.binding.is_provisional,
            "registered_ack": self._registered_ack,
            "last_connect_error": self._last_connect_error,
            "loss_policy": self.cfg.loss_policy.value,
            "rates": {
                "telemetry_interval_s": self.cfg.telemetry_interval_s,
                "status_interval_s": self.cfg.status_interval_s,
            },
            "stats": dict(self.stats),
        }


# --- helpers ------------------------------------------------------------------


def _safe_url(url: str) -> str:
    """A URL with any embedded credential stripped, safe to log."""

    parsed = urlparse(url)
    netloc = parsed.hostname or ""
    if parsed.port:
        netloc = f"{netloc}:{parsed.port}"
    if parsed.username:
        netloc = f"<redacted>@{netloc}"
    return f"{parsed.scheme}://{netloc}{parsed.path}"


def _safe_error(error: BaseException) -> str:
    """An exception rendered for a log line, with credentials scrubbed."""

    return str(redact(str(error)))[:200] or error.__class__.__name__


# The decisive marker, measured against python-socketio 5.11.4 / engineio
# 4.14.0: when a server's connect handler raises `ConnectionRefusedError`, the
# client raises `ConnectionError("One or more namespaces failed to connect")`.
# The transport reached the server and the server said no -- which is exactly
# what a rejection is. The server's *reason* does not reach the client at all
# (the namespace `connect_error` handler is never invoked in this case), so
# the Pi can know that it was refused but not why. That gap is documented in
# docs/communication/ROBOT_BACKEND_PROTOCOL.md.
_NAMESPACE_REJECTION = "namespaces failed to connect"

# Secondary markers, for a server or library version that does surface a
# reason. Deliberately specific to *authorization*: bare "refused" is not here,
# because "Connection refused" is ECONNREFUSED -- the most common way a connect
# fails when the backend is simply down. Classifying that as a credential
# problem would put the Pi on the long rejection backoff every time the server
# restarted, delaying its return by up to a minute.
_REJECTION_MARKERS = (
    "unauthor",       # Unauthorized / unauthorised
    "forbidden",
    "invalid token",
    "invalid credential",
    "bad credential",
    "authentication failed",
    "unknown robot",
    "401",
    "403",
)

# A transport-level failure looks like this and is never a rejection.
_NETWORK_MARKERS = ("cannot connect to host", "timeout", "timed out")


def _looks_like_rejection(error: BaseException) -> bool:
    """Whether a connect failure was the server refusing us, not a dead network.

    Worth distinguishing: a refused credential will be refused again one second
    later, so retrying at network speed only generates load on a server that
    has already said no, while a network failure should be retried promptly so
    the Pi reconnects as soon as the backend returns.

    An unrecognized message is treated as a network failure and retried on the
    normal backoff. That is the safe direction to be wrong in: the cost is some
    extra connect attempts, whereas the opposite error would leave a robot
    slow to reconnect to a backend that is perfectly healthy.
    """

    text = str(error).lower()
    if any(marker in text for marker in _NETWORK_MARKERS):
        return False
    if _NAMESPACE_REJECTION in text:
        return True
    return any(marker in text for marker in _REJECTION_MARKERS)
