"""The Pi's single link to the FalconAut backend.

One object owns the whole boundary: the socket, its lifecycle, the outbound
rates and the inbound dispatch. There is exactly one Socket.IO client in this
repository and it is here.

The FalconAut lifecycle
-----------------------
::

    DISCONNECTED -> CONNECTING -> CONNECTED      (anonymous; no handshake auth)
                                      |
                                    AUTH
                                      v
                               AUTHENTICATING
                                      |
                                AUTH_SUCCESS
                                      v
                               AUTHENTICATED -> STREAMING

The socket connects **anonymously**: FalconAut reads no `handshake.auth`, no
query credential and no custom auth header. The robot authenticates by emitting
`AUTH` once the socket is open, and is authenticated only when `AUTH_SUCCESS`
comes back carrying a session token.

Why CONNECTED and AUTHENTICATED are different states
----------------------------------------------------
Because an authentication failure arrives as a **silent server-side
`disconnect(true)`** -- no error event, no reason. From the transport's point
of view that is identical to the backend restarting or the Wi-Fi dropping. The
only thing that distinguishes them is what the link was waiting for when the
socket closed, so the link records that explicitly rather than trying to
classify an exception string after the fact.

That distinction matters operationally: a refused credential will be refused
again in one second, so it backs off hard and clears a stale token, while a
backend that is merely down should be retried promptly so the robot comes back
as soon as it can.

Reconnection is ours, not the library's
---------------------------------------
`python-socketio` can reconnect by itself, but its retries are invisible to the
rest of the agent -- state would sit at CONNECTED while the library quietly
failed in a loop. Since a dashboard's "online" indicator is downstream of that
state, the link drives its own connect loop with `reconnection=False`, so every
attempt, failure and backoff is observable and reportable.

One socket, one set of listeners
--------------------------------
The client is built **once** and its handlers are registered **once**, then
reused across every reconnect. `socket.io-client` reuses its emitter across
reconnects, so registering handlers inside a connect callback would add a
second copy on the first reconnect and a third on the next -- and a duplicated
`COMMAND` handler executes the operator's command twice. `handler_registrations`
is exported in `describe()` so a test can assert it never grows.

Behaviour when the backend is unavailable
-----------------------------------------
The Pi keeps running. Perception, localization, navigation and the decision
layer are unaffected: the backend is a supervisor, not a controller, and losing
it must never be able to *start* motion. What losing it can do is stop motion,
after `loss_grace_s`, under the `pause` policy.
"""

from __future__ import annotations

import asyncio
import logging
import random
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple
from urllib.parse import urlparse

import socketio  # type: ignore

from robotx.communication.commands import CommandExecutor, CommandTarget
from robotx.communication.commitment_store import CommitmentStore
from robotx.communication.engine import (
    OFFER,
    TERMINATING_COMMANDS,
    Envelope,
    EnvelopeRejection,
    EnvelopeRejectReason,
    Offer,
    OfferDecision,
    OfferVerdict,
    build_custody_event,
    build_engine_ack,
    build_offer_response,
    check_fence,
    check_sequence,
    parse_envelope,
    parse_offer,
    verify_signature,
)
from robotx.communication.protocol import (
    AuthMethod,
    CommandRejection,
    CommandStatus,
    EventLevel,
    InboundCommand,
    ProtocolBinding,
    agent_capabilities,
    build_auth_payload,
    build_command_ack_payload,
    build_event_payload,
    build_heartbeat_payload,
    build_register_payload,
    build_status_payload,
    build_task_complete_payload,
    build_telemetry_payload,
    now_ms,
    parse_auth_success,
    parse_command,
    parse_stop_event,
    parse_task_assign,
    redact,
)
from robotx.mission.evidence import TrackFix, assess_completion
from robotx.mission.mission import ActiveMission, MissionRejected, MissionStatus
from robotx.communication.token_store import TokenStore
from robotx.config.logging_setup import log_event
from robotx.state.robot_state import BackendLinkStatus, OperatingMode, RobotState


logger = logging.getLogger(__name__)

AGENT_VERSION = "2.2"

# How often the publish loop wakes. Each channel decides from its own elapsed
# time whether it is due, so this only bounds scheduling jitter.
_PUBLISH_TICK_S = 0.25

# Identical event messages inside this window are emitted once. Stops a
# flapping subsystem from writing thousands of Event rows.
_EVENT_DEDUPE_S = 30.0
# Hard ceiling on event emissions, whatever their content.
_EVENT_MIN_INTERVAL_S = 1.0
# Engine envelopes held for a sequence gap, across all commitments.
_MAX_HELD_ENVELOPES = 16


class BackendLossPolicy(str, Enum):
    """What an active mission does when the backend link is lost."""

    PAUSE = "pause"        # suspend the mission after the grace period
    CONTINUE = "continue"  # keep driving; the link is not a safety interlock


@dataclass(frozen=True)
class BackendConfig:
    """Everything the link needs, resolved once at startup."""

    enabled: bool = False
    # No default host. The backend runs on a laptop today and on a deployed
    # server later; neither is this Pi, so any built-in value would be wrong.
    # Must be set via ROBOTX_SOCKET_SERVER_URL; see `validate_backend_url`.
    server_url: str = ""
    # Must equal the commissioned `Robot.robotId` (== `Agent.agentId`),
    # exactly and case-sensitively. No default: the backend has none either,
    # and an unknown id is a silent disconnect.
    robot_id: str = ""
    binding: ProtocolBinding = ProtocolBinding()

    # --- credentials ---------------------------------------------------------
    # The one-time 6-digit code from POST /api/robots/commission. Used only
    # when no session token is stored; it has a 300 s TTL.
    pairing_code: Optional[str] = None
    # An operator-supplied session token, overriding whatever is on disk.
    # Normally unset: the token comes from AUTH_SUCCESS and is persisted.
    robot_token: Optional[str] = None
    token_path: Optional[str] = None

    # How long to wait for AUTH_SUCCESS before giving up on this attempt.
    auth_timeout_s: float = 10.0

    # The backend's COMMAND_SIGNING_KEY, for verifying engine envelopes. No
    # backend mechanism provisions it; unset means no OFFER can be admitted.
    command_signing_key: Optional[str] = None
    # Where commitment high-water marks persist across restarts. None keeps
    # them in memory only (tests).
    commitment_path: Optional[str] = None

    # Outbound rates. Telemetry is the only high-frequency channel and it is
    # still an order of magnitude below the camera's 20 FPS.
    telemetry_interval_s: float = 1.0
    # Liveness cadence. Sent whenever the link is authenticated, whether or not
    # the robot has a position to report -- see `build_heartbeat_payload`.
    heartbeat_interval_s: float = 2.0
    status_interval_s: float = 5.0
    # A position older than this is not sent as telemetry at all.
    max_position_age_s: float = 5.0

    # Inbound guards.
    command_max_age_s: Optional[float] = 120.0

    # Connect backoff.
    backoff_initial_s: float = 1.0
    backoff_max_s: float = 60.0
    # Auth failures do not resolve themselves by retrying quickly.
    backoff_auth_failed_s: float = 60.0

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
        if settings.socket_enabled:
            # Fail at configuration time, where the agent already turns a bad
            # backend config into a loud, non-fatal "link not started".
            validate_backend_url(settings.socket_server_url)
            validate_robot_id(settings.robot_id)
        return cls(
            enabled=settings.socket_enabled,
            server_url=settings.socket_server_url,
            robot_id=settings.robot_id,
            binding=binding,
            pairing_code=getattr(settings, "pairing_code", None),
            robot_token=settings.robot_token,
            token_path=getattr(settings, "backend_token_path", None),
            auth_timeout_s=getattr(settings, "backend_auth_timeout_s", 10.0),
            command_signing_key=getattr(settings, "command_signing_key", None),
            commitment_path=getattr(settings, "commitment_state_path", None),
            telemetry_interval_s=settings.backend_telemetry_interval_s,
            heartbeat_interval_s=getattr(settings, "backend_heartbeat_interval_s", 2.0),
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


def validate_backend_url(url: str) -> str:
    """Check a backend URL is something the link can actually connect to.

    Raises `ValueError` naming what is wrong. Accepts `http`/`https` (and the
    `ws`/`wss` spellings), so the same setting serves a laptop on the LAN
    (`http://<LAPTOP-LAN-IP>:<PORT>`) and a deployed backend
    (`https://<host>`) with no code change.
    """

    text = (url or "").strip()
    if not text:
        raise ValueError(
            "ROBOTX_SOCKET_SERVER_URL is not set; point it at the backend, e.g. "
            "http://<LAPTOP-LAN-IP>:<PORT> or https://<deployed-host>"
        )
    parsed = urlparse(text)
    if parsed.scheme not in ("http", "https", "ws", "wss"):
        raise ValueError(
            f"ROBOTX_SOCKET_SERVER_URL scheme {parsed.scheme!r} is not http/https/ws/wss"
        )
    if not parsed.hostname:
        raise ValueError("ROBOTX_SOCKET_SERVER_URL has no host")
    return text


def validate_robot_id(robot_id: str) -> str:
    """The configured identity, or ValueError. Never defaulted, never trimmed.

    It must equal the commissioned `Robot.robotId` exactly -- including case
    and any whitespace -- so it is checked for presence only, not "fixed".
    """

    if not isinstance(robot_id, str) or not robot_id.strip():
        raise ValueError(
            "ROBOTX_ROBOT_ID is not set; it must equal the commissioned Robot.robotId exactly"
        )
    return robot_id


ClientFactory = Callable[["BackendConfig"], Any]
AgentAlive = Callable[[], bool]


def _default_client_factory(cfg: BackendConfig) -> Any:
    # reconnection=False: see the module docstring. `logger`/`engineio_logger`
    # stay off because the library logs full payloads, which would put the
    # session token into the Pi's log files.
    return socketio.AsyncClient(
        reconnection=False,
        logger=False,
        engineio_logger=False,
        ssl_verify=cfg.tls_verify,
    )


# A native robot client identifies itself as one. FalconAut keys some behaviour
# off a client-type heuristic over the request headers, so the Pi must not look
# like a browser: no `Origin`, and nothing Mozilla-shaped in the User-Agent.
NATIVE_CLIENT_HEADERS = {"User-Agent": f"robotx-pi/{AGENT_VERSION}"}


class BackendLink:
    """Connects, authenticates, publishes, receives commands, reports its truth."""

    def __init__(
        self,
        cfg: BackendConfig,
        state: RobotState,
        target: CommandTarget,
        *,
        client_factory: ClientFactory = _default_client_factory,
        token_store: Optional[TokenStore] = None,
        agent_alive: Optional[AgentAlive] = None,
        commitment_store: Optional[CommitmentStore] = None,
    ) -> None:
        self.cfg = cfg
        # What HEARTBEAT vouches for. None means "no agent loop to vouch for"
        # (a link driven directly, as in tests); the RobotAgent always passes
        # its own `is_alive`.
        self._agent_alive = agent_alive
        self._heartbeat_suppressed = False
        self.state = state
        self.executor = CommandExecutor(target)
        self._target = target
        self._client_factory = client_factory
        self.tokens = token_store if token_store is not None else TokenStore(cfg.token_path)
        self.commitments = (
            commitment_store if commitment_store is not None else CommitmentStore(cfg.commitment_path)
        )
        self._signing_key: Optional[bytes] = (
            cfg.command_signing_key.encode("utf-8") if cfg.command_signing_key else None
        )
        # Envelopes that arrived ahead of a sequence gap, keyed (commitment, seq).
        self._held_envelopes: Dict[Tuple[str, int], Dict[str, Any]] = {}
        # Measured fixes actually sent since each accepted commitment was
        # granted: the backend's completion evidence, mirrored. In memory
        # only -- a restart loses the mission, and with it any claim.
        self._tracks: Dict[str, List[TrackFix]] = {}
        self._granted_at_ms: Dict[str, int] = {}
        # TELEMETRY `sequence` must strictly increase per robot, including
        # across restarts. Seeded from the wall clock in ms: with NTP (which
        # the backend requires anyway) a restart always starts above the last
        # value sent, and 1 Hz never catches up with the clock.
        self._telemetry_sequence = now_ms()
        # Instant of the last fix sent, so one fix is never sent twice.
        self._last_position_sent: Optional[float] = None

        self._sio: Optional[Any] = None
        self._task: Optional[asyncio.Task] = None
        self._running = False

        self._connected = asyncio.Event()
        self._authenticated = asyncio.Event()
        self._auth_failed = asyncio.Event()

        self._status = BackendLinkStatus.DISABLED
        self._detail = ""
        self._attempt = 0
        self._last_connect_error: str = ""
        self._auth_method: Optional[AuthMethod] = None
        self._auth_failure_detail: str = ""
        # Incremented once per set of handler registrations. Must stay at 1 for
        # the life of the process; asserted by test.
        self.handler_registrations = 0

        self._last_telemetry_t = 0.0
        self._last_heartbeat_t = 0.0
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
            "auth_attempts": 0,
            "auth_successes": 0,
            "auth_failures": 0,
            "heartbeats_sent": 0,
            "heartbeats_suppressed": 0,
            "telemetry_sent": 0,
            "positions_sent": 0,
            "positions_omitted": 0,
            "status_sent": 0,
            "events_sent": 0,
            "events_suppressed": 0,
            "commands_received": 0,
            "commands_rejected": 0,
            "commands_refused": 0,
            "commands_duplicate": 0,
            "engine_commands_received": 0,
            "engine_commands_admitted": 0,
            "engine_commands_not_admitted": 0,
            "engine_redeliveries": 0,
            "offers_received": 0,
            "offers_accepted": 0,
            "offers_rejected": 0,
            "offers_deferred": 0,
            "custody_events_sent": 0,
            "tasks_received": 0,
            "tasks_rejected": 0,
            "tasks_recovery_resends": 0,
            "task_completes_sent": 0,
            "task_completes_withheld": 0,
            "task_completes_acked": 0,
            "task_completes_verifying": 0,
            "stop_events_ignored": 0,
            "stop_events_received": 0,
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
            self._set_status(BackendLinkStatus.DISABLED, "backend link disabled by configuration")
            return
        try:
            validate_backend_url(self.cfg.server_url)
            validate_robot_id(self.cfg.robot_id)
        except ValueError as e:
            self._set_status(BackendLinkStatus.DISABLED, f"configuration error: {e}")
            log_event(logger, "backend.config_invalid", str(e), level=logging.ERROR)
            return

        self._warn_about_configuration()
        self._running = True
        self._set_status(BackendLinkStatus.CONNECTING, "starting")
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
        self._authenticated.clear()
        self._set_status(
            BackendLinkStatus.DISABLED if not self.cfg.enabled else BackendLinkStatus.DISCONNECTED,
            "link stopped",
        )

    def _warn_about_configuration(self) -> None:
        """Say out loud, once, what this link does and does not guarantee."""

        unconfirmed = self.cfg.binding.unconfirmed
        if unconfirmed:
            log_event(
                logger,
                "backend.protocol_unconfirmed",
                "these binding names follow the FalconAut convention but were not "
                "quoted literally in the contract; override them with "
                "ROBOTX_PROTOCOL_FILE if the backend disagrees",
                level=logging.WARNING,
                unconfirmed=",".join(unconfirmed),
            )
        token, pairing_code = self._available_credential()
        if not token and not pairing_code:
            log_event(
                logger,
                "backend.no_credential",
                "no session token on disk and no ROBOTX_PAIRING_CODE set. Run "
                "`python -m robotx.communication.commissioning` to get a pairing "
                "code; AUTH cannot succeed without one.",
                level=logging.WARNING,
            )
        if not self.cfg.uses_tls:
            log_event(
                logger,
                "backend.no_tls",
                "backend URL is not https/wss: the session token and all telemetry "
                "travel in clear text",
                level=logging.WARNING,
                scheme=urlparse(self.cfg.server_url).scheme,
            )

    # --- credentials ----------------------------------------------------------

    def _available_credential(self) -> tuple:
        """The credential AUTH will present, as `(token, pairing_code)`.

        An explicitly configured token wins over the stored one, and a stored
        token wins over the pairing code -- the code is single-use with a 300 s
        TTL, so it is spent only when there is nothing else.
        """

        if self.cfg.robot_token:
            return (self.cfg.robot_token, None)
        stored = self.tokens.load(robot_id=self.cfg.robot_id)
        if stored is not None:
            return (stored.token, None)
        return (None, self.cfg.pairing_code)

    # --- connection state -----------------------------------------------------

    @property
    def status(self) -> BackendLinkStatus:
        return self._status

    @property
    def connected(self) -> bool:
        """Whether the link can carry robot traffic, i.e. is authenticated."""

        return self._status.is_up

    @property
    def socket_open(self) -> bool:
        return self._status.socket_open

    def _set_status(self, status: BackendLinkStatus, detail: str = "") -> None:
        """Record link state in exactly one place: the authoritative RobotState."""

        changed = status is not self._status
        self._status = status
        self._detail = detail
        self.state.update_communication(
            backend=status,
            backend_detail=detail,
            backend_since=time.time(),
        )
        if changed:
            log_event(
                logger,
                "backend.link_status",
                detail,
                level=logging.INFO if status.is_up else logging.WARNING,
                status=status.value,
            )

    async def wait_connected(self, timeout_s: float = 10.0) -> bool:
        """Wait until the link is authenticated and usable."""

        try:
            await asyncio.wait_for(self._authenticated.wait(), timeout=timeout_s)
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

    # --- the connect/authenticate/publish loop --------------------------------

    async def _run(self) -> None:
        while self._running:
            # 1. Connect the transport.
            try:
                await self._connect_once()
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001 - the link must never die
                self.stats["connect_failures"] += 1
                self._last_connect_error = repr(e)[:200]
                # A transport failure is never an auth failure. The backend
                # being down is the single most common reason a connect fails,
                # and treating it as a refused credential would put the robot
                # on the long backoff every time the server restarted.
                self._set_status(
                    BackendLinkStatus.DISCONNECTED, f"connect failed: {_safe_error(e)}"
                )
                await self._sleep_while_running(self.next_backoff_s(self._attempt))
                self._attempt += 1
                continue

            # 2. Authenticate. FalconAut refuses by disconnecting silently, so
            #    this waits on an outcome rather than on an exception.
            authenticated = await self._await_authentication()
            if not authenticated:
                self.stats["auth_failures"] += 1
                self._set_status(
                    BackendLinkStatus.AUTH_FAILED,
                    self._auth_failure_detail or "authentication did not complete",
                )
                await self._handle_auth_failure()
                await self._sleep_while_running(self.cfg.backoff_auth_failed_s)
                self._attempt += 1
                continue

            # 3. Stream until the link drops.
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
        self._set_status(BackendLinkStatus.CONNECTING, f"attempt {self._attempt + 1}")

        self._connected.clear()
        self._authenticated.clear()
        self._auth_failed.clear()
        self._auth_failure_detail = ""

        sio = self._ensure_client()

        # No `auth=` and no credential in the query string: FalconAut's socket
        # connection is anonymous and the server reads neither. The credential
        # goes in the AUTH event, once the socket is open.
        await sio.connect(
            self.cfg.server_url,
            namespaces=[self.cfg.binding.namespace],
            headers=dict(NATIVE_CLIENT_HEADERS),
        )

    def _ensure_client(self) -> Any:
        """The one client, built and wired exactly once.

        Reused across reconnects. Building a fresh client per attempt would
        also work, but reusing one is what makes the "listeners registered
        once" property checkable: `handler_registrations` must be 1 forever.
        """

        if self._sio is None:
            self._sio = self._client_factory(self.cfg)
            self._register_handlers(self._sio)
            self.handler_registrations += 1
        return self._sio

    def _register_handlers(self, sio: Any) -> None:
        """Register every listener. Called once per client instance, ever."""

        ns = self.cfg.binding.namespace

        @sio.event(namespace=ns)
        async def connect() -> None:  # noqa: D401
            self.stats["connects"] += 1
            self._connected.set()
            self._disconnected_since = None
            self._loss_action_taken = False
            self._set_status(
                BackendLinkStatus.CONNECTED, f"connected to {_safe_url(self.cfg.server_url)}"
            )
            await self._send_auth()

        @sio.event(namespace=ns)
        async def disconnect() -> None:  # noqa: D401
            self.stats["disconnects"] += 1
            was = self._status
            self._connected.clear()
            self._authenticated.clear()
            self._disconnected_since = time.monotonic()

            # A disconnect while we were waiting for AUTH_SUCCESS *is* the
            # authentication failure: FalconAut refuses by calling
            # disconnect(true) with no error event. Nothing else distinguishes
            # it from the backend going away.
            if was in (BackendLinkStatus.CONNECTED, BackendLinkStatus.AUTHENTICATING):
                self._auth_failure_detail = (
                    "backend closed the connection while authenticating: the "
                    "credential was refused"
                )
                self._auth_failed.set()
                return

            self._set_status(BackendLinkStatus.DISCONNECTED, "server closed the connection")

        @sio.event(namespace=ns)
        async def connect_error(data: Any) -> None:  # noqa: D401
            self._last_connect_error = str(redact(data))[:200]
            log_event(
                logger,
                "backend.connect_error",
                self._last_connect_error,
                level=logging.WARNING,
            )

        # Both success event names, because the contract names two and binding
        # only one means a silent, total failure to authenticate if the backend
        # picks the other.
        for success_event in self.cfg.binding.auth_success_events():
            @sio.on(success_event, namespace=ns)
            async def on_auth_success(data: Any = None) -> None:
                await self._on_auth_success(data)

        if self.cfg.binding.auth_failed:
            @sio.on(self.cfg.binding.auth_failed, namespace=ns)
            async def on_auth_failed(data: Any = None) -> None:
                self._note_recv()
                self._auth_failure_detail = f"backend refused AUTH: {str(redact(data))[:160]}"
                self._auth_failed.set()

        @sio.on(self.cfg.binding.command, namespace=ns)
        async def on_command(data: Any = None) -> None:
            await self._on_command(data)

        if self.cfg.binding.stop:
            @sio.on(self.cfg.binding.stop, namespace=ns)
            async def on_stop(data: Any = None) -> None:
                await self._on_stop_event(data)

        if self.cfg.binding.task_assign:
            @sio.on(self.cfg.binding.task_assign, namespace=ns)
            async def on_task_assign(data: Any = None) -> None:
                await self._on_task_assign(data)

        if self.cfg.binding.engine_command:
            @sio.on(self.cfg.binding.engine_command, namespace=ns)
            async def on_engine_command(data: Any = None) -> None:
                await self._on_engine_command(data)

        if self.cfg.binding.task_complete_ack:
            @sio.on(self.cfg.binding.task_complete_ack, namespace=ns)
            async def on_task_complete_ack(data: Any = None) -> None:
                await self._on_task_complete_ack(data)

        @sio.on("*", namespace=ns)
        async def on_unexpected(event: str, *args: Any) -> None:
            # An event this Pi has no binding for. Counted and logged rather
            # than guessed at.
            self.stats["unexpected_events"] += 1
            self._note_recv()
            log_event(
                logger,
                "backend.unexpected_event",
                "received an event this agent has no binding for",
                level=logging.WARNING,
                socket_event=str(event)[:60],
            )

    # --- authentication -------------------------------------------------------

    async def _send_auth(self) -> None:
        """Emit AUTH with whichever credential is available."""

        token, pairing_code = self._available_credential()
        try:
            payload, method = build_auth_payload(
                robot_id=self.cfg.robot_id, token=token, pairing_code=pairing_code
            )
        except ValueError as e:
            self._auth_failure_detail = str(e)
            self._auth_failed.set()
            return

        self._auth_method = method
        self.stats["auth_attempts"] += 1
        self._set_status(
            BackendLinkStatus.AUTHENTICATING, f"authenticating with {method.value.lower()}"
        )
        # `require_auth=False`: this is the event that establishes auth.
        await self._emit(self.cfg.binding.auth, payload, require_auth=False)

    async def _await_authentication(self) -> bool:
        """Wait for AUTH_SUCCESS, an explicit refusal, or a timeout."""

        success = asyncio.create_task(self._authenticated.wait())
        failure = asyncio.create_task(self._auth_failed.wait())
        try:
            done, pending = await asyncio.wait(
                {success, failure},
                timeout=self.cfg.auth_timeout_s,
                return_when=asyncio.FIRST_COMPLETED,
            )
        finally:
            for task in (success, failure):
                if not task.done():
                    task.cancel()

        if self._authenticated.is_set():
            return True
        if not done:
            self._auth_failure_detail = (
                f"no AUTH_SUCCESS within {self.cfg.auth_timeout_s:.0f}s"
            )
        return False

    async def _on_auth_success(self, data: Any) -> None:
        """AUTH_SUCCESS: persist the token and start streaming.

        The backend emits both AUTH_SUCCESS and AUTH_OK for one AUTH (handoff
        §3). The first one authenticates this connection; the second is
        noted and ignored, so the link is not knocked back out of STREAMING.
        """

        self._note_recv()
        if self._authenticated.is_set():
            return
        result = parse_auth_success(data)

        if result.has_token and result.token:
            # Persisted so the next reconnect does not need a human to issue a
            # fresh pairing code.
            self.tokens.save(robot_id=self.cfg.robot_id, token=result.token)
        else:
            log_event(
                logger,
                "backend.auth_no_token",
                "AUTH_SUCCESS carried no recognizable session token; this robot "
                "will need a pairing code again after a restart",
                level=logging.WARNING,
            )

        self.stats["auth_successes"] += 1
        self._authenticated.set()
        self._set_status(BackendLinkStatus.AUTHENTICATED, "authenticated by the backend")

    async def _handle_auth_failure(self) -> None:
        """React to a refused credential.

        A refused *token* is discarded, so the next attempt falls back to the
        pairing code if one is configured. A token the backend has revoked will
        never start working again, and keeping it would leave the robot
        retrying the same rejected credential until someone logged in.

        A refused *pairing code* is kept: it is far more likely to have expired
        (300 s TTL) than to be wrong, and the operator needs to see which code
        the robot is still trying.
        """

        log_event(
            logger,
            "backend.auth_failed",
            self._auth_failure_detail,
            level=logging.ERROR,
            auth_method=None if self._auth_method is None else self._auth_method.value,
        )
        if self._auth_method is AuthMethod.TOKEN and not self.cfg.robot_token:
            self.tokens.clear()
            log_event(
                logger,
                "backend.token_discarded",
                "discarded the stored session token after it was refused; the next "
                "attempt will use ROBOTX_PAIRING_CODE if one is set",
                level=logging.WARNING,
            )

        sio = self._sio
        if sio is not None:
            try:
                await sio.disconnect()
            except Exception:
                logger.debug("Ignoring error disconnecting after auth failure", exc_info=True)

    # --- outbound -------------------------------------------------------------

    async def _emit(self, event: str, payload: Dict[str, Any], *, require_auth: bool = True) -> bool:
        """Emit one payload. Returns whether it left the process.

        A failure here is the link's own business: it is counted and reported,
        never raised at the agent, which has a robot to keep running.

        Nothing but AUTH is emitted before authentication. The backend will not
        attribute an anonymous socket's telemetry to this robot, so sending it
        early would be writing into a void while looking like success.
        """

        if not event:
            # An unbound optional channel. Not an error; just not part of this
            # contract.
            return False

        sio = self._sio
        if sio is None:
            return False
        if require_auth and not self.connected:
            return False
        if not require_auth and not self.socket_open:
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

    async def _publish_loop(self) -> None:
        """Stream telemetry for as long as the link stays authenticated."""

        self._set_status(BackendLinkStatus.STREAMING, "streaming telemetry")
        # An optional, operator-bound registration channel. Empty by default:
        # FalconAut identifies the robot through AUTH, not a register event.
        if self.cfg.binding.register:
            await self._emit(
                self.cfg.binding.register,
                build_register_payload(
                    robot_id=self.cfg.robot_id,
                    binding=self.cfg.binding,
                    agent_version=AGENT_VERSION,
                    capabilities=agent_capabilities(),
                ),
            )
        if self.cfg.binding.status:
            await self._publish_status(force=True)

        # On every (re)connect: AUTH (done), then HEARTBEAT, then TELEMETRY
        # (handoff §15) -- immediately, so the backend does not wait a full
        # interval to learn the robot is back.
        await self._publish_heartbeat()
        self._last_heartbeat_t = time.monotonic()
        await self._publish_telemetry()
        self._last_telemetry_t = time.monotonic()

        while self._running and self.connected:
            now = time.monotonic()
            if now - self._last_heartbeat_t >= self.cfg.heartbeat_interval_s:
                await self._publish_heartbeat()
                self._last_heartbeat_t = now
            if now - self._last_telemetry_t >= self.cfg.telemetry_interval_s:
                await self._publish_telemetry()
                self._last_telemetry_t = now
            # Checked every tick rather than on an interval: a handover or a
            # finished delivery should not wait a telemetry period. Both are
            # no-ops unless mission state records something not yet reported.
            await self._publish_custody()
            await self._publish_task_complete()
            if self.cfg.binding.status and now - self._last_status_t >= self.cfg.status_interval_s:
                await self._publish_status()
                self._last_status_t = now
            await asyncio.sleep(_PUBLISH_TICK_S)

    async def _publish_heartbeat(self) -> None:
        """Say "alive", on a cadence, regardless of what the sensors know.

        Nothing about sensors gates it -- only authentication and a live agent
        loop. Telemetry legitimately goes silent when the robot has no
        trustworthy position -- indoors, that is always -- and without a
        heartbeat the backend cannot tell that silence apart from a dead robot.

        Gated on the agent loop being alive, when there is one. The heartbeat
        claims "this robot is alive", and an open socket driven by its own task
        is not evidence of that: it would keep beating through a hung agent.
        Suppressing it instead lets the backend's freshness budget expire,
        which is the truthful outcome.
        """

        if self._agent_alive is not None and not self._agent_alive():
            self.stats["heartbeats_suppressed"] += 1
            if not self._heartbeat_suppressed:
                self._heartbeat_suppressed = True
                log_event(
                    logger,
                    "backend.heartbeat_suppressed",
                    "agent loop has not ticked recently; not reporting this robot alive",
                    level=logging.WARNING,
                )
            return
        if self._heartbeat_suppressed:
            self._heartbeat_suppressed = False
            log_event(logger, "backend.heartbeat_resumed", "agent loop is ticking again")

        # The commitment form renews the mission lease, so it is sent only
        # while the Rover is genuinely carrying that mission out.
        held = self._held_mission()
        payload = (
            build_heartbeat_payload(commitment_id=held[0].commitment_id, fence=held[0].fence_wire)
            if held is not None
            else build_heartbeat_payload()
        )
        if await self._emit(self.cfg.binding.heartbeat, payload):
            self.stats["heartbeats_sent"] += 1

    async def _publish_telemetry(self) -> None:
        self._telemetry_sequence += 1
        frame = build_telemetry_payload(
            self.state.snapshot(),
            sequence=self._telemetry_sequence,
            max_position_age_s=self.cfg.max_position_age_s,
            last_position_timestamp=self._last_position_sent,
        )
        if not frame.has_position:
            # Not an error, and never a reason to put something else in lat/lon.
            self.stats["positions_omitted"] += 1
            logger.debug("Telemetry without position: %s", frame.position_omitted)
        if not await self._emit(self.cfg.binding.telemetry, frame.payload):
            return
        self.stats["telemetry_sent"] += 1
        if frame.has_position:
            self.stats["positions_sent"] += 1
            self._last_position_sent = frame.position_timestamp
            # Every measured fix the backend now holds is completion evidence
            # for the commitment in progress; mirror it.
            held = self.commitments.held()
            if held is not None and held.commitment_id in self._tracks:
                self._tracks[held.commitment_id].append(TrackFix(
                    t_ms=frame.payload["timestamp"],
                    lat=frame.payload["lat"],
                    lon=frame.payload["lon"],
                ))

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
        """Publish an operator-visible event, if the channel is bound.

        Rate limited and deduplicated. Returns False when `binding.event` is
        empty, which is the default -- the FalconAut robot contract does not
        declare this channel, and the Pi does not emit undeclared events.
        """

        if not self.cfg.binding.event:
            return False

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
            if len(self._recent_events) > 64:
                for key, seen in sorted(self._recent_events.items(), key=lambda kv: kv[1])[:32]:
                    self._recent_events.pop(key, None)
        return sent

    # --- inbound: operator commands -------------------------------------------

    async def _on_command(self, data: Any) -> None:
        """Operator COMMAND: validate, apply, then acknowledge -- if applied.

        The backend's COMMAND_ACK has no FAILED form (handoff §7, §13). A
        command this Rover rejects or refuses is therefore not acknowledged at
        all; the backend marks it FAILED after its 5/10/15 s redeliveries,
        which is exactly what happened. An unknown `type` is never acked.
        """

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
            return

        outcome = self._execute(parsed)
        if outcome.status is CommandStatus.ACK:
            if await self._emit(self.cfg.binding.command_ack,
                                build_command_ack_payload(command_id=outcome.command_id)):
                self.stats["acks_sent"] += 1
        else:
            self.stats["commands_refused"] += 1

    async def _on_stop_event(self, data: Any) -> None:
        """The bare STOP event: task cancellation. Obeyed; never acknowledged.

        Routed through the same idempotent executor as COMMAND, so a STOP that
        arrives on both channels for the same id still stops the robot once.
        """

        self.stats["stop_events_received"] += 1
        self._note_recv()
        command = parse_stop_event(data, expected_robot_id=self.cfg.robot_id)
        if command is None:
            # Addressed to another robot: neither obeyed nor acknowledged.
            self.stats["stop_events_ignored"] += 1
            log_event(
                logger,
                "command.wrong_robot",
                "ignored a STOP addressed to another robot",
                level=logging.WARNING,
            )
            return
        self._execute(command)

    def _execute(self, command: InboundCommand) -> Any:
        outcome = self.executor.execute(command)
        if outcome.duplicate:
            self.stats["commands_duplicate"] += 1
        return outcome

    # --- inbound: TASK_ASSIGN (recovery only) ---------------------------------

    async def _on_task_assign(self, data: Any) -> None:
        """TASK_ASSIGN never starts a mission on this Rover.

        Its only producer is the backend's post-restart recovery sweep, which
        re-sends the cached route of a task already ASSIGNED or IN_PROGRESS,
        with no commitment and no fence (handoff §6). OFFER is the one way a
        mission is assigned. So this is parsed, correlated with the commitment
        the Rover holds, logged -- and nothing else. No reply exists for it.
        """

        self.stats["tasks_received"] += 1
        self._note_recv()

        try:
            mission = parse_task_assign(data, expected_robot_id=self.cfg.robot_id)
        except MissionRejected as e:
            self.stats["tasks_rejected"] += 1
            log_event(logger, "task_assign.unreadable", e.detail, level=logging.WARNING,
                      reason=e.reason.value, task_id=e.task_id)
            return

        held = self.commitments.held()
        correlated = held is not None and held.task_id == mission.task_id
        self.stats["tasks_recovery_resends"] += 1
        log_event(
            logger,
            "task_assign.not_authoritative",
            "TASK_ASSIGN is a recovery re-send, not an assignment; no mission started",
            level=logging.INFO if correlated else logging.WARNING,
            task_id=mission.task_id,
            held_commitment=held.commitment_id if correlated else None,
        )

    # --- inbound: Assignment Engine envelopes ---------------------------------

    async def _on_engine_command(self, data: Any) -> None:
        """Admit one signed engine envelope, in the handoff's order, then act.

        Anything not admitted gets no COMMAND_ACK, no OFFER_* response and no
        effect; see `robotx.communication.engine`.
        """

        self.stats["engine_commands_received"] += 1
        self._note_recv()

        # Steps 1-2: addressee, structure, validity window.
        parsed = parse_envelope(data, expected_agent_id=self.cfg.robot_id)
        if isinstance(parsed, EnvelopeRejection):
            self._not_admitted(parsed)
            return
        envelope = parsed

        # Step 3: signature. Never skipped; no key means not admitted.
        rejection = verify_signature(envelope.raw, self._signing_key)
        if rejection is not None:
            self._not_admitted(rejection)
            return

        record = self.commitments.get(envelope.commitment_id)

        # An exact redelivery of an envelope already admitted: the ACK may have
        # been lost, and a repeated ACK is harmless (command.handler.js:52).
        # Nothing is applied and no OFFER_* response is sent again.
        if record is not None and envelope.outbox_id in record.acked_outbox_ids:
            self.stats["engine_redeliveries"] += 1
            await self._emit(self.cfg.binding.command_ack, build_engine_ack(envelope))
            return

        if record is not None and record.tombstoned:
            self._not_admitted(EnvelopeRejection(
                EnvelopeRejectReason.TOMBSTONED, "commitment was withdrawn", envelope.commitment_id))
            return

        # Steps 4-5: fence, then sequence.
        highest_fence = record.highest_fence if record else None
        highest_sequence = record.highest_sequence if record else None
        rejection = check_fence(envelope, highest_fence) or check_sequence(envelope, highest_sequence)
        if rejection is not None:
            if rejection.reason is EnvelopeRejectReason.OUT_OF_ORDER:
                self._hold(envelope)
            self._not_admitted(rejection)
            return

        offer = None
        if envelope.command == OFFER:
            offer = parse_offer(envelope)
            if isinstance(offer, EnvelopeRejection):
                self._not_admitted(offer)
                return

        # Step 6: persist the high-water marks (and that this outboxId is being
        # acknowledged) before acting on anything.
        record = self.commitments.update(
            envelope.commitment_id,
            highest_fence=envelope.fence,
            fence_wire=envelope.fence_wire,
            highest_sequence=envelope.sequence,
            task_id=offer.task_id if offer is not None else (record.task_id if record else None),
            acked_outbox_ids=(record.acked_outbox_ids if record else []) + [envelope.outbox_id],
        )
        if record is None:
            self._not_admitted(EnvelopeRejection(
                EnvelopeRejectReason.NOT_PERSISTED,
                "high-water marks could not be persisted; not acting",
                envelope.commitment_id,
            ))
            return

        self.stats["engine_commands_admitted"] += 1
        await self._apply_envelope(envelope, offer, record)

        # A successor that arrived early and was held can now be admitted.
        successor = self._held_envelopes.pop((envelope.commitment_id, envelope.sequence + 1), None)
        if successor is not None:
            await self._on_engine_command(successor)

    def _hold(self, envelope: Envelope) -> None:
        if len(self._held_envelopes) >= _MAX_HELD_ENVELOPES:
            return
        self._held_envelopes[(envelope.commitment_id, envelope.sequence)] = envelope.raw

    def _not_admitted(self, rejection: EnvelopeRejection) -> None:
        self.stats["engine_commands_not_admitted"] += 1
        log_event(
            logger,
            "engine.not_admitted",
            rejection.detail,
            level=logging.INFO if rejection.reason is EnvelopeRejectReason.DUPLICATE else logging.WARNING,
            reason=rejection.reason.value,
            commitment_id=rejection.commitment_id,
        )

    async def _apply_envelope(self, envelope: Envelope, offer: Optional[Offer], record: Any) -> None:
        ack = build_engine_ack(envelope)

        if envelope.command in TERMINATING_COMMANDS:
            # Stop that mission -- only if it is the one being driven -- then
            # tombstone the commitment, then acknowledge.
            mission = self.state.snapshot().mission
            if mission is not None and record.task_id and mission.task_id == record.task_id \
                    and mission.status.is_active:
                try:
                    self._target.stop_mission(f"engine {envelope.command} ({envelope.commitment_id})")
                except Exception:
                    logger.exception("Failed to stop the mission for %s", envelope.command)
            self.commitments.update(envelope.commitment_id, tombstoned=True)
            log_event(logger, "engine.commitment_ended", command=envelope.command,
                      commitment_id=envelope.commitment_id)
            await self._emit(self.cfg.binding.command_ack, ack)
            return

        await self._emit(self.cfg.binding.command_ack, ack)
        if offer is not None:
            await self._respond_to_offer(offer, record)
        # Other admitted commands have no producer today and no invented effect.

    async def _respond_to_offer(self, offer: Offer, record: Any) -> None:
        """Exactly one of OFFER_ACCEPT / OFFER_REJECT / OFFER_DEFER, ever.

        The Rover's own feasibility decides (`assess_offer`); nothing here
        accepts because an offer arrived. The response is persisted before it
        is sent, so neither a redelivery nor a restart can produce a second one
        -- a second ACCEPT would reset a Leg the Rover may already be driving.
        """

        self.stats["offers_received"] += 1
        if record.response is not None:
            log_event(logger, "offer.already_answered", commitment_id=offer.commitment_id,
                      response=record.response)
            return
        if offer.offer_expiry is not None and offer.offer_expiry <= time.time():
            log_event(logger, "offer.expired", "offerExpiry has passed; no response is possible",
                      level=logging.WARNING, commitment_id=offer.commitment_id)
            return

        try:
            decision = self._target.assess_offer(offer)
        except Exception:  # noqa: BLE001 - a bug here must decline, not accept
            logger.exception("assess_offer failed")
            decision = OfferDecision.reject("AGENT_ERROR")

        accepted = False
        if decision.verdict is OfferVerdict.ACCEPT:
            try:
                self._target.assign_mission(decision.mission, custody_required=True)
                accepted = True
            except MissionRejected as e:
                decision = OfferDecision.reject(f"MISSION_REFUSED: {e.reason.value}")
            except Exception:  # noqa: BLE001
                logger.exception("assign_mission failed")
                decision = OfferDecision.reject("AGENT_ERROR")

        try:
            payload = build_offer_response(offer, decision)
        except ValueError as e:
            # Only a DEFER can fail here, and only with an `until` the backend
            # would refuse. Declining is the honest fallback.
            log_event(logger, "offer.defer_invalid", str(e), level=logging.WARNING,
                      commitment_id=offer.commitment_id)
            decision = OfferDecision.reject(decision.reason or "DEFER_UNAVAILABLE")
            payload = build_offer_response(offer, decision)

        persisted = self.commitments.update(offer.commitment_id, response=decision.verdict.value)
        if persisted is None:
            # Could not record the answer, so it must not be given: a restart
            # would not know it had been, and could answer twice.
            if accepted:
                self._target.stop_mission("offer response could not be persisted")
            return

        event = {
            OfferVerdict.ACCEPT: self.cfg.binding.offer_accept,
            OfferVerdict.REJECT: self.cfg.binding.offer_reject,
            OfferVerdict.DEFER: self.cfg.binding.offer_defer,
        }[decision.verdict]
        if accepted:
            self._tracks[offer.commitment_id] = []
            self._granted_at_ms[offer.commitment_id] = now_ms()
        await self._emit(event, payload)
        self.stats[{
            OfferVerdict.ACCEPT: "offers_accepted",
            OfferVerdict.REJECT: "offers_rejected",
            OfferVerdict.DEFER: "offers_deferred",
        }[decision.verdict]] += 1
        log_event(
            logger,
            "offer.answered",
            level=logging.INFO if accepted else logging.WARNING,
            commitment_id=offer.commitment_id,
            task_id=offer.task_id,
            verdict=decision.verdict.value,
            reason=decision.reason,
        )

    async def _on_task_complete_ack(self, data: Any) -> None:
        """The backend's verdict on a TASK_COMPLETE. Never retried either way."""

        self._note_recv()
        reply = data if isinstance(data, dict) else {}
        if reply.get("verifying"):
            self.stats["task_completes_verifying"] += 1
            log_event(logger, "task.complete_verifying",
                      "backend found the evidence insufficient; the task is with an operator. "
                      "Not retrying the claim.", level=logging.WARNING,
                      task_id=str(reply.get("taskId"))[:60])
        else:
            self.stats["task_completes_acked"] += 1
            log_event(logger, "task.complete_acked", task_id=str(reply.get("taskId"))[:60])

    # --- outbound: mission events ---------------------------------------------

    def _held_mission(self) -> Optional[Tuple[Any, ActiveMission]]:
        """The held commitment and the mission genuinely carrying it out.

        None unless *both* exist and match. After a restart the commitment
        record survives but the mission does not, and a Rover that is not
        executing a mission must not renew its lease or report on it.
        """

        record = self.commitments.held()
        if record is None:
            return None
        mission = self.state.snapshot().mission
        if mission is None or not mission.custody_required or mission.task_id != record.task_id:
            return None
        if mission.status is MissionStatus.ABORTED:
            return None
        return record, mission

    async def _publish_custody(self) -> None:
        """CUSTODY_EVENT for each handover the mission state records, once each.

        Driven only by `custody_acquired_at` / `custody_released_at`, which
        only a genuine custody observation at a measured arrival can set
        (`MissionManager.record_custody`). Never by arrival, a timer or a
        command.
        """

        held = self._held_mission()
        if held is None:
            return
        record, mission = held
        for kind, at in (("ACQUIRED", mission.custody_acquired_at),
                         ("RELEASED", mission.custody_released_at)):
            if at is None or kind in record.custody_sent:
                continue
            payload = build_custody_event(
                commitment_id=record.commitment_id, fence=record.fence_wire, kind=kind
            )
            if await self._emit(self.cfg.binding.custody_event, payload):
                self.stats["custody_events_sent"] += 1
                record = self.commitments.update(
                    record.commitment_id, custody_sent=record.custody_sent + [kind]
                ) or record
                log_event(logger, "custody.reported", commitment_id=record.commitment_id, kind=kind)

    async def _publish_task_complete(self) -> None:
        """TASK_COMPLETE, once, and only on evidence the backend would accept.

        Requires, all of: the mission complete (custody RELEASED and reported),
        both arrivals decided on measured positions, and the L1 evidence check
        (`robotx.mission.evidence`) passing on the fixes this link actually
        sent since the commitment was granted. `lat`/`lon` are the last of
        those measured fixes. Anything short of that is withheld, not sent --
        never with a dead-reckoned or remembered position.
        """

        held = self._held_mission()
        if held is None:
            return
        record, mission = held
        if not mission.is_complete or "RELEASED" not in record.custody_sent:
            return

        cid = record.commitment_id
        track = self._tracks.get(cid, [])
        granted = self._granted_at_ms.get(cid)
        failures: Tuple[str, ...]
        if not mission.arrivals_measured:
            failures = ("an arrival was decided on an unmeasured position",)
        elif granted is None:
            failures = ("no position track since the commitment was granted",)
        else:
            verdict = assess_completion(
                track,
                granted_at_ms=granted,
                claim_at_ms=now_ms(),
                final_stop=mission.mission.drop,
                commanded_path=list(mission.mission.path_to_pickup) + list(mission.mission.path_to_drop),
            )
            failures = verdict.failures

        if failures:
            self.stats["task_completes_withheld"] += 1
            self.commitments.update(cid, completion="WITHHELD")
            log_event(
                logger,
                "task.complete_withheld",
                "not claiming completion: " + "; ".join(failures),
                level=logging.WARNING,
                task_id=record.task_id,
                commitment_id=cid,
            )
            return

        last = track[-1]
        payload = build_task_complete_payload(task_id=record.task_id, lat=last.lat, lon=last.lon)
        if await self._emit(self.cfg.binding.task_complete, payload):
            self.stats["task_completes_sent"] += 1
            self.commitments.update(cid, completion="SENT")
            log_event(logger, "task.complete_reported", task_id=record.task_id, commitment_id=cid)

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

        `streaming` is the honest bottom line: the socket is open, the backend
        has authenticated this robot, and telemetry is flowing. A connected but
        unauthenticated socket is not an integration.
        """

        return {
            "enabled": self.cfg.enabled,
            "status": self._status.value,
            "detail": self._detail,
            "server": _safe_url(self.cfg.server_url),
            "tls": self.cfg.uses_tls,
            "protocol": self.cfg.binding.to_dict(),
            "streaming": self._status is BackendLinkStatus.STREAMING,
            "authenticated": self._status.is_up,
            "auth_method": None if self._auth_method is None else self._auth_method.value,
            "auth_failure": self._auth_failure_detail,
            "credential": self.tokens.describe(robot_id=self.cfg.robot_id),
            "pairing_code": "SET" if self.cfg.pairing_code else "UNSET",
            "robot_id": self.cfg.robot_id,
            # Never the key itself. UNSET means no OFFER can be admitted.
            "command_signing_key": "SET" if self._signing_key else "UNSET",
            "held_commitment": (
                None if self.commitments.held() is None
                else {
                    "commitmentId": self.commitments.held().commitment_id,
                    "taskId": self.commitments.held().task_id,
                }
            ),
            "handler_registrations": self.handler_registrations,
            "last_connect_error": self._last_connect_error,
            "loss_policy": self.cfg.loss_policy.value,
            "rates": {
                "telemetry_interval_s": self.cfg.telemetry_interval_s,
                "heartbeat_interval_s": self.cfg.heartbeat_interval_s,
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
