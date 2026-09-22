"""The one authoritative representation of what the Pi knows about the robot.

Every subsystem writes its result here once per tick, and every reader
(telemetry, the HTTP API, health) reads from here. No subsystem keeps its own
parallel copy of the robot's mode, position, or intent.

It holds only what the Pi can actually know. There is no battery percentage,
no wheel odometry and no motor feedback, because in the target architecture the
Pi does not own those sensors -- the ESP32 does, and the link to it does not
exist yet. Those fields are absent rather than filled with placeholder numbers.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Any, Dict, Optional

from robotx.control.motion import MotionIntent
from robotx.diagnostics.health import HealthReport, HealthStatus
from robotx.hardware.gps import GpsReading, GPSStatus
from robotx.navigation.navigator import NavigationState
from robotx.perception.types import PerceptionResult, PerceptionStatus
from robotx.localization.position import Position


class MissionRefused(Exception):
    """The agent will not carry out a mission change, and why.

    A *considered* refusal -- no route to resume, no home position to return
    to -- as opposed to a bug. It lives here, in the domain, rather than in the
    communication package, so the agent can raise it without importing its own
    transport. Callers turn it into whatever their protocol calls a rejection.
    """


class OperatingMode(str, Enum):
    """What the agent is trying to do."""

    IDLE = "IDLE"          # running, no mission
    AUTO = "AUTO"          # mission active, following a route
    PAUSED = "PAUSED"      # mission held: route retained, motion suspended
    STOPPED = "STOPPED"    # explicitly halted by an operator
    ERROR = "ERROR"        # agent-level failure

    @property
    def mission_active(self) -> bool:
        """Whether the decision layer may produce forward motion.

        `PAUSED` is false here and that is the whole point of the mode: the
        route survives so `RESUME` has something to go back to, while the
        decision layer keeps issuing a hold exactly as it does when idle.
        Pausing must never be a state in which the robot still drives.
        """

        return self is OperatingMode.AUTO

    @property
    def has_route(self) -> bool:
        """Whether a route is expected to still be loaded in this mode."""

        return self in (OperatingMode.AUTO, OperatingMode.PAUSED)


class LinkStatus(str, Enum):
    """State of an outbound link to another system."""

    DISABLED = "DISABLED"                # switched off by configuration
    NOT_IMPLEMENTED = "NOT_IMPLEMENTED"  # planned, no code path exists yet
    CONNECTING = "CONNECTING"
    CONNECTED = "CONNECTED"
    DISCONNECTED = "DISCONNECTED"
    REJECTED = "REJECTED"                # server refused the handshake

    @property
    def is_up(self) -> bool:
        return self is LinkStatus.CONNECTED


@dataclass(frozen=True)
class CommunicationState:
    """Outbound links, as observed rather than as hoped.

    `backend` is written only by the transport, from Socket.IO's own connect
    and disconnect callbacks. Nothing else may set it, so the state cannot
    drift into claiming a link that is not there -- which is the ground truth a
    dashboard's "online" indicator ultimately rests on.
    """

    # Pi -> ESP32 motor/safety controller. Out of scope for this stage.
    esp32: LinkStatus = LinkStatus.NOT_IMPLEMENTED
    # Pi -> FalconAut backend.
    backend: LinkStatus = LinkStatus.DISABLED
    backend_detail: str = ""
    # When the current backend link state was entered.
    backend_since: Optional[float] = None
    # Last time an emit to the backend completed without raising. This is the
    # only evidence the Pi has that the link carries traffic; a TCP connection
    # that no longer delivers looks identical to a healthy one until you write.
    backend_last_send_at: Optional[float] = None
    # Last message received from the backend, for the same reason.
    backend_last_recv_at: Optional[float] = None
    # True while event names are guesses rather than the real contract.
    backend_protocol_provisional: bool = True

    def to_dict(self) -> Dict[str, Any]:
        return {
            "esp32": self.esp32.value,
            "backend": self.backend.value,
            "backend_detail": self.backend_detail,
            "backend_since": self.backend_since,
            "backend_last_send_at": self.backend_last_send_at,
            "backend_last_recv_at": self.backend_last_recv_at,
            "backend_protocol_provisional": self.backend_protocol_provisional,
        }


@dataclass(frozen=True)
class RobotSnapshot:
    """Immutable point-in-time copy of the whole robot state."""

    robot_id: str
    mode: OperatingMode
    started_at: float
    updated_at: float
    uptime_s: float
    gps: GpsReading
    position: Optional[Position]
    navigation: NavigationState
    perception: PerceptionResult
    motion_intent: MotionIntent
    communication: CommunicationState
    health: HealthReport
    last_error: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "mode": self.mode.value,
            "uptime_s": round(self.uptime_s, 1),
            "started_at": self.started_at,
            "updated_at": self.updated_at,
            "gps": self.gps.to_dict(),
            "position": None if self.position is None else self.position.to_dict(),
            "navigation": self.navigation.to_dict(),
            "perception": self.perception.to_dict(),
            "motion_intent": self.motion_intent.to_dict(),
            "communication": self.communication.to_dict(),
            "health": self.health.to_dict(),
            "last_error": self.last_error,
        }


class RobotState:
    """Mutable, thread-safe holder of the current robot state.

    The agent loop writes; the HTTP handlers and telemetry read. All reads go
    through `snapshot()`, which returns a consistent immutable copy rather than
    a live view that could change mid-serialization.
    """

    def __init__(self, robot_id: str) -> None:
        self._lock = threading.Lock()
        self._robot_id = robot_id
        self._started_at = time.time()

        self._mode = OperatingMode.IDLE
        self._gps = GpsReading(status=GPSStatus.UNAVAILABLE)
        self._position: Optional[Position] = None
        self._navigation = NavigationState()
        self._perception = PerceptionResult.unavailable(PerceptionStatus.DISABLED)
        self._motion_intent = MotionIntent.hold("agent starting")
        self._communication = CommunicationState()
        self._health = HealthReport(status=HealthStatus.UNKNOWN)
        self._last_error: Optional[str] = None
        self._updated_at = self._started_at

    # --- identity ------------------------------------------------------------

    @property
    def robot_id(self) -> str:
        return self._robot_id

    @property
    def started_at(self) -> float:
        return self._started_at

    # --- mode ----------------------------------------------------------------

    @property
    def mode(self) -> OperatingMode:
        with self._lock:
            return self._mode

    def set_mode(self, mode: OperatingMode, *, error: Optional[str] = None) -> None:
        """Change the operating mode.

        A recorded error is deliberately *not* cleared by a mode change: the
        agent stopping after a failure must not erase why it failed. Clearing
        is explicit, via `clear_error()`.
        """

        with self._lock:
            self._mode = mode
            if error is not None:
                self._last_error = error
            self._updated_at = time.time()

    def clear_error(self) -> None:
        with self._lock:
            self._last_error = None
            self._updated_at = time.time()

    # --- per-tick updates ----------------------------------------------------

    def update_gps(self, reading: GpsReading, position: Optional[Position]) -> None:
        with self._lock:
            self._gps = reading
            # Keep the last known position when a fix drops out; `gps.status`
            # and `position.timestamp` tell a reader how old it is.
            if position is not None:
                self._position = position
            self._updated_at = time.time()

    def update_navigation(self, navigation: NavigationState) -> None:
        with self._lock:
            self._navigation = navigation
            self._updated_at = time.time()

    def update_perception(self, perception: PerceptionResult) -> None:
        with self._lock:
            self._perception = perception
            self._updated_at = time.time()

    def update_motion_intent(self, intent: MotionIntent) -> None:
        with self._lock:
            self._motion_intent = intent
            self._updated_at = time.time()

    def update_health(self, health: HealthReport) -> None:
        with self._lock:
            self._health = health
            self._updated_at = time.time()

    def update_communication(self, **changes: Any) -> None:
        with self._lock:
            self._communication = replace(self._communication, **changes)
            self._updated_at = time.time()

    # --- reads ---------------------------------------------------------------

    def snapshot(self) -> RobotSnapshot:
        now = time.time()
        with self._lock:
            return RobotSnapshot(
                robot_id=self._robot_id,
                mode=self._mode,
                started_at=self._started_at,
                updated_at=self._updated_at,
                uptime_s=now - self._started_at,
                gps=self._gps,
                position=self._position,
                navigation=self._navigation,
                perception=self._perception,
                motion_intent=self._motion_intent,
                communication=self._communication,
                health=self._health,
                last_error=self._last_error,
            )
