"""The one authoritative representation of what the Pi knows about the robot.

Every subsystem writes its result here once per tick, and every reader
(telemetry, the HTTP API, health) reads from here. No subsystem keeps its own
parallel copy of the robot's mode, position, or intent.

It holds only what the Pi can actually know. The ESP32 owns the motor and
range sensors; what it reports arrives through `robotx.esp32` and is held here
as `controller` (operational TELEMETRY) and `controller_diag` (DIAG), exactly as
reported. There is no battery percentage and no wheel odometry, because the
ESP32 protocol does not carry them; those fields are absent rather than filled
with placeholder numbers.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Any, Dict, Optional

from robotx.control.motion import MotionIntent
from robotx.control.safety import UNEVALUATED, SafetyDecision
from robotx.hardware.battery import BATTERY_UNAVAILABLE_REASON
from robotx.diagnostics.health import HealthReport, HealthStatus
# Esp32LinkStatus is defined beside the link that produces it and re-exported
# here, where every consumer has always imported it from.
from robotx.esp32.state import ControllerDiag, ControllerState, Esp32LinkStatus  # noqa: F401
from robotx.hardware.gps import GpsReading, GPSStatus
from robotx.mission.mission import ActiveMission
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


class BackendLinkStatus(str, Enum):
    """State of the Pi's link to the RobotX backend.

    Normal progression::

        DISCONNECTED -> CONNECTING -> CONNECTED
                     -> AUTHENTICATING -> AUTHENTICATED -> STREAMING

    `CONNECTED` and `AUTHENTICATED` are kept apart deliberately. The backend's
    socket connects anonymously, so a connected socket says nothing about
    whether this robot is allowed to be on it -- and an authentication failure
    arrives as a silent server-side disconnect, which is indistinguishable from
    a network drop unless the link records which of the two it was waiting for.

    This enum is **specific to the backend**. The ESP32 link has its own
    (`Esp32LinkStatus`) because a UART link does not authenticate, does not
    stream and cannot be "refused" -- sharing one enum would force an ESP32
    into states that have no meaning for it.
    """

    DISABLED = "DISABLED"                # switched off by configuration
    CONNECTING = "CONNECTING"
    CONNECTED = "CONNECTED"              # socket open, anonymous, not yet authed
    AUTHENTICATING = "AUTHENTICATING"    # AUTH emitted, awaiting AUTH_SUCCESS
    AUTHENTICATED = "AUTHENTICATED"      # AUTH_SUCCESS received
    STREAMING = "STREAMING"              # authenticated and publishing telemetry
    DISCONNECTED = "DISCONNECTED"
    AUTH_FAILED = "AUTH_FAILED"          # backend refused this robot's credential

    @property
    def is_up(self) -> bool:
        """Whether the link can actually carry robot traffic.

        A merely `CONNECTED` socket cannot: until authentication succeeds the
        backend will not attribute anything sent on it to this robot.
        """

        return self in (BackendLinkStatus.AUTHENTICATED, BackendLinkStatus.STREAMING)

    @property
    def socket_open(self) -> bool:
        """Whether the transport is up, regardless of authentication."""

        return self in (
            BackendLinkStatus.CONNECTED,
            BackendLinkStatus.AUTHENTICATING,
            BackendLinkStatus.AUTHENTICATED,
            BackendLinkStatus.STREAMING,
        )


@dataclass(frozen=True)
class PowerState:
    """Battery/power, in the shape `robotx.hardware.battery` already defines.

    Present in state so that **no consumer has to call the hardware itself**.
    Telemetry built from a snapshot that then reached past it for a live
    battery read would be reporting two different instants in one frame.

    Today the only value this can hold is `UNAVAILABLE`: this robot has no
    fuel gauge, ADC or divider. When battery sensing arrives it will be on the
    ESP32, and the parser will write this same field -- every consumer keeps
    working and can still tell measured from unavailable.
    """

    status: str = "UNAVAILABLE"
    percent: Optional[float] = None
    voltage_v: Optional[float] = None
    source: str = BATTERY_UNAVAILABLE_REASON

    @property
    def is_measured(self) -> bool:
        return self.status != "UNAVAILABLE" and self.percent is not None

    @classmethod
    def from_status_dict(cls, data: Dict[str, Any]) -> "PowerState":
        """Build from the mapping `battery_status()` returns."""

        return cls(
            status=str(data.get("status", "UNAVAILABLE")),
            percent=data.get("percent"),
            voltage_v=data.get("voltage_v"),
            source=str(data.get("source", BATTERY_UNAVAILABLE_REASON)),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "status": self.status,
            "percent": self.percent,
            "voltage_v": self.voltage_v,
            "source": self.source,
        }


@dataclass(frozen=True)
class CommunicationState:
    """The Rover's two outbound links, as observed rather than as hoped.

    The two halves are **independent**, and that is the point. The ESP32 is the
    robot's own hardware; the backend is a remote supervisor. A rover with a
    dead ESP32 and a healthy backend is in serious trouble, and a rover with a
    live ESP32 and no backend is merely unsupervised -- collapsing them into
    one verdict loses exactly the distinction an operator needs.

    Each half is written only by its own transport, so the state cannot drift
    into claiming a link that is not there.

    No backend *protocol* detail lives here. Which event names are in use and
    which credential authenticated are facts about a Socket.IO client, not
    about the robot, and they belong to that client's own `describe()`.
    """

    # --- Pi <-> ESP32 motor/sensor controller --------------------------------
    esp32: Esp32LinkStatus = Esp32LinkStatus.NOT_IMPLEMENTED
    esp32_detail: str = ""
    # When the current ESP32 link state was entered.
    esp32_since: Optional[float] = None
    # Last time anything at all arrived from the ESP32. Protocol-agnostic on
    # purpose: bytes arriving is observable without knowing their framing.
    esp32_last_rx_at: Optional[float] = None

    # --- Pi -> RobotX backend ------------------------------------------------
    backend: BackendLinkStatus = BackendLinkStatus.DISABLED
    backend_detail: str = ""
    # When the current backend link state was entered.
    backend_since: Optional[float] = None
    # Last time an emit to the backend completed without raising. This is the
    # only evidence the Pi has that the link carries traffic; a TCP connection
    # that no longer delivers looks identical to a healthy one until you write.
    backend_last_send_at: Optional[float] = None
    # Last message received from the backend, for the same reason.
    backend_last_recv_at: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "esp32": self.esp32.value,
            "esp32_detail": self.esp32_detail,
            "esp32_since": self.esp32_since,
            "esp32_last_rx_at": self.esp32_last_rx_at,
            "backend": self.backend.value,
            "backend_detail": self.backend_detail,
            "backend_since": self.backend_since,
            "backend_last_send_at": self.backend_last_send_at,
            "backend_last_recv_at": self.backend_last_recv_at,
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
    power: PowerState
    communication: CommunicationState
    health: HealthReport
    # The safety gate's verdict on `motion_intent`. That field always holds the
    # *gated* intent -- what may actually be acted on -- and this records what
    # the gate did to get there. Without it an operator looking at a stopped
    # rover cannot tell a navigation hold from a safety veto.
    #
    # Defaulted so that constructing a snapshot does not require knowing about
    # every field, and defaulted to UNEVALUATED specifically: a snapshot built
    # without a verdict reports "not yet cleared to move", never "cleared".
    safety: SafetyDecision = UNEVALUATED
    # The RobotX task this Rover is carrying out, and how far through it it is.
    # None until one is assigned; a finished mission is kept until the next
    # assignment replaces it, so a consumer can still see how the last one
    # ended rather than watching it vanish at the moment it completed.
    #
    # Defaulted so that a snapshot can be built without knowing about
    # missions -- a Rover driving a locally supplied route has none.
    mission: Optional[ActiveMission] = None
    last_error: Optional[str] = None
    # What the ESP32 reports, as reported. None until an ESP32 link exists and
    # has something to say. DIAG is kept separate from the operational block.
    controller: Optional[ControllerState] = None
    controller_diag: Optional[ControllerDiag] = None

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
            "mission": None if self.mission is None else self.mission.to_dict(),
            "perception": self.perception.to_dict(),
            "motion_intent": self.motion_intent.to_dict(),
            "safety": self.safety.to_dict(),
            "power": self.power.to_dict(),
            "communication": self.communication.to_dict(),
            "controller": None if self.controller is None else self.controller.to_dict(),
            "controller_diag": None
            if self.controller_diag is None
            else self.controller_diag.to_dict(),
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
        self._mission: Optional[ActiveMission] = None
        self._perception = PerceptionResult.unavailable(PerceptionStatus.DISABLED)
        self._motion_intent = MotionIntent.hold("agent starting")
        self._safety = UNEVALUATED
        self._power = PowerState()
        self._communication = CommunicationState()
        self._health = HealthReport(status=HealthStatus.UNKNOWN)
        self._controller: Optional[ControllerState] = None
        self._controller_diag: Optional[ControllerDiag] = None
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

    def update_mission(self, mission: Optional[ActiveMission]) -> None:
        """Record the active mission and its progress.

        Written by the agent from the mission manager's result, in the same
        tick as the navigation state it was derived from. This is the only
        place an assigned route is stored: the manager holds it to drive the
        navigator, and every *consumer* -- telemetry, the local API, the
        backend link -- reads it from here, so there is no second answer to
        "what is this Rover delivering".
        """

        with self._lock:
            self._mission = mission
            self._updated_at = time.time()

    def update_perception(self, perception: PerceptionResult) -> None:
        with self._lock:
            self._perception = perception
            self._updated_at = time.time()

    def update_motion_intent(self, intent: MotionIntent) -> None:
        with self._lock:
            self._motion_intent = intent
            self._updated_at = time.time()

    def update_safety(self, decision: SafetyDecision) -> None:
        """Record the gate's verdict alongside the intent it produced.

        Written by the agent in the same tick as `update_motion_intent`, from
        the same `SafetyDecision`, so the two can never describe different
        evaluations.
        """

        with self._lock:
            self._safety = decision
            self._updated_at = time.time()

    def update_power(self, power: PowerState) -> None:
        """Record the battery/power reading.

        Written by whichever producer owns the measurement -- today the Pi's
        `battery` module, which reports UNAVAILABLE; later the ESP32 parser.
        Consumers read it from a snapshot and never call the producer.
        """

        with self._lock:
            self._power = power
            self._updated_at = time.time()

    def update_health(self, health: HealthReport) -> None:
        with self._lock:
            self._health = health
            self._updated_at = time.time()

    def update_communication(self, **changes: Any) -> None:
        with self._lock:
            self._communication = replace(self._communication, **changes)
            self._updated_at = time.time()

    def update_controller(
        self, controller: Optional[ControllerState], diag: Optional[ControllerDiag]
    ) -> None:
        """Record what the ESP32 reported. Written by the agent from the link."""

        with self._lock:
            self._controller = controller
            self._controller_diag = diag
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
                mission=self._mission,
                perception=self._perception,
                motion_intent=self._motion_intent,
                safety=self._safety,
                power=self._power,
                communication=self._communication,
                health=self._health,
                last_error=self._last_error,
                controller=self._controller,
                controller_diag=self._controller_diag,
            )
