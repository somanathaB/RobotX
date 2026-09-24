"""What the ESP32 reported, as immutable values RobotState can hold.

Pure dataclasses: no serial, no threads. `robotx.state.robot_state` imports
these, so this module must never import the state package back.

No value here is invented. Every field comes from a frame the ESP32 actually
sent; a documented-but-optional field it did not send is `None`, and before
the first TELEMETRY the whole telemetry block is `None`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Mapping, Optional, Tuple


class Esp32LinkStatus(str, Enum):
    """State of the Pi's link to the ESP32 motor/sensor controller.

    Produced by `robotx.esp32.link.Esp32Link.status()` and written into
    `RobotState` only by the agent (which re-exports this enum from
    `robotx.state.robot_state`, where consumers import it). What the ESP32
    *says* -- telemetry, faults -- is not here: this is purely whether the
    link can be relied on. `UP` means valid TELEMETRY is fresh and, when the
    link transmits, a PING has been answered on this connection.

    Deliberately its own enum rather than the backend's: a UART link does not
    authenticate, stream or get refused.

    `NOT_IMPLEMENTED` remains the default for a state nobody has written yet;
    a running agent reports `DISABLED` when the link is switched off.
    """

    NOT_IMPLEMENTED = "NOT_IMPLEMENTED"  # no link has reported (default)
    DISABLED = "DISABLED"                # switched off by configuration
    DISCONNECTED = "DISCONNECTED"        # port not open (or failed; retrying)
    CONNECTING = "CONNECTING"            # port open; no fresh TELEMETRY / PING ACK yet
    UP = "UP"                            # fresh TELEMETRY, and bidirectional if transmitting
    STALE = "STALE"                      # was receiving; TELEMETRY stopped
    DEGRADED = "DEGRADED"                # frames arrive but a fault is latched (reboot, no ACKs)
    STOPPING = "STOPPING"                # the link is shutting down

    @property
    def is_up(self) -> bool:
        return self is Esp32LinkStatus.UP


@dataclass(frozen=True)
class ControllerTelemetry:
    """One ESP32 TELEMETRY frame (PROTOCOL.md section 11), field for field."""

    uptime_ms: int
    state: str
    block_reason: str
    last_seq: Optional[int]
    last_reject: str
    left_cmd: int
    right_cmd: int
    left_applied: int
    right_applied: int
    front_left_cm: Optional[float]   # uncalibrated; None when not a valid measurement
    front_right_cm: Optional[float]
    front_valid: bool
    front_obstacle: bool
    front_warning: Optional[bool]
    rear_mm: Tuple[Optional[int], ...]
    rear_available: Optional[bool]
    rear_obstacle: Optional[bool]
    rear_sensor_fault: Optional[bool]
    forward_blocked: bool
    reverse_blocked: bool
    safety_stop: bool
    command_age_ms: int
    command_timeout: bool
    motor_drive_available: bool
    received_at: float               # wall clock when the Pi parsed it

    @classmethod
    def from_frame(cls, data: Mapping[str, Any], received_at: float) -> "ControllerTelemetry":
        """Build from a TELEMETRY frame `protocol.decode_line` already validated."""

        return cls(
            uptime_ms=data["uptime_ms"],
            state=data["state"],
            block_reason=data["block_reason"],
            last_seq=data["last_seq"],
            last_reject=data["last_reject"],
            left_cmd=data["left_cmd"],
            right_cmd=data["right_cmd"],
            left_applied=data["left_applied"],
            right_applied=data["right_applied"],
            front_left_cm=data["front_left_cm"],
            front_right_cm=data["front_right_cm"],
            front_valid=data["front_valid"],
            front_obstacle=data["front_obstacle"],
            front_warning=data.get("front_warning"),
            rear_mm=tuple(data["rear_mm"]),
            rear_available=data.get("rear_available"),
            rear_obstacle=data.get("rear_obstacle"),
            rear_sensor_fault=data.get("rear_sensor_fault"),
            forward_blocked=data["forward_blocked"],
            reverse_blocked=data["reverse_blocked"],
            safety_stop=data["safety_stop"],
            command_age_ms=data["command_age_ms"],
            command_timeout=data["command_timeout"],
            motor_drive_available=data["motor_drive_available"],
            received_at=received_at,
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "uptime_ms": self.uptime_ms,
            "state": self.state,
            "block_reason": self.block_reason,
            "last_seq": self.last_seq,
            "last_reject": self.last_reject,
            "motor": {
                "drive_available": self.motor_drive_available,
                "left_cmd": self.left_cmd,
                "right_cmd": self.right_cmd,
                "left_applied": self.left_applied,
                "right_applied": self.right_applied,
            },
            "front": {
                "valid": self.front_valid,
                "obstacle": self.front_obstacle,
                "warning": self.front_warning,
                "left_cm": self.front_left_cm,
                "right_cm": self.front_right_cm,
            },
            "rear": {
                "available": self.rear_available,
                "obstacle": self.rear_obstacle,
                "sensor_fault": self.rear_sensor_fault,
                "mm": list(self.rear_mm),
            },
            "gates": {
                "forward_blocked": self.forward_blocked,
                "reverse_blocked": self.reverse_blocked,
                "safety_stop": self.safety_stop,
            },
            "watchdog": {
                "command_age_ms": self.command_age_ms,
                "command_timeout": self.command_timeout,
            },
            "received_at": self.received_at,
        }


@dataclass(frozen=True)
class LinkCounters:
    """What the Pi side of the link has seen since the link started."""

    frames_ok: int = 0
    telemetry: int = 0
    diag: int = 0
    events: int = 0
    acks: int = 0
    esp32_errors: int = 0         # ERROR frames: the ESP32 refused something we sent
    rejected_lines: int = 0       # every Rejected line except EMPTY and the one after open
    bad_crc: int = 0
    too_long: int = 0
    partial_on_open: int = 0      # first line after open: expected garbage, not a fault
    ack_timeouts: int = 0
    unmatched_responses: int = 0  # an ACK/ERROR for a seq we were not waiting on
    commands_sent: int = 0
    stale_commands_dropped: int = 0
    open_failures: int = 0
    disconnects: int = 0

    def to_dict(self) -> Dict[str, int]:
        return dict(self.__dict__)


@dataclass(frozen=True)
class ControllerState:
    """The ESP32 as RobotX sees it: identity, latest telemetry, link facts."""

    telemetry: Optional[ControllerTelemetry] = None
    telemetry_age_s: Optional[float] = None
    proto: Optional[int] = None          # from READY, PING ACK or DIAG SYSTEM
    firmware: Optional[str] = None       # READY `fw`, if a READY was seen
    ready_seen: bool = False
    reboot_count: int = 0
    reboot_latched: bool = False
    last_event: Optional[str] = None
    last_error: Optional[str] = None     # most recent ERROR reason from the ESP32
    ping_rtt_ms: Optional[float] = None
    bidirectional: bool = False          # a PING has been answered this connection
    transmit_enabled: bool = False
    motion_enabled: bool = False
    motion_ready: bool = False           # the link would carry a DRIVE right now
    counters: LinkCounters = field(default_factory=LinkCounters)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "telemetry": None if self.telemetry is None else self.telemetry.to_dict(),
            "telemetry_age_s": None
            if self.telemetry_age_s is None
            else round(self.telemetry_age_s, 3),
            "proto": self.proto,
            "firmware": self.firmware,
            "ready_seen": self.ready_seen,
            "reboot_count": self.reboot_count,
            "reboot_latched": self.reboot_latched,
            "last_event": self.last_event,
            "last_error": self.last_error,
            "ping_rtt_ms": None if self.ping_rtt_ms is None else round(self.ping_rtt_ms, 1),
            "bidirectional": self.bidirectional,
            "transmit_enabled": self.transmit_enabled,
            "motion_enabled": self.motion_enabled,
            "motion_ready": self.motion_ready,
            "counters": self.counters.to_dict(),
        }


@dataclass(frozen=True)
class ControllerDiag:
    """The latest DIAG section of each kind, for diagnostics and health.

    Kept apart from `ControllerState` on purpose: DIAG is detailed hardware
    diagnostics on a slow rotation, not operational state, and it does not
    belong in the telemetry frame.
    """

    sections: Mapping[str, Mapping[str, Any]] = field(default_factory=dict)
    received_at: Mapping[str, float] = field(default_factory=dict)

    @property
    def system(self) -> Optional[Mapping[str, Any]]:
        return self.sections.get("SYSTEM")

    def to_dict(self) -> Dict[str, Any]:
        return {
            name: {"received_at": self.received_at.get(name), **dict(data)}
            for name, data in sorted(self.sections.items())
        }
