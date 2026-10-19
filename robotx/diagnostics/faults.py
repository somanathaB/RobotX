"""The Rover's active fault set, for TELEMETRY `faults[]`.

Contract: PHYSICAL_ROBOTX_BACKEND_CONTRACT.md section 4 (and P2B1 handoff
section 16) -- "the COMPLETE active set, every frame":

    {"code": "...", "severity": INFO|DEGRADED|BLOCKING|CRITICAL,
     "component": PI|ESP32|LINK|SENSOR|CAMERA|MOTOR|GNSS|POWER,
     "active": true, "sinceMs": <epoch ms>}

`[]` means measured and nothing active; omitting the block means not
measured. The backend accepts the block and does not read it yet (P2B1
section 5, ACCEPTED-IGNORED), so it changes nothing there today; it is sent
so the record is complete and honest when a reader arrives.

What is here, and what is not
-----------------------------
Every fault is derived from state the Pi actually holds: its own e-stop latch,
mode and mission, the ESP32 link's verdict, the ESP32's own TELEMETRY (only
while fresh -- a stale frame says nothing about the controller now, and that
staleness is itself the fault reported), the GPS reading, the navigator's
accuracy verdict and perception's status. Nothing about power: the Rover
measures no battery (the DEMO simulation is not a measurement), so no POWER
fault is ever claimed, healthy or not.

`SAFETY_STOP` is the ESP32's proximity stop and is reported as such
(`ESP32_SAFETY_STOP_LATCHED`), never as an e-stop. The Pi's latch is a
software latch (`PI_EMERGENCY_STOP_LATCHED`); this Rover has no e-stop circuit.

Pure apart from `FaultTracker`, which only remembers when each code was first
seen active so `sinceMs` is the onset, not the report time.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Dict, Iterable, List, Optional

from robotx.esp32.state import Esp32LinkStatus

if TYPE_CHECKING:  # robot_state cannot be imported first (robotx.mission import cycle)
    from robotx.state.robot_state import RobotSnapshot

SEVERITIES = ("INFO", "DEGRADED", "BLOCKING", "CRITICAL")
COMPONENTS = ("PI", "ESP32", "LINK", "SENSOR", "CAMERA", "MOTOR", "GNSS", "POWER")

# A paused mission's reason, by the prefix the agent records it under
# (`RobotAgent` sets `last_error` to "<prefix>: <why>" for each automatic pause).
_PAUSE_PREFIXES = (
    ("obstacle:", "MISSION_PAUSED_OBSTACLE", "SENSOR"),
    ("navigation:", "MISSION_PAUSED_NAVIGATION", "PI"),
    ("controller:", "MISSION_PAUSED_CONTROLLER", "ESP32"),
    ("backend:", "MISSION_PAUSED_BACKEND_LINK", "LINK"),
    ("recovery:", "MISSION_PAUSED_RECOVERY", "PI"),
)


@dataclass(frozen=True)
class Fault:
    code: str
    severity: str
    component: str

    def __post_init__(self) -> None:
        if self.severity not in SEVERITIES:
            raise ValueError(f"{self.severity!r} is not a contract severity")
        if self.component not in COMPONENTS:
            raise ValueError(f"{self.component!r} is not a contract component")


def _blocking(code: str, component: str) -> Fault:
    return Fault(code, "BLOCKING", component)


def active_faults(
    snapshot: "RobotSnapshot",
    *,
    estop_engaged: bool,
    esp32_attached: bool,
    esp32_config_error: Optional[str] = None,
    motion_enabled: bool,
    controller_max_age_s: float,
    position_issue: Optional[str] = None,
) -> List[Fault]:
    """Every fault active in `snapshot`, in a fixed order, each code once."""

    faults: List[Fault] = []

    # --- the Pi -----------------------------------------------------------------
    if estop_engaged:
        faults.append(_blocking("PI_EMERGENCY_STOP_LATCHED", "PI"))
    if snapshot.mode.value == "ERROR":
        faults.append(Fault("AGENT_ERROR", "CRITICAL", "PI"))

    # --- the motor/sensor controller ----------------------------------------------
    if not esp32_attached:
        faults.append(_blocking("ESP32_CONFIG_INVALID" if esp32_config_error else "ESP32_LINK_DISABLED", "LINK"))
    else:
        if not motion_enabled:
            # ROBOTX_ESP32_MOTION_ENABLED=0: the Pi will not transmit motion.
            faults.append(_blocking("PI_MOTION_DISABLED", "PI"))
        link = snapshot.communication.esp32
        if not link.is_up:
            faults.append(_blocking(f"ESP32_LINK_{link.value}", "LINK"))
        controller = snapshot.controller
        if controller is not None and controller.reboot_latched:
            faults.append(_blocking("ESP32_REBOOT_UNACKNOWLEDGED", "ESP32"))
        telemetry = controller.telemetry if controller is not None else None
        age = controller.telemetry_age_s if controller is not None else None
        if telemetry is None or age is None or age > controller_max_age_s:
            if link is not Esp32LinkStatus.STALE:  # already said, by the link
                faults.append(_blocking("ESP32_TELEMETRY_STALE", "ESP32"))
        else:
            if not telemetry.motor_drive_available:
                faults.append(_blocking("MOTOR_DRIVE_UNAVAILABLE", "MOTOR"))
            if not telemetry.front_valid:
                faults.append(_blocking("FRONT_SENSING_INVALID", "SENSOR"))
            if telemetry.safety_stop:
                faults.append(_blocking("ESP32_SAFETY_STOP_LATCHED", "ESP32"))
            if telemetry.command_timeout:
                faults.append(_blocking("ESP32_COMMAND_TIMEOUT", "ESP32"))
            if telemetry.rear_sensor_fault:
                # Blocks reverse only, which this Rover never commands.
                faults.append(Fault("REAR_SENSOR_FAULT", "DEGRADED", "SENSOR"))

    # --- position -----------------------------------------------------------------
    # No receiver at all reads UNAVAILABLE here (the agent's GPS-off reading).
    if not snapshot.gps.has_fix:
        faults.append(_blocking(f"GNSS_{snapshot.gps.status.value}", "GNSS"))
    elif position_issue is not None:
        faults.append(_blocking("GNSS_ACCURACY_INSUFFICIENT", "GNSS"))

    # --- perception ---------------------------------------------------------------
    perception = snapshot.perception
    if not perception.is_usable:
        status = perception.status.value
        faults.append(_blocking("PERCEPTION_NO_FRAME_METADATA" if status == "OK" else f"PERCEPTION_{status}",
                                "CAMERA"))

    # --- the mission held ---------------------------------------------------------
    mission = snapshot.mission
    if mission is not None and mission.is_active:
        if mission.interrupted:
            faults.append(_blocking("MISSION_INTERRUPTED", "PI"))
        if snapshot.mode.value == "PAUSED":
            faults.append(_pause_fault(snapshot.last_error))

    seen: Dict[str, Fault] = {}
    for fault in faults:
        seen.setdefault(fault.code, fault)
    return list(seen.values())


def _pause_fault(last_error: Optional[str]) -> Fault:
    text = last_error or ""
    for prefix, code, component in _PAUSE_PREFIXES:
        if text.startswith(prefix):
            return _blocking(code, component)
    # An operator (or backend) PAUSE: the Rover is fine, and waiting.
    return Fault("MISSION_PAUSED", "INFO", "PI")


class FaultTracker:
    """Remembers each active code's onset, so `sinceMs` says when it began."""

    def __init__(self) -> None:
        self._since_ms: Dict[str, int] = {}

    def report(self, faults: Iterable[Fault], *, now: Optional[float] = None) -> List[Dict[str, Any]]:
        """The `faults[]` block for these active faults; forgets the cleared ones."""

        now_ms = int(round((time.time() if now is None else now) * 1000))
        faults = list(faults)
        active = {f.code for f in faults}
        for code in list(self._since_ms):
            if code not in active:
                del self._since_ms[code]
        out = []
        for fault in faults:
            since = self._since_ms.setdefault(fault.code, now_ms)
            out.append({"code": fault.code, "severity": fault.severity, "component": fault.component,
                        "active": True, "sinceMs": since})
        return out
