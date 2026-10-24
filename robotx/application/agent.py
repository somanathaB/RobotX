"""The Pi robot agent: startup order, the main loop, and shutdown.

Startup order (each step depends only on the ones before it):

    configuration -> logging -> camera -> perception -> GPS
                  -> navigation + state -> running

The agent degrades rather than refusing to start: if the camera is missing, or
GPS cannot open its serial port, the agent still runs and reports those
subsystems as unavailable. That is what makes the Pi testable on its own. What
it will never do is act as if a missing subsystem were a healthy one -- the
decision layer stops the robot whenever perception is not usable.

The agent does not start motors, and imports no motor driver. It publishes a
`MotionIntent`; when the ESP32 link is enabled, the *gated* decision is handed
to `robotx.esp32.link.Esp32Link`, which is the only path to the ESP32 and
carries motion only when `ROBOTX_ESP32_MOTION_ENABLED` is set.

Each tick:
    GPS -> position -> ESP32 status -> navigation -> (with perception) decision
    -> safety gate -> motion intent -> ESP32 link -> state -> telemetry / health
"""

from __future__ import annotations

import asyncio
import logging
import os
import time
from typing import Any, Callable, Dict, List, Optional, Sequence

from robotx.config.logging_setup import log_event, setup_logging
from robotx.config.settings import Settings
from robotx.control.applied_motion import AppliedMotionConfig, AppliedMotionMonitor
from robotx.control.bench_motion import BenchMotion, BenchMotionConfig, BenchOutcome, BenchRefused
from robotx.control.decision import DecisionConfig, DecisionMaker
from robotx.control.motion import MotionIntent
from robotx.control.obstacle_recovery import ObstacleRecovery, ObstacleRecoveryConfig
from robotx.control.safety import RangeReading, RangeStatus, SafetyConfig, SafetyDecision, SafetyGate
from robotx.diagnostics.faults import Fault, FaultTracker, active_faults
from robotx.diagnostics.health import (
    ComponentHealth,
    HealthConfig,
    HealthMonitor,
    HealthStatus,
)
from robotx.localization.esp32_gps import Esp32GpsReader
from robotx.esp32.front_range import check_stop_distance, front_range
from robotx.esp32.link import Esp32Config, Esp32Link, Esp32Status
from robotx.esp32.sim_transport import HostSimulatorPort
from robotx.hardware.camera import CameraConfig, CameraError, CameraStatus, CameraStream
from robotx.hardware.gps import GPSConfig, GPSReader, GpsReading, GPSStatus
from robotx.mission.admission import AdmissionConfig, check_route_consistency, check_start_position
from robotx.mission.manager import MissionAssignment, MissionManager
from robotx.mission.mission import (
    ActiveMission,
    Mission,
    MissionRejected,
    MissionRejectReason,
)
from robotx.navigation.navigator import (
    NavigationConfig,
    NavigationState,
    NavigationStatus,
    Navigator,
)
from robotx.navigation.progress import ProgressConfig, ProgressMonitor
from robotx.perception.pipeline import PerceptionPipeline
from robotx.perception.types import PerceptionResult, PerceptionStatus
from robotx.localization.local_frame import (
    DEFAULT_LOCAL_ORIGIN,
    DeadReckoner,
    DeadReckoningConfig,
    LocalFrame,
)
from robotx.localization.position import LatLon, PositionConfig, PositionEstimator
from robotx.hardware.battery import battery_status
from robotx.hardware.demo_battery import DemoBatteryConfig, DemoBatterySimulator
from robotx.state.robot_state import (
    BackendLinkStatus,
    Esp32LinkStatus,
    PowerState,
    MissionRefused,
    OperatingMode,
    RobotSnapshot,
    RobotState,
)
from robotx.state.telemetry import build_telemetry


logger = logging.getLogger(__name__)

# OFFER_REJECT reason for a Rover whose perception is not usable. Outside the
# readiness vocabulary on purpose (protocol.READINESS_BLOCKERS is the backend's,
# and has no such code): the backend handles it as a per-Leg refusal.
PERCEPTION_UNAVAILABLE = "PERCEPTION_UNAVAILABLE"
# OFFER_REJECT reason while an operator's bench pulse is running (a few seconds).
BENCH_TEST_ACTIVE = "BENCH_TEST_ACTIVE"

# A loop that has not completed a tick for this long is not alive, and the
# backend heartbeat stops. Well inside the backend's ~10 s freshness budget, so
# a hung agent goes stale there rather than being reported alive by its socket.
AGENT_STALL_S = 5.0

TelemetrySink = Callable[[Dict[str, Any]], None]

# Subsystem status -> health. Looked up with `.get`, so an unhandled value
# degrades the report rather than raising inside the agent loop.
_CAMERA_HEALTH = {
    CameraStatus.STREAMING: (HealthStatus.HEALTHY, ""),
    CameraStatus.STARTING: (HealthStatus.DEGRADED, "waiting for first frame"),
    CameraStatus.STALLED: (HealthStatus.FAILED, "frames stopped arriving"),
    CameraStatus.FAILED: (HealthStatus.FAILED, "capture failed"),
    CameraStatus.STOPPED: (HealthStatus.UNKNOWN, "not started"),
}

_PERCEPTION_HEALTH = {
    PerceptionStatus.OK: (HealthStatus.HEALTHY, ""),
    PerceptionStatus.DISABLED: (HealthStatus.UNKNOWN, "perception disabled"),
    PerceptionStatus.NO_FRAME: (HealthStatus.DEGRADED, "no camera frame"),
    PerceptionStatus.STALE: (HealthStatus.DEGRADED, "results are stale"),
    PerceptionStatus.DETECTOR_ERROR: (HealthStatus.FAILED, "inference failing"),
}

_GPS_HEALTH = {
    GPSStatus.FIX: (HealthStatus.HEALTHY, ""),
    GPSStatus.NO_FIX: (HealthStatus.DEGRADED, "receiver reports no fix"),
    GPSStatus.STARTING: (HealthStatus.DEGRADED, "waiting for first sentence"),
    GPSStatus.STALE: (HealthStatus.DEGRADED, "last fix has aged out"),
    GPSStatus.DISCONNECTED: (HealthStatus.FAILED, "serial port unavailable"),
    GPSStatus.UNAVAILABLE: (HealthStatus.FAILED, "GPS dependencies missing"),
}



def _is_obstacle_stop(intent: MotionIntent) -> bool:
    """Whether a gated intent is a stop for something in the rover's path.

    By reason text, as before: the decision layer's camera stops all say
    "obstacle" (or "person detected"), and so does the gate's range veto.
    """

    return intent.is_stop and ("obstacle" in intent.reason or "person detected" in intent.reason)

class RobotAgent:
    """Owns every Pi subsystem and the loop that ties them together."""

    def __init__(self, settings: Settings) -> None:
        # The offer-acceptance demonstration relaxes motion prerequisites only
        # because nothing can move: refused, not quietly corrected, otherwise.
        if settings.offer_acceptance_demo and (settings.esp32_motion_enabled or settings.bench_motion_enabled):
            raise ValueError(
                "ROBOTX_OFFER_ACCEPTANCE_DEMO=1 requires ROBOTX_ESP32_MOTION_ENABLED=0 and "
                "ROBOTX_BENCH_MOTION_ENABLED=0: the mode accepts offers without driving, so no "
                "motor command path may be enabled"
            )
        # The bench wheel test is the opposite: a pulse needs the motor path, and
        # it is bound to an offer from a backend. Never combined with the other demo.
        if settings.bench_wheel_demo and not (
            settings.esp32_motion_enabled and settings.bench_motion_enabled and settings.socket_enabled
            and not settings.offer_acceptance_demo
        ):
            raise ValueError(
                "ROBOTX_BENCH_WHEEL_DEMO=1 requires ROBOTX_ESP32_MOTION_ENABLED=1, "
                "ROBOTX_BENCH_MOTION_ENABLED=1, ROBOTX_SOCKET_ENABLED=1 and ROBOTX_OFFER_ACCEPTANCE_DEMO=0"
            )
        self.settings = settings
        self.state = RobotState(settings.robot_id)

        self.camera: Optional[CameraStream] = None
        self.perception: Optional[PerceptionPipeline] = None
        self.gps: Optional[GPSReader] = None
        # Set when GPS was not started because its port is the ESP32's UART.
        self._gps_port_conflict: Optional[str] = None
        # Set when GPS is enabled but has no source to read (ESP32 link off).
        self._gps_no_source: Optional[str] = None
        # The one owner of the ESP32 UART. None when ROBOTX_ESP32_ENABLED=0.
        self.esp32: Optional[Esp32Link] = None
        # Why an enabled ESP32 link was not started, if its config was invalid.
        self._esp32_config_error: Optional[str] = None
        # ESP32 reboots already acted on, so each triggers the latch once.
        self._esp32_reboots_handled = 0
        # Link interruptions already seen: the link's (link_losses, disconnects)
        # counters. Each new one pauses an active mission, even when the link
        # lost and re-proved itself between two ticks (see `tick`, step 2b).
        self._esp32_interruptions_seen = (0, 0)
        # Typed as Any to keep `socketio` out of this module's imports; see
        # `_start_backend_link`. None whenever no backend is configured.
        self.backend: Optional[Any] = None

        self.position_estimator = PositionEstimator(PositionConfig.from_settings(settings))

        # Fallback localization for a rover with no usable GPS. Built only when
        # enabled, so a GPS-equipped rover carries no dead-reckoning state that
        # could be read by mistake.
        self.local_frame = LocalFrame(
            origin=(
                (float(settings.local_origin_lat), float(settings.local_origin_lon))
                if settings.local_origin_lat is not None
                and settings.local_origin_lon is not None
                else DEFAULT_LOCAL_ORIGIN
            )
        )
        self.dead_reckoner: Optional[DeadReckoner] = (
            DeadReckoner(DeadReckoningConfig.from_settings(settings))
            if settings.deadreckon_enabled
            else None
        )

        self.navigator = Navigator(NavigationConfig.from_settings(settings))
        # Holds the RobotX assignment and keeps the navigator pointed at the
        # leg being driven. It plans nothing: the waypoints are RobotX's.
        self.missions = MissionManager(self.navigator)
        self.decision = DecisionMaker(DecisionConfig.from_settings(settings))
        self.safety = SafetyGate(SafetyConfig.from_settings(settings))
        # When an obstacle-stopped rover may move again, and when it asks for help.
        self.obstacles = ObstacleRecovery(ObstacleRecoveryConfig.from_settings(settings))
        # When a navigation hold or a drive that gets nowhere asks for help.
        self.progress = ProgressMonitor(ProgressConfig.from_settings(settings))
        # When the ESP32 reports it is not applying the DRIVE being sent.
        self.applied_motion = AppliedMotionMonitor(AppliedMotionConfig.from_settings(settings))
        # The operator's bench pulse (robotx.control.bench_motion). Idle unless
        # `start_bench_motion` is called, which only the guarded, disabled-by-
        # default bench route does. Never a mission.
        self.bench = BenchMotion(BenchMotionConfig.from_settings(settings))
        # Bench wheel-test demonstration: tasks this process accepted without a
        # mission. A pulse may be bound only to one of these, so a restart makes
        # every earlier acceptance ineligible.
        self._bench_wheel_tasks: set = set()
        # Whether the previous tick's gated intent moved the wheels: what the
        # applied-motion monitor judges the ESP32's feedback against.
        self._intent_moving = False
        # The ESP32's state as of the previous tick's link check, for the one
        # consumer that must read it before this tick's check runs: the GPS
        # heading (see `_wheels_driven`).
        self._last_controller: Optional[Any] = None
        # Route admission against where the Rover stands (robotx.mission.admission).
        self.admission = AdmissionConfig.from_settings(settings)
        self.health = HealthMonitor(HealthConfig.from_settings(settings))
        # When each active fault began, for TELEMETRY faults[] (`fault_report`).
        self._fault_tracker = FaultTracker()
        # DEMO battery simulation, only when explicitly enabled; otherwise the
        # battery stays UNAVAILABLE exactly as before.
        self.demo_battery: Optional[DemoBatterySimulator] = self._build_demo_battery(settings)

        self._task: Optional[asyncio.Task] = None
        self._running = False
        self._started = False
        self._last_telemetry: Dict[str, Any] = {}
        self._last_telemetry_t = 0.0
        self._last_health_t = 0.0
        # Monotonic time the last tick completed. What the backend heartbeat
        # vouches for: see `is_alive`.
        self._last_tick_at: Optional[float] = None
        self._telemetry_sinks: List[TelemetrySink] = []

        # Retained across a pause so RESUME and RETURN have something real to
        # act on. Both stay empty/None until a mission actually provides them.
        self._mission_route: List[LatLon] = []
        self._mission_origin: Optional[LatLon] = None

    @staticmethod
    def _build_demo_battery(settings: Settings) -> Optional[DemoBatterySimulator]:
        """The simulated pack, or None. A bad setting disables it, loudly."""

        if not settings.demo_battery_enabled:
            return None
        try:
            # A recharge must be reported in a CHARGING frame before drain resumes
            # only when there is a backend to report to.
            cfg = DemoBatteryConfig.from_settings(settings, confirm_reports=settings.socket_enabled)
            simulator = DemoBatterySimulator(cfg)
        except ValueError as e:
            log_event(logger, "demo_battery.config_invalid", "demo battery simulation NOT enabled",
                      level=logging.ERROR, error=str(e))
            return None
        log_event(logger, "demo_battery.enabled",
                  "SIMULATED battery telemetry is ON (demo); this is not a measurement",
                  level=logging.WARNING, drain_model=cfg.drain_model.value,
                  threshold=cfg.recharge_threshold_percent, recharge_s=cfg.recharge_duration_s)
        return simulator

    def battery_telemetry_sent(self, payload: Dict[str, Any]) -> None:
        """The backend link sent a TELEMETRY frame (see `DemoBatterySimulator.note_reported`)."""

        if self.demo_battery is not None:
            self.demo_battery.note_reported(payload)

    # --- lifecycle -----------------------------------------------------------

    async def start(self) -> None:
        """Bring up subsystems in order, then start the agent loop."""

        if self._started:
            return
        self._started = True

        setup_logging(self.settings.log_level)
        log_event(
            logger,
            "agent.starting",
            robot_id=self.settings.robot_id,
            log_level=self.settings.log_level,
        )
        if self.settings.offer_acceptance_demo:
            log_event(
                logger,
                "agent.offer_acceptance_demo",
                "OFFER-ACCEPTANCE DEMONSTRATION: offers are admitted and accepted, no mission is ever "
                "started and motor commands are disabled",
                level=logging.WARNING,
            )
        if self.settings.bench_wheel_demo:
            log_event(
                logger,
                "agent.bench_wheel_demo",
                "BENCH WHEEL-TEST DEMONSTRATION: offers are accepted without a mission; an operator may "
                "run one bounded wheel pulse per request for an accepted task. Rover must be on a stand.",
                level=logging.WARNING,
            )

        self._start_camera()
        self._start_perception()
        # Before GPS: GPS must not be allowed onto the ESP32's UART.
        self._start_esp32()
        self._start_gps()

        self.state.update_communication(
            esp32=Esp32LinkStatus.DISABLED,
            esp32_detail=(
                f"configuration error: {self._esp32_config_error}"
                if self._esp32_config_error
                else "disabled by configuration"
            ),
            backend=BackendLinkStatus.DISABLED,
            backend_detail="",
        )
        self._update_esp32()
        self.state.set_mode(OperatingMode.IDLE)

        self._running = True
        self._task = asyncio.create_task(self._loop(), name="agent-loop")

        # Last, and never blocking: the agent is fully operational before the
        # backend is even contacted, so a missing or broken backend cannot
        # delay or prevent the robot coming up.
        await self._start_backend_link()

        log_event(logger, "agent.started", hz=self.settings.agent_hz)

    async def _start_backend_link(self) -> None:
        """Attach the backend link, if one is configured.

        Imported here rather than at module scope so the agent's dependency on
        a Socket.IO library is confined to the case where a backend is actually
        enabled. With `ROBOTX_SOCKET_ENABLED=0` -- the default -- `socketio` is
        never imported, and the standalone agent keeps exactly the runtime
        footprint it had before this integration existed.
        """

        if not self.settings.socket_enabled:
            return

        from robotx.communication.backend_link import BackendConfig, BackendLink

        try:
            cfg = BackendConfig.from_settings(self.settings)
        except Exception as e:
            # A bad ROBOTX_PROTOCOL_FILE must not take the robot down, but it
            # must be loud: the operator believes a contract is in force.
            log_event(
                logger,
                "backend.config_invalid",
                "backend link not started",
                level=logging.ERROR,
                error=repr(e),
            )
            self.state.update_communication(
                backend=BackendLinkStatus.DISABLED, backend_detail=f"configuration error: {e}"
            )
            return

        self.backend = BackendLink(
            cfg,
            self.state,
            self,
            agent_alive=self.is_alive,
            telemetry_sent=self.battery_telemetry_sent if self.demo_battery is not None else None,
            readiness=self.readiness_blockers,
            faults=self.fault_report,
        )
        await self.backend.start()

    async def stop(self) -> None:
        """Stop the loop and release every hardware resource."""

        if not self._started:
            return
        log_event(logger, "agent.stopping")
        self._running = False

        # Close the backend first: it reads state, and a link still publishing
        # while subsystems are being torn down would report a robot that is
        # halfway gone as if it were running.
        backend, self.backend = self.backend, None
        if backend is not None:
            await backend.stop()

        task, self._task = self._task, None
        if task is not None:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            except Exception:
                logger.exception("Agent loop raised during shutdown")

        # Release in reverse order of acquisition. The ESP32 link goes first:
        # the loop has stopped, so nothing can submit to it any more, and if it
        # was carrying motion it sends a final STOP before closing the port.
        if self.esp32 is not None:
            self.esp32.stop()
            self._update_esp32()
        if self.perception is not None:
            self.perception.stop()
        if self.gps is not None:
            self.gps.stop()
        if self.camera is not None:
            self.camera.stop()

        self.state.update_motion_intent(MotionIntent.stop("agent shutting down"))
        self.state.set_mode(OperatingMode.STOPPED)
        self._started = False
        log_event(logger, "agent.stopped")

    def _start_camera(self) -> None:
        if not self.settings.camera_enabled:
            log_event(logger, "camera.disabled", "disabled by configuration")
            return

        camera = CameraStream(CameraConfig.from_settings(self.settings))
        try:
            camera.start()
        except CameraError as e:
            # Not fatal: the agent must still run so the rest can be validated.
            log_event(
                logger,
                "camera.unavailable",
                "continuing without camera",
                level=logging.ERROR,
                error=str(e),
            )
            return
        self.camera = camera

    def _start_perception(self) -> None:
        self.perception = PerceptionPipeline.build(self.camera, self.settings)
        self.perception.start()

    def _start_gps(self) -> None:
        if not self.settings.gps_enabled:
            log_event(logger, "gps.disabled", "disabled by configuration")
            return
        if self.settings.gps_source == "esp32":
            # The receiver is on the ESP32's I2C bus; its frames arrive on the
            # ESP32 link, so there is no Pi serial port to open at all.
            if self.esp32 is None:
                self._gps_no_source = (
                    "ROBOTX_GPS_SOURCE=esp32 but the ESP32 link is not running "
                    "(ROBOTX_ESP32_ENABLED=0 or an invalid ESP32 configuration)"
                )
                log_event(logger, "gps.no_esp32_link",
                          "ROBOTX_GPS_SOURCE=esp32 but the ESP32 link is not running; no position source",
                          level=logging.ERROR)
                return
            self.gps = Esp32GpsReader(self.esp32, stale_after_s=self.settings.gps_stale_after_s)
            log_event(logger, "gps.source", "position from the ESP32's GPS frames", source="esp32")
            return
        if _same_device(self.settings.gps_port, self.settings.esp32_port):
            # The ESP32 UART is reserved whether or not the link is enabled:
            # the ESP32 is wired there either way. A GPS reader on it would
            # steal frames from the link (or from a bench test), and reconfigure
            # the line to the GPS baud rate.
            self._gps_port_conflict = self.settings.gps_port
            log_event(
                logger,
                "gps.port_conflict",
                "GPS not started: its port is reserved for the ESP32 UART",
                level=logging.CRITICAL,
                gps_port=self.settings.gps_port,
                esp32_port=self.settings.esp32_port,
                esp32_enabled=self.settings.esp32_enabled,
            )
            return
        self.gps = GPSReader(GPSConfig.from_settings(self.settings))
        self.gps.start()

    def _start_esp32(self) -> None:
        if not self.settings.esp32_enabled:
            log_event(logger, "esp32.disabled", "disabled by configuration")
            return
        try:
            cfg = Esp32Config.from_settings(self.settings)
            # The ESP32's front sensors are this rover's forward range, and the
            # Pi must stop for them before the ESP32 gate latches (RX-07).
            check_stop_distance(self.settings.safety_stop_distance_cm)
        except ValueError as e:
            # Same stance as a bad backend config: the robot still comes up,
            # with no motor link -- which is the safe state -- and says why.
            self._esp32_config_error = str(e)
            log_event(logger, "esp32.config_invalid", "ESP32 link not started",
                      level=logging.ERROR, error=str(e))
            return
        exe = self.settings.esp32_simulator_exe
        if exe:
            # Software-only: the ESP32 repository's host simulator stands in for
            # the UART. No serial device is opened and nothing can reach a motor.
            log_event(logger, "esp32.simulated",
                      "ESP32 HOST SIMULATOR in place of the UART -- no serial device is opened",
                      level=logging.WARNING, exe=exe)
            self.esp32 = Esp32Link(cfg, port_factory=lambda: HostSimulatorPort(exe, read_timeout_s=cfg.read_timeout_s))
        else:
            self.esp32 = Esp32Link(cfg)
        self.esp32.start()

    def _update_esp32(self) -> Optional[Esp32Status]:
        """Copy the link's view into state, and act on a detected ESP32 reboot.

        A reboot latches the existing emergency stop: whatever the ESP32 was
        doing before it restarted is unknown, so nothing moves again until an
        operator clears the latch (which also acknowledges the reboot).
        """

        if self.esp32 is None:
            return None
        status = self.esp32.status()
        self._last_controller = status.controller
        self.state.update_communication(
            esp32=status.link,
            esp32_detail=status.detail,
            esp32_since=status.since,
            esp32_last_rx_at=status.last_rx_at,
        )
        self.state.update_controller(status.controller, status.diag)
        if status.controller.reboot_count > self._esp32_reboots_handled:
            self._esp32_reboots_handled = status.controller.reboot_count
            self.emergency_stop(f"ESP32 rebooted: {status.detail}")
        return status

    def _wheels_driven(self) -> bool:
        """Whether the wheels were driven forward since the previous tick.

        The only displacement a GPS-track heading may be derived from: the
        previous tick's gated intent drove the wheels and -- with an ESP32
        attached -- its latest TELEMETRY reports forward output actually
        applied. A rover the controller held still has not moved, whatever
        its fixes wander; a heading taken from that wander would steer the
        next leg in a random direction.
        """

        if not self._intent_moving:
            return False
        if self.esp32 is None:
            return True
        controller = self._last_controller
        telemetry = controller.telemetry if controller is not None else None
        if telemetry is None:
            return False
        left, right = telemetry.left_applied, telemetry.right_applied
        return min(left, right) >= 0 and max(left, right) > 0

    def _front_range(self, status: Optional[Esp32Status]) -> Optional[RangeReading]:
        """The ESP32's front range for the safety gate, or None with no link attached.

        Judged on the same status this tick's link check used, with the link's
        own freshness limit (`robotx.esp32.front_range`).
        """

        if self.esp32 is None:
            return None
        if status is None:
            return RangeReading(RangeStatus.UNKNOWN)
        return front_range(
            status.controller.telemetry,
            status.controller.telemetry_age_s,
            link_up=status.link.is_up,
            stale_after_s=self.esp32.cfg.stale_after_s,
        )

    def _esp32_interrupted(self, status: Optional[Esp32Status]) -> bool:
        """Whether the link was lost or reopened since the previous tick.

        Read from the link's own counters, so a loss that was already
        recovered by the time this tick looked is still seen.
        """

        if status is None:
            return False
        counters = status.controller.counters
        seen = (counters.link_losses, counters.disconnects)
        interrupted = seen != self._esp32_interruptions_seen
        self._esp32_interruptions_seen = seen
        return interrupted

    # --- mission control (local, no backend required) ------------------------

    @property
    def mode(self) -> OperatingMode:
        """Current operating mode. Part of the command-target contract."""

        return self.state.mode

    def start_mission(self, waypoints: Sequence[LatLon]) -> None:
        """Load a locally supplied route and switch to AUTO.

        The bench path: a route from a test, a script or the local HTTP API,
        with no RobotX task behind it. `assign_mission` is the production path.
        Deliberately kept separate -- a locally driven route has no `taskId`,
        and giving it one would make the two indistinguishable in telemetry.
        """

        self._refuse_route_in_bench_wheel_demo("a local route")
        if len(waypoints) < 1:
            raise ValueError("a mission needs at least one waypoint")

        route = list(waypoints)
        self.navigator.set_route(route)
        self._mission_route = route
        self._begin_run()
        self.state.set_mode(OperatingMode.AUTO)
        log_event(logger, "mission.started", waypoints=len(route))

    def assign_mission(self, mission: Mission, *, custody_required: bool = False) -> MissionAssignment:
        """Accept a validated RobotX assignment and drive to the pickup.

        The mission arrives already validated -- `parse_task_assign` refused it
        at the wire boundary if it was not -- so what is left here is the
        Rover's own admissibility: it will not take on a delivery it cannot
        drive. Raises `MissionRejected`, which the caller reports back.

        Idempotent per `taskId`. RobotX redelivering the mission already in
        progress (a reconnect, a retry) continues from where the Rover is
        instead of restarting the leg, because restarting would send a Rover
        halfway to the drop back to waypoint zero of the pickup leg.
        """

        if self.settings.offer_acceptance_demo:
            raise MissionRejected(
                MissionRejectReason.OFFER_ACCEPTANCE_DEMO,
                "offer-acceptance demonstration: this Rover starts no missions (ROBOTX_OFFER_ACCEPTANCE_DEMO=1)",
                task_id=mission.task_id,
            )
        if self.settings.bench_wheel_demo:
            raise MissionRejected(
                MissionRejectReason.BENCH_WHEEL_DEMO,
                "bench wheel-test demonstration: this Rover starts no missions (ROBOTX_BENCH_WHEEL_DEMO=1)",
                task_id=mission.task_id,
            )
        if self.safety.estop_engaged:
            raise MissionRejected(
                MissionRejectReason.ESTOP_ENGAGED,
                "emergency stop is latched; clear it before assigning a task",
                task_id=mission.task_id,
            )
        if self.bench.active:
            raise MissionRejected(
                MissionRejectReason.MISSION_ACTIVE,
                "a bench motor pulse is running; no task is assigned until it has stopped",
                task_id=mission.task_id,
            )

        assignment = self.missions.assign(mission, custody_required=custody_required)
        active = assignment.active
        self.state.update_mission(active)

        if assignment.duplicate:
            # Already driving this task: no route reload, no mode change, no
            # reset of anything. Nothing to do is the correct response.
            return assignment

        self._mission_route = list(active.route)
        self._begin_run()
        self.state.clear_error()
        self.state.set_mode(OperatingMode.AUTO)
        log_event(
            logger,
            "mission.accepted",
            task_id=active.task_id,
            segment=active.segment.value,
            waypoints=len(active.route),
        )
        return assignment

    # --- engine offers and custody -------------------------------------------

    def accept_offer(self, mission: Mission) -> Optional[MissionAssignment]:
        """Act on an offer `assess_offer` accepted: what accepting means on this Rover.

        Normally that is driving it (`assign_mission`, custody required). In the
        offer-acceptance demonstration it is only the acceptance: nothing is
        loaded, the mode is unchanged, and with no mission held no HEARTBEAT
        renews the lease -- so the backend recovers the Leg instead of believing
        it is being delivered. Returns None then. The bench wheel-test
        demonstration accepts the same way, and only remembers the task so an
        operator may later bind one bench pulse to it (`start_bench_motion`):
        accepting itself never moves anything.
        """

        if not self._accepts_without_mission:
            return self.assign_mission(mission, custody_required=True)
        if self.settings.bench_wheel_demo:
            self._bench_wheel_tasks.add(mission.task_id)
        log_event(
            logger,
            "offer.accepted_without_motion",
            "bench wheel-test demonstration: accepted, no mission started, no motion until an operator "
            "requests a bench pulse" if self.settings.bench_wheel_demo
            else "offer-acceptance demonstration: accepted, no mission started",
            level=logging.WARNING,
            task_id=mission.task_id,
            pickup=mission.pickup,
            drop=mission.drop,
            mode=self.state.mode.value,
        )
        return None

    @property
    def _accepts_without_mission(self) -> bool:
        """Either demonstration: an accepted offer starts no mission."""

        return self.settings.offer_acceptance_demo or self.settings.bench_wheel_demo

    def _refuse_route_in_bench_wheel_demo(self, what: str) -> None:
        """Motion is enabled in the bench wheel test, so nothing may load a route to drive."""

        if self.settings.bench_wheel_demo:
            raise MissionRefused(f"bench wheel-test demonstration: {what} is never driven (ROBOTX_BENCH_WHEEL_DEMO=1)")

    def assess_offer(self, offer: Any) -> Any:
        """Can this Rover genuinely carry out `offer`? Its own answer, nothing more.

        Not assignment logic: no ranking, no comparison with other robots, no
        opinion on whether the offer is a good one. It checks only whether the
        physical means to execute it exist right now, in a fixed order, and
        rejects with the first thing missing. Nothing here is relaxed to make
        an offer acceptable -- an unassignable Rover is the correct outcome
        while the hardware evidence does not exist.
        """

        from robotx.communication.engine import OfferDecision, offer_to_mission

        if not offer.stops or any(not stop.path for stop in offer.stops):
            return OfferDecision.reject("NO_EXECUTABLE_PATH")
        try:
            mission = offer_to_mission(offer)
            check_route_consistency(mission, self.admission)
        except MissionRejected as e:
            log_event(logger, "offer.route_refused", e.detail, level=logging.WARNING,
                      task_id=e.task_id, reason=e.reason.value)
            return OfferDecision.reject(f"UNSUPPORTED_MISSION: {e.reason.value}")

        snapshot = self.state.snapshot()
        blockers = self.readiness_blockers(snapshot)
        # The Rover-level checks, in the same fixed order as always, with the
        # one offer-level check (a mission already held) in its old place:
        # after the e-stop and ERROR, before the motor link.
        if blockers and blockers[0] in ("ESTOP_LATCHED", "AGENT_ERROR"):
            return OfferDecision.reject(blockers[0])
        if self.bench.active:
            return OfferDecision.reject(BENCH_TEST_ACTIVE)
        if snapshot.mission is not None and snapshot.mission.is_active:
            return OfferDecision.reject("MISSION_IN_PROGRESS")
        if blockers:
            return OfferDecision.reject(blockers[0])
        # Not a readiness code (the backend's vocabulary has none for it), but a
        # Rover that cannot see cannot drive: the decision layer stops for
        # unusable perception on every tick, so accepting would hold a Leg the
        # Rover will sit on. The faults[] report names it beside the refusal.
        # Not in either demonstration: accepting drives nothing, so nothing needs to see
        # (a bench pulse checks perception itself, when one is requested).
        if not self._accepts_without_mission and not snapshot.perception.is_usable:
            return OfferDecision.reject(f"{PERCEPTION_UNAVAILABLE}: {snapshot.perception.status.value}")
        # Only now is there a measured, navigable position to judge it from
        # (and if somehow not, check_start_position refuses rather than guess).
        here = snapshot.position.lat_lon if snapshot.position is not None else None
        try:
            check_start_position(mission, here, self.admission)
        except MissionRejected as e:
            log_event(logger, "offer.route_refused", e.detail, level=logging.WARNING,
                      task_id=e.task_id, reason=e.reason.value)
            return OfferDecision.reject(f"UNSUPPORTED_MISSION: {e.reason.value}")
        return OfferDecision.accept(mission)

    def readiness_blockers(self, snapshot: Optional[Any] = None) -> List[str]:
        """Every reason this Rover would refuse any offer right now, in `assess_offer`'s order.

        The single source of both `assess_offer`'s Rover-level refusals and the
        TELEMETRY `readiness.blockers` (ROBOTX_PI_READINESS_HANDOFF P-1), so the
        backend is told exactly what an offer would be refused for -- before it
        sends one. Every applicable code, not only the first. `[]` only when
        every check positively passed; a check that cannot be evaluated raises,
        and the caller then reports nothing rather than `[]`.

        Offer-specific reasons (`MISSION_IN_PROGRESS`, `NO_EXECUTABLE_PATH`,
        `UNSUPPORTED_MISSION`) are not here: they are about one offer, not the
        Rover.
        """

        snapshot = self.state.snapshot() if snapshot is None else snapshot
        blockers: List[str] = []
        if self.safety.estop_engaged:
            blockers.append("ESTOP_LATCHED")
        if snapshot.mode is OperatingMode.ERROR:
            blockers.append("AGENT_ERROR")
        # Motor authority belongs to the ESP32. Without that link the Rover
        # cannot move at all, whatever it accepts. An UP link is not yet a
        # motor link: motion must be enabled on the Pi, and the ESP32's fresh
        # TELEMETRY must report its drive available and no controller safety
        # fault (safety_stop, front_valid=false, command_timeout) -- see
        # Esp32Link._motion_ready. `ROBOTX_ESP32_MOTION_ENABLED=false` is
        # therefore always NO_MOTOR_LINK -- except in the offer-acceptance
        # demonstration, which never drives and so needs only the UP link (the
        # ESP32 is still the source of the stop latch the backend's F7 reads).
        # The bench wheel test drives a pulse, so it keeps the full check.
        demo = self.settings.offer_acceptance_demo
        if not snapshot.communication.esp32.is_up or (
            not demo and self.esp32 is not None and not self.esp32.status().motion_ready
        ):
            blockers.append("NO_MOTOR_LINK")
        # A fix navigation will not drive on is no fix.
        if (
            snapshot.position is None
            or not snapshot.position.is_measured
            or not snapshot.gps.has_fix
            or not self.navigator.usable(snapshot.position)
        ):
            blockers.append("NO_POSITION_FIX")
        # No parcel changes hands on a mission that is never started.
        if not self._accepts_without_mission and not self.custody_sensing_available():
            blockers.append("NO_CUSTODY_SENSING")
        return blockers

    def active_faults(self, snapshot: Optional[Any] = None) -> List[Fault]:
        """Every fault active right now (robotx.diagnostics.faults), from one snapshot."""

        snapshot = self.state.snapshot() if snapshot is None else snapshot
        cfg = getattr(self.esp32, "cfg", None)
        controller = snapshot.controller
        if cfg is not None:
            motion_enabled = bool(cfg.motion_enabled)
        else:
            motion_enabled = bool(controller is not None and controller.motion_enabled)
        return active_faults(
            snapshot,
            estop_engaged=self.safety.estop_engaged,
            esp32_attached=self.esp32 is not None,
            esp32_config_error=self._esp32_config_error,
            motion_enabled=motion_enabled,
            controller_max_age_s=float(getattr(cfg, "stale_after_s", 1.0)),
            position_issue=self.navigator.position_issue(snapshot.position),
            bench_active=self.bench.active,
        )

    def fault_report(self, snapshot: Optional[Any] = None) -> List[Dict[str, Any]]:
        """TELEMETRY `faults[]`: the active set, each with the instant it began."""

        return self._fault_tracker.report(self.active_faults(snapshot))

    def start_bench_motion(self, *, speed: float, duration_s: float, source: str,
                           task_id: Optional[str] = None) -> Dict[str, Any]:
        """Start one bench pulse (robotx.control.bench_motion), or raise `BenchRefused`.

        Every precondition is the real one, checked now; none is relaxed for a
        bench. The pulse then runs on the agent's own tick, through the safety
        gate and the ESP32 link, and stops on its own.

        Normally a pulse refuses while any backend commitment is held. In the
        bench wheel-test demonstration it is the reverse: `task_id` is required
        and must be an offer this process accepted whose commitment is still
        held, over a connected backend link. The pulse is bound to it and ends
        when that stops being true (`_check_bench_binding`, `end_bench_task`).
        """

        def refuse(code: str, detail: str) -> None:
            log_event(logger, "bench.refused", detail, level=logging.WARNING, code=code, source=source)
            raise BenchRefused(code, detail)

        if self.bench.active:
            refuse("BENCH_RUNNING", "a bench pulse is already running")
        wait = self.bench.cooling_down()
        if wait > 0:
            refuse("COOLDOWN", f"the last pulse ended moments ago; wait {wait:.0f}s")
        if not (self.bench.cfg.min_speed <= speed <= self.bench.cfg.speed):
            refuse("BAD_SPEED", f"speed must be in [{self.bench.cfg.min_speed:.3f}, {self.bench.cfg.speed}]")
        if not (0.5 <= duration_s <= self.bench.cfg.duration_s):
            refuse("BAD_DURATION", f"duration_s must be in [0.5, {self.bench.cfg.duration_s}]")
        if self.safety.estop_engaged:
            refuse("ESTOP_LATCHED", "the emergency stop is latched")
        snapshot = self.state.snapshot()
        if snapshot.mode is not OperatingMode.IDLE:
            refuse("MODE_NOT_IDLE", f"mode is {snapshot.mode.value}; a bench pulse runs only from IDLE")
        if self.missions.has_active_mission:
            refuse("MISSION_HELD", "a mission is held")
        commitments = getattr(self.backend, "commitments", None)
        held = commitments.held() if commitments is not None else None
        commitment_id: Optional[str] = None
        if self.settings.bench_wheel_demo:
            if not task_id:
                refuse("TASK_REQUIRED", "the bench wheel test runs only for an accepted task: give its task_id")
            if task_id not in self._bench_wheel_tasks:
                refuse("TASK_NOT_ACCEPTED", f"task {task_id!r} was not accepted by this agent since it started")
            if held is None or held.task_id != task_id:
                refuse("TASK_NOT_HELD", f"the commitment for task {task_id!r} is no longer held "
                                        "(recalled, withdrawn or ended)")
            if not getattr(self.backend, "connected", False):
                refuse("BACKEND_NOT_CONNECTED", "the backend link is not connected and authenticated")
            assert held is not None
            commitment_id = held.commitment_id
        else:
            if task_id is not None:
                refuse("TASK_BINDING_UNAVAILABLE", "a bench pulse is bound to a task only with ROBOTX_BENCH_WHEEL_DEMO=1")
            if held is not None:
                refuse("COMMITMENT_HELD", "a backend commitment is held")
        if self.esp32 is None:
            refuse("NO_ESP32_LINK", "no ESP32 link is running")
        assert self.esp32 is not None
        if not self.esp32.cfg.motion_enabled:
            refuse("MOTION_DISABLED", "ROBOTX_ESP32_MOTION_ENABLED is off: the link writes no DRIVE")
        status = self.esp32.status()
        if not status.motion_ready:
            tel = status.controller.telemetry
            faults = ", ".join(tel.safety_faults) if tel is not None and tel.safety_faults else "none"
            refuse("CONTROLLER_NOT_READY",
                   f"ESP32 link {status.link.value} ({status.detail}); drive_available="
                   f"{None if tel is None else tel.motor_drive_available}; controller faults: {faults}; "
                   f"reboot_latched={status.controller.reboot_latched}")
        tel = status.controller.telemetry
        front = self._front_range(status)
        if front is None or front.status is not RangeStatus.VALID or front.obstacle_reported \
                or front.distance_cm is None or front.distance_cm <= self.safety.cfg.stop_distance_cm \
                or (tel is not None and tel.forward_blocked):
            refuse("FRONT_NOT_CLEAR",
                   f"the forward range must be valid and beyond {self.safety.cfg.stop_distance_cm:.0f} cm "
                   f"(status={None if front is None else front.status.value}, "
                   f"distance={None if front is None else front.distance_cm}, "
                   f"forward_blocked={None if tel is None else tel.forward_blocked})")
        if not snapshot.perception.is_usable:
            refuse("PERCEPTION_UNUSABLE", f"perception is {snapshot.perception.status.value}")
        return self.bench.start(speed=speed, duration_s=duration_s, source=source,
                                task_id=task_id, commitment_id=commitment_id)

    def bench_wheel_status(self) -> Dict[str, Any]:
        """The bench wheel-test demonstration as the bench route reports it."""

        commitments = getattr(self.backend, "commitments", None)
        held = commitments.held() if commitments is not None else None
        return {
            "enabled": bool(self.settings.bench_wheel_demo),
            "backend_connected": bool(getattr(self.backend, "connected", False)),
            "accepted_tasks": sorted(t for t in self._bench_wheel_tasks if t),
            "held_task_id": held.task_id if held is not None else None,
            "held_commitment_id": held.commitment_id if held is not None else None,
        }

    def end_bench_task(self, commitment_id: Optional[str], reason: str) -> bool:
        """The backend ended a commitment (RECALL / WITHDRAW / ABORT_MISSION): stop its bench pulse.

        Called by the backend link before it tombstones the commitment, whether
        or not a mission is held. Returns whether a running pulse was ended. The
        task also stops being eligible for another pulse.
        """

        if commitment_id is None:
            return False
        commitments = getattr(self.backend, "commitments", None)
        record = commitments.get(commitment_id) if commitments is not None else None
        if record is not None and record.task_id:
            self._bench_wheel_tasks.discard(record.task_id)
        if self.bench.driving and self.bench.bound_commitment == commitment_id:
            self.bench.abort("TASK_CANCELLED", f"task cancelled: {reason}")
            return True
        return False

    def _check_bench_binding(self) -> None:
        """End a task-bound pulse whose task is no longer held, or whose backend link dropped.

        The per-tick backstop for `end_bench_task`: a cancellation that never
        arrives as a terminating envelope still ends the pulse, here or at its
        own deadline -- and the ESP32 watchdog stops the wheels if the Pi
        itself goes silent.
        """

        bound = self.bench.bound_commitment
        if not self.bench.driving or bound is None:
            return
        commitments = getattr(self.backend, "commitments", None)
        held = commitments.held() if commitments is not None else None
        if held is None or held.commitment_id != bound:
            self.bench.abort("TASK_RELEASED", "the accepted task's commitment is no longer held")
        elif not getattr(self.backend, "connected", False):
            self.bench.abort("BACKEND_LINK_LOST", "the backend link dropped during the pulse")

    def custody_sensing_available(self) -> bool:
        """Whether a parcel handover on this Rover can be truthfully reported.

        There is no load sensor and no compartment switch. The one truthful
        source is a person at the stop confirming it, and only when the
        deployment says that confirmation path is staffed
        (`ROBOTX_CUSTODY_CONFIRMATION=operator`). Otherwise the Rover must not
        accept a mission that requires ACQUIRED / RELEASED.
        """

        return self.settings.custody_confirmation == "operator"

    def record_custody(self, kind: str, *, source: str) -> Any:
        """Record a genuine handover. Raises `MissionRefused` when not at that stop.

        The only way custody enters mission state, and today nothing on this
        Rover calls it: see `custody_sensing_available`.
        """

        active = self.missions.record_custody(kind, source=source)
        self.state.update_mission(active)
        return active

    def _begin_run(self) -> None:
        """Shared setup for starting to drive, whatever supplied the route."""

        # Remember where this run began, so a later RETURN has a real place to
        # go back to instead of an invented one. Only recorded when there is an
        # actual fix; a mission started without GPS leaves this None and RETURN
        # will correctly refuse.
        snapshot = self.state.snapshot()
        if snapshot.position is not None and snapshot.gps.status is GPSStatus.FIX:
            self._mission_origin = snapshot.position.lat_lon

        # Start each run from a clean pose. Drift accumulated while idling --
        # or during a previous mission -- describes nothing about this one, and
        # carrying it over would start the rover already convinced it is
        # somewhere it is not.
        if self.dead_reckoner is not None:
            self.dead_reckoner.reset()
        # Likewise the GPS heading: it says where the robot last moved, and since
        # then it may have been turned or carried. A run starts with no heading
        # and establishes one by moving.
        self.position_estimator.reset()
        # An obstacle episode, or a stall, belongs to the run it happened in.
        self.obstacles.reset()
        self.progress.reset()
        self.applied_motion.reset()

    def stop_mission(self, reason: str = "operator stop") -> None:
        """Halt: clear the route, drop to STOPPED, and publish a stop intent."""

        self.bench.abort("OPERATOR_STOP", f"stop: {reason}")
        self.navigator.clear_route()
        self._mission_route = []
        # Mark the task abandoned rather than leaving it mid-leg. A mission
        # still in TO_DROP with no route would otherwise be "arrived" by
        # whatever route came next, and reported as a delivery that never
        # happened.
        self.missions.abandon(reason)
        self.state.update_mission(self.missions.active)
        self.obstacles.reset()
        self.progress.reset()
        self.state.set_mode(OperatingMode.STOPPED)
        self.state.update_motion_intent(MotionIntent.stop(reason))
        log_event(logger, "mission.stopped", reason=reason)

    def pause_mission(self, reason: str = "operator pause") -> None:
        """Suspend the mission, keeping the route so it can be resumed.

        The route stays loaded in the navigator. `OperatingMode.PAUSED` is not
        `mission_active`, so the decision layer issues a hold on the very next
        tick -- pausing does not depend on anything downstream noticing a flag.
        """

        self.bench.abort("OPERATOR_PAUSE", f"pause: {reason}")
        if self.safety.estop_engaged and self.state.mode is OperatingMode.STOPPED:
            # Already held harder than a pause holds it. Demoting STOPPED to
            # PAUSED here would let a RESUME reach AUTO with the latch still on.
            self.state.update_motion_intent(MotionIntent.stop(reason))
            log_event(logger, "mission.pause_while_estopped",
                      "emergency stop is latched; the mission stays STOPPED", reason=reason)
            return
        self.state.set_mode(OperatingMode.PAUSED)
        self.state.update_motion_intent(MotionIntent.stop(reason))
        log_event(logger, "mission.paused", reason=reason, waypoints=len(self._mission_route))

    def resume_mission(self, reason: str = "operator resume") -> None:
        """Return a paused mission to AUTO.

        Refuses when there is no route left to resume rather than dropping into
        AUTO with an empty navigator, which would report NAVIGATING with
        nowhere to go. The caller turns this into a FAILED acknowledgement.

        Resuming authorizes nothing by itself. Each tick still needs a usable
        position (NAVIGATING), usable perception, a clear corridor -- sustained
        for `obstacle_clear_hold_s` if an obstacle stopped the rover -- the
        safety gate, and a motion-ready ESP32 link; anything missing holds the
        rover, and a link that cannot carry motion pauses the mission again.
        """

        self._refuse_route_in_bench_wheel_demo("a resumed route")
        if self.safety.estop_engaged:
            # Never AUTO under a latch: clearing it is a separate operator act.
            raise MissionRefused("emergency stop is latched; clear it before resuming")
        # The mission manager is asked first: on the drop leg it holds the
        # route being driven, while `_mission_route` may still describe the
        # pickup leg that finished before the pause.
        route = list(self.missions.current_route()) or self._mission_route
        if not route:
            raise MissionRefused("no route is loaded; nothing to resume")
        self._mission_route = route
        if self.navigator.planner.route_length() == 0:
            self.navigator.set_route(route)
        self.obstacles.restart()
        # An operator RESUME gives navigation a fresh allowance; the reasons it
        # stopped still have to clear before anything moves.
        self.progress.reset()
        self.applied_motion.reset()
        active = self.missions.active
        if active is not None and active.interrupted:
            # This RESUME is the explicit decision the interruption waited for.
            self.state.update_mission(self.missions.clear_interruption())
        self.state.clear_error()
        self.state.set_mode(OperatingMode.AUTO)
        log_event(logger, "mission.resumed", reason=reason)

    def return_to_base(self, reason: str = "operator return") -> None:
        """Route back to where the current mission started.

        There is no depot coordinate anywhere in this robot's configuration or
        hardware, so "base" can only mean the recorded mission origin -- or an
        explicitly configured home. With neither, this refuses. Inventing a
        destination and driving to it is the single worst thing this method
        could do.
        """

        self._refuse_route_in_bench_wheel_demo("a return to base")
        if self.safety.estop_engaged:
            raise MissionRefused("emergency stop is latched; clear it before returning to base")
        home = self._home_position()
        if home is None:
            raise MissionRefused(
                "no home position: none configured (ROBOTX_HOME_LAT/ROBOTX_HOME_LON) "
                "and no GPS fix was recorded when the mission started"
            )
        # Going home replaces the mission's route, so the mission is over. Left
        # active it would be marked complete the moment the Rover reached home,
        # reporting a delivery to the wrong coordinate entirely.
        self.missions.abandon(f"return to base: {reason}")
        self.state.update_mission(self.missions.active)

        self.navigator.set_route([home])
        self._mission_route = [home]
        self.state.set_mode(OperatingMode.AUTO)
        log_event(logger, "mission.returning", reason=reason)

    def _home_position(self) -> Optional[LatLon]:
        """Configured home, else the position this mission started from."""

        lat = self.settings.home_lat
        lon = self.settings.home_lon
        if lat is not None and lon is not None:
            return (float(lat), float(lon))
        return self._mission_origin

    # --- emergency stop ------------------------------------------------------

    def emergency_stop(self, reason: str = "emergency stop") -> None:
        """Latch the safety gate shut and halt immediately.

        Distinct from `stop_mission`, and both are needed. `stop_mission` is a
        mission-level decision: the run is over, and a later mission may start
        normally. This is a safety-level one: whatever the mission says, no
        motion intent leaves this Pi until a human clears it. The latch lives
        in the gate rather than in the operating mode precisely so that no
        mode change -- including a backend RESUME -- can lift it as a
        side effect.

        The mode is also driven to STOPPED so the decision layer stops
        proposing motion at all, rather than proposing intents that the gate
        then silently refuses every tick.
        """

        self.safety.engage_estop(reason)
        self.bench.abort("ESTOP", f"emergency stop: {reason}")
        self.state.set_mode(OperatingMode.STOPPED, error=f"emergency stop: {reason}")
        self.state.update_motion_intent(MotionIntent.stop(f"safety: {reason}"))
        # A mission caught by the stop is kept -- the operator decides its fate
        # -- but marked interrupted, so neither its lease nor a clear of the
        # latch can carry it on as though nothing happened.
        if self.missions.has_active_mission:
            self.state.update_mission(self.missions.interrupt(f"emergency stop: {reason}"))
        log_event(logger, "mission.emergency_stop", reason, level=logging.CRITICAL)

    def clear_emergency_stop(self, reason: str = "operator clear") -> bool:
        """Release the latch. Returns whether one was actually engaged.

        Deliberately does not resume anything. Clearing the latch only makes
        motion *possible* again; starting a mission stays a separate, explicit
        act, so that the operator who clears a fault is never the reason the
        robot began to drive.
        """

        released = self.safety.clear_estop(reason)
        if released:
            self.state.clear_error()
        # The same operator act acknowledges a latched ESP32 reboot. The link
        # still has to re-prove itself with a fresh PING before carrying motion.
        if self.esp32 is not None:
            self.esp32.acknowledge_reboot()
        # Recovery: a mission the stop interrupted goes STOPPED -> PAUSED, never
        # AUTO. Its route is kept, its lease may be renewed again, and only an
        # explicit RESUME (or a STOP that abandons it) decides what happens next.
        if released and self.missions.has_active_mission and self.state.mode is OperatingMode.STOPPED:
            why = "emergency stop cleared; the interrupted mission is PAUSED until an explicit RESUME"
            self.state.set_mode(OperatingMode.PAUSED, error=f"recovery: {why}")
            self.state.update_motion_intent(MotionIntent.stop(why))
            log_event(logger, "mission.recovered_paused", why, level=logging.WARNING,
                      task_id=self.missions.active.task_id if self.missions.active else None)
        return released

    def mission_lease_renewable(self) -> bool:
        """Whether a held mission's lease may be renewed right now.

        Only while the Rover could carry the mission on without an operator
        first clearing something: mode AUTO or PAUSED (a pause ends with a
        RESUME), and no emergency stop latched. STOPPED under a latch, or
        ERROR, is a Rover that cannot deliver; renewing would tell the backend
        it is still on the job while the lease's whole purpose is to let the
        backend recover a Leg from a Rover that is not.
        """

        return not self.safety.estop_engaged and self.state.mode in (OperatingMode.AUTO, OperatingMode.PAUSED)

    @property
    def emergency_stopped(self) -> bool:
        return self.safety.estop_engaged

    # --- controller reset (operator) -----------------------------------------

    _RESET_MODES = (OperatingMode.PAUSED, OperatingMode.IDLE, OperatingMode.STOPPED)

    def reset_controller(self, reason: str = "operator reset") -> None:
        """Ask the ESP32 to clear its safety-stop and command-timeout latches.

        An explicit operator action, never taken automatically. RESET stops the
        ESP32's motors and clears its latches; it is not a reboot, so it is
        entirely separate from the reboot -> emergency-stop path. It changes
        nothing on the Pi: not the mode, the route, the mission, the e-stop.
        A mission paused by the fault stays PAUSED until an explicit RESUME,
        and a fault still present simply pauses it again.

        Refused (`MissionRefused`) while AUTO -- the next control tick could
        send DRIVE straight after the RESET -- and whenever the link refuses
        it (`Esp32Link.request_reset`). A refused request sends nothing.
        """

        mode = self.state.mode
        refusal: Optional[str] = None
        if mode not in self._RESET_MODES:
            refusal = f"mission is {mode.value}; pause it before resetting the ESP32 controller"
        elif self.esp32 is None:
            refusal = "no ESP32 link is running"
        else:
            refusal = self.esp32.request_reset()
        if refusal is not None:
            log_event(logger, "controller.reset_refused", refusal, level=logging.WARNING,
                      reason=reason, mode=mode.value)
            raise MissionRefused(f"controller RESET refused: {refusal}")
        log_event(logger, "controller.reset_requested", "RESET will be sent to the ESP32 once",
                  level=logging.WARNING, reason=reason, mode=mode.value)

    def resume_idle(self) -> None:
        """Leave STOPPED/ERROR and go back to IDLE, clearing the last error.

        Refused (`MissionRefused`) while a mission is still held: IDLE would
        orphan it, still active, with a route loaded and nothing ever driving or
        ending it. Stop it (which abandons it) or resume it instead.
        """

        active = self.missions.active
        if active is not None and active.is_active:
            raise MissionRefused(
                f"task {active.task_id} is still held ({active.status.value}); "
                "stop it (abandons it) or resume it before going IDLE"
            )
        self.state.clear_error()
        self.state.set_mode(OperatingMode.IDLE)
        self.state.update_motion_intent(MotionIntent.hold("idle"))
        log_event(logger, "mission.idle")

    def add_telemetry_sink(self, sink: TelemetrySink) -> None:
        """Register a local consumer of telemetry payloads.

        Kept deliberately generic: this is where a future backend transport
        would attach, without the agent knowing what a backend is.
        """

        self._telemetry_sinks.append(sink)

    @property
    def last_telemetry(self) -> Dict[str, Any]:
        return dict(self._last_telemetry)

    def is_alive(self, *, now: Optional[float] = None) -> bool:
        """Whether the agent loop has completed a tick recently.

        This, not an open socket, is what the backend heartbeat asserts. The
        link runs on its own task and would keep beating through a hung or
        crash-looping agent; gating it here lets the backend's freshness budget
        expire exactly when the robot stops sensing and deciding.
        """

        if self._last_tick_at is None:
            return False
        now = time.monotonic() if now is None else now
        return now - self._last_tick_at <= AGENT_STALL_S

    # --- loop ----------------------------------------------------------------

    async def _loop(self) -> None:
        period = 1.0 / max(0.5, float(self.settings.agent_hz))

        while self._running:
            started = time.monotonic()
            try:
                self.tick()
                self._last_tick_at = time.monotonic()
            except asyncio.CancelledError:
                raise
            except Exception as e:
                # The loop must not die; the robot must stop.
                log_event(
                    logger,
                    "agent.tick_failed",
                    "unhandled error in agent tick",
                    level=logging.ERROR,
                    exc_info=True,
                )
                self.state.set_mode(OperatingMode.ERROR, error=repr(e))
                self.state.update_motion_intent(MotionIntent.stop(f"agent error: {e}"))

            elapsed = time.monotonic() - started
            await asyncio.sleep(max(0.0, period - elapsed))

    def tick(self) -> RobotSnapshot:
        """Run one full sense -> decide -> publish cycle."""

        now = time.monotonic()

        # 1. GPS -> position
        gps_reading = (
            self.gps.get_reading()
            if self.gps is not None
            else _gps_unavailable()
        )
        position = self.position_estimator.update(gps_reading, driven=self._wheels_driven())

        # A real fix always wins. Dead reckoning is only ever consulted when
        # GPS produced nothing, so attaching a receiver is all it takes to stop
        # using the integrated estimate -- there is no mode to remember to
        # switch, and no way for the two to be blended into a position that is
        # neither measured nor honestly labelled.
        if position is None and self.dead_reckoner is not None:
            position = self.dead_reckoner.position(self.local_frame)

        self.state.update_gps(gps_reading, position)

        # 2. Power. The Pi is the producer only because no battery hardware
        #    exists; when it does it will be on the ESP32 and its parser will
        #    write this same field. Either way consumers read it from state.
        #    With the DEMO simulation on, it is the producer instead -- once per
        #    tick, so no interval is drained twice.
        if self.demo_battery is not None:
            self.state.update_power(PowerState.from_status_dict(self._update_demo_battery(position)))
        else:
            self.state.update_power(PowerState.from_status_dict(battery_status()))

        # 2b. ESP32 link: what it reports goes into state before anything
        #     decides, and a link that can no longer carry motion pauses a
        #     running mission rather than letting it continue unexecuted. That
        #     includes an UP link whose fresh TELEMETRY reports a controller
        #     safety fault (Esp32Link._motion_ready). The pause is not undone
        #     when the fault clears: resuming stays an explicit RESUME.
        #
        #     Sampling `motion_ready` once a tick is not enough on its own: the
        #     link can lose its proof and re-prove itself (RX-24) between two
        #     ticks, and a mission that never saw it would drive on. So a new
        #     link loss or disconnect since the last tick pauses too, whatever
        #     the link says now.
        link = self.esp32
        esp32 = self._update_esp32()
        interrupted = self._esp32_interrupted(esp32)
        applied_fault: Optional[str] = None
        if (
            link is not None
            and esp32 is not None
            and link.cfg.motion_enabled
            and self.state.mode.mission_active
            and (not esp32.motion_ready or interrupted)
        ):
            why = f"{esp32.link.value} ({esp32.detail})"
            telemetry = esp32.controller.telemetry
            if esp32.link.is_up and telemetry is not None and telemetry.safety_faults:
                why += f"; controller reports {', '.join(telemetry.safety_faults)}"
            if interrupted and esp32.motion_ready:
                why = "the link was lost and re-proven since the last tick; now " + why
            self.pause_mission(f"ESP32 cannot carry motion: {why}")
            # Recorded like the obstacle and navigation pauses, under its own
            # prefix, so /state and health say which subsystem stopped the run.
            self.state.set_mode(OperatingMode.PAUSED, error=f"controller: ESP32 cannot carry motion: {why}")

        # 2b'. Applied motion: what the ESP32 reports it did with the DRIVEs
        #      the Pi has been sending (ACK GATED/REJECTED, applied outputs).
        #      A bounded disagreement pauses the mission with the reason
        #      (robotx.control.applied_motion); a fault never resumes anything.
        if link is not None and esp32 is not None and link.cfg.motion_enabled:
            applied = self.applied_motion.update(
                commanding=(self.state.mode.mission_active or self.bench.driving) and self._intent_moving,
                controller=esp32.controller,
                stale_after_s=link.cfg.stale_after_s,
            )
            if applied.fault and self.bench.driving:
                applied_fault = f"controller: {applied.reason}"   # judged by the bench run below
            elif applied.fault:
                why = f"controller: {applied.reason}"
                self.pause_mission(why)
                self.state.set_mode(OperatingMode.PAUSED, error=why)

        # 2c. Backend link (Y4): its loss policy runs on this loop's own cadence,
        #     before anything decides, so neither a reconnect backoff nor a
        #     connect or AUTH wait can hold back the pause -- and a mission it
        #     pauses produces the hold on this same tick.
        if self.backend is not None:
            self.backend.apply_loss_policy()
        self._check_bench_binding()

        # 3. Navigation
        navigation = self.navigator.update(position)
        self.state.update_navigation(navigation)

        # 3b. Mission progress. Driven entirely by the navigation verdict above
        #     -- which was computed from the localization pose against the
        #     waypoints RobotX supplied -- so there is exactly one authority on
        #     whether a leg is finished, and it is not this one.
        self._advance_mission(
            navigation, position_measured=position is not None and position.is_measured
        )

        # 4. Perception (already computed on its own thread)
        perception = (
            self.perception.latest()
            if self.perception is not None
            else PerceptionResult.unavailable(PerceptionStatus.DISABLED)
        )
        self.state.update_perception(perception)

        # 5. Decision -> proposed motion intent (published, never actuated here)
        mode = self.state.mode
        proposed = self.decision.decide(
            mission_active=mode.mission_active,
            navigation=navigation,
            perception=perception,
        )
        # The bench pulse replaces the proposal -- never the gate. It is judged
        # by the same SafetyGate as a mission (perception, range, e-stop, speed),
        # with no obstacle-recovery hold: a stop ends it, and nothing resumes it.
        bench_active = self.bench.active
        if bench_active:
            proposed = self.decision.screen(perception) or self.bench.intent()
        gate_active = mode.mission_active or bench_active

        # 5b. Safety gate has the last word. What lands in state is the *gated*
        #     intent, because state is what telemetry and (once it exists) the
        #     ESP32 link read -- an intent the gate refused must never be
        #     visible anywhere as though it were going to happen.
        #
        #     The forward range is the ESP32's front ultrasonic pair (RX-07),
        #     read from the TELEMETRY sampled in step 2b. With a link attached
        #     it is required: stale, invalid or missing never reads as clear.
        #     Without one (a bench Pi) no range is passed, and
        #     `SafetyConfig.require_range_sensor` decides what that means.
        front = self._front_range(esp32)
        range_expected = self.esp32 is not None
        safety = self.safety.evaluate(
            proposed,
            mission_active=gate_active,
            perception=perception,
            obstacle=front,
            range_expected=range_expected,
        )

        # 5b'. Obstacle recovery (RX-08). An obstacle stop -- the decision
        #      layer's (camera: obstacle or person) or the gate's (measured
        #      range) -- opens a blocked episode; motion resumes only after
        #      `obstacle_clear_hold_s` of clear, usable perception. A held tick
        #      is re-evaluated as a STOP, so the gate keeps the last word. An
        #      episode that lasts `obstacle_assist_after_s` pauses the mission
        #      for an operator. A timer never clears an obstacle.
        recovery = self.obstacles.update(
            active=mode.mission_active,
            obstacle=mode.mission_active and _is_obstacle_stop(safety.intent),
            perception_usable=perception.is_usable,
            range_usable=not range_expected or (front is not None and front.status is RangeStatus.VALID),
        )
        if recovery.hold and not safety.intent.is_stop:
            safety = self.safety.evaluate(
                MotionIntent.stop(recovery.reason),
                mission_active=mode.mission_active,
                perception=perception,
                obstacle=front,
                range_expected=range_expected,
            )
        if recovery.assist:
            why = f"blocked for {recovery.blocked_for_s:.0f}s: operator assistance needed"
            self.pause_mission(f"obstacle: {why}")
            self.state.set_mode(OperatingMode.PAUSED, error=f"obstacle: {why}")

        # 5b''. Bounded navigation stops: a hold for no usable position or off
        #       route, or a drive that gets no closer, ends in a pause with its
        #       reason instead of going on silently (robotx.navigation.progress).
        #       A pause here turns this tick's intent into a stop before the
        #       ESP32 link sees it.
        stall = self.progress.update(
            active=self.state.mode.mission_active,
            navigation=navigation,
            moving=not safety.intent.is_stop,
            sensing_issue=_sensing_issue(perception, front, range_expected=range_expected),
        )
        if stall.assist:
            self.pause_mission(stall.reason)
            self.state.set_mode(OperatingMode.PAUSED, error=stall.reason)
            if not safety.intent.is_stop:
                safety = self.safety.evaluate(
                    MotionIntent.stop(stall.reason),
                    mission_active=False,
                    perception=perception,
                )
        if bench_active:
            outcome = self.bench.observe(
                gated=safety.intent,
                controller=esp32.controller if esp32 is not None else None,
                motion_ready=esp32 is not None and esp32.motion_ready,
                interrupted=interrupted,
                estop=self.safety.estop_engaged,
                mode_idle=self.state.mode is OperatingMode.IDLE,
                mission_held=self.missions.has_active_mission,
                applied_fault=applied_fault,
            )
            if not self.bench.driving and not safety.intent.is_stop:
                # Ended on this tick: the STOP is what the link writes next.
                safety = self.safety.evaluate(MotionIntent.stop("bench test: pulse ended"),
                                              mission_active=False, perception=perception)
            if outcome is BenchOutcome.STOP_UNCONFIRMED:
                self.emergency_stop("bench test: the ESP32 never confirmed the STOP")
            if outcome is not None:
                # A pulse on blocks moved no rover: no heading, no pose from it.
                self.position_estimator.reset()
                self.applied_motion.reset()
        intent = safety.intent
        self._intent_moving = not intent.is_stop
        self.state.update_safety(safety)
        self.state.update_motion_intent(intent)

        # 5c. The gate's decision -- never the proposal -- is what the ESP32
        #     link may carry. It sends nothing unless motion is enabled.
        if self.esp32 is not None:
            self.esp32.submit(safety)

        # Close the dead-reckoning loop with the *gated* intent, never the
        # proposed one. Only what survives the safety gate can reach the
        # motors, so only that describes how the rover actually moved -- a
        # reckoner fed the pre-gate intent would keep integrating motion for a
        # rover that was vetoed into standing still, and walk its estimate off
        # into a place the rover has never been.
        if self.dead_reckoner is not None:
            self.dead_reckoner.integrate(linear=intent.linear, angular=intent.angular)

        # The same gated intent tells the GPS heading when it has gone stale:
        # whenever the two sides differ the rover turns, and the held heading --
        # the direction it last moved -- no longer says where it faces. The next
        # one comes from straight driving after the turn. Standing still does not.
        if intent.turns:
            self.position_estimator.invalidate_heading()

        # Obstacle stops are no longer fed into the route planner's blocked
        # counter: three ticks of one brief obstacle made REROUTE_NEEDED
        # permanent (RX-08), and nothing on this rover can reroute. Step 5b'
        # handles them.

        # 6. Health, evaluated from a single consistent snapshot
        if now - self._last_health_t >= self.settings.health_interval_s:
            components = self._component_health(self.state.snapshot())
            self.state.update_health(self.health.evaluate(components))
            self._last_health_t = now

        # 7. Telemetry (local)
        snapshot = self.state.snapshot()
        if now - self._last_telemetry_t >= self.settings.telemetry_interval_s:
            self._publish_telemetry(snapshot)
            self._last_telemetry_t = now

        return snapshot

    def _update_demo_battery(self, position: Optional[Any]) -> Dict[str, Any]:
        """One step of the DEMO battery simulation, from this tick's state.

        Working = driving a mission (AUTO). Idle = mode IDLE with no mission
        held: the backend simulator's own condition for starting a charge.
        Only a measured fix counts toward distance; dead reckoning never does.
        """

        assert self.demo_battery is not None
        mode = self.state.mode
        fix = (
            (position.latitude, position.longitude, position.timestamp)
            if position is not None and position.is_measured
            else None
        )
        return self.demo_battery.update(
            working=mode is OperatingMode.AUTO,
            idle=mode is OperatingMode.IDLE and not self.missions.has_active_mission,
            fix=fix,
        )

    def _advance_mission(self, navigation: NavigationState, *, position_measured: bool) -> None:
        """Step the assigned mission once, and react to what it did.

        On a leg change the navigator has just been given the next route, so
        the navigation state computed earlier in this tick describes the leg
        that finished. Nothing acts on it: the decision layer sees ARRIVED and
        holds for one tick, and the next tick navigates the new leg. A tick of
        standing still at a pickup is the correct behaviour anyway.
        """

        update = self.missions.update(navigation, position_measured=position_measured)
        self.state.update_mission(update.active)

        if update.segment_changed:
            # Keep the retained route in step with the leg now being driven, so
            # a PAUSE/RESUME across the transition resumes the right one.
            self._mission_route = list(self.missions.current_route())

        if update.completed and update.active is not None:
            self._finish_mission(update.active)

    def _finish_mission(self, active: ActiveMission) -> None:
        """The drop was reached: stand down, and leave the task reportable.

        The mission is *not* cleared from state. `TASK_COMPLETE` is emitted by
        the backend link from this snapshot, and a Rover that forgot the task
        the instant it finished would have nothing left to report -- least of
        all across a reconnect, which is exactly when the report is at risk.
        """

        self._mission_route = []
        self.state.set_mode(OperatingMode.IDLE)
        self.state.update_motion_intent(MotionIntent.hold("mission complete"))
        log_event(
            logger,
            "mission.complete",
            "delivery finished; standing by",
            task_id=active.task_id,
        )

    def _publish_telemetry(self, snapshot: RobotSnapshot) -> None:
        payload = build_telemetry(snapshot)
        self._last_telemetry = payload
        for sink in self._telemetry_sinks:
            try:
                sink(payload)
            except Exception:
                logger.exception("Telemetry sink raised; continuing")

    def _component_health(self, snapshot: RobotSnapshot) -> Dict[str, ComponentHealth]:
        """Per-subsystem health, all derived from one consistent snapshot.

        Taking a single snapshot matters: evaluating each component against its
        own snapshot could mix states from different instants and produce a
        health report that never actually existed.
        """

        return {
            "camera": self._camera_health(),
            "perception": self._perception_health(snapshot),
            "gps": self._gps_health(snapshot),
            "navigation": self._navigation_health(snapshot),
            "esp32": self._esp32_health(snapshot),
            "backend": self._backend_health(snapshot),
        }

    def _camera_health(self) -> ComponentHealth:
        if self.camera is None:
            if not self.settings.camera_enabled:
                return ComponentHealth(
                    "camera", HealthStatus.UNKNOWN, "disabled by configuration"
                )
            return ComponentHealth("camera", HealthStatus.FAILED, "camera did not start")

        # `.get` rather than `[]`: an unrecognized status must degrade the
        # report, never raise and take the whole agent tick down with it.
        status, detail = _CAMERA_HEALTH.get(
            self.camera.get_status(), (HealthStatus.UNKNOWN, "unrecognized camera status")
        )
        return ComponentHealth("camera", status, detail)

    def _perception_health(self, snapshot: RobotSnapshot) -> ComponentHealth:
        result = snapshot.perception
        status, detail = _PERCEPTION_HEALTH.get(
            result.status, (HealthStatus.UNKNOWN, "unrecognized perception status")
        )
        return ComponentHealth("perception", status, result.error or detail)

    def _gps_health(self, snapshot: RobotSnapshot) -> ComponentHealth:
        if self.gps is None and self._gps_port_conflict is not None:
            return ComponentHealth(
                "gps",
                HealthStatus.FAILED,
                f"not started: {self._gps_port_conflict} is reserved for the ESP32 UART "
                "(set ROBOTX_GPS_PORT to the receiver's own port, or ROBOTX_GPS_ENABLED=0)",
            )
        if self.gps is None and self._gps_no_source is not None:
            return ComponentHealth("gps", HealthStatus.FAILED, self._gps_no_source)
        if self.gps is None:
            return ComponentHealth(
                "gps", HealthStatus.UNKNOWN, "disabled by configuration"
            )

        reading = snapshot.gps
        status, detail = _GPS_HEALTH.get(
            reading.status, (HealthStatus.UNKNOWN, "unrecognized GPS status")
        )
        if reading.status is GPSStatus.FIX and snapshot.position is not None:
            issue = self.navigator.position_issue(snapshot.position)
            if issue is not None:
                return ComponentHealth("gps", HealthStatus.DEGRADED, issue)
        return ComponentHealth("gps", status, reading.error or detail)

    def _navigation_health(self, snapshot: RobotSnapshot) -> ComponentHealth:
        state = snapshot.navigation
        if state.status is NavigationStatus.REROUTE_NEEDED:
            return ComponentHealth("navigation", HealthStatus.DEGRADED, "reroute needed")
        if state.status is NavigationStatus.NO_POSITION:
            return ComponentHealth(
                "navigation",
                HealthStatus.DEGRADED,
                state.position_issue or "route loaded but no position",
            )
        return ComponentHealth("navigation", HealthStatus.HEALTHY, state.status.value)

    def _esp32_health(self, snapshot: RobotSnapshot) -> ComponentHealth:
        """Health of the link to the robot's own motor/sensor controller.

        Judged entirely on its own. A backend outage says nothing about this
        link, and this link's state must never be influenced by whether a
        remote platform happens to be reachable.

        `NOT_IMPLEMENTED` and `DISABLED` are UNKNOWN rather than FAILED: a
        rover with no ESP32 link configured would otherwise be permanently
        unhealthy for a component it does not have.

        An UP link is only HEALTHY when the ESP32 itself reports nothing
        wrong. What it reports -- drive unavailable, a sensor it no longer
        trusts, a latched stop -- degrades the component with the reason, so
        an operator sees *why* the rover will not move without reading DIAG.
        """

        status = snapshot.communication.esp32
        detail = snapshot.communication.esp32_detail or status.value

        if status is Esp32LinkStatus.NOT_IMPLEMENTED:
            return ComponentHealth("esp32", HealthStatus.UNKNOWN, "no ESP32 link has reported")
        if status is Esp32LinkStatus.DISABLED:
            if self._esp32_config_error:
                # Enabled by the operator but not started: a real fault.
                return ComponentHealth(
                    "esp32", HealthStatus.FAILED, f"not started: {self._esp32_config_error}"
                )
            return ComponentHealth("esp32", HealthStatus.UNKNOWN, "disabled by configuration")
        if status is Esp32LinkStatus.STOPPING:
            return ComponentHealth("esp32", HealthStatus.UNKNOWN, "stopping")
        if status is Esp32LinkStatus.CONNECTING:
            return ComponentHealth("esp32", HealthStatus.DEGRADED, detail)
        if status is Esp32LinkStatus.DEGRADED:
            return ComponentHealth("esp32", HealthStatus.DEGRADED, detail)
        if status.is_up:
            problems = _controller_problems(snapshot)
            if problems:
                return ComponentHealth("esp32", HealthStatus.DEGRADED, "; ".join(problems))
            return ComponentHealth("esp32", HealthStatus.HEALTHY, detail)
        if status is Esp32LinkStatus.STALE:
            # The port is open but the controller has gone quiet. That is worse
            # than not being connected, because motion may still be commanded.
            return ComponentHealth("esp32", HealthStatus.FAILED, f"ESP32 has gone quiet; {detail}")
        return ComponentHealth("esp32", HealthStatus.FAILED, f"ESP32 link is down; {detail}")

    def _backend_health(self, snapshot: RobotSnapshot) -> ComponentHealth:
        """Health of the link to the RobotX backend, judged on its own.

        A link nobody enabled is UNKNOWN, not FAILED: not measuring something
        is not the same as it being broken, and a standalone Rover would
        otherwise report itself unhealthy forever. A link that *was* enabled
        and is not up is a genuine fault, because an operator is expecting
        supervision that is not happening.

        Nothing here consults the ESP32. The Rover must be operable and
        testable with no backend at all, so a backend verdict that could drag
        down the robot's own hardware health would defeat the point.
        """

        status = snapshot.communication.backend
        detail = snapshot.communication.backend_detail or status.value

        if status is BackendLinkStatus.AUTH_FAILED:
            # The backend refused this robot's credential. Retrying will not
            # fix it -- a human has to commission the robot again -- so it is a
            # failure rather than a transient.
            return ComponentHealth(
                "backend",
                HealthStatus.FAILED,
                f"backend refused this robot's credential; {detail}",
            )
        if status is BackendLinkStatus.DISABLED:
            return ComponentHealth("backend", HealthStatus.UNKNOWN, "disabled by configuration")
        if status.is_up:
            return ComponentHealth("backend", HealthStatus.HEALTHY, detail)
        # Enabled but not streaming: the robot is running unsupervised.
        return ComponentHealth(
            "backend", HealthStatus.DEGRADED, f"backend link is not streaming; {detail}"
        )


def _sensing_issue(
    perception: PerceptionResult, front: Optional[RangeReading], *, range_expected: bool
) -> Optional[str]:
    """Why the rover cannot sense its way forward right now, or None.

    Each is a stop the decision layer or the safety gate already makes on its
    own; naming it lets the progress monitor bound it (`ProgressMonitor`).
    """

    if not perception.is_usable:
        return f"perception unavailable ({perception.status.value})"
    if range_expected and (front is None or front.status is not RangeStatus.VALID):
        status = "missing" if front is None else front.status.value
        return f"forward range sensor not reporting ({status})"
    return None


def _gps_unavailable() -> GpsReading:
    """The reading reported when GPS is switched off by configuration."""

    return GpsReading(status=GPSStatus.UNAVAILABLE, error="GPS disabled")


def _same_device(a: str, b: str) -> bool:
    """Whether two device paths name the same tty (symlinks resolved)."""

    return os.path.realpath(a) == os.path.realpath(b)


def _controller_problems(snapshot: RobotSnapshot) -> List[str]:
    """What the ESP32 itself reports as wrong, from TELEMETRY and DIAG SYSTEM."""

    controller = snapshot.controller
    if controller is None or controller.telemetry is None:
        return []
    tel = controller.telemetry
    problems: List[str] = []
    if not tel.motor_drive_available:
        system = snapshot.controller_diag.system if snapshot.controller_diag else None
        why = (system or {}).get("motor_drive_status") or tel.block_reason
        problems.append(f"motor drive unavailable ({why})")
    if not tel.front_valid:
        problems.append("front sensing not valid")
    if tel.rear_sensor_fault:
        problems.append("rear sensor fault")
    if tel.safety_stop:
        problems.append("ESP32 safety stop latched")
    if tel.command_timeout:
        problems.append("ESP32 command watchdog latched")
    return problems
