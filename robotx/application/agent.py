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
from robotx.control.decision import DecisionConfig, DecisionMaker
from robotx.control.motion import MotionIntent
from robotx.control.safety import SafetyConfig, SafetyDecision, SafetyGate
from robotx.diagnostics.health import (
    ComponentHealth,
    HealthConfig,
    HealthMonitor,
    HealthStatus,
)
from robotx.esp32.link import Esp32Config, Esp32Link, Esp32Status
from robotx.hardware.camera import CameraConfig, CameraError, CameraStatus, CameraStream
from robotx.hardware.gps import GPSConfig, GPSReader, GpsReading, GPSStatus
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


class RobotAgent:
    """Owns every Pi subsystem and the loop that ties them together."""

    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.state = RobotState(settings.robot_id)

        self.camera: Optional[CameraStream] = None
        self.perception: Optional[PerceptionPipeline] = None
        self.gps: Optional[GPSReader] = None
        # Set when GPS was not started because its port is the ESP32's UART.
        self._gps_port_conflict: Optional[str] = None
        # The one owner of the ESP32 UART. None when ROBOTX_ESP32_ENABLED=0.
        self.esp32: Optional[Esp32Link] = None
        # Why an enabled ESP32 link was not started, if its config was invalid.
        self._esp32_config_error: Optional[str] = None
        # ESP32 reboots already acted on, so each triggers the latch once.
        self._esp32_reboots_handled = 0
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
        self.health = HealthMonitor(HealthConfig.from_settings(settings))

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

        self.backend = BackendLink(cfg, self.state, self, agent_alive=self.is_alive)
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
        except ValueError as e:
            # Same stance as a bad backend config: the robot still comes up,
            # with no motor link -- which is the safe state -- and says why.
            self._esp32_config_error = str(e)
            log_event(logger, "esp32.config_invalid", "ESP32 link not started",
                      level=logging.ERROR, error=str(e))
            return
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

        if self.safety.estop_engaged:
            raise MissionRejected(
                MissionRejectReason.ESTOP_ENGAGED,
                "emergency stop is latched; clear it before assigning a task",
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
        except MissionRejected as e:
            return OfferDecision.reject(f"UNSUPPORTED_MISSION: {e.reason.value}")

        snapshot = self.state.snapshot()
        if self.safety.estop_engaged:
            return OfferDecision.reject("ESTOP_LATCHED")
        if snapshot.mode is OperatingMode.ERROR:
            return OfferDecision.reject("AGENT_ERROR")
        if snapshot.mission is not None and snapshot.mission.is_active:
            return OfferDecision.reject("MISSION_IN_PROGRESS")
        # Motor authority belongs to the ESP32. Without that link the Rover
        # cannot move at all, whatever it accepts.
        if not snapshot.communication.esp32.is_up:
            return OfferDecision.reject("NO_MOTOR_LINK")
        # An UP link is not yet a motor link: motion must be enabled on the Pi
        # and the ESP32 must report that its drive is actually available.
        if self.esp32 is not None and not self.esp32.status().motion_ready:
            return OfferDecision.reject("NO_MOTOR_LINK")
        if snapshot.position is None or not snapshot.position.is_measured or not snapshot.gps.has_fix:
            return OfferDecision.reject("NO_POSITION_FIX")
        if not self.custody_sensing_available():
            return OfferDecision.reject("NO_CUSTODY_SENSING")
        return OfferDecision.accept(mission)

    def custody_sensing_available(self) -> bool:
        """Whether anything on this Rover can observe a parcel handover.

        False: there is no load sensor, no compartment switch and no agreed
        operator-confirmation path. Without one the Rover could never truthfully
        report ACQUIRED or RELEASED, so it must not accept a mission that
        requires them.
        """

        return False

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

    def stop_mission(self, reason: str = "operator stop") -> None:
        """Halt: clear the route, drop to STOPPED, and publish a stop intent."""

        self.navigator.clear_route()
        self._mission_route = []
        # Mark the task abandoned rather than leaving it mid-leg. A mission
        # still in TO_DROP with no route would otherwise be "arrived" by
        # whatever route came next, and reported as a delivery that never
        # happened.
        self.missions.abandon(reason)
        self.state.update_mission(self.missions.active)
        self.state.set_mode(OperatingMode.STOPPED)
        self.state.update_motion_intent(MotionIntent.stop(reason))
        log_event(logger, "mission.stopped", reason=reason)

    def pause_mission(self, reason: str = "operator pause") -> None:
        """Suspend the mission, keeping the route so it can be resumed.

        The route stays loaded in the navigator. `OperatingMode.PAUSED` is not
        `mission_active`, so the decision layer issues a hold on the very next
        tick -- pausing does not depend on anything downstream noticing a flag.
        """

        self.state.set_mode(OperatingMode.PAUSED)
        self.state.update_motion_intent(MotionIntent.stop(reason))
        log_event(logger, "mission.paused", reason=reason, waypoints=len(self._mission_route))

    def resume_mission(self, reason: str = "operator resume") -> None:
        """Return a paused mission to AUTO.

        Refuses when there is no route left to resume rather than dropping into
        AUTO with an empty navigator, which would report NAVIGATING with
        nowhere to go. The caller turns this into a FAILED acknowledgement.
        """

        # The mission manager is asked first: on the drop leg it holds the
        # route being driven, while `_mission_route` may still describe the
        # pickup leg that finished before the pause.
        route = list(self.missions.current_route()) or self._mission_route
        if not route:
            raise MissionRefused("no route is loaded; nothing to resume")
        self._mission_route = route
        if self.navigator.planner.route_length() == 0:
            self.navigator.set_route(route)
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
        self.state.set_mode(OperatingMode.STOPPED, error=f"emergency stop: {reason}")
        self.state.update_motion_intent(MotionIntent.stop(f"safety: {reason}"))
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
        return released

    @property
    def emergency_stopped(self) -> bool:
        return self.safety.estop_engaged

    def resume_idle(self) -> None:
        """Leave STOPPED/ERROR and go back to IDLE, clearing the last error."""

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
        position = self.position_estimator.update(gps_reading)

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
        self.state.update_power(PowerState.from_status_dict(battery_status()))

        # 2b. ESP32 link: what it reports goes into state before anything
        #     decides, and a link that can no longer carry motion pauses a
        #     running mission rather than letting it continue unexecuted.
        link = self.esp32
        esp32 = self._update_esp32()
        if (
            link is not None
            and esp32 is not None
            and link.cfg.motion_enabled
            and self.state.mode.mission_active
            and not esp32.motion_ready
        ):
            self.pause_mission(f"ESP32 cannot carry motion: {esp32.link.value} ({esp32.detail})")

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

        # 5b. Safety gate has the last word. What lands in state is the *gated*
        #     intent, because state is what telemetry and (once it exists) the
        #     ESP32 link read -- an intent the gate refused must never be
        #     visible anywhere as though it were going to happen.
        #
        #     No range reading is passed: this rover has no working forward
        #     range sensor yet, and `SafetyConfig.require_range_sensor` is
        #     where that absence is handled honestly rather than here.
        safety = self.safety.evaluate(
            proposed,
            mission_active=mode.mission_active,
            perception=perception,
        )
        intent = safety.intent
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

        # A blocked leg feeds back into rerouting. Read off the gated intent so
        # a safety veto for a measured obstacle drives rerouting exactly as the
        # decision layer's vision-based one does.
        if mode.mission_active and intent.is_stop and "obstacle" in intent.reason:
            self.navigator.report_blocked()

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
        if self.gps is None:
            return ComponentHealth(
                "gps", HealthStatus.UNKNOWN, "disabled by configuration"
            )

        reading = snapshot.gps
        status, detail = _GPS_HEALTH.get(
            reading.status, (HealthStatus.UNKNOWN, "unrecognized GPS status")
        )
        return ComponentHealth("gps", status, reading.error or detail)

    def _navigation_health(self, snapshot: RobotSnapshot) -> ComponentHealth:
        state = snapshot.navigation
        if state.status is NavigationStatus.REROUTE_NEEDED:
            return ComponentHealth("navigation", HealthStatus.DEGRADED, "reroute needed")
        if state.status is NavigationStatus.NO_POSITION:
            return ComponentHealth(
                "navigation", HealthStatus.DEGRADED, "route loaded but no position"
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
