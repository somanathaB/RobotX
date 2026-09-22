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
`MotionIntent`; the ESP32 will consume it once that link exists.

Each tick:
    GPS -> position -> navigation -> (with perception) decision -> motion intent
    -> state -> telemetry / health
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any, Callable, Dict, List, Optional, Sequence

from robotx.config.logging_setup import log_event, setup_logging
from robotx.config.settings import Settings
from robotx.control.decision import DecisionConfig, DecisionMaker
from robotx.control.motion import MotionIntent
from robotx.diagnostics.health import (
    ComponentHealth,
    HealthConfig,
    HealthMonitor,
    HealthStatus,
)
from robotx.hardware.camera import CameraConfig, CameraError, CameraStatus, CameraStream
from robotx.hardware.gps import GPSConfig, GPSReader, GpsReading, GPSStatus
from robotx.navigation.navigator import NavigationConfig, Navigator, NavigationStatus
from robotx.perception.pipeline import PerceptionPipeline
from robotx.perception.types import PerceptionResult, PerceptionStatus
from robotx.localization.position import LatLon, PositionConfig, PositionEstimator
from robotx.state.robot_state import (
    LinkStatus,
    MissionRefused,
    OperatingMode,
    RobotSnapshot,
    RobotState,
)
from robotx.state.telemetry import build_telemetry


logger = logging.getLogger(__name__)

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
        # Typed as Any to keep `socketio` out of this module's imports; see
        # `_start_backend_link`. None whenever no backend is configured.
        self.backend: Optional[Any] = None

        self.position_estimator = PositionEstimator(PositionConfig.from_settings(settings))
        self.navigator = Navigator(NavigationConfig.from_settings(settings))
        self.decision = DecisionMaker(DecisionConfig.from_settings(settings))
        self.health = HealthMonitor(HealthConfig.from_settings(settings))

        self._task: Optional[asyncio.Task] = None
        self._running = False
        self._started = False
        self._last_telemetry: Dict[str, Any] = {}
        self._last_telemetry_t = 0.0
        self._last_health_t = 0.0
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
        self._start_gps()

        self.state.update_communication(
            esp32=LinkStatus.NOT_IMPLEMENTED,
            backend=LinkStatus.DISABLED,
            backend_detail="",
        )
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
                backend=LinkStatus.DISABLED, backend_detail=f"configuration error: {e}"
            )
            return

        self.backend = BackendLink(cfg, self.state, self)
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

        # Release in reverse order of acquisition.
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
        self.gps = GPSReader(GPSConfig.from_settings(self.settings))
        self.gps.start()

    # --- mission control (local, no backend required) ------------------------

    @property
    def mode(self) -> OperatingMode:
        """Current operating mode. Part of the command-target contract."""

        return self.state.mode

    def start_mission(self, waypoints: Sequence[LatLon]) -> None:
        """Load a route and switch to AUTO."""

        if len(waypoints) < 1:
            raise ValueError("a mission needs at least one waypoint")

        route = list(waypoints)
        self.navigator.set_route(route)
        self._mission_route = route
        # Remember where this run began, so a later RETURN has a real place to
        # go back to instead of an invented one. Only recorded when there is an
        # actual fix; a mission started without GPS leaves this None and RETURN
        # will correctly refuse.
        snapshot = self.state.snapshot()
        if snapshot.position is not None and snapshot.gps.status is GPSStatus.FIX:
            self._mission_origin = snapshot.position.lat_lon
        self.state.set_mode(OperatingMode.AUTO)
        log_event(logger, "mission.started", waypoints=len(route))

    def stop_mission(self, reason: str = "operator stop") -> None:
        """Halt: clear the route, drop to STOPPED, and publish a stop intent."""

        self.navigator.clear_route()
        self._mission_route = []
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

        if not self._mission_route:
            raise MissionRefused("no route is loaded; nothing to resume")
        if self.navigator.planner.route_length() == 0:
            self.navigator.set_route(self._mission_route)
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

    # --- loop ----------------------------------------------------------------

    async def _loop(self) -> None:
        period = 1.0 / max(0.5, float(self.settings.agent_hz))

        while self._running:
            started = time.monotonic()
            try:
                self.tick()
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
        self.state.update_gps(gps_reading, position)

        # 2. Navigation
        navigation = self.navigator.update(position)
        self.state.update_navigation(navigation)

        # 3. Perception (already computed on its own thread)
        perception = (
            self.perception.latest()
            if self.perception is not None
            else PerceptionResult.unavailable(PerceptionStatus.DISABLED)
        )
        self.state.update_perception(perception)

        # 4. Decision -> motion intent (published, never actuated here)
        mode = self.state.mode
        intent = self.decision.decide(
            mission_active=mode.mission_active,
            navigation=navigation,
            perception=perception,
        )
        self.state.update_motion_intent(intent)

        # A blocked leg feeds back into rerouting.
        if mode.mission_active and intent.is_stop and "obstacle" in intent.reason:
            self.navigator.report_blocked()

        # 5. Health, evaluated from a single consistent snapshot
        if now - self._last_health_t >= self.settings.health_interval_s:
            components = self._component_health(self.state.snapshot())
            self.state.update_health(self.health.evaluate(components))
            self._last_health_t = now

        # 6. Telemetry (local)
        snapshot = self.state.snapshot()
        if now - self._last_telemetry_t >= self.settings.telemetry_interval_s:
            self._publish_telemetry(snapshot)
            self._last_telemetry_t = now

        return snapshot

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
            "communication": self._communication_health(snapshot),
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

    def _communication_health(self, snapshot: RobotSnapshot) -> ComponentHealth:
        """Health of the outbound links, judged against what was asked for.

        A link nobody enabled is UNKNOWN, not FAILED: not measuring something
        is not the same as it being broken, and a standalone Pi would otherwise
        report itself unhealthy forever. A link that *was* enabled and is not
        up is a genuine fault, because an operator is expecting supervision
        that is not happening.
        """

        comms = snapshot.communication
        detail = f"esp32={comms.esp32.value}, backend={comms.backend.value}"

        if comms.backend is LinkStatus.REJECTED:
            # Credentials or robot identity are wrong. Retrying will not fix
            # it, so it is a failure rather than a transient.
            return ComponentHealth(
                "communication", HealthStatus.FAILED, f"backend rejected the connection; {detail}"
            )
        if comms.backend is LinkStatus.CONNECTED:
            if comms.backend_protocol_provisional:
                return ComponentHealth(
                    "communication",
                    HealthStatus.DEGRADED,
                    f"connected on a PROVISIONAL protocol binding; {detail}",
                )
            return ComponentHealth("communication", HealthStatus.HEALTHY, detail)
        if comms.backend in (LinkStatus.DISCONNECTED, LinkStatus.CONNECTING):
            return ComponentHealth(
                "communication", HealthStatus.DEGRADED, f"backend link is down; {detail}"
            )
        return ComponentHealth("communication", HealthStatus.UNKNOWN, detail)


def _gps_unavailable() -> GpsReading:
    """The reading reported when GPS is switched off by configuration."""

    return GpsReading(status=GPSStatus.UNAVAILABLE, error="GPS disabled")
