"""Centralized configuration for the RobotX Pi agent.

Every `ROBOTX_*` environment variable is read here and nowhere else. Modules
import `SETTINGS` (or receive a `Settings` instance) rather than calling
`os.environ` themselves.

Defaults are plain literals on the dataclass fields; `Settings.from_env()`
overlays the environment. That split keeps the defaults inspectable and makes
the settings object constructible in tests without mutating global state.

No secret ever has a default value here -- secrets are `None` unless the
operator sets the corresponding environment variable. See `.env.example`.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, fields
from typing import Any, Mapping, Optional


Env = Mapping[str, str]


def _get(env: Env, key: str, default: Optional[str]) -> Optional[str]:
    value = env.get(key)
    return default if value is None else value


def _get_str(env: Env, key: str, default: str) -> str:
    value = env.get(key)
    return default if value is None else value


def _get_int(env: Env, key: str, default: int) -> int:
    value = env.get(key)
    if value is None:
        return default
    try:
        return int(value.strip())
    except ValueError:
        raise ValueError(f"{key} must be an integer, got {value!r}") from None


def _get_float(env: Env, key: str, default: float) -> float:
    value = env.get(key)
    if value is None:
        return default
    try:
        return float(value.strip())
    except ValueError:
        raise ValueError(f"{key} must be a number, got {value!r}") from None


def _get_opt_float(env: Env, key: str, default: Optional[float]) -> Optional[float]:
    """A float that may legitimately be absent.

    An empty string is treated as unset, so a systemd `EnvironmentFile` line
    left blank means "not configured" rather than a parse error.
    """

    value = env.get(key)
    if value is None or not value.strip():
        return default
    try:
        return float(value.strip())
    except ValueError:
        raise ValueError(f"{key} must be a number, got {value!r}") from None


def _get_bool(env: Env, key: str, default: bool) -> bool:
    value = env.get(key)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "y", "on"}


@dataclass(frozen=True)
class Settings:
    """Immutable configuration snapshot for one process."""

    # --- Identity / logging -------------------------------------------------
    # Must equal the commissioned Robot.robotId exactly (case-sensitive). No
    # default: required when the backend link is enabled. For local
    # development set it to whatever the unit was commissioned as.
    robot_id: str = ""
    log_level: str = "INFO"

    # --- Local HTTP API -----------------------------------------------------
    api_host: str = "0.0.0.0"
    api_port: int = 8000

    # --- Agent loop ---------------------------------------------------------
    agent_hz: float = 10.0
    telemetry_interval_s: float = 1.5
    health_interval_s: float = 2.0

    # --- Camera (Raspberry Pi Camera Module 3 via Picamera2) ----------------
    camera_enabled: bool = True
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: int = 20
    # A frame older than this means the capture thread has stalled or died.
    camera_stale_after_s: float = 2.0

    # --- Perception ---------------------------------------------------------
    perception_enabled: bool = True
    detection_hz: float = 2.0
    detection_backend: str = "opencv"  # opencv | yolo | auto
    detection_min_conf: float = 0.35
    yolo_model_path: str = "yolov8n.pt"
    # Detector runs on a downscaled frame; bboxes are scaled back to full res.
    detection_inference_width: int = 320
    detection_inference_height: int = 240
    # A perception result older than this is treated as unusable (fail safe).
    perception_stale_after_s: float = 3.0

    # Detector tuning (OpenCV backend). These were previously read ad hoc from
    # the environment inside the detector itself.
    detector_mog2_history: int = 250
    detector_mog2_var_threshold: float = 32.0
    detector_mog2_warmup_frames: int = 20
    detector_mog2_min_area_px: int = 4000
    detector_min_motion_ratio: float = 0.001
    detector_static_canny1: int = 60
    detector_static_canny2: int = 160
    detector_static_min_area_px: int = 2000
    detector_equalize: bool = True

    # --- GPS ----------------------------------------------------------------
    gps_enabled: bool = True
    gps_port: str = "/dev/ttyAMA0"
    gps_baudrate: int = 9600
    gps_timeout_s: float = 1.0
    # A fix older than this is reported as STALE.
    gps_stale_after_s: float = 5.0
    # Seconds to wait before retrying a failed serial open.
    gps_reconnect_interval_s: float = 5.0

    # --- Position estimation ------------------------------------------------
    # Minimum movement between two fixes before a GPS-track heading is derived.
    position_heading_min_move_m: float = 1.5
    # Minimum speed-over-ground (m/s) before the NMEA track angle is trusted.
    position_heading_min_speed_mps: float = 0.5

    # --- Navigation ---------------------------------------------------------
    nav_waypoint_arrival_m: float = 8.0
    nav_off_route_m: float = 25.0
    nav_reroute_after_n: int = 8
    nav_blocked_reroute_after_n: int = 3
    nav_target_speed_mps: float = 0.25

    # Optional online routing. Without an API key the agent runs on locally
    # supplied waypoint lists only -- it never fabricates a route.
    google_maps_api_key: Optional[str] = None
    directions_min_interval_s: float = 15.0
    directions_cache_ttl_s: float = 300.0

    # --- Decision / motion intent -------------------------------------------
    # Normalized [0..1] speeds requested from the (future) motor controller.
    motion_cruise_speed: float = 0.45
    motion_turn_speed: float = 0.35
    motion_slow_speed: float = 0.25
    motion_max_speed: float = 0.75
    # Detection area (px^2, full-resolution frame) that forces a STOP.
    obstacle_stop_area_px: int = 30000
    # Detection area (px^2) that forces a SLOW.
    obstacle_slow_area_px: int = 10000
    # Smallest detection area taken seriously at all.
    obstacle_min_area_px: int = 1500
    # Fraction of frame width treated as the robot's collision corridor.
    obstacle_center_zone_ratio: float = 0.33
    # Steering: full-scale turn at this heading error (degrees).
    steering_full_scale_deg: float = 45.0

    # --- Local frame / dead reckoning ----------------------------------------
    # Lets the rover navigate with no GPS, by integrating the motion it
    # commanded into a local metres frame pinned to an origin. Open loop: the
    # estimate drifts without bound and is labelled DEAD_RECKONING everywhere
    # it appears. See robotx.localization.local_frame.
    deadreckon_enabled: bool = False
    # Lat/lon the local frame is pinned to. With none set, a synthetic origin
    # is used and the rover reports itself at (0, 0) -- obviously not a real
    # place, which is the point.
    local_origin_lat: Optional[float] = None
    local_origin_lon: Optional[float] = None
    # CALIBRATE THESE ON THE REAL ROVER. The defaults are placeholders; used
    # as-is the pose will be wrong by a large factor.
    deadreckon_max_speed_mps: float = 0.4
    deadreckon_turn_rate_dps: float = 90.0
    deadreckon_max_step_s: float = 0.5
    # There is deliberately no setting to publish a dead-reckoned position to
    # the backend: only measured positions leave the Pi. See
    # `build_telemetry_payload`.

    # --- Safety gate ---------------------------------------------------------
    # The last check a motion intent passes before it may leave the Pi. These
    # limits are deliberately separate from the decision-layer ones above: the
    # gate exists to catch a mis-set or buggy decision layer, which it could not
    # do if it read the same numbers. See robotx.control.safety.
    safety_max_speed: float = 0.75
    safety_max_intent_age_s: float = 1.0
    safety_stop_distance_cm: float = 30.0
    # Leave 0 until a forward range sensor is actually wired; see SafetyConfig.
    safety_require_range_sensor: bool = False

    # --- Health -------------------------------------------------------------
    health_cpu_warn_percent: float = 90.0
    health_memory_warn_percent: float = 90.0
    health_temp_warn_c: float = 80.0

    # --- Backend link --------------------------------------------------------
    # Disabled by default: the Pi agent must run standalone, and no backend is
    # reachable from this repository. Enabling it starts a real Socket.IO
    # client; see docs/communication/.
    socket_enabled: bool = False
    # No default host: the backend is never on this Pi. Set it to the laptop's
    # LAN address for local development (http://<LAPTOP-LAN-IP>:<PORT>) or to
    # the deployed backend (https://<host>). Required when socket_enabled.
    socket_server_url: str = ""
    socket_namespace: str = "/robot"
    socket_reconnect: bool = True

    # --- FalconAut credentials ----------------------------------------------
    # The 6-digit code from POST /api/robots/commission. One-time, 300 s TTL,
    # needed only until the first AUTH_SUCCESS returns a session token.
    pairing_code: Optional[str] = None
    # An explicit session token, overriding whatever is stored on disk. Rarely
    # needed: the normal source of a token is AUTH_SUCCESS.
    # Read only from the environment -- never hardcode a real value here.
    robot_token: Optional[str] = None
    # Where the session token from AUTH_SUCCESS is persisted, so a reconnect
    # does not need a human to issue a fresh pairing code.
    backend_token_path: Optional[str] = None
    # How long to wait for AUTH_SUCCESS before treating the attempt as failed.
    backend_auth_timeout_s: float = 10.0
    # The backend's COMMAND_SIGNING_KEY (>= 32 bytes), used to verify signed
    # Assignment Engine envelopes. SECRET. The backend has no mechanism to
    # provision it; unset means no OFFER can be admitted (verification is
    # never skipped).
    command_signing_key: Optional[str] = None
    # Commitment fence/sequence high-water marks and tombstones, persisted
    # across restarts because engine delivery is at-least-once.
    commitment_state_path: Optional[str] = "~/.robotx/commitments.json"

    # Path to a JSON file overriding the FalconAut event names. The built-in
    # binding follows the contract; this exists to correct any name the backend
    # spells differently, with no code change. See ProtocolBinding.
    protocol_file: Optional[str] = None

    # Outbound rates. Telemetry is the only high-frequency channel, and at 1 Hz
    # it is already 20x below the camera: the database should not grow at the
    # speed of the image sensor.
    backend_telemetry_interval_s: float = 1.0
    # Liveness cadence. Sent whenever the link is authenticated, including when
    # there is no GPS fix and therefore no telemetry at all -- otherwise a
    # healthy indoor robot is indistinguishable from a crashed one.
    backend_heartbeat_interval_s: float = 2.0
    backend_status_interval_s: float = 5.0
    # Positions older than this are not published as live telemetry at all.
    backend_max_position_age_s: float = 5.0
    # Inbound commands older than this are rejected rather than acted on.
    backend_command_max_age_s: float = 120.0

    backend_backoff_initial_s: float = 1.0
    backend_backoff_max_s: float = 60.0
    # Verify the server's TLS certificate. Turning this off makes the
    # credential interceptable; it exists only for a lab with a self-signed
    # certificate and is a deliberate, visible downgrade.
    backend_tls_verify: bool = True

    # What an active mission does when the backend becomes unreachable:
    # "pause" (default) suspends it after the grace period; "continue" keeps
    # driving. Neither can ever start motion.
    backend_loss_policy: str = "pause"
    backend_loss_grace_s: float = 30.0

    # --- Home position -------------------------------------------------------
    # Where a backend RETURN command sends the robot. With neither this nor a
    # recorded mission origin, RETURN is refused rather than given a guess.
    home_lat: Optional[float] = None
    home_lon: Optional[float] = None

    # --- Motor GPIO pins (bench tooling only) -------------------------------
    # The Pi agent does NOT drive motors; motor authority belongs to the ESP32.
    # These remain only for the standalone bench scripts under tests/hardware/
    # and the retained legacy direct-drive controller.
    motor_left_in1: int = 5
    motor_left_in2: int = 6
    motor_left_ena: int = 12  # PWM
    motor_right_in3: int = 13
    motor_right_in4: int = 19
    motor_right_enb: int = 18  # PWM
    motor_pwm_hz: int = 1000
    motor_invert_left: bool = False
    motor_invert_right: bool = False
    max_motor_duty: float = 0.75

    # --- Bench-only sensor pins (ESP32-owned in the target architecture) ----
    encoder_left_pin: int = 23
    encoder_right_pin: int = 24
    encoder_pulses_per_rev: int = 20
    wheel_diameter_m: float = 0.065
    ultrasonic_trigger_pin: int = 20
    ultrasonic_echo_pin: int = 21
    ultrasonic_poll_hz: float = 10.0
    ir_left_pin: int = 16
    ir_right_pin: int = 26
    ir_center_pin: int = 25
    ir_active_low: bool = True

    # --- Legacy direct-drive controller (retained, not wired in) ------------
    control_hz: float = 10.0
    obstacle_distance_cm: float = 35.0
    avoid_turn_seconds: float = 0.5
    avoid_forward_seconds: float = 0.6
    target_speed_mps: float = 0.25

    @classmethod
    def from_env(cls, env: Optional[Env] = None) -> "Settings":
        """Build settings from environment variables, falling back to defaults."""

        e: Env = os.environ if env is None else env
        d = cls()  # defaults

        return cls(
            robot_id=_get_str(e, "ROBOTX_ROBOT_ID", d.robot_id),
            log_level=_get_str(e, "ROBOTX_LOG_LEVEL", d.log_level),
            api_host=_get_str(e, "ROBOTX_API_HOST", d.api_host),
            api_port=_get_int(e, "ROBOTX_API_PORT", d.api_port),
            agent_hz=_get_float(e, "ROBOTX_AGENT_HZ", d.agent_hz),
            telemetry_interval_s=_get_float(e, "ROBOTX_TELEMETRY_INTERVAL_S", d.telemetry_interval_s),
            health_interval_s=_get_float(e, "ROBOTX_HEALTH_INTERVAL_S", d.health_interval_s),
            camera_enabled=_get_bool(e, "ROBOTX_CAMERA_ENABLED", d.camera_enabled),
            camera_width=_get_int(e, "ROBOTX_CAMERA_WIDTH", d.camera_width),
            camera_height=_get_int(e, "ROBOTX_CAMERA_HEIGHT", d.camera_height),
            camera_fps=_get_int(e, "ROBOTX_CAMERA_FPS", d.camera_fps),
            camera_stale_after_s=_get_float(e, "ROBOTX_CAMERA_STALE_AFTER_S", d.camera_stale_after_s),
            perception_enabled=_get_bool(e, "ROBOTX_PERCEPTION_ENABLED", d.perception_enabled),
            detection_hz=_get_float(e, "ROBOTX_DETECTION_HZ", d.detection_hz),
            detection_backend=_get_str(e, "ROBOTX_DETECTION_BACKEND", d.detection_backend),
            detection_min_conf=_get_float(e, "ROBOTX_DETECTION_MIN_CONF", d.detection_min_conf),
            yolo_model_path=_get_str(e, "ROBOTX_YOLO_MODEL_PATH", d.yolo_model_path),
            detection_inference_width=_get_int(e, "ROBOTX_DETECTION_INFERENCE_WIDTH", d.detection_inference_width),
            detection_inference_height=_get_int(e, "ROBOTX_DETECTION_INFERENCE_HEIGHT", d.detection_inference_height),
            perception_stale_after_s=_get_float(e, "ROBOTX_PERCEPTION_STALE_AFTER_S", d.perception_stale_after_s),
            detector_mog2_history=_get_int(e, "ROBOTX_MOG2_HISTORY", d.detector_mog2_history),
            detector_mog2_var_threshold=_get_float(e, "ROBOTX_MOG2_VAR_THRESHOLD", d.detector_mog2_var_threshold),
            detector_mog2_warmup_frames=_get_int(e, "ROBOTX_MOG2_WARMUP", d.detector_mog2_warmup_frames),
            detector_mog2_min_area_px=_get_int(e, "ROBOTX_MOG2_MIN_AREA", d.detector_mog2_min_area_px),
            detector_min_motion_ratio=_get_float(e, "ROBOTX_MIN_MOTION_RATIO", d.detector_min_motion_ratio),
            detector_static_canny1=_get_int(e, "ROBOTX_STATIC_CANNY1", d.detector_static_canny1),
            detector_static_canny2=_get_int(e, "ROBOTX_STATIC_CANNY2", d.detector_static_canny2),
            detector_static_min_area_px=_get_int(e, "ROBOTX_STATIC_MIN_AREA", d.detector_static_min_area_px),
            detector_equalize=_get_bool(e, "ROBOTX_EQUALIZE", d.detector_equalize),
            gps_enabled=_get_bool(e, "ROBOTX_GPS_ENABLED", d.gps_enabled),
            gps_port=_get_str(e, "ROBOTX_GPS_PORT", d.gps_port),
            gps_baudrate=_get_int(e, "ROBOTX_GPS_BAUDRATE", d.gps_baudrate),
            gps_timeout_s=_get_float(e, "ROBOTX_GPS_TIMEOUT_S", d.gps_timeout_s),
            gps_stale_after_s=_get_float(e, "ROBOTX_GPS_STALE_AFTER_S", d.gps_stale_after_s),
            gps_reconnect_interval_s=_get_float(e, "ROBOTX_GPS_RECONNECT_INTERVAL_S", d.gps_reconnect_interval_s),
            position_heading_min_move_m=_get_float(e, "ROBOTX_HEADING_MIN_MOVE_M", d.position_heading_min_move_m),
            position_heading_min_speed_mps=_get_float(e, "ROBOTX_HEADING_MIN_SPEED_MPS", d.position_heading_min_speed_mps),
            nav_waypoint_arrival_m=_get_float(e, "ROBOTX_NAV_WAYPOINT_ARRIVAL_M", d.nav_waypoint_arrival_m),
            nav_off_route_m=_get_float(e, "ROBOTX_NAV_OFF_ROUTE_M", d.nav_off_route_m),
            nav_reroute_after_n=_get_int(e, "ROBOTX_NAV_REROUTE_AFTER_N", d.nav_reroute_after_n),
            nav_blocked_reroute_after_n=_get_int(e, "ROBOTX_NAV_BLOCKED_REROUTE_AFTER_N", d.nav_blocked_reroute_after_n),
            nav_target_speed_mps=_get_float(e, "ROBOTX_NAV_TARGET_SPEED_MPS", d.nav_target_speed_mps),
            google_maps_api_key=_get(e, "ROBOTX_GOOGLE_MAPS_API_KEY", d.google_maps_api_key),
            directions_min_interval_s=_get_float(e, "ROBOTX_DIRECTIONS_MIN_INTERVAL_S", d.directions_min_interval_s),
            directions_cache_ttl_s=_get_float(e, "ROBOTX_DIRECTIONS_CACHE_TTL_S", d.directions_cache_ttl_s),
            motion_cruise_speed=_get_float(e, "ROBOTX_MOTION_CRUISE_SPEED", d.motion_cruise_speed),
            motion_turn_speed=_get_float(e, "ROBOTX_MOTION_TURN_SPEED", d.motion_turn_speed),
            motion_slow_speed=_get_float(e, "ROBOTX_MOTION_SLOW_SPEED", d.motion_slow_speed),
            motion_max_speed=_get_float(e, "ROBOTX_MOTION_MAX_SPEED", d.motion_max_speed),
            obstacle_stop_area_px=_get_int(e, "ROBOTX_OBSTACLE_STOP_AREA_PX", d.obstacle_stop_area_px),
            obstacle_slow_area_px=_get_int(e, "ROBOTX_OBSTACLE_SLOW_AREA_PX", d.obstacle_slow_area_px),
            obstacle_min_area_px=_get_int(e, "ROBOTX_OBSTACLE_MIN_AREA_PX", d.obstacle_min_area_px),
            obstacle_center_zone_ratio=_get_float(e, "ROBOTX_OBSTACLE_CENTER_ZONE_RATIO", d.obstacle_center_zone_ratio),
            steering_full_scale_deg=_get_float(e, "ROBOTX_STEERING_FULL_SCALE_DEG", d.steering_full_scale_deg),
            deadreckon_enabled=_get_bool(e, "ROBOTX_DEADRECKON_ENABLED", d.deadreckon_enabled),
            local_origin_lat=_get_opt_float(e, "ROBOTX_LOCAL_ORIGIN_LAT", d.local_origin_lat),
            local_origin_lon=_get_opt_float(e, "ROBOTX_LOCAL_ORIGIN_LON", d.local_origin_lon),
            deadreckon_max_speed_mps=_get_float(e, "ROBOTX_DEADRECKON_MAX_SPEED_MPS", d.deadreckon_max_speed_mps),
            deadreckon_turn_rate_dps=_get_float(e, "ROBOTX_DEADRECKON_TURN_RATE_DPS", d.deadreckon_turn_rate_dps),
            deadreckon_max_step_s=_get_float(e, "ROBOTX_DEADRECKON_MAX_STEP_S", d.deadreckon_max_step_s),
            safety_max_speed=_get_float(e, "ROBOTX_SAFETY_MAX_SPEED", d.safety_max_speed),
            safety_max_intent_age_s=_get_float(e, "ROBOTX_SAFETY_MAX_INTENT_AGE_S", d.safety_max_intent_age_s),
            safety_stop_distance_cm=_get_float(e, "ROBOTX_SAFETY_STOP_DISTANCE_CM", d.safety_stop_distance_cm),
            safety_require_range_sensor=_get_bool(e, "ROBOTX_SAFETY_REQUIRE_RANGE_SENSOR", d.safety_require_range_sensor),
            health_cpu_warn_percent=_get_float(e, "ROBOTX_HEALTH_CPU_WARN_PERCENT", d.health_cpu_warn_percent),
            health_memory_warn_percent=_get_float(e, "ROBOTX_HEALTH_MEMORY_WARN_PERCENT", d.health_memory_warn_percent),
            health_temp_warn_c=_get_float(e, "ROBOTX_HEALTH_TEMP_WARN_C", d.health_temp_warn_c),
            socket_enabled=_get_bool(e, "ROBOTX_SOCKET_ENABLED", d.socket_enabled),
            socket_server_url=_get_str(e, "ROBOTX_SOCKET_SERVER_URL", d.socket_server_url),
            socket_namespace=_get_str(e, "ROBOTX_SOCKET_NAMESPACE", d.socket_namespace),
            socket_reconnect=_get_bool(e, "ROBOTX_SOCKET_RECONNECT", d.socket_reconnect),
            pairing_code=_get(e, "ROBOTX_PAIRING_CODE", d.pairing_code),
            robot_token=_get(e, "ROBOTX_ROBOT_TOKEN", d.robot_token),
            backend_token_path=_get(e, "ROBOTX_BACKEND_TOKEN_PATH", d.backend_token_path),
            command_signing_key=_get(e, "ROBOTX_COMMAND_SIGNING_KEY", d.command_signing_key),
            commitment_state_path=_get(e, "ROBOTX_COMMITMENT_STATE_PATH", d.commitment_state_path),
            backend_auth_timeout_s=_get_float(
                e, "ROBOTX_BACKEND_AUTH_TIMEOUT_S", d.backend_auth_timeout_s
            ),
            protocol_file=_get(e, "ROBOTX_PROTOCOL_FILE", d.protocol_file),
            backend_telemetry_interval_s=_get_float(
                e, "ROBOTX_BACKEND_TELEMETRY_INTERVAL_S", d.backend_telemetry_interval_s
            ),
            backend_heartbeat_interval_s=_get_float(
                e, "ROBOTX_BACKEND_HEARTBEAT_INTERVAL_S", d.backend_heartbeat_interval_s
            ),
            backend_status_interval_s=_get_float(
                e, "ROBOTX_BACKEND_STATUS_INTERVAL_S", d.backend_status_interval_s
            ),
            backend_max_position_age_s=_get_float(
                e, "ROBOTX_BACKEND_MAX_POSITION_AGE_S", d.backend_max_position_age_s
            ),
            backend_command_max_age_s=_get_float(
                e, "ROBOTX_BACKEND_COMMAND_MAX_AGE_S", d.backend_command_max_age_s
            ),
            backend_backoff_initial_s=_get_float(
                e, "ROBOTX_BACKEND_BACKOFF_INITIAL_S", d.backend_backoff_initial_s
            ),
            backend_backoff_max_s=_get_float(e, "ROBOTX_BACKEND_BACKOFF_MAX_S", d.backend_backoff_max_s),
            backend_tls_verify=_get_bool(e, "ROBOTX_BACKEND_TLS_VERIFY", d.backend_tls_verify),
            backend_loss_policy=_get_str(e, "ROBOTX_BACKEND_LOSS_POLICY", d.backend_loss_policy),
            backend_loss_grace_s=_get_float(e, "ROBOTX_BACKEND_LOSS_GRACE_S", d.backend_loss_grace_s),
            home_lat=_get_opt_float(e, "ROBOTX_HOME_LAT", d.home_lat),
            home_lon=_get_opt_float(e, "ROBOTX_HOME_LON", d.home_lon),
            motor_left_in1=_get_int(e, "ROBOTX_MOTOR_LEFT_IN1", d.motor_left_in1),
            motor_left_in2=_get_int(e, "ROBOTX_MOTOR_LEFT_IN2", d.motor_left_in2),
            motor_left_ena=_get_int(e, "ROBOTX_MOTOR_LEFT_ENA", d.motor_left_ena),
            motor_right_in3=_get_int(e, "ROBOTX_MOTOR_RIGHT_IN3", d.motor_right_in3),
            motor_right_in4=_get_int(e, "ROBOTX_MOTOR_RIGHT_IN4", d.motor_right_in4),
            motor_right_enb=_get_int(e, "ROBOTX_MOTOR_RIGHT_ENB", d.motor_right_enb),
            motor_pwm_hz=_get_int(e, "ROBOTX_MOTOR_PWM_HZ", d.motor_pwm_hz),
            motor_invert_left=_get_bool(e, "ROBOTX_MOTOR_INVERT_LEFT", d.motor_invert_left),
            motor_invert_right=_get_bool(e, "ROBOTX_MOTOR_INVERT_RIGHT", d.motor_invert_right),
            max_motor_duty=_get_float(e, "ROBOTX_MAX_MOTOR_DUTY", d.max_motor_duty),
            encoder_left_pin=_get_int(e, "ROBOTX_ENCODER_LEFT_PIN", d.encoder_left_pin),
            encoder_right_pin=_get_int(e, "ROBOTX_ENCODER_RIGHT_PIN", d.encoder_right_pin),
            encoder_pulses_per_rev=_get_int(e, "ROBOTX_ENCODER_PULSES_PER_REV", d.encoder_pulses_per_rev),
            wheel_diameter_m=_get_float(e, "ROBOTX_WHEEL_DIAMETER_M", d.wheel_diameter_m),
            ultrasonic_trigger_pin=_get_int(e, "ROBOTX_ULTRASONIC_TRIGGER_PIN", d.ultrasonic_trigger_pin),
            ultrasonic_echo_pin=_get_int(e, "ROBOTX_ULTRASONIC_ECHO_PIN", d.ultrasonic_echo_pin),
            ultrasonic_poll_hz=_get_float(e, "ROBOTX_ULTRASONIC_POLL_HZ", d.ultrasonic_poll_hz),
            ir_left_pin=_get_int(e, "ROBOTX_IR_LEFT_PIN", d.ir_left_pin),
            ir_right_pin=_get_int(e, "ROBOTX_IR_RIGHT_PIN", d.ir_right_pin),
            ir_center_pin=_get_int(e, "ROBOTX_IR_CENTER_PIN", d.ir_center_pin),
            ir_active_low=_get_bool(e, "ROBOTX_IR_ACTIVE_LOW", d.ir_active_low),
            control_hz=_get_float(e, "ROBOTX_CONTROL_HZ", d.control_hz),
            obstacle_distance_cm=_get_float(e, "ROBOTX_OBSTACLE_DISTANCE_CM", d.obstacle_distance_cm),
            avoid_turn_seconds=_get_float(e, "ROBOTX_AVOID_TURN_SECONDS", d.avoid_turn_seconds),
            avoid_forward_seconds=_get_float(e, "ROBOTX_AVOID_FORWARD_SECONDS", d.avoid_forward_seconds),
            target_speed_mps=_get_float(e, "ROBOTX_TARGET_SPEED_MPS", d.target_speed_mps),
        )

    def public_summary(self) -> dict:
        """Configuration snapshot safe to log or expose. Secrets are elided."""

        # The pairing code is a credential too: it is single-use, but for its
        # 300 second life anyone holding it can enrol as this robot.
        secret_fields = {"robot_token", "pairing_code", "google_maps_api_key", "command_signing_key"}
        out: dict[str, Any] = {}
        for f in fields(self):
            value = getattr(self, f.name)
            if f.name in secret_fields:
                out[f.name] = "SET" if value else "UNSET"
            else:
                out[f.name] = value
        return out


SETTINGS = Settings.from_env()
