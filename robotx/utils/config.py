import os
from dataclasses import dataclass
from typing import Optional


def _env(key: str, default: Optional[str] = None) -> Optional[str]:
    return os.environ.get(key, default)


def _env_int(key: str, default: int) -> int:
    value = os.environ.get(key)
    if value is None:
        return default
    return int(value)


def _env_float(key: str, default: float) -> float:
    value = os.environ.get(key)
    if value is None:
        return default
    return float(value)


def _env_bool(key: str, default: bool) -> bool:
    value = os.environ.get(key)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "y", "on"}


@dataclass(frozen=True)
class Settings:
    # General
    robot_id: str = _env("ROBOTX_ROBOT_ID", "robotx-pi") or "robotx-pi"
    log_level: str = _env("ROBOTX_LOG_LEVEL", "INFO") or "INFO"

    # FastAPI
    api_host: str = _env("ROBOTX_API_HOST", "0.0.0.0") or "0.0.0.0"
    api_port: int = _env_int("ROBOTX_API_PORT", 8000)

    # Socket.IO (robot -> remote server)
    socket_server_url: str = _env("ROBOTX_SOCKET_SERVER_URL", "http://localhost:3000") or "http://localhost:3000"
    socket_namespace: str = _env("ROBOTX_SOCKET_NAMESPACE", "/robot") or "/robot"
    socket_reconnect: bool = _env_bool("ROBOTX_SOCKET_RECONNECT", True)
    # Bearer credential sent in the Socket.IO connect handshake (`auth=`).
    # Read only from the environment -- never hardcode a real value here.
    # If unset, the client still connects (non-fatally) but logs a warning,
    # since there is no backend yet in this repo to authenticate against;
    # see ROBOTX_PI_REMEDIATION_PLAN.md (R-02) for the required backend contract.
    robot_token: Optional[str] = _env("ROBOTX_ROBOT_TOKEN", None)

    # Motor pins (BCM numbering) for L298N
    motor_left_in1: int = _env_int("ROBOTX_MOTOR_LEFT_IN1", 5)
    motor_left_in2: int = _env_int("ROBOTX_MOTOR_LEFT_IN2", 6)
    motor_left_ena: int = _env_int("ROBOTX_MOTOR_LEFT_ENA", 12)  # PWM

    motor_right_in3: int = _env_int("ROBOTX_MOTOR_RIGHT_IN3", 13)
    motor_right_in4: int = _env_int("ROBOTX_MOTOR_RIGHT_IN4", 19)
    motor_right_enb: int = _env_int("ROBOTX_MOTOR_RIGHT_ENB", 18)  # PWM

    motor_pwm_hz: int = _env_int("ROBOTX_MOTOR_PWM_HZ", 1000)
    motor_invert_left: bool = _env_bool("ROBOTX_MOTOR_INVERT_LEFT", False)
    motor_invert_right: bool = _env_bool("ROBOTX_MOTOR_INVERT_RIGHT", False)

    # Encoders
    encoder_left_pin: int = _env_int("ROBOTX_ENCODER_LEFT_PIN", 23)
    encoder_right_pin: int = _env_int("ROBOTX_ENCODER_RIGHT_PIN", 24)
    encoder_pulses_per_rev: int = _env_int("ROBOTX_ENCODER_PULSES_PER_REV", 20)
    wheel_diameter_m: float = _env_float("ROBOTX_WHEEL_DIAMETER_M", 0.065)

    # Ultrasonic (HC-SR04)
    ultrasonic_trigger_pin: int = _env_int("ROBOTX_ULTRASONIC_TRIGGER_PIN", 20)
    ultrasonic_echo_pin: int = _env_int("ROBOTX_ULTRASONIC_ECHO_PIN", 21)
    ultrasonic_poll_hz: float = _env_float("ROBOTX_ULTRASONIC_POLL_HZ", 10.0)

    # IR sensors (digital)
    ir_left_pin: int = _env_int("ROBOTX_IR_LEFT_PIN", 16)
    ir_right_pin: int = _env_int("ROBOTX_IR_RIGHT_PIN", 26)
    ir_center_pin: int = _env_int("ROBOTX_IR_CENTER_PIN", 25)
    ir_active_low: bool = _env_bool("ROBOTX_IR_ACTIVE_LOW", True)

    # Camera
    camera_index: int = _env_int("ROBOTX_CAMERA_INDEX", 0)
    camera_width: int = _env_int("ROBOTX_CAMERA_WIDTH", 640)
    camera_height: int = _env_int("ROBOTX_CAMERA_HEIGHT", 480)
    camera_fps: int = _env_int("ROBOTX_CAMERA_FPS", 20)

    # Detection
    detection_backend: str = _env("ROBOTX_DETECTION_BACKEND", "opencv") or "opencv"  # opencv|yolo
    yolo_model_path: str = _env("ROBOTX_YOLO_MODEL_PATH", "yolov8n.pt") or "yolov8n.pt"
    detection_min_conf: float = _env_float("ROBOTX_DETECTION_MIN_CONF", 0.35)

    # Navigation
    gps_port: str = _env("ROBOTX_GPS_PORT", "/dev/ttyAMA0") or "/dev/ttyAMA0"
    gps_baudrate: int = _env_int("ROBOTX_GPS_BAUDRATE", 9600)

    google_maps_api_key: Optional[str] = _env("ROBOTX_GOOGLE_MAPS_API_KEY", None)
    directions_min_interval_s: float = _env_float("ROBOTX_DIRECTIONS_MIN_INTERVAL_S", 15.0)
    directions_cache_ttl_s: float = _env_float("ROBOTX_DIRECTIONS_CACHE_TTL_S", 300.0)

    # Controller
    control_hz: float = _env_float("ROBOTX_CONTROL_HZ", 10.0)
    telemetry_interval_s: float = _env_float("ROBOTX_TELEMETRY_INTERVAL_S", 1.5)

    obstacle_distance_cm: float = _env_float("ROBOTX_OBSTACLE_DISTANCE_CM", 35.0)
    avoid_turn_seconds: float = _env_float("ROBOTX_AVOID_TURN_SECONDS", 0.5)
    avoid_forward_seconds: float = _env_float("ROBOTX_AVOID_FORWARD_SECONDS", 0.6)

    target_speed_mps: float = _env_float("ROBOTX_TARGET_SPEED_MPS", 0.25)
    max_motor_duty: float = _env_float("ROBOTX_MAX_MOTOR_DUTY", 0.75)


SETTINGS = Settings()
