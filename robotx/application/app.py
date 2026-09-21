import asyncio
import logging
from typing import AsyncGenerator, Optional

from fastapi import FastAPI
from fastapi.responses import StreamingResponse

from robotx.communication.socket_client import RobotSocketClient, SocketConfig
from robotx.control.robot_controller import ControllerConfig, RobotController
from robotx.hardware.encoders import EncoderConfig, EncoderReader
from robotx.hardware.gps import GPSConfig, GPSReader
from robotx.hardware.ir import IRConfig, IRSensors
from robotx.hardware.motors import MotorDriver, MotorPins
from robotx.hardware.camera import CameraStream
from robotx.hardware.ultrasonic import UltrasonicConfig, UltrasonicSensor
from robotx.navigation.directions_client import GoogleMapsDirections
from robotx.navigation.route_planner import PlannerConfig, RoutePlanner
from robotx.perception.object_detector import ObjectDetector
from robotx.config.settings import SETTINGS


def _setup_logging() -> None:
    logging.basicConfig(
        level=getattr(logging, SETTINGS.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )


app = FastAPI(title="RobotX", version="1.0")


class AppState:
    controller: Optional[RobotController] = None
    socket: Optional[RobotSocketClient] = None
    camera: Optional[CameraStream] = None


state = AppState()


@app.get("/health")
async def health():
    return {
        "ok": True,
        "robot_id": SETTINGS.robot_id,
        "socket_connected": bool(state.socket and state.socket.sio.connected),
    }


async def _mjpeg_stream() -> AsyncGenerator[bytes, None]:
    boundary = b"frame"
    while True:
        cam = state.camera
        if cam is None:
            await asyncio.sleep(0.2)
            continue
        jpg = cam.get_jpeg()
        if jpg is None:
            await asyncio.sleep(0.05)
            continue
        yield b"--" + boundary + b"\r\n"
        yield b"Content-Type: image/jpeg\r\n"
        yield f"Content-Length: {len(jpg)}\r\n\r\n".encode()
        yield jpg + b"\r\n"
        await asyncio.sleep(0.05)


@app.get("/camera")
async def camera_stream():
    return StreamingResponse(
        _mjpeg_stream(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.on_event("startup")
async def on_startup() -> None:
    _setup_logging()

    motors = MotorDriver(
        left=MotorPins(
            in1=SETTINGS.motor_left_in1,
            in2=SETTINGS.motor_left_in2,
            en=SETTINGS.motor_left_ena,
            invert=SETTINGS.motor_invert_left,
        ),
        right=MotorPins(
            in1=SETTINGS.motor_right_in3,
            in2=SETTINGS.motor_right_in4,
            en=SETTINGS.motor_right_enb,
            invert=SETTINGS.motor_invert_right,
        ),
        pwm_hz=SETTINGS.motor_pwm_hz,
        max_duty=SETTINGS.max_motor_duty,
    )

    encoders = EncoderReader(
        EncoderConfig(
            left_pin=SETTINGS.encoder_left_pin,
            right_pin=SETTINGS.encoder_right_pin,
            pulses_per_rev=SETTINGS.encoder_pulses_per_rev,
            wheel_diameter_m=SETTINGS.wheel_diameter_m,
        ),
        sample_hz=10.0,
    )

    ultrasonic = UltrasonicSensor(
        UltrasonicConfig(
            trigger_pin=SETTINGS.ultrasonic_trigger_pin,
            echo_pin=SETTINGS.ultrasonic_echo_pin,
        ),
        poll_hz=SETTINGS.ultrasonic_poll_hz,
    )

    ir = IRSensors(
        IRConfig(
            left_pin=SETTINGS.ir_left_pin,
            right_pin=SETTINGS.ir_right_pin,
            center_pin=SETTINGS.ir_center_pin,
            active_low=SETTINGS.ir_active_low,
        )
    )

    gps = GPSReader(GPSConfig(port=SETTINGS.gps_port, baudrate=SETTINGS.gps_baudrate))
    camera = CameraStream(index=SETTINGS.camera_index, width=SETTINGS.camera_width, height=SETTINGS.camera_height, fps=SETTINGS.camera_fps)
    detector = ObjectDetector(backend=SETTINGS.detection_backend, min_conf=SETTINGS.detection_min_conf, yolo_model_path=SETTINGS.yolo_model_path)

    planner = RoutePlanner(PlannerConfig())
    maps = GoogleMapsDirections(
        api_key=SETTINGS.google_maps_api_key,
        min_interval_s=SETTINGS.directions_min_interval_s,
        cache_ttl_s=SETTINGS.directions_cache_ttl_s,
    )

    controller = RobotController(
        cfg=ControllerConfig(
            control_hz=SETTINGS.control_hz,
            telemetry_interval_s=SETTINGS.telemetry_interval_s,
            obstacle_distance_cm=SETTINGS.obstacle_distance_cm,
            target_speed_mps=SETTINGS.target_speed_mps,
            max_cmd=SETTINGS.max_motor_duty,
            avoid_turn_seconds=SETTINGS.avoid_turn_seconds,
            avoid_forward_seconds=SETTINGS.avoid_forward_seconds,
        ),
        motors=motors,
        encoders=encoders,
        ultrasonic=ultrasonic,
        ir=ir,
        gps=gps,
        camera=camera,
        detector=detector,
        planner=planner,
        maps=maps,
        robot_id=SETTINGS.robot_id,
    )

    sock = RobotSocketClient(
        SocketConfig(
            server_url=SETTINGS.socket_server_url,
            namespace=SETTINGS.socket_namespace,
            robot_id=SETTINGS.robot_id,
            reconnect=SETTINGS.socket_reconnect,
            robot_token=SETTINGS.robot_token,
        )
    )

    async def telemetry_hook(payload):
        await sock.enqueue_telemetry(payload)

    controller.set_telemetry_hook(telemetry_hook)

    # Connect socket client (non-fatal if offline)
    async def socket_boot():
        try:
            await sock.connect()
            sock.start_background_tasks(controller.handle_command, telemetry_interval_s=SETTINGS.telemetry_interval_s)
        except Exception as e:
            logging.getLogger(__name__).warning("Socket connect failed: %s", e)

    asyncio.create_task(socket_boot())

    controller.start()

    state.controller = controller
    state.socket = sock
    state.camera = camera


@app.on_event("shutdown")
async def on_shutdown() -> None:
    if state.controller is not None:
        await state.controller.stop()
    if state.socket is not None:
        await state.socket.close()
