# Dependency Map

File-level import graph, generated from the source tree and verified acyclic at
both module and package level.

## Layers

```
config          (leaf — imports nothing internal)
  ▲
hardware        camera, gps, battery, + bench-only GPIO drivers
  ▲
localization    GPS fix -> position + heading
  ▲
navigation      route progress -> desired heading
  ▲
control         decision -> MotionIntent
  ▲
state           the authoritative snapshot + telemetry
  ▲
application     agent lifecycle + HTTP API (composition root)

perception      (parallel branch: config, hardware -> perception -> control, state)
diagnostics     (parallel branch: config -> diagnostics -> state, application)
communication   (backend boundary; imported lazily by application/agent.py)
esp32           (ESP32 UART boundary: config, control -> esp32 -> state, application)
```

## Modules

| Module | Internal imports | Key external |
|---|---|---|
| `config/settings.py` | — | `os`, `dataclasses` |
| `config/logging_setup.py` | — | `logging` |
| `hardware/battery.py` | — | — |
| `hardware/camera.py` | `config.logging_setup` | `picamera2`, `cv2` |
| `hardware/gps.py` | `config.logging_setup` | `serial`, `pynmea2` |
| `hardware/motors.py` | — | `RPi.GPIO` |
| `hardware/encoders.py` | — | `RPi.GPIO` |
| `hardware/ultrasonic.py` | — | `RPi.GPIO` |
| `hardware/ir.py` | — | `RPi.GPIO` |
| `localization/position.py` | `hardware.gps` | `math` |
| `perception/types.py` | — | — |
| `perception/object_detector.py` | — | `cv2`, `ultralytics` (optional) |
| `perception/object_tracker.py` | — | — |
| `perception/temporal_filter.py` | `perception.object_tracker` | — |
| `perception/pipeline.py` | `config.logging_setup`, `perception.object_detector`, `perception.types` | `cv2` |
| `navigation/route_planner.py` | `localization.position` | — |
| `navigation/navigator.py` | `config.logging_setup`, `localization.position`, `navigation.route_planner` | — |
| `navigation/directions_client.py` | — | `httpx` |
| `control/motion.py` | — | — |
| `control/decision.py` | `config.logging_setup`, `control.motion`, `navigation.navigator`, `perception.types` | — |
| `diagnostics/health.py` | `config.logging_setup` | `os`, `pathlib` |
| `esp32/protocol.py` | — | `json`, `re` |
| `esp32/transport.py` | `esp32.protocol` | `serial` (lazy), `fcntl`, `termios` |
| `esp32/state.py` | — | — |
| `esp32/link.py` | `config.logging_setup`, `control.motion`, `control.safety`, `esp32.*` | `threading` |
| `state/robot_state.py` | `control.motion`, `diagnostics.health`, `esp32.state`, `hardware.gps`, `localization.position`, `navigation.navigator`, `perception.types` | — |
| `state/telemetry.py` | `hardware.battery`, `state.robot_state` | — |
| `application/agent.py` | `config.*`, `control.decision`, `control.motion`, `diagnostics.health`, `esp32.link`, `hardware.camera`, `hardware.gps`, `localization.position`, `navigation.navigator`, `perception.pipeline`, `perception.types`, `state.*` | `asyncio` |
| `application/main.py` | `application.agent`, `config.*`, `diagnostics.health`, `state.telemetry` | `fastapi`, `pydantic` |

## Not in the agent's import graph

| Module | Why |
|---|---|
| `communication/protocol.py` | Wire contract: event names as data, payload builders, inbound validation, redaction. |
| `communication/commands.py` | Validated command -> agent mission intent, idempotently. Imports `protocol`, `state`. |
| `communication/backend_link.py` | The one Socket.IO client. Imports `protocol`, `commands`, `state`. |
| `control/robot_controller.py` | Retained legacy direct-drive loop. Not started by the application. |
| `perception/experimental/*` | Retained experimental vision pipeline. Imported only by `tests/perception/test_vision.py`. |
| `hardware/motors.py`, `encoders.py`, `ultrasonic.py`, `ir.py` | ESP32-owned in the target architecture. Used only by bench scripts and the legacy loop. |

The agent path imports **no** motor driver and **no** `RPi.GPIO`.

## Tests

| Test | Imports |
|---|---|
| `tests/unit/*` | Pure Python + the `robotx` packages. No hardware, no network. |
| `tests/hardware/*` | Real GPIO / camera / serial. Manual only. |
| `tests/control/test_controller.py` | Real motors + sensors. Manual only. |
| `tests/perception/test_vision.py` | Real camera + the experimental pipeline. Manual only. |

## External dependencies

| Package | Used by | Required? |
|---|---|---|
| `fastapi`, `uvicorn`, `pydantic` | `application/` | Yes, to run the agent |
| `picamera2` | `hardware/camera.py` | Yes, for camera (apt, not pip) |
| `opencv-python` (`cv2`) | camera JPEG encode, detector, pipeline | Yes, for perception |
| `pyserial`, `pynmea2` | `hardware/gps.py` | Yes, for GPS |
| `RPi.GPIO` | bench-only hardware drivers | Only for bench scripts |
| `httpx` | `navigation/directions_client.py` | Only for online routing |
| `python-socketio` | `communication/backend_link.py` | Imported lazily; not loaded when `ROBOTX_SOCKET_ENABLED=0` |
| `ultralytics` | YOLO detector backend | Optional; not installed |

The unit tests and the health monitor add **no** dependencies — `unittest` and
`/proc` are both stdlib/kernel.
