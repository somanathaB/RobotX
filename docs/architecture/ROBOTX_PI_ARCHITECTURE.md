# RobotX Pi Agent — Architecture

**Status of this document:** describes the structure as of the 2026-09-22 architecture cleanup (`ROBOTX_ARCHITECTURE_CLEANUP_REPORT.md`, building on the earlier `ROBOTX_PI_ARCHITECTURE_REFACTOR_PLAN.md` pass). It documents what exists on disk today, distinguishing IMPLEMENTED / PARTIALLY IMPLEMENTED / EXPERIMENTAL / NOT IMPLEMENTED where relevant. It does not describe a fleet-management platform, a simulation environment, or any backend server — none of those exist in this repository. See `ROBOTX_DEEP_CURRENT_STATE_AUDIT.md` for the full line-by-line audit this structure was originally derived from, and `ROBOTX_PI_REMEDIATION_PLAN.md` for the safety/security remediation history (R-00 through R-10).

---

## 1. System Overview

RobotX is a single-robot Raspberry Pi control agent. It is **not** a fleet-management backend, has no database, no multi-robot registry, and no frontend. One Python process, started with `uvicorn`, does everything:

- Serves two read-only HTTP endpoints (`/health`, `/camera`) via FastAPI.
- Runs a 10 Hz control loop (`RobotController`) that reads sensors, applies obstacle-avoidance safety logic, follows GPS routes, and drives motors.
- Connects outbound as a Socket.IO client to an external server (not part of this repo) to receive commands and emit telemetry.

```
                    ┌─────────────────────────────────────────────┐
  HTTP  ───────────►│  robotx.application.main  (FastAPI, uvicorn) │
  GET /health         /health, /camera                             │
  GET /camera         on_startup(): builds every subsystem below    │
                    │  and calls RobotController.start()            │
                    └──────────────────┬──────────────────────────┘
                                       │ composes
              ┌────────────────────────┼────────────────────────────┐
              ▼                        ▼                            ▼
   robotx.control            robotx.communication          robotx.hardware /
   RobotController            RobotSocketClient              navigation / perception
   (production control loop)  (Socket.IO client,             (sensors, motors, camera,
                               robot -> external server)      routing, detection)
                                       │
                                       ▼
                         External Socket.IO server
                         (NOT part of this repository —
                          only the client-side contract
                          is implemented here)
```

---

## 2. Folder Structure

```
robotx/
├── application/     FastAPI app + composition root (entry point)
├── communication/   Socket.IO client (robot -> external server)
├── config/          Centralized environment-variable settings
├── control/         The one production control loop
├── hardware/        Direct physical I/O (GPIO, camera, serial GPS)
├── navigation/       Route planning + Google Directions client
└── perception/      Camera-frame interpretation + experimental vision pipeline

tests/hardware/      Standalone hardware-exercise scripts (motors, ultrasonic, gps, camera) — NOT automated/CI-safe
tests/control/       Standalone script exercising the STOP/FORWARD decision rule on real hardware — NOT automated/CI-safe
tests/perception/    Standalone script exercising the experimental vision pipeline on real hardware — NOT automated/CI-safe
docs/architecture/   This document + DEPENDENCY_MAP.md
```

---

## 3. Package Responsibilities

### `robotx.config` — [IMPLEMENTED]
`settings.py` defines a single frozen dataclass, `Settings`, instantiated once as `SETTINGS`. Every `ROBOTX_*` environment variable is read here and nowhere else. All other packages import `SETTINGS` rather than calling `os.environ` directly.

### `robotx.hardware` — [IMPLEMENTED]
Direct physical I/O. Every module here either drives `RPi.GPIO`/`picamera2`/a serial port, or is a no-op/mock stand-in used when that hardware library is unavailable (e.g. developing on a laptop).

| Module | Class | Device |
|---|---|---|
| `motors.py` | `MotorDriver` | L298N dual H-bridge, differential drive |
| `encoders.py` | `EncoderReader` | Single-channel wheel encoders (GPIO interrupts) |
| `ultrasonic.py` | `UltrasonicSensor` | HC-SR04, exposes `UltrasonicStatus` (VALID/TIMEOUT/OUT_OF_RANGE/ERROR/DISCONNECTED/STALE/UNKNOWN) — see R-03 |
| `ir.py` | `IRSensors` | 3x digital IR (no debounce) |
| `camera.py` | `CameraStream` | Picamera2 threaded frame capture (moved here from `perception/` in this refactor — it is device I/O, not interpretation) |
| `gps.py` | `GPSReader` | Serial NMEA-0183 parsing via `pyserial`+`pynmea2` (moved here from `navigation/` in this refactor — it is device I/O, not routing) |

Hardware modules never import `communication`, `application`, or FastAPI/Socket.IO. Motors fail safe: `MotorDriver.stop()` is called from `RobotController`'s `finally`/`except` blocks on any error or shutdown.

### `robotx.navigation` — [IMPLEMENTED]
Pure routing logic plus one external HTTP dependency. No GPIO.

| Module | Class | Responsibility |
|---|---|---|
| `route_planner.py` | `RoutePlanner`, `haversine_m()` | Waypoint tracking, off-route/reroute heuristics |
| `directions_client.py` | `GoogleMapsDirections`, `decode_polyline()` | Google Directions HTTP client, polyline decode, file-based cache + rate limit |

**[MISSING]** No geofencing/zone/campus-boundary concept — any lat/lon can be requested as a destination.

### `robotx.perception` — [IMPLEMENTED] (production path) / [EXPERIMENTAL] (vision-decision pipeline)
Camera-frame interpretation.

| Module | Class | Status |
|---|---|---|
| `object_detector.py` | `ObjectDetector` | [IMPLEMENTED] — OpenCV (MOG2 + Canny) or optional YOLOv8 backend; used by `RobotController` |
| `object_tracker.py` | `ObjectTracker`, `PrimaryObjectTracker` | [IMPLEMENTED] — IoU multi-object tracking; not used by the production controller today, used by the experimental pipeline |
| `temporal_filter.py` | `TemporalFilter`, `ActionSmoother` | [IMPLEMENTED] (`TemporalFilter`) / [UNUSED] (`ActionSmoother` has no callers anywhere) |
| `vision_controller.py` | `VisionController` | **[EXPERIMENTAL — NOT WIRED INTO PRODUCTION]**. Continuous-turning, hysteresis-based obstacle avoidance with motion-depth trend estimation. Imports only other `perception.*` modules — never `hardware` or `control`. Exercised exclusively by `tests/perception/test_vision.py`. Living in `perception/` because its dependencies were always perception-only; this is a structural placement, **not** a promotion — it is still not called by `RobotController` |
| `decision_engine.py` | `DecisionEngine` | **[EXPERIMENTAL — NOT WIRED INTO PRODUCTION]**. 36-line fixed-threshold rule engine paired with `vision_controller.py`. Same non-production status as above |

Per `ROBOTX_PI_REMEDIATION_PLAN.md` (R-05), the choice to keep `VisionController`/`DecisionEngine` as reference-only rather than promoting them into `RobotController` was a deliberate decision (different safety philosophy, no bench-test evidence for this robot's camera/mounting) — not an oversight, and not revisited by this structural refactor.

### `robotx.control` — [IMPLEMENTED], safety-critical
`robot_controller.py` — `RobotController` is the **only** production control loop, wired into `robotx.application.main` at startup. Responsibilities:
- Reads GPS/ultrasonic/IR/encoders every tick (10 Hz default), throttled camera detection (2 Hz default).
- `_safety_blocked()`: true if ultrasonic status is anything other than a fresh `VALID` in-range reading, OR any IR triggered, OR perception reports a person/obstacle. Conservative-by-design (R-03).
- Mode dispatch (`IDLE`/`AUTO`/`MANUAL`/`RETURN`/`STOPPED`/`ERROR`): `AUTO`/`RETURN` run `_avoid()` when blocked, else route-follow via `RoutePlanner`; `MANUAL` runs `_apply_manual()`, which **also** consults the same `blocked` signal — forward-class motion is refused while blocked, backward/turning remain available (R-01).
- Builds the telemetry dict and forwards it to whatever hook `robotx.application.main` registered (decoupled from the transport).
- Fails safe: motors are stopped in `except Exception` inside the loop and in `stop()`'s `finally`-style cleanup.

This module intentionally does **not** import `robotx.communication` or FastAPI — it only calls back through the injected `telemetry_hook` and receives commands through `handle_command()`, called by whatever transport the application layer wires up.

### `robotx.communication` — [PARTIALLY IMPLEMENTED — client-side only]
`socket_client.py` — `RobotSocketClient` is the one network transport in the repository: an outbound Socket.IO client.
- `connect()` sends `auth={"robot_id": ..., "token": SETTINGS.robot_token}` in the Engine.IO/Socket.IO handshake (R-02). If `ROBOTX_ROBOT_TOKEN` is unset, it logs an explicit warning and still connects (there is no server in this repo to authenticate against).
- Inbound `command`/`manual` events are queued (`asyncio.Queue`, FIFO, no dedup) and dispatched one at a time to whatever handler `robotx.application.main` registered (`RobotController.handle_command`).
- Outbound `telemetry` is drop-oldest-on-full (`maxsize=5`) — never blocks the control loop.
- `emit_status()` exists but has zero call sites anywhere in the repo — dead code, not wired to anything.

**Backend integration requirement (out of this repo's scope):** the external Socket.IO server must read the `auth` payload during its own `connect` handler and reject unknown/mismatched tokens — see `ROBOTX_PI_REMEDIATION_PLAN.md` R-02 for the exact contract this client assumes.

### `robotx.application` — [IMPLEMENTED]
`main.py` is the composition root and sole entry point:
- Defines the FastAPI `app` instance and its two routes (`/health`, `/camera`, both unauthenticated, both read-only).
- `on_startup()`: constructs every hardware/navigation/perception object from `SETTINGS`, wires them into one `RobotController`, constructs one `RobotSocketClient`, registers the telemetry hook, connects the socket (non-fatally — a failed connection does not stop the robot's local control loop), and calls `controller.start()`.
- `on_shutdown()`: stops the controller (which fails-safe-stops all hardware) and closes the socket.

This is the only file allowed to import from every other package — it is the composition root, not a library other modules import from.

---

## 4. Dependency Direction

```
config
   ^
   | (every package reads SETTINGS)
   |
hardware  navigation  perception  ← no internal deps between these three
   ^           ^           ^
   └───────────┴───────────┘
               |
            control            (RobotController: the only production consumer of all three)
               |
        communication            (independent — talks to control only via injected callbacks)
               |
          application            (composition root — the only place all packages are imported together)
```

Confirmed by import inspection: **no circular imports exist**. `hardware`, `navigation`, and `perception` never import `control`, `communication`, or `application`. `control.robot_controller` never imports `communication` or `application` directly — it receives commands and emits telemetry only through plain callables injected by `application.main`. `perception.vision_controller`/`decision_engine` (experimental) import only other `perception.*` modules.

See `DEPENDENCY_MAP.md` for the complete file-level import graph.

---

## 5. Configuration

All configuration is centralized in `robotx.config.settings.SETTINGS` (a frozen dataclass populated once at import time from `ROBOTX_*` environment variables). No module outside `robotx/config/settings.py` calls `os.environ` for application configuration. See the module itself for the full variable list (motor pins, sensor pins, socket URL/namespace/token, detection backend, control-loop tuning, etc.) — reproduced in full in `ROBOTX_DEEP_CURRENT_STATE_AUDIT.md` §33 (paths there are pre-refactor; the settings themselves are unchanged).

---

## 6. Test Structure

**[IMPLEMENTED]** Six standalone scripts, laid out to mirror the package they exercise, each directly touching real GPIO/camera/serial hardware. None are automated (no `pytest`, no assertions) and none should ever be run as part of an automated check, CI, or refactor validation, because every one of them can move motors, open the real camera, or requires a human to observe the robot.

| Script | Location | Touches |
|---|---|---|
| `test_motors.py` | `tests/hardware/` | Real motors (drives forward/backward) |
| `test_gps.py` | `tests/hardware/` | Real serial GPS |
| `test_ultrasonic.py` | `tests/hardware/` | Real HC-SR04 |
| `test_camera.py` | `tests/hardware/` | Real Picamera2 |
| `test_controller.py` | `tests/control/` | Real motors + ultrasonic + IR + encoders (independent reimplementation — does **not** exercise `RobotController`; misleadingly named, tracked as R-05, not renamed here) |
| `test_vision.py` | `tests/perception/` | Real Picamera2 + `VisionController`/`DecisionEngine` (the only exerciser of the experimental pipeline) |

`test_controller.py` and `test_vision.py` live outside `tests/hardware/` because they are organized by *what they test* (control-loop decision logic, the perception/vision pipeline) rather than by *which physical sensor they happen to touch* — both still require real hardware and must never be run automatically.

**[NOT IMPLEMENTED]** No automated, hardware-free test suite exists yet for the pure-logic code (`haversine_m`, `RoutePlanner`, `decode_polyline`, `ObjectTracker`, `DecisionEngine`). This is tracked as R-05/future work in `ROBOTX_PI_REMEDIATION_PLAN.md`. `tests/communication/` and `tests/navigation/` were not created — no test content exists for those packages yet, and empty directories were deliberately not scaffolded.

---

## 7. Production Startup

```bash
cd /home/pi/Desktop/RobotX
venv/bin/python -m uvicorn robotx.application.main:app --host 0.0.0.0 --port 8000
```

Import-only sanity check (starts no hardware threads):
```bash
venv/bin/python -c "import robotx.application.main; print('ok')"
```

Always use `venv/bin/python -m <module>` (never the `venv/bin/pip`/`venv/bin/uvicorn` shim scripts directly) — see `ROBOTX_PI_REMEDIATION_PLAN.md` R-00 for why the shims' shebangs are unreliable on this Pi.

---

## 8. Hardware Diagnostics

Run manually, one at a time, never automated:
```bash
venv/bin/python tests/hardware/test_motors.py      # SAFETY: wheels off the ground
venv/bin/python tests/hardware/test_ultrasonic.py
venv/bin/python tests/hardware/test_gps.py
venv/bin/python tests/hardware/test_camera.py
venv/bin/python tests/control/test_controller.py   # SAFETY: wheels off the ground
venv/bin/python tests/perception/test_vision.py
```
See `TEST_README.md` for expected output and troubleshooting per script.

---

## 9. Safety-Critical Paths

| Path | Where | Status |
|---|---|---|
| Ultrasonic non-VALID status blocks forward motion | `hardware/ultrasonic.py` (`UltrasonicStatus`), `control/robot_controller.py` (`_safety_blocked`) | [IMPLEMENTED] — R-03 |
| MANUAL mode obeys the same safety gate as AUTO/RETURN | `control/robot_controller.py` (`_apply_manual`) | [IMPLEMENTED, scoped] — R-01. Only forward-class motion is refused; backward/turning remain available (no rear sensor exists) |
| Socket.IO connect handshake carries a bearer token | `communication/socket_client.py` (`connect`) | [PARTIALLY IMPLEMENTED — client only] — R-02. No server in this repo verifies it |
| Motors stop on unhandled exception / shutdown | `control/robot_controller.py` (`_loop`'s `except Exception`, `stop()`) | [IMPLEMENTED] |
| Max commanded motor duty ceiling | `hardware/motors.py` (`MotorDriver.max_duty`, default 0.75) | [IMPLEMENTED] |
| Comm-loss watchdog (stale command persists after disconnect) | — | **[NOT IMPLEMENTED]** — tracked as R-06 |
| Command payload schema validation | — | **[NOT IMPLEMENTED]** — tracked as R-07 |
| Dedicated hardware e-stop | — | **[NOT IMPLEMENTED]** |
| Real battery telemetry | — | **[NOT IMPLEMENTED]** — `battery.percent` is a hardcoded `76.0`, tracked as R-04 |

None of the above statuses changed as a result of this architecture refactor — every safety-relevant method moved verbatim (diff-verified against the pre-refactor checkpoint commit) with only its containing file's import block edited.
