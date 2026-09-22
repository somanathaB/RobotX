# RobotX Pi Agent — Architecture Refactor Plan

> **HISTORICAL DOCUMENT.** This was the plan for the first restructuring pass. A second cleanup pass followed the same day (`application/app.py`→`main.py`, test scripts split by package under `tests/`) — see `ROBOTX_ARCHITECTURE_CLEANUP_REPORT.md` and `docs/architecture/ROBOTX_PI_ARCHITECTURE.md` for the current, authoritative structure.

**Date:** 2026-09-22
**Scope:** Pure architectural reorganization (folder/file/module structure, naming, import wiring).
**Explicitly out of scope:** R-04 through R-10 (battery telemetry, VisionController promotion decision, comm-loss watchdog, command schema validation, systemd, logging cleanup, `.env.example`) from `ROBOTX_PI_REMEDIATION_PLAN.md`. None of that feature/safety work is touched here. R-00 through R-03 (already implemented and verified) must remain functionally intact after this refactor — this plan only relocates the code that implements them, never rewrites its logic.

This plan was produced by reading every `.py` file in the repository (26 files, 5,284 lines), the existing `ROBOTX_DEEP_CURRENT_STATE_AUDIT.md`, `ROBOTX_PI_REMEDIATION_PLAN.md`, `README.md`, and `TEST_README.md`, and by extracting every `import`/`from` statement across the tree to build the dependency map in §3.

---

## 1. Current Structure

```
RobotX/
├── README.md
├── TEST_README.md
├── ROBOTX_DEEP_CURRENT_STATE_AUDIT.md
├── ROBOTX_PI_REMEDIATION_PLAN.md
├── requirements.txt
├── .gitignore
├── venv/
├── frame.jpg, frames/frame.jpg          # stray debug artifacts from test_cv.py/test_vision.py runs
├── test_motor.py                        # standalone hardware exerciser (real GPIO)
├── test_gps.py                          # standalone hardware exerciser (real serial)
├── test_ultrasonic.py                   # standalone hardware exerciser (real GPIO)
├── test_controller.py                   # standalone hardware exerciser (real GPIO) — does NOT test robotx.control.controller
├── test_cv.py                           # standalone hardware exerciser (real camera)
├── test_vision.py                       # standalone hardware exerciser (real camera + detector)
└── robotx/
    ├── __init__.py                      # 1-line docstring
    ├── app/
    │   ├── __init__.py                  # empty
    │   ├── main.py                      # FastAPI app + composition root (194 lines)
    │   └── sockets.py                   # Socket.IO client, robot -> external server (148 lines)
    ├── control/
    │   ├── __init__.py                  # empty
    │   ├── controller.py                # RobotController — PRODUCTION control loop (470 lines)
    │   ├── decision_engine.py           # DecisionEngine — 36-line rule engine, test-only
    │   └── vision_controller.py         # VisionController — advanced pipeline, test-only (798 lines)
    ├── hardware/
    │   ├── __init__.py                  # empty
    │   ├── motors.py                    # MotorDriver (L298N)
    │   ├── encoders.py                  # EncoderReader
    │   ├── ultrasonic.py                # UltrasonicSensor / UltrasonicStatus (R-03)
    │   └── ir.py                        # IRSensors
    ├── navigation/
    │   ├── __init__.py                  # empty
    │   ├── gps.py                       # GPSReader — serial NMEA device driver
    │   ├── maps.py                      # GoogleMapsDirections — HTTP routing client
    │   └── planner.py                   # RoutePlanner — waypoint/reroute logic
    ├── perception/
    │   ├── __init__.py                  # empty
    │   ├── camera.py                    # CameraStream — Picamera2 device driver
    │   ├── detection.py                 # ObjectDetector (OpenCV + optional YOLOv8 backends), 889 lines
    │   ├── tracking.py                  # ObjectTracker, PrimaryObjectTracker
    │   └── filter.py                    # TemporalFilter, ActionSmoother (unused)
    └── utils/
        ├── __init__.py                  # empty
        └── config.py                    # Settings dataclass / SETTINGS (all ROBOTX_* env vars)
```

No `tests/`, `scripts/`, `docs/` directories exist. No `pyproject.toml`. Git repo exists (`RobotX/.git`) but has **zero commits** — every file is currently untracked.

---

## 2. Problems With Current Structure

1. **Hardware/perception boundary violation.** `perception/camera.py` and `navigation/gps.py` are device drivers (Picamera2 capture, serial NMEA parsing) — the task's own boundary rules classify camera and GPS serial devices as `HARDWARE`, not perception/navigation. Today they sit outside `hardware/`, so "where do I change camera capture?" doesn't point at `hardware/` the way it should for every other sensor.
2. **Vision/control boundary violation.** `control/vision_controller.py` and `control/decision_engine.py` import only `robotx.perception.*` (confirmed by grep — zero imports of `hardware`, zero imports of `control`'s own `controller.py`) and are exercised only by `test_vision.py`. They are perception-decision code, not control code, and are physically misplaced in `control/`.
3. **Vague/mismatched names.** `app/main.py` is really "the application" (composition root + FastAPI wiring), not just "main". `app/sockets.py` is specifically the *Socket.IO client* — communication, not generic "sockets". `utils/config.py` is the *only* file in `utils/` and contains only settings — `utils` as a catch-all name doesn't reflect that. `control/controller.py`'s class is `RobotController` — the file name doesn't say which controller once more than one exists in the package. `navigation/maps.py` is really a Google Directions HTTP client, not generic "maps". `navigation/planner.py`'s class is `RoutePlanner`. `perception/detection.py`'s class is `ObjectDetector`. `perception/tracking.py`'s primary class is `ObjectTracker`. `perception/filter.py`'s primary class is `TemporalFilter`.
4. **Misleading test file.** `test_controller.py` does not import or exercise `robotx.control.controller.RobotController` at all — it's an independent reimplementation of a simpler decision rule (already flagged as R-05 in the remediation plan, not yet fixed).
5. **No test/diagnostic organization.** All six standalone hardware-exercise scripts sit at repo root, indistinguishable at a glance from application source, and are not marked as hardware-touching.
6. **No architecture documentation.** Nothing describes package responsibilities, dependency direction, or which modules are safety-critical versus experimental/unwired, beyond prose buried in the two audit documents.
7. **Stray debug artifacts** (`frame.jpg`, `frames/frame.jpg`) committed at repo root from prior manual `test_cv.py`/`test_vision.py` runs.

---

## 3. Dependency Map (as it exists today)

```
robotx/utils/config.py                    (SETTINGS)  — no internal deps
    ^
    | imported by every file below that needs configuration
    |
robotx/hardware/motors.py                 (MotorDriver, MotorPins)         — no internal deps
robotx/hardware/encoders.py               (EncoderReader, EncoderConfig)   — no internal deps
robotx/hardware/ultrasonic.py             (UltrasonicSensor, UltrasonicStatus, UltrasonicReading) — no internal deps
robotx/hardware/ir.py                     (IRSensors, IRConfig)            — no internal deps
robotx/navigation/gps.py                  (GPSReader, GPSConfig)           — no internal deps [DEVICE DRIVER, misplaced — see §2.1]
robotx/navigation/maps.py                 (GoogleMapsDirections, decode_polyline) — no internal deps
robotx/navigation/planner.py              (RoutePlanner, haversine_m)      — no internal deps
robotx/perception/camera.py               (CameraStream)                  — no internal deps [DEVICE DRIVER, misplaced — see §2.1]
robotx/perception/tracking.py             (ObjectTracker, PrimaryObjectTracker) — no internal deps
robotx/perception/detection.py            (ObjectDetector, summarize_detections) — no internal deps
robotx/perception/filter.py               (TemporalFilter, ActionSmoother) -> imports perception/tracking.py (Track)

robotx/control/decision_engine.py         (DecisionEngine) — no internal deps
robotx/control/vision_controller.py       (VisionController) -> imports perception/camera.py, perception/detection.py,
                                                                  perception/filter.py, perception/tracking.py
                                                                  [test-only; NOT imported by controller.py or main.py]

robotx/control/controller.py              (RobotController) -> imports hardware/{motors,encoders,ultrasonic,ir}.py,
                                                                  navigation/{gps,maps,planner}.py,
                                                                  perception/{camera,detection}.py
                                                                  [PRODUCTION — wired into app/main.py]

robotx/app/sockets.py                     (RobotSocketClient, SocketConfig) — no internal deps (external: python-socketio)

robotx/app/main.py                        (FastAPI app, on_startup/on_shutdown) -> imports
                                                                  app/sockets.py, control/controller.py,
                                                                  hardware/*, navigation/*, perception/*, utils/config.py
                                                                  [ENTRY POINT — composition root]

test_motor.py, test_gps.py, test_ultrasonic.py, test_controller.py
    -> import (inside main(), not at module scope) robotx.hardware.*, robotx.navigation.gps, robotx.utils.config
    [standalone; do not import app/main.py, control/controller.py, or app/sockets.py]

test_cv.py
    -> imports (inside functions) robotx.perception.camera, robotx.perception.detection, robotx.utils.config

test_vision.py
    -> imports (inside functions) robotx.control.vision_controller, robotx.control.decision_engine,
                                    robotx.perception.filter, robotx.perception.tracking
    [the ONLY exerciser of VisionController/DecisionEngine]
```

**Confirmed:** no circular imports anywhere. Layering today is already one-directional: `utils → hardware/navigation/perception → control → app`. `control/controller.py` and `control/vision_controller.py` never import each other. Nothing in `hardware/`, `navigation/`, or `perception/` imports `control/` or `app/`.

---

## 4. Proposed Structure

```
RobotX/
├── robotx/
│   ├── __init__.py                          [unchanged]
│   │
│   ├── application/                          (was: app/)
│   │   ├── __init__.py
│   │   └── app.py                            (was: app/main.py) — FastAPI app, composition root, lifecycle
│   │
│   ├── communication/                        (was: app/sockets.py, promoted to its own package)
│   │   ├── __init__.py
│   │   └── socket_client.py                  (was: app/sockets.py) — RobotSocketClient, SocketConfig
│   │
│   ├── config/                               (was: utils/)
│   │   ├── __init__.py
│   │   └── settings.py                       (was: utils/config.py) — Settings, SETTINGS
│   │
│   ├── hardware/
│   │   ├── __init__.py
│   │   ├── motors.py                         [unchanged name/content]
│   │   ├── encoders.py                       [unchanged name/content]
│   │   ├── ultrasonic.py                     [unchanged name/content — R-03 status model stays as-is]
│   │   ├── ir.py                             [unchanged name/content]
│   │   ├── camera.py                         (was: perception/camera.py) — device driver, moved per HARDWARE boundary
│   │   └── gps.py                            (was: navigation/gps.py) — device driver, moved per HARDWARE boundary
│   │
│   ├── navigation/
│   │   ├── __init__.py
│   │   ├── route_planner.py                  (was: navigation/planner.py) — RoutePlanner, haversine_m
│   │   └── directions_client.py              (was: navigation/maps.py) — GoogleMapsDirections, decode_polyline
│   │
│   ├── perception/
│   │   ├── __init__.py
│   │   ├── object_detector.py                (was: perception/detection.py) — ObjectDetector (OpenCV + YOLOv8)
│   │   ├── object_tracker.py                 (was: perception/tracking.py) — ObjectTracker, PrimaryObjectTracker
│   │   ├── temporal_filter.py                (was: perception/filter.py) — TemporalFilter, ActionSmoother
│   │   ├── vision_controller.py              (was: control/vision_controller.py) — EXPERIMENTAL, test-only, unchanged content
│   │   └── decision_engine.py                (was: control/decision_engine.py) — EXPERIMENTAL, test-only, unchanged content
│   │
│   └── control/
│       ├── __init__.py
│       └── robot_controller.py               (was: control/controller.py) — RobotController, PRODUCTION control loop
│
├── tests/
│   └── hardware/                              (all six scripts — NONE are automated/CI-safe; all touch real hardware)
│       ├── __init__.py
│       ├── test_motor.py                      [unchanged content — moved only]
│       ├── test_gps.py                        [unchanged content — moved only]
│       ├── test_ultrasonic.py                 [unchanged content — moved only]
│       ├── test_controller.py                 [unchanged content — moved only; still misleadingly named per R-05, NOT renamed here]
│       ├── test_cv.py                         [unchanged content — moved only]
│       └── test_vision.py                     [unchanged content — moved only]
│
├── docs/
│   └── architecture/
│       ├── ROBOTX_PI_ARCHITECTURE.md          (new)
│       └── DEPENDENCY_MAP.md                  (new)
│
├── deployment/                                 [NOT created — no systemd unit exists yet (R-08, deferred); nothing to hold]
├── scripts/                                    [NOT created — the six hardware scripts already live in tests/hardware/; no separate diagnostic-tooling class exists]
├── ROBOTX_PI_ARCHITECTURE_REFACTOR_PLAN.md    (this file)
├── ROBOTX_ARCHITECTURE_REFACTOR_REPORT.md     (new, written after execution)
├── ROBOTX_DEEP_CURRENT_STATE_AUDIT.md         [kept as historical record, banner note added pointing at new paths]
├── ROBOTX_PI_REMEDIATION_PLAN.md              [kept, file:line references updated to new paths/filenames]
├── README.md                                  [updated: folder tree, run command, wiring table unchanged]
├── TEST_README.md                             [updated: script paths -> tests/hardware/*]
├── requirements.txt                           [unchanged]
├── .gitignore                                 [unchanged, or extended to ignore frame.jpg/frames/ — see §11]
└── venv/                                      [untouched — R-00 already fixed this]
```

### Deliberately NOT done (over-engineering guardrails)

- **No `hardware/motors/` or `hardware/sensors/` subpackages.** The prompt's target tree nests motors/sensors/camera/gps into subpackages, but `hardware/` will hold 6 flat files after this move (motors, encoders, ultrasonic, ir, camera, gps) — flat is more legible than nesting single-file "packages" for this file count. Revisit only if the hardware layer grows materially (e.g., a real battery driver is added later per R-04).
- **No `perception/detectors/` or `perception/tracking/` subpackages.** `object_detector.py` (889 lines) internally contains both the OpenCV and YOLOv8 backends behind one `ObjectDetector` class with a `backend=` switch. Splitting it into `detectors/opencv_detector.py` + `detectors/yolo_detector.py` is a real code change (not a pure move) to a safety-relevant, currently-untested file — flagged in §11 as a candidate for a **future, separate, behavior-preserving pass**, not done in this structural refactor.
- **No `control/safety_controller.py`.** The remediation plan's R-01 entry explicitly records that a centralized `SafetyController` module was proposed and then **deliberately deferred "per explicit scope instruction"** in favor of a targeted gate inside `_apply_manual()`. This refactor does not revisit that decision — `_safety_blocked()`/`_avoid()`/`_apply_manual()` move together, verbatim, inside `robot_controller.py`. Extracting them into a new module would be indistinguishable from re-opening R-01's deferred design change, which is feature/safety work, not structural relocation. Flagged in §11 as a decision for a future, explicit approval — not assumed here.
- **No `missions/` or `telemetry/` packages.** No mission-management or dedicated telemetry-service code exists today — telemetry construction is ~20 lines inline in `RobotController._loop()` and "mission" is just a destination lat/lon. Creating these packages now would be empty ceremony.
- **No `docs/hardware/`, `docs/communication/`, `docs/operations/`.** `README.md`, `TEST_README.md`, and `ROBOTX_PI_REMEDIATION_PLAN.md` already cover this content at repo root; splitting them into `docs/` subfolders with no new content would be reorganization for its own sake. Only `docs/architecture/` is created, because it holds two genuinely new documents.
- **No `application/lifecycle.py` / `application/dependencies.py` split.** `app/main.py` is 194 lines with one composition function (`on_startup`) and two routes. Splitting it into `app.py` + `lifecycle.py` + `dependencies.py` (3 files for what's currently ~150 lines of real logic) doesn't earn its complexity yet. Kept as one `application/app.py`.

---

## 5. File-by-File Migration Table

| # | Old path | New path | Rename reason | Class/function moved |
|---|---|---|---|---|
| 1 | `robotx/utils/config.py` | `robotx/config/settings.py` | `utils` is a banned vague name; this is the settings module | `Settings`, `SETTINGS` |
| 2 | `robotx/utils/__init__.py` | *(deleted, package removed)* | package renamed to `config/` | — |
| 3 | `robotx/hardware/motors.py` | `robotx/hardware/motors.py` | unchanged — already accurate | `MotorDriver`, `MotorPins` |
| 4 | `robotx/hardware/encoders.py` | `robotx/hardware/encoders.py` | unchanged | `EncoderReader`, `EncoderConfig` |
| 5 | `robotx/hardware/ultrasonic.py` | `robotx/hardware/ultrasonic.py` | unchanged (R-03 status model untouched) | `UltrasonicSensor`, `UltrasonicStatus`, `UltrasonicReading` |
| 6 | `robotx/hardware/ir.py` | `robotx/hardware/ir.py` | unchanged | `IRSensors`, `IRConfig` |
| 7 | `robotx/perception/camera.py` | `robotx/hardware/camera.py` | HARDWARE boundary: Picamera2 device I/O belongs in `hardware/`, per task's own boundary rules | `CameraStream` |
| 8 | `robotx/navigation/gps.py` | `robotx/hardware/gps.py` | HARDWARE boundary: serial NMEA device I/O belongs in `hardware/` | `GPSReader`, `GPSConfig` |
| 9 | `robotx/navigation/planner.py` | `robotx/navigation/route_planner.py` | matches naming-rules example (`routing.py` → `route_planner.py`); disambiguates from the class name | `RoutePlanner`, `haversine_m` |
| 10 | `robotx/navigation/maps.py` | `robotx/navigation/directions_client.py` | "maps" is vague; this is specifically a Google Directions HTTP client | `GoogleMapsDirections`, `decode_polyline` |
| 11 | `robotx/perception/detection.py` | `robotx/perception/object_detector.py` | matches primary class `ObjectDetector` | `ObjectDetector`, `summarize_detections`, `draw_detections` |
| 12 | `robotx/perception/tracking.py` | `robotx/perception/object_tracker.py` | matches naming-rules example (`tracking.py` → `object_tracker.py`) | `ObjectTracker`, `PrimaryObjectTracker`, `Track`, `PrimaryTrack` |
| 13 | `robotx/perception/filter.py` | `robotx/perception/temporal_filter.py` | "filter" is vague; matches primary class `TemporalFilter` | `TemporalFilter`, `ActionSmoother` |
| 14 | `robotx/control/vision_controller.py` | `robotx/perception/vision_controller.py` | PERCEPTION boundary: imports only `perception.*`, never `hardware`/`control`; it is vision decision-making, not motor control. Content unchanged — remains test-only/experimental, still not imported by `robot_controller.py` or `app.py` | `VisionController`, `VisionControllerConfig`, `draw_avoidance_debug` |
| 15 | `robotx/control/decision_engine.py` | `robotx/perception/decision_engine.py` | same rationale as #14 — zero internal imports, pure pixel-rule logic paired with `vision_controller.py` | `DecisionEngine` |
| 16 | `robotx/control/controller.py` | `robotx/control/robot_controller.py` | matches naming-rules example (`controller.py` → `robot_controller.py`); disambiguates from `perception/vision_controller.py` now sitting one package over | `RobotController`, `ControllerConfig`, `bearing_rad` |
| 17 | `robotx/app/sockets.py` | `robotx/communication/socket_client.py` | matches naming-rules example (`sockets.py` → `socket_client.py`); COMMUNICATION boundary — this is the only network transport in the repo, promoted out of `app/` into its own package | `RobotSocketClient`, `SocketConfig` |
| 18 | `robotx/app/main.py` | `robotx/application/app.py` | `app/` renamed to `application/` (avoid clashing with the `app` FastAPI variable name / module name collision); composition root | `app` (FastAPI instance), `on_startup`, `on_shutdown`, routes |
| 19 | `robotx/app/__init__.py` | *(deleted, package renamed)* | — | — |
| 20 | `test_motor.py` | `tests/hardware/test_motor.py` | relocated only — real GPIO, not CI-safe | — |
| 21 | `test_gps.py` | `tests/hardware/test_gps.py` | relocated only — real serial device | — |
| 22 | `test_ultrasonic.py` | `tests/hardware/test_ultrasonic.py` | relocated only — real GPIO | — |
| 23 | `test_controller.py` | `tests/hardware/test_controller.py` | relocated only. **Not renamed** despite being misleadingly named (doesn't test `RobotController`) — that rename is already tracked as R-05 in the remediation plan with its own rationale; renaming here would preempt that decision outside this refactor's scope | — |
| 24 | `test_cv.py` | `tests/hardware/test_cv.py` | relocated only — real camera | — |
| 25 | `test_vision.py` | `tests/hardware/test_vision.py` | relocated only — real camera + detector | — |
| 26 | `frame.jpg`, `frames/frame.jpg` | *(deleted)* | stray debug artifacts from manual test runs, not source; regenerated on demand by `test_cv.py`/`test_vision.py`. See §11 for confirmation before deletion. | — |

**Untouched files:** `README.md`, `TEST_README.md`, `ROBOTX_PI_REMEDIATION_PLAN.md` (content updated in place, not moved), `ROBOTX_DEEP_CURRENT_STATE_AUDIT.md` (banner added, not moved), `requirements.txt`, `.gitignore`, `venv/`.

---

## 6. Responsibility of Every Resulting Module

| Package | Responsibility | Must NOT contain |
|---|---|---|
| `robotx.config` | Centralized `ROBOTX_*` environment-variable configuration (`Settings`/`SETTINGS`) | Any logic that reads env vars outside this module |
| `robotx.hardware` | Direct physical I/O: GPIO motor/encoder/ultrasonic/IR control, Picamera2 capture, serial GPS NMEA reads. Fails safe on exception; mock/no-op fallback when GPIO/camera absent | Socket.IO, FastAPI, mission orchestration, routing math |
| `robotx.navigation` | Route planning math (`RoutePlanner`, haversine/bearing), Google Directions HTTP client. Pure logic + one external HTTP dependency | GPIO, camera, motor commands |
| `robotx.perception` | Camera-frame interpretation: object detection (OpenCV/YOLO), multi-object tracking, temporal filtering, and the experimental vision-decision pipeline (`vision_controller.py`/`decision_engine.py`, explicitly test-only) | Socket.IO transport, direct motor commands, GPIO |
| `robotx.control` | The one production control loop (`RobotController`): reads hardware/navigation/perception, decides mode logic, applies safety gates, calls `motors.set_speed()`, builds telemetry | Socket.IO/FastAPI wiring |
| `robotx.communication` | Socket.IO client transport: connect/auth handshake (R-02), inbound command queueing, outbound telemetry emission | Motor-control algorithms, sensor reads |
| `robotx.application` | FastAPI app instance, HTTP routes (`/health`, `/camera`), startup/shutdown composition root that wires every subsystem together | Business logic belonging to control/perception/navigation |

---

## 7. Import/Dependency Changes Required

Every import of a moved/renamed module must be updated. Full list of edit sites:

| File (new location) | Old import | New import |
|---|---|---|
| `robotx/application/app.py` | `from robotx.app.sockets import RobotSocketClient, SocketConfig` | `from robotx.communication.socket_client import RobotSocketClient, SocketConfig` |
| `robotx/application/app.py` | `from robotx.control.controller import ControllerConfig, RobotController` | `from robotx.control.robot_controller import ControllerConfig, RobotController` |
| `robotx/application/app.py` | `from robotx.hardware.encoders import EncoderConfig, EncoderReader` | unchanged |
| `robotx/application/app.py` | `from robotx.hardware.ir import IRConfig, IRSensors` | unchanged |
| `robotx/application/app.py` | `from robotx.hardware.motors import MotorDriver, MotorPins` | unchanged |
| `robotx/application/app.py` | `from robotx.hardware.ultrasonic import UltrasonicConfig, UltrasonicSensor` | unchanged |
| `robotx/application/app.py` | `from robotx.navigation.gps import GPSConfig, GPSReader` | `from robotx.hardware.gps import GPSConfig, GPSReader` |
| `robotx/application/app.py` | `from robotx.navigation.maps import GoogleMapsDirections` | `from robotx.navigation.directions_client import GoogleMapsDirections` |
| `robotx/application/app.py` | `from robotx.navigation.planner import PlannerConfig, RoutePlanner` | `from robotx.navigation.route_planner import PlannerConfig, RoutePlanner` |
| `robotx/application/app.py` | `from robotx.perception.camera import CameraStream` | `from robotx.hardware.camera import CameraStream` |
| `robotx/application/app.py` | `from robotx.perception.detection import ObjectDetector` | `from robotx.perception.object_detector import ObjectDetector` |
| `robotx/application/app.py` | `from robotx.utils.config import SETTINGS` | `from robotx.config.settings import SETTINGS` |
| `robotx/control/robot_controller.py` | `from robotx.hardware.encoders import EncoderReader` | unchanged |
| `robotx/control/robot_controller.py` | `from robotx.hardware.ir import IRSensors` | unchanged |
| `robotx/control/robot_controller.py` | `from robotx.hardware.motors import MotorDriver` | unchanged |
| `robotx/control/robot_controller.py` | `from robotx.hardware.ultrasonic import UltrasonicReading, UltrasonicSensor, UltrasonicStatus` | unchanged |
| `robotx/control/robot_controller.py` | `from robotx.navigation.gps import GPSReader` | `from robotx.hardware.gps import GPSReader` |
| `robotx/control/robot_controller.py` | `from robotx.navigation.maps import GoogleMapsDirections` | `from robotx.navigation.directions_client import GoogleMapsDirections` |
| `robotx/control/robot_controller.py` | `from robotx.navigation.planner import LatLon, RoutePlanner, haversine_m` | `from robotx.navigation.route_planner import LatLon, RoutePlanner, haversine_m` |
| `robotx/control/robot_controller.py` | `from robotx.perception.camera import CameraStream` | `from robotx.hardware.camera import CameraStream` |
| `robotx/control/robot_controller.py` | `from robotx.perception.detection import ObjectDetector, summarize_detections` | `from robotx.perception.object_detector import ObjectDetector, summarize_detections` |
| `robotx/perception/vision_controller.py` | `from robotx.perception.camera import CameraStream` | unchanged (same package now) |
| `robotx/perception/vision_controller.py` | `from robotx.perception.detection import ObjectDetector` | `from robotx.perception.object_detector import ObjectDetector` |
| `robotx/perception/vision_controller.py` | `from robotx.perception.filter import TemporalFilter, TemporalFilterConfig` | `from robotx.perception.temporal_filter import TemporalFilter, TemporalFilterConfig` |
| `robotx/perception/vision_controller.py` | `from robotx.perception.tracking import ObjectTracker, PrimaryObjectTracker, PrimaryTrack, Track` | `from robotx.perception.object_tracker import ObjectTracker, PrimaryObjectTracker, PrimaryTrack, Track` |
| `robotx/perception/temporal_filter.py` | `from robotx.perception.tracking import Track` | `from robotx.perception.object_tracker import Track` |
| `tests/hardware/test_motor.py` | `from robotx.hardware.motors import ...` / `from robotx.utils.config import SETTINGS` | motors import unchanged; `from robotx.config.settings import SETTINGS` |
| `tests/hardware/test_gps.py` | `from robotx.navigation.gps import ...` / `from robotx.utils.config import SETTINGS` | `from robotx.hardware.gps import GPSConfig, GPSReader`; `from robotx.config.settings import SETTINGS` |
| `tests/hardware/test_ultrasonic.py` | `from robotx.hardware.ultrasonic import ...` / `from robotx.utils.config import SETTINGS` | ultrasonic import unchanged; `from robotx.config.settings import SETTINGS` |
| `tests/hardware/test_controller.py` | `from robotx.hardware.motors/ultrasonic/ir/encoders import ...` / `from robotx.utils.config import SETTINGS` | hardware imports unchanged; `from robotx.config.settings import SETTINGS` |
| `tests/hardware/test_cv.py` | `from robotx.utils.config import SETTINGS`, `from robotx.perception.detection import ObjectDetector`, `from robotx.perception.camera import CameraStream` | `from robotx.config.settings import SETTINGS`; `from robotx.perception.object_detector import ObjectDetector`; `from robotx.hardware.camera import CameraStream` |
| `tests/hardware/test_vision.py` | `from robotx.control.vision_controller import VisionController, VisionControllerConfig` / `from robotx.control.decision_engine import DecisionEngine` / `from robotx.perception.filter import ActionSmoother` / `from robotx.perception.tracking import PrimaryObjectTracker` / `from robotx.control.vision_controller import draw_avoidance_debug` | `from robotx.perception.vision_controller import VisionController, VisionControllerConfig, draw_avoidance_debug`; `from robotx.perception.decision_engine import DecisionEngine`; `from robotx.perception.temporal_filter import ActionSmoother`; `from robotx.perception.object_tracker import PrimaryObjectTracker` |

No changes to third-party imports (`fastapi`, `socketio`, `RPi.GPIO`, `picamera2`, `cv2`, `pyserial`, `pynmea2`, `httpx`) anywhere.

---

## 8. Entry Points

**Before:**
```
venv/bin/python -m uvicorn robotx.app.main:app --host 0.0.0.0 --port 8000
```

**After:**
```
venv/bin/python -m uvicorn robotx.application.app:app --host 0.0.0.0 --port 8000
```

Import check (updated):
```
venv/bin/python -c "import robotx.application.app; print('ok')"
```

The `venv/bin/python -m <module>` invocation form (R-00's fix) is unaffected by this refactor — it only depends on `venv/bin/python` being executable, not on any specific module path.

---

## 9. Test Migration Plan

Current state: **zero automated tests exist** (confirmed by audit §31 — no `pytest`, no `unittest`, no assertions anywhere). All six scripts are manual, human-monitored, hardware-touching exercisers.

This refactor:
- Creates `tests/hardware/` and moves all six scripts there verbatim (content unchanged, only their internal `robotx.*` imports updated per §7).
- Adds `tests/hardware/__init__.py` (empty) so the scripts are importable as a package if ever needed — but they remain intended for direct `python tests/hardware/test_X.py` invocation, matching current usage.
- Does **not** create `tests/unit/`, `tests/integration/`, or `tests/fixtures/` — there is no unit/integration test content to put in them today, and empty directories are explicitly disallowed by the task's own instructions. Building a real `pytest` suite for the pure-logic pieces (`haversine_m`, `RoutePlanner`, `decode_polyline`, `ObjectTracker`, `DecisionEngine`) is tracked as R-05/future work in the remediation plan, not part of this structural refactor.
- Updates `TEST_README.md`'s `cd`/`python <script>.py` instructions to `python tests/hardware/<script>.py`.

**No test in this repository moves the robot as part of this refactor.** The migration only changes file paths and import lines inside these scripts; it does not execute them.

---

## 10. Risks

| Risk | Mitigation |
|---|---|
| A missed import update breaks `robotx.application.app` at startup | Run `python_compile`-equivalent (`python -m py_compile`) on every file, then `import robotx.application.app` as a syntax+wiring check, after every subsystem move (§12) |
| Renaming `robot_controller.py`'s safety-critical methods accidentally changes behavior | Methods are moved **verbatim** — no line inside `_safety_blocked`/`_avoid`/`_apply_manual`/`handle_command` is edited, only the file's own path and its imports change |
| `venv/bin/python` stops working after moving files | Nothing under `venv/` is touched; R-00's fix (`chmod +x`) is independent of source file locations |
| Stale references to old module paths left in docs or code | Full-repo grep for `robotx.app.`, `robotx.utils.`, `robotx.control.vision_controller`, `robotx.control.decision_engine`, `robotx.perception.camera`, `robotx.perception.detection`, `robotx.perception.tracking`, `robotx.perception.filter`, `robotx.navigation.gps`, `robotx.navigation.maps`, `robotx.navigation.planner` after all moves complete (§ validation) |
| Deleting `frame.jpg`/`frames/frame.jpg` removes something the user wanted kept | These are confirmed-regeneratable debug artifacts (written by `test_cv.py`/`test_vision.py` on every run); confirmed via audit as debug-only, not referenced by any code path that reads them back. Deleted only via `git rm`, fully recoverable from git history once committed |
| `git mv` on an uncommitted, un-init'd-history repo loses the "before" state | An initial checkpoint commit of the current tree (§13) is made *before* any move, so `git diff`/`git revert` against that commit is always available |

---

## 11. Flagged Decisions (not executed automatically — noted for visibility)

1. **`perception/object_detector.py` backend split.** The file is 889 lines implementing both an OpenCV backend and an optional YOLOv8 backend behind one class. Splitting into `perception/detectors/opencv_detector.py` + `yolo_detector.py` (matching the prompt's suggested tree) is a genuine code edit to safety-relevant, zero-test-coverage code — deferred to a dedicated, separately-reviewed pass, not bundled into this move-only refactor.
2. **`control/safety_controller.py` extraction.** Not created. See §4's "Deliberately NOT done" — this would re-open a design decision the remediation plan (R-01) already recorded as explicitly deferred. If a centralized safety-gate module is wanted, it should be its own approved change, not a side effect of a folder reorganization.
3. **`test_controller.py` rename.** Not renamed despite being misleadingly named (R-05, already tracked separately). Only relocated.
4. **`frame.jpg` / `frames/frame.jpg` deletion.** These will be removed as stray debug artifacts (§5, row 26) unless the user indicates otherwise before execution.

---

## 12. Migration Execution Order

Matches the task's own recommended phase order, adapted to what actually exists:

1. **Checkpoint commit** — commit the current, unmodified tree so every subsequent change is diffable/revertable.
2. **Configuration** — `utils/` → `config/` (1 file, zero fan-in risk beyond import lines).
3. **Hardware** — move `camera.py`/`gps.py` into `hardware/`; update the two files that import them (`app/main.py`, `control/controller.py`).
4. **Perception** — rename `detection.py`/`tracking.py`/`filter.py`; move `vision_controller.py`/`decision_engine.py` in from `control/`.
5. **Navigation** — rename `planner.py` → `route_planner.py`, `maps.py` → `directions_client.py`.
6. **Control** — rename `controller.py` → `robot_controller.py`.
7. **Communication** — move `app/sockets.py` → `communication/socket_client.py`.
8. **Application** — move `app/main.py` → `application/app.py`; update every import listed in §7.
9. **Tests** — move all six root-level scripts into `tests/hardware/`, update their imports.
10. **Documentation** — update `README.md`, `TEST_README.md`, `ROBOTX_PI_REMEDIATION_PLAN.md` file:line references; add banner to `ROBOTX_DEEP_CURRENT_STATE_AUDIT.md`; write `docs/architecture/ROBOTX_PI_ARCHITECTURE.md` and `docs/architecture/DEPENDENCY_MAP.md`.

After each numbered step: `python -m py_compile` every touched file, `grep` for stale old-path imports across the whole tree, and (from step 8 onward) `venv/bin/python -c "import robotx.application.app; print('ok')"`.

---

## 13. Rollback Strategy

- Step 1 of execution is a git commit of the exact current tree (nothing moved yet). Every later step is its own commit (or a small group of commits per subsystem), so `git revert <sha>` or `git reset --soft <sha>` (never `--hard` without explicit confirmation) can undo any single subsystem's move independently.
- No destructive commands (`git reset --hard`, `git clean -fd`, `rm -rf`) will be used at any point; file removals go through `git rm` so they remain recoverable from history.
- Because the venv, hardware wiring, safety thresholds, and Socket.IO auth payload are never edited by this plan, rollback of the *structural* changes cannot regress R-00 through R-03 — those live entirely inside file content that only moves, never changes.

---

## 14. Post-Refactor Validation Checklist

1. `find . -name "*.py" -not -path "./venv/*" | xargs -n1 python3 -m py_compile` — all files compile.
2. `grep -rn "robotx\.app\.\|robotx\.utils\.\|robotx\.control\.vision_controller\|robotx\.control\.decision_engine\|robotx\.perception\.camera\|robotx\.perception\.detection\b\|robotx\.perception\.tracking\|robotx\.perception\.filter\|robotx\.navigation\.gps\|robotx\.navigation\.maps\|robotx\.navigation\.planner" . --include=*.py --include=*.md` — zero stale references outside historical-audit documents (which are explicitly banner-noted as historical).
3. `venv/bin/python -c "import robotx.application.app; print('ok')"` succeeds (import-only, starts no hardware threads — `on_startup` only runs under uvicorn).
4. Re-verify R-00: `venv/bin/python` still has the execute bit and is unmodified.
5. Re-verify R-01: `_apply_manual()` inside `robot_controller.py` is byte-for-byte identical to the pre-move version (diffed against the checkpoint commit) except for its containing file's path.
6. Re-verify R-02: `communication/socket_client.py`'s `connect()` still builds `auth={"robot_id":..., "token":...}` identically.
7. Re-verify R-03: `hardware/ultrasonic.py`'s `UltrasonicStatus` enum and `robot_controller.py`'s non-VALID-blocks logic are unchanged.
8. `TaskStop`/manual review: no `tests/hardware/*.py` script was executed automatically as part of validation (would move motors) — each is listed under "Manual Hardware Verification Required" in the final report instead.

---

## Next Step

Proceed to execution per §12, starting with the checkpoint commit, unless the flagged decisions in §11 need a different resolution first.
