# RobotX Deep Current-State Audit

**Audit date:** 2026-09-22
**Repository:** `/home/pi/Desktop/RobotX`
**Method:** Full manual source inspection. Every `.py` file in the repository (26 files, ~5,166 lines, excluding `venv/` and `__pycache__/`) was read in its entirety. No documentation, prior conversation, or filename was taken at face value — conclusions below are traced to specific lines of code.

---

## 1. Executive Summary

**RobotX, as it exists on disk today, is a single-robot Raspberry Pi control stack written in Python**, not the multi-tenant Node/Express/Prisma/Socket.IO fleet-management platform (with an assignment engine, DTARO optimizer, campus/zone isolation, database-backed task pipeline, etc.) that a generic "RobotX" audit brief might assume. **That entire class of system does not exist anywhere in this repository.** There is no `package.json`, no Prisma schema, no database, no Redis, no server-side Socket.IO hub, no web frontend, no multi-robot assignment logic, and no user/campus/task data model of any kind.

What actually exists is:

- A **FastAPI service** (`robotx/app/main.py`) that runs directly on the robot's Raspberry Pi, exposing `/health` and an MJPEG `/camera` stream.
- A **Socket.IO *client*** (`robotx/app/sockets.py`) that connects *outbound* from the robot to some external, not-included server, to send telemetry and receive four command types (`START`, `STOP`, `RETURN`, `MANUAL`).
- A **hardware abstraction layer** for an L298N-driven differential-drive chassis: motors, wheel encoders, one HC-SR04 ultrasonic sensor, three digital IR sensors — all using `RPi.GPIO`, with mock fallbacks when GPIO is unavailable.
- A **navigation stack**: a serial NMEA GPS reader, a Google Directions API client with caching/rate-limiting, and a waypoint-following/reroute planner using haversine math.
- A **perception stack**: a Picamera2-based camera reader, and an object detector with two backends (OpenCV background-subtraction/edge-detection, or optional YOLOv8), plus an IoU-based multi-object tracker and temporal filters.
- **Two independent, non-interoperating control brains**: the production `RobotController` (wired into the FastAPI app) which uses simple threshold-based obstacle avoidance, and a much more sophisticated `VisionController` + `DecisionEngine` (continuous turning, hysteresis, motion-depth trend estimation) that is **only ever imported by a standalone test script** (`test_vision.py`) — it is not reachable from the running application at all.
- **Five standalone hardware test scripts** (`test_motor.py`, `test_gps.py`, `test_ultrasonic.py`, `test_controller.py`, `test_cv.py`) that exercise real hardware modules directly, without the FastAPI app or Socket.IO.

There is **no ESP32 anywhere in this codebase** — search results for "ESP32" produced zero matches. The Pi's own GPIO pins drive the L298N motor driver, ultrasonic sensor, and IR sensors directly; there is no intermediate microcontroller in the current design at all. GPS is read directly by the Pi over a UART serial port.

**Physical-hardware readiness is genuinely high for a single robot** — the motor, encoder, ultrasonic, IR, and camera drivers are real GPIO/Picamera2 code, not simulations, and they fail safe (motors stop on exceptions). But several things are placeholder or missing: **battery telemetry is a hardcoded constant (`76.0`)**, there is no authentication anywhere in the Socket.IO client, there is no dedicated hardware emergency-stop, and the currently-installed Python virtual environment (`venv/`) is **not executable** (`venv/bin/python` has mode `664`, no execute bit — the documented run command would fail as-is).

There is no "simulation system" in the sense the audit brief anticipates (no `VirtualRobot`, no fleet simulator) — the closest analog is the `_MockGPIO`/`_MockPWM` classes in `motors.py`, which exist purely so the code can be imported on non-Pi machines without crashing; they are not a simulation *environment*, just a no-op stand-in for missing GPIO.

---

## 2. Audit Scope and Method

- Every source file under `/home/pi/Desktop/RobotX` (excluding `venv/`, `__pycache__/`, `.git/`, `frames/`) was read fully with a file-reading tool — not sampled, not grepped-and-guessed.
- Cross-references (imports, call sites) were verified with `grep -rn` across the whole tree to determine what is actually wired together versus merely present.
- Runtime environment was inspected directly on the Pi: `git log`/`git status`, `venv/` package listing and file permissions, and Python import checks (`RPi.GPIO`, `picamera2`, `cv2`, `fastapi`, `socketio`, etc.) against both the system Python and the project venv.
- No code was modified. No dependencies were installed. No database or migration exists to inspect (there is no database).
- Sections of the requested 44-section template that describe infrastructure absent from this repository (Prisma, Redis, DTARO, campus isolation, assignment engine, PM2/Docker, multi-robot fleet, dashboard/analytics) are explicitly marked **[MISSING]** rather than described hypothetically. Where the template's section headings don't map to anything in this codebase, the section says so plainly instead of inventing content.

---

## 3. Repository Structure

```
/home/pi/Desktop/RobotX/
├── README.md                       # Accurate, current setup/run instructions
├── TEST_README.md                  # Standalone hardware-test instructions
├── requirements.txt                # Python deps (fastapi, uvicorn, socketio client, opencv, pyserial, pynmea2, RPi.GPIO)
├── .gitignore                      # venv/, __pycache__/, *.pyc, .env, *.log
├── frame.jpg                       # Debug artifact from a prior test_cv.py/test_vision.py run
├── frames/frame.jpg                # Another debug artifact
├── test_controller.py              # Standalone: ultrasonic+IR -> STOP/FORWARD loop (real GPIO)
├── test_motor.py                   # Standalone: forward/stop/backward/stop motor exercise
├── test_gps.py                     # Standalone: prints NMEA fixes every 2s
├── test_ultrasonic.py              # Standalone: prints HC-SR04 distance continuously
├── test_cv.py                      # Standalone: Picamera2 preview + optional detection
├── test_vision.py                  # Standalone: exercises VisionController + DecisionEngine (headless)
├── venv/                           # Python 3.13 venv — present but NON-EXECUTABLE (see §32)
└── robotx/                         # The actual application package
    ├── __init__.py                 # 1-line docstring only
    ├── app/
    │   ├── __init__.py              # empty
    │   ├── main.py                  # FastAPI entry point; wires every subsystem together
    │   └── sockets.py               # Socket.IO AsyncClient wrapper (robot -> remote server)
    ├── control/
    │   ├── __init__.py              # empty
    │   ├── controller.py            # RobotController — the PRODUCTION control loop (used by main.py)
    │   ├── decision_engine.py       # DecisionEngine — 36-line rule engine, used ONLY by test_vision.py
    │   └── vision_controller.py     # VisionController — advanced pipeline, used ONLY by test_vision.py
    ├── hardware/
    │   ├── __init__.py              # empty
    │   ├── motors.py                 # L298N differential-drive driver (RPi.GPIO + PWM, with mock fallback)
    │   ├── encoders.py               # Single-channel pulse counters -> RPM/m/s (GPIO interrupts)
    │   ├── ultrasonic.py             # HC-SR04 driver (manual trigger/echo timing, polling thread)
    │   └── ir.py                    # 3x digital IR sensor reader
    ├── navigation/
    │   ├── __init__.py              # empty
    │   ├── gps.py                    # NMEA GPS reader over pyserial + pynmea2 (background thread)
    │   ├── maps.py                   # Google Directions API client (polyline decode, cache, rate limit)
    │   └── planner.py                # Waypoint tracking, off-route/reroute heuristics (haversine)
    ├── perception/
    │   ├── __init__.py              # empty
    │   ├── camera.py                 # Picamera2-backed threaded camera reader + MJPEG helper
    │   ├── detection.py              # ObjectDetector: OpenCV (MOG2 motion + Canny edges) or YOLOv8
    │   ├── tracking.py               # ObjectTracker (IoU multi-object) + PrimaryObjectTracker (single-object EMA/trend)
    │   └── filter.py                 # TemporalFilter (stable-label window) + ActionSmoother (unused by production path)
    └── utils/
        ├── __init__.py              # empty
        └── config.py                 # Settings dataclass — all config via ROBOTX_* env vars with defaults
```

**Note on `.git`:** the repository has a `master` branch with **zero commits** (`git log` reports "does not have any commits yet"; `git status` shows every file as untracked). There is no commit history to audit — every file is currently uncommitted working-tree content.

There is no `docker-compose.yml`, `Dockerfile`, `pyproject.toml`, `setup.py`, CI config (`.github/workflows`, etc.), or systemd unit file anywhere in the tree or in `/etc/systemd/system/` on this Pi.

---

## 4. Technology Stack

| Layer | Technology | Evidence |
|---|---|---|
| Language/runtime | Python 3.13 (venv), CPython | `venv/lib/python3.13/`, `python3 --version` = 3.13.5 |
| Web/API framework | FastAPI + Uvicorn | `robotx/app/main.py:5`, `requirements.txt:1-2` |
| Real-time transport | `python-socketio[asyncio_client]` (client only) | `robotx/app/sockets.py:6`, `requirements.txt:3` |
| HTTP client | `httpx` (async) | `robotx/navigation/maps.py:145`, `requirements.txt:4` |
| Computer vision | OpenCV (`opencv-python`), optional `ultralytics` (YOLOv8) | `requirements.txt:5,18-22`, `robotx/perception/detection.py` |
| Camera capture | Picamera2 (libcamera), NOT `cv2.VideoCapture` | `robotx/perception/camera.py:118` |
| GPIO | `RPi.GPIO` (with an in-process mock fallback) | `robotx/hardware/motors.py:22-54`, `requirements.txt:16` |
| Serial/GPS | `pyserial` + `pynmea2` | `robotx/navigation/gps.py:52-53`, `requirements.txt:6-7` |
| Mapping/routing | Google Maps Directions HTTP API (no SDK) | `robotx/navigation/maps.py:136-150` |
| No database | — | Confirmed absent: no ORM, no `.sql`, no schema file anywhere |
| No message broker | — | No Redis, no queue library anywhere in `requirements.txt` or imports |
| No frontend | — | No `package.json`, no JS/TS/HTML source anywhere in the tree |

---

## 5. Runtime Entry Points

There is exactly **one** application entry point:

```
venv/bin/uvicorn robotx.app.main:app --host 0.0.0.0 --port 8000
```
(`README.md:90`, `robotx/app/main.py:29` defines `app = FastAPI(...)`)

Additional, independent entry points are the five standalone test scripts, each runnable directly with `python3 <script>.py` and none of which start the FastAPI server:

- `test_motor.py`, `test_gps.py`, `test_ultrasonic.py`, `test_controller.py` — import real hardware/navigation modules from `robotx.*` and drive them in a bare `while True` loop with `print()` output. No FastAPI, no Socket.IO.
- `test_cv.py` — imports `robotx.perception.camera` and optionally `robotx.perception.detection`; opens a live preview window or, headless, writes `frame.jpg` periodically.
- `test_vision.py` — the only place in the repository that imports and exercises `VisionController`/`DecisionEngine` (see §14, §39 for why this matters).

**[MISSING]** There is no CLI entry point, no `__main__.py`, no console-script, and — per §32 — currently no functioning `venv/bin/python` to even launch uvicorn with, because the interpreter binary lacks the execute bit.

---

## 6. System Architecture

The real, traced runtime flow (there is no frontend, no assignment engine, and no fleet layer, so the template's multi-tier diagram collapses to this):

```
                     ┌───────────────────────────────────────────┐
                     │   Raspberry Pi 5  (robotx package)         │
                     │                                             │
  HTTP  ────────────►│  FastAPI app (main.py)                     │
  GET /health          - /health  -> {ok, robot_id, socket status}│
  GET /camera          - /camera  -> MJPEG stream from CameraStream│
                     │                                             │
                     │  on_startup(): constructs & wires:          │
                     │   MotorDriver, EncoderReader, UltrasonicSensor,
                     │   IRSensors, GPSReader, CameraStream,       │
                     │   ObjectDetector, RoutePlanner,             │
                     │   GoogleMapsDirections, RobotSocketClient   │
                     │   -> RobotController (the control loop)     │
                     │                                             │
                     │  RobotController._loop() @ control_hz (10Hz)│
                     │   reads GPS/ultrasonic/IR/encoders,          │
                     │   throttled camera detection (2Hz default), │
                     │   decides motor speeds per mode,            │
                     │   pushes telemetry dict via telemetry_hook  │
                     │                                             │
                     │  RobotSocketClient (sockets.py)              │
                     │   outbound Socket.IO AsyncClient             │
                     │   emits: robot_hello, telemetry, status      │
                     │   receives: command, manual  -> queued to    │
                     │      controller.handle_command()             │
                     └──────────────────┬──────────────────────────┘
                                         │ Socket.IO over network
                                         ▼
                     ┌───────────────────────────────────────────┐
                     │  External "remote server" — NOT PRESENT     │
                     │  IN THIS REPOSITORY. Only its URL/namespace  │
                     │  contract is defined (ROBOTX_SOCKET_SERVER_URL,
                     │  ROBOTX_SOCKET_NAMESPACE=/robot).            │
                     └───────────────────────────────────────────┘
```

There is no fleet/dashboard/assignment tier in this repo to draw further up the chain — the Socket.IO server the robot talks to is an external dependency whose implementation is out of scope of this codebase (`robotx/app/sockets.py` only implements the *client* side of the contract documented in `README.md:98-124`).

---

## 7. Frontend Architecture

**[MISSING]** There is no frontend application anywhere in this repository — no `package.json`, no React/Vue/Svelte/HTML/CSS/JS source files of any kind. The closest thing to a UI is:
- FastAPI's autogenerated Swagger docs at `/docs` (mentioned in `TEST_README.md:271-273`), which is a debug console for the two existing HTTP endpoints, not a product UI.
- The raw MJPEG stream at `GET /camera`, viewable directly in a browser (`README.md:143`).

Any frontend/dashboard for viewing telemetry, sending commands, or visualizing fleet state would need to live in the (not-present) external Socket.IO server this robot connects out to.

---

## 8. Backend Architecture

**Server entry point:** `robotx/app/main.py:29` — `app = FastAPI(title="RobotX", version="1.0")`.

### Routes (this is the entire route table — two endpoints)

| METHOD | PATH | FILE | HANDLER | AUTH | INPUT | OUTPUT | ERROR HANDLING |
|---|---|---|---|---|---|---|---|
| GET | `/health` | `robotx/app/main.py:41-47` | `health()` | None | none | `{"ok": true, "robot_id": str, "socket_connected": bool}` | None needed (no failure path) |
| GET | `/camera` | `robotx/app/main.py:68-73` | `camera_stream()` | None | none | `multipart/x-mixed-replace` MJPEG stream via `StreamingResponse` | `_mjpeg_stream()` (`main.py:50-65`) sleeps and retries if camera/frame is `None`; never raises |

**[MISSING]** No `POST`/`PUT`/`DELETE` endpoints exist. Commands (`START`/`STOP`/`RETURN`/`MANUAL`) arrive exclusively via inbound Socket.IO events, not HTTP (`robotx/app/sockets.py:51-64`).

### Startup wiring (`main.py:76-186`, `on_startup`)

This function is the single most important piece of backend "architecture" in the repo — it is where every subsystem is instantiated and connected, all from `SETTINGS` (`robotx/utils/config.py`):

1. Builds `MotorDriver` from `SETTINGS.motor_*` pins (`main.py:80-95`).
2. Builds `EncoderReader`, `UltrasonicSensor`, `IRSensors`, `GPSReader`, `CameraStream`, `ObjectDetector` (`main.py:97-126`).
3. Builds `RoutePlanner` and `GoogleMapsDirections` (`main.py:128-133`).
4. Builds `RobotController`, passing every one of the above objects in by constructor injection (`main.py:135-155`) — this is the composition root; there is no DI framework, just explicit wiring.
5. Builds `RobotSocketClient` (`main.py:157-164`), defines `telemetry_hook` (`main.py:166-169`) that forwards controller telemetry into the socket client's outbound queue, and registers it with `controller.set_telemetry_hook(...)`.
6. Fires `asyncio.create_task(socket_boot())` (`main.py:172-179`) — connects the socket **non-fatally**: a failed connection is caught and logged as a warning, so the robot boots and runs its control loop even if the remote server is unreachable.
7. Calls `controller.start()` (`main.py:181`), which starts all hardware background threads and launches the control `asyncio` task (`controller.py:129-141`).

**Shutdown** (`main.py:188-193`): stops the controller (which stops motors, camera, GPS, ultrasonic, encoders — `controller.py:143-170`) and closes the socket client.

### Middleware / auth / validation / logging

- **[MISSING]** No middleware is registered (no CORS, no auth middleware, no rate limiting) — `FastAPI()` is constructed with zero extra config (`main.py:29`).
- **[MISSING]** No request validation beyond FastAPI's own (moot, since there are no body-carrying routes).
- **Logging** (`[IMPLEMENTED]`, minimal): `_setup_logging()` (`main.py:22-26`) configures the root logger from `SETTINGS.log_level`; individual modules use `logging.getLogger(__name__)` (e.g. `controller.py:21`, `sockets.py:9`) but most operational visibility is actually via bare `print()` statements in `detection.py` and `vision_controller.py` (see §19).

### Workers / queues / scheduled jobs

**[MISSING]** No Celery/RQ/cron/APScheduler. The only "background jobs" are Python `threading.Thread` daemons (encoder sampler, ultrasonic poller, GPS reader, camera loop — each hardware module owns its own thread) and `asyncio` tasks (`RobotController._loop`, socket telemetry/command loops). These are in-process, not a distributed worker system.

---

## 9. Database / Prisma Architecture

**[MISSING] — entirely.** There is no Prisma schema, no migrations directory, no SQL file, no ORM import (SQLAlchemy, Django ORM, etc.), and no embedded database (SQLite) anywhere in the repository. Grep for "prisma", "\.sql", "sqlite", "postgres", "mysql" across the tree returns nothing outside `venv/`.

The only on-disk persistence in the entire application is:
- `robotx/navigation/maps.py:68,82-107` — a flat JSON file cache for Google Directions routes, written to `/tmp/robotx_directions_cache.json` by default (`cache_path` parameter), with a TTL (`cache_ttl_s`, default 300s). This is a simple file cache, not a database — no schema, no indices, no relations, just a dict of `"{origin}->{dest}": {created_t, route}` keyed strings.

There is nothing to document in terms of models/fields/relationships/indexes because none exist. Every "state" field described in §12–§13 below (robot mode, position, battery, sensor readings) lives only in Python in-memory instance attributes of `RobotController` (`controller.py:103-123`) and is never persisted to disk or a database. **A process restart loses all robot state.**

---

## 10. Redis Architecture

**[MISSING].** No `redis` package in `requirements.txt`, no `import redis`, no `aioredis` anywhere in the codebase. There is no caching layer, pub/sub, or deduplication mechanism backed by Redis. (The closest thing to caching anywhere is the flat-file JSON directions cache described in §9.)

---

## 11. Authentication and Authorization

**[MISSING] across the board.** Concretely:

- **HTTP endpoints** (`/health`, `/camera`): no auth of any kind — anyone who can reach port 8000 can view the live camera feed and health status.
- **Socket.IO client → server**: `RobotSocketClient.connect()` (`sockets.py:66-67`) calls `self.sio.connect(self.cfg.server_url, namespaces=[self.cfg.namespace])` with **no auth token, no header, no query-param credential** of any kind. The only identity assertion is a `robot_hello` event sent *after* connecting, carrying nothing but a self-reported string: `{"robot_id": self.cfg.robot_id}` (`sockets.py:44`). Any client that knows (or guesses) the namespace `/robot` and connects to the server URL can impersonate this robot or send it commands, as far as this code is concerned — **there is no verification that a `command` event actually originated from an authorized operator; the robot processes any well-formed `command`/`manual` payload it receives** (`sockets.py:51-64`, unconditionally enqueued and later executed by `controller.handle_command()`).
- **[CONFIGURATION DEPENDENT]** Whatever authentication exists, if any, would have to be implemented on the (absent) external Socket.IO server side and/or added to this client — it is not present in this repo today.
- **Google Maps API key**: read from `ROBOTX_GOOGLE_MAPS_API_KEY` env var (`config.py:91`), passed as a URL query parameter to Google's API (`maps.py:141`) — standard for that API, but there's no key rotation/secret-manager integration; whatever protects it is outside this code (env var hygiene only).
- **No WebAuthn/passkeys/JWT/cookies/sessions/roles/campus isolation** — none of this applies; there is no multi-user or multi-tenant concept in this codebase at all.

**Security implication:** this is a single-robot design that assumes the operator network/server is already trusted; if pointed at an untrusted or shared network, any party who can reach the socket namespace can send `MANUAL` motor commands or a spoofed `START` with an arbitrary destination.

---

## 12. Robot Domain Architecture

There is only **one robot** modeled by this codebase — its own local process. "Robot domain" here means the objects `RobotController` owns and how they interact; there's no multi-robot registry, fleet table, or robot-record abstraction, because this code *is* the robot, not a server managing robots.

| Component | File | Purpose | Instantiated | Communicates via | State held |
|---|---|---|---|---|---|
| `RobotController` | `robotx/control/controller.py:73` | Central decision loop: sensors → mode logic → actuation → telemetry | `main.py:135-155`, once at app startup | Calls into every hardware/nav/perception object directly (in-process method calls); pushes telemetry dicts to a hook | `_mode`, `_status_msg`, `_destination`, `_home`, `_last_gps_pos`, `_last_heading`, `_manual_cmd`, `_last_detections` (`controller.py:103-123`) — all in-memory only |
| `MotorDriver` | `robotx/hardware/motors.py:69` | Differential-drive actuation over L298N | Built in `main.py:80-95` | `RPi.GPIO.output`/`PWM` calls | `_last_cmd` (last left/right duty), `_started` flag |
| `EncoderReader` | `robotx/hardware/encoders.py:26` | Wheel-speed feedback | Built in `main.py:97-105` | GPIO rising-edge interrupts + background sampler thread | pulse counts, computed RPM/m-per-s |
| `UltrasonicSensor` | `robotx/hardware/ultrasonic.py:19` | Front obstacle distance | Built in `main.py:107-113` | Manual trigger/echo GPIO timing in a polling thread | `_last_distance_cm` |
| `IRSensors` | `robotx/hardware/ir.py:19` | 3-point line/obstacle digital sensing | Built in `main.py:115-122` | Direct `GPIO.input()` reads (no thread; polled by controller loop) | none (stateless reads) |
| `GPSReader` | `robotx/navigation/gps.py:13` | NMEA lat/lon fix | Built in `main.py:124` | Background thread reading `pyserial` + `pynmea2.parse()` | `_location`, `_last_fix_t`, `_error` |
| `CameraStream` | `robotx/perception/camera.py:8` | Live BGR frames | Built in `main.py:125` | Background thread pulling from a Picamera2 singleton manager | `_frame`, `_last_ok_t` |
| `ObjectDetector` | `robotx/perception/detection.py:209` | Person/obstacle detection | Built in `main.py:126` | Pure function call from controller loop (`asyncio.to_thread`) | detector-internal state (MOG2 background model) |
| `RoutePlanner` | `robotx/navigation/planner.py:28` | Waypoint progress + reroute decision | Built in `main.py:128` | Pure in-memory | `_route`, `_idx`, `_offroute_count`, `_blocked_count` |
| `GoogleMapsDirections` | `robotx/navigation/maps.py:60` | Route fetch (external HTTP) | Built in `main.py:129-133` | `httpx` async HTTP to Google Directions API | in-memory + JSON file cache |
| `RobotSocketClient` | `robotx/app/sockets.py:23` | Outbound Socket.IO transport | Built in `main.py:157-164` | `python-socketio` AsyncClient over the network | connection state, two `asyncio.Queue`s (telemetry, commands) |

### How robot identity is established

Purely by configuration: `SETTINGS.robot_id` defaults to `"robotx-pi"` (`config.py:34`), overridable via `ROBOTX_ROBOT_ID`. This string is sent once in `robot_hello` on connect (`sockets.py:44`) and included in every telemetry payload (`controller.py:381`). **There is no cryptographic identity, no certificate, no per-robot secret** — identity is a self-asserted string.

---

## 13. Robot Lifecycle

Tracing the exact code for each stage the audit brief asks about:

| Stage | Code responsible | Status |
|---|---|---|
| **REGISTER/COMMISSION** | None — `SETTINGS.robot_id` is just an env var; no registration handshake, no provisioning flow | **[MISSING]** |
| **CONNECT** | `RobotSocketClient.connect()` (`sockets.py:66-67`) called from `main.py:174` inside a fire-and-forget `asyncio.create_task` | [IMPLEMENTED] |
| **AUTHENTICATE** | Not present — connection is anonymous; `robot_hello` is informational only, not verified | **[MISSING]** |
| **HEARTBEAT** | No dedicated heartbeat event. Telemetry (`controller.py:378-401`, emitted every `telemetry_interval_s`, default 1.5s) doubles as a liveness signal; `/health` also reports `socket_connected` (`main.py:46`) | **[PARTIALLY IMPLEMENTED]** (telemetry cadence substitutes for heartbeat; no explicit ping/pong contract beyond Socket.IO's own engine.io pings) |
| **TELEMETRY** | `RobotController._loop()` builds and calls `self._telemetry_hook(telemetry)` (`controller.py:378-401`); `main.py:166-167` forwards it into `sock.enqueue_telemetry()`; `RobotSocketClient.telemetry_loop()` (`sockets.py:100-108`) emits it as a `telemetry` Socket.IO event | [IMPLEMENTED] |
| **COMMAND** | Inbound `command`/`manual` Socket.IO events (`sockets.py:51-64`) → `_command_queue` → `RobotSocketClient.command_loop()` (`sockets.py:110-116`) → `controller.handle_command(cmd)` (registered as the handler in `main.py:175`) | [IMPLEMENTED] |
| **TASK/MISSION** | `handle_command` interprets `START` (with a `destination` lat/lon) as the entire "mission" concept — sets `_mode="AUTO"` and calls `_ensure_route()` (`controller.py:176-183`) | **[PARTIALLY IMPLEMENTED]** — there is no mission object, no multi-step task list, no ETA, no task ID; a "mission" is just "drive toward this one lat/lon" |
| **PROGRESS** | `RoutePlanner.progress()` (`planner.py:95-98`) — fraction of waypoints consumed; included in telemetry's `route.progress` field (`controller.py:392-397`) | [IMPLEMENTED] (rudimentary — waypoint index ÷ waypoint count, not distance-based) |
| **COMPLETION** | **[MISSING]** — there is no explicit "arrived" detection or completion event. The planner clamps its waypoint index at the last index (`planner.py:76`, `min(self._idx + 1, len(self._route) - 1)`) but nothing transitions `_mode` away from `AUTO` or notifies the server that the destination was reached |
| **DISCONNECT/RECONNECT** | Handled by the underlying `python-socketio` client's own reconnection logic (`reconnection=cfg.reconnect` at `sockets.py:28`); the `disconnect` handler just clears an `asyncio.Event` (`sockets.py:46-49`) — no state is persisted or replayed across a reconnect | **[PARTIALLY IMPLEMENTED]** (library-level reconnect only; no application-level resync of missed commands) |

---

## 14. Simulation Architecture

**There is no simulation system in this codebase** in the sense the audit brief describes (no `VirtualRobot` class, no fleet simulator, no simulated battery drain/movement model, no Redis-backed dedup, no offer/accept/reject/defer logic). Searching the entire tree for "simulat", "virtual", "mock" (outside GPIO) confirms this.

The only simulation-adjacent code is the GPIO/PWM mock fallback in `robotx/hardware/motors.py:6-54`:

```python
class _MockPWM: ...      # motors.py:6-19
class _MockGPIO: ...     # motors.py:22-54
try:
    import RPi.GPIO as GPIO
except Exception:
    GPIO = _MockGPIO()
```

This exists **only so `MotorDriver` can be imported and unit-exercised on a non-Pi machine** (e.g. a laptop) without raising `ModuleNotFoundError`. It has no analog for encoders, ultrasonic, or IR — those modules (`encoders.py:8-11`, `ultrasonic.py:7-10`, `ir.py:5-8`) instead set `GPIO = None` when the import fails and simply **return `None`/zero/`{"left": None, ...}`** from their read methods (`ultrasonic.py:70-71`, `ir.py:44-45`) rather than fabricating plausible sensor data. This is a genuine gap, not a simulator: on a non-Pi host, ultrasonic and IR silently report "no data" forever rather than simulating a scene.

**What is real vs. what is a stand-in:**
- **REAL**: motor GPIO output/PWM math, encoder pulse-to-RPM math, ultrasonic trigger/echo timing, IR digital reads, GPS NMEA parsing, camera capture via Picamera2, OpenCV/YOLO detection, haversine/bearing navigation math, Google Directions HTTP calls.
- **STAND-IN (not simulation, just no-op)**: `_MockGPIO`/`_MockPWM` (motors only), and the `GPIO is None` early-returns in encoders/ultrasonic/IR/GPS.
- **HARDCODED, NOT REAL AT ALL**: `"battery": {"percent": 76.0}` in telemetry (`controller.py:386`) — there is no battery-voltage-sensing code anywhere in the repository (confirmed by grep — see §26). This constant will be sent to whatever server consumes telemetry as if it were live data.

**What would have to change for a real robot:** nothing — this code *is* written for the real robot already. The relevant question is inverted from the brief's framing: **the risk is that this code has never been proven to run *simulated*** (there's no test harness that exercises `RobotController._loop()` end-to-end without physical GPIO/camera/GPS hardware present), so the "what changes for physical integration" question doesn't really apply — the harder question, addressed in §36, is what's still missing to make the *existing* physical-hardware code trustworthy (battery telemetry, auth, e-stop).

---

## 15. Assignment Engine

**[MISSING].** There is no task-creation, candidate-robot-selection, scoring, or offer/accept pipeline anywhere in this codebase, because there is exactly one robot and no server-side task system in this repo. The nearest analog — "how does this robot get a destination?" — is entirely client-side: a `START` command's `payload.destination` lat/lon (`controller.py:176-179`) is accepted uncritically as the new target; there is no validation that the destination is reachable, in-bounds, or authorized.

---

## 16. DTARO / Optimization

**[MISSING].** No optimizer, no distributed task-routing algorithm, no worker pool. The only "algorithm" present is the reroute heuristic in `RoutePlanner.should_reroute()` (`planner.py:88-93`): reroute if blocked ≥3 times or off-route for ≥8 consecutive position updates. This runs synchronously inside `RobotController._loop()` at `control_hz` (default 10 Hz) — not a background worker, not distributed, and O(1) per call (no search/optimization, just counter comparisons).

---

## 17. Routing and Distance Calculation

- **Distance/bearing math**: `haversine_m()` and `bearing_rad()` (`planner.py:10-17`, `controller.py:32-38`) — real, correct spherical-earth formulas, used for waypoint-arrival detection, off-route detection, and heading-error steering.
- **Route generation**: `GoogleMapsDirections.get_route()` (`maps.py:117-163`) calls the real Google Directions HTTP API (`maps.googleapis.com/maps/api/directions/json`, driving mode, no alternatives), decodes the returned encoded polyline into a list of `(lat, lon)` waypoints (`decode_polyline()`, `maps.py:17-51`), and caches the result both in memory and to a JSON file (`/tmp/robotx_directions_cache.json` by default).
- **Rate limiting**: a naive global "one call per `min_interval_s` (default 15s)" gate (`maps.py:126-129`) that **fails fast** (raises) rather than queuing, if a cache miss occurs during the cooldown window.
- **Fallback behavior**: if the API key is missing, `get_route()` raises immediately (`maps.py:118-119`); the caller (`controller.py:224-227`, `244-247`) catches this and just sets a human-readable `_status_msg`, **leaving the robot with no route** (it will then sit at `motors.stop()` per `controller.py:370-373`, "Waiting for route/GPS").
- **Where routing enters the control loop**: `RoutePlanner.next_waypoint()` feeds `_route_follow_command()` (`controller.py:295-314`), which computes a bearing error against `_last_heading` (itself derived from consecutive GPS fixes via `_update_heading()`, `controller.py:249-256` — **there is no compass/IMU**, heading is inferred purely from GPS movement) and converts it into differential left/right motor duty cycles via a simple proportional steering gain (`steer_gain = 0.25`).
- **What the robot actually receives**: a **full polyline waypoint list** (from Google Directions), not just a high-level command or single destination — the robot itself does turn-by-turn waypoint following locally; the server only ever sends one destination point via `START`.

**[MISSING]** No zones, no campus boundaries, no geofencing — any lat/lon can be requested as a destination.

---

## 18. Zone and Campus Logic

**[MISSING]** as a navigation/business concept (no campus isolation, no geofence, no multi-tenant zone model — see §15, §17).

The word "zone" **does** appear, but in a completely different, perception-local sense: `detection.py:_zone_for_cx()` (`detection.py:41-49`) and `vision_controller.py:_center_zone_for_width()` (`vision_controller.py:79-87`) divide the **camera frame** into LEFT/CENTER/RIGHT thirds to decide which way to steer around an obstacle. This is a per-frame image-processing concept, not a geographic/campus one — do not conflate the two if referencing "zones" from other RobotX documentation.

---

## 19. Real-Time / Socket.IO Architecture

Full event table for the one real-time surface in this codebase (`robotx/app/sockets.py`, namespace `/robot` by default, configurable via `ROBOTX_SOCKET_NAMESPACE`):

| EVENT | DIRECTION | SENDER | RECEIVER | PAYLOAD | PURPOSE | AUTH | DB EFFECT | REDIS EFFECT | FAILURE BEHAVIOR |
|---|---|---|---|---|---|---|---|---|---|
| `connect` (engine.io built-in) | robot → server | `RobotSocketClient` | external server | none | Establish transport | **none** | n/a | n/a | Library auto-reconnects if `reconnect=True` (default) |
| `robot_hello` | robot → server | `sockets.py:44` (on `connect` handler) | external server | `{"robot_id": str}` | Self-announce identity | **none** — unauthenticated | n/a | n/a | Fire-and-forget `emit`; no ack awaited |
| `telemetry` | robot → server | `RobotSocketClient.telemetry_loop()` (`sockets.py:100-108`) | external server | see §20 schema | Periodic state broadcast | none | n/a (no DB in this repo) | n/a | Emit failures logged as warning, loop continues; queue drops oldest if full (`enqueue_telemetry`, `sockets.py:91-98`, `maxsize=5`) |
| `status` | robot → server | `RobotSocketClient.emit_status()` (`sockets.py:88-89`) | external server | caller-defined dict | Optional ad-hoc status push | none | n/a | n/a | **[UNUSED / DEAD CODE]** — `emit_status()` is defined but never called anywhere in the codebase (grep confirms zero call sites outside its own definition) |
| `command` | server → robot | external server | `sockets.py:51-56` `on_command` handler | `{"type": "START"\|"STOP"\|"RETURN"\|"MANUAL", "payload": {...}}` | Instruct robot | none — any well-formed dict is queued and executed | n/a | n/a | Non-dict payloads are silently dropped (`if not isinstance(data, dict): return`); no ack sent back |
| `manual` | server → robot | external server | `sockets.py:58-64` `on_manual` handler | arbitrary dict (or non-dict, wrapped as `{"raw": data}`) | Convenience alias that gets normalized into a `MANUAL` command | none | n/a | n/a | Same as `command` |
| `disconnect` (engine.io built-in) | — | — | `sockets.py:46-49` | none | Clears internal connected-Event | none | n/a | n/a | Underlying reconnection logic (library-level) attempts to reconnect |

**Reconnect behavior**: delegated entirely to `python-socketio`'s `AsyncClient(reconnection=True)` (`sockets.py:28`) — no custom backoff, no resync-missed-commands logic on the application side.

**Ordering/dedup**: `_command_queue` is a plain FIFO `asyncio.Queue` (`sockets.py:31`) — commands are processed strictly in arrival order, one at a time, by a single `command_loop()` task; there is no deduplication of repeated/duplicate commands, and no idempotency key.

**Broadcast scope**: N/A — this is a point-to-point client, not a room/namespace-broadcasting server; "broadcast scope" concepts (rooms, namespaces beyond the single configured one) don't apply here because the server side isn't in this repo.

---

## 20. Telemetry Pipeline

Full source-to-sink trace:

```
RobotController._loop()  (controller.py:316-415), every control_hz tick (default 10 Hz)
  reads: GPS location, ultrasonic distance, IR state, encoder speed
  throttled (detection_hz, default 2 Hz): camera frame -> ObjectDetector.detect_objects()
  every telemetry_interval_s (default 1.5s):
    builds telemetry dict (controller.py:380-399) — see schema below
    -> await self._telemetry_hook(telemetry)          [main.py:166-167, the injected hook]
       -> await sock.enqueue_telemetry(payload)         [RobotSocketClient, sockets.py:91-98]
          -> asyncio.Queue(maxsize=5); drops OLDEST entry if full, never blocks
RobotSocketClient.telemetry_loop()  (sockets.py:100-108), a separate asyncio task
  awaits queue.get() -> sio.emit("telemetry", telemetry, namespace="/robot")
  (network) -> external server (not in this repo)
```

### Telemetry schema (from `controller.py:380-399`, this is the literal, exhaustive shape sent)

```json
{
  "robot_id": "robotx-pi",
  "mode": "IDLE|AUTO|MANUAL|RETURN|STOPPED|ERROR",
  "status": "<human-readable string>",
  "location": {"lat": float|null, "lon": float|null, "fix_age_s": float|null},
  "speed": {"left_rpm": float, "right_rpm": float, "left_mps": float, "right_mps": float, "avg_mps": float},
  "battery": {"percent": 76.0},
  "sensors": {"ultrasonic_cm": float|null, "ir": {"left": bool|null, "right": bool|null, "center": bool|null}},
  "perception": {"summary": {"person": bool, "obstacle": bool, "count": int}, "detections": [...up to 10]},
  "route": {"has_route": bool, "progress": float, "next_waypoint": [lat, lon]|null, "destination": [lat, lon]|null},
  "ts": 1234567890.123
}
```

- **Frequency**: capped by `telemetry_interval_s` (default 1.5s = ~0.67 Hz), independent of the 10 Hz control loop.
- **Persistence**: **[MISSING]** — telemetry is never written to disk or a database anywhere in this repo; it exists only as an in-flight dict emitted over the socket.
- **Caching**: none beyond the 5-slot `asyncio.Queue` used purely for backpressure (drop-oldest), not for lookups.
- **Historical storage**: **[MISSING]**.
- **Validation**: **[MISSING]** — the dict is constructed from live sensor reads with no schema validation (no Pydantic model) before being emitted; a `None` GPS fix, `None` ultrasonic reading, etc. is sent as literal JSON `null` and it's up to the (absent) consumer to handle that.
- **Stale/missing-robot handling**: **[MISSING]** — there is no last-seen timestamp tracking or staleness detection on the robot side; that would necessarily live on the (absent) server side.
- **Does telemetry assume simulation?** No — every field except `battery.percent` is sourced from a real sensor/GPS/planner read. `battery.percent` is the one field that is unconditionally fabricated (see §14, §26).

---

## 21. Robot Commands / Control Pipeline

Traced end-to-end for each of the four command types (`controller.py:172-212`, `handle_command`):

| Command | Payload | Effect | File:Line |
|---|---|---|---|
| `START` | `{destination: {lat, lon}}` | Sets `_destination`, `_mode="AUTO"`, calls `_ensure_route()` which fetches a Google Directions polyline from current GPS position to destination and loads it into `RoutePlanner` | `controller.py:176-183`, `214-227` |
| `STOP` | none | Sets `_mode="STOPPED"`, immediately calls `motors.stop()` | `controller.py:185-189` |
| `RETURN` | `{home: {lat, lon}}` (optional) | Sets `_mode="RETURN"`; if `home` omitted, uses first-ever GPS fix (`_home`, set opportunistically in the control loop at `controller.py:328-329`); fetches a route back | `controller.py:191-199`, `229-247` |
| `MANUAL` | `{action, speed, left?, right?}` | Sets `_mode="MANUAL"` and stores the raw manual command dict for `_apply_manual()` to execute every control tick | `controller.py:201-210`, `417-440` |

`_apply_manual()` (`controller.py:417-440`) supports `forward`/`backward`/`left`/`right`/`set_speed` (explicit per-wheel duty)/anything-else-stops. **No bounds/rate-limiting on manual speed beyond `max_cmd` clamping** (`config.py:104`, `max_motor_duty` default 0.75) — a malicious or buggy `MANUAL` command executes immediately, every control tick, until superseded.

**Safety interlock ordering** (`controller.py:346-376`): regardless of mode, `_safety_blocked()` (ultrasonic distance < threshold, OR any IR triggered, OR person/obstacle detected by vision) is computed every tick; if true while in `AUTO`/`RETURN` mode, the loop runs `_avoid()` (a blind left/right probing turn sequence, `controller.py:267-293`) instead of route-following. **Note: `_safety_blocked()` is *not* consulted while in `MANUAL` mode** (`controller.py:351-352`, `_apply_manual()` is called unconditionally) — a manual joystick-style command can drive the robot into an obstacle the ultrasonic/IR/vision would otherwise have stopped it for. This is a concrete safety gap, not a hypothetical one.

---

## 22. Raspberry Pi Integration

This *is* the Raspberry Pi codebase — everything in `robotx/hardware/`, `robotx/perception/camera.py`, and `robotx/navigation/gps.py` targets the Pi directly via `RPi.GPIO` and `picamera2`. Verified real, on-device library presence (checked directly on this Pi):

```
RPi.GPIO   -> importable on system Python (apt-installed)      [REAL]
picamera2  -> importable on system Python (apt-installed)      [REAL]
```

Grep results for the audit brief's keyword list, classified:

| Keyword | Hits | Classification |
|---|---|---|
| `raspberry`/`Raspberry Pi` | `README.md`, docstrings in `camera.py:14`, `main.py` (none), `robotx/__init__.py:1` | Documentation + one class docstring — accurate, not aspirational |
| `RPi.GPIO` | `motors.py:52`, `encoders.py:9`, `ultrasonic.py:8`, `ir.py:6` | **[IMPLEMENTED]** real hardware driver imports |
| `GPIO` (pins, setup, output) | throughout `hardware/` | **[IMPLEMENTED]** |
| `motor`/`motor driver` (L298N) | `motors.py` entire file | **[IMPLEMENTED]** |
| `ultrasonic` | `hardware/ultrasonic.py` entire file | **[IMPLEMENTED]** |
| `infrared`/`IR` | `hardware/ir.py` entire file | **[IMPLEMENTED]** |
| `GPS` | `navigation/gps.py` entire file | **[IMPLEMENTED]** (generic NMEA, not M9N-specific — see §26) |
| `M9N` | zero hits | **[MISSING]** as a specifically-named module; any NMEA-speaking GPS (including a u-blox M9N) works because the parser is generic |
| `serial`/`/dev/tty*` | `gps.py:59` (`serial.Serial(self.cfg.port, ...)`, default `/dev/ttyAMA0`) | **[IMPLEMENTED]** |
| `UART` | mentioned in `README.md:45` (raspi-config step) | Documentation only — the actual serial open call is in `gps.py:59` |
| `PWM` | `motors.py` (`GPIO.PWM`, `_left_pwm`/`_right_pwm`) | **[IMPLEMENTED]** |
| `hardware` (generic) | package name `robotx/hardware/` | **[IMPLEMENTED]** — a real package, not a placeholder directory |
| `sensor` (generic) | ultrasonic + IR + encoders + GPS + camera | **[IMPLEMENTED]** across the board |
| `ESP32` | **zero hits, anywhere, including venv** | **[MISSING]** — confirmed absolutely absent |

**Conclusion for §22/§23 (Physical Robot Communication Design):** The backend does **not** expect a Raspberry Pi → ESP32 → sensors/motors chain. The actual, current architecture is:

```
Backend (Socket.IO server, external)
   ↓ Socket.IO (network)
Raspberry Pi  (this repo's FastAPI/asyncio process)
   ↓ RPi.GPIO (direct pin I/O, no intermediate MCU)
Motors (L298N) / Ultrasonic (HC-SR04) / IR sensors
   ↓ pyserial (UART)
GPS module (any NMEA-0183 device, e.g. a u-blox M9N)
   ↓ Picamera2 (libcamera / CSI)
Camera
```

There is no GPIO-direct-from-backend design either — the Pi runs its own local control loop (`RobotController`) autonomously; the remote server only sends high-level intents (`START`/`STOP`/`RETURN`/`MANUAL`), not low-level actuator signals.

---

## 23. ESP32 / Hardware Integration

**[MISSING] entirely.** As established in §22, there is no ESP32, no I2C/SPI bridge to a co-processor, and no firmware source of any kind in this repository. If a future design wants to introduce an ESP32 (e.g. for hard-real-time PWM/encoder handling offloaded from the Pi's Python control loop), that would be a **new component**, not a refactor of anything existing — none of the current hardware modules assume or leave room for a serial-to-MCU protocol.

---

## 24. Sensors

| Sensor | Driver file | Real/Placeholder | Notes |
|---|---|---|---|
| HC-SR04 ultrasonic | `robotx/hardware/ultrasonic.py` | **[IMPLEMENTED]** real | Manual trigger/echo pulse timing (`_measure_distance_cm`, `ultrasonic.py:69-98`), 30ms timeout, rejects readings ≤0 or >500cm, background poll thread |
| 3× digital IR | `robotx/hardware/ir.py` | **[IMPLEMENTED]** real | Simple digital read, active-low configurable; **no debounce/filtering** |
| Wheel encoders (2×, single-channel) | `robotx/hardware/encoders.py` | **[IMPLEMENTED]** real | GPIO rising-edge interrupt counting + 10Hz sampler thread → RPM → m/s via wheel circumference. **Single-channel = no direction sensing** (can't distinguish forward vs. backward rotation from the encoder alone — direction is only known from the commanded motor sign, not measured) |
| GPS (NMEA) | `robotx/navigation/gps.py` | **[IMPLEMENTED]** real | Generic `pynmea2` parser — works with any NMEA-0183 GPS, including a u-blox M9N, but nothing M9N-specific (no UBX binary protocol, no RTK, no fix-quality/HDOP field exposed to telemetry) |
| Camera (CSI, Picamera2) | `robotx/perception/camera.py` | **[IMPLEMENTED]** real | Threaded singleton manager (`_Picamera2Manager`) with refcounted acquire/release so multiple consumers (MJPEG stream + controller detection) can share one camera instance |
| Battery voltage/current | — | **[MISSING]** | No ADC/INA219/voltage-divider code anywhere; telemetry's `battery.percent` is the literal constant `76.0` (`controller.py:386`) |
| IMU/compass | — | **[MISSING]** | Heading is inferred purely from consecutive GPS fixes (`_update_heading`, `controller.py:249-256`) — no gyro/accelerometer/magnetometer anywhere |
| Bumper/e-stop switch | — | **[MISSING]** | No dedicated safety-critical hardware input; obstacle avoidance is entirely software-mediated (ultrasonic/IR/vision thresholds) |

---

## 25. Motors / Motion Control

`robotx/hardware/motors.py` — **[IMPLEMENTED]**, real:

- `MotorDriver` targets an **L298N dual H-bridge**, differential drive, 2 motors (`MotorPins(in1, in2, en)` per side).
- `set_speed(left, right)` takes `[-1.0, 1.0]` per wheel; direction via `IN1`/`IN2` HIGH/LOW combination, magnitude via `ChangeDutyCycle()` on a software PWM channel (`GPIO.PWM`, default 1000 Hz), clamped to `max_duty` (default 0.75 = 75% max commanded duty even at full-scale input — a hard safety ceiling).
- Convenience primitives: `forward()`, `backward()`, `turn_left()`, `turn_right()`, `brake()` (active braking by driving both H-bridge inputs HIGH briefly), `stop()`.
- **Fail-safe design**: `RobotController.stop()` calls `motors.stop()` in a `finally` (`controller.py:151-154`), and the main control loop's `except Exception` handler forces `motors.stop()` before continuing (`controller.py:405-412`) — this is a genuine, verified fail-safe path, not aspirational.
- Closed-loop speed control: a basic PI controller (`_PI` class, `controller.py:56-70`, `kp=1.2, ki=0.6`) drives a target linear speed (`target_speed_mps`, default 0.25 m/s) using encoder feedback (`_route_follow_command`, `controller.py:295-314`) — real, if simplistic (no anti-windup beyond output clamping, no per-wheel independent speed loop, just a single shared base speed + steering offset).

---

## 26. GPS / M9N

Already covered fully in §22/§24. Summary: **[IMPLEMENTED]** generically for NMEA GPS, **[MISSING]** any M9N-specific (UBX binary, RTK, fix-quality) features. Default serial port `/dev/ttyAMA0` at 9600 baud (`config.py:88-89`), overridable via `ROBOTX_GPS_PORT`/`ROBOTX_GPS_BAUDRATE`.

---

## 27. Charging / Battery

**[MISSING].** Grep confirms zero occurrences of "charge", "charging", "dock", "voltage", "ADC", "INA219", or any battery-sensing library across the entire non-venv tree. The only battery-related code is the hardcoded telemetry constant discussed in §14/§20/§24. There is no low-battery behavior (no auto-return-to-home-on-low-battery, no charge-state machine) anywhere.

---

## 28. Safety / Emergency Handling

| Mechanism | Status | Evidence |
|---|---|---|
| Software obstacle avoidance (ultrasonic + IR + vision) | **[IMPLEMENTED]** | `_safety_blocked()` + `_avoid()`, `controller.py:258-293` |
| Fail-safe motor stop on unhandled exception | **[IMPLEMENTED]** | `controller.py:405-412` |
| Fail-safe motor stop on graceful shutdown | **[IMPLEMENTED]** | `controller.py:143-154` |
| Safety check applies in MANUAL mode | **[MISSING]** | `controller.py:351-352` — `_apply_manual()` runs unconditionally, bypassing `_safety_blocked()` (see §21) |
| Dedicated hardware emergency-stop (physical button/relay cutting motor power independent of software) | **[MISSING]** | No such GPIO pin, no relay/kill-switch code anywhere |
| Max commanded duty ceiling | **[IMPLEMENTED]** | `MotorDriver.max_duty` (default 0.75), `config.py:104` |
| Obstacle-avoidance maneuver is itself safety-checked | **[PARTIALLY IMPLEMENTED]** | `_avoid()` (`controller.py:267-293`) blindly turns/advances for fixed durations (`avoid_turn_seconds`/`avoid_forward_seconds`) and only re-checks ultrasonic (not IR, not vision) before declaring itself clear |
| Watchdog for stale/lost sensor data (e.g., ultrasonic returning `None` for a long time) | **[MISSING]** | `_safety_blocked()` treats `distance_cm is None` as "not blocked" (`controller.py:258-259`, the `<` comparison against `None` short-circuits via the `is not None` guard) — **a disconnected/failed ultrasonic sensor is treated the same as "no obstacle," not as an error state** |

**This last point is a concrete, evidence-based safety gap**: if the HC-SR04 sensor fails, is unplugged, or times out (`ultrasonic.py:83-85`, `88-90` — returns `None` on 30ms timeout), the robot's safety logic silently falls back to trusting IR + vision alone rather than treating "no ultrasonic reading" as itself hazardous.

---

## 29. Security Audit

Findings, ranked by severity, all traced to code (no secret values reproduced):

| Severity | Finding | Evidence |
|---|---|---|
| **CRITICAL** | Socket.IO client has zero authentication — any party able to connect to the configured namespace can send `MANUAL`/`START`/`STOP`/`RETURN` commands that the robot will execute unconditionally | `sockets.py:51-64`, `controller.py:172-212` |
| **CRITICAL** | `MANUAL` commands bypass all obstacle-avoidance safety checks | `controller.py:351-352` |
| **HIGH** | No TLS/origin verification is enforced in code — `socketio.AsyncClient(...)` is constructed with no explicit `ssl_verify` or certificate pinning; whatever transport security exists depends entirely on `ROBOTX_SOCKET_SERVER_URL` being `https://`/`wss://`, which is a deployment choice, not something the code enforces | `sockets.py:28`, `config.py:42` |
| **HIGH** | `/health` and `/camera` HTTP endpoints are unauthenticated and bound to `0.0.0.0` by default — anyone on the network can view the live camera feed | `config.py:38` (`api_host` default `0.0.0.0`), `main.py:41-73` |
| **MEDIUM** | No input validation on `START`'s `destination` lat/lon — arbitrary floats are accepted and immediately used to request a real, rate-limited/paid Google Directions API call | `controller.py:177-179` |
| **MEDIUM** | Google Maps API key is passed as a plain URL query parameter (standard for this API, but confirms it will appear in any request logs on the network path or at Google's edge — normal for this API, flagged for awareness only) | `maps.py:141` |
| **LOW** | No rate limiting on FastAPI endpoints (moot today given only 2 read-only endpoints exist, but relevant if more are added) | `main.py` — no middleware registered |

No secret values were found hardcoded in source (the only credential-shaped config, `ROBOTX_GOOGLE_MAPS_API_KEY`, is read exclusively from an environment variable — `config.py:91`).

---

## 30. Performance / Scalability

This is a **single-process, single-robot** application — "scalability" in the fleet sense doesn't apply. Code-level observations:

- **CODE-LEVEL OBSERVATION**: Object detection runs in a background thread via `asyncio.to_thread(self.detector.detect_objects, frame)` (`controller.py:340`) so it doesn't block the 10 Hz control loop event loop — a correct pattern for a CPU-bound OpenCV call inside `asyncio`.
- **CODE-LEVEL OBSERVATION**: `ObjectDetector.detect_objects()` (`detection.py:313-504`) calls `print()` on **every single invocation** — up to ~10 lines of stdout per detection cycle (raw detections, filtered detections, per-detection listing, largest area, collision flag, decision, contour counts) at up to `detection_hz` (default 2 Hz) in the production path, and considerably more verbosely inside `VisionController.step()` (`vision_controller.py:643-670`) at up to `detection_hz` (default 5 Hz) in the test-only path. **Unbuffered console I/O at this frequency is a real, measurable CPU/IO cost on a Raspberry Pi** and should be gated behind the existing `logging` module (already configured elsewhere in the app) rather than bare `print()`.
- **CODE-LEVEL OBSERVATION**: The Google Directions JSON file cache (`maps.py:97-107`) rewrites the **entire cache file** on every new cache entry (`self.cache_path.write_text(json.dumps(payload))`) rather than appending — with a `cache_ttl_s` of 300s this is self-limiting in practice (few distinct entries expected) but would not scale if many unique routes were requested.
- **CODE-LEVEL OBSERVATION**: `RoutePlanner`/`GPSReader`/`UltrasonicSensor` are all O(1) per-tick; no unbounded array growth was found (deques in `tracking.py`/`filter.py` all use `maxlen`).
- **INFERRED RISK**: `EncoderReader`'s background sampler (`encoders.py:89-119`) and `UltrasonicSensor`'s poller (`ultrasonic.py:56-67`) are independent Python `threading.Thread`s running alongside the `asyncio` event loop used by FastAPI/the controller — on the CPython GIL, heavy `asyncio.to_thread` detection work could contend with these hardware-critical polling threads for real CPU time on a Pi 5's cores, though the Pi 5's 4 cores likely have enough headroom; this was not benchmarked in this codebase (no benchmark code exists to trace — **OBSERVED BENCHMARK RESULTS: none exist**).
- **[MISSING]** No load testing, no profiling artifacts, no benchmark scripts anywhere in the repo.

---

## 31. Testing

There is **no automated test suite** (no `pytest`, no `unittest`, no `tests/` directory, no assertions-based test file). What exists instead are **five interactive/manual hardware-exercise scripts** plus one perception-debug script, none of which are runnable in CI without physical hardware or a display:

| Script | What it actually does | Mocked vs real | Automated? |
|---|---|---|---|
| `test_motor.py` | Drives real motors forward/stop/backward/stop using `robotx.hardware.motors` + live `SETTINGS` | Real GPIO | No — manual, timed, requires human observation ("keep wheels off the ground") |
| `test_gps.py` | Prints real NMEA fixes every 2s using `robotx.navigation.gps` | Real serial hardware | No — infinite loop, human-terminated |
| `test_ultrasonic.py` | Prints real HC-SR04 distance continuously | Real GPIO | No — infinite loop, human-terminated |
| `test_controller.py` | Reimplements a simplified STOP/FORWARD decision loop using real motors/ultrasonic/IR/encoders directly (does **not** import `robotx.control.controller` at all — it's a standalone rewrite, not a test of `RobotController`) | Real GPIO | No — infinite loop |
| `test_cv.py` | Opens a live Picamera2 preview (or headless `frame.jpg` writes), optionally runs `ObjectDetector` | Real camera | No — interactive/manual |
| `test_vision.py` | The **only** exerciser of `VisionController`/`DecisionEngine`; runs a headless detection+decision loop, prints structured debug output, writes debug frames | Real camera, real detector | No — long-running loop, human-monitored |

**Consequences:**
- **`RobotController` — the actual production control loop wired into the FastAPI app — has zero automated or even manual standalone test coverage.** `test_controller.py`'s name is misleading: it never imports or exercises `robotx.control.controller.RobotController`; it's an independent, simpler reimplementation.
- **No unit tests exist for pure logic that could trivially be unit-tested without hardware**: `haversine_m`/`bearing_rad` (pure math), `RoutePlanner.should_reroute()` (pure state machine), `decode_polyline()` (pure string parsing), `ObjectTracker`/`PrimaryObjectTracker` (pure data structures), `DecisionEngine.decide()` (pure function). All of these could run in CI with zero hardware and currently have no coverage.
- **No integration test** exercises the Socket.IO command→controller→motor path end-to-end.
- **No failure/reconnect scenario test** exists for the Socket.IO client.
- **Assignment-correctness testing**: not applicable (§15).

---

## 32. Deployment / Runtime

- **How the backend starts**: manually, via `venv/bin/uvicorn robotx.app.main:app --host 0.0.0.0 --port 8000` (`README.md:88-91`). **This command cannot currently succeed on this Pi** — verified directly:

  ```
  $ stat -c "%a %n" venv/bin/python
  664 venv/bin/python
  $ ./venv/bin/python -c "print('hi')"
  bash: ./venv/bin/python: Permission denied
  ```

  The venv's own Python interpreter binary is missing the execute bit (mode `664` instead of the normal `755`). **[CRITICAL DEPLOYMENT FINDING]** — as of this audit, the documented run command fails immediately with `Permission denied`, before any application code even loads. This is a filesystem/permissions issue on this specific Pi's copy of the repo, not a code defect, but it blocks every other verification step (nothing in this report was validated by actually *running* the app — all conclusions are from static code reading, since the venv cannot currently execute).
  - Separately, the venv **does** have most of `requirements.txt` already `pip install`-ed (`fastapi`, `uvicorn`-family, `cv2`/opencv, `httpx`, `aiohttp`, `engineio` were all found under `venv/lib/python3.13/site-packages/`), so once the execute bit is restored (`chmod +x venv/bin/python*` or recreating the venv), it is likely close to runnable — modulo whatever else `pip check` would reveal, which was not run given the permission blocker.
- **System Python** (outside the venv) has `RPi.GPIO`, `picamera2`, and `pyserial` available (apt-installed), but **not** `fastapi`, `cv2`, `socketio`, `httpx`, or `pynmea2` — confirming the app is meant to run from the venv (for the pip-only deps) while relying on `--system-site-packages` for the apt-only-installable `picamera2` (as `TEST_README.md:192-196` explicitly instructs, though it's unclear from the filesystem alone whether *this* venv was created with `--system-site-packages`, since `picamera2` isn't itself pip-installed into it).
- **Frontend start**: N/A, no frontend.
- **Workers**: N/A, no separate worker processes — everything runs in the single uvicorn process (background hardware threads + asyncio tasks live inside it).
- **PM2 / systemd / Docker**: **[MISSING]** — confirmed no systemd unit file exists in `/etc/systemd/system/` matching "robot", no Docker/Compose files in the repo, no PM2 config. **RobotX does not currently run as a Pi service** — it must be started manually in a terminal per the README, and will not survive a reboot or crash without a human re-running the uvicorn command.
- **Ports**: `8000` (FastAPI/uvicorn, configurable via `ROBOTX_API_PORT`) is the only port this application binds. It initiates *outbound* connections to whatever port the external Socket.IO server listens on (server-side, not this repo's concern).
- **Health checks**: `/health` exists and is suitable for an external health-check probe, but nothing in this repo currently calls it periodically or restarts the process on failure — no watchdog.
- **Logging**: to stdout/stderr only (`logging.basicConfig`, `main.py:22-26`) plus raw `print()` in the detection modules (§30) — **no log file rotation, no structured/JSON logging, no log shipping** exists.

---

## 33. Configuration

All configuration is centralized in `robotx/utils/config.py` as a frozen dataclass, `SETTINGS`, populated once at import time from environment variables via small `_env`/`_env_int`/`_env_float`/`_env_bool` helpers (`config.py:6-28`). No secret values are reproduced below.

| NAME | PURPOSE | USED BY | REQUIRED/OPTIONAL | DEFAULT | WHAT BREAKS IF MISSING |
|---|---|---|---|---|---|
| `ROBOTX_ROBOT_ID` | Robot's self-reported identity string | `sockets.py` (`robot_hello`), telemetry | Optional | `robotx-pi` | Nothing breaks; robot identifies itself with a generic default, indistinguishable from any other unconfigured unit |
| `ROBOTX_LOG_LEVEL` | Root logger level | `main.py:_setup_logging` | Optional | `INFO` | Falls back to `INFO` |
| `ROBOTX_API_HOST` / `ROBOTX_API_PORT` | FastAPI bind address/port | uvicorn invocation | Optional | `0.0.0.0` / `8000` | Falls back to binding all interfaces on 8000 |
| `ROBOTX_SOCKET_SERVER_URL` | Where the robot connects for real-time commands/telemetry | `sockets.py` | **Effectively required** for any remote control | `http://localhost:3000` | Robot will try (and fail, non-fatally) to connect to `localhost:3000`; it will still run its local control loop, but no commands can ever arrive |
| `ROBOTX_SOCKET_NAMESPACE` | Socket.IO namespace | `sockets.py` | Optional | `/robot` | Must match the server's expected namespace or the connection/handshake will not route events correctly |
| `ROBOTX_SOCKET_RECONNECT` | Enable client auto-reconnect | `sockets.py` | Optional | `true` | Disabling means a dropped connection is never retried |
| `ROBOTX_MOTOR_LEFT_IN1/IN2/ENA`, `ROBOTX_MOTOR_RIGHT_IN3/IN4/ENB` | L298N GPIO pin numbers (BCM) | `main.py` motor wiring | Optional (must match actual wiring) | see `config.py:47-53` | Wrong/missing values drive the wrong physical pins — could be a wiring hazard, not just a software no-op |
| `ROBOTX_MOTOR_PWM_HZ` | PWM frequency | `MotorDriver` | Optional | `1000` | Falls back to 1000 Hz |
| `ROBOTX_MOTOR_INVERT_LEFT/RIGHT` | Flip a motor's direction in software | `MotorDriver` | Optional | `false` | Wrong wiring polarity would need this to avoid moving backward when told to go forward |
| `ROBOTX_ENCODER_LEFT_PIN/RIGHT_PIN`, `_PULSES_PER_REV`, `ROBOTX_WHEEL_DIAMETER_M` | Encoder GPIO + speed-math calibration | `EncoderReader` | Optional | see `config.py:60-63` | Wrong pulses-per-rev/diameter silently produces wrong speed telemetry and a miscalibrated PI speed controller |
| `ROBOTX_ULTRASONIC_TRIGGER_PIN/ECHO_PIN`, `_POLL_HZ` | HC-SR04 wiring/rate | `UltrasonicSensor` | Optional | see `config.py:66-68` | Wrong pins = no/garbage distance readings, directly weakening obstacle avoidance |
| `ROBOTX_IR_LEFT/RIGHT/CENTER_PIN`, `_ACTIVE_LOW` | IR sensor wiring/polarity | `IRSensors` | Optional | see `config.py:71-74` | Wrong polarity inverts triggered/not-triggered logic |
| `ROBOTX_CAMERA_INDEX/WIDTH/HEIGHT/FPS` | Camera capture parameters | `CameraStream` | Optional | `0`/`640`/`480`/`20` | Falls back to defaults; `index` is actually unused by Picamera2 path (`camera.py:19`) |
| `ROBOTX_DETECTION_BACKEND` | `opencv`\|`yolo`\|`auto` | `ObjectDetector` | Optional | `opencv` | `auto` prefers OpenCV unless `ROBOTX_AUTO_PREFER_YOLO=1` |
| `ROBOTX_YOLO_MODEL_PATH` | Path to a YOLOv8 `.pt` weights file | `ObjectDetector._init_yolo` | Required only if using the `yolo` backend | `yolov8n.pt` | `RuntimeError` if `ultralytics` isn't installed or the weights file is missing/unreachable |
| `ROBOTX_DETECTION_MIN_CONF` | Minimum detector confidence | `ObjectDetector` | Optional | `0.35` | Falls back to default threshold |
| `ROBOTX_GPS_PORT` / `ROBOTX_GPS_BAUDRATE` | Serial device/baud for GPS | `GPSReader` | Optional (must match actual wiring) | `/dev/ttyAMA0` / `9600` | Wrong port = GPS reader silently sits in an exception loop, never producing a fix (`gps.py:60-62`) |
| `ROBOTX_GOOGLE_MAPS_API_KEY` | **Secret** — Google Directions API key | `GoogleMapsDirections` | Required for any `START`/`RETURN` routing | none | `RuntimeError("Google Maps API key missing")` on the first route request — robot can still be driven `MANUAL` but never gets an `AUTO` route |
| `ROBOTX_DIRECTIONS_MIN_INTERVAL_S` / `_CACHE_TTL_S` | Directions API rate-limit/cache tuning | `GoogleMapsDirections` | Optional | `15.0` / `300.0` | Falls back to defaults |
| `ROBOTX_CONTROL_HZ` | Control loop frequency | `RobotController` | Optional | `10.0` | Falls back to 10 Hz |
| `ROBOTX_TELEMETRY_INTERVAL_S` | Telemetry emit cadence | `RobotController`/`RobotSocketClient` | Optional | `1.5` | Falls back to default |
| `ROBOTX_OBSTACLE_DISTANCE_CM` | Ultrasonic obstacle threshold | `RobotController._safety_blocked` | Optional | `35.0` | Falls back to default; too small a value directly weakens safety margin |
| `ROBOTX_AVOID_TURN_SECONDS` / `_AVOID_FORWARD_SECONDS` | Blind-avoidance maneuver timing | `RobotController._avoid` | Optional | `0.5` / `0.6` | Falls back to defaults |
| `ROBOTX_TARGET_SPEED_MPS` / `ROBOTX_MAX_MOTOR_DUTY` | Cruise speed target / hard duty ceiling | `RobotController`, `MotorDriver` | Optional | `0.25` / `0.75` | Falls back to defaults; `MAX_MOTOR_DUTY` is a genuine safety ceiling — raising it directly raises max physical speed |
| Various `ROBOTX_MOG2_*`, `ROBOTX_STATIC_*`, `ROBOTX_COLLISION_THRESHOLD`, `ROBOTX_MEDIUM_THRESHOLD`, `ROBOTX_LOW_CONF`, `ROBOTX_FAST_APPROACH_THRESHOLD`, `ROBOTX_ROI_TOP_RATIO`, `ROBOTX_DISABLE_ROI`, `ROBOTX_SAVE_DEBUG`, `ROBOTX_DEBUG_PATH`, `ROBOTX_HEADLESS`, `ROBOTX_AUTO_PREFER_YOLO` | Fine-grained detection/decision tuning knobs (mostly `detection.py`/`vision_controller.py`) | Various | Optional | see individual `os.environ.get(...)` calls | Each falls back to an in-code default; none are required for the app to boot |

---

## 34. Code Quality / Technical Debt

Ranked by severity with concrete reasons:

**CRITICAL**
1. **`MANUAL` mode bypasses all obstacle-avoidance safety checks** (`controller.py:351-352`). A remote operator (or, per §29, anyone who can reach the unauthenticated Socket.IO namespace) can drive the robot into an obstacle the same firmware would otherwise stop for in `AUTO` mode. Concrete failure: send `{"type":"MANUAL","payload":{"action":"forward","speed":1.0}}` toward a wall — ultrasonic/IR/vision are read every tick but never consulted while `_mode == "MANUAL"`.
2. **No authentication on the command channel** (§11, §29) — same class of risk, systemic rather than a single code path.

**HIGH**
3. **`VisionController` + `DecisionEngine` (the more sophisticated, hysteresis-based, motion-depth-aware avoidance logic) are completely disconnected from the running application** (§14, §39) — they exist only for `test_vision.py`. This is a large amount of well-developed logic (798 + 36 lines) that provides zero production benefit today; either it should be wired into `RobotController` (replacing the much simpler `_safety_blocked`/`_avoid` logic) or it should be clearly labeled as an experimental/reference module so future maintainers don't assume it's active.
4. **`ObjectDetector.detect_objects()` prints on every call** (§30) — 6-10 `print()` statements per invocation, at 2-5 Hz, is meaningful, unbuffered I/O overhead on a Pi and pollutes stdout logs; should route through the already-configured `logging` module at `DEBUG` level.
5. **Ultrasonic sensor failure (`None` reading) is treated as "no obstacle" rather than an error state** (§28) — a disconnected/broken sensor silently degrades safety rather than triggering a fail-safe stop or at least a distinct warning mode.

**MEDIUM**
6. **`test_controller.py` does not test `RobotController`** despite its name — it's an independent reimplementation of a much simpler decision rule. This is misleading to a future maintainer looking for coverage of the real production controller (§31).
7. **Duplicated obstacle-avoidance logic across three places**: `RobotController._safety_blocked`/`_avoid` (simple threshold + blind turn), `DecisionEngine.decide()` (36-line rule engine, fixed pixel thresholds `213`/`426`/`30000`/`10000` hardcoded rather than derived from `frame_w` the way `detection.py`'s `_zone_for_cx` does it), and `VisionController.step()` (the most sophisticated version, with its own re-implementation of zone/threshold logic). These three have **different magic numbers for conceptually the same thresholds** (e.g., collision-area threshold is `15000` in `detection.py:365`, `30000` in `decision_engine.py:25`, and `30000`/`10000` again but reused differently in `vision_controller.py:16-17`) — a change to tuning in one does not propagate to the others, and there is no single source of truth.
8. **Magic numbers throughout perception code** without named constants tied to physical units: e.g. `detection.py`'s `HARD_MIN_AREA = 800`, `FULL_SPAN_RATIO = 0.80`, MOG2 `varThreshold=32`, `ROBOTX_MOG2_WARMUP=20` frames — reasonable as tuning defaults, but scattered across module-level constants, `__init__` defaults, and env-var fallbacks inconsistently (some are class constants, some are `os.environ.get()` calls buried mid-function, e.g. `detection.py:684`, `734`).
9. **`camera_index`/`ROBOTX_CAMERA_INDEX` config exists but is unused** — `CameraStream.__init__` explicitly notes "`index` is accepted for backward compatibility but is unused with Picamera2" (`camera.py:19-20`) — dead configuration surface that could confuse an operator into thinking they can select a camera by index.
10. **Stray leftover debug comments** committed into production logic: `# 🔥 ADD THIS BLOCK (avoid single-noise detections)` (`detection.py:722`) and `# 🔥 ADD THIS LINE (ignore noisy top region)` (`detection.py:741`) — these read like inline AI/pair-programming scratch notes that were never cleaned up; harmless functionally but signal the file needs a pass for comment hygiene.

**LOW**
11. **`RobotSocketClient.emit_status()` is dead code** — defined (`sockets.py:88-89`) but never called anywhere (§19).
12. **Stale `__pycache__` reference to a `camera_picamera2.cpython-313.pyc`** with no corresponding source file (confirmed via `find`) — evidence the camera module was renamed/restructured at some point; harmless (pycache isn't tracked/shipped) but worth a `find . -name __pycache__ -exec rm -rf {} +` before packaging.
13. **No `pyproject.toml`/`setup.py`** — the package is only importable via `PYTHONPATH`/being in the working directory; there's no installable distribution, no `pip install -e .` story, no declared minimum Python version beyond what's implied by 3.13 venv on disk.

**No circular dependencies were found** — the module layering (`utils` → `hardware`/`navigation`/`perception` → `control` → `app`) is clean and one-directional; `control/controller.py` and `control/vision_controller.py` do not import each other, and neither imports `app/*`.

---

## 35. Actual vs Intended Architecture

| FEATURE | DOCUMENTED / INTENDED (per README/TEST_README) | ACTUAL CODE | STATUS | EVIDENCE | GAP | NEXT ACTION |
|---|---|---|---|---|---|---|
| FastAPI health/camera service | "FastAPI service (health + MJPEG camera streaming)" | Exactly this, two routes | [IMPLEMENTED] | `main.py:41-73` | None | — |
| Socket.IO telemetry/commands | "Real-time Socket.IO client (telemetry + instant commands)" | Exactly this, but unauthenticated | [IMPLEMENTED], insecure | `sockets.py` | No auth | Add token/HMAC auth to `robot_hello` + verify on every `command` |
| Hardware drivers | "motors, encoders, ultrasonic, IR" | All four real, GPIO-based | [IMPLEMENTED] | `robotx/hardware/*` | Encoders are single-channel (no direction sensing) | Acceptable for now; note limitation |
| Navigation (GPS + Directions + reroute) | "GPS + Google Directions route fetching + reroute logic" | All three real | [IMPLEMENTED] | `robotx/navigation/*` | No geofencing/zones | Add destination bounds-check if needed |
| Perception (OpenCV/YOLO) | "OpenCV camera + OpenCV detector; optional YOLOv8 backend" | Real, both backends work | [IMPLEMENTED] | `robotx/perception/*` | Advanced tracker/temporal-filter pipeline (`VisionController`) unused in production | Decide: promote `VisionController` into the live controller, or delete/relabel as experimental |
| "Central controller loop (safety + obstacle avoidance + route following)" | as stated | Real, but MANUAL mode skips safety checks | [PARTIALLY IMPLEMENTED] | `controller.py:316-415`, `351-352` | Safety gap in MANUAL mode | Route `_apply_manual` through `_safety_blocked` too, or add a distinct manual-override confirmation |
| Robot commissioning/registration | Not documented as existing | Absent | [MISSING] | — | No registration handshake | Out of scope for a single-robot repo unless a fleet server is introduced |
| Robot authentication | Not documented as existing | Absent | [MISSING] | §11 | No identity verification | Needed before exposing this to any untrusted network |
| Telemetry | "telemetry (every ~1-2s)" | Matches, ~1.5s default | [IMPLEMENTED] | `controller.py:379`, `config.py:97` | `battery.percent` is fake | Wire a real battery ADC (see §36) |
| Heartbeat | "status (optional)" | `status` event defined but never emitted; telemetry substitutes | [PARTIALLY IMPLEMENTED] | §13, §19 | No explicit heartbeat contract | Either wire `emit_status` in or drop it |
| Assignment / DTARO / campus isolation / dashboard / analytics / super admin / Redis / workers / scaling | Not documented in this repo's own README as existing | Absent | [MISSING] | §9, §10, §15, §16, §18 | These are fleet-platform concepts that belong to a different (not-present) system | Not applicable to this codebase's scope |
| Simulation | Not documented as existing | Absent (only GPIO mock stand-ins) | [MISSING] | §14 | No simulated-robot test harness | Consider adding one to enable CI testing of `RobotController._loop()` without hardware |
| Physical robot | Implicit (this whole repo targets one) | Real GPIO/Picamera2/serial hardware code | [IMPLEMENTED] | throughout | venv currently non-executable on this Pi (§32) | Fix venv permissions; then verify end-to-end on real hardware |

---

## 36. Physical Robot Integration Readiness

### A. Backend readiness
**Current state**: The Pi-side "backend" (FastAPI + controller + Socket.IO client) is functionally complete for a single robot talking to *some* external server implementing the documented protocol (`README.md:98-124`).
**Existing code**: `robotx/app/main.py`, `robotx/app/sockets.py`, `robotx/control/controller.py`.
**Missing code**: Authentication on the Socket.IO handshake; a real external server to talk to (out of this repo's scope, but nothing in this repo can be end-to-end verified without one).
**Dependencies**: A reachable `ROBOTX_SOCKET_SERVER_URL`.
**Required changes**: Add auth (token in `robot_hello` or a signed connection query param), verified server-side.

### B. Raspberry Pi readiness
**Current state**: Code correctly targets Pi-specific APIs (`RPi.GPIO`, Picamera2/libcamera) rather than generic Linux/USB-webcam APIs — this is real Pi-native integration, not a generic-Linux stand-in.
**Existing code**: entire `robotx/hardware/`, `robotx/perception/camera.py`.
**Missing code**: None for the sensors currently modeled; battery ADC readout is the one clear gap (§27).
**Dependencies**: `RPi.GPIO`/`picamera2` confirmed importable on this Pi's system Python; **venv currently broken (§32) — must `chmod +x venv/bin/python*` or rebuild the venv before anything can run.**
**Required changes**: Fix venv executable permissions; verify `python3-picamera2` is installed via apt (README already documents this); decide whether the venv needs `--system-site-packages` to see `picamera2` (unclear from filesystem inspection alone whether it currently does).

### C. ESP32 readiness
**Current state**: **Not part of the design at all.**
**Existing code**: none.
**Missing code**: everything, if this is ever wanted.
**Dependencies**: N/A today.
**Required changes**: N/A unless a future architectural decision introduces an ESP32 co-processor — would be new work, not a modification of existing files.

### D. Motor-control readiness
**Current state**: Real, working, fail-safe L298N differential-drive control.
**Existing code**: `robotx/hardware/motors.py`.
**Missing code**: None for the current L298N design. Would need new code only for a different motor driver IC (e.g., an ESC-based brushless setup).
**Dependencies**: Correct BCM pin wiring matching `config.py` defaults/env overrides.
**Required changes**: None required; consider adding a hardware watchdog/deadman-timer (auto-stop if no control-loop tick within N ms) as defense-in-depth.

### E. Sensor readiness
**Current state**: Ultrasonic, IR, encoders all real and wired into the control loop.
**Existing code**: `robotx/hardware/ultrasonic.py`, `ir.py`, `encoders.py`.
**Missing code**: Battery voltage sensing (§27); sensor-failure/staleness detection (§28).
**Dependencies**: Correct wiring per `README.md:47-55`.
**Required changes**: Add an ADC-based battery monitor (e.g., INA219 over I2C, or a voltage-divider + Pi's own ADC if using a HAT) and wire its reading into `controller.py:386` in place of the hardcoded `76.0`.

### F. GPS readiness
**Current state**: Real, generic NMEA GPS support.
**Existing code**: `robotx/navigation/gps.py`.
**Missing code**: No fix-quality/HDOP/satellite-count exposure; no UBX-specific handling for a u-blox M9N if higher-precision features are wanted.
**Dependencies**: Correct serial port/baud, antenna sky view.
**Required changes**: None required for basic operation; optional enhancement to expose fix quality in telemetry for better route-following decisions.

### G. Telemetry readiness
**Current state**: Fully wired, real sensor data end-to-end except battery.
**Existing code**: `controller.py:378-401`, `sockets.py:100-108`.
**Missing code**: Battery telemetry (real), persistence/history (if desired — currently entirely ephemeral).
**Dependencies**: A consumer on the other end of the Socket.IO connection (not in this repo).
**Required changes**: Replace hardcoded battery constant; consider whether any telemetry history is needed given there's currently zero persistence.

### H. Command/control readiness
**Current state**: Functional, but with the MANUAL-mode safety gap (§21, §28, §34) and no authentication (§11, §29).
**Existing code**: `controller.py:172-212`, `417-440`.
**Required changes**: Route `_apply_manual()` through (or alongside) `_safety_blocked()`; add command-source authentication.

### I. Safety readiness
**Current state**: Software-only obstacle avoidance with real fail-safe motor stops on error/shutdown; no hardware e-stop; ultrasonic failure isn't distinguished from "no obstacle."
**Existing code**: `controller.py:143-170`, `258-293`, `405-412`.
**Missing code**: Dedicated e-stop input; sensor-failure-as-hazard handling.
**Required changes**: Treat `distance_cm is None` (sensor failure/timeout) as a blocking condition rather than a pass-through; consider a physical e-stop GPIO input wired to force `_mode = "STOPPED"`.

### J. End-to-end readiness
**Current state**: **Cannot currently be verified at all on this Pi** because `venv/bin/python` lacks the execute bit (§32) — this blocks running the app, and therefore blocks verifying anything in sections A–I empirically rather than by static reading.
**Required first action, before anything else in this section**: `chmod +x venv/bin/python venv/bin/python3* venv/bin/pip*` (or recreate the venv), then run `venv/bin/python -c "import robotx.app.main; print('ok')"` (the import check the README itself recommends, `README.md:128-133`) to confirm the app can even be imported before attempting a full hardware run.

---

## 37. Critical Findings

1. **The venv on this Pi is currently non-executable** (`venv/bin/python` mode `664`) — the documented run command fails immediately (§32).
2. **`MANUAL` mode bypasses all obstacle/person-detection safety checks** — a genuine collision-risk code path (§21, §28, §34).
3. **No authentication anywhere on the Socket.IO command channel** — any network party able to reach the configured namespace can drive the robot or spoof its identity (§11, §29).
4. **Battery telemetry is entirely fabricated** (`{"percent": 76.0}`, hardcoded) — any consumer of this telemetry (dashboard, low-battery return-to-home logic, etc.) is working with fake data today (§14, §20, §27).
5. **The more advanced obstacle-avoidance pipeline (`VisionController`/`DecisionEngine`) is not connected to the running application** — only the simpler `RobotController`/`_safety_blocked` logic actually protects the physical robot in production (§14, §34, §39).
6. **Ultrasonic sensor failure is silently treated as "no obstacle"** rather than a hazard state (§28, §34).

---

## 38. Missing Components

- Battery voltage/current sensing hardware + software (§27, §36-E)
- Any authentication/authorization for the Socket.IO command channel (§11, §29, §36-H)
- Dedicated hardware emergency-stop (§28, §36-I)
- IMU/compass (heading is GPS-derived only, degrades or fails at low speed / stationary) (§24)
- Mission/task completion signaling (arrival detection, "mission complete" event) (§13)
- Robot registration/commissioning flow (§13)
- Persistence of any kind (database, telemetry history) (§9, §20)
- Automated test suite for `RobotController` (§31)
- systemd/process-supervision so the app survives reboots/crashes (§32)
- ESP32 or any microcontroller intermediary (confirmed entirely absent, not merely unused) (§22, §23)
- Zones/geofencing/campus-boundary concept for navigation (§17, §18)

---

## 39. Partially Implemented Components

- **Heartbeat**: substituted by telemetry cadence; no explicit ping/pong or staleness contract on the application layer (§13, §19).
- **Reroute logic**: exists and is real, but is a simple counter-threshold heuristic, not an optimized/cost-aware replanner (§16).
- **Obstacle avoidance**: two separate implementations exist at very different sophistication levels — the simple one (`RobotController`) is live; the sophisticated one (`VisionController`+`DecisionEngine`, with hysteresis, motion-depth trend estimation, and anti-flicker decision buffering) is fully built but only reachable from `test_vision.py` (§14, §34, §35).
- **Safety checking**: comprehensive in `AUTO`/`RETURN` modes, absent in `MANUAL` mode (§21, §28).
- **GPS-based heading**: works only when the robot has moved ≥1m between fixes (`_update_heading`, `controller.py:253`); stationary or slow-turning-in-place scenarios have no heading source at all, so route-following steering degrades to "no correction" (`controller.py:303-304`, returns `(base, base)` when `_last_heading is None`).

---

## 40. Recommended Fixes

In priority order, each tied to a concrete file:

1. **Fix venv permissions** — `chmod +x venv/bin/python venv/bin/python3*` (or recreate venv) so anything can actually be run and verified (§32).
2. **Gate `_apply_manual()` behind (or alongside) `_safety_blocked()`** in `controller.py` — at minimum, refuse a `forward`-class manual command when blocked, mirroring the `AUTO` behavior (§21, §28, §34).
3. **Add authentication to the Socket.IO handshake** in `robotx/app/sockets.py` — e.g., a shared-secret HMAC or bearer token included in the `connect()` `auth=` parameter (supported natively by `python-socketio`), verified server-side (§11, §29).
4. **Replace the hardcoded battery telemetry** in `controller.py:386` with a real read from an ADC/fuel-gauge IC once that hardware exists; until then, consider emitting `null` instead of a fabricated number so downstream consumers don't silently trust fake data (§14, §20, §27).
5. **Decide the fate of `VisionController`/`DecisionEngine`** — either integrate the more advanced pipeline into `RobotController` (replacing `_safety_blocked`/`_avoid`), or explicitly document them as an experimental/reference implementation not used in production, so future contributors don't assume they're active (§14, §34, §39).
6. **Treat ultrasonic `None` (sensor failure/timeout) as a blocking condition**, not a pass-through, in `_safety_blocked()` (`controller.py:258-259`) (§28, §34).
7. **Replace `print()` calls in `detection.py`/`vision_controller.py`** with `logging.getLogger(__name__).debug(...)` calls, gated by `ROBOTX_LOG_LEVEL` (§30, §34).
8. **Consolidate the three duplicated obstacle-threshold implementations** (`detection.py`, `decision_engine.py`, `vision_controller.py`) into one shared, parameterized module so tuning changes propagate consistently (§34).
9. **Add a minimal `pytest` suite for pure-logic code** that needs no hardware: `haversine_m`/`bearing_rad`, `RoutePlanner`, `decode_polyline`, `ObjectTracker`/`PrimaryObjectTracker`, `DecisionEngine` (§31).
10. **Add a systemd unit** so the FastAPI process is supervised and restarts automatically (§32).

---

## 41. Recommended Implementation Roadmap

Given what's actually in the codebase today (a working single-robot Pi stack, not a fleet platform needing to be built from scratch), the roadmap is about **hardening what exists**, not building the phases the generic brief assumed (there is no simulation-to-hardware migration needed — the code already targets real hardware):

**PHASE 1 — Make the existing stack runnable and verifiable**
- Files to modify: none (filesystem permission fix only: `venv/bin/python*`).
- Tests required: `venv/bin/python -c "import robotx.app.main; print('ok')"`, then `GET /health`.
- Acceptance criteria: uvicorn starts; `/health` returns `ok: true`.
- Risk: none — this is a prerequisite, not a design change.

**PHASE 2 — Close the safety gaps**
- Files to modify: `robotx/control/controller.py` (`_apply_manual`, `_safety_blocked`).
- Tests required: manual bench test — issue a `MANUAL forward` command with an obstacle in range and confirm the robot now stops; disconnect the ultrasonic sensor and confirm the robot now treats it as blocked rather than clear.
- Acceptance criteria: both scenarios above behave safely.
- Risk: over-restricting MANUAL mode could make legitimate close-quarters manual maneuvering (e.g., docking) impossible — may need an explicit, deliberate override flag rather than a blanket bypass-removal.

**PHASE 3 — Add command-channel authentication**
- Files to modify: `robotx/app/sockets.py` (client-side: send a token); the external server (out of repo scope) must verify it.
- New components: a shared secret / token issuance mechanism (not currently designed anywhere in this repo).
- Acceptance criteria: an unauthenticated connection attempt is rejected by the server; this repo's client successfully authenticates with a valid token.
- Risk: requires coordinated change with whatever server this robot talks to, which isn't in this repository — cannot be completed unilaterally from this codebase alone.

**PHASE 4 — Real battery telemetry**
- New components: an ADC/fuel-gauge driver module under `robotx/hardware/` (e.g. `battery.py`), following the same `Config`+reader-class pattern as `ultrasonic.py`/`ir.py`.
- Files to modify: `robotx/control/controller.py` (wire the new reader in, replace the hardcoded dict), `robotx/utils/config.py` (new pins/I2C address config), `robotx/app/main.py` (instantiate it in `on_startup`).
- Dependencies: choice of hardware (INA219 over I2C is a common, well-supported choice for a Pi).
- Acceptance criteria: telemetry's `battery.percent` reflects a real, changing value under load/charge.
- Risk: low — additive, doesn't touch existing control logic.

**PHASE 5 — Reconcile the two obstacle-avoidance pipelines**
- Files to modify: `robotx/control/controller.py` (if promoting `VisionController` logic in) or `TEST_README.md`/module docstrings (if formally demoting `vision_controller.py`/`decision_engine.py` to "experimental/reference only").
- Tests required: whichever direction is chosen, add coverage per Phase 6.
- Acceptance criteria: a future contributor reading the code has one unambiguous, documented obstacle-avoidance implementation to trust.
- Risk: promoting the more complex pipeline into production changes real robot behavior — should be bench-tested thoroughly before field use, per the existing README's own safety notes.

**PHASE 6 — Automated test coverage for pure logic**
- New files: a `tests/` directory, `pytest` added to `requirements.txt` (dev-only).
- Files covered: `planner.py`, `maps.py::decode_polyline`, `tracking.py`, `decision_engine.py` (or its replacement per Phase 5).
- Acceptance criteria: `pytest` runs green in CI with no hardware attached.
- Risk: none — purely additive.

**PHASE 7 — Process supervision**
- New files: a systemd unit (e.g. `/etc/systemd/system/robotx.service`, outside this repo's own tree, or checked in as a reference file within it).
- Acceptance criteria: `systemctl restart robotx` works; a killed process is auto-restarted; the service starts on boot.
- Risk: low.

**PHASE 8 (only if genuinely desired) — ESP32 co-processor**
- This is **new architecture**, not a fix to anything existing. Only pursue if there's a specific reason (e.g., needing hard-real-time PWM/encoder timing the Pi's Python `asyncio` loop can't guarantee). Would require: new firmware repo/directory, a defined serial/I2C protocol, and a new `robotx/hardware/esp32_bridge.py`-style module replacing direct GPIO calls in `motors.py`/`encoders.py`/`ultrasonic.py`/`ir.py`. Not recommended as a default next step given the current GPIO-direct design already works.

---

## 42. Exact Files Likely to Require Modification

| File | Why |
|---|---|
| `robotx/control/controller.py` | MANUAL-mode safety gate (§21/§28), ultrasonic-failure handling (§28), battery telemetry wiring (§27) |
| `robotx/app/sockets.py` | Add authentication to the connection handshake (§11) |
| `robotx/utils/config.py` | New config for any battery-sensor pins/I2C address, and any auth-token env var |
| `robotx/app/main.py` | Instantiate a new battery-sensor module in `on_startup` |
| `robotx/perception/detection.py`, `robotx/control/vision_controller.py` | Replace `print()` with `logging` calls (§30); resolve threshold duplication (§34) |
| `TEST_README.md` / module docstrings | Document the `VisionController`/`DecisionEngine` production-vs-test status explicitly, whichever way Phase 5 is resolved |
| New: `robotx/hardware/battery.py` | Real battery sensing driver (Phase 4) |
| New: `tests/` | Automated coverage for pure-logic modules (Phase 6) |
| New: systemd unit file | Process supervision (Phase 7) |

---

## 43. Risks and Dependencies

- **This audit could not run the application** on this Pi due to the broken venv permissions (§32) — every conclusion here is from static source reading and direct filesystem/import checks, not from an observed running system. Runtime behavior (actual GPIO timing accuracy, actual camera FPS achieved, actual Socket.IO round-trip latency) is unverified.
- **The external Socket.IO server this robot talks to is entirely out of scope** — its existence, protocol fidelity to `README.md:98-124`, and any auth it might already enforce could not be assessed, because it is not part of this repository.
- **Google Maps API key availability/quota** is an external dependency gating all `AUTO`/`RETURN` routing — not something this codebase controls.
- **Physical wiring correctness** (BCM pin assignments matching actual hardware) cannot be verified from source alone — it's asserted by `config.py` defaults and the README's wiring table, but a mismatch would cause silent hardware misbehavior, not a code error.
- **Python 3.13** is a very recent interpreter version; `RPi.GPIO`, `picamera2`, and other C-extension-backed packages should be double-checked for 3.13 wheel/ABI compatibility on Raspberry Pi OS if any install issues arise (not verified as an active problem here, since imports succeeded on system Python, but flagged as a dependency-freshness risk given the specific stack).

---

## 44. Final Current-State Summary

RobotX today is a **real, single-robot, Raspberry-Pi-native control stack** — not a fleet-management platform, not a simulation environment, and not primarily aspirational documentation. The hardware layer (motors, encoders, ultrasonic, IR, camera, GPS) is genuinely implemented against real Pi GPIO/serial/Picamera2 APIs and fails safe on error. The navigation layer does real waypoint-following math against real Google Directions routes. The perception layer has two real, working object-detection backends.

The gaps that matter most are not "missing features" in the grand fleet-platform sense the audit brief anticipated — they are **specific, fixable safety and security holes in an otherwise-working system**: MANUAL mode bypasses collision safety, the command channel has no authentication, battery data is fabricated, and (as of the moment of this audit) the deployed venv on this specific Pi cannot even execute due to a file-permission issue. None of these require a from-scratch rebuild; all are targeted, well-scoped fixes to specific files identified above.

---

## Appendix A — Important Files

- `robotx/app/main.py` — FastAPI entry point and composition root
- `robotx/app/sockets.py` — Socket.IO client (robot → external server)
- `robotx/control/controller.py` — production control loop (`RobotController`)
- `robotx/control/vision_controller.py` — advanced, test-only vision/decision pipeline
- `robotx/control/decision_engine.py` — 36-line rule engine, test-only
- `robotx/hardware/motors.py`, `encoders.py`, `ultrasonic.py`, `ir.py` — real GPIO drivers
- `robotx/navigation/gps.py`, `maps.py`, `planner.py` — GPS, routing, waypoint tracking
- `robotx/perception/camera.py`, `detection.py`, `tracking.py`, `filter.py` — camera + CV pipeline
- `robotx/utils/config.py` — all runtime configuration
- `test_motor.py`, `test_gps.py`, `test_ultrasonic.py`, `test_controller.py`, `test_cv.py`, `test_vision.py` — standalone hardware exercisers
- `README.md`, `TEST_README.md` — accurate, current setup/operational documentation

## Appendix B — Important Functions / Classes

- `RobotController._loop()` (`controller.py:316`) — the entire real-time decision cycle
- `RobotController._safety_blocked()` / `_avoid()` (`controller.py:258`, `267`) — obstacle-avoidance logic
- `RobotController.handle_command()` (`controller.py:172`) — command dispatch
- `MotorDriver.set_speed()` / `_apply_motor()` (`motors.py:143`, `125`) — actuation
- `RoutePlanner.should_reroute()` / `update_position()` (`planner.py:88`, `63`) — navigation state machine
- `GoogleMapsDirections.get_route()` (`maps.py:117`) — external routing call
- `ObjectDetector.detect_objects()` (`detection.py:313`) — production detection entry point
- `VisionController.step()` (`vision_controller.py:338`) — advanced, test-only decision pipeline
- `RobotSocketClient._setup_handlers()` (`sockets.py:39`) — all inbound event wiring

## Appendix C — Communication Events

See full table in §19. Summary: `robot_hello`, `telemetry`, `status` (robot→server, `status` unused); `command`, `manual` (server→robot); plus engine.io's own `connect`/`disconnect`.

## Appendix D — Database Models

None exist. See §9.

## Appendix E — Environment Variables

Full table in §33 (no secret values reproduced).
