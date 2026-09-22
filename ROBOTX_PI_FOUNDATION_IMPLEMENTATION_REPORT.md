# RobotX Pi — Standalone Agent Foundation: Implementation Report

**Date:** 2026-09-22
**Scope:** Raspberry Pi 5 only. No ESP32 work, no backend work, no motor control.
**Outcome:** The Pi is now a coherent standalone robot agent whose subsystems can
be run and validated without an ESP32, motors, or the FalconAut backend.

---

## 1. What was inspected

Every Python file in `robotx/` and `tests/` (5,291 lines before this work), plus
`README.md`, `TEST_README.md`, `requirements.txt`, `.gitignore`, and all five
existing architecture/audit/remediation documents.

The current source was treated as the source of truth. The prior audit's
findings were re-derived rather than trusted:

- **Confirmed still accurate:** competing perception pipelines, hardcoded
  battery telemetry, `print()` in hot paths, no automated tests, no health
  layer, no `.env.example`.
- **Found to be inaccurate:** the claim that configuration was centralized. It
  was not — **23 `os.environ` reads** lived outside `config/`, mostly inside the
  detector, so the documented "all configuration in one place" was false.
- **Found in addition:** `GPSReader` accepted positions from NMEA sentences
  that explicitly reported an *invalid* fix; the camera module contained a
  hidden second capture path (a module-level `get_frame()` singleton); the
  detector wrote debug JPEGs to `/tmp` from inside the inference path.

Hardware was inspected directly: camera is an **IMX708** (Camera Module 3) on
`/base/axi/.../imx708@1a`; serial devices present are `/dev/ttyAMA0` and
`/dev/serial0 → ttyAMA10`; `dialout`, `video` and `gpio` group membership
confirmed. Installed: `picamera2`, `cv2`, `pyserial`, `pynmea2`, `RPi.GPIO`,
`fastapi`. **Not** installed: `ultralytics`, `pytest`.

---

## 2. Architecture before

One FastAPI process started `RobotController`, a 470-line loop that read every
sensor, decided, **drove the L298N motors over GPIO**, and built a telemetry
dict inline. Alongside it sat a second, richer perception pipeline
(`VisionController` + `DecisionEngine`, 834 lines) that nothing in production
called.

```
main.py ──► RobotController ──► MotorDriver ──► GPIO ──► motors
               │  reads: gps, ultrasonic, ir, encoders, camera, detector
               │  builds telemetry inline, including "battery": {"percent": 76.0}
               └──► Socket.IO client (auto-connecting to localhost:3000)

VisionController + DecisionEngine  ──► (nothing; test script only)
```

Problems with this shape, against the target architecture:

- The Pi held motor authority, which belongs to the ESP32.
- No robot-state object existed; state was private fields on the controller.
- No health monitoring, no telemetry boundary, no motion-intent abstraction.
- Two perception pipelines, neither clearly labelled as the production one.
- The app auto-connected to a backend that does not exist, on startup.

---

## 3. Problems found

| # | Problem | Severity |
|---|---|---|
| 1 | Pi drove motors directly; no motion-intent boundary for the ESP32 | Architectural |
| 2 | `"battery": {"percent": 76.0}` reported as a real reading (R-04) | Honesty |
| 3 | No robot-state model; state scattered across controller privates | Architectural |
| 4 | No health monitoring at all; `/health` always returned `ok: true` | Gap |
| 5 | Two perception pipelines, production one not identifiable from the code | Ambiguity |
| 6 | `ObjectDetector` also did filtering, zone math, collision logic, decision recommendation, debug-image writing and 13 `print()`s | Mixed responsibility |
| 7 | 24 `print()` calls in hot paths (R-09) | Noise |
| 8 | 23 `os.environ` reads outside `config/`, contradicting the docs | Config drift |
| 9 | GPS accepted positions from sentences reporting an invalid fix; ignored satellites, altitude, speed, track; no reconnect after a failed serial open | Correctness |
| 10 | Heading derived from consecutive GPS fixes, undocumented as a limitation | Honesty |
| 11 | Camera had a hidden singleton capture path and a redundant second buffer thread | Duplication |
| 12 | Camera failure raised out of FastAPI startup, killing the whole app | Fragility |
| 13 | Untyped dict detections everywhere; no perception result model | Modelling |
| 14 | "Saw nothing" and "could not see" were the same empty list | **Safety** |
| 15 | Zero automated tests; all `tests/` files were manual hardware scripts | Coverage |
| 16 | Socket.IO client auto-connected at startup to a non-existent backend | Coupling |
| 17 | No `.env.example`; no documentation of required configuration (R-10) | Ops |

---

## 4. Changes made

### New modules

| Module | Responsibility |
|---|---|
| `config/logging_setup.py` | One logging configuration; `log_event()` for stable, greppable event names |
| `hardware/battery.py` | Single source of truth: no battery sensing exists (closes R-04) |
| `localization/position.py` | GPS fix → `Position`, with explicitly labelled heading sources |
| `perception/types.py` | `Detection`, `PerceptionResult`, `PerceptionStatus`, `FrameMetadata` |
| `perception/pipeline.py` | The one production perception path, on its own thread |
| `navigation/navigator.py` | Position + route → `NavigationState` |
| `control/motion.py` | `MotionIntent` — the Pi→ESP32 seam |
| `control/decision.py` | Navigation + perception → `MotionIntent` |
| `diagnostics/health.py` | Health aggregation + `/proc`-based host metrics |
| `state/robot_state.py` | The authoritative robot state |
| `state/telemetry.py` | One telemetry payload from one snapshot |
| `application/agent.py` | Lifecycle, startup order, the agent loop, shutdown |

### Rewritten

- **`application/main.py`** — now a thin HTTP window over `RobotAgent`. Uses the
  modern `lifespan` context manager instead of deprecated `@app.on_event`.
  Starts no motors and constructs no backend client. Added `/state`,
  `/telemetry`, `/config`, and local mission control so the Pi can be driven
  through a full mission without any external system.
- **`config/settings.py`** — defaults are now literals on the fields and
  `Settings.from_env()` overlays the environment, which makes settings testable
  and gives correct env semantics. A malformed numeric value now raises at
  startup naming the variable, rather than silently falling back. Added
  `public_summary()` with secret elision. Absorbed all 23 stray `os.environ`
  reads.
- **`hardware/gps.py`** — `GpsFix`/`GpsReading` with explicit `GPSStatus`.
  Rejects `GGA` quality 0 and `RMC` status `V` (previously accepted as
  positions). Parses satellites, altitude, speed over ground and track angle.
  Retries a failed serial open instead of silently giving up forever.
- **`hardware/camera.py`** — removed the hidden singleton path and the redundant
  second buffer thread. Added `CameraStatus` (distinguishing *starting* from
  *stalled* from *failed*), a `CameraError` carrying operator guidance, and
  recovery logging.
- **`perception/object_detector.py`** — detection only. Removed all `print()`s,
  the decision/collision logic, zone computation and `/tmp` debug-image writing.
  Tuning moved to a `DetectorConfig` built from settings. Detection algorithms
  themselves preserved.
- **`navigation/route_planner.py`** — added `route_length()`, `waypoint_index()`,
  `arrived()`; geo math now shared with `localization` rather than duplicated.

### Moved

- `perception/vision_controller.py`, `perception/decision_engine.py` →
  `perception/experimental/`, with a README explaining what they are, why they
  are kept, and the three concrete reasons they were not promoted.
- Position estimation → its own `localization/` layer, so `navigation` and
  `state` no longer import each other.

### Retained deliberately

- `control/robot_controller.py` — the legacy direct-drive loop, now carrying a
  prominent banner. **Not started by the application.** Kept because R-01 (the
  MANUAL forward gate) and R-03 (ultrasonic status blocking) are reviewed safety
  rules that should inform the ESP32 firmware, and because it remains useful for
  drivetrain bench work. Its GPS calls were adapted to the new reader and its
  fake battery value replaced.
- `communication/socket_client.py` — R-02's handshake token is intact. Now
  documented as the future backend boundary and **not constructed by the
  agent**; `ROBOTX_SOCKET_ENABLED` defaults to off.

---

## 5. Final architecture

```
                      ┌──────────────── RobotAgent (application/agent.py) ─────────────┐
                      │                                                                │
  Camera ──► Perception pipeline ──► PerceptionResult ──┐                               │
  (hardware)  (own thread, 2 Hz)                        │                               │
                                                        ▼                               │
  GPS ──► Localization ──► Position ──► Navigation ──► Decision ──► MotionIntent        │
  (hardware)                            (navigator)   (control)    (control/motion)     │
                                                        │                               │
                      │         everything above writes once per tick into              │
                      │                     RobotState (state/)                         │
                      │                          │                                      │
                      │            ┌─────────────┴─────────────┐                        │
                      │            ▼                           ▼                        │
                      │    Telemetry (local)          Health (diagnostics/)             │
                      └────────────────────────────────────────────────────────────────┘
                                             │
                                  FastAPI (application/main.py)
                          /health /state /telemetry /config /camera /mission/*

  Future seams (NOT implemented):
    MotionIntent ──X──► ESP32 (UART)          telemetry sink ──X──► FalconAut backend
```

Package dependency direction, verified acyclic at module and package level:

```
config ◄─ hardware ◄─ localization ◄─ navigation ◄─ control ◄─ state ◄─ application
           ▲                              ▲            ▲         ▲
           └─ perception ─────────────────┴────────────┘         │
           diagnostics ──────────────────────────────────────────┘
```

---

## 6. Folder and module responsibilities

| Package | Owns | Must not |
|---|---|---|
| `config/` | All `ROBOTX_*` reading; logging setup | Import any other robotx package |
| `hardware/` | Camera, GPS, battery availability (+ bench GPIO drivers) | Interpret data, know about routes |
| `localization/` | GPS fix → position + heading | Know about waypoints or serial ports |
| `perception/` | Frames → detections → `PerceptionResult` | Decide anything, actuate, print |
| `navigation/` | Route progress, target waypoint, desired heading | Pick speeds, touch GPIO |
| `control/` | Decision → `MotionIntent` | Drive motors (the agent path imports no motor driver) |
| `diagnostics/` | Health status and host metrics | Invent a metric it cannot read |
| `state/` | The one authoritative state + telemetry payload | Hold fabricated values |
| `communication/` | Future backend transport | Be started by the agent (it is not) |
| `application/` | Lifecycle, composition, HTTP API | Contain logic that belongs to a layer below |

---

## 7. Camera / CV status

**Camera — working, verified on this hardware.** IMX708 (Camera Module 3) via
Picamera2/libcamera. Opens at 640×480 RGB888, converted to BGR. Start took
0.21 s, stop 0.43 s, frames delivered continuously. One process-wide,
reference-counted device handle (libcamera permits only one). Status is explicit
(`STARTING` / `STREAMING` / `STALLED` / `FAILED`), so a stalled camera is
distinguishable from an empty scene. Capture errors are logged once, and ten
consecutive failures mark the device failed rather than silently retrying.

**CV — working, modest.** `PerceptionPipeline` is the single production path:
inference runs on its own thread at 2 Hz on a 320×240 downscale, bboxes scaled
back to full resolution. Measured **3.9 ms per inference** on live frames. The
OpenCV backend combines MOG2 background subtraction (moving obstacles) with a
Canny/contour fallback (static ones). YOLOv8 remains available as a backend but
`ultralytics` is not installed and needs a torch build this Pi lacks — so in
practice everything is labelled `obstacle`; `person` requires YOLO.

**No distance or depth field exists anywhere in the perception output**, and
this is deliberate and documented in `perception/types.py`. The camera is
monocular with no depth sensor, no stereo pair and no calibrated object-size
table. What is exposed instead is `area_px` and `largest_area_delta_px` — an
uncalibrated "is it getting bigger" signal, named so it cannot be read as metres.

Detection thresholds are configurable but **not calibrated** for this robot's
camera height and angle.

---

## 8. GPS status

**Partially working. The serial layer works; no receiver is transmitting.**

The port `/dev/ttyAMA0` @ 9600 opens successfully and the reader runs, but
**zero NMEA sentences were received** during testing — status correctly remained
`NO_FIX` with `sentences_seen: 0`. Either no module is connected or powered, or
it is on a different port (`/dev/serial0 → ttyAMA10` also exists on this Pi).

What the implementation now does (software-verified, not yet fix-verified):
parses `GGA` (fix quality, satellites, altitude) and `RMC` (validity, speed over
ground, track angle); **rejects sentences reporting an invalid fix** (`GGA`
quality 0, `RMC` status `V`), which the previous implementation accepted as
positions; ages a fix out to `STALE`; retries a failed serial open. Fields the
receiver does not send stay `None`. No receiver-specific capability is assumed.

**Heading limitation, stated plainly:** this robot has no compass, no IMU and no
magnetometer, so it has **no heading at all while stationary**. Two sources
exist, each labelled in `Position.heading_source`: `NMEA_TRACK` (the receiver's
course over ground, trusted only above a minimum speed) and `GPS_TRACK` (bearing
between consecutive fixes far enough apart to be real movement). Both describe
where the robot *went*, not where it *faces*.

---

## 9. Navigation status

**Implemented; validated in software only.** `Navigator` consumes a `Position`
and a route and produces a `NavigationState`: status, target waypoint,
distance to target and destination, desired heading, current heading, signed
heading error, waypoint index and progress. `RoutePlanner` handles waypoint
advance, arrival, and accumulating off-route/blocked evidence before calling for
a reroute (one bad fix does not trigger one).

Routes come from a local waypoint list, so the agent navigates fully offline.
The Google Directions client remains available but is not required and is unused
by default. Navigation touches no GPIO, picks no speed, and calls no motor.

Never validated on a moving robot.

---

## 10. Motion-intent status

**Implemented.** `MotionIntent(command, left, right, reason, timestamp)`, with
constructors for stop/hold/forward/reverse/turn, derived `linear`/`angular`, and
round-trip dict serialization.

Velocities are **normalized to [-1, 1], not PWM or duty cycle** — a deliberate
choice, since the Pi does not know the gear ratio, battery voltage or H-bridge
limits. Converting intent into hardware units is the motor controller's job,
which means recalibrating the drivetrain requires no Pi change. Out-of-range
values are clamped inside the frozen dataclass, so an invalid velocity cannot
leave the module. `timestamp` exists so a downstream consumer can refuse a stale
intent.

This is the ESP32 seam. **No UART code, no protocol, no framing was written.**

---

## 11. Robot-state status

**Implemented.** `RobotState` is the single authoritative holder: identity,
mode, GPS reading, position, navigation state, perception result, motion intent,
communication links, health, and timestamps. Thread-safe; readers get an
immutable `RobotSnapshot` rather than a live view.

No module keeps a parallel copy of mode, position or intent. Modes: `IDLE`,
`AUTO`, `STOPPED`, `ERROR`. A recorded error is not cleared by a mode change —
the agent stopping after a failure must not erase why it failed.

There is no battery, odometry or motor-feedback field, because the Pi does not
own those sensors. They are absent rather than filled with placeholders.

---

## 12. Telemetry status

**Implemented, local only.** `build_telemetry()` produces one payload from one
snapshot: schema version, timestamp, identity, mode, GPS, position, navigation,
perception summary, motion intent, battery, health, communication, last error.
JSON-serializable, verified by test.

Battery reports `{"status": "UNAVAILABLE", "percent": null, "voltage_v": null,
"source": "no battery sensing hardware on this robot"}`. **The hardcoded `76.0`
is gone** (closes R-04), and a test asserts it cannot return.

Telemetry is not connected to any backend. `RobotAgent.add_telemetry_sink()` is
where a future transport attaches, so the backend consumes this schema rather
than inventing a second one.

---

## 13. Health-monitoring status

**Implemented and verified on this Pi.** Components: camera, perception, GPS,
navigation, communication, system. States: `HEALTHY` / `DEGRADED` / `FAILED` /
`UNKNOWN`, worst component wins; `UNKNOWN` (not measurable here, e.g. a disabled
camera) degrades rather than fails.

Host metrics come from `/proc/stat`, `/proc/meminfo`, `/proc/loadavg`,
`/proc/uptime`, `statvfs` and `/sys/class/thermal` — **no new dependency**, and
anything unreadable is `null`, never zero. Live readings observed: CPU 4.8 %,
memory 51.5 % (1,965 MB available), CPU temp 41.4 °C, load 0.12, disk 17.7 %,
uptime 2,294 s.

---

## 14. Tests executed, with exact results

### Automated suite — `tests/unit/` (new; 7 files, 1,661 lines)

```
$ venv/bin/python -m unittest discover -s tests/unit -t .
Ran 162 tests in 0.392s
OK
```

Stdlib `unittest` — pytest is not installed and was not added. No hardware, no
network, no new dependencies.

| File | Tests | Covers |
|---|---|---|
| `test_config.py` | 9 | Defaults, env overrides, type coercion, malformed-value rejection, secret elision, immutability |
| `test_perception.py` | 25 | Detection model, no-distance-field guarantee, detector filtering, pipeline OK/NO_FRAME/STALE/DETECTOR_ERROR/DISABLED, area-delta tracking |
| `test_gps_and_position.py` | 27 | NMEA GGA/RMC parsing, invalid-fix rejection, garbage tolerance, staleness, geo math, all three heading cases |
| `test_navigation.py` | 18 | Waypoint advance, arrival, off-route and blocked rerouting, desired heading, heading error sign |
| `test_motion_and_decision.py` | 37 | Intent construction/clamping/serialization, every decision branch including all fail-safes |
| `test_state_telemetry_health.py` | 25 | State authority and snapshot isolation, telemetry schema, no fake battery, health aggregation, live metric sanity |
| `test_agent.py` | 21 | Full tick with fake subsystems, mission control, telemetry sinks, lifecycle, recovery from a failing tick |

### Validation checks

| # | Check | Result |
|---|---|---|
| 1 | `compileall robotx tests` | **PASS** — all files compile |
| 2 | Unit suite | **PASS** — 162/162 |
| 3 | Import every module | **PASS** — 38 modules, 0 failures |
| 4 | Manual scripts parse | **PASS** — all parse |
| 5 | Stale module paths | **PASS** — none in code or current docs |
| 6 | Hardcoded fake telemetry | **PASS** — none |
| 7 | `print()` in production code | **PASS** — 0 (was 24; experimental bench tool still prints by design) |
| 8 | Motor GPIO in the agent path | **PASS** — agent imports no motor/encoder/ultrasonic/IR module; `RPi.GPIO` never loaded |
| 9 | ESP32 integration | **PASS** — not implemented; no UART, no protocol module |
| 10 | Backend integration | **PASS** — `socketio` and `communication` never loaded by the agent |
| 11 | Dependency cycles | **PASS** — none at module or package level |
| 12 | `os.environ` outside `config/` | **PASS** — 0 (was 23) |
| 13 | Docs match the source tree | **PASS** — regenerated from the tree |

### Live hardware runs (read-only; no motor was energized)

| Run | Result |
|---|---|
| Camera open/capture/release | **PASS** — IMX708 at 640×480, frames `(480, 640, 3)`, start 0.21 s, stop 0.43 s |
| GPS serial open | **PASS** — port opened; **0 NMEA sentences received**, status correctly `NO_FIX` |
| Agent under uvicorn, real hardware | **PASS** — camera `HEALTHY`, perception `HEALTHY` (3.9 ms/frame), GPS `DEGRADED` ("receiver reports no fix"), overall `DEGRADED` |
| `GET /health`, `/state`, `/telemetry`, `/config` | **PASS** — all returned correct payloads; secrets reported `UNSET` |
| `GET /camera` | **PASS** — valid multipart MJPEG, JPEG frames confirmed in the byte stream |
| `POST /mission/start` then read state | **PASS** — mode `AUTO`, intent `STOP — no GPS position` (correct fail-safe, not forward motion) |
| `POST /mission/start` with lat 999 | **PASS** — HTTP 422 |
| Shutdown | **PASS** — perception stopped, GPS stopped, camera released, in order |

---

## 15. Hardware tests NOT executed

These were **not** run, and nothing in this report should be read as evidence
for them:

- **No GPS fix was ever acquired** — no receiver is transmitting on the
  configured port. Everything downstream of GPS is therefore untested against
  real data: real position, real heading, real route following.
- **No motor was driven. No wheel turned.** The agent has no motor authority and
  the legacy loop was not run.
- **No obstacle-avoidance behaviour was observed on a moving robot.** Avoidance
  is tested against synthetic detections only.
- **No ultrasonic or IR reading was taken.**
- **No outdoor navigation run.**
- **No sustained thermal or endurance run** — the longest live run was ~15 s.
- **No YOLO inference** — `ultralytics` is not installed.

`TEST_README.md` carries the manual checklist for all of these.

---

## 16. Remaining limitations

1. **The Pi cannot stop the robot.** It has no motor authority. Its guarantee is
   narrower: it will not *request* forward motion unless it positively knows the
   path is clear. Actually stopping is the ESP32's job.
2. **No GPS receiver is currently transmitting**, so the whole
   GPS → position → navigation chain is unproven against real data.
3. **No heading while stationary**, and a moving heading describes travel, not
   facing. No compass, IMU or magnetometer exists.
4. **No distance measurement.** Obstacle proximity is bounding-box area in
   pixels — coarse and uncalibrated.
5. **Detection is modest.** Motion and strong edges only. Not a reliable person
   detector without YOLO, which is not installed.
6. **Perception thresholds are uncalibrated** for this camera mounting.
7. **The HTTP API is unauthenticated.** Trusted local network only.
8. **No process supervision** (R-08 still open) — manual start.
9. **No geofencing.** Any lat/lon can be requested as a waypoint.
10. **The experimental pipeline is now clearly non-production**, but it is still
    a second body of vision code that will drift if nobody maintains it.

---

## 17. What should be done next

**Before anything else:** resolve the GPS receiver. Confirm wiring, power, baud
rate, and whether it is on `/dev/ttyAMA0` or `/dev/serial0`, then verify a real
fix outdoors with `tests/hardware/test_gps.py`. Until that works, the navigation
half of the agent is unproven.

Then, roughly in order:

1. **Calibrate perception** against the real camera mounting — the area
   thresholds in `.env.example` are placeholders, not measurements.
2. **Bench-validate the decision layer**: drive the robot by hand past obstacles
   and check the intents in `/state` against what you would have wanted.
3. **Define the Pi↔ESP32 UART protocol** from `MotionIntent` — a separate task,
   including the ESP32's duty of refusing a stale intent.
4. **Add process supervision** (systemd unit, R-08).
5. **Decide the experimental pipeline's fate**: either re-derive its thresholds
   from configuration and promote it deliberately, or delete it. Leaving it
   indefinitely is the option that rots.
6. **Authenticate the HTTP API** if the Pi will ever sit on an untrusted network.

**Not done here, and deliberately so:** ESP32 integration, UART, motor control,
backend integration. The Pi standalone foundation is complete and documented.

---

## Appendix: documents

| Document | Status |
|---|---|
| `docs/architecture/ROBOTX_PI_ARCHITECTURE.md` | Rewritten |
| `docs/architecture/DEPENDENCY_MAP.md` | Regenerated from the tree |
| `ROBOTX_PI_CURRENT_STATE.md` | New |
| `README.md` | Rewritten |
| `TEST_README.md` | Rewritten (automated + manual checklist) |
| `.env.example` | New (closes R-10) |
| `robotx/perception/experimental/README.md` | New |
| `ROBOTX_PI_REMEDIATION_PLAN.md` | Status updated (R-04, R-05, R-09, R-10 resolved) |
| `ROBOTX_ARCHITECTURE_CLEANUP_REPORT.md` | Marked historical |
| Earlier audit/refactor reports | Already carried historical banners |
