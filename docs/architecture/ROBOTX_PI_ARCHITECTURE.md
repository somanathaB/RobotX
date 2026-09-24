# RobotX Raspberry Pi Agent — Architecture

**Scope:** the Raspberry Pi 5 only. The ESP32 motor/safety controller and the
FalconAut backend are outside this repository; this document marks where each
will attach, and nothing here implements either.

**Status:** describes the code on disk after the standalone-agent restructuring
(`ROBOTX_PI_FOUNDATION_IMPLEMENTATION_REPORT.md`). For what is and is not
working, see `ROBOTX_PI_CURRENT_STATE.md`.

---

## 1. What the Pi is

The Pi is the robot's **high-level brain**. It senses, decides, and publishes
what it wants to happen. It does not actuate.

```
    Camera ──────────► Perception ──┐
                                    │
    GPS ────► Localization ─────► Navigation ──► Decision ──► Motion Intent
                    │                   │            │             │
                    └───────────────────┴────────────┴─────► Robot State
                                                                  │
                                                    ┌─────────────┴────────┐
                                                    ▼                      ▼
                                               Telemetry              Health
                                                (local)            (diagnostics)
```

Everything above runs without an ESP32, without motors, and without a backend.
That is the point: the Pi's subsystems can be validated on their own.

**Motor authority belongs to the ESP32.** The Pi agent imports no motor driver
and touches no motor GPIO. It ends at `MotionIntent` — a normalized request
that something downstream may choose to honour.

---

## 2. Packages

```
robotx/
├── config/         Settings + logging setup           (leaf: depends on nothing)
├── hardware/       Physical I/O: camera, GPS, battery-availability
│                   (+ motor/encoder/ultrasonic/IR drivers for bench use only)
├── localization/   GPS fixes -> position + heading
├── perception/     Frames -> detections -> PerceptionResult
│   └── experimental/   Older vision pipeline, NOT production (see its README)
├── navigation/     Route progress -> desired heading and target waypoint
├── control/        Decision -> MotionIntent   (+ retained legacy direct-drive loop)
├── diagnostics/    Health monitoring and host metrics
├── state/          The authoritative robot state + telemetry aggregation
├── communication/  Socket.IO link to the FalconAut backend (off by default)
└── application/    Agent lifecycle + local HTTP API (composition root)
```

### Dependency direction

```
config  ◄── hardware ◄── localization ◄── navigation ◄── control ◄── state ◄── application
            ▲                                  ▲            ▲         ▲
            └── perception ────────────────────┴────────────┘         │
            diagnostics ──────────────────────────────────────────────┘
```

Verified acyclic at both module and package level. `communication` is a leaf
that nothing imports.

---

## 3. Responsibilities

### `config` — configuration and logging
`settings.py` holds one frozen `Settings` dataclass. Defaults are literals on
the fields; `Settings.from_env()` overlays `ROBOTX_*` environment variables.
**No module outside this package reads `os.environ`.** A malformed numeric
value raises at startup rather than silently falling back — a misconfigured
robot should fail loudly. Secrets have no defaults and are reported as
`SET`/`UNSET` by `public_summary()`, never echoed.

`logging_setup.py` configures logging once and provides `log_event()`, which
tags lifecycle transitions with stable, greppable names (`camera.connected`,
`gps.fix_acquired`, `health.changed`, `decision.changed`).

### `hardware` — physical I/O
| Module | Purpose | In the agent? |
|---|---|---|
| `camera.py` | Picamera2 capture, BGR frames, explicit `CameraStatus` | Yes |
| `gps.py` | Serial NMEA, explicit `GPSStatus`, reconnect | Yes |
| `battery.py` | Single source of truth: no battery sensing exists | Yes |
| `motors.py`, `encoders.py`, `ultrasonic.py`, `ir.py` | GPIO drivers | **No** — bench scripts and the legacy loop only |

The camera device is held by one process-wide reference-counted manager, since
libcamera permits a single open handle. `CameraStream` is a consumer handle.

### `localization` — position and heading
Turns a `GpsReading` into a `Position`. This is the only place GPS becomes
geography.

**Heading is a known weak point and is labelled as such.** The robot has no
compass, no IMU and no magnetometer, so it has *no heading at all while
stationary*. Every `Position` carries a `heading_source`:

| Source | Meaning |
|---|---|
| `NMEA_TRACK` | Course over ground from the receiver's RMC sentence, trusted only above a minimum speed |
| `GPS_TRACK` | Bearing between two consecutive fixes far enough apart to be movement rather than GPS wander |
| `NONE` | Stationary or insufficient movement — heading is `None` |

Both sources describe the direction the robot **moved**, not the direction it
**faces**. They coincide only while driving forward in a straight line.
Neither can tell that the robot is facing backwards or rotating in place.

### `perception` — what the camera sees
One production path: `PerceptionPipeline` runs `ObjectDetector` on the latest
frame, on its own thread at `detection_hz`, so inference never stalls the agent
loop. The agent reads the most recent `PerceptionResult`.

`PerceptionResult.status` is the important part:

| Status | Meaning |
|---|---|
| `OK` | A frame was captured and processed — the detection list is meaningful |
| `NO_FRAME` | Camera delivered nothing, or its frames went stale |
| `DETECTOR_ERROR` | Inference raised |
| `STALE` | The last result has aged past the limit |
| `DISABLED` | Perception is switched off |

"I saw nothing" (`OK` with no detections) and "I could not see" (anything else)
are distinct values, and the decision layer treats only the first as clearance.

**No distance, depth, or time-to-collision field exists anywhere in the
perception output.** The camera is monocular with no depth sensor, no stereo
pair and no calibrated object-size table; it cannot measure distance. What it
does support is bounding-box area in pixels and that area's change over time,
exposed as `area_px` and `largest_area_delta_px` — an uncalibrated "is it
getting bigger" signal, named so it cannot be mistaken for metres.

`perception/experimental/` holds an older pipeline that is not wired into
anything. See its README for what it is and why it was not promoted.

### `navigation` — where to aim
`Navigator` consumes a `Position` and a route and produces a `NavigationState`:
target waypoint, distance, desired heading, signed heading error, progress, and
a status (`IDLE` / `NO_POSITION` / `NAVIGATING` / `ARRIVED` / `REROUTE_NEEDED`).
`RoutePlanner` tracks waypoint advance and off-route/blocked accumulation.

Navigation never touches GPIO, never picks a speed, and never talks to a motor.
Routes come from a local waypoint list; the Google Directions client remains
available but is not required — the agent runs fully offline without it.

### `control` — deciding, not driving
`decision.py` turns navigation + perception into a `MotionIntent`. Its posture:
**anything not positively known is a reason to stop.**

| Situation | Intent |
|---|---|
| No active mission | `HOLD` |
| Perception not `OK` (stale, errored, no frame, disabled) | `STOP` |
| Person detected, at any size or position | `STOP` |
| No GPS position | `STOP` |
| Obstacle in the drive corridor, large or growing fast | `STOP` |
| Obstacle in the corridor, smaller | `TURN_LEFT`/`TURN_RIGHT` away from it |
| Route clear, heading known | `FORWARD` with proportional steering |
| Route clear, heading unknown | `FORWARD` at creep speed (to establish a GPS track) |

`motion.py` defines `MotionIntent`: `command`, `left`, `right`, `reason`,
`timestamp`. Velocities are **normalized to [-1, 1], not PWM or duty cycle** —
the Pi does not know the gear ratio, battery voltage or H-bridge limits, so it
must not speak in those units. Converting intent to duty is the motor
controller's job, which means recalibrating the drivetrain needs no Pi change.

`robot_controller.py` is the **retained legacy direct-drive loop**. The
application no longer starts it. It is kept because it carries reviewed safety
behaviour (R-01, R-03) that is the reference for whatever runs on the ESP32,
and because it remains usable for bench-testing the drivetrain.

### `diagnostics` — health
`HealthMonitor` combines per-subsystem statuses with host metrics read from
`/proc/stat`, `/proc/meminfo`, `/proc/loadavg`, `/proc/uptime`, `statvfs` and
`/sys/class/thermal` — no extra dependency, and anything unreadable is reported
as `null` rather than zero.

States: `HEALTHY` / `DEGRADED` / `FAILED` / `UNKNOWN`. `UNKNOWN` means "not
measurable on this hardware" (for example, a disabled camera) and degrades the
overall status rather than failing it; the worst component wins.

### `state` — one authoritative representation
`RobotState` is the single place holding mode, GPS, position, navigation,
perception, motion intent, communication and health. Subsystems write once per
tick; readers call `snapshot()` for a consistent immutable copy. **No module
keeps a parallel copy of the robot's mode, position or intent.**

`telemetry.py` builds the local telemetry payload from that snapshot — the one
served on `GET /telemetry`. The backend link builds its own payloads in
`communication/protocol.py`, shaped to the FalconAut data model rather than to
the Pi's internals; both read the same snapshot, so they cannot disagree about
what the robot is doing.

Unknowable values are explicit: battery reports
`{"status": "UNAVAILABLE", "percent": null, ...}`. It is never a number.

### `application` — lifecycle and local API
`agent.py` owns the startup order, the loop, and shutdown:

```
configuration -> logging -> camera -> perception -> GPS -> navigation/state -> running
```

Startup **degrades rather than refusing**: a missing camera or unopenable
serial port does not stop the agent, because the rest must remain testable.
What it never does is treat a missing subsystem as a healthy one — the decision
layer stops the robot whenever perception is unusable. Shutdown releases in
reverse: perception thread, GPS serial, camera device.

`main.py` is a thin HTTP window onto the agent:

| Endpoint | Purpose |
|---|---|
| `GET /health` | Overall health, per-subsystem status, host metrics |
| `GET /state` | Full authoritative robot state |
| `GET /telemetry` | The local telemetry payload |
| `GET /config` | Effective configuration (secrets shown as SET/UNSET) |
| `GET /backend` | Backend link state, protocol binding, message counters |
| `GET /camera` | MJPEG preview |
| `POST /mission/start` | Load a waypoint route, switch to AUTO |
| `POST /mission/stop` | Halt and clear the route |
| `POST /mission/pause` | Suspend the mission, keeping the route |
| `POST /mission/resume` | Resume a paused mission |
| `POST /mission/idle` | Return to IDLE |

Endpoints are unauthenticated: local trusted network only.

---

## 4. Where the ESP32 attaches

`robotx/esp32/` is the Pi side of the UART link, protocol v2 as specified in
`PROTOCOL.md` (framing `PAYLOAD*CRC4\n`, CRC-16/CCITT-FALSE, 115200 8N1 on
`/dev/ttyAMA0`). It is its own boundary, separate from `communication/`.

```
DecisionMaker -> MotionIntent -> SafetyGate -> SafetyDecision ─┐
                                                              ▼
RobotState <── agent.tick() <── Esp32Link.status()      Esp32Link.submit()
  controller       (per tick)        ▲                        │
  controller_diag                    │ one I/O thread owns the port
  communication.esp32                └── /dev/ttyAMA0 ◄───────┘
```

| Module | Role |
|---|---|
| `protocol.py` | CRC, strict inbound decoding (per-type documented fields), command encoding for **PING, STOP and DRIVE only** |
| `transport.py` | pyserial port (exclusive, `TIOCEXCL`), bounded line assembly |
| `state.py` | `Esp32LinkStatus`, `ControllerTelemetry`, `ControllerState`, `ControllerDiag` — values only |
| `link.py` | `Esp32Link`: the port's single owner, connection state, sequence, ACK matching, reboot detection |

Rules the link enforces:

- **Off by default.** `ROBOTX_ESP32_ENABLED=0`. With `ROBOTX_ESP32_TRANSMIT_ENABLED=0`
  it never writes; otherwise it sends a resync LF per connection and one PING
  to prove the Pi -> ESP32 direction. `ROBOTX_ESP32_MOTION_ENABLED=0` (default)
  means no STOP or DRIVE, ever.
- **Only the gate's output.** `submit()` accepts a `SafetyDecision`, never a
  raw intent. Vetoed, stopping or stale decisions become STOP; DRIVE is sent
  only while the link is UP and the ESP32 reports `motor_drive_available`.
  One pending motion command at most, dropped if not sent within 0.3 s,
  never retried, never replayed after a reconnect.
- **Reboots latch.** A second READY or `uptime_ms` going backwards latches the
  agent's existing emergency stop. Clearing it acknowledges the reboot; the
  link must then re-PING before it carries motion.
- **GPS stays off the UART.** The agent refuses to start GPS on the ESP32's
  device and reports GPS health FAILED with the reason.

`Esp32LinkStatus`: `DISABLED`, `DISCONNECTED` (retrying with capped backoff),
`CONNECTING` (port open; waiting for TELEMETRY — the boot I2C scan can take
~2 min — or for the PING ACK), `UP`, `STALE` (TELEMETRY stopped for 1 s),
`DEGRADED` (reboot latched, protocol mismatch, or 3 unanswered commands),
`STOPPING`. TELEMETRY goes to `snapshot.controller` and local telemetry; DIAG
goes to `snapshot.controller_diag` and health, never to the telemetry frame.

The ESP32 side still owns converting DRIVE units to PWM, the duty ceiling and
deadband, obstacle gating, and its own 2 s command watchdog.

## 5. Where the backend attaches

`communication/backend_link.py` is the one Socket.IO client. It reads an
immutable `RobotSnapshot` and calls four mission methods
(`stop_mission`, `pause_mission`, `resume_mission`, `return_to_base`). That is
the entire coupling — there is no import path from the communication package to
hardware, GPIO, navigation internals or the decision layer, and a test enforces
it by walking the AST of every module in the package.

```
RobotSnapshot ──► BackendLink ──► Socket.IO ──► backend
                      ▲                            │
              mission methods ◄── CommandExecutor ◄─┘
```

`ROBOTX_SOCKET_ENABLED` defaults to off, and the `socketio` import is inside
`RobotAgent._start_backend_link`, so the standalone agent never loads the
library at all.

**Status:** the Pi side is implemented and tested against a real Socket.IO
transport. The FalconAut backend is not present in this repository, so the
actual event names are unverified and configured through
`ROBOTX_PROTOCOL_FILE`; until one is supplied the link reports
`integrated: false`. See `docs/communication/ROBOT_BACKEND_PROTOCOL.md`.

---

## 6. Safety-relevant behaviour

| Behaviour | Where | Status |
|---|---|---|
| Unusable perception stops the robot | `control/decision.py` | Implemented |
| Person detection stops the robot | `control/decision.py` | Implemented |
| Missing GPS position stops the robot | `control/decision.py` | Implemented |
| Agent-loop exception stops the robot and records the error | `application/agent.py` | Implemented |
| GPS fix staleness is explicit, never silently reused | `hardware/gps.py` | Implemented |
| Camera stall is distinguishable from an empty scene | `hardware/camera.py`, `perception/pipeline.py` | Implemented |
| Ultrasonic non-VALID status blocks forward motion (R-03) | `hardware/ultrasonic.py`, legacy loop | Retained, legacy path only |
| MANUAL obeys the obstacle gate (R-01) | legacy loop | Retained, legacy path only |
| Hardware E-stop | — | **Not implemented** (ESP32 concern) |
| Comm-loss watchdog | — | **Not implemented** (needs a link to lose) |

The Pi cannot itself guarantee the robot stops: it has no motor authority. Its
guarantee is narrower and worth stating precisely — **it will not publish a
forward intent unless it positively knows the path is clear.**

### Timing of intent changes

Intent is produced by the agent loop (10 Hz default), so starting a mission
takes up to one tick to show a driving intent — the robot holds first, which is
the safe direction. **Stopping does not wait for a tick**: `stop_mission()`
publishes a STOP intent synchronously, so there is no window in which a stale
forward intent remains readable after a stop.
