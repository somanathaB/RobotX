# RobotX Pi — Current State

What actually works on the Raspberry Pi today, what half-works, and what does
not exist. Scope is the Pi only; the ESP32 and the FalconAut backend are
separate systems and are out of scope here.

Last verified: 2026-09-22, on the Raspberry Pi 5 this repository lives on.
Latest pass: production-readiness audit (201 automated tests, all passing).

---

## Summary

The Pi runs as a standalone robot agent. It captures camera frames, detects
objects, reads GPS, estimates position, follows a waypoint route, decides what
motion it wants, and reports local telemetry and health — **without** an ESP32,
motors, or a backend.

It does not move the robot. It publishes intent. Motor authority is the
ESP32's, and that link does not exist yet.

---

## Implemented and verified on this hardware

| Capability | Evidence |
|---|---|
| Camera capture (Pi Camera Module 3 / IMX708) | Opened at 640x480, streamed BGR frames, released cleanly. Start 0.21 s, stop 0.43 s. |
| Perception pipeline end to end | Ran at 2 Hz on live frames, ~3.9 ms per inference at 320x240, OpenCV backend. |
| GPS serial acquisition | `/dev/ttyAMA0` @ 9600 opened successfully. |
| Agent lifecycle | Started, ran, and shut down cleanly under uvicorn; perception thread, GPS serial and camera all released in order. |
| HTTP API | `/health`, `/state`, `/telemetry`, `/config`, `/camera`, `/mission/*` all exercised against a live server. |
| MJPEG camera stream | Valid multipart JPEG stream served over HTTP. |
| Host health metrics | CPU 4.8 %, memory 51.5 %, temp 41.4 °C, load, disk and uptime all read from `/proc` and `/sys`. |
| Fail-safe decisions | With no GPS fix, an active mission produced `STOP — no GPS position`, not forward motion. |
| Configuration validation | Invalid waypoint rejected with HTTP 422; malformed numeric env var raises at startup. |

## Implemented, verified only in software

These have automated tests but the physical condition has not been reproduced.

| Capability | Status |
|---|---|
| NMEA parsing (GGA/RMC: quality, satellites, altitude, speed, track) | Tested against real-shaped sentences, including invalid-fix rejection. **No live fix has been parsed** — see below. |
| Heading estimation from GPS | Tested both sources and the stationary case. Never exercised on a moving robot. |
| Route following and waypoint advance | Tested with synthetic coordinates. Never driven. |
| Obstacle avoidance decisions | Tested with synthetic detections. Never validated against a real obstacle at a real distance. |
| Stale-GPS and stale-perception handling | Tested via injected timestamps. |
| Health aggregation | Tested; live metrics confirmed on this Pi. |

## Partially implemented

| Item | What works | What is missing |
|---|---|---|
| **GPS fix** | Serial port opens; the reader runs and reports status honestly. | **No GPS receiver is transmitting — confirmed by exhaustive probe, see below.** Until this is resolved, nothing downstream of GPS has been exercised with real data. |
| **Heading** | Two GPS-derived sources with explicit labelling. | No compass, IMU or magnetometer. The robot has **no heading at all while stationary**, and a moving heading describes where it *went*, not where it *faces*. |
| **Object detection** | OpenCV motion (MOG2) + static (Canny/contour) backends run in real time on the Pi. | Classification is coarse: everything is `obstacle` unless YOLO is used. Labels `person` only via YOLO, which is not installed. Thresholds are heuristic and have not been calibrated against this robot's camera mounting. |
| **Online routing** | Google Directions client with caching and rate limiting. | Needs an API key; unused by default. The agent runs on locally supplied waypoints instead. |

## Production-readiness audit (2026-09-22)

Every Pi responsibility validatable without GPS data, ESP32, motors or backend.

**Status key**

| Label | Meaning |
|---|---|
| **PASS** | Physically validated on this hardware |
| **PASS-SYNTHETIC** | Software behaviour validated with controlled test data; hardware not exercised |
| **BLOCKED-HARDWARE** | Cannot be validated — required hardware or data unavailable |
| **NOT-TESTED** | Intentionally not executed |

| Responsibility | Status | Evidence |
|---|---|---|
| Camera lifecycle and capture stability | **PASS** | 18.69 fps over 60 s, 0 stalls, σ 3.3 ms; start 133 ms / stop 424 ms; 5-min soak with no leak |
| Computer-vision pipeline | **PASS** | 4.6 ms mean inference on live frames; 0 false positives in 60 cycles at max sensor gain |
| Perception types and result model | **PASS-SYNTHETIC** | 25 tests; no distance/depth field anywhere in output |
| Perception failure vs "no object" | **PASS-SYNTHETIC** | `OK`+empty is usable; `NO_FRAME`/`STALE`/`DETECTOR_ERROR`/`DISABLED` are not. **Defect found and fixed** — see below |
| Robot state management | **PASS-SYNTHETIC** | Single authority; snapshot isolation verified; dead `record_error()` removed |
| Localization interfaces | **PASS-SYNTHETIC** | Real parser + estimator driven by generated NMEA; real GPS still reports UNAVAILABLE/NO_FIX |
| Navigation | **PASS-SYNTHETIC** | Full synthetic traverse start → waypoint → ARRIVED; 16 fixture-driven tests |
| MotionIntent generation | **PASS-SYNTHETIC** | Normalized [-1,1], no motor command issued; agent holds no motor driver |
| Telemetry and schema consistency | **PASS** | Live payload verified; battery `UNAVAILABLE`/`null`; JSON-serializable |
| Health / diagnostics | **PASS** | Live host metrics; per-subsystem status. **2 defects found and fixed** |
| Configuration and environment | **PASS-SYNTHETIC** | 9 tests; malformed values raise at startup; secrets elided |
| Logging and error handling | **PASS** | 0 `print()` in production; no silent swallowing; event-tagged transitions |
| Graceful startup / shutdown | **PASS** | Ordered release verified live: perception → GPS → camera |
| FastAPI application lifecycle | **PASS** | `lifespan` start/stop verified under uvicorn |
| Thread / resource cleanup | **PASS** | 5 → 5 threads over 5 min, → 1 after stop; no leftover processes |
| Dependency boundaries | **PASS** | 0 cycles at module and package level |
| Safety on camera/GPS/perception loss | **PASS-SYNTHETIC** | 7 mid-run degradation tests: each fault stops the robot on the next tick |
| Never controls motors / assumes ESP32 | **PASS** | `RPi.GPIO`, motor, encoder, socket modules all confirmed unloaded at runtime |
| **GPS hardware + NMEA** | **BLOCKED-HARDWARE** | Zero bytes; see below |
| **Navigation on real GPS data** | **BLOCKED-HARDWARE** | Blocked behind GPS |
| Motor actuation | **NOT-TESTED** | Out of scope — ESP32 owns motor authority |
| ESP32 integration | **NOT-TESTED** | Out of scope this pass |
| Backend integration | **NOT-TESTED** | Out of scope this pass |
| Multi-hour endurance | **NOT-TESTED** | Longest run 5 minutes |

### Defects found and fixed in this audit

1. **Perception result with no frame metadata bypassed the obstacle check
   (safety).** `is_usable` checked only `status`, and the corridor check
   silently no-opped when `frame` was `None` — so an `OK` result carrying a
   40,000 px obstacle dead centre produced `FORWARD` at cruise speed.
   Reproduced, fixed (`is_usable` now requires frame metadata), and locked down
   by regression tests.
2. **Health evaluation mixed four separate snapshots**, so a report could
   combine states from different instants. Now derived from one snapshot.
3. **Health lookups raised `KeyError` on an unrecognized status**, which would
   fail the agent tick and force `ERROR` mode. Now degrade to `UNKNOWN`.
4. **`CameraStream` reported its requested resolution, not the device's
   actual one** — misleading whenever libcamera hands back a different config.
5. **Unguarded read-modify-write** on the perception area-delta, reachable when
   a bench script calls `step_once()` while the pipeline thread runs.
6. **`/camera` hung forever** when the camera produced nothing, holding the
   connection open indefinitely. Now closes after ~5 s.
7. **Dead code**: `RobotState.record_error()` had no callers. Removed.

## Physical validation results (2026-09-22)

Measured on this Pi, not inferred. Steps refer to the agreed validation sequence.

### Step 1 — Camera: capture PASS, scene unusable

| Measure | Result | Verdict |
|---|---|---|
| Achieved frame rate | 18.69 fps (1,121 unique frames in 60 s) vs 20 configured | PASS |
| Frame interval | median 51.9 ms, p95 59.3 ms, max 60.6 ms, σ 3.3 ms | PASS |
| Capture stalls / frozen frames | 0 | PASS |
| `start()` / `stop()` latency | 133 ms / 424 ms | PASS |
| Thermal rise over 60 s | +1.1 °C | PASS |
| Mean brightness | 14.4 / 255 | **FAIL** |
| Sharpness (Laplacian variance) | 13 | **FAIL** |

The two failures are one cause, not two: **scene illuminance is 0.46 lux**, below
full moonlight. Sensor metadata confirms it is not a software fault — analogue
gain is pinned at maximum (16.0 of 16.0) and exposure sits at 49.5 ms.

**Frame rate caps exposure.** At 20 fps, exposure cannot exceed 50 ms whatever
the sensor's own limit. Lowering `ROBOTX_CAMERA_FPS` directly buys low-light
performance — relevant if the rover must operate at dusk.

Still requires a human: aiming/field-of-view check, focus against a known
target, and colour correctness (BGR channel order) against a known-coloured
object.

### Step 2 — Computer vision: PASS, with a measured blind spot

| Measure | Result | Verdict |
|---|---|---|
| False positives, 60 cycles at max sensor gain | 0 detections in 60/60 frames | PASS |
| Inference latency | mean 4.6 ms, p95 5.4 ms, max 6.6 ms (320×240) | PASS |
| Detects 100×100 px object | yes | PASS |
| Ignores 12×12 px speck | yes | PASS |
| Separates two objects | yes, 2 detections | PASS |
| Decision escalation as object grows | FORWARD → STOP | PASS |

The zero false-positive result is meaningful precisely *because* the scene is
dark: maximum analogue gain means maximum sensor noise, the worst case for a
motion detector. The pipeline did not phantom-brake.

**Measured blind spot: objects below ~90×90 px in the 640×480 frame are
invisible.** Measured floor 90 px, predicted 89 px. Detection runs at 320×240,
so a 90 px object is 45 px at inference scale (2,025 px²) against the 2,000 px²
`ROBOTX_STATIC_MIN_AREA` gate. Converting that to real-world object size at real
distance needs a tape measure, a known object, and light.

To lower the floor: raise `ROBOTX_DETECTION_INFERENCE_WIDTH/HEIGHT` (costs CPU)
or lower `ROBOTX_STATIC_MIN_AREA` (costs false positives).

### Step 3 — GPS: BLOCKED on hardware, not configuration

Exhaustively probed. **Zero bytes received** — not a baud mismatch, no signal:

- `/dev/ttyAMA0` and `/dev/ttyAMA10`, at 4800 / 9600 / 19200 / 38400 / 57600 /
  115200 — 0 bytes on all twelve combinations
- No USB serial device (`lsusb` shows root hubs only)
- No I²C device on any bus (`i2cdetect` bus 1 entirely empty)
- GPIO 14/15 correctly muxed as `TXD0`/`RXD0` (`a4` ALT function), RX idling high
- No serial console holding the port; no `serial-getty` service running

Note `/dev/serial0` symlinks to `ttyAMA10`, the **debug connector**, not the GPIO
header. `ttyAMA0` (RP1 `serial@30000`, enabled by `dtparam=uart0=on`) is the
GPIO 14/15 UART, so the configured default is correct.

**Re-tested after the GPS was powered and its antenna reconnected — unchanged.**
Still zero bytes on all twelve port/baud combinations. Additional evidence from
the second run:

- GPIO 15 (Pi RX) read **high on 2,000/2,000 samples** — line idle, nothing
  driving it.
- GPIO 14 (Pi TX) read **788 high / 712 low while transmitting** — the Pi's own
  UART transmitter demonstrably works and drives its pin.
- I²C re-scanned with the module powered: nothing on the header bus (bus 1).
  Buses 4/11 are the camera (EEPROM 0x50 + sensor); buses 13/14 show all-
  addresses-respond, the signature of a floating bus, not real devices.
- No USB serial device appeared after power-on.
- Only GPIO 2/3 (I²C) and 14/15 (UART0) are in ALT mode — **no other UART is
  enabled**, so a module wired to different pins would have no driver at all.

**Conclusion: the Pi side is proven good; the GPS module is not transmitting.**
Note the antenna is not the variable here — a powered GPS emits NMEA continuously
with or without an antenna (with empty fields until it locks). Zero bytes means
the module is not sending anything to the Pi.

Remaining physical candidates, most likely first: GPS TX not crossed to Pi RX
(GPIO 15, header pin 10); module VCC not actually at 3.3 V; no common ground
between module and Pi; module wired to non-UART pins; faulty module or wire.

### Step 4 — Localization / navigation on real data: BLOCKED behind step 3

### Step 5 — Soak: PASS (5 minutes, camera + perception + agent loop + active mission)

| Measure | Result | Verdict |
|---|---|---|
| Memory | RSS 99.2 → 99.4 MB (+0.2 MB) | PASS |
| Threads | 5 → 5, dropping to 1 after shutdown | PASS |
| Perception status | `OK` for the entire run | PASS |
| Inference latency drift | 3.58 – 7.05 ms, no upward trend | PASS |
| Accumulated errors | none | PASS |
| Thermal | 41.4 → 43.0 °C (+1.6) | PASS |

Scope limit: run with GPS disabled, so the decision layer stayed in its
`STOP — no GPS position` branch throughout. **Route-following was never
exercised**, and 5 minutes catches fast leaks and startup transients only — it
does not substitute for a multi-hour endurance run.

## Not implemented

| Item | Note |
|---|---|
| **ESP32 link** | No UART code, no protocol, no framing. Out of scope for this stage. `MotionIntent` is the seam. |
| **Backend integration** | Socket.IO client exists (with R-02 handshake token) but is not constructed by the agent and is off by default. Telemetry is local only. |
| **Motor actuation from the agent** | Deliberate. The agent imports no motor driver. |
| **Battery sensing** | No fuel-gauge IC, no ADC, no voltage divider on this robot. Telemetry reports `UNAVAILABLE`/`null` rather than a number. |
| **YOLO detection** | `ultralytics` is not installed and needs a torch build this Pi does not have. |
| **Distance / depth estimation** | Monocular camera. Not possible without hardware or calibration this robot lacks; deliberately absent rather than faked. |
| **Hardware E-stop** | ESP32 concern. |
| **Comm-loss watchdog** | Nothing to lose contact with yet. |
| **Command authentication on the HTTP API** | Endpoints are unauthenticated; local trusted network only. |
| **Process supervision (systemd)** | No unit file. Manual start. (Remediation R-08, still open.) |
| **Geofencing** | Any lat/lon can be requested as a waypoint. |

## Hardware-dependent, not tested

Physical validation still outstanding, and which must not be assumed from the
results above:

- **No GPS fix acquired** — no receiver transmitting (see step 3). Blocks all of
  step 4.
- **No camera validation in usable light** — every optical measurement so far was
  taken at 0.46 lux. Aiming, focus and colour checks all still pending.
- **No detection of a real object at a real distance** — step 2 used synthetic
  scenes. The ~90 px blind spot has not been converted to centimetres.
- **No motor was driven; no wheel turned.**
- No obstacle-avoidance behaviour observed on a moving robot.
- No ultrasonic or IR sensor reading was taken.
- No outdoor navigation run.
- No multi-hour endurance run (longest was the 5-minute soak).

See `TEST_README.md` for the manual checklist covering these.

## Retained but not production

| Code | Why kept |
|---|---|
| `robotx/control/robot_controller.py` | The previous direct-drive loop. Carries reviewed safety rules (R-01 MANUAL gate, R-03 ultrasonic status) that are the reference for the ESP32. Usable for bench-testing the drivetrain. Not started by the application. |
| `robotx/perception/experimental/` | An older, richer vision pipeline with bench-tuned hysteresis and an area-trend heuristic. Not promoted: its thresholds are hard-coded pixel literals tied to one camera mounting, and it has a different safety posture. See its README. |
| `robotx/communication/socket_client.py` | Client half of a future backend link, including R-02's handshake token. |
| `robotx/hardware/{motors,encoders,ultrasonic,ir}.py` | ESP32-owned in the target architecture; still needed by the bench scripts. |

## Known limitations worth stating plainly

1. **The Pi cannot stop the robot.** It has no motor authority. Its guarantee
   is only that it will not *ask* for forward motion unless it positively knows
   the path is clear. Actually stopping is the ESP32's job.
2. **No heading while stationary.** The robot cannot know which way it faces
   until it moves far enough for GPS to show a track.
3. **No distance measurement.** Obstacle proximity is inferred from bounding-box
   area in pixels — a coarse, uncalibrated proxy, not metres.
4. **Detection quality is modest.** The OpenCV backends find motion and strong
   edges. They are not a reliable person detector; `person` requires YOLO.
5. **Perception thresholds are uncalibrated** for this robot's camera height and
   angle. They are configurable, and should be tuned on the bench.

## Next steps

1. Resolve the GPS receiver: confirm wiring, power, and whether it is on
   `/dev/ttyAMA0` or `/dev/serial0`, then verify a real fix outdoors.
2. Calibrate perception thresholds against the real camera mounting.
3. Define the Pi↔ESP32 UART protocol from `MotionIntent` (separate task).
4. Add process supervision (systemd unit) — remediation R-08.
5. Consider authenticating the local HTTP API if the Pi will sit on an
   untrusted network.
