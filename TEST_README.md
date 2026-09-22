# Testing RobotX on the Pi

Two kinds of test live here, and they are not interchangeable.

| | Automated (unit) | Automated (integration) | Manual |
|---|---|---|---|
| Location | `tests/unit/` | `tests/integration/` | `tests/hardware/`, `tests/control/`, `tests/perception/` |
| Touches hardware | No | No | Yes — real camera, serial port, GPIO, motors |
| Touches the network | No | Loopback only | No |
| Safe to run any time | Yes | Yes | **No** |
| Asserts anything | Yes | Yes | No — a human watches the output |

---

## Automated tests

Stdlib `unittest`. No pytest, no network, no hardware, no new dependencies.

```bash
cd /home/pi/Desktop/RobotX
venv/bin/python -m unittest discover -s tests/unit -t .
```

Verbose, or a single file:

```bash
venv/bin/python -m unittest discover -s tests/unit -t . -v
venv/bin/python -m unittest tests.unit.test_motion_and_decision -v
```

| File | Covers |
|---|---|
| `test_config.py` | Defaults, env overrides, type coercion, malformed values, secret elision |
| `test_perception.py` | Detection model, detector filtering, pipeline statuses and error paths |
| `test_gps_and_position.py` | NMEA parsing, invalid-fix rejection, staleness, heading estimation |
| `test_navigation.py` | Waypoint advance, arrival, rerouting, desired heading, heading error |
| `test_motion_and_decision.py` | Motion intent construction and clamping, every decision branch |
| `test_state_telemetry_health.py` | Authoritative state, telemetry schema, no fake battery, health aggregation |
| `test_agent.py` | Full agent tick with fake subsystems, lifecycle, mission control, failure recovery, mid-run degradation |
| `test_navigation_synthetic.py` | Full localization → navigation → decision chain on deterministic NMEA fixtures |
| `test_audit_regressions.py` | One test per defect found in the production-readiness audit |
| `test_protocol.py` | Backend payloads, refusal to fabricate battery/position, inbound validation, credential redaction |
| `test_commands.py` | Command execution, refusals, idempotency, the communication/hardware boundary |
| `test_backend_link.py` | Link lifecycle, backoff, rate limiting, dispatch, honest status reporting |
| `test_agent_backend_commands.py` | The real agent under STOP/PAUSE/RETURN/RESUME, including safety non-bypass |

These deliberately do **not** import any hardware library. Do not add a test
here that needs a camera, a serial port, or GPIO.

---

## Integration tests — real Socket.IO transport

```bash
venv/bin/python -m unittest discover -s tests/integration -t .
```

These bind a loopback TCP port and run a real `socketio.AsyncServer` against
the real `socketio.AsyncClient` the agent uses: genuine Engine.IO handshake,
genuine JSON, nothing faked on the client side. They cover the handshake and
credential delivery, a server that refuses the token, telemetry and status,
the command round-trip, duplicates, disconnect, reconnect, re-registration,
and a backend that is absent at boot and appears later.

Still hardware-free and safe to run any time; they take ~12 s.

**What they do not prove:** anything about the FalconAut backend. The test
server speaks the Pi's own PROVISIONAL event names, because the real ones are
not available in this repository. A green run means the transport and the
state machine are correct, not that the integration is done.

### Measurement harness (manual, uses the real camera)

```bash
PYTHONPATH=. venv/bin/python tests/integration/soak_backend_link.py --seconds 120
```

Runs the full agent twice — with and without the backend link — and prints the
CPU, loop-timing, perception and memory difference. Takes a few minutes. No
motors are touched; none exist in the process.

---

## Manual hardware verification

None of these are automated, and none should ever be run by a script, CI job,
or as refactor validation. Each needs a human watching the robot.

Run one at a time:

```bash
venv/bin/python tests/hardware/test_camera.py
venv/bin/python tests/hardware/test_gps.py
venv/bin/python tests/hardware/test_ultrasonic.py
venv/bin/python tests/hardware/test_motors.py        # SAFETY: wheels off the ground
venv/bin/python tests/control/test_controller.py     # SAFETY: wheels off the ground
venv/bin/python tests/perception/test_vision.py
```

### Checklist

Record the date and the result. An unrun check is not a pass.

#### 1. Camera — `tests/hardware/test_camera.py`

- [ ] Camera opens without error
- [ ] Frames arrive within ~5 s (shape printed, e.g. `(480, 640, 3)`)
- [ ] `frame.jpg` is written and shows the expected scene (headless), or the
      preview window opens (with a display)
- [ ] Detections print when something moves in view
- [ ] Ctrl+C exits cleanly and releases the camera

If it fails: `rpicam-hello --list-cameras`, check the ribbon cable and CSI
connector, confirm `python3-picamera2` is installed and the venv was created
with `--system-site-packages`.

#### 2. GPS — `tests/hardware/test_gps.py`

- [ ] Serial port opens (`gps.connected` is logged)
- [ ] NMEA sentences are received (`sentences` climbs above 0)
- [ ] Status reaches `FIX` outdoors with a clear sky view
- [ ] Latitude/longitude are plausible for your location
- [ ] Satellite count and altitude are reported
- [ ] Unplugging the receiver produces a status change, not a silent hang

A cold start outdoors can take several minutes. `sentences` staying at 0 means
the receiver is not transmitting on that port — check power, TX/RX wiring, baud
rate, and whether the module is on `/dev/serial0` instead of `/dev/ttyAMA0`.

#### 3. Perception on live camera — `tests/perception/test_vision.py`

Exercises the **experimental** pipeline (`robotx/perception/experimental/`),
not the production one. The production path is covered by the camera check
above.

- [ ] Actions change sensibly as an obstacle moves through the frame
- [ ] Frame rate is usable on this Pi

#### 4. Ultrasonic — `tests/hardware/test_ultrasonic.py`

ESP32-owned in the target architecture; this exercises the Pi-side driver.

- [ ] Distance tracks an object moved toward and away from the sensor
- [ ] Covering the sensor produces a non-VALID status, not a stale number

#### 5. Motors — `tests/hardware/test_motors.py` — **WHEELS OFF THE GROUND**

- [ ] Both wheels spin forward, then backward
- [ ] Left and right turn the expected way (invert settings if not)
- [ ] Motors stop on Ctrl+C and on exit

#### 6. Legacy direct-drive loop — `tests/control/test_controller.py` — **WHEELS OFF THE GROUND**

This does **not** exercise the production agent. It is an independent
stop/forward rule over real sensors, kept for drivetrain bench work.

- [ ] Blocking the ultrasonic sensor stops the motors
- [ ] Clearing it resumes forward drive
- [ ] Motors stop on exit

#### 7. Full agent on real hardware

```bash
venv/bin/python -m uvicorn robotx.application.main:app --host 0.0.0.0 --port 8000
```

- [ ] Startup logs show camera connected, perception started, GPS starting
- [ ] `curl localhost:8000/health` — camera and perception `HEALTHY`
- [ ] `curl localhost:8000/state` — perception status `OK`, detections update
- [ ] `http://<pi-ip>:8000/camera` shows live video in a browser
- [ ] `curl localhost:8000/telemetry` — battery is `UNAVAILABLE`/`null`
- [ ] With no GPS fix, starting a mission yields intent `STOP — no GPS position`
- [ ] Ctrl+C shuts down cleanly: perception stopped, GPS stopped, camera released
- [ ] No wheel moves at any point (the agent has no motor authority)

---

## What has actually been verified

`ROBOTX_PI_CURRENT_STATE.md` records which of these were run and what happened.
Checks that were not performed are listed there as not performed. Do not
record a check as passing unless you watched it pass.
