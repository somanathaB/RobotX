# RobotX — Raspberry Pi robot agent

The high-level brain of an autonomous delivery rover, running on a Raspberry Pi 5.

The Pi senses and decides. Motor authority and the safety reflexes belong to an
ESP32: the Pi publishes a **motion intent**, and the safety-gated result goes to
the ESP32 over a UART (`robotx/esp32/`) as STOP/DRIVE commands — only when
`ROBOTX_ESP32_MOTION_ENABLED=1`, and never otherwise. With the ESP32 link and
the backend link both off (the defaults) the agent runs and can be validated
entirely on its own.

```
Camera ─► Perception ─┐
                      ├─► Decision ─► Safety gate ─► ESP32 (UART) ─► motors
GPS ─► Position ─► Navigation ─┘            │
                                            ▼
                                   Robot State ─► Telemetry + Health
                                            │
                                            ▼
                             Socket.IO ─► RobotX backend (OFFERs in, reports out)
```

**Deploying to the robot:** follow
[docs/operations/V1_PI_DEPLOYMENT.md](docs/operations/V1_PI_DEPLOYMENT.md)
(systemd unit in [deployment/](deployment/robotx-agent.service)); operating it:
[docs/operations/V1_PI_OPERATOR_RUNBOOK.md](docs/operations/V1_PI_OPERATOR_RUNBOOK.md).
The setup below is the bench / development path.

## What it does today

- Captures frames from the Pi Camera Module 3 (Picamera2/libcamera)
- Detects objects (OpenCV motion + edge backends; optional YOLOv8)
- Reads position either from the ESP32's GPS frames (`ROBOTX_GPS_SOURCE=esp32`,
  the V1 rover's wiring) or from an NMEA receiver on the Pi's own serial port
  (the bench default), with explicit fix validity and staleness
- Estimates position and a GPS-derived heading
- Follows a waypoint route and computes the desired heading
- Decides what motion it wants, failing safe on anything it does not know
- Sends the gated motion to the ESP32 over the UART link, when motion is enabled
- Aggregates one authoritative robot state, telemetry, and health
- Serves a small local HTTP API for inspection, custody confirmation and safety
- Optionally links to the RobotX backend over Socket.IO. **Off by default.** The
  connection is anonymous; the robot authenticates with an `AUTH` event (a
  one-time pairing code, then a persisted session token). It receives signed,
  fenced OFFERs from the assignment engine, answers each exactly once, reports
  custody and `TASK_COMPLETE`, and applies `STOP`/`PAUSE`/`RETURN`/`RESUME`
  commands with acknowledgement. A lost link pauses an active mission within the
  bound the lease requires (Y4). Verified end to end against the real backend
  with the ESP32 host simulator (Gate 3 / Gate 3b); not yet on the physical
  robot. See `docs/communication/ROBOT_BACKEND_PROTOCOL.md`.

`ROBOTX_PI_CURRENT_STATE.md` is a 2026-09-22 snapshot (hardware evidence for the
camera and vision pipeline); the V1 deployment documents above are current.

## Setup

1. **Enable interfaces**: `sudo raspi-config` → enable Camera and Serial
   (disable the serial login console, so the ESP32 can use the GPIO 14/15 UART,
   `/dev/ttyAMA0`).

2. **Create the venv with system packages** — Picamera2 comes from apt, not pip:

   ```bash
   sudo apt install -y python3-picamera2
   cd <repository-checkout>
   python3 -m venv --system-site-packages venv
   venv/bin/python -m pip install -r requirements.txt
   ```

   Always invoke through `venv/bin/python -m <module>`, never the
   `venv/bin/pip` / `venv/bin/uvicorn` shim scripts. If this venv was ever
   copied from another path, those shims' shebangs point at a directory that no
   longer exists and fail with "bad interpreter" even after `chmod +x`.
   `venv/bin/python -m ...` only needs `venv/bin/python` itself.
   (Remediation R-00.)

3. **Configure**: `.env.example` lists every setting with its built-in default.
   The defaults are a working **bench** configuration (no backend, no ESP32, no
   motion), so nothing is required to start on the bench. A V1 robot needs the
   robot id, backend URL, signing key, ESP32, GPS source and custody settings —
   see section 12 of the deployment guide. Never leave a setting present but
   empty: `.env.example` explains why.

## Run

```bash
cd <repository-checkout>
venv/bin/python -m robotx.application
```

This serves the local API on `ROBOTX_API_HOST`:`ROBOTX_API_PORT`, by default
`127.0.0.1:8000` -- reachable from the Pi itself only. The API is
unauthenticated, so exposing it to the network is an explicit choice: set
`ROBOTX_API_HOST` to the interface to serve on (`0.0.0.0` for every interface),
on a trusted network only. V1 keeps the default and operators use SSH. With
`ROBOTX_SOCKET_ENABLED=1` RobotX admits missions: `POST /mission/start` and
`POST /mission/resume` answer 409, and a paused mission is resumed only by a
backend RESUME command, after the manual checks in the operator runbook. On the
robot the service runs this same command under systemd
([deployment/robotx-agent.service](deployment/robotx-agent.service)).

The agent starts even if the camera or GPS is missing — those subsystems report
as unavailable and the decision layer refuses to request motion. That is what
makes the Pi testable on its own.

### Endpoints

| Endpoint | Purpose |
|---|---|
| `GET /health` | Overall health, per-subsystem status, host metrics |
| `GET /state` | Full robot state |
| `GET /telemetry` | Local telemetry payload |
| `GET /config` | Effective configuration (secrets shown as SET/UNSET) |
| `GET /backend` | Backend link state, protocol binding, message counters |
| `GET /camera` | MJPEG preview |
| `POST /mission/start` | Load a waypoint route and switch to AUTO (bench mode only: 409 with `ROBOTX_SOCKET_ENABLED=1`) |
| `POST /mission/stop` | Halt and clear the route |
| `POST /mission/pause` | Suspend the mission, keeping the route |
| `POST /mission/resume` | Resume a paused mission (409 if there is nothing to resume; bench mode only: 409 with `ROBOTX_SOCKET_ENABLED=1`) |
| `POST /mission/idle` | Return to IDLE (not while a mission is active: see the operator runbook) |
| `POST /safety/estop` | Latch the software emergency stop: no motion intent leaves the Pi until cleared. Not a physical e-stop |
| `POST /safety/clear` | Release the latch (also acknowledges an ESP32 reboot); resumes nothing |
| `POST /mission/custody` | Operator custody confirmation `{"kind": "ACQUIRED"\|"RELEASED"}`, only with `ROBOTX_CUSTODY_CONFIRMATION=operator` and only at that measured stop |

```bash
curl -s http://127.0.0.1:8000/health | jq
curl -s -X POST http://127.0.0.1:8000/mission/start \
  -H 'Content-Type: application/json' \
  -d '{"waypoints":[{"lat":37.4219,"lon":-122.0840}]}'
```

Endpoints are unauthenticated. They listen on loopback by default; on a V1 robot
they are used over SSH, on the Pi itself.

## Tests

Automated, hardware-free (stdlib `unittest`, no pytest needed):

```bash
venv/bin/python -m unittest discover -s tests/unit -t .         # 924 tests
venv/bin/python -m unittest discover -s tests/integration -t .  # 55 tests
```

Counts as verified on Linux (Python 3.11 and 3.13, x86_64; Python 3.13 on
emulated ARM64). On Windows a few tests that depend on POSIX file modes or
socket timing fail; run the suites on Linux.

The integration suite runs a real Socket.IO server on a loopback port and
drives the real client against it — genuine handshake, genuine JSON, nothing
faked on the client side.

Manual hardware checks are **not** automated and never run as part of a check —
each one touches the real camera, serial port, or motors. See `TEST_README.md`.

## Configuration

Everything is read from `ROBOTX_*` environment variables in
`robotx/config/settings.py`, and nowhere else. Common ones:

| Variable | Default | Purpose |
|---|---|---|
| `ROBOTX_ROBOT_ID` | none (empty) | Must equal the commissioned robot id exactly; required with the backend link |
| `ROBOTX_LOG_LEVEL` | `INFO` | Logging verbosity |
| `ROBOTX_API_HOST` | `127.0.0.1` | Local API bind address; another interface is an explicit, network-exposing choice |
| `ROBOTX_API_PORT` | `8000` | Local API port |
| `ROBOTX_CAMERA_ENABLED` | `1` | Turn the camera off for headless testing |
| `ROBOTX_CAMERA_WIDTH/HEIGHT/FPS` | 640/480/20 | Capture settings |
| `ROBOTX_DETECTION_BACKEND` | `opencv` | `opencv`, `yolo`, or `auto` |
| `ROBOTX_DETECTION_HZ` | `2.0` | Inference rate |
| `ROBOTX_GPS_SOURCE` | `pi_serial` | `pi_serial` or `esp32` (V1: `esp32`) |
| `ROBOTX_GPS_PORT` | `/dev/ttyAMA0` | GPS serial device (`pi_serial` only) |
| `ROBOTX_GPS_BAUDRATE` | `9600` | GPS baud rate |
| `ROBOTX_ESP32_ENABLED` | `0` | Enable the ESP32 UART link (V1: `1`) |
| `ROBOTX_ESP32_PORT` | `/dev/ttyAMA0` | ESP32 UART (`/dev/serial0` is refused) |
| `ROBOTX_ESP32_MOTION_ENABLED` | `0` | Allow STOP/DRIVE to the ESP32; at `0` every OFFER is rejected |
| `ROBOTX_CUSTODY_CONFIRMATION` | `none` | `none` or `operator` (V1: `operator`) |
| `ROBOTX_AGENT_HZ` | `10.0` | Agent loop rate |
| `ROBOTX_SOCKET_ENABLED` | `0` | Enable the backend link (and the engine-mode lock on local start/resume) |
| `ROBOTX_SOCKET_SERVER_URL` | none (empty) | Backend address, e.g. `https://<backend-host>`; required with the backend link |
| `ROBOTX_COMMAND_SIGNING_KEY` | unset | The backend's command signing key (secret); without it no OFFER is admitted |
| `ROBOTX_COMMITMENT_STATE_PATH` | `~/.robotx/commitments.json` | Persisted commitment marks; never empty |
| `ROBOTX_ROBOT_TOKEN` | unset | Optional override of the persisted session token, sent in `AUTH` (secret; leave unset) |
| `ROBOTX_PROTOCOL_FILE` | unset | JSON file giving the real backend event names |
| `ROBOTX_BACKEND_LOSS_POLICY` | `pause` | `pause` on link loss (Y4: `continue` is refused while the backend link is enabled) |
| `ROBOTX_BACKEND_LOSS_GRACE_S` | `10.0` | seconds after a detected link loss before an active mission pauses (Y4: at most 10) |
| `ROBOTX_BACKEND_HEARTBEAT_INTERVAL_S` | `2.0` | seconds between HEARTBEATs, which renew the mission's lease (Y4: above 0, at most 2) |

See `.env.example` for the full list. Secrets are never given defaults and are
never logged.

## Documentation

| Document | Contents |
|---|---|
| `docs/operations/V1_PI_DEPLOYMENT.md` | V1 installation on the Pi: systemd, configuration, signing key, pairing, verification, rollback |
| `docs/operations/V1_PI_OPERATOR_RUNBOOK.md` | V1 operation: RESUME, STOP, PAUSE, custody, e-stop, stranded goods, out of service |
| `docs/architecture/ROBOTX_PI_ARCHITECTURE.md` | Subsystem boundaries, data flow, ESP32/backend seams |
| `docs/communication/ROBOT_BACKEND_PROTOCOL.md` | The robot ↔ backend wire contract, and what the backend must implement |
| `docs/communication/SOCKET_IO_ARCHITECTURE.md` | How the Pi's backend boundary is built, and its measured cost |
| `docs/architecture/DEPENDENCY_MAP.md` | File-level import graph |
| `ROBOTX_PI_CURRENT_STATE.md` | Historical snapshot (2026-09-22): hardware evidence for the camera and vision pipeline |
| `ROBOTX_PI_FOUNDATION_IMPLEMENTATION_REPORT.md` | The standalone-agent restructuring |
| `ROBOTX_PI_PRODUCTION_READINESS_AUDIT_REPORT.md` | Production-readiness audit: defects found, fixed and tested |
| `ROBOTX_PI_REMEDIATION_PLAN.md` | Safety/security findings R-00 … R-10 |
| `ROBOTX_PI_BACKEND_INTEGRATION_AUDIT_REPORT.md` | The backend integration: audit, design, tests, blockers |
| `TEST_README.md` | Manual hardware verification procedures |
| `robotx/perception/experimental/README.md` | The retained non-production vision pipeline |

## Safety

- Motor authority is the ESP32's. The Pi sends it STOP/DRIVE commands only when
  `ROBOTX_ESP32_MOTION_ENABLED=1`, and only what the safety gate let through; the
  ESP32 stops the motors by itself when commands stop arriving (2 s watchdog).
  The Pi's guarantee is that it will not *request* forward motion unless it
  positively knows the path is clear.
- Any unknown — stale perception, no camera frame, no GPS fix, a failed tick —
  produces a STOP intent.
- `POST /safety/estop` is a software latch, not a physical emergency stop. The
  robot has no physical e-stop yet; see the operator runbook.
- `tests/hardware/test_motors.py` and `tests/control/test_controller.py` drive
  real motors. **Wheels off the ground.**
