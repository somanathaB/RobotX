# RobotX — Raspberry Pi robot agent

The high-level brain of an autonomous delivery rover, running on a Raspberry Pi 5.

The Pi senses and decides. It does **not** drive motors: in the target
architecture an ESP32 owns motor authority and safety reflexes, and the Pi's
job ends at publishing a **motion intent**. That link does not exist yet, and
nothing here depends on it — the agent runs and can be validated entirely on
its own.

```
Camera ─► Perception ─┐
                      ├─► Decision ─► Motion Intent ─► (future: ESP32)
GPS ─► Position ─► Navigation ─┘            │
                                            ▼
                                   Robot State ─► Telemetry + Health
                                            │
                                            ▼
                             Socket.IO ─► FalconAut backend (optional)
```

## What it does today

- Captures frames from the Pi Camera Module 3 (Picamera2/libcamera)
- Detects objects (OpenCV motion + edge backends; optional YOLOv8)
- Reads NMEA GPS over serial, with explicit fix validity and staleness
- Estimates position and a GPS-derived heading
- Follows a waypoint route and computes the desired heading
- Decides what motion it wants, failing safe on anything it does not know
- Aggregates one authoritative robot state, telemetry, and health
- Serves a small local HTTP API for inspection and mission control
- Optionally links to the FalconAut backend over Socket.IO: a credential in the
  connect handshake, rate-controlled telemetry, and `STOP`/`PAUSE`/`RETURN`/
  `RESUME` commands with acknowledgement. **Off by default.** Two caveats that
  matter: the real backend event names are still unverified, and the channel is
  not authenticated until a backend actually validates the credential. See
  `docs/communication/ROBOT_BACKEND_PROTOCOL.md`.

For the honest status of each piece, read **`ROBOTX_PI_CURRENT_STATE.md`**.

## Setup

1. **Enable interfaces**: `sudo raspi-config` → enable Camera and Serial
   (disable the serial console so the GPS can use the port).

2. **Create the venv with system packages** — Picamera2 comes from apt, not pip:

   ```bash
   cd /home/pi/Desktop/RobotX
   python3 -m venv --system-site-packages venv
   venv/bin/python -m pip install -r requirements.txt
   sudo apt install -y python3-picamera2
   ```

   Always invoke through `venv/bin/python -m <module>`, never the
   `venv/bin/pip` / `venv/bin/uvicorn` shim scripts. If this venv was ever
   copied from another path, those shims' shebangs point at a directory that no
   longer exists and fail with "bad interpreter" even after `chmod +x`.
   `venv/bin/python -m ...` only needs `venv/bin/python` itself.
   (Remediation R-00.)

3. **Configure**: copy `.env.example` and set what differs on your robot. Every
   setting has a working default; nothing is required to start.

## Run

```bash
cd /home/pi/Desktop/RobotX
venv/bin/python -m uvicorn robotx.application.main:app --host 0.0.0.0 --port 8000
```

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
| `POST /mission/start` | Load a waypoint route and switch to AUTO |
| `POST /mission/stop` | Halt and clear the route |
| `POST /mission/pause` | Suspend the mission, keeping the route |
| `POST /mission/resume` | Resume a paused mission (409 if there is nothing to resume) |
| `POST /mission/idle` | Return to IDLE |

```bash
curl -s localhost:8000/health | jq
curl -s -X POST localhost:8000/mission/start \
  -H 'Content-Type: application/json' \
  -d '{"waypoints":[{"lat":37.4219,"lon":-122.0840}]}'
```

Endpoints are unauthenticated — put this on a trusted local network only.

## Tests

Automated, hardware-free (stdlib `unittest`, no pytest needed):

```bash
venv/bin/python -m unittest discover -s tests/unit -t .         # 345 tests
venv/bin/python -m unittest discover -s tests/integration -t .  # 25 tests
```

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
| `ROBOTX_ROBOT_ID` | `robotx-pi` | Identity in telemetry |
| `ROBOTX_LOG_LEVEL` | `INFO` | Logging verbosity |
| `ROBOTX_CAMERA_ENABLED` | `1` | Turn the camera off for headless testing |
| `ROBOTX_CAMERA_WIDTH/HEIGHT/FPS` | 640/480/20 | Capture settings |
| `ROBOTX_DETECTION_BACKEND` | `opencv` | `opencv`, `yolo`, or `auto` |
| `ROBOTX_DETECTION_HZ` | `2.0` | Inference rate |
| `ROBOTX_GPS_PORT` | `/dev/ttyAMA0` | GPS serial device |
| `ROBOTX_GPS_BAUDRATE` | `9600` | GPS baud rate |
| `ROBOTX_AGENT_HZ` | `10.0` | Agent loop rate |
| `ROBOTX_SOCKET_ENABLED` | `0` | Enable the backend link |
| `ROBOTX_SOCKET_SERVER_URL` | `http://localhost:3000` | Backend address |
| `ROBOTX_ROBOT_TOKEN` | unset | Handshake credential (secret) |
| `ROBOTX_PROTOCOL_FILE` | unset | JSON file giving the real backend event names |
| `ROBOTX_BACKEND_LOSS_POLICY` | `pause` | `pause` or `continue` on link loss |

See `.env.example` for the full list. Secrets are never given defaults and are
never logged.

## Documentation

| Document | Contents |
|---|---|
| `docs/architecture/ROBOTX_PI_ARCHITECTURE.md` | Subsystem boundaries, data flow, ESP32/backend seams |
| `docs/communication/ROBOT_BACKEND_PROTOCOL.md` | The robot ↔ backend wire contract, and what the backend must implement |
| `docs/communication/SOCKET_IO_ARCHITECTURE.md` | How the Pi's backend boundary is built, and its measured cost |
| `docs/architecture/DEPENDENCY_MAP.md` | File-level import graph |
| `ROBOTX_PI_CURRENT_STATE.md` | What works, what half-works, what does not exist |
| `ROBOTX_PI_FOUNDATION_IMPLEMENTATION_REPORT.md` | The standalone-agent restructuring |
| `ROBOTX_PI_PRODUCTION_READINESS_AUDIT_REPORT.md` | Production-readiness audit: defects found, fixed and tested |
| `ROBOTX_PI_REMEDIATION_PLAN.md` | Safety/security findings R-00 … R-10 |
| `ROBOTX_PI_BACKEND_INTEGRATION_AUDIT_REPORT.md` | The backend integration: audit, design, tests, blockers |
| `TEST_README.md` | Manual hardware verification procedures |
| `robotx/perception/experimental/README.md` | The retained non-production vision pipeline |

## Safety

- The agent publishes intent; it cannot itself stop the robot, because it has
  no motor authority. Its guarantee is that it will not *request* forward
  motion unless it positively knows the path is clear.
- Any unknown — stale perception, no camera frame, no GPS fix, a failed tick —
  produces a STOP intent.
- `tests/hardware/test_motors.py` and `tests/control/test_controller.py` drive
  real motors. **Wheels off the ground.**
