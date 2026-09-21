
# RobotX (Raspberry Pi autonomous delivery robot)

This repo contains a complete, modular robot stack that runs **on the Raspberry Pi**:

- FastAPI service (health + MJPEG camera streaming)
- Real-time Socket.IO client (telemetry + instant commands)
- Hardware drivers (motors, encoders, ultrasonic, IR)
- Navigation (GPS + Google Directions route fetching + reroute logic)
- Perception (OpenCV camera + OpenCV detector; optional YOLOv8 backend)
- Central controller loop (safety + obstacle avoidance + route following)

## Folder structure

```
robotx/
	app/
		main.py         # FastAPI entry point
		sockets.py      # Socket.IO client
	hardware/
		motors.py
		encoders.py
		ultrasonic.py
		ir.py
	navigation/
		gps.py
		maps.py
		planner.py
	perception/
		camera.py
		detection.py
	control/
		controller.py
	utils/
		config.py
requirements.txt
```

## Raspberry Pi setup

1) Enable interfaces (recommended)

- `sudo raspi-config`
- Enable **Camera** (if using CSI camera)
- Enable **Serial** (for GPS) and disable serial console

2) Wiring (BCM pins)

Defaults are in `robotx/utils/config.py` and can be overridden by env vars.

- L298N left: `IN1=5`, `IN2=6`, `ENA(PWM)=12`
- L298N right: `IN3=13`, `IN4=19`, `ENB(PWM)=18`
- Encoders: left `23`, right `24`
- Ultrasonic: trigger `20`, echo `21`
- IR: left `16`, center `25`, right `26`

3) Install Python deps

```bash
cd /home/pi/Desktop/RobotX
python3 -m venv venv
venv/bin/python -m pip install -r requirements.txt
```

Note: use `venv/bin/python -m pip ...` / `venv/bin/python -m uvicorn ...` (not the
`venv/bin/pip` / `venv/bin/uvicorn` shim scripts directly). If this venv was ever
copied or moved from another path, those shim scripts' shebang lines still point at
the original location and will fail with "Permission denied" / "bad interpreter"
even after `chmod +x`. Invoking via `venv/bin/python -m <module>` only depends on
`venv/bin/python` itself being executable, so it is unaffected by that. See
ROBOTX_PI_REMEDIATION_PLAN.md (R-00) for how this was diagnosed on this Pi.

4) GPIO access

Run as `root` (or ensure GPIO permissions are set correctly) for `RPi.GPIO`.

## Configuration

Set environment variables to match your hardware + server:

- `ROBOTX_SOCKET_SERVER_URL` (e.g. `http://YOUR_SERVER:3000`)
- `ROBOTX_SOCKET_NAMESPACE` (default: `/robot`)
- `ROBOTX_GOOGLE_MAPS_API_KEY` (for route fetching)
- Motor pins: `ROBOTX_MOTOR_LEFT_IN1`, `...` etc.
- GPS: `ROBOTX_GPS_PORT` (default `/dev/ttyAMA0`), `ROBOTX_GPS_BAUDRATE`

Detection backend:

- `ROBOTX_DETECTION_BACKEND=opencv` (default, lightweight)
- `ROBOTX_DETECTION_BACKEND=yolo` (requires installing `ultralytics` + a working `torch` build)

## Run

Start the robot stack (FastAPI + controller + Socket.IO client):

```bash
cd /home/pi/Desktop/RobotX
venv/bin/python -m uvicorn robotx.app.main:app --host 0.0.0.0 --port 8000
```

Endpoints:

- `GET /health`
- `GET /camera` (MJPEG stream)

## Socket.IO protocol (expected)

Namespace: `ROBOTX_SOCKET_NAMESPACE` (default `/robot`)

Robot emits:

- `telemetry` (every ~1–2s)
- `status` (optional)
- `robot_hello` (on connect)

Robot receives:

- `command` payload: `{ "type": "START"|"STOP"|"RETURN"|"MANUAL", "payload": { ... } }`

Examples:

- START with destination:

```json
{ "type": "START", "payload": { "destination": { "lat": 37.4219999, "lon": -122.0840575 } } }
```

- MANUAL:

```json
{ "type": "MANUAL", "payload": { "action": "forward", "speed": 0.4 } }
```

## Quick tests

1) Import check

```bash
cd /home/pi/Desktop/RobotX
venv/bin/python -c "import robotx.app.main; print('ok')"
```

2) Health check (after running uvicorn)

```bash
curl -s http://localhost:8000/health | jq
```

3) Camera stream

Open: `http://<pi-ip>:8000/camera` in a browser.

## Safety notes

- The controller fails safe: any unhandled error triggers motor stop.
- Always test with wheels off the ground first.

