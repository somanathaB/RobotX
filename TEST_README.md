# RobotX subsystem tests (no frontend required)

This document describes standalone test scripts you can run directly on the Raspberry Pi to validate each subsystem independently.

**Frontend is NOT required** for these tests.

---

## 1) Setup

From the project root:

```bash
cd /home/pi/Desktop/RobotX
```

### Create/activate venv

```bash
python3 -m venv venv
source venv/bin/activate
```

### Install dependencies

```bash
pip install -r requirements.txt
```

### (Alternative) Install dependencies with `uv`

If `pip install` is slow or fails on the Pi, you can use `uv`.

Install `uv` (once):

```bash
pip install -U uv
```

Then install requirements:

```bash
uv pip install -r requirements.txt
```

Notes:
- Hardware access (GPIO) may require running as `sudo` depending on your OS permissions.
- If you run as `sudo`, prefer: `sudo -E env "PATH=$PATH" python ...` so your venv is used.

---

## 2) Run each test

All tests are standalone and can be run from the repo root.

### Test Camera (Picamera2 / libcamera)

```bash
python tests/hardware/test_camera.py
```

Expected:
- A live camera window opens.
- Press **q** to quit.
- If the detection module is available, detections may be printed to the terminal.

Notes:
- Raspberry Pi Camera Module 3 uses **libcamera/Picamera2**.
- Do **NOT** use `cv2.VideoCapture(0)` for the CSI camera.
- If you are using a Python venv, Picamera2 (apt-installed) may not be visible unless you create the venv with `--system-site-packages`.

---

### Test GPS

```bash
python tests/hardware/test_gps.py
```

Expected:
- Latitude/longitude printed every ~2 seconds once a fix is available.
- If no fix/device, you’ll see “No GPS fix yet” plus error info.

Config:
- `ROBOTX_GPS_PORT` (default `/dev/ttyAMA0`)
- `ROBOTX_GPS_BAUDRATE` (default `9600`)

---

### Test Motors

```bash
python tests/hardware/test_motors.py
```

Expected:
- Robot moves forward for ~2 seconds, stops, then moves backward for ~2 seconds, then stops.

Safety:
- Keep wheels off the ground for the first run.
- Script always attempts to STOP motors on error or Ctrl+C.

---

### Test Ultrasonic

```bash
python tests/hardware/test_ultrasonic.py
```

Expected:
- Distance values printed continuously.
- Moving an object in front of the sensor should change readings.

---

### Test Controller (simplified)

```bash
python tests/control/test_controller.py
```

Expected:
- Prints sensor readings and a decision (`FORWARD` / `STOP`).
- Logic is intentionally simple:
  - If ultrasonic distance < threshold OR any IR sensor is triggered => STOP
  - Else => FORWARD

This test does NOT require:
- Socket.IO server
- Google Maps API
- Full navigation routing

---

### Test Vision Decision System (camera + YOLOv8)

```bash
python tests/perception/test_vision.py
```

Expected:
- Runs headless (no GUI window required)
- Prints in real-time:
  - `Objects: [...]`
  - `Stable: [...]`
  - `Action: STOP/SLOW/MOVE_FORWARD`
  - plus backend and approximate detection FPS
- By default, writes a debug frame to `./frame.jpg` every ~2 seconds
  - Disable: `ROBOTX_SAVE_DEBUG=0 python tests/perception/test_vision.py`
  - Change path: `ROBOTX_DEBUG_PATH=/tmp/frame.jpg python tests/perception/test_vision.py`

Notes:
- The vision pipeline uses detector backend `auto`:
  - If `ultralytics` + `torch` are installed, it will use YOLOv8.
  - Otherwise it will fall back to a lightweight OpenCV detector (still produces STOP/SLOW/MOVE_FORWARD).
- Installing `ultralytics` typically pulls in `torch` and can be very large.
- If install is slow or fails, see the Troubleshooting section below.

---

## 3) Troubleshooting

### Camera not detected

Symptoms:
- `ERROR: Could not start Picamera2 camera` or `No cameras available!`

Fixes:

- Install Picamera2 (libcamera):

  ```bash
  sudo apt update
  sudo apt install -y python3-picamera2
  ```

- Test camera at OS level:

  ```bash
  rpicam-hello --list-cameras
  rpicam-hello -t 0
  ```

  On some images/versions the command is `libcamera-hello`:

  ```bash
  libcamera-hello --list-cameras
  libcamera-hello -t 0
  ```

- If you are using a venv, create it with system packages so it can import Picamera2:

  ```bash
  python3 -m venv --system-site-packages venv_cam
  source venv_cam/bin/activate
  pip install -r requirements.txt
  python tests/hardware/test_camera.py
  ```

- If running headless (no GUI): `cv2.imshow` may fail. Use an attached display or X forwarding.

## Raspberry Pi Camera Module 3 fix

- Raspberry Pi Camera Module 3 uses libcamera. The RobotX camera module uses Picamera2 under the hood.
- `cv2.VideoCapture()` is not used and `/dev/video0` is not required.

### YOLOv8 / ultralytics install issues

Symptoms:
- `RuntimeError: ultralytics is not installed`
- `pip install ultralytics` fails due to missing `torch` wheels

Fixes:
- First try: `pip install -r requirements.txt`
- If you try installing YOLO, avoid using `/tmp` (often a small tmpfs on Raspberry Pi):

  ```bash
  mkdir -p /home/pi/pip-tmp
  TMPDIR=/home/pi/pip-tmp pip install --no-cache-dir ultralytics
  ```

- Ensure you’re using piwheels when available: `pip config get global.index-url` and `pip -v install ultralytics`
- If `torch` wheels are not available for your OS/Python, consider:
  - Using Raspberry Pi OS 64-bit with a Python version that has torch wheels available
  - Installing torch via a known wheel source for Pi (varies by distro/version)

### Headless display (no HDMI)

Symptoms:
- `cv2.imshow failed` or no window appears

Fixes:
- Connect a display to the Pi
- Or use X forwarding / VNC
- If you only need console output, you can still run the controller logic without `imshow` by adapting `tests/perception/test_vision.py` (ask if you want a headless mode)

### Permission issues (GPIO)

Symptoms:
- Motor/sensors do nothing or you see permission errors.

Fixes:
- Run as root: `sudo -E env "PATH=$PATH" python tests/hardware/test_motors.py`
- Ensure `RPi.GPIO` is installed: `pip show RPi.GPIO`
- Confirm pin numbering is BCM (these scripts use BCM pins).

### GPS not locking

Symptoms:
- “No GPS fix yet” forever.

Fixes:
- Confirm serial port: `/dev/ttyAMA0` vs `/dev/ttyS0`
- Ensure serial is enabled and serial console disabled (`raspi-config`)
- Check wiring (TX/RX swapped is common)
- Verify module has sky view; initial lock can take time

### Motors not responding

Fixes:
- Verify L298N ENA/ENB jumpers and wiring to PWM pins
- Check battery voltage / motor power supply
- Confirm pin mapping matches your wiring (override with env vars; see `robotx/config/settings.py`)

---

## 4) Notes

- These tests are designed to debug the robot **without any frontend/dashboard**.
- If/when you run the full FastAPI app later, Swagger UI is available at:
  - `http://<pi-ip>:8000/docs`
