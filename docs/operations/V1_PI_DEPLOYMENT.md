# RobotX Pi — V1 deployment

How to install the RobotX Pi agent on a Raspberry Pi 5 as a systemd service,
pair it with the RobotX backend, and bring it from "installed" to "allowed to
drive". Day-to-day procedures (RESUME, STOP, custody, e-stop, stranded goods)
are in [V1_PI_OPERATOR_RUNBOOK.md](V1_PI_OPERATOR_RUNBOOK.md).

Each step is marked with what backs it:

- **[SW-VERIFIED]** — checked against this repository's code, tests, or a Linux
  container. Not the same as having run on the robot.
- **[HW-UNVERIFIED]** — depends on the physical Pi, ESP32, wiring, GPS, camera,
  network or site, and has **not** been run on the physical robot yet.

**This document does not claim the robot is ready.** No step below has been run
on the physical Pi. Treat the first installation as a commissioning exercise:
stop at the first check that does not give the expected result.

---

## 1. Scope

In scope: the Pi agent (`python -m robotx.application`), its configuration,
pairing with the backend, the systemd unit, logging, time synchronisation, the
first OFFER, enabling motion, rollback and taking the robot out of service.

Out of scope: the backend deployment (only the two backend facts the Pi depends
on are stated: the signing key and the link timing), the ESP32 firmware, and
the dashboard.

## 2. Release baseline

| What | Value |
|---|---|
| Branch | `gate3-pi-proposal` on `https://github.com/somanathaB/RobotX.git` |
| Verified software candidate | `130e3f6` — application code of `099e386` (Gate 3b navigation) plus verified dependency pins |
| Release commit | the commit that contains this document (`<release-commit>` below) |

Deploy `<release-commit>`. It must differ from `130e3f6` only in deployment and
documentation files. Check that before installing (empty output = correct):

```bash
git diff --stat 130e3f6 <release-commit> -- robotx requirements.txt
```

What each older commit can and cannot be used for is in section 24.

## 3. Supported Python versions

[SW-VERIFIED] On `130e3f6`: 22/22 targeted, 924/924 unit and 55/55 integration
tests on

- Python 3.11, Linux x86_64;
- Python 3.13, Linux x86_64;
- Python 3.13, Linux ARM64 (emulated, see section 4).

Use the OS's own Python 3 (3.11 or 3.13). Python 3.10 has only been
syntax-checked. The Pi previously recorded Python 3.13.5.

## 4. ARM64 verification status

The ARM64 result above was produced in **Docker on an x86_64 laptop, emulating
aarch64 (QEMU)**. It proves the dependencies install and the test suites pass
on ARM64 Linux. It does **not** prove anything about the Pi 5's camera stack,
UART, GPIO, CPU load or temperature. [HW-UNVERIFIED] for the physical Pi.

## 5. OS assumptions

[HW-UNVERIFIED] Raspberry Pi 5, 64-bit Raspberry Pi OS (Debian-based, systemd,
`apt`, `raspi-config`, camera stack from `apt`). The exact OS release on the
robot has not been confirmed. Record it during installation:

```bash
cat /etc/os-release
python3 --version
uname -m        # expect aarch64
```

## 6. apt packages

```bash
sudo apt update
sudo apt install -y git python3-venv python3-picamera2 jq
```

- `python3-picamera2` — the camera library. It is **not** installable with pip;
  the virtual environment sees it through `--system-site-packages`.
- `jq` — used by every check in this document and the operator runbook.
- `requirements.txt` still lists `RPi.GPIO==0.7.1` (bench scripts only; the
  agent never imports it). On ARM64 it is built from source, which needs a C
  compiler and the Python headers (`gcc`, `python3-dev`) if the image does not
  already have them. [SW-VERIFIED] in containers only.
- `rpicam-hello` (used in section 7) comes from Raspberry Pi OS's camera apps;
  install them if the image lacks them. [HW-UNVERIFIED]

## 7. Serial and camera configuration

[HW-UNVERIFIED] The ESP32 is wired to the Pi's GPIO 14/15 UART. On the Pi 5 that
UART is `/dev/ttyAMA0` when `dtparam=uart0=on` is set in
`/boot/firmware/config.txt`; `/dev/serial0` points at the debug connector
(`ttyAMA10`) and is refused by the agent. (Evidence: `ROBOTX_PI_CURRENT_STATE.md`,
2026-09-22 probe on this Pi.)

1. `sudo raspi-config` → Interface Options → Serial Port: login shell over
   serial **No**, serial port hardware **Yes**. Reboot.
2. Confirm:

   ```bash
   ls -l /dev/ttyAMA0 /dev/serial0
   systemctl is-active serial-getty@ttyAMA0.service   # expect: inactive
   rpicam-hello --list-cameras                       # expect the camera listed
   ls -l /dev/ttyAMA0 /dev/video* /dev/media* /dev/dma_heap/* 2>/dev/null
   ```

   The service user gets the groups `dialout` and `video` (section 8). Note the
   group that owns each device above; if any is owned by a group other than
   these two, stop: the unit's `SupplementaryGroups=` would need to change.

## 8. The `robotx` system user

A dedicated account, with no login shell, whose home is the state directory:

```bash
sudo useradd --system --home-dir /var/lib/robotx --shell /usr/sbin/nologin --user-group robotx
sudo usermod -aG dialout,video robotx
id robotx        # expect groups robotx, dialout, video
```

The `gpio` group is not needed: the agent drives no GPIO; motor authority is the
ESP32's. [SW-VERIFIED] (no GPIO import in the agent); account creation was
checked in a Debian container.

Every account that can log in to the Pi can reach the loopback API (see the
runbook, "Access"). Keep the Pi's accounts to the operators.

## 9. Checkout

```bash
sudo install -d -m 0755 /opt/robotx
sudo git clone --branch gate3-pi-proposal https://github.com/somanathaB/RobotX.git /opt/robotx/RobotX-Pi
sudo git -C /opt/robotx/RobotX-Pi checkout --detach <release-commit>
sudo git -C /opt/robotx/RobotX-Pi rev-parse HEAD          # must print <release-commit>
sudo git -C /opt/robotx/RobotX-Pi status --porcelain      # must print nothing
sudo git -C /opt/robotx/RobotX-Pi diff --stat 130e3f6 HEAD -- robotx requirements.txt   # must print nothing
```

The tree stays owned by root and is read-only to the service
(`ProtectSystem=strict`). Do not copy a virtual environment from another
machine or path (remediation R-00): always create it in place, as below.

## 10. Virtual environment

```bash
sudo python3 -m venv --system-site-packages /opt/robotx/RobotX-Pi/venv
```

`--system-site-packages` is required so `picamera2` from `apt` is importable.
Always run Python as `venv/bin/python -m <module>`, never through the `venv/bin/*`
shim scripts.

## 11. Dependency installation

```bash
sudo /opt/robotx/RobotX-Pi/venv/bin/python -m pip install -r /opt/robotx/RobotX-Pi/requirements.txt
sudo /opt/robotx/RobotX-Pi/venv/bin/python -m pip check
```

`requirements.txt` pins `python-engineio==4.14.0` and `aiohttp==3.14.4`: the
link-loss timing (Y4) relies on this Socket.IO client honouring the backend's
handshake timing. Do not upgrade them on the robot.

Import checks, as the service user, from the working directory the unit uses:

```bash
cd /opt/robotx/RobotX-Pi
sudo -u robotx venv/bin/python -c "import robotx.application.main; print('ok')"
sudo -u robotx venv/bin/python -c "import picamera2; print('ok')"     # [HW-UNVERIFIED]
sudo -u robotx venv/bin/python -m uvicorn --version
```

Optional, recommended once per OS image: run the test suites on the Pi from a
**separate** checkout as an ordinary (non-root) user — not from `/opt`, and not
as root (root bypasses the file-permission checks some tests make):

```bash
git clone --branch gate3-pi-proposal https://github.com/somanathaB/RobotX.git ~/robotx-test
cd ~/robotx-test && git checkout --detach <release-commit>
python3 -m venv --system-site-packages venv
venv/bin/python -m pip install -r requirements.txt
venv/bin/python -m unittest discover -s tests/unit -t .          # expect 924 OK
venv/bin/python -m unittest discover -s tests/integration -t .   # expect 55 OK
```

## 12. Environment file

The agent reads only environment variables; systemd loads them from
`/etc/robotx/robotx-agent.env`. The file holds a secret (the signing key), so it
is owned by root and readable by nobody else — systemd reads it as root before
starting the service.

```bash
sudo install -d -m 0700 -o root -g root /etc/robotx
sudo install -m 0600 -o root -g root /dev/null /etc/robotx/robotx-agent.env
sudoedit /etc/robotx/robotx-agent.env
```

V1 contents. Replace each `<...>`; add nothing else unless a later section says
so. One `NAME=value` per line, no quotes, no spaces around `=`, no comments on
the same line as a value.

```ini
ROBOTX_ROBOT_ID=<commissioned-robot-id>
ROBOTX_LOG_LEVEL=INFO

ROBOTX_API_HOST=127.0.0.1
ROBOTX_API_PORT=8000

ROBOTX_SOCKET_ENABLED=1
ROBOTX_SOCKET_SERVER_URL=https://<backend-host>
ROBOTX_BACKEND_TLS_VERIFY=1
ROBOTX_BACKEND_TOKEN_PATH=/var/lib/robotx/backend_session.json
ROBOTX_COMMITMENT_STATE_PATH=/var/lib/robotx/commitments.json
ROBOTX_COMMAND_SIGNING_KEY=<64 hex characters, identical to the backend's COMMAND_SIGNING_KEY>
ROBOTX_BACKEND_HEARTBEAT_INTERVAL_S=2.0
ROBOTX_BACKEND_LOSS_POLICY=pause
ROBOTX_BACKEND_LOSS_GRACE_S=10.0

ROBOTX_ESP32_ENABLED=1
ROBOTX_ESP32_PORT=/dev/ttyAMA0
ROBOTX_ESP32_TRANSMIT_ENABLED=1
ROBOTX_ESP32_MOTION_ENABLED=0

ROBOTX_GPS_ENABLED=1
ROBOTX_GPS_SOURCE=esp32

ROBOTX_CUSTODY_CONFIRMATION=operator
```

Must **not** appear in this file:

| Variable | Why |
|---|---|
| `ROBOTX_ESP32_SIMULATOR_EXE` | Replaces the UART with a simulator: the robot would accept real OFFERs and never move. |
| `ROBOTX_PAIRING_CODE` | Goes in `pairing.env`, temporarily (section 17). |
| `ROBOTX_ROBOT_TOKEN` | Overrides the stored token and is never discarded, even when the backend rejects it. |
| `ROBOTX_SOCKET_NAMESPACE` | Read but has no effect. |
| `ROBOTX_HOME_LAT` / `ROBOTX_HOME_LON` | Not used in V1: RETURN abandons the engine mission it interrupts. |

Rules that the agent does **not** enforce for you ([SW-VERIFIED] in
`robotx/config/settings.py`):

- **Never leave a value empty.** An empty on/off setting is read as off
  (`ROBOTX_BACKEND_TLS_VERIFY=` disables certificate checks;
  `ROBOTX_SOCKET_ENABLED=` disables the backend link and the engine-mode lock on
  `/mission/start` and `/mission/resume`). An empty
  `ROBOTX_COMMITMENT_STATE_PATH=` keeps the commitment marks in memory only. An
  empty number stops the agent at startup.
- `esp32` and `operator` are compared exactly. `ESP32` or `Operator` silently
  fall back to the bench defaults, and every OFFER is then rejected.
- `ROBOTX_ROBOT_ID` is not trimmed: it must equal the commissioned id exactly.
- A backend setting the link refuses (bad URL, loss grace above 10, heartbeat
  above 2, policy `continue`) does **not** stop the service: the agent runs
  with the link disabled and logs `backend.config_invalid`. Section 20 catches it.

Check the file:

```bash
sudo stat -c '%U:%G %a %n' /etc/robotx /etc/robotx/robotx-agent.env
#   expect: root:root 700 /etc/robotx
#           root:root 600 /etc/robotx/robotx-agent.env
sudo grep -nE '^[A-Z0-9_]+=[[:space:]]*$' /etc/robotx/robotx-agent.env   # expect no output (no empty values)
sudo grep -c $'\r' /etc/robotx/robotx-agent.env                          # expect 0 (no Windows line endings)
sudo grep -c '^ROBOTX_ESP32_SIMULATOR_EXE' /etc/robotx/robotx-agent.env   # expect 0
```

Keep a dated copy before every later edit:

```bash
sudo cp -p /etc/robotx/robotx-agent.env /etc/robotx/robotx-agent.env.bak-$(date +%Y%m%d-%H%M)
```

## 13. Signing-key provisioning

The backend signs every engine command (OFFER, WITHDRAW, RECALL, ...) with
HMAC-SHA256 under its `COMMAND_SIGNING_KEY`; the Pi verifies with
`ROBOTX_COMMAND_SIGNING_KEY`. One value, identical on both sides, persistent.
Both sides use the UTF-8 bytes of the string exactly as written — neither
decodes hex — and require at least 32 bytes. [SW-VERIFIED] in
`robotx/communication/engine.py` and the backend's `commandSigning.js`.

**Generate** once, on the operator's trusted machine (not on the Pi, never in
the repository), and store it in the team's secret store:

```bash
openssl rand -hex 32
```

64 hex characters. Hex is chosen because it contains nothing a parser could
reinterpret: no quotes, spaces, `#`, `$`, `=` or backslashes. systemd strips
surrounding whitespace and processes quotes and backslashes; the Pi does not
strip anything — so any other character set risks the two sides holding
different bytes.

**Backend.** Set `COMMAND_SIGNING_KEY` in the backend's process environment
(the hosting provider's environment settings). The backend refuses to boot the
engine with a key shorter than 32 bytes. Its local demonstration launcher mints a
random key when none is set — a robot can never verify that one.

**Pi.** Add the line to `/etc/robotx/robotx-agent.env` with `sudoedit` (never
`echo ... >>`, which leaves the key in shell history):

```ini
ROBOTX_COMMAND_SIGNING_KEY=<the 64 hex characters>
```

**Verify without revealing the key:**

```bash
# on the Pi: length (expect 64)
sudo awk -F= '$1=="ROBOTX_COMMAND_SIGNING_KEY"{print length($2)}' /etc/robotx/robotx-agent.env
# on the Pi: fingerprint
sudo awk -F= '$1=="ROBOTX_COMMAND_SIGNING_KEY"{printf "%s", $2}' /etc/robotx/robotx-agent.env | sha256sum | cut -c1-12
# on the operator machine, from the stored key (typed, not echoed): must print the same 12 characters
read -rs K; printf '%s' "$K" | sha256sum | cut -c1-12; unset K
```

After the service is running (section 19):
`curl -s http://127.0.0.1:8000/backend | jq .command_signing_key` → `"SET"`.
The functional proof is section 22: an OFFER answered, with no
`engine.not_admitted` and `BAD_SIGNATURE` in the journal.

**Rotation.** Neither side accepts two keys at once, and RECALL and WITHDRAW are
signed too. Rotate only when the backend has **no unreleased commitment for any
robot** (runbook, "Checking commitment ownership", with the robot filter
removed): during a mismatch every engine command — including a RECALL — is
silently not admitted. Order: backend, then each Pi (edit, then restart while
IDLE), then section 22 again. A key that may have leaked (lost SD card, copied
file) is compromised for the **whole fleet**.

## 14. journald

The agent logs to stderr; systemd puts it in the journal under the identifier
`robotx-agent`. Every agent line arrives at the same journal priority, so filter
by event name, not with `journalctl -p`.

[HW-UNVERIFIED] Keep the journal across reboots — incident review depends on it
— with a size cap to protect the SD card. Check first:

```bash
journalctl --header | grep -i 'file path'      # /var/log/journal/... = persistent; /run/log/journal/... = volatile
```

If it is volatile:

```bash
sudo install -d -m 0755 /etc/systemd/journald.conf.d
printf '[Journal]\nStorage=persistent\nSystemMaxUse=200M\n' | sudo tee /etc/systemd/journald.conf.d/robotx.conf
sudo systemctl restart systemd-journald
```

Reading it:

```bash
journalctl -u robotx-agent -b            # this boot
journalctl -u robotx-agent -f            # follow
journalctl -u robotx-agent -b | grep -E 'agent\.(started|start_failed)|backend\.(config_invalid|auth_failed|loss_pause|heartbeat_too_slow)|esp32\.simulated|api\.network_exposed'
```

No secret is ever logged: the signing key, pairing code and token appear only as
`SET`/`UNSET`. [SW-VERIFIED]

## 15. Time synchronisation

Required, not cosmetic. [SW-VERIFIED] The wall clock is used for:

| Use | Where | Effect of a wrong clock |
|---|---|---|
| OFFER validity (`notValidAfter`, 20 s offer lifetime) | `engine.py` | Pi clock ~20 s fast: no OFFER is ever admitted |
| COMMAND age (120 s) | `protocol.py` | Pi clock >120 s fast: operator STOP/PAUSE/RESUME commands (dashboard or operator command API) rejected |
| TELEMETRY sequence seed | `backend_link.py` | Pi clock behind its previous run: every position refused as a stale sequence for the whole run |
| Position timestamps | backend `time.max_clock_skew` = **500 ms** | Pi clock >0.5 s fast: positions refused as ahead of the server |

The unit is ordered `After=time-sync.target`. On Debian that target is only held
back until the clock is actually synchronised when a wait service is enabled:

```bash
systemctl is-active systemd-timesyncd chrony 2>/dev/null
sudo systemctl enable systemd-time-wait-sync.service    # if systemd-timesyncd is the client
# or, if chrony is the client:
sudo systemctl enable chrony-wait.service
```

Verify:

```bash
timedatectl                        # "System clock synchronized: yes", "NTP service: active"
timedatectl timesync-status        # systemd-timesyncd only: offset
date -u; curl -sI https://<backend-host>/health | grep -i '^date:'   # coarse cross-check (1 s resolution)
```

[HW-UNVERIFIED] Whether the Pi 5's RTC has a battery, what the clock reads at
boot before synchronisation, and whether the site's network allows NTP. **If NTP
is blocked, the wait service never completes and the agent never starts.** That
is fail-safe (no motion, no OFFERs), but it must be diagnosed with
`systemctl status systemd-time-wait-sync`, not by disabling the wait.

## 16. systemd installation

```bash
sudo install -m 0644 /opt/robotx/RobotX-Pi/deployment/robotx-agent.service /etc/systemd/system/robotx-agent.service
sudo systemd-analyze verify /etc/systemd/system/robotx-agent.service    # expect no output
sudo systemctl daemon-reload
```

[SW-VERIFIED] `systemd-analyze verify` passes on this unit in a Debian trixie
container. [HW-UNVERIFIED] The hardening directives have not run on the Pi; if
the camera or UART fails only under the service (but works for
`sudo -u robotx` by hand), report it rather than removing directives one by one.

What the unit guarantees:

- starts only as `venv/bin/python -m robotx.application`, which binds
  `ROBOTX_API_HOST:ROBOTX_API_PORT` through `main.run()` (never
  `uvicorn robotx.application.main:app`, which bypasses it);
- runs as `robotx` with `dialout` and `video`, from `/opt/robotx/RobotX-Pi`;
- creates `/var/lib/robotx` (mode 0700, owned by `robotx`) for the token and
  commitment files;
- restarts on failure after 5 s, at most 5 starts in 300 s, then stays failed
  until `sudo systemctl reset-failed robotx-agent`;
- waits for time synchronisation (section 15); has no dependency on the UART or
  camera devices (the ESP32 link reconnects by itself).

Do not `enable` it yet; section 17 starts it for the first time.

`sudo systemd-analyze security robotx-agent` gives an informational exposure
score; it is not a pass/fail check.

## 17. Commissioning and pairing

The backend issues a one-time 6-digit pairing code (300 s lifetime) for the
robot id. The first AUTH spends it and returns a session token, which the Pi
stores in `/var/lib/robotx/backend_session.json`; from then on the token is used.

**1. Get the code — on the operator's workstation, not on the Pi.** The route
requires an authenticated dashboard user. Log in first as described in the
operator runbook, "Operator command API" → "Authenticate" (the backend returns
the session only as a `token` cookie, kept in `$COOKIES`). Then send the same
request the repository's commissioning helper sends:

```bash
curl -sS -b "$COOKIES" -X POST "https://<backend-host>/api/robots/commission" \
  -H 'Content-Type: application/json' \
  -d '{"robotId":"<commissioned-robot-id>","simulated":false}'
```

The response carries `"pairingCode"` and `"expiresIn":300`. Delete the cookie
file when finished (`rm -f "$COOKIES"`). The repository's helper
(`python -m robotx.communication.commissioning`) is not used here: it needs the
`requests` package, which `requirements.txt` does not install (helper debt, not a
V1 blocker), and it would put an operator credential on the robot.

**2. Within 300 s, on the Pi:**

```bash
sudo install -m 0600 -o root -g root /dev/null /etc/robotx/pairing.env
sudoedit /etc/robotx/pairing.env          # one line: ROBOTX_PAIRING_CODE=<6 digits>
sudo systemctl start robotx-agent
curl -s http://127.0.0.1:8000/backend | jq '{status, authenticated, auth_method, auth_failure, credential, pairing_code}'
```

Expected: `authenticated: true`, `auth_method: "PAIRING_CODE"`,
`credential.token: "SET"`.

If `auth_failure` names a refusal: the code expired or was wrong. Get a new code
and repeat step 2 — at most a few times: **five failed pairing attempts lock
pairing for this robot for an hour**, and unlocking is a backend override
(`POST /api/robots/<id>/pairing/unlock`, gated as a quarantine override: elevated
role, recorded reason and, where configured, a second approver).

## 18. Pairing-code removal

Immediately after section 17 succeeds, with the robot IDLE:

```bash
sudo rm /etc/robotx/pairing.env
sudo systemctl restart robotx-agent
curl -s http://127.0.0.1:8000/backend | jq '{authenticated, auth_method, pairing_code, credential}'
```

Expected: `authenticated: true`, `auth_method: "TOKEN"`, `pairing_code: "UNSET"`,
`credential.token: "SET"`, `credential.path: "/var/lib/robotx/backend_session.json"`.

Why the restart: the code stays in the running process's environment until it
restarts. If the stored token is ever rejected, a configured code is retried
every 60 s and runs into the five-attempt lockout within minutes.

Then enable start at boot:

```bash
sudo systemctl enable robotx-agent
```

## 19. Loopback API verification

```bash
sudo ss -ltnp | grep ':8000'                             # expect 127.0.0.1:8000 only
journalctl -u robotx-agent -b | grep -c api.network_exposed   # expect 0
curl -s http://127.0.0.1:8000/health | jq '{ok, mode}'
```

From another machine on the same network, `http://<pi-address>:8000/health` must
fail to connect. The API is unauthenticated; in V1 it is reached only over SSH,
on the Pi. Use `127.0.0.1`, not a host name, in every command.

## 20. Backend verification

```bash
curl -s http://127.0.0.1:8000/backend | jq '{enabled, status, streaming, authenticated, tls, server, command_signing_key, loss_policy, link_loss, rates, last_connect_error}'
journalctl -u robotx-agent -b | grep -E 'backend\.(config_invalid|heartbeat_too_slow|no_tls)'   # expect no output
```

Expected:

| Field | Value |
|---|---|
| `enabled` | `true` |
| `status` / `streaming` | `"STREAMING"` / `true` |
| `tls` | `true` |
| `command_signing_key` | `"SET"` |
| `loss_policy` | `"pause"` |
| `link_loss.grace_s` | `10.0` |
| `link_loss.detection_bound_s` | `15` or less |
| `rates.heartbeat_interval_s` | `2.0` |

`status: "DISABLED"` with `enabled: true` means a backend setting was refused —
`detail` and the `backend.config_invalid` log line say which. A
`detection_bound_s` above 15 (for example 45) or a `backend.heartbeat_too_slow`
line means the backend is not running the Y4 link timing (10 s + 5 s): **do not
enable motion** — the robot could then keep driving after its lease is handed to
another robot.

## 21. First startup checks

```bash
curl -s http://127.0.0.1:8000/config | jq '{robot_id, socket_enabled, backend_tls_verify, gps_enabled, gps_source, custody_confirmation, esp32_enabled, esp32_port, esp32_motion_enabled, esp32_simulator_exe, api_host, commitment_state_path, backend_token_path}'
journalctl -u robotx-agent -b | grep -c esp32.simulated      # expect 0
curl -s http://127.0.0.1:8000/health | jq '.health.components | map_values(.status)'
curl -s http://127.0.0.1:8000/state | jq '{mode, safety: .safety.rule, esp32: .communication.esp32, motion_ready: .controller.motion_ready, reboot_latched: .controller.reboot_latched, gps: .gps.status, position: .position.source}'
```

Expected: the `/config` values of section 12 (`esp32_simulator_exe: ""`);
`mode: "IDLE"`; `esp32: "UP"`; `reboot_latched: false`; `motion_ready: false`
(motion is still disabled); camera, perception and esp32 components `HEALTHY`.
GPS: `FIX` and `position: "GPS"` outdoors with sky view; indoors `NO_FIX` is
expected. [HW-UNVERIFIED] for everything that depends on the ESP32, the GPS and
the camera.

A camera that fails to start is **not retried**: the agent runs without it and
the robot will never drive (perception unavailable means stop). If
`components.camera.status` is `FAILED` with "camera did not start", fix the
cause and restart the service.

## 22. First OFFER with motion disabled

With `ROBOTX_ESP32_MOTION_ENABLED=0` the robot rejects every OFFER with
`NO_MOTOR_LINK`. That is used here to prove — with no possibility of motion —
that the signing key, the robot id and the admission path work end to end.

Have an OFFER produced for this robot (create a task the backend will offer to
it), then:

```bash
journalctl -u robotx-agent --since -10min | grep -E 'offer\.answered|engine\.not_admitted'
```

Expected: `offer.answered ... verdict=REJECT reason=NO_MOTOR_LINK`, and no
`engine.not_admitted`. On the backend, the OFFER ends rejected.

| Instead you see | Meaning |
|---|---|
| `engine.not_admitted ... reason=SIGNATURE_UNVERIFIABLE` | no signing key on the Pi (section 13) |
| `engine.not_admitted ... reason=BAD_SIGNATURE` | the two keys differ (compare fingerprints, section 13) |
| `engine.not_admitted ... reason=NOT_VALID_AFTER_PASSED` | clock skew or a delayed delivery (section 15) |
| `engine.not_admitted ... reason=ADDRESSED_TO_ANOTHER_AGENT` | `ROBOTX_ROBOT_ID` differs from the commissioned id |
| a REJECT other than `NO_MOTOR_LINK` (`NO_POSITION_FIX`, `NO_CUSTODY_SENSING`, `ESTOP_LATCHED`, ...) | the named condition; fix it before enabling motion |
| nothing at all | the backend did not offer to this robot; check the backend, not the Pi |

## 23. Safe motion enablement

[HW-UNVERIFIED] — none of this has been done on the robot. Do not skip steps.

**There is no physical emergency stop on this robot yet** (the ESP32 firmware has
no e-stop input). The software e-stop (`POST /safety/estop`) is **not** a
substitute for a physical emergency disconnect. Until one is fitted, a person
able to cut drive power must be within reach for every motion test.

1. Sections 19–22 all pass. The backend shows `link_loss.detection_bound_s` ≤ 15.
2. Wheels off the ground (rover on a stand).
3. Check the ESP32's own sensor state: `curl -s http://127.0.0.1:8000/health | jq .health.components.esp32`.
   The `detail` carries what the ESP32 itself reports (drive unavailable and why,
   front sensing not valid, a latched safety stop). The right-front ultrasonic is
   recorded as faulty, and the firmware blocks forward motion on a front-sensor
   fault — resolve the hardware, do not work around it.
4. Edit the environment file: `ROBOTX_ESP32_MOTION_ENABLED=1` (keep a dated copy
   first, section 12). Restart while IDLE: `sudo systemctl restart robotx-agent`.
5. Expect `controller.motion_ready: true` in `/state`. If it stays `false`, the
   ESP32 is not reporting its drive as available — look at `/health`
   `components.esp32.detail`.
6. Software e-stop check, wheels still off the ground: `POST /safety/estop`,
   confirm `/state` `.safety.rule == "estop"`; then `POST /safety/clear` (runbook).
7. First supervised OFFER on the ground: a short route, clear area, an operator
   at the robot with the means to cut power, another at the dashboard. Follow the
   runbook's custody procedure at both stops.
8. Watch the link-loss pause once, deliberately (for example by disconnecting the
   Pi's network during a supervised run): expect `backend.loss_pause` within about
   25 s and the robot holding. Then follow the runbook's RESUME procedure.
   Plan for the consequence: after any PAUSE, including this automatic one, the
   backend offers the robot no new work until escalated (runbook, "PAUSE
   semantics"). Schedule this test with the backend owner.

## 24. Rollback

[SW-VERIFIED] (from the commit history; not exercised on the robot).

| Commit | Use |
|---|---|
| `099e386` / `130e3f6` (and the release commit) | V1 functional candidate: the only commits with working V1 navigation. |
| `db10b15`, `8073c71` | **Emergency software rollback only, with `ROBOTX_ESP32_MOTION_ENABLED=0`.** They run under this unit, keep the engine-mode lock (H1), the loopback bind (H2) and the link-loss pause (Y4), and read the same settings — but their navigation does not complete a V1 route. |
| `7c58cd6` | **Not a rollback target.** No `robotx/application/__main__.py`, so `python -m robotx.application` cannot start it; it binds every interface by default and lacks the engine-mode lock. |
| anything older, `master`, `laptop-pi` | **Not rollback targets.** They lack the link-loss pause and/or drop every OFFER. |

Never roll back while this robot holds a commitment (runbook, "Checking
commitment ownership"). If goods are on board, follow the runbook's
stranded-goods procedure instead.

1. The robot is IDLE and the backend shows no unreleased commitment for it.
2. Keep a dated copy of the environment file (section 12). For any target other
   than the V1 candidate, set `ROBOTX_ESP32_MOTION_ENABLED=0`.
3. Stop: `sudo systemctl stop robotx-agent`; `journalctl -u robotx-agent -n 20`
   shows `agent.stopped`.
4. Back up the state — never delete `commitments.json` (it is what stops a
   replayed OFFER from being answered twice):
   `sudo cp -a /var/lib/robotx /var/lib/robotx.bak-$(date +%Y%m%d-%H%M)`
5. Switch the code:

   ```bash
   sudo git -C /opt/robotx/RobotX-Pi fetch origin
   sudo git -C /opt/robotx/RobotX-Pi checkout --detach <target>
   sudo git -C /opt/robotx/RobotX-Pi rev-parse HEAD        # must print <target>
   sudo git -C /opt/robotx/RobotX-Pi status --porcelain    # must print nothing
   ```

6. Dependencies: if `sudo git -C /opt/robotx/RobotX-Pi diff --stat <from> <target> -- requirements.txt`
   is not empty, rebuild the virtual environment (sections 10–11). Older commits
   do not pin `python-engineio`/`aiohttp`; pip may then resolve newer versions
   than were verified.
7. The unit: the installed copy in `/etc/systemd/system` keeps working (the
   rollback targets have no `deployment/` directory; nothing reads it at run
   time). Restore it only if it was changed: keep
   `/etc/systemd/system/robotx-agent.service.bak` before any unit change, then
   `sudo systemctl daemon-reload`.
8. Start and verify: `sudo systemctl start robotx-agent`, then sections 19–21.
   `/config` must show `esp32_motion_enabled: false` for a non-V1 target.
9. A robot on anything but the V1 candidate stays out of service (section 25).

To return to the V1 candidate, repeat with `<release-commit>`, restore the
environment file copy, and redo sections 19–22.

## 25. Out-of-service procedure

Prerequisite: the backend shows no unreleased commitment for this robot. If it
does, finish, PAUSE or STOP the mission first (runbook); with goods on board,
use the stranded-goods procedure.

**A — out of service, still visible on the dashboard** (preferred for short
periods): set `ROBOTX_ESP32_MOTION_ENABLED=0`, `sudo systemctl restart robotx-agent`
while IDLE, then `curl -s -X POST http://127.0.0.1:8000/safety/estop`. Every OFFER
is rejected (`NO_MOTOR_LINK`, and `ESTOP_LATCHED`), no motion command can leave
the Pi, and telemetry continues. A restart clears the software e-stop: re-apply
it after every restart.

**B — offline:** `sudo systemctl disable --now robotx-agent`. The robot goes
offline on the dashboard.

Back into service: reverse A or B, then sections 19–22 (and 23 if motion was off).

## 26. Hardware-dependent checks still outstanding

None of these has been done on the physical robot:

- the OS release, Python version and CPU/thermal behaviour on the Pi 5 (only
  emulated ARM64 has been tested);
- which device node is the GPIO 14/15 UART, and the ESP32 link over it;
- the camera under the service user and the unit's hardening;
- a GPS fix through the ESP32, outdoors;
- ESP32 motion readiness, the faulty front ultrasonic, the 2 s ESP32 watchdog
  stopping the motors when commands stop;
- a physical emergency stop (none fitted);
- the RTC battery, the clock at boot, and NTP reachability at the site;
- the link-loss pause on the real network (measured only in software, 24.6 s);
- a complete supervised delivery with operator custody confirmation;
- clean shutdown: `esp32.final_stop` on `systemctl stop` after motion (an open
  `/camera` viewer prevents it; see the runbook).
