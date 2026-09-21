# RobotX Pi Agent — Remediation Plan

**Date:** 2026-09-22
**Status:** R-00, R-01, R-02, R-03 implemented and verified (see "Implementation status" below). R-04 through R-10 not yet started — awaiting approval to proceed.
**Method:** Every finding below was re-derived directly from the current files on disk (not copied from the prior audit without verification). File:line references point at the code as it exists right now. Findings are classified CRITICAL / HIGH / MEDIUM / LOW.

## Implementation status (2026-09-22)

| ID | Status | Notes |
|---|---|---|
| R-00 | **FIXED** | `venv/bin/python` execute bit restored (`chmod +x`); root cause fully diagnosed (venv copied from a now-nonexistent path, breaking console-script shebangs beyond just the permission bit). Fix is to always invoke `venv/bin/python -m <module>` rather than the broken shim scripts, which were intentionally left unmodified. `README.md`/`TEST_README.md` updated to the correct path and invocation form. |
| R-01 | **FIXED** (scoped) | `_apply_manual()` now receives the same `blocked` signal AUTO/RETURN already use and refuses net-forward motion while blocked. Backward/turning remain available (no rear sensor exists). This is a targeted gate on the existing method, not a new centralized `SafetyController` module — that larger refactor from the original plan draft was deferred per explicit scope instruction. |
| R-02 | **PARTIALLY FIXED — client-side only, by design** | `RobotSocketClient.connect()` now sends `auth={"robot_id":..., "token":...}` via `python-socketio`'s native handshake-level auth mechanism, sourced from `ROBOTX_ROBOT_TOKEN` (env var only, never hardcoded). This repo has no server to verify it against, so the channel remains unauthenticated in practice until a backend implements verification. See "R-02 backend integration requirement" below. |
| R-03 | **FIXED** | `UltrasonicSensor` now exposes `get_reading()` returning an explicit `UltrasonicStatus` (VALID/TIMEOUT/OUT_OF_RANGE/ERROR/DISCONNECTED/STALE/UNKNOWN) instead of collapsing all failure modes into `None`. `RobotController._safety_blocked()` and `_avoid()` now treat any non-VALID status as blocking. `get_distance()` kept as a backward-compatible wrapper for the two standalone test scripts that use it. |

### R-02 backend integration requirement (what remains outside this repo)

This repo can only implement the **client** half of authentication. For the channel to actually be secured, the external Socket.IO server (not present in this repository) must:
1. Accept the `auth` object delivered in the Engine.IO/Socket.IO connect handshake (this is a standard `python-socketio`/`socket.io` server-side feature — reading `environ["asgi.scope"]["auth"]` or the framework's equivalent `auth` argument in its `connect` handler).
2. Look up `auth["robot_id"]` and verify `auth["token"]` against a stored credential for that robot.
3. Reject the connection (raise `ConnectionRefusedError` in the server's `connect` handler, or return `False`) if the token is missing, unknown, or mismatched — python-socketio propagates this back to the client as a `ConnectionError` from `sio.connect()`, which this repo's `socket_boot()` in `main.py` already handles non-fatally (the robot keeps running its local control loop even if the socket rejects).
4. Reject any `command`/`manual` event arriving on a socket that never completed step 2-3 successfully (should not be reachable if step 3 is enforced at connect time, but defense-in-depth is recommended server-side).

No token issuance/rotation mechanism is implemented or assumed — `ROBOTX_ROBOT_TOKEN` is treated as an opaque bearer string the operator provisions out-of-band (e.g. set once via the systemd `EnvironmentFile`, not yet created — see R-08, deferred). This is intentionally not a JWT/HMAC/cert scheme — inventing one without a backend to match it against would create exactly the "incompatible invented protocol" the task instructions warned against.

All findings from `ROBOTX_DEEP_CURRENT_STATE_AUDIT.md` were re-checked against source. All were confirmed still present except where noted. One new finding (R-00) was discovered during re-verification that the original audit did not identify.

---

## R-00 — venv is broken in a second way the original audit did not catch

**Severity:** CRITICAL (blocks everything else)
**Affected files:** `venv/bin/python`, `venv/bin/pip`, `venv/bin/pip3`, `venv/bin/pip3.13`, `venv/bin/uvicorn`, `venv/bin/fastapi`, `venv/bin/httpx`, `venv/bin/watchfiles`, `venv/bin/websockets`, `venv/pyvenv.cfg`

**Root cause:** Verified directly:
- `venv/bin/python` is missing the execute bit (`-rw-rw-r--`, mode 664) — confirms the original audit's finding.
- `venv/bin/python`'s **content is correct** — `md5sum` matches `/usr/bin/python3.13` byte-for-byte, so the interpreter itself is not corrupted, just not executable.
- There is **no `venv/bin/python3` or `venv/bin/python3.13`** in the directory at all (a normal `python -m venv` creates these as symlinks alongside `python`) — only `python` exists, as a regular file, not a symlink.
- `venv/pyvenv.cfg` records `command = /usr/bin/python3 -m venv --system-site-packages /home/pi/RobotX/venv_cam` — this venv was originally created at path `/home/pi/RobotX/venv_cam`, not its current location `/home/pi/Desktop/RobotX/venv`. That original path no longer exists on this Pi (`/home/pi/RobotX` is gone).
- Every console-script shim in `venv/bin/` (`pip`, `pip3`, `pip3.13`, `uvicorn`, `fastapi`, `httpx`, `watchfiles`, `websockets`) has the shebang `#!/home/pi/RobotX/venv_cam/bin/python3` — a **stale, nonexistent path**.

**Conclusion:** This venv directory was copied/rsynced/re-extracted from its original build location to its current path. The copy operation flattened symlinks into a single regular file (`python`) and dropped the permission bit, and none of the pip-generated console-script shebangs were regenerated for the new location. Simply `chmod +x venv/bin/python` (the original audit's proposed fix) is necessary but **not sufficient** — `venv/bin/uvicorn` and `venv/bin/pip` would still fail with "bad interpreter: No such file or directory" even after the chmod, because their shebang points at a path that doesn't exist on this machine.

**Proposed fix:**
1. `chmod +x venv/bin/python` (restores the one binary that's actually needed).
2. Do **not** patch every shebang by hand (fragile, easy to miss one, and pip-generated shims regenerate their own paths on reinstall anyway). Instead, standardize the documented run/install commands to always invoke `venv/bin/python -m <module>` (`venv/bin/python -m pip install -r requirements.txt`, `venv/bin/python -m uvicorn robotx.app.main:app ...`), which only depends on `venv/bin/python` being executable, not on any shebang line.
3. Update `README.md` and `deployment/robotx-agent.service` (new, Phase 17) to use the `-m` invocation form exclusively.

**Safety impact:** None — this is a pure tooling/deployment fix, no runtime behavior changes.
**Compatibility impact:** None — `-m` invocation is standard and behaves identically to the console-script shims.
**Testing required:** `venv/bin/python -c "import robotx.app.main; print('ok')"` must succeed; `venv/bin/python -m uvicorn --version` must succeed.
**Documentation required:** `README.md` run instructions, `ROBOTX_PI_SETUP_AND_RUN.md` (new).

---

## R-01 — MANUAL mode bypasses all obstacle/person-detection safety checks

**Severity:** CRITICAL
**Affected files:** `robotx/control/controller.py:351-352`, `robotx/control/controller.py:417-440` (`_apply_manual`)

**Root cause:** In `RobotController._loop()`, `_safety_blocked()` is computed every tick from ultrasonic + IR + vision (`controller.py:346`), but it is only *consulted* in the `AUTO`/`RETURN` branch (`controller.py:354-373`). The `MANUAL` branch (`controller.py:351-352`) calls `self._apply_manual()` unconditionally — `blocked` is never read. A `{"type":"MANUAL","payload":{"action":"forward","speed":1.0}}` command drives the robot at commanded speed regardless of what the sensors see.

**Proposed fix:** Introduce one centralized safety gate that every mode's motor command passes through before reaching `MotorDriver`, rather than patching the `MANUAL` branch in isolation (patching in isolation would just create a fourth duplicate safety check alongside the existing `_safety_blocked`/`_avoid`/`DecisionEngine`/`VisionController` implementations the audit already flagged as duplicated in §34). Concretely:
- Add a `SafetyController` (new module, `robotx/control/safety.py`) that takes the current sensor snapshot and a *requested* `(left, right)` motor command from any mode, and returns an *approved* `(left, right)` command — zeroing or clamping it if blocked, regardless of which mode requested it.
- `RobotController._loop()` computes the requested command for whichever mode is active (AUTO/RETURN route-follow, MANUAL joystick, or explicit STOP), then always routes it through `SafetyController.filter(requested, sensor_snapshot)` before calling `motors.set_speed(...)`.
- A `forward`/`backward`-class manual command while blocked is reduced to `(0, 0)`; turning in place is still permitted since it does not close distance to a detected obstacle (this preserves legitimate close-quarters manual maneuvering — e.g. turning away from an obstacle — the concern the original audit flagged in its own Phase-2 risk note).

**Safety impact:** Closes the CRITICAL collision-risk gap. This is the highest-priority code change in this plan.
**Compatibility impact:** MANUAL forward/backward commands issued while an obstacle is within `obstacle_distance_cm` will now be refused (motors held at zero) instead of executed — this is an intentional, documented behavior change for any existing operator tooling.
**Testing required:** Unit test with a mocked sensor snapshot: MANUAL forward while `distance_cm < threshold` → motors receive `(0,0)`; MANUAL forward while clear → motors receive the requested command; MANUAL turn while blocked → still permitted.
**Documentation required:** `ROBOTX_PI_SAFETY_MODEL.md` (new).

---

## R-02 — No authentication on the Socket.IO command channel

**Severity:** CRITICAL
**Affected files:** `robotx/app/sockets.py:26-67`, `robotx/utils/config.py`

**Root cause:** `RobotSocketClient.connect()` (`sockets.py:66-67`) calls `self.sio.connect(url, namespaces=[...])` with no `auth=` parameter. `robot_hello` (`sockets.py:44`) sends a self-reported `robot_id` string with no proof of identity. Any party that can reach the configured namespace can emit `command`/`manual` events (`sockets.py:51-64`), which are queued and executed unconditionally by `controller.handle_command()`.

**Proposed fix:** `python-socketio`'s `AsyncClient.connect()` natively supports an `auth=` dict, delivered to the server during the Engine.IO handshake before any event is processed. Add:
- `ROBOTX_ROBOT_TOKEN` (new required-for-remote-operation env var, read only from environment, never hardcoded) — a bearer credential for this specific robot.
- `RobotSocketClient.connect()` passes `auth={"robot_id": ..., "token": SETTINGS.robot_token}`.
- Since the external server implementation is **out of scope of this repo** (confirmed absent — see audit §6, §36-A), this repo can only implement the *client*-side half of the handshake. Document the required server-side contract explicitly in `ROBOTX_PI_ROBOT_AGENT_PROTOCOL.md` (new) so whoever builds the backend has an unambiguous spec: reject `connect` if `auth.token` doesn't match a known robot record; reject any `command`/`manual` event on a socket that didn't complete `auth`.
- On the client: if `ROBOTX_ROBOT_TOKEN` is unset, log a clear startup warning (`"ROBOTX_ROBOT_TOKEN not set — socket connection is unauthenticated; do not use on an untrusted network"`) rather than silently connecting unauthenticated, so the gap is visible in logs rather than invisible.

**Safety impact:** Prevents unauthorized parties from issuing `MANUAL`/`START`/`STOP`/`RETURN` commands. Indirect safety impact (this is primarily a security/authorization fix), but the consequence of exploitation is unauthorized robot motion.
**Compatibility impact:** Requires the (not-yet-built) external server to implement matching token verification; this repo cannot complete this fix unilaterally — client-side abstraction only, per the task's own constraint not to invent an incompatible backend protocol.
**Testing required:** Unit test that `SocketConfig`/`RobotSocketClient` correctly includes `auth` in the connect call when a token is configured; a warning is logged when it is not.
**Documentation required:** `ROBOTX_PI_ROBOT_AGENT_PROTOCOL.md` (new), `.env.example` (new).

---

## R-03 — Ultrasonic sensor failure/timeout is treated as "no obstacle"

**Severity:** CRITICAL (safety-adjacent — silent failure mode)
**Affected files:** `robotx/hardware/ultrasonic.py:69-98`, `robotx/control/controller.py:258-265`

**Root cause:** `_measure_distance_cm()` returns `None` on GPIO unavailable, 30ms echo timeout, or an out-of-range reading (`ultrasonic.py:83-85`, `88-90`, `96-97`) — all three distinct failure modes collapse to the same `None`. `_safety_blocked()` (`controller.py:258-259`) only trips when `distance_cm is not None and distance_cm < threshold` — a `None` reading (sensor unplugged, timed out, or GPIO absent) is treated as "not blocked," identical to a genuine clear reading.

**Proposed fix:** Replace the `Optional[float]` return with an explicit status model:
```python
class UltrasonicStatus(Enum):
    VALID = "valid"           # fresh in-range reading
    NO_OBSTACLE = "no_obstacle"  # not used as a distinct state from VALID; VALID + distance implies it
    TIMEOUT = "timeout"       # echo pulse timed out
    OUT_OF_RANGE = "out_of_range"  # reading <=0 or >500cm
    DISCONNECTED = "disconnected"  # GPIO unavailable (no RPi.GPIO)
    STALE = "stale"           # last valid reading is older than N poll cycles
    UNKNOWN = "unknown"       # not yet sampled since start()
```
`UltrasonicSensor.get_reading() -> UltrasonicReading(status, distance_cm, age_s)` replaces the bare `get_distance()` (kept as a thin deprecated wrapper returning just `distance_cm` for the two existing callers that will be migrated). `SafetyController` (R-01) treats any status other than a fresh `VALID` reading with `distance_cm >= threshold` as **blocking** — i.e., `TIMEOUT`/`OUT_OF_RANGE`/`DISCONNECTED`/`STALE`/`UNKNOWN` all conservatively block forward motion, per the task's own default-to-conservative instruction. Turning in place and reverse remain available since they don't require forward-obstacle sensing to be safe by the same logic already used for MANUAL turning in R-01 — however, since ultrasonic only covers the front, this needs explicit documentation of exactly what "blocking" means as an "affects forward motion only" scope, not a full immobilize.

**Safety impact:** Closes a silent, real safety gap — a disconnected/failed ultrasonic sensor currently degrades safety without any indication. After the fix, sensor failure is conservative (blocks forward motion) and observable in telemetry/logs.
**Compatibility impact:** If the ultrasonic sensor is not wired up at all (e.g., bench-testing without it), the robot will now refuse forward AUTO/MANUAL motion by design — this must be clearly documented as intended, not a bug, and callable out in `ROBOTX_PI_HARDWARE_INTERFACE.md`.
**Testing required:** Unit tests for all seven states (valid, obstacle, timeout, malformed/out-of-range, exception during measurement, repeated failure, recovery-to-valid) using a mocked GPIO — per Phase 4/18 of the task instructions.
**Documentation required:** `ROBOTX_PI_SAFETY_MODEL.md`, `ROBOTX_PI_HARDWARE_INTERFACE.md`.

---

## R-04 — Battery telemetry is a hardcoded constant reported as real

**Severity:** HIGH
**Affected files:** `robotx/control/controller.py:386`

**Root cause:** `"battery": {"percent": 76.0}` is a literal constant in the telemetry dict. Confirmed via grep: no ADC, INA219, voltage-divider, or any battery-sensing code exists anywhere in the repository.

**Proposed fix:** No battery-sensing hardware exists on this robot today (not verified present — no I2C fuel-gauge IC, no ADC HAT referenced anywhere in config/wiring docs). Per the task's explicit instruction not to invent hardware that doesn't exist, the fix is **representational, not a fake sensor**:
- Telemetry's `battery` field becomes `{"state": "UNAVAILABLE", "percent": null}` where `state` is one of `REAL | ESTIMATED | SIMULATED | UNAVAILABLE`.
- Add a `BatteryReader` interface (`robotx/hardware/battery.py`, new) with a single implementation today: `UnavailableBatteryReader` that always returns `state=UNAVAILABLE, percent=None`. This defines the seam for a future real reader (e.g. INA219-backed) without fabricating one now.
- `main.py` instantiates `UnavailableBatteryReader` by default; a future `ROBOTX_BATTERY_BACKEND=ina219` config switch (not implemented now, documented as FUTURE INTEGRATION) would swap it.

**Safety impact:** None directly (no low-battery auto-behavior exists to feed today), but prevents a downstream consumer (dashboard, auto-return-on-low-battery logic if ever built) from silently trusting fabricated data.
**Compatibility impact:** Any existing consumer of `telemetry.battery.percent` as a bare float will need to handle the new `{state, percent}` shape and a possible `null` percent — this is a breaking schema change, called out explicitly in the protocol doc.
**Testing required:** Telemetry schema test confirming `battery.state == "UNAVAILABLE"` and `percent is None` in the absence of real hardware.
**Documentation required:** `ROBOTX_PI_ROBOT_AGENT_PROTOCOL.md`, `ROBOTX_PI_HARDWARE_INTERFACE.md`.

---

## R-05 — VisionController/DecisionEngine disconnected from production; RobotController has zero test coverage

**Severity:** HIGH
**Affected files:** `robotx/control/vision_controller.py`, `robotx/control/decision_engine.py`, `robotx/control/controller.py`, `test_vision.py`, `test_controller.py`

**Root cause:** Confirmed by grep — `VisionController`/`DecisionEngine` are imported only by `test_vision.py`; `RobotController` never imports either. `test_controller.py` does not import `robotx.control.controller` at all (misleading filename, independent reimplementation).

**Decision (per task Phase 10, evaluated against actual code/hardware requirements):** **OPTION B** — retain `RobotController`'s simpler `_safety_blocked`/`_avoid` as the live safety-relevant obstacle logic (now centralized into `SafetyController` per R-01/R-03), and formally document `VisionController`/`DecisionEngine` as **experimental/reference-only**, not production. Rationale:
- Promoting `VisionController` (Option A) would change live robot motion behavior (hysteresis, hold timers, hard-coded pixel-zone thresholds at `213`/`426`/`30000`/`10000` not derived from actual frame width) without any bench-test evidence it's tuned for this robot's actual camera/mounting — the task explicitly says not to change behavior without ability to verify against hardware, and no such verification exists.
- The two pipelines have materially different safety philosophies (`VisionController` allows continuous turning without a full stop; `RobotController`'s `_avoid()` performs a full stop first) — merging them (Option C, "keep both behind interfaces") without a deliberate integration test plan would create exactly the "which one is actually protecting the robot" ambiguity the audit already flagged.
- `VisionController`/`DecisionEngine` are not deleted (798 + 36 lines of real, working logic) — they are relabeled via module docstrings and `TEST_README.md` as explicitly experimental, so a future maintainer doesn't assume they're live.

**Proposed fix:**
1. Add a top-of-file docstring to `vision_controller.py` and `decision_engine.py`: `"""EXPERIMENTAL / NOT WIRED INTO PRODUCTION. Exercised only by test_vision.py. See ROBOTX_PI_ARCHITECTURE.md §Vision Pipelines for the promotion decision and rationale."""`
2. Rename `test_controller.py`'s misleading self-description — add a header comment clarifying it does **not** exercise `RobotController`.
3. Add a real `pytest` suite (Phase 18) that imports and exercises `RobotController`'s pure-logic pieces (`_safety_blocked`, `_route_follow_command`, `handle_command`) with mocked hardware — closing the "zero coverage" gap the audit identified, without touching real GPIO.

**Safety impact:** None (documentation + test-only change; no live behavior changes).
**Compatibility impact:** None.
**Testing required:** New `tests/test_controller.py` (mocked hardware) — see Phase 18 plan below.
**Documentation required:** `ROBOTX_PI_ROBOT_AGENT_ARCHITECTURE.md` (new) must state this decision and rationale explicitly.

---

## R-06 — No communication-loss watchdog; stale commands can persist indefinitely

**Severity:** HIGH
**Affected files:** `robotx/app/sockets.py`, `robotx/control/controller.py`

**Root cause:** Confirmed — there is no timestamp on inbound `command`/`manual` events, no staleness check, and no watchdog tied to socket disconnect. If the socket disconnects while `_mode == "MANUAL"` mid-forward-command, `_apply_manual()` continues driving that same `_manual_cmd` every tick forever (`controller.py:351-352`) — the mode/command state is never revisited on disconnect.

**Proposed fix:**
- `RobotSocketClient` already clears an `asyncio.Event` on `disconnect` (`sockets.py:46-49`). Wire a callback from that disconnect handler into `RobotController`: on socket disconnect, if `_mode == "MANUAL"`, force `_mode = "STOPPED"` and `motors.stop()` immediately (do not wait for the next control tick's normal path, since that path doesn't currently re-check connection state at all).
- Add a `ROBOTX_COMM_WATCHDOG_TIMEOUT_S` (default e.g. 3.0s) — if no telemetry hook has successfully emitted (i.e., no confirmed live connection) for longer than this, independent of an explicit `disconnect` event firing, force the same safe-stop. This covers a "half-open" TCP connection that never fires `disconnect` cleanly.
- AUTO/RETURN mode is unaffected by disconnect alone (per task: "communication loss" safe-state) — also force-stop, since continuing to autonomously drive without a way to receive a remote STOP is not a state this task's safety posture should default to; document this as the chosen conservative default rather than assumed.

**Safety impact:** Closes a real "runs forever on stale command" gap — directly addresses Phase 8/19 of the task ("Never allow a stale movement command to continue indefinitely").
**Compatibility impact:** A robot currently mid-mission that loses connection will now stop and require an explicit new command after reconnect, rather than continuing blind — an intentional behavior change.
**Testing required:** Unit test: simulate disconnect while in MANUAL/AUTO → assert `motors.stop()` called and mode transitions to a safe state.
**Documentation required:** `ROBOTX_PI_SAFETY_MODEL.md`.

---

## R-07 — No command validation / schema on inbound Socket.IO commands

**Severity:** MEDIUM
**Affected files:** `robotx/app/sockets.py:51-64`, `robotx/control/controller.py:172-212`

**Root cause:** `on_command`/`on_manual` only check `isinstance(data, dict)` (`sockets.py:54`, `60`). `handle_command()` then does ad-hoc, per-field `.get()`/type coercion with no schema, no command ID, no timestamp/freshness check (confirmed — `controller.py:172-212`). A malformed `START` with a non-numeric `destination.lat` will raise inside `float(dest["lat"])` uncaught inside `handle_command`, which is itself called from `RobotSocketClient.command_loop()`'s `try/except Exception` (`sockets.py:113-116`) — so it's logged and doesn't crash the process, but the command is silently dropped with no ack/nack to the sender.

**Proposed fix:** Add a small Pydantic-based command schema (`robotx/app/commands.py`, new) — `STOP`, `START{destination:{lat,lon}}`, `RETURN{home?:{lat,lon}}`, `MANUAL{action, speed, left?, right?}` — validated once at the socket boundary before being queued. Invalid commands are rejected with a logged reason and (once R-02's auth exists) an optional `command_rejected` ack event back to the server. No new command types are added beyond the four already in the protocol, per the task's "only implement commands that are actually needed" instruction.

**Safety impact:** Prevents a malformed command from reaching `handle_command()` in a partially-applied state (e.g., `_mode` set before the destination parse fails).
**Compatibility impact:** Low — existing well-formed commands are unaffected; malformed ones (already effectively no-ops today, just less visibly) are now explicitly rejected.
**Testing required:** Unit tests for each command type's valid/invalid payloads.
**Documentation required:** `ROBOTX_PI_ROBOT_AGENT_PROTOCOL.md`.

---

## R-08 — No process supervision (systemd)

**Severity:** MEDIUM
**Affected files:** none existing; new `deployment/robotx-agent.service`

**Root cause:** Confirmed — no systemd unit in `/etc/systemd/system/`, no Docker/PM2 config anywhere. A crash or reboot requires manual restart.

**Proposed fix:** Add `deployment/robotx-agent.service` (not installed/enabled automatically — a template for the operator to review and install). Runs as user `pi` (not root — GPIO access on recent Raspberry Pi OS is available to the `gpio`/`dialout` group without root), `WorkingDirectory=/home/pi/Desktop/RobotX`, `ExecStart=.../venv/bin/python -m uvicorn robotx.app.main:app --host ... --port ...`, `Restart=on-failure`, `RestartSec=2`, reads `EnvironmentFile=/home/pi/Desktop/RobotX/.env` for configuration (never inline secrets in the unit file).

**Safety impact:** Indirect — ensures the safety-relevant control loop restarts after a crash rather than leaving the robot in an unsupervised state (though motors already fail-safe-stop on exception per the existing `except Exception` handler in `_loop()`).
**Compatibility impact:** None until explicitly installed by the operator (`systemctl enable`) — this plan only adds the file, per the task's instruction not to enable/install services blindly.
**Testing required:** `systemd-analyze verify deployment/robotx-agent.service` (safe, does not start anything).
**Documentation required:** `ROBOTX_PI_SETUP_AND_RUN.md`.

---

## R-09 — Verbose `print()` in hot paths (detection.py, vision_controller.py)

**Severity:** LOW
**Affected files:** `robotx/perception/detection.py` (13 `print()` call sites), `robotx/control/vision_controller.py` (11 `print()` call sites)

**Root cause:** Confirmed via grep. Runs at up to `detection_hz` (2 Hz production, 5 Hz test-path), unbuffered stdout I/O on every detection cycle.

**Proposed fix:** Replace with `logger.debug(...)` calls gated by `ROBOTX_LOG_LEVEL`. Since `vision_controller.py` is being relabeled experimental (R-05) rather than actively maintained, only `detection.py`'s print calls (the ones in the live production path) are in scope for this pass; `vision_controller.py`'s are noted but deferred.

**Safety impact:** None.
**Compatibility impact:** None (log output moves from stdout to the logging module; operators relying on grepping raw stdout need `ROBOTX_LOG_LEVEL=DEBUG`).
**Testing required:** None beyond visual confirmation logging still emits at DEBUG.
**Documentation required:** None beyond a changelog entry.

---

## R-10 — No `.env.example`, secrets handling relies entirely on ambient env vars

**Severity:** LOW
**Affected files:** `robotx/utils/config.py` (fine as-is — already env-var based, no hardcoded secrets found), missing `.env.example`

**Root cause:** Confirmed — no secret values are hardcoded anywhere (verified by grep for API-key-shaped strings). `.gitignore` already excludes `.env`. But there is no `.env.example` template documenting which variables an operator needs to set, including the new `ROBOTX_ROBOT_TOKEN` (R-02).

**Proposed fix:** Add `.env.example` listing every `ROBOTX_*` variable from `config.py` with placeholder/default values and comments, with real secrets never filled in — matching the existing `config.py:33-104` variable set plus `ROBOTX_ROBOT_TOKEN`.

**Safety impact:** None.
**Compatibility impact:** None, additive.
**Testing required:** None.
**Documentation required:** `ROBOTX_PI_SETUP_AND_RUN.md`.

---

## Summary Table

| ID | Finding | Severity | Primary file(s) |
|---|---|---|---|
| R-00 | venv shebangs point at a nonexistent path (beyond just the missing +x bit) | CRITICAL | `venv/bin/*` |
| R-01 | MANUAL mode bypasses obstacle safety | CRITICAL | `controller.py` |
| R-02 | No auth on Socket.IO command channel | CRITICAL | `sockets.py` |
| R-03 | Ultrasonic failure treated as "no obstacle" | CRITICAL | `ultrasonic.py`, `controller.py` |
| R-04 | Battery telemetry is fabricated | HIGH | `controller.py` |
| R-05 | VisionController/DecisionEngine disconnected + zero test coverage | HIGH | `vision_controller.py`, `decision_engine.py`, `controller.py` |
| R-06 | No comm-loss watchdog / stale command handling | HIGH | `sockets.py`, `controller.py` |
| R-07 | No command validation/schema | MEDIUM | `sockets.py`, `controller.py` |
| R-08 | No process supervision | MEDIUM | new `deployment/` |
| R-09 | print() in hot paths | LOW | `detection.py` |
| R-10 | No `.env.example` | LOW | new |

---

## Execution order for implementation (maps to task's own 24-step sequence)

1. R-00 (venv) — unblocks everything else being actually run/verified.
2. R-01, R-03 (centralized `SafetyController`, done together since R-03's status model feeds directly into R-01's gate) — highest safety priority.
3. R-06 (comm-loss watchdog) — depends on R-01's safety gate existing to call into.
4. R-07 (command validation) — independent, can run in parallel with R-06.
5. R-02 (auth abstraction) — client-side only; documented contract for the backend.
6. R-04 (battery representation).
7. R-05 (vision pipeline documentation decision + test suite scaffolding).
8. R-08, R-10 (supervision + config template).
9. R-09 (logging cleanup).
10. Full documentation pass (Phases 21-22) + second audit (Phase 28) + changelog (Phase 29) + implementation status report (Phase 30).

Each step will be bench-tested (mocked hardware, no motor motion) before being marked complete. Any step requiring physical verification (actual motor stop under a real obstacle, actual ultrasonic disconnect) will be called out explicitly as a **manual hardware test required** rather than claimed as verified.
