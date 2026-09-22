# RobotX Pi — Production-Readiness Audit Report

**Date:** 2026-09-22
**Scope:** Raspberry Pi 5 only. No ESP32, no motors, no backend, no GPS wiring touched.
**Result:** 7 defects found and fixed, one of them a genuine safety hole. 201
automated tests pass. GPS remains blocked on hardware.

---

## 1. What was inspected

Every Pi responsibility named in the brief, read with fresh eyes rather than
re-running the previous pass's checks:

- `application/` — agent lifecycle, tick sequence, FastAPI lifespan, HTTP surface
- `perception/` — result model, pipeline threading, detector, failure statuses
- `control/` — decision branches, motion intent, the retained legacy loop
- `localization/`, `navigation/` — estimator, planner, navigator
- `state/`, `diagnostics/` — authority, snapshots, telemetry schema, health
- `hardware/` — camera manager refcounting, GPS reader, battery availability
- `config/` — settings parsing, secret handling, logging setup
- Dependency graph, exception handling, dead code, thread safety
- The existing test suite, audited for tests that assert too little

Two things were checked by *running* rather than reading: the live agent under
uvicorn against real hardware, and the dependency graph via AST analysis.

---

## 2. Defects found and fixed

### 2.1 SAFETY — perception without frame metadata bypassed the obstacle check

**Severity: high.** `PerceptionResult.is_usable` returned `True` for any `OK`
status, regardless of whether frame metadata was present. The decision layer's
corridor check then silently no-opped:

```python
def _blocking_obstacle(self, perception):
    if perception.frame is None:
        return None          # <-- no obstacle reported, whatever is in view
```

So a result with `status=OK`, a 40,000 px obstacle dead centre, and
`frame=None` fell through to route-following. Reproduced before fixing:

```
perception.is_usable : True
detections           : 1 largest 40000px
frame metadata       : None
RESULTING INTENT     : FORWARD | on course (cautious)     <-- drives into it
```

**Fix:** `is_usable` now requires frame metadata as well as an `OK` status —
without the frame's dimensions the geometric check cannot run, so the result is
not usable for decisions. The decision layer reports the distinct reason
`perception unavailable (no frame metadata)`.

**After:**
```
AFTER fix: STOP | perception unavailable (no frame metadata)
```

**Why it mattered despite the pipeline always setting `frame`:** the guarantee
lived only in the pipeline's implementation, not in the type. Any other producer
of a `PerceptionResult` — a future sensor, a replay harness, a test — would have
silently inherited the hole. There is now a test asserting the pipeline's
guarantee *and* a test asserting the type is safe without it.

### 2.2 Health evaluation mixed four different snapshots

`_component_health()` called `self.state.snapshot()` separately in each of four
component methods. Beyond the redundant lock traffic, a single health report
could combine camera state from one instant with GPS state from another — a
report describing a robot state that never existed. Now one snapshot is taken
and passed down.

### 2.3 Health lookups could crash the agent tick

`mapping[status]` on three status enums. Adding a new enum member would raise
`KeyError` inside the tick, which the loop catches by forcing `ERROR` mode and a
stop intent — i.e. extending an enum could halt the robot. Now `.get()` with an
`UNKNOWN` fallback, and the maps are module-level constants.

### 2.4 CameraStream reported the requested resolution, not the actual one

libcamera permits one configuration per process. A second `CameraStream`
requesting a different size receives the first one's frames but reported its
own requested dimensions through `.width`, `.height` and `describe()`. Decisions
were unaffected (perception reads dimensions from the frame array), but
telemetry and diagnostics would have lied. Now reports the device's config once
attached.

### 2.5 Unguarded read-modify-write in the perception pipeline

`_prev_largest_area` was mutated outside the lock. Reachable whenever a bench
script calls `step_once()` while the pipeline thread is running. Now guarded.

### 2.6 `/camera` hung forever on a silent camera

The MJPEG generator looped indefinitely when `get_jpeg()` returned `None`,
holding the connection open with no data — indistinguishable to a client from a
perfectly static scene. Now closes the stream after ~5 s of empty polls and logs
`camera.stream_ended`.

### 2.7 Dead code

`RobotState.record_error()` had no callers anywhere. Removed.

---

## 3. What was NOT changed, and why

- **GPS implementation** — untouched, as instructed. No software fault exists:
  the port opens, the reader runs, and it reports `NO_FIX` honestly.
- **Legacy direct-drive controller** — still retained, still unwired.
- **Experimental perception pipeline** — still quarantined.
- **Socket.IO client** — still not constructed by the agent.
- **ESP32 / backend / motors / wiring** — out of scope this pass.

---

## 4. Test coverage added

| Area | Tests | What it locks down |
|---|---|---|
| `test_audit_regressions.py` | 15 | One per defect above, so none can silently return |
| `test_navigation_synthetic.py` | 16 | Full localization → navigation → decision chain on deterministic NMEA fixtures |
| `test_agent.py` (new class) | 7 | Mid-run degradation: each subsystem failing *while driving* |
| `test_motion_and_decision.py` | +1, 1 strengthened | Replaced a test that only asserted `intent is not None` |

The synthetic navigation fixtures generate real NMEA sentences with correct
checksums and feed them through the **real** parser, estimator, navigator and
decision maker. Only the serial port is substituted. The fixtures are themselves
tested (they must parse back to the input coordinates), so the tests built on
them are not vacuous. A determinism test asserts two identical runs agree.

The degradation tests deliberately drive a good tick first and assert the robot
is moving, then inject a fault and assert the *next* tick stops — the dangerous
case is a robot that is already in motion when it goes blind.

---

## 5. Tests executed — exact results

```
$ venv/bin/python -m unittest discover -s tests/unit -t .
Ran 201 tests in 0.684s
OK
```

| File | Tests |
|---|---|
| `test_agent.py` | 28 |
| `test_motion_and_decision.py` | 38 |
| `test_gps_and_position.py` | 27 |
| `test_perception.py` | 25 |
| `test_state_telemetry_health.py` | 25 |
| `test_navigation.py` | 18 |
| `test_navigation_synthetic.py` | 16 |
| `test_audit_regressions.py` | 15 |
| `test_config.py` | 9 |
| **Total** | **201** |

### Static and structural checks

| Check | Result |
|---|---|
| `compileall robotx tests` | PASS |
| All modules import | PASS — 40 modules, 0 failures |
| Dependency cycles | PASS — none at module or package level |
| `print()` in production code | PASS — 0 |
| Silent exception swallowing | PASS — none in the agent path |
| `RPi.GPIO` loaded by agent | PASS — False |
| Motor / encoder / ultrasonic / IR modules loaded | PASS — all False |
| `socketio` / socket client loaded | PASS — False |
| `serial` loaded by agent | PASS — False (GPS disabled in that check) |

### Live hardware re-verification (after the changes)

| Check | Result |
|---|---|
| Agent under uvicorn | Camera HEALTHY, perception HEALTHY, overall DEGRADED (GPS) |
| Perception on live frames | `OK`, frame 640×480, detections 0 |
| `/camera` MJPEG | Valid multipart JPEG stream |
| Mission with no GPS | `AUTO` → intent `STOP \| no GPS position` |
| Battery in telemetry | `UNAVAILABLE` / `null` |
| Shutdown | perception → GPS → camera, in order; no leftover processes |

No motor was energized at any point.

---

## 6. Remaining blockers

**GPS produces zero NMEA bytes.** Re-confirmed after the module was powered and
its antenna reconnected: 0 bytes across `/dev/ttyAMA0` and `/dev/ttyAMA10` at
six baud rates, no USB serial device, no I²C device on the header bus.

Evidence the Pi side is not at fault:
- GPIO 14 (Pi TX) measured **788 high / 712 low** while transmitting — the UART
  works and drives its pin.
- GPIO 15 (Pi RX) measured **high on 2,000/2,000 samples** — idle, nothing
  driving it.
- Pins correctly muxed (`a4` = `TXD0`/`RXD0`); `dtparam=uart0=on` set; no serial
  console holding the port.

A powered GPS transmits NMEA with or without an antenna (fields empty until it
locks), so the antenna is not the variable. **This is a wiring or module-power
fault on the GPS side.**

Consequence: **navigation is not physically validated and must not be described
as such.** Everything downstream of GPS carries PASS-SYNTHETIC at best.

---

## 7. What must be physically tested later

| Item | Needs |
|---|---|
| GPS fix acquisition | A transmitting module; then outdoor sky view |
| Real position, heading, route following | A GPS fix, then a moving robot |
| Camera aim, focus, colour correctness | Light, and a known target |
| Obstacle detection at real distance | Light, a real object, a tape measure |
| The ~90×90 px detection floor in centimetres | As above |
| Motor response to MotionIntent | ESP32 integration (later pass) |
| Multi-hour endurance and thermal behaviour | A long run |

The camera measurements taken so far were all at **0.46 lux** — below full
moonlight. Every optical conclusion is provisional until re-measured in light.

---

## 8. What is ready for ESP32 integration

The Pi-side contract is stable and, as of this audit, defended by tests:

- **`MotionIntent`** — `command`, `left`, `right`, `reason`, `timestamp`.
  Velocities normalized to [-1, 1], never PWM or duty cycle. Round-trips
  through `to_dict()`/`from_dict()`. `timestamp` exists so the ESP32 can refuse
  a stale intent.
- **The publication point** — `RobotAgent.tick()` writes the intent into
  `RobotState` every cycle; a transport reads
  `agent.state.snapshot().motion_intent`.
- **Link status reporting** — `RobotState.update_communication(esp32=...)`,
  currently `NOT_IMPLEMENTED`.
- **Verified guarantees:** the agent holds no motor driver, never loads
  `RPi.GPIO`, stops synchronously on command (no stale-intent window), and
  publishes a stop intent for every known failure mode.

**Not ready, and deliberately absent:** UART code, protocol framing, baud
negotiation, handshake, acknowledgements. That is the next pass.

The ESP32 side must still own: converting normalized velocity to PWM, the duty
ceiling, E-stop, ultrasonic/IR reflexes, encoder feedback, and refusing stale or
implausible intents. The Pi cannot stop the robot; it can only decline to ask
for motion.

---

## 9. Honest summary

The Pi agent is in good shape as a standalone system: it starts, runs, degrades
safely, cleans up after itself, and reports its own state without inventing
values. The audit found one real safety hole and six smaller defects, all fixed
and regression-tested.

What it is **not** is a validated navigation system. No GPS fix has ever been
acquired on this robot, no wheel has turned, and every camera measurement was
taken in the dark. The software is ready for those tests; the tests have not
happened.
