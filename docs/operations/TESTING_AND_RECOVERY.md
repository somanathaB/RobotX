# Testing, simulated missions and interrupted-mission recovery

What this covers: the repeatable test suites, the end-to-end simulated mission,
the read-only live preflight, and the safety behaviour added with them
(interrupted-mission recovery, applied-motion monitoring, route admission).

**Nothing here is hardware evidence.** Every offline suite runs against
simulated hardware (an in-memory ESP32 firmware model, invented wheel
kinematics, fake GPS and camera). A green run shows the Pi's software does the
right thing with that feedback. It does not show that a wheel turned or a parcel
moved. Every run of the test runner ends by listing those items as NOT TESTED.

## 1. Environment

The runner needs a development venv in the checkout. It is gitignored, and it
must never be the deployed `/opt` one:

```bash
cd /home/pi/Desktop/RobotX-Pi
python3 -m venv --system-site-packages venv
venv/bin/python -m pip install -r requirements.txt
```

## 2. Test runner — `scripts/run_tests.py`

```bash
venv/bin/python -m scripts.run_tests                  # all offline suites (~8 min)
venv/bin/python -m scripts.run_tests unit             # one suite
venv/bin/python -m scripts.run_tests safety esp32     # several
venv/bin/python -m scripts.run_tests --live preflight # read-only report on the running agent
venv/bin/python -m scripts.run_tests --json           # machine-readable
```

| Suite | What it exercises |
|---|---|
| `unit` | every test in `tests/unit` |
| `mission` | OFFER → `assess_offer` → route admission → assignment → navigation → custody → TASK_COMPLETE, including a real Socket.IO link to a simulated backend (`tests/integration/test_simulated_delivery.py`) |
| `navigation` | GPS heading (fix wander is never a heading), navigator, decision layer, progress/stall and sensing-hold bounds, obstacle recovery |
| `telemetry` | the Pi → backend wire: TELEMETRY `status`/`safety`/`readiness`/`faults[]`, transition frames, HEARTBEAT lease, custody and completion reports, the backend's OFFER signature test vector |
| `delivery` | a **genuine signed OFFER** over a real Socket.IO link to a simulated backend and simulated ESP32 (`tests/integration/test_simulated_delivery.py`): admission and replay refusal, delivery to completion, WITHDRAW, PAUSE/RESUME, task cancellation, backend loss, ESP32 reboot, perception loss |
| `safety` | safety gate, e-stop latch, interrupted-mission recovery, obstacle and navigation stops, operator-route guards |
| `esp32` | protocol framing, link proof/re-proof, ACK timeouts, DRIVE ACK GATED/REJECTED, zero applied output, using fake transports and the simulated firmware model |
| `injection` | guarded test injection (unit, real agent process, systemd sandbox) **and** `scripts.simulated_mission`, all scenarios |
| `integration` | every test in `tests/integration` |
| `static` | byte-compiles everything; mypy must be clean on the new modules. The whole package has 63 mypy findings from before this work; they are reported as a count only |
| `preflight` | live, read-only; requires `--live` |

| Status | Meaning |
|---|---|
| PASS | the suite ran and nothing failed (skips are counted) |
| FAIL | a test failed or errored, or the suite could not run. For `preflight` it means the live rover is not READY for a motion test, which is a readiness verdict and not a software failure |
| SKIP | nothing ran: every test was skipped, or (preflight) no agent is answering |
| NOT TESTED | this runner did not exercise it at all. It is never a pass |

Expected skips: `tests/integration/test_esp32_sim_link.py` needs the ESP32
repository's host simulator (`ROBOTX_ESP32_SIMULATOR_EXE`), and this Pi has no
toolchain for it. `test_injection_sandbox` needs a systemd user manager.

## 3. A simulated test mission, end to end

```bash
venv/bin/python -m scripts.simulated_mission                       # all scenarios (~80 s)
venv/bin/python -m scripts.simulated_mission --scenario estop --verbose
```

The script drives one TEST-INJECTED mission per scenario through the real
`POST /test/mission/inject` handler. The handler is called in-process, with the
token, loopback and provenance checks all applied. The mission then goes through
the real agent loop, admission, navigation, safety gate and ESP32 link protocol.
Custody is recorded through `POST /mission/custody`, and stopping goes through
`/safety/estop`, `/safety/clear` and `/mission/resume`.

| Scenario | Expectation |
|---|---|
| `nominal` | admitted → pickup → custody ACQUIRED → drop → RELEASED → COMPLETE; no reverse or pivot DRIVE |
| `estop` | e-stop mid drop leg → STOPPED, mission held and marked interrupted, no DRIVE, lease withheld, RESUME refused (409); clear → PAUSED with no DRIVE; explicit RESUME → AUTO → COMPLETE |
| `reboot` | simulated ESP32 reboot mid-leg: the same as `estop`, plus the link re-proving itself with a PING |
| `obstacle` | camera obstacle → no DRIVE while in view; leg resumes after the clear hold; COMPLETE |
| `applied-zero` | firmware applies 0 to every DRIVE → PAUSED `controller: DRIVE … applies 0/0` within ~1.5 s |
| `gated` | firmware GATEs every DRIVE → PAUSED `controller: ESP32 GATED every DRIVE …` |
| `route-far` | route starts 150 m from the rover → 409 `ROUTE_START_TOO_FAR`, nothing assigned |

What it never does: open a serial port, bind a socket, contact a backend, read
`/etc/robotx`, or touch the running service. Settings are an explicit
dictionary, and the injection log goes to a temporary directory. The engine-mode
path (a signed OFFER over a real Socket.IO link to a simulated backend) is
covered by `tests/integration/test_simulated_delivery.py` in the `mission`
suite.

Injection is unchanged: it is off by default, needs the token and loopback
checks, gets no bypass, and never clears a latch. For the physical procedure, see
[TEST_MISSION_INJECTION.md](TEST_MISSION_INJECTION.md).

## 4. Read-only live preflight

```bash
venv/bin/python -m tests.mission_injection.preflight          # human-readable
venv/bin/python -m tests.mission_injection.preflight --json
```

The preflight only does `GET /health /state /config /backend /readiness
/test/injection` on 127.0.0.1, plus `systemctl is-active`, `timedatectl`,
`vcgencmd get_throttled` and `git rev-parse` of `/opt/robotx/RobotX-Pi`. It never
POSTs, opens a port, clears a latch or changes a service. It reports:

- **Pi health:** temperature, load, memory and disk, plus under-voltage or
  throttling now and since boot.
- **Links:** backend destination, AUTH and streaming; ESP32 link state and
  TELEMETRY age.
- **Latches:** the ESP32 reboot latch and the e-stop.
- **Motion:** motion enabled and `motion_ready`; motor drive and PCA status.
- **Sensing:** front and rear sensor faults, the ESP32 obstacle gates, and
  localization accuracy.
- **Missions:** the mission state (including `interrupted_by`), the agent's own
  readiness blockers (`/readiness`), and the last DRIVE ACK and applied output.

`/readiness` is new in this branch. An agent that predates it answers 404, and
that check is then NOT TESTED.

## 5. Interrupted-mission recovery

This fixes the following defect. An e-stop during a mission (an operator's, or
the latch an ESP32 reboot engages) left the mode STOPPED with the mission still
active. A backend RESUME was refused (it needs PAUSED), HEARTBEAT kept renewing
the mission lease, and `/mission/idle` could put the rover IDLE with an active
mission and route still loaded.

| From | Event | To |
|---|---|---|
| AUTO / PAUSED with a mission | e-stop or ESP32 reboot | **STOPPED**; mission kept, `interrupted_by` set; no motion; **lease not renewed** |
| STOPPED (latched) | RESUME, RETURN, `/mission/idle` | refused; mode stays STOPPED |
| STOPPED (latched) | PAUSE | mode stays STOPPED (would otherwise let a RESUME reach AUTO under the latch) |
| STOPPED (latched), mission held | operator clear (`/safety/clear`) | **PAUSED**, never AUTO; lease renewed again; reboot acknowledged, link must re-prove |
| PAUSED | explicit RESUME | AUTO; `interrupted_by` cleared; every readiness and safety gate still applies |
| STOPPED / PAUSED | STOP | mission ABORTED; then `/mission/idle` is allowed |
| STOPPED, no mission | operator clear | STOPPED (unchanged) |
| any | `/mission/idle` while a mission is held | 409; stop it or resume it |

Policy decisions:

- **No lease renewal while the rover cannot deliver.** HEARTBEAT renews the
  commitment lease only in AUTO or PAUSED and with no e-stop latched
  (`RobotAgent.mission_lease_renewable`). STOPPED under a latch, and ERROR, both
  send the plain `{}` heartbeat. If the operator does not clear within the
  backend's lease window (60 s), the backend recovers the Leg. That is the
  honest outcome for a rover that needs a human first. A later RESUME naming a
  released commitment is refused by the existing commitment binding.
- **Clearing never resumes.** Clearing only moves the mission to PAUSED, so an
  explicit RESUME is still the act that allows motion. This keeps the existing
  rule that the operator who clears a fault is never the reason the robot began
  to drive.
- **TELEMETRY `status` is unchanged** (STOPPED is still reported as IDLE).
  `readiness.blockers` (`ESTOP_LATCHED`) and `safety.stopLatch` already tell the
  backend the rover is latched. Changing the backend-facing status vocabulary is
  left to a backend-coordinated change.

## 6. Applied-motion monitor (`robotx/control/applied_motion.py`)

| Condition | Action |
|---|---|
| DRIVE ACK `REJECTED` (not `STALE_SEQ`, which is the protocol resyncing) | pause at once |
| DRIVE ACKs `GATED` with no `ACCEPTED` between, for `ROBOTX_MOTION_APPLIED_TIMEOUT_S` | pause |
| fresh TELEMETRY with `applied` 0 on every side driven at or beyond the deadband, for the timeout and ≥ 2 distinct frames | pause |

The reason is recorded as `controller: …` in `last_error`. Resuming is an
explicit RESUME, and a fault that persists pauses the mission again.

Each common false alarm is guarded against:

| Case | How it is handled |
|---|---|
| STOP commands | the monitor runs only while the last command written was a DRIVE |
| Pauses and idle | the monitor runs only in AUTO, with the previous tick's gated intent moving |
| Motion disabled | the link writes nothing, so the monitor never runs |
| A DRIVE inside the ±40 deadband | that side is not judged |
| Stale or missing TELEMETRY | the timer neither advances nor resets |

Limitation: `applied` is the PWM the firmware outputs, not wheel motion. A
stalled motor or a slipping wheel is still caught only by the progress monitor
(60 s of driving with no progress).

## 7. Route admission (`robotx/mission/admission.py`)

`Mission.create` already refuses non-finite, out-of-range, malformed, empty and
over-long routes. On top of that, every OFFER and every test injection must
satisfy these checks:

| Check | Default | Refusal |
|---|---|---|
| each path end meets its stop (pathToPickup end, pathToDrop start and end) | 50 m | `UNSUPPORTED_MISSION: ROUTE_INCONSISTENT` |
| pathToPickup starts at the rover | 25 m | `UNSUPPORTED_MISSION: ROUTE_START_TOO_FAR` |
| pickup is within reach (straight line) | 300 m | `UNSUPPORTED_MISSION: PICKUP_TOO_FAR` |

The start check closes a real gap. The route planner measures the first segment
from wherever the rover stands, so a route that started far away was driven as
one unchecked straight line. The limits are configurable within bounds (see
`.env.example`), no route point is changed, and no campus boundary is assumed.
The readiness checks (e-stop, motor link, position fix, custody) are still
evaluated first.

## 8. The genuine OFFER path, end to end (what runs, what is still untested)

```
OFFER (signed envelope, event name = command)          BackendLink._on_engine_command
  -> addressee, notValidAfter, HMAC-SHA256 signature   engine.parse_envelope / verify_signature
  -> fence, sequence; persist high-water marks          check_fence / check_sequence / CommitmentStore
  -> COMMAND_ACK {outboxId, fence, authorityEpoch}
  -> RobotAgent.assess_offer: paths, route consistency, readiness blockers,
     perception, start position                          -> OFFER_ACCEPT | OFFER_REJECT {reason}
  -> RobotAgent.assign_mission(custody_required=True)  -> MissionManager -> Navigator
tick: GPS -> position (heading only from driven movement) -> ESP32 status -> backend loss policy
  -> navigation -> mission progress -> perception -> decision -> safety gate
  -> obstacle / progress / sensing-hold bounds -> Esp32Link.submit -> DRIVE | STOP
feedback: ESP32 ACK + TELEMETRY applied output -> applied-motion monitor -> pause on disagreement
custody: operator-confirmed ACQUIRED at a measured pickup arrival, RELEASED at the drop -> CUSTODY_EVENT
completion: COMPLETE only after RELEASED -> TASK_COMPLETE (Pi-side evidence check) -> TASK_COMPLETE_ACK
```

`venv/bin/python -m scripts.run_tests delivery` drives that whole path with the
production code. The backend (`ContractServer`) and the ESP32/wheels/GPS/camera
(`tests/fixtures/sim_rover.py`) are simulated. The OFFERs are signed with a
test key; the canonical form itself is pinned to the backend's own vector
(`tests/fixtures/backend_offer_signature_vector.json`, from
RobotX-Admin-Dashboard `docs/contracts/fixtures/`).

### TELEMETRY additions (contract section 4; the backend accepts and ignores all three today)

- `status`: `PAUSED` for a Rover STOPPED with a mission still held (e-stop or
  ESP32 reboot mid-mission) instead of `IDLE`. The backend records a unit's
  own PAUSED as an operator hold: after RESUME, an operator lifts it with
  `POST /api/robots/:id/clear-fault`.
- `safety`: only while the ESP32's TELEMETRY is fresh (otherwise omitted, as
  `stopLatch` always was): `estop {present:false}`, `observedAtMs`,
  `safetyStop {active, latched, reason}` (the proximity stop, never an
  e-stop), `motorDriveAvailable`, `commandTimeout`, `stopLatch`.
- `faults[]`: the complete active set, `{code, severity, component, active,
  sinceMs}` in the contract's vocabulary (`robotx/diagnostics/faults.py`);
  `[]` = measured, none; omitted when the agent loop has stalled. The same set
  is on `GET /readiness`.
- A frame is sent at once (at most every 0.5 s) whenever `status`, `safety`,
  `readiness` or the active fault codes change.

### Bounded stops added

- Unusable perception refuses an offer (`PERCEPTION_UNAVAILABLE: <status>`):
  a per-Leg refusal, because the backend's readiness vocabulary has no code
  for it. `faults[]` names it.
- Unusable perception, or a forward range sensor that is not reporting, while
  there is a waypoint to drive to pauses the mission after
  `ROBOTX_NAV_HOLD_ASSIST_AFTER_S` (default 30 s) with the reason.

### Still NOT TESTED (no simulation can show these)

Physical wheel actuation; the real ESP32 answering DRIVE with non-zero applied
output; a real RobotX OFFER and its lease; GPS outdoors on the rover; a
physical e-stop (none exists). The runner prints this list on every run.
