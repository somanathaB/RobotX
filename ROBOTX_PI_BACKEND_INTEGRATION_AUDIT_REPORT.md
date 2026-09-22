# RobotX Pi ↔ FalconAut backend integration — audit, implementation and test report

**Date:** 2026-09-22 · **Scope:** Raspberry Pi 5 agent only · **Motors:** never engaged

---

## 1. Executive summary

The Pi side of the backend integration is **implemented and tested against a
real Socket.IO transport**. It is **not integrated with FalconAut**, and cannot
be, because the backend does not exist in this environment.

### The central finding

**Neither the FalconAut backend source nor `schema.prisma` is present on this
machine.** Verified by filesystem-wide search across `/home`, `/opt`, `/srv`
and `/var/www`: no backend tree, no Prisma schema file, no robot simulator, and
nothing serving Socket.IO on any port. The only `FalconAut` mentions anywhere
are in this repository's own prose.

That has one consequence, and it shapes everything below:

| Class of fact | Available? | How it was handled |
|---|---|---|
| **Data model** — field names, enum values (`robotId`, `lat`, `lon`, `speed`, `battery`, `STOP`/`PAUSE`/`RETURN`/`RESUME`, `SENT`/`ACK`/`FAILED`, `INFO`/`WARNING`/`CRITICAL`) | Yes — enumerated in the task specification | Used **verbatim**, including camelCase |
| **Socket.IO envelope** — event names, namespace, handshake shape, ack mechanism | **No** | **Not invented.** Made configuration, defaulted to this repo's pre-existing names, and marked `PROVISIONAL` |

No backend event or payload was fabricated. Every transport-level name is a
`ProtocolBinding` value loadable from `ROBOTX_PROTOCOL_FILE`, so the real
contract plugs in with **no Python change**. Until one is supplied, the link
reports `integrated: false` and the communication health component reads
`DEGRADED`, however healthy the socket is.

### What was verified, on real hardware

- **370 automated tests pass** (345 unit + 25 integration), up from 201.
- The integration suite drives the **real `socketio.AsyncClient`** against a
  real `socketio.AsyncServer` — genuine Engine.IO handshake, genuine JSON.
- A **live process run** (uvicorn + agent + link) round-tripped a command and
  returned an `ACK`.
- A **120 s soak with the real Camera Module 3** measured the link's cost at
  **0.0 CPU points and +0.04 ms of loop period**.

### What is blocked

Nine backend-side facts remain unknown (§25). The load-bearing ones: the real
event names (**B-1**), whether the backend validates the token at all
(**B-7**), and whether `battery` is nullable (**B-4** — the Pi always sends
`null` and will not send a number).

### Production readiness

**NOT PRODUCTION INTEGRATED.** The Pi side is production-*ready* pending
B-1/B-2; the system is not integrated until a backend is available to verify
against. Detail in §27.

---

## 2. Previous Pi architecture

Prior work left a modular standalone agent: `application`, `communication`,
`config`, `control`, `diagnostics`, `hardware`, `localization`, `navigation`,
`perception`, `state`. It ran without motors, without an ESP32 and without a
backend — 201 unit tests, all passing.

The backend boundary was a stub. `communication/socket_client.py` (164 lines)
was **imported by nothing**, explicitly documented as a future boundary. Its
event names (`robot_hello`, `command`, `manual`, `telemetry`, `status`) were
inherited from the original unaudited codebase and had **never been checked
against any server** — the prior remediation plan (R-02) says so directly.

It also carried defects that the prior audit had catalogued but not fixed:
no command schema or validation (R-07), no command IDs, no acknowledgement of
any kind, no staleness check, no comm-loss watchdog (R-06), and a `manual`
event that took arbitrary motion commands straight from the wire.

### Answers to the five audit questions

**What does the Pi currently know?** Operating mode; GPS fix with explicit
validity and age; position and a GPS-derived heading (no compass or IMU
exists); navigation progress; perception results; its own `MotionIntent`;
host health. It does **not** know battery charge (no sensing hardware),
wheel odometry, or true orientation.

**What does the backend expect?** At the data-model level: `Telemetry`
(`robotId`, `lat`, `lon`, `speed`, `battery`), `Robot` status/battery/online
fields, `Command` with type and status, `Event` with a level and message. At
the wire level: **unknown, and not determinable from anything available.**

**What could the Pi send?** Nothing. Telemetry was local only; the client was
never constructed.

**What could the Pi receive?** Nothing. Had the client been wired up, it would
have accepted any `command` or `manual` dict with no validation, no ID and no
acknowledgement.

**What was missing?** The entire link: connection management, registration,
rate control, command validation, acknowledgement, idempotency, reconnection,
link-state truthfulness, and a protocol definition that was not a guess
masquerading as a contract.

---

## 3. Current Pi architecture

```
   camera ─► perception ─┐
                         ├─► RobotState ──► RobotSnapshot ──┐
   GPS ─► localization ──┤        ▲                         │
                         │        │                         ▼
   navigation ───────────┤        │                  BackendLink ──► Socket.IO
                         │        │                    ▲     │
   decision ─► MotionIntent       │                    │     ▼
                                  └── mission methods ◄── CommandExecutor
                                  (stop/pause/resume/return)
```

`BackendLink` reads an immutable snapshot and calls four mission methods. That
is the **entire** coupling. Enforced by a test that walks the AST of every
module in `robotx/communication/`: no `RPi`, `gpiozero`, `serial`, `picamera2`,
`cv2`, no `robotx.hardware.*`, no `robotx.perception.*`.

That test earned its place immediately — it caught `protocol.py` importing
`GPSStatus` from `robotx.hardware.gps` merely to name an enum value. Replaced
with `GpsReading.has_fix`.

**ESP32 readiness:** unchanged and untouched. The seam is still `MotionIntent`,
and the backend link has no path to it. The two transports are independent by
construction: neither knows the other exists.

---

## 4. Backend schema analysis — what belongs on the wire

Not every column should be transmitted. The division:

| Model / field | On the wire? | Reasoning |
|---|---|---|
| `Telemetry.robotId/lat/lon/speed/battery` | **Sent** | This is the high-frequency payload, and it is all of it |
| `Telemetry.createdAt` | Not sent | Server clock orders rows across robots. The Pi sends `capturedAt` separately so delay is detectable |
| `Robot.status` | **Sent** (status channel) | The agent's operating mode; only the Pi knows it |
| `Robot.battery` | **Sent as `null`** | No sensing hardware exists (§ B-4) |
| `Robot.simulated` | **Sent once**, at registration | `false`. A fact about what kind of agent this process is, known here and nowhere else |
| `Robot.isOnline` | **Never sent** | Derived from the connection the server holds. A robot asserting its own liveness can lie about being alive |
| `Robot.socketId` | **Never sent** | Only the server knows which socket it is holding |
| `Robot.lastSeenAt` | **Never sent** | Server clock; a robot's may be wrong or spoofed |
| `Robot.id`, `locationId`, `campusId`, `zoneId` | **Never sent** | Database keys and fleet assignment, not robot knowledge |
| `Robot.currentTaskId` | **Never sent** | Backend-assigned |
| `Command.type/status/executedAt` | **Sent** (ack) | The robot is the authority on what it did and when |
| `Command.issuedAt` | Received only | Used for the staleness check |
| `Event.robotId/type/message/taskId` | **Sent**, event-driven | `taskId` only when the Pi actually holds one |
| `Decision` (`WAIT`/`REROUTE`/`CANCEL`, `imageUrl`) | **NOT IMPLEMENTED** | Requires an image-upload path and a decision-request protocol, neither of which is derivable. The Pi has the perception data to populate it; the contract is missing |
| `Task` / `Mission` / `Leg` / `Agent` | **NOT IMPLEMENTED** | No wire protocol available (§ B-9) |

---

## 5. Actual Socket.IO protocol discovered — **BLOCKED**

**None.** No backend source, no simulator, no server documentation, no running
service. The audit searched the whole filesystem for `socket.io`, `io.on`,
`socket.emit`, namespaces, middleware, room names, robot registration, Redis,
and a simulated-robot implementation. Nothing outside this repository's own
client and its prose.

**No second robot protocol was created**, because there is no first one to
conflict with. The provisional binding reuses the names already in this repo so
that no *new* invention entered the codebase:

| Direction | Binding field | Provisional default |
|---|---|---|
| Pi → backend | `register` | `robot_hello` |
| Pi → backend | `telemetry` | `telemetry` |
| Pi → backend | `status` | `status` |
| Pi → backend | `event` | `event` |
| Pi → backend | `command_result` | `command_ack` |
| backend → Pi | `command` | `command` |
| backend → Pi | `registered` | `registered` |
| — | `namespace` | `/robot` |

The legacy `manual` event was **deliberately dropped**: it accepted arbitrary
motion commands off the wire, which is incompatible with the required safety
chain, and it corresponds to nothing in the backend command enum.

Supplying the real contract is a JSON file and an environment variable —
see `docs/communication/ROBOT_BACKEND_PROTOCOL.md` §14.

**Status: BLOCKED (B-1).**

---

## 6. Pi ↔ backend data flow

**Pi → backend**

```
camera ─► perception ─┐
GPS ─► localization ──┼─► RobotState ─► RobotSnapshot ─► protocol.py payload builders
navigation ───────────┤                                          │
decision ─► intent ───┘                        rate control in BackendLink
                                                                 │
                    telemetry 1 Hz ── status 0.2 Hz ── event (deduped) ── ack (per command)
                                                                 ▼
                                                            Socket.IO
```

**Backend → Pi**

```
command ─► parse_command (schema, robotId, age, size)
             │                     │
        rejection              InboundCommand
             │                     │
        FAILED ack          CommandExecutor (idempotent per commandId)
                                   │
                          agent mission method
                                   │
                    mode change ─► decision layer (safety still applies)
                                   │
                            MotionIntent ─► [future ESP32] ─► motors
                                   │
                              ACK / FAILED ─► backend
```

No internal Python object is ever serialized onto the wire. Payloads are built
explicitly in `protocol.py`, which is the only module that knows the wire
format.

---

## 7. Authentication flow — **PARTIAL (client-side only)**

```
Pi ── Engine.IO handshake, auth={"robotId": ..., "token": ...} ──► backend
                                                                     │
                                          accept ◄──────── verify ───┤
                                          refuse ◄──────────────────┘
```

The credential travels in the connect handshake, delivered before the server
processes any event — the only point at which a connection can be refused
outright. It is never repeated in an event payload, where a server's event log
would capture it.

**Honest status: client-side credential transport exists, but the channel is
not authenticated.** Nothing in this repository verifies the token, and no
backend that does was available. A token nobody checks provides no security.

The Pi handles refusal correctly and this **was** verified — against a real
server that genuinely refuses (§21): the link enters `REJECTED`, backs off for
60 s rather than hammering, and the agent keeps running.

**Finding (verified, python-socketio 5.11.4):** the server's rejection *reason*
never reaches the client. The client raises
`ConnectionError("One or more namespaces failed to connect")` and the namespace
`connect_error` handler is **not invoked**. The Pi can know it was refused, not
why. Documented; a backend needing to convey a reason must send it as an event
after accepting.

**No credential appears in any log line, payload, or API response.** `/config`
and `/backend` report `SET`/`UNSET` only, `redact()` scrubs credential-shaped
keys from everything loggable, embedded URL credentials are stripped, and the
library's own payload logging is disabled for exactly this reason. Asserted by
test.

---

## 8. Robot identity flow

`ROBOTX_ROBOT_ID` → handshake `auth.robotId` → `register` payload → every
outbound payload → checked against every inbound command's `robotId`.

A command naming a different robot is **refused**. Routing is the server's job,
but a robot that executes anything it receives is one misconfigured room away
from obeying another robot's stop.

`simulated: false` is asserted at registration. Registration is **repeated on
every reconnect**, because the server assigns a new `socketId` each time and
the old association is void — verified by test.

---

## 9. Telemetry flow — four channels, four rates

| Channel | Rate | Size (measured) | Content |
|---|---|---|---|
| Core telemetry | **1 Hz** | 154 B | `robotId`, `lat`, `lon`, `speed`, `battery`, `capturedAt`, `positionAgeS` |
| Health/status | **0.2 Hz** | 782 B | mode, health, subsystem statuses, last-known position with age, motion intent, navigation progress |
| Event | Event-driven, 30 s dedupe, ≤1/s | ~150 B | `INFO`/`WARNING`/`CRITICAL` + message |
| Diagnostic | **Not sent** | — | CPU, memory, temperature, detection details — local HTTP only |

**Rate justification.** Camera 20 FPS, perception ~2 Hz, telemetry 1 Hz. The Pi
does **not** emit at camera rate: twenty database rows per second per robot is
not a dashboard. **B-3** — whether each event writes a `Telemetry` row — is
unresolved, so 1 Hz is the conservative choice. A smoother live map should come
from a separate realtime path, not from raising this number.

### Refusal to fabricate

- **No fix, no frame.** If GPS status is not `FIX`, or the position is older
  than 5 s, **no telemetry is sent**. The last fix is never resent as current.
- **`battery` is always `null`.** No fuel gauge, no ADC, no divider.
- **`speed` is `null`, not `0.0`**, when unreported. "Not measured" and
  "stationary" are different claims.
- Stale positions appear **only** on the status channel, tagged `ageS` and
  `isFresh: false`, so a dashboard can grey out a last-known marker.

Observed live: during the 120 s soak, `telemetry_sent: 0`, `telemetry_skipped:
125`, `status_sent: 25`. The GPS had no fix indoors and the Pi correctly sent
no position at all while remaining connected and reporting its mode.

---

## 10. Command flow

`command` → `parse_command` (type, ID, `robotId`, age, 64 KiB cap) →
`CommandExecutor.execute` (idempotency → mission method) → ack.

| Command | Effect | Refused when |
|---|---|---|
| `STOP` | Route cleared, mode → `STOPPED`, stop intent | **Never** — a stop must work from every state, including `ERROR` |
| `PAUSE` | Mode → `PAUSED`, **route retained**, stop intent | Never |
| `RESUME` | Mode → `AUTO` | Not currently `PAUSED`, or no retained route |
| `RETURN` | Route to home, mode → `AUTO` | **No home position known** |

`PAUSE` vs `STOP` is deliberate: `STOP` ends the run, `PAUSE` keeps the route so
`RESUME` has something to return to. Collapsing them would make `RESUME`
meaningless, and the backend enum clearly intends the pair to work together.

`RETURN` uses `ROBOTX_HOME_LAT`/`LON` if configured, else the GPS position
recorded at mission start. **With neither it is refused** — there is no depot
coordinate in this robot's configuration, and driving to an invented
destination is the worst thing this command could do.

### Safety (Phase 8) — **PASS**

A command changes *mission state*, nothing else. The executor's target
interface is four methods wide and a test asserts it contains no method through
which a speed, a steering value or a pin could be set.

After a backend `RESUME`, the robot still stops for: a detected person, a
blocking obstacle, unusable/stale perception, and a lost GPS fix. Each is a
separate passing test.

### Lifecycle questions answered

| Question | Answer |
|---|---|
| Command ID on the wire? | Required. `commandId`, or `id`. Without one the command is rejected — there is no row to acknowledge |
| Duplicates? | Suppressed per `commandId` for 15 min (256 ids, LRU). Re-reports the original outcome; never re-executes |
| When is `executedAt` set? | When the intent was applied (ACK), or when the attempt concluded (FAILED) |
| What is `FAILED`? | A rejection (6 reasons) or a refusal the agent can explain. Always carries a reason |
| After reconnect? | Registration repeats. Stale commands are rejected by age; replays hit the idempotency cache |
| Are commands replayed by the backend? | **Unknown — B-6.** The Pi is safe either way |

---

## 11. ACK flow

```json
{"schemaVersion":1,"robotId":"robotx-pi","commandId":"cmd-123",
 "status":"ACK","executedAt":1758561234.9,"reason":"mission stopped and route cleared"}
```

**`ACK` means applied, not received.** The backend enum is `SENT`|`ACK`|`FAILED`
with no separate terminal "executed" state — so if `ACK` meant "bytes
received", there would be no way to ever report the command took effect. The
Pi applies first and acknowledges second. **The ordering is asserted by a
test** that records the sequence and requires `["applied", "acked"]`.

The Pi **never** reports `SENT`; `build_command_result_payload` raises if asked
to. That is the backend's state for "issued", and a robot claiming it would
overwrite the server's own record.

A rejection with no `commandId` is logged locally and **not** acknowledged —
there is no row to fail, so an ack would be shouting into the void.

**B-8:** whether the backend expects Socket.IO callback acks instead of an ack
event is unverified.

---

## 12. Reconnection flow

The Pi drives its own connect loop (`reconnection=False`). The library can
reconnect itself, but its retries are invisible to the agent — state would sit
at `CONNECTED` while the library quietly failed in a loop. Since the dashboard's
online indicator is downstream of that state, every attempt is made observable.

Backoff: exponential from 1 s, capped at 60 s, times a random factor in
[0.5, 1.0]. **The jitter is not decoration** — a fleet that lost the same
backend would otherwise reconnect in lockstep and knock it over again the
moment it returned. Refusals use a flat 60 s instead.

### A real bug found and fixed during testing

The first classifier treated any error containing `"refused"` as an auth
rejection. **`"Connection refused"` is `ECONNREFUSED`** — the most common way a
connect fails when the backend is simply *down*. That would have put a Pi on the
60 s rejection backoff every time the server restarted, delaying its return by
up to a minute. Caught by a test written against realistic error strings, then
resolved by measuring what the library actually raises:

| Client exception text | Meaning | Backoff |
|---|---|---|
| `One or more namespaces failed to connect` | Transport reached the server; server refused | Rejected (60 s) |
| `Cannot connect to host …` | Network/DNS failure | Normal, jittered |

Unrecognized text is treated as a **network** failure — the safe direction: the
cost is a few extra attempts, whereas the reverse would leave a robot slow to
find a healthy backend.

---

## 13. Failure behaviour (Phase 11)

| Scenario | Behaviour | Status |
|---|---|---|
| Backend down at boot | Agent starts normally; link retries; mission untouched | **PASS** (integration test) |
| Backend dies mid-connection | Detected via disconnect callback; state updated | **PASS** (integration test) |
| Backend returns later | Reconnects without a restart; telemetry resumes | **PASS** (integration test) |
| Wi-Fi drops / returns | Same code path as above | **PASS-SYNTHETIC** (simulated by dropping the socket, not by cycling a radio) |
| Auth rejected | `REJECTED`, long backoff, agent keeps running | **PASS** (real refusing server) |
| Malformed server event | Rejected with a reason; never obeyed | **PASS** |
| Unknown command | `FAILED` with `UNKNOWN_TYPE`; nothing executed | **PASS** |
| Duplicate command | Executed once, acknowledged twice, identical `executedAt` | **PASS** (unit + integration + live) |
| Unknown robotId | Refused | **PASS** |
| Unexpected event name | Counted as `unexpected_events`, logged, not obeyed | **PASS** |
| Telemetry build failure | Emit failures counted, never raised at the agent | **PASS** |
| GPS unavailable | No telemetry sent; status keeps flowing | **PASS** |
| Camera / perception unavailable | Reported in status; decision layer stops the robot | **PASS** (pre-existing tests still green) |
| Internal agent exception | Loop survives, mode → `ERROR`, stop intent | **PASS** (pre-existing) |
| Pi process restart | Clean shutdown; link closed before subsystems | **PASS** |

### What the Pi does when disconnected

`ROBOTX_BACKEND_LOSS_POLICY`:

- **`pause` (default)** — after a 30 s grace period, an active mission is
  paused. The grace means a brief Wi-Fi blip does not strand the robot; the
  pause means an autonomous robot is not crossing a campus unsupervised.
- **`continue`** — the mission runs on; the backend is a supervisor, not a
  controller.

**Neither policy can start motion.** There is no code path from link loss to
anything but a pause, and a test asserts no action is taken from `IDLE`,
`PAUSED` or `STOPPED`. Additionally: **backend disconnect cannot create
uncontrolled motion because the Pi cannot create motion at all** — it has no
motor authority. Once the ESP32 exists, its command timeout is the final
low-level failsafe, independent of this link.

---

## 14. Physical vs simulated — **PARTIAL**

`Robot.simulated` distinguishes them; this agent sends `false`.

**No simulator exists in this repository**, so the intended "one protocol for
both" could not be compared or verified. If a simulator exists in the backend
repo, it is the authoritative source for the event names and should be used to
build the binding file — the Pi will then speak exactly what the simulator
speaks, with no code change.

The payloads were shaped around the shared `Telemetry` and `Command` models
precisely so one protocol can serve both. **Status: cannot be verified here.**

---

## 15. Files changed

| File | Change |
|---|---|
| `robotx/application/agent.py` | Added `mode` property, `pause_mission`, `resume_mission`, `return_to_base`, `_home_position`; mission route/origin retention; lazy backend-link startup (last, non-blocking) and shutdown (first); real communication health |
| `robotx/application/main.py` | Added `GET /backend`, `POST /mission/pause`, `POST /mission/resume` |
| `robotx/state/robot_state.py` | Added `MissionRefused`, `OperatingMode.PAUSED` + `has_route`, `LinkStatus.REJECTED` + `is_up`, and four observability fields on `CommunicationState` |
| `robotx/config/settings.py` | Added 14 backend settings + `home_lat`/`home_lon`; `_get_opt_float` |
| `robotx/communication/__init__.py` | Package documentation and the lazy-import rule |
| `requirements.txt` | `python-socketio` reclassified from unused to used; `aiohttp` named explicitly for the integration suite |
| `README.md`, `TEST_README.md` | Backend link, new endpoints, integration suite |
| `docs/architecture/ROBOTX_PI_ARCHITECTURE.md` | Backend seam rewritten from "not wired up" to its actual state |
| `docs/architecture/DEPENDENCY_MAP.md` | Communication package entries replaced |
| `.env.example` | Backend section rewritten; home position added |

## 16. Files created

| File | Lines | Purpose |
|---|---|---|
| `robotx/communication/protocol.py` | 677 | Wire contract: binding, payload builders, validation, redaction |
| `robotx/communication/backend_link.py` | 766 | The one Socket.IO client: lifecycle, backoff, rates, dispatch |
| `robotx/communication/commands.py` | 266 | Command → mission intent, idempotently |
| `tests/unit/test_protocol.py` | 458 | 56 tests |
| `tests/unit/test_backend_link.py` | 643 | 44 tests |
| `tests/unit/test_commands.py` | 300 | 25 tests |
| `tests/unit/test_agent_backend_commands.py` | 237 | 19 tests |
| `tests/integration/test_socketio_transport.py` | 596 | 25 tests, real transport |
| `tests/integration/soak_backend_link.py` | 280 | Measurement harness (manual) |
| `docs/communication/ROBOT_BACKEND_PROTOCOL.md` | 469 | The contract, and what the backend must implement |
| `docs/communication/SOCKET_IO_ARCHITECTURE.md` | 202 | How the boundary is built, and its measured cost |

## 17. Files removed

| File | Why |
|---|---|
| `robotx/communication/socket_client.py` | Superseded. Keeping it would mean **two Socket.IO clients**, two connection lifecycles and two answers to "is the robot online" — the exact anti-pattern the final audit checks for. Its only real capability (the handshake token) is preserved and extended. Its `manual` event was intentionally not carried forward: it accepted arbitrary motion off the wire |

Nothing else was deleted. No working perception, navigation or control code was
rewritten.

---

## 18. Exact implementation changes

1. **`ProtocolBinding`** — event names as data, not literals. `PROVISIONAL` by
   default, `FILE` when loaded from `ROBOTX_PROTOCOL_FILE`. A missing or
   malformed file **raises** rather than falling back silently, because a
   silent fallback lets an operator believe the real contract is in force
   while the Pi talks to nobody.
2. **Payload builders** mapped to the FalconAut models, with backend-owned
   fields deliberately absent.
3. **`TelemetryFrame`** — a refusal to produce a payload is a first-class
   return value carrying its reason.
4. **`parse_command`** — never raises (it runs on a remote-fed socket
   callback); 6 rejection reasons; 64 KiB cap; three timestamp formats
   accepted; an unparseable timestamp yields "none", never "now".
5. **`CommandExecutor`** — LRU+TTL idempotency; `MissionRefused` → `FAILED`
   with a reason; any other exception → `FAILED` without killing the link.
6. **`BackendLink`** — own connect loop, jittered backoff, rejection
   classification, four rate-controlled channels, event dedupe, link-loss
   policy, honest `describe()`.
7. **`OperatingMode.PAUSED`** — not `mission_active`, so the decision layer
   issues a hold on the very next tick. Pausing does not depend on anything
   downstream noticing a flag.
8. **`MissionRefused`** placed in `robotx/state` — so the agent can refuse a
   mission change without importing its own transport. Keeps the dependency
   direction communication → domain.
9. **Lazy `socketio` import** — with `ROBOTX_SOCKET_ENABLED=0` the library is
   never loaded; verified at runtime.

---

## 19. Test strategy

Three layers, each proving something the others cannot:

1. **Unit, faked transport** — deterministic coverage of lifecycle, backoff,
   rates, dispatch, payload correctness and refusals.
2. **Unit, real agent** — what a command actually does to a running robot,
   including that it cannot bypass perception or position validity.
3. **Integration, real Socket.IO** — genuine handshake, genuine JSON, genuine
   disconnects. The transport is **not** faked here, per the requirement.

Plus a **live process run** (uvicorn + agent + link + real server) and a
**hardware soak** with the real camera.

Deliberate negative-space coverage: no fabricated battery, no stale position,
no self-asserted `isOnline`, no token in anything loggable, no motion method on
the command interface, no hardware import in the communication package.

---

## 20. Unit test results — **PASS**

```
venv/bin/python -m unittest discover -s tests/unit -t .
Ran 345 tests ... OK
```

| File | Tests |
|---|---|
| Pre-existing suite (9 files) | 201 |
| `test_protocol.py` | 56 |
| `test_backend_link.py` | 44 |
| `test_commands.py` | 25 |
| `test_agent_backend_commands.py` | 19 |
| **Total** | **345** |

**All 201 pre-existing tests still pass.** No regressions.

Mapping to the 20 required areas: payload serialization ✓, validation ✓, robot
identity ✓, auth payload ✓, connection state ✓, reconnection ✓, backoff ✓,
telemetry generation ✓, rate limiting ✓, command parsing ✓, unknown commands ✓,
malformed commands ✓, duplicates ✓, ACK ✓, disconnect ✓, GPS unavailable ✓,
camera unavailable ✓, perception unavailable ✓, navigation unavailable ✓,
shutdown ✓.

---

## 21. Integration test results — **PASS (real transport)**

```
venv/bin/python -m unittest discover -s tests/integration -t .
Ran 25 tests in 11.8s ... OK
```

Real `socketio.AsyncServer` on a loopback port; the link builds its **normal**
`socketio.AsyncClient`. Nothing on the client side is faked.

| Group | Verified |
|---|---|
| Handshake & registration | Connects; `register` arrives with `robotId`, `simulated: false`, capabilities; credential reaches the server in the handshake; state reports `CONNECTED` only once it is |
| Authentication rejection | A server that genuinely refuses → `REJECTED`, never connected, still not `integrated` |
| Telemetry | Arrives as declared JSON; `battery` null; **no telemetry without a position** while status keeps flowing; rate bounded by configuration |
| Commands | Applied and acknowledged; unknown → `FAILED`; wrong robot refused; duplicate executes once with identical `executedAt`; a malformed command does not break the next |
| Disconnect / reconnect | Server hang-up detected; reconnects; **re-registers**; telemetry resumes; commands work again afterwards |
| Backend unavailable | Retries against a dead port without touching the mission; **connects once a backend appears on that port**, with no restart |
| Honest claims | A fully working transport still reports `integrated: false` under a provisional binding |

### Live process verification — **PASS**

Real uvicorn process running `robotx.application.main:app` with
`ROBOTX_SOCKET_ENABLED=1`, against a validating Socket.IO server:

- Registered with `simulated: false` and capabilities `{battery: false, motion: false}`
- `GET /backend` → `status: CONNECTED`, `integrated: false`, full counters
- `GET /config` → `robot_token: "UNSET"` (elided correctly)
- `PAUSE` → `ACK` `"paused with no active mission"`; `GET /state` → `"PAUSED"`
- Duplicate `PAUSE` → 2 acks, **identical `executedAt`**, executed once
- Unknown `EXPLODE` → `FAILED: UNKNOWN_TYPE: 'EXPLODE' is not one of ['STOP','PAUSE','RETURN','RESUME']`
- Final counters: `commands_received: 3, commands_rejected: 1, commands_duplicate: 1, acks_sent: 3, emit_failures: 0, unexpected_events: 0`

---

## 22. Real backend test results — **BLOCKED / NOT PERFORMED**

The FalconAut backend is not available in this environment: no source, no
deployment, no reachable address, nothing listening. **No test against the real
backend was performed, and none is claimed.**

Consequently **NOT TESTED**, every one of them: backend authenticates the Pi ·
backend marks the robot online · `socketId` association · backend receives
telemetry · robot live state updates · dashboard reflects live state · backend
records an acknowledgement · backend marks the robot offline/stale · backend
restores online state on reconnect.

The local protocol harness (§21) stands in for these and is explicitly labelled
as such. It exercises the Pi's half correctly; it says nothing about FalconAut.

---

## 23. Performance measurements — **PASS**

Raspberry Pi 5, real Camera Module 3 (IMX708) streaming, OpenCV perception,
agent loop at 10 Hz. Two 120 s windows, same process configuration.

| Metric | Agent alone | Agent + link | Delta |
|---|---|---|---|
| CPU (% of one core) | 14.0 | 14.0 | **0.0** |
| Loop period, median | 100.49 ms | 100.53 ms | **+0.04 ms** |
| Loop period, p95 | 101.07 ms | 101.26 ms | +0.19 ms |
| Perception processing | 4.55 ms | 4.89 ms | +0.34 ms |
| Max RSS | 122.6 MB | 127.6 MB | +5.0 MB |
| CPU temperature | 43.0 °C | 41.9 °C | — |

| Measurement | Value |
|---|---|
| Socket connect latency (loopback) | **8.7 ms** |
| Build one telemetry payload | **5.2 µs** |
| JSON-encode it | **7.6 µs** |
| Telemetry payload size | **154 bytes** |
| Status payload size | **782 bytes** |
| Emit failures over the soak | **0** |

**Conclusion:** the link does not measurably degrade perception or navigation.
The agent loop held its 10 Hz target to within 0.5 % with the link active. At
1 Hz, ~13 µs of serialization is immeasurable against a 100 ms loop — which is
the point of decoupling telemetry from camera rate.

Reconnection time was not measured as a standalone figure; it is dominated by
the configured backoff (1 s initial, jittered), and reconnection itself is
verified functionally in §21.

---

## 24. Security findings

| Area | Finding | Severity |
|---|---|---|
| **Backend token validation** | **The channel is not authenticated.** The Pi transports a credential; nothing verifies it. Until the backend does, anyone who can reach the namespace can command this robot | **CRITICAL — backend-side (B-7)** |
| **TLS** | `http://` by default. A warning is logged at startup naming the scheme. `ROBOTX_BACKEND_TLS_VERIFY=1` by default; disabling it is a deliberate, documented downgrade | **HIGH — deployment** |
| Credential storage | Environment variable only. No default, never hardcoded, `.env` gitignored | Resolved |
| Credential logging | Never logged. `redact()` scrubs credential-shaped keys; URL credentials stripped; library payload logging disabled; `/config` and `/backend` show `SET`/`UNSET`. Asserted by test | Resolved |
| Robot identity spoofing | Commands for another `robotId` refused client-side. **Server-side binding of token↔robotId is still required** (B-7) | Partial |
| Replay / stale commands | Rejected beyond 120 s. Unparseable timestamps are not treated as "now" | Resolved |
| Duplicate commands | Idempotent for 15 min; never executed twice | Resolved |
| Command authorization | **No per-command authorization exists.** Any party on an accepted socket can issue any of the four commands. All four are de-escalating except `RETURN`/`RESUME`, and none can bypass perception safety | **MEDIUM — backend-side** |
| Malformed payloads | 6 rejection reasons; parsing never raises | Resolved |
| Oversized payloads | 64 KiB cap before any walk of the structure | Resolved |
| Unexpected events | Counted and logged, never obeyed | Resolved |
| Connection flooding | Jittered exponential backoff; 60 s on refusal | Resolved |
| Local HTTP API | Still unauthenticated, including the new `/backend` and mission endpoints. Pre-existing; trusted-network only | **MEDIUM — pre-existing** |
| Physical robot safety | The Pi has no motor authority. No backend input can cause motion | Resolved by architecture |

---

## 25. Remaining blockers

| # | Blocker | Owner | Impact |
|---|---|---|---|
| **B-1** | Real Socket.IO event names and namespace unknown | Backend | Link cannot be pointed at FalconAut. **Everything else is ready** |
| **B-2** | Handshake `auth` shape unverified | Backend | Server may expect a different key or a header |
| **B-3** | Does each telemetry event write a `Telemetry` row? | Backend | Determines the correct rate; 1 Hz is conservative |
| **B-4** | Is `battery` nullable? | Backend | The Pi always sends `null` and will not send a number |
| **B-5** | Permitted `Robot.status` values | Backend | A mode mapping may be needed |
| **B-6** | Command replay policy after reconnect | Backend | Pi is safe either way; behaviour should be agreed |
| **B-7** | Does the backend validate the token? | Backend | **Until it does, this channel is unauthenticated** |
| **B-8** | Socket.IO callback acks vs an ack event? | Backend | Would change ack delivery |
| **B-9** | Task/Mission wire protocol, if any | Backend | Task association is **NOT IMPLEMENTED** |
| **B-10** | Decision/image-upload protocol (`WAIT`/`REROUTE`/`CANCEL`, `imageUrl`) | Backend | **NOT IMPLEMENTED**; the Pi has the perception data, the contract is missing |

---

## 26. Known limitations

- **The provisional binding is a guess.** It is reported as such everywhere and
  is trivially replaceable, but a connection over it proves only that *a*
  Socket.IO server accepted a connection.
- **No task or mission protocol.** The Pi carries an optional `taskId` on
  events and never receives one (B-9).
- **No `Decision` reporting.** No image-upload path (B-10).
- **Rejection reasons are opaque.** python-socketio does not deliver the
  server's message; the Pi knows it was refused, not why.
- **Battery will always be null** until sensing hardware exists on the ESP32.
- **Position freshness depends on GPS.** Indoors the robot sends status but no
  telemetry. This is correct behaviour and the backend must expect it.
- **Wi-Fi loss was simulated** by dropping the socket, not by cycling the radio.
- **Simulator parity unverified** — no simulator available (§14).
- **Local HTTP API remains unauthenticated** (pre-existing).
- **ESP32 link does not exist**, so no motion is possible at all.

---

## 27. Production readiness assessment

**NOT PRODUCTION INTEGRATED.**

**Pi side: production-ready, pending B-1/B-2.** Implemented, tested on a real
transport, measured on real hardware, with a clean and enforced boundary. The
one change needed to point it at FalconAut is a JSON file and an environment
variable.

**System: not integrated, and not securable today.** The backend is absent, so
zero end-to-end verification against FalconAut was possible. The channel is
unauthenticated until the backend validates the credential (B-7).

**Safe to deploy in its default configuration** (`ROBOTX_SOCKET_ENABLED=0`):
the agent is byte-for-byte the standalone agent it was, and does not even load
`socketio`. Safe to enable on a trusted network for backend bring-up.

**Physical safety is unaffected by any of this.** The Pi has no motor
authority; no backend input, malformed payload or connection failure can move
the robot.

---

## Summary

| Area | Status | Evidence |
|------|--------|----------|
| Socket.IO connection | **PASS** | 25 integration tests over a real transport; live uvicorn run; 8.7 ms connect latency |
| Authentication | **PARTIAL** | Credential delivered in the handshake and refusal handled correctly (real refusing server). **Not authentication until the backend verifies it — B-7** |
| Robot identity | **PASS** | `robotId` in handshake + every payload; wrong-robot commands refused; re-registration on reconnect, all tested |
| Telemetry | **PASS-SYNTHETIC** | Arrives as declared JSON over a real socket; rate bounded; no position → no frame. Against a **test** server, not FalconAut |
| Commands | **PASS** | All 4 applied and refused correctly; unknown/malformed/stale/wrong-robot rejected; unit + integration + live |
| ACK | **PASS** | `ACK`-after-apply ordering asserted; `FAILED` reasons; idempotent `executedAt` verified in all three layers |
| Reconnection | **PASS** | Disconnect detected, reconnect, re-register, telemetry resumes, commands work again. Backoff jittered and capped |
| Backend state update | **NOT TESTED** | No backend exists. `isOnline`/`socketId`/`lastSeenAt` are deliberately backend-owned |
| Dashboard live update | **NOT TESTED** | No dashboard available |
| Security | **PARTIAL** | No credential in any log/payload/API (tested); size, staleness and duplicate guards in place. TLS off by default; server-side validation absent (B-7) |
| Performance | **PASS** | Real camera, 2×120 s: **+0.0 CPU points, +0.04 ms loop period**, +5 MB RSS |
| Physical robot safety | **PASS** | No motor driver in the process; command interface is 4 mission methods; `RESUME` cannot bypass perception or position validity — each a passing test |
| Protocol contract | **BLOCKED** | Backend source absent. Names are `PROVISIONAL`; link reports `integrated: false` |
| Task / mission protocol | **NOT IMPLEMENTED** | No contract available (B-9) |
| ESP32 link | **NOT IMPLEMENTED** | Out of scope; boundary preserved |

---

### EXACT NEXT ACTIONS

**A. Pi software**
1. On receipt of the real event names (B-1), write the binding JSON, set
   `ROBOTX_PROTOCOL_FILE`, and confirm `/backend` reports `integrated: true`.
   No code change.
2. If B-2 shows a different handshake shape, adjust the `auth` dict in
   `BackendLink._connect_once` — one line.
3. If B-5 shows `Robot.status` does not accept the five modes, add a mapping in
   `build_status_payload`.
4. After B-3, revisit `ROBOTX_BACKEND_TELEMETRY_INTERVAL_S`.
5. Implement task association (B-9) and `Decision` reporting (B-10) once
   contracts exist. Do not implement them before.
6. Add authentication to the local HTTP API before any untrusted network.

**B. Backend software**
1. **Validate `auth.token` against `auth.robotId` in the `connect` handler and
   refuse otherwise (B-7).** Highest priority — nothing else makes this channel
   secure.
2. Publish the event names and namespace (B-1) — ideally by pointing at the
   simulator's implementation, so both robot types share one protocol.
3. Make `battery` nullable, or exclude it from validation (B-4).
4. Confirm the telemetry ingestion path (B-3) and the replay policy (B-6).
5. Serve TLS and issue per-robot credentials.
6. Derive `isOnline`/`socketId`/`lastSeenAt` from the connection — the Pi sends
   none of them, by design.
7. Decide `Command` authorization: who may issue `RETURN`/`RESUME`.

**C. Hardware**
1. Confirm the GPS receiver is attached and getting a fix outdoors — the soak
   ran with `NO_FIX`, so no live position has been published end-to-end.
2. Add battery sensing (ESP32-side) to resolve the permanent `null`.

**D. ESP32**
1. Implement the UART link consuming `MotionIntent`.
2. Implement the command timeout as the final low-level failsafe, independent
   of the backend link.
3. Report link state via `RobotState.update_communication(esp32=...)`.
4. Move battery, ultrasonic, IR and encoder sensing onto the ESP32.

**E. Manual validation (once a backend exists)**
1. Bring the Pi up against the real backend with the real binding, **wheels off
   the ground or motors disconnected**.
2. Verify in the database: robot marked online, `socketId` correct,
   `lastSeenAt` advancing, `Telemetry` rows at the expected rate.
3. Confirm the dashboard reflects live state.
4. Issue a `STOP`; confirm the `Command` row reaches `ACK` with `executedAt`.
5. Disconnect the Pi; confirm the backend marks it offline per its own logic.
6. Reconnect; confirm online state restores and no stale command re-executes.
7. Verify a **wrong** token is refused by the server.
8. Cycle Wi-Fi for real and confirm the `pause` policy behaves as documented.
