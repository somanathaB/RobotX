# RobotX Pi → FalconAut backend integration — implementation report

**Date:** 2026-09-22 · **Scope:** Raspberry Pi 5 agent only · **Motors:** never engaged · **ESP32:** not touched

---

## 0. Read this first

Two things shape everything below, and neither is a matter of opinion.

**1. `ROBOTX_BACKEND_PI_INTEGRATION_CONTRACT_AUDIT.md` was never delivered.**
It was promised twice and both messages arrived without it. It does not exist
on this machine, in this repository, or anywhere in git history — verified by
filesystem-wide search. Neither does the FalconAut backend source: no
`robot.handler.js`, no `socket.server.js`, no `VirtualRobot.js`, no
`schema.prisma`, nothing listening on any port.

What I implemented against is **the set of protocol facts stated in the task
prompts themselves**, which I treated as authoritative. Those gave event names,
the authentication sequence, the commissioning endpoint, the timestamp
semantics and the command surface. They did **not** give the exact field names
inside every payload. Where a name was not stated, §3.3 says so explicitly
rather than quietly guessing.

**2. Nothing here was tested against the real FalconAut backend.** It is not
reachable from this environment, by your own confirmation. Every transport
result in this report is `PASS-SYNTHETIC`: verified against a local Socket.IO
server built to the contract. No result is labelled `PASS`.

---

## 1. FalconAut contract as implemented

### 1.1 Connection

| Property | Value | Attested? |
|---|---|---|
| Connection | **Anonymous** — no `handshake.auth`, no query credential, no auth header | Yes |
| Client type | Native robot client: `User-Agent: robotx-pi/2.2`, **no `Origin`**, nothing Mozilla-shaped | Yes |
| Namespace | `/` (default) | **No** — inferred |

### 1.2 Authentication

```
POST /api/robots/commission  ->  6-digit pairing code, 300 s TTL
          |
   socket connects ANONYMOUSLY
          |
   Pi ── AUTH {robotId, pairingCode|token} ──> backend
          |
   backend ── AUTH_SUCCESS {token} ──> Pi        (success)
   backend ── disconnect(true), silently ──> Pi  (failure)
          |
   token persisted -> used for every later reconnect
```

### 1.3 Events

| Direction | Event | Attested? |
|---|---|---|
| Pi → backend | `AUTH` | Yes |
| Pi → backend | `TELEMETRY` | **No** — follows the contract's uppercase convention |
| Pi → backend | `COMMAND_RESULT` | **No** — same |
| backend → Pi | `AUTH_SUCCESS` | Yes |
| backend → Pi | `AUTH_FAILED` | **No** — may not exist; failure is a silent disconnect |
| backend → Pi | `COMMAND` (STOP/PAUSE/RETURN/RESUME) | Yes |
| backend → Pi | `STOP` (bare, task cancellation) | Yes |

**Four unconfirmed names.** They are reported by `GET /backend` under
`protocol.unconfirmed`, logged as a warning at startup, and each is a one-line
override in `ROBOTX_PROTOCOL_FILE` — **no code change** (§9).

### 1.4 Events the Pi deliberately does NOT send

`register`, `status` and `event` are bound to the empty string, meaning "never
emitted". The previous implementation invented a `robot_hello` registration
event and a `status` channel; the FalconAut contract declares neither.
**Emitting an event a backend never declared is how a client invents protocol**,
so these are now silent unless an operator binds them explicitly.

---

## 2. Telemetry contract

`TELEMETRY` payload, exactly these fields and no others:

| Field | Type | Req. | Nullable | Units | Pi source | Real? | If unavailable |
|---|---|---|---|---|---|---|---|
| `robotId` | string | yes | no | — | `ROBOTX_ROBOT_ID` | Real | n/a |
| `lat` | number | yes | no | degrees, 7 dp | GPS (NMEA) | **Real** | **frame not sent** |
| `lon` | number | yes | no | degrees, 7 dp | GPS (NMEA) | **Real** | **frame not sent** |
| `speed` | number | yes | **yes** | m/s, 3 dp | GPS speed-over-ground | **Real** | `null` |
| `battery` | number | yes | **yes** | percent | **none exists** | — | **always `null`** |
| `timestamp` | integer | yes | no | **epoch ms** | Pi clock at measurement | Real | n/a |

### 2.1 Refusals to fabricate

- **`battery` is always `null`.** There is no fuel gauge, ADC or divider on this
  robot. The old codebase hardcoded `76.0`; that is gone. **If FalconAut's
  `battery` column is non-nullable, that is a BLOCKER** (§12, B-2) — it will not
  be resolved by inventing a number.
- **No fix, no frame.** If GPS status is not `FIX`, or the position is older
  than `ROBOTX_BACKEND_MAX_POSITION_AGE_S` (5 s), **no telemetry is sent at
  all**. The last known fix is never resent as current. Observed live: 41
  consecutive frames skipped indoors while the robot stayed online (§7.3).
- **`speed` is `null`, not `0.0`,** when the receiver does not report it.
  "Not measured" and "stationary" are different claims.
- Distance, depth, obstacle state, motor state and ESP32 sensor values are
  **not sent at all**, because no such data exists on this robot.

### 2.2 The timestamp

`timestamp` is the instant the **position was measured**, in epoch
milliseconds, as an `int`:

```python
"timestamp": now_ms(position.timestamp)   # int(round(seconds * 1000))
```

It is **not** the moment the payload was built, not a startup constant, and not
a server timestamp. If a frame is delayed, the backend sees the measurement
time — which is what makes the delay visible rather than invisible.

Four separate tests defend this: it is an `int`, it is 13 digits (seconds would
be 10 — the most likely unit error), it converts a *given* instant rather than
reading the clock, and **successive frames carry advancing, distinct values**
(a startup constant would pass a single-frame check and still be wrong).

---

## 3. Command contract

### 3.1 Inbound

`COMMAND` → `parse_command` (type, id, robotId, age, 64 KiB cap) →
`CommandExecutor.execute` (idempotent) → `COMMAND_RESULT`.

| Command | Internal effect | Refused when |
|---|---|---|
| `STOP` | Route cleared, mode → `STOPPED`, hold intent | **Never** — must work from every state |
| `PAUSE` | Mode → `PAUSED`, **route retained**, hold intent | Never |
| `RESUME` | Mode → `AUTO` | Not `PAUSED`, or no retained route |
| `RETURN` | Route to home, mode → `AUTO` | **No home position known** |

The bare `STOP` event is handled on the **same idempotent path**, keyed
`stop:<taskId>` (or `stop:<commandId>`). Unlike `COMMAND`, it never rejects a
malformed payload: every failure mode of obeying a stop is "the robot stopped
when it need not have".

### 3.2 No fake physical execution

A command changes **mission state only**. `CommandTarget` is four methods wide
and a test asserts it exposes no method through which a speed, a steering value
or a GPIO pin could be set. The Pi has **no motor authority at all** — the ESP32
link does not exist — so:

> An `ACK` means *this robot applied the intent*, **not** that a rover
> physically moved or stopped. `agent_capabilities()` declares
> `motion: false` and `battery: false` for exactly this reason.

Physical actuation remains unvalidated and will stay so until ESP32 integration.

### 3.3 Outbound `COMMAND_RESULT`

```json
{"robotId":"robotx-pi","commandId":"c-1","status":"ACK",
 "executedAt":1790095705517,"reason":"mission stopped and route cleared"}
```

`executedAt` is epoch ms, matching the observation convention. `status` is
`ACK` or `FAILED`; the Pi **never** reports `SENT` (the builder raises if
asked) — that is the backend's own state for "issued".

---

## 4. Idempotency

The backend redispatches at roughly **5 s, 10 s, 15 s**. Handling:

1. `commandId` is required; a command without one is rejected (`MISSING_ID`)
   and **not** acknowledged — there is no row to fail.
2. `CommandExecutor.seen()` checks a bounded LRU (256 ids, 15 min TTL).
3. New → executed exactly once, outcome recorded.
4. Duplicate → **never re-executed**; the *original* `status`, `reason` and
   `executedAt` are re-reported, so the backend cannot see a command's outcome
   or completion time change on redelivery.

Verified live (§7.2): `live-stop` delivered 4×, `executedAt` identical
(`1790095705517`) on all four, `stop_mission` called **once**. The cache also
survives a reconnect — a replayed command after reconnect did not re-execute.

---

## 5. Socket.IO lifecycle

```
DISCONNECTED → CONNECTING → CONNECTED → AUTHENTICATING → AUTHENTICATED → STREAMING
```

`CONNECTED` and `AUTHENTICATED` are separate states on purpose. FalconAut's
connection is anonymous, so an open socket says nothing about whether this
robot is allowed on it — and **an auth failure arrives as a silent
`disconnect(true)`**, which at the transport layer is identical to the backend
restarting. The only thing distinguishing them is *what the link was waiting
for when the socket closed*, so the link records that explicitly.

### 5.1 The bug this replaces

The previous implementation classified auth failures by **matching the text of
a connect exception**. That approach cannot see a silent disconnect at all, and
it had a live footgun: `"Connection refused"` is ECONNREFUSED — a backend that
is merely *down* — and the old marker list risked putting the robot on a 60 s
"rejected" backoff every time the server restarted. That classifier is deleted.

Now: transport failure → `DISCONNECTED`, normal jittered backoff. Refused
credential → `AUTH_FAILED`, long backoff. A test asserts that **seven** distinct
failure strings, including the literal `"Unauthorized"`, all count as transport
failures when they come from `connect()` — because the transport never got far
enough for this robot to have been refused.

### 5.2 Listeners registered exactly once

The client is built once and handlers registered once, then reused across every
reconnect. `handler_registrations` is exported in `describe()` and asserted to
stay at **1** after three kick/reconnect cycles — a duplicated `COMMAND`
listener would execute the operator's command twice. Confirmed live after a
real reconnect.

### 5.3 Reconnect and credentials

A refused **token** is discarded, so the next attempt falls back to the pairing
code — a revoked token would otherwise be retried forever. A refused **pairing
code** is kept: with a 300 s TTL, expiry is far likelier than a wrong code.

---

## 6. Files changed

### Modified

| File | Change |
|---|---|
| `robotx/communication/protocol.py` | FalconAut binding (AUTH/AUTH_SUCCESS/COMMAND/STOP); `now_ms()`; telemetry reshaped to the contract fields with epoch-ms `timestamp`; `build_auth_payload`, `parse_auth_success`, `parse_stop_event`; `executedAt` → epoch ms; `UNCONFIRMED_NAMES`; undeclared channels unbound |
| `robotx/communication/backend_link.py` | Anonymous connect; AUTH/AUTH_SUCCESS flow; six-state lifecycle; single client with once-only handler registration; silent-disconnect auth detection; bare `STOP` handler; native-client headers; token integration; emit gated on authentication |
| `robotx/state/robot_state.py` | `LinkStatus` + `AUTHENTICATING`/`AUTHENTICATED`/`STREAMING`/`AUTH_FAILED`, `is_up`, `socket_open`; `CommunicationState` protocol source + auth method |
| `robotx/application/agent.py` | Communication health rewritten for the new states; `AUTH_FAILED` → `FAILED` |
| `robotx/application/main.py` | `GET /backend` reports `streaming`/`authenticated` instead of `integrated` |
| `robotx/config/settings.py` | `pairing_code`, `backend_token_path`, `backend_auth_timeout_s`; pairing code treated as a secret in `public_summary()` |
| `.env.example` | Commissioning and credential section rewritten |

### Created

| File | Lines | Purpose |
|---|---|---|
| `robotx/communication/token_store.py` | 209 | Atomic, 0600 session-token persistence |
| `robotx/communication/commissioning.py` | 208 | `POST /api/robots/commission` client + one-time CLI |
| `tests/unit/test_auth_and_commissioning.py` | 257 | 37 tests: timestamps, AUTH, tokens, commissioning |

### Test files rewritten

| File | Change |
|---|---|
| `tests/integration/test_socketio_transport.py` | Server rewritten as a **FalconAut contract server** (anonymous connect, AUTH → AUTH_SUCCESS, silent refusal); 25 → **34** tests |
| `tests/unit/test_backend_link.py` | Auth tests replaced; 44 → **57** |
| `tests/unit/test_protocol.py` | Binding/telemetry/timestamp tests updated; **58** |

**Not touched:** perception, navigation, decision, motion, camera, GPS drivers,
ESP32 seam. No unrelated refactoring.

---

## 7. Test results

### 7.1 Automated — 431 tests, all passing

```
venv/bin/python -m unittest discover -s tests -t .
Ran 431 tests in 10.197s ... OK
```

| Suite | Tests |
|---|---|
| Unit | 397 |
| Integration (real Socket.IO transport) | 34 |

The integration suite drives the **real `socketio.AsyncClient`** against a real
`socketio.AsyncServer` — genuine Engine.IO handshake, genuine JSON, genuine
disconnects. Nothing on the client side is faked.

### 7.2 The 20 required scenarios

| # | Scenario | Status | Evidence |
|---|---|---|---|
| 1 | Pi connects | **PASS-SYNTHETIC** | Integration + live process |
| 2 | `AUTH` emitted after connect (not in handshake) | **PASS-SYNTHETIC** | Server logged `auth=None`, then `AUTH{robotId,pairingCode}` |
| 3 | `AUTH_SUCCESS` parsed, token persisted | **PASS-SYNTHETIC** | Token on disk, mode 0600 |
| 4 | Authentication failure | **PASS-SYNTHETIC** | Silent disconnect → `AUTH_FAILED`; agent kept running |
| 5 | Telemetry transmitted | **PASS-SYNTHETIC** | Contract fields over a real socket |
| 6 | Required telemetry fields present | **PASS-SYNTHETIC** | `robotId/lat/lon/speed/battery/timestamp` |
| 7 | Timestamp correct | **PASS-SYNTHETIC** | Epoch ms, int, 13 digits, advances per frame |
| 8 | Null/unavailable values | **PASS-SYNTHETIC** | `battery` null always; no fix → no frame (41 skips live) |
| 9 | `COMMAND STOP` | **PASS-SYNTHETIC** | ACK, `stop_mission` once |
| 10 | `COMMAND PAUSE` | **PASS-SYNTHETIC** | ACK live |
| 11 | `COMMAND RETURN` | **PASS-SYNTHETIC** | Applied; refused honestly with no home position |
| 12 | `COMMAND RESUME` | **PASS-SYNTHETIC** | ACK from PAUSED; FAILED from STOPPED |
| 13 | Duplicate `commandId` | **PASS-SYNTHETIC** | 4 deliveries, identical `executedAt`, executed once |
| 14 | Malformed command | **PASS-SYNTHETIC** | 7 junk payloads; agent survived; next command obeyed |
| 15 | Disconnect during streaming | **PASS-SYNTHETIC** | Detected; state updated |
| 16 | Reconnect | **PASS-SYNTHETIC** | Reconnected using the **persisted token** |
| 17 | Telemetry resumes | **PASS-SYNTHETIC** | Integration test |
| 18 | Listener count stable | **PASS-SYNTHETIC** | `handler_registrations == 1` after 3 cycles |
| 19 | Backend unavailable at startup | **PASS-SYNTHETIC** | Retries; `auth_failures == 0`; mission untouched |
| 20 | Backend appears later | **PASS-SYNTHETIC** | Connects with no restart |

Also verified: **clean shutdown** (`PASS-SYNTHETIC`) — socket closed, no task
left running, safe when never started.

### 7.3 Live process run

Real uvicorn running `robotx.application.main:app`, real `BackendLink`, real
GPS hardware, against a contract server in a separate process.

```
CONNECTING → CONNECTED → AUTHENTICATING (pairing_code) → AUTHENTICATED → STREAMING
```

```
[server] CONNECT  auth=None  ua='robotx-pi/2.2'  origin=None
[server] AUTH {"robotId":"robotx-pi-live","pairingCode":"424242"}
[server] -> AUTH_SUCCESS (token issued)
```

| Check | Result |
|---|---|
| Anonymous connect | `auth=None`, no `Origin`, non-browser UA |
| Token persisted | `session.json`, mode `0600` |
| `PAUSE` | `ACK` "paused with no active mission" |
| `RESUME` | `FAILED` "no route is loaded; nothing to resume" |
| `RETURN` | `FAILED` "no home position…" — honest refusal, no invented destination |
| `STOP` | `ACK` "mission stopped and route cleared" |
| **Duplicate ×4** | 4 ACKs, `executedAt` **identical**, executed **once** |
| Bare `STOP` ×2 | Deduped on `stop:task-1` |
| Malformed ×4 | 3 dropped (no id), `EXPLODE` → `FAILED: UNKNOWN_TYPE` |
| **Telemetry** | **0 sent, 41 skipped** — GPS `NO_FIX` indoors |
| Backend killed | `DISCONNECTED`, `connect_failures=2`, **`auth_failures=0`** |
| Backend restarted | Reconnected, `AUTH{token:…}` — **not** the pairing code |
| After reconnect | `auth_successes=2`, `connects=2`, `handler_registrations=1` |

Final counters: `commands_received=11, commands_rejected=4,
commands_duplicate=4, stop_events_received=2, acks_sent=10, emit_failures=0,
unexpected_events=0`.

---

## 8. Status summary

### PASS
**None.** Nothing has been tested against the real FalconAut backend.

### PASS-SYNTHETIC
All 20 scenarios in §7.2, plus clean shutdown. Verified against a local
contract-compatible Socket.IO server and a live agent process.

### NOT TESTED
- Backend authenticates the Pi · backend marks the robot online · `socketId`
  association · `lastSeenAt` advancing
- Backend persists a `Telemetry` row · telemetry rate acceptable to the backend
- Backend records a `Command` acknowledgement
- Backend marks the robot offline/stale, and restores it on reconnect
- **Dashboard live update** — no dashboard was observed. Not claimed.
- Simulator parity — no simulator available
- Wi-Fi loss for real (simulated by dropping the socket)

### BLOCKED
- **B-1 — Real backend unreachable.** Every `PASS` in this document is blocked
  on this and nothing else.
- **B-2 — Is `battery` nullable?** The Pi sends `null` and will not send a
  number. A non-nullable column rejects every frame.
- **B-3 — Live telemetry on real GPS.** The receiver is connected and reading,
  but has **no fix indoors**. No Pi-measured position has ever been published
  end to end. Needs an outdoor test.
- **B-4 — Four unconfirmed binding names** (§1.3).
- **B-5 — Commissioning request/response shape.** The client has never reached
  a live endpoint. Its response parser is deliberately tolerant.
- **B-6 — `COMMAND_RESULT` delivery mechanism.** Whether FalconAut expects an
  emitted event or a Socket.IO callback ack is unverified.

### NOT IMPLEMENTED
- Task/mission assignment — out of scope, and `ENGINE_ENABLED=false` backend-side
- `Decision` reporting and image upload — no contract
- ESP32 link — out of scope; the `MotionIntent` seam is untouched

---

## 9. Configuration

| Variable | Default | Purpose |
|---|---|---|
| `ROBOTX_SOCKET_ENABLED` | `0` | Master switch. At 0, `socketio` is never imported |
| `ROBOTX_SOCKET_SERVER_URL` | `http://localhost:3000` | FalconAut base URL |
| `ROBOTX_ROBOT_ID` | `robotx-pi` | Robot identity, sent in `AUTH` |
| `ROBOTX_PAIRING_CODE` | unset | **Secret.** 6-digit code, 300 s TTL. First boot only |
| `ROBOTX_BACKEND_TOKEN_PATH` | `~/.robotx/backend_session.json` | Where `AUTH_SUCCESS`'s token is persisted (0600) |
| `ROBOTX_ROBOT_TOKEN` | unset | **Secret.** Explicit token override; rarely needed |
| `ROBOTX_BACKEND_AUTH_TIMEOUT_S` | `10.0` | Wait for `AUTH_SUCCESS` before failing the attempt |
| `ROBOTX_PROTOCOL_FILE` | unset | JSON overriding any event name |
| `ROBOTX_BACKEND_TELEMETRY_INTERVAL_S` | `1.0` | Telemetry rate |
| `ROBOTX_BACKEND_MAX_POSITION_AGE_S` | `5.0` | Older positions are not sent at all |
| `ROBOTX_BACKEND_COMMAND_MAX_AGE_S` | `120.0` | Stale commands rejected |
| `ROBOTX_BACKEND_LOSS_POLICY` | `pause` | `pause` \| `continue`. Neither can start motion |
| `ROBOTX_HOME_LAT` / `_LON` | unset | Where `RETURN` goes. Unset ⇒ `RETURN` refused |

Overriding an unconfirmed name needs no code change:

```json
{ "telemetry": "OBSERVATION", "command_result": "COMMAND_ACK", "namespace": "/robot" }
```
```bash
export ROBOTX_PROTOCOL_FILE=/etc/robotx/protocol.json
```

---

## 10. Exact commands

### Commission the robot (one-time, requires a reachable backend)

```bash
cd /home/pi/Desktop/RobotX
venv/bin/python -m robotx.communication.commissioning \
    --url http://<falconaut-host>:3000 \
    --robot-id robotx-pi
```

Prints the 6-digit code and the export lines. **The code expires in 300
seconds** — start the agent inside that window.

If the endpoint needs an operator credential, add `--bearer <token>`. If it is
not reachable from the Pi, obtain the code from the FalconAut dashboard and set
`ROBOTX_PAIRING_CODE` by hand; the Pi side is identical either way.

### Start the Pi agent

```bash
cd /home/pi/Desktop/RobotX
export ROBOTX_SOCKET_ENABLED=1
export ROBOTX_SOCKET_SERVER_URL=http://<falconaut-host>:3000
export ROBOTX_ROBOT_ID=robotx-pi
export ROBOTX_PAIRING_CODE=<6-digit code>     # first boot only
venv/bin/python -m uvicorn robotx.application.main:app --host 0.0.0.0 --port 8000
```

Watch it:

```bash
curl -s localhost:8000/backend | python3 -m json.tool   # expect status: STREAMING
curl -s localhost:8000/state   | python3 -m json.tool   # mode, gps, position
```

After the first `AUTH_SUCCESS`, **unset `ROBOTX_PAIRING_CODE`** — the persisted
token takes over. To force re-pairing, delete
`~/.robotx/backend_session.json`.

### Run the tests

```bash
venv/bin/python -m unittest discover -s tests -t .              # all 431
venv/bin/python -m unittest discover -s tests/integration -t .  # 34, real socket
```

---

## 11. Exact steps for the real integration test

When FalconAut is running, in order. Stop at the first failure.

1. **Reachability** — `curl -sv http://<host>:3000/socket.io/?EIO=4&transport=polling`
   from the Pi. Must return an Engine.IO handshake.
2. **Commission** — run the command in §10. If the response shape differs,
   `--json` prints it raw; the parser accepts `pairingCode`/`code`/`pairing_code`
   at the top level or one container down.
3. **Start the agent** with the code. Watch for
   `backend.link_status status=AUTHENTICATED`.
4. **If it hangs at `AUTHENTICATING`** → the `AUTH` payload field names are
   wrong. Check FalconAut's `robot.handler.js` for what it reads, then adjust
   `build_auth_payload` in `protocol.py` (one dict).
5. **If it disconnects immediately** → credential refused. The Pi reports
   `AUTH_FAILED` with `auth_failure` in `GET /backend`. Check the pairing
   code's 300 s TTL first.
6. **If `unexpected_events > 0`** in `GET /backend` → the backend is sending an
   event this binding does not know. The event name is in the logs. Add it via
   `ROBOTX_PROTOCOL_FILE`.
7. **Verify the token persisted** — `ls -l ~/.robotx/backend_session.json`
   (mode 0600). Restart the agent **without** `ROBOTX_PAIRING_CODE` and confirm
   `auth_method: TOKEN`.
8. **Telemetry — take the robot outdoors.** Confirm `GET /state` shows
   `gps.status: FIX`, then confirm `telemetry_sent` rises in `GET /backend`.
   **This is the one step that cannot be done at a desk (B-3).**
9. **Verify in the database** — a `Telemetry` row per frame, `timestamp` in
   epoch ms matching the Pi's clock, `battery` accepted as `NULL` (**B-2**).
10. **Dashboard** — confirm the robot appears online and the marker moves.
    Until someone watches this happen, dashboard live update stays NOT TESTED.
11. **Commands** — issue `STOP`, `PAUSE`, `RESUME`, `RETURN` from the
    dashboard. Confirm each `Command` row reaches `ACK` with `executedAt`.
    Remember `RETURN` is refused unless `ROBOTX_HOME_LAT/LON` is set.
12. **Duplicates** — let the backend's 5/10/15 s redispatch fire. Confirm the
    Pi's `commands_duplicate` rises while the physical/mission effect happens
    once.
13. **Reconnect** — restart FalconAut. Confirm the Pi reconnects on the stored
    token, telemetry resumes, and no stale command re-executes.
14. **Wrong credential** — start with a bogus `ROBOTX_PAIRING_CODE` and a
    deleted token file. Confirm `AUTH_FAILED`, not an infinite fast retry.

---

## 12. Known limitations

- **Four binding names are unconfirmed** (§1.3). A connection over them proves
  the Pi's state machine, not the contract.
- **`battery` will be `null` forever** until sensing hardware exists on the
  ESP32.
- **No live position has ever been published** — GPS has no fix indoors (B-3).
- **The token is a 0600 JSON file.** That stops other accounts and
  world-readable backups. It does not stop root, SD-card removal, or memory
  inspection. Right trade for an MVP on a trusted network; a TPM or encrypted
  partition is the answer if that changes.
- **TLS is off by default** (`http://`). A warning naming the scheme is logged
  at startup. The token and all telemetry travel in clear text.
- **The local HTTP API is unauthenticated**, including `/backend` and the
  mission endpoints. Pre-existing; trusted network only.
- **`ACK` never means physical motion.** No actuator exists.
- **Wi-Fi loss was simulated** by dropping the socket, not by cycling the radio.
- **The commissioning client has never reached a live endpoint** (B-5).

---

## 13. Bottom line

**The Pi is ready to be pointed at FalconAut.** The lifecycle, authentication,
commissioning, token persistence, telemetry shape, command handling,
idempotency and reconnection are implemented, exercised over a real Socket.IO
transport by 431 automated tests, and demonstrated in a live agent process
including a real disconnect/reconnect cycle that re-authenticated on the
persisted token.

**The integration is not done, and nothing here says it is.** No byte has
reached FalconAut. The realistic residual work when a backend appears is
adjusting field names inside three payloads — §11 says exactly where and how —
plus taking the robot outdoors for a GPS fix.

---

### What I need from you

1. **`ROBOTX_BACKEND_PI_INTEGRATION_CONTRACT_AUDIT.md`, actually pasted**, or
   `robot.handler.js` + the commission route. Specifically, four things:
   the telemetry event name, the command-result event name and delivery
   mechanism, the exact `AUTH` field names, and the namespace.
2. **Is FalconAut's `battery` column nullable?** (B-2) — a yes/no that decides
   whether telemetry is accepted at all.
3. **A reachable FalconAut instance**, to convert every `PASS-SYNTHETIC` above
   into `PASS`.
