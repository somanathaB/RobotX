# RobotX Pi ↔ FalconAut backend protocol

What the Raspberry Pi agent puts on the wire, what it accepts, and what the
backend must do for this link to be real.

> **Read this first.** The FalconAut backend source is **not present** in this
> repository or anywhere on this machine. Neither is `schema.prisma`. That was
> verified by a filesystem-wide search, and it has a specific consequence:
>
> - **Field names and enum values below are derived** from the FalconAut data
>   model (`Robot`, `Telemetry`, `Command`, `Event`) and are used verbatim.
> - **Event names and the namespace are NOT derived.** They are guesses,
>   inherited from the client that already existed in this repo. They are
>   marked `PROVISIONAL` and are configuration, not code.
>
> Until the real event names are supplied, this Pi will connect, publish and
> respond correctly *to a server that happens to use these names*, and will
> report `integrated: false` in every case. Do not describe the system as
> integrated on the strength of a successful connection.

---

## 1. Architecture

```
FalconAut backend  ──┐
                     │  Socket.IO (implemented, contract unverified)
Raspberry Pi 5  ─────┤
                     │  UART (NOT IMPLEMENTED)
ESP32  ──────────────┤
                     │
Motor controller ────┤
                     │
Motors ──────────────┘
```

The Pi **cannot move the robot**. It owns no motor driver and imports none.
It publishes a `MotionIntent`; nothing consumes it yet. Every command in this
document changes the Pi's *intent*, never a motor.

---

## 2. Connection lifecycle

| Step | Actor | What happens |
|---|---|---|
| 1 | Pi | Connects to `ROBOTX_SOCKET_SERVER_URL`, namespace from the binding |
| 2 | Pi | Sends `auth` in the **Engine.IO handshake** (before any event) |
| 3 | Backend | Accepts, or raises `ConnectionRefusedError` to refuse |
| 4 | Pi | On accept: emits `register`, then a `status` frame immediately |
| 5 | Pi | Publishes `telemetry` (1 Hz) and `status` (0.2 Hz) while connected |
| 6 | Backend | May emit `command` at any time |
| 7 | Pi | Applies the command, **then** emits `command_result` |
| 8 | Either | On disconnect the Pi backs off and reconnects, re-registering |

The Pi drives its own reconnection (`reconnection=False` on the client) so
every attempt is visible in agent state. See
[SOCKET_IO_ARCHITECTURE.md](SOCKET_IO_ARCHITECTURE.md).

---

## 3. Authentication

The handshake `auth` object:

```json
{ "robotId": "robotx-pi", "token": "<ROBOTX_ROBOT_TOKEN>" }
```

**Status: client-side credential transport exists, but the channel is not
authenticated.** The Pi sends the credential. Nothing in this repository
verifies it, and no backend that does was available to test against. A token
in a handshake that nobody checks provides no security at all.

For the channel to actually be authenticated, the backend must:

1. Read `auth` in its `connect` handler.
2. Look up `auth.robotId` and verify `auth.token` against the stored
   credential for **that specific robot** — binding the two together, so a
   valid token for robot A cannot register as robot B.
3. Refuse otherwise (`raise ConnectionRefusedError(...)` / return `False`).
4. Reject any `command`-channel traffic on a socket that did not complete
   step 2, as defence in depth.

The Pi handles refusal correctly: the link enters `REJECTED`, backs off for
`backoff_rejected_s` rather than retrying at network speed, and the agent
keeps running. This is verified end-to-end in
`tests/integration/test_socketio_transport.py`.

**Known gap:** `python-socketio` does *not* deliver the server's rejection
message to the client. The Pi can tell it was refused, but not why. Verified
against python-socketio 5.11.4 / engineio 4.14.0: the client raises
`ConnectionError("One or more namespaces failed to connect")` and the
namespace `connect_error` handler is never invoked. If the backend needs the
robot to distinguish "bad token" from "unknown robot", it must send that as an
event after accepting the connection, not as a refusal reason.

**The token is never logged**, never placed in an event payload, and never
returned by `/backend` or `/config` (both report `SET`/`UNSET` only).

---

## 4. Robot identity

`robotId` is `ROBOTX_ROBOT_ID` and appears in the handshake, in every outbound
payload, and is checked against every inbound command.

The Pi asserts exactly one `Robot` column beyond its id: `simulated: false`.
It is the only party that knows what kind of agent it is.

**Fields the Pi never sends**, because the backend owns them:

| Field | Why it is backend-owned |
|---|---|
| `id` | Database primary key |
| `socketId` | Only the server knows which socket it is holding |
| `isOnline` | Derived from the connection the server can see. A robot that asserts its own liveness can lie about being alive — which is exactly the failure this integration must not have |
| `lastSeenAt` | Server clock; a robot's clock may be wrong or spoofed |
| `locationId`, `campusId`, `zoneId` | Fleet assignment, not robot knowledge |
| `currentTaskId` | Assigned by the backend |
| `createdAt` / `issuedAt` | Server clock orders records across robots |

---

## 5. Channels and rates

Four separate streams, deliberately at different rates. `Telemetry` is marked
HIGH FREQUENCY in the data model, so it carries the minimum and nothing else.

| Channel | Default rate | Purpose | Maps to |
|---|---|---|---|
| `telemetry` | **1 Hz** | Position and speed | `Telemetry` row |
| `status` | **0.2 Hz** (5 s) | Mode, health, subsystems | `Robot` columns |
| `event` | Event-driven, deduped | Operator-visible incidents | `Event` row |
| `command_result` | Per command | ACK / FAILED | `Command` update |

Diagnostics (CPU, memory, temperature, detection details, frame metadata) are
**not sent at all**. They are available on the Pi's local HTTP API.

### Why 1 Hz and not camera FPS

The camera runs at 20 FPS and perception at ~2 Hz. Telemetry is decoupled from
both. Twenty database rows per second per robot is not a dashboard, it is a
landfill. If the backend wants a smoother live map than 1 Hz, the right fix is
a separate realtime path (a Redis pub/sub fan-out, or an in-memory live-state
channel) that does **not** write a row per frame — not raising this number.

**Open question for the backend team:** does every `telemetry` event write a
`Telemetry` row, or is there a separate ingestion path? This determines the
correct rate and is recorded as blocker **B-3**.

---

## 6. Telemetry payload

Emitted **only** when there is a fresh GPS fix.

```json
{
  "schemaVersion": 1,
  "robotId": "robotx-pi",
  "lat": 12.9716,
  "lon": 77.5946,
  "speed": 0.8,
  "battery": null,
  "capturedAt": 1758561234.123,
  "positionAgeS": 0.15
}
```

154 bytes encoded, measured.

### `battery` is always `null`

This robot has **no battery-sensing hardware**: no fuel gauge, no ADC, no
voltage divider. A plausible-looking number would be indistinguishable from a
measurement, so the Pi sends `null`.

**Blocker B-4:** if `Telemetry.battery` / `Robot.battery` are non-nullable, the
backend will reject or mis-store these frames. The column must be nullable, or
the field omitted from validation. The Pi will not resolve this by inventing a
number.

### No fix means no frame

If GPS status is not `FIX`, or the position is older than
`ROBOTX_BACKEND_MAX_POSITION_AGE_S` (default 5 s), **no telemetry is sent**.
The last known fix is never resent as a current one — that would tell a
dashboard the robot is parked where it was ten minutes ago.

The `status` channel keeps flowing throughout, carrying the last known position
tagged with `ageS` and `isFresh: false`, so a dashboard can show a greyed-out
last-known marker. The backend should expect a robot to be connected and
healthy while sending no telemetry at all; that is a robot indoors or under
cover, not a fault.

`speed` is `null`, not `0.0`, when the receiver does not report it. "Not
measured" and "stationary" are different claims.

---

## 7. Status payload

```json
{
  "schemaVersion": 1,
  "robotId": "robotx-pi",
  "status": "AUTO",
  "battery": null,
  "reportedAt": 1758561234.5,
  "uptimeS": 3600.0,
  "health": { "status": "HEALTHY", "components": { "camera": "HEALTHY", "gps": "HEALTHY" } },
  "subsystems": { "gps": "FIX", "perception": "OK", "navigation": "NAVIGATING", "camera": "HEALTHY" },
  "position": { "lat": 12.9716, "lon": 77.5946, "ageS": 0.2, "isFresh": true,
                "headingDeg": 91.0, "headingSource": "NMEA_TRACK", "satellites": 9 },
  "motionIntent": { "command": "FORWARD", "reason": "path clear" },
  "navigation": { "status": "NAVIGATING", "waypointIndex": 1, "waypointsTotal": 3, "progress": 0.33 },
  "lastError": null,
  "protocol": { "provisional": true, "source": "PROVISIONAL" }
}
```

~780 bytes encoded, measured.

`status` is the agent's operating mode and is the single answer to "what is
this robot doing": `IDLE`, `AUTO`, `PAUSED`, `STOPPED`, `ERROR`.

**Open question B-5:** `Robot.status` is a backend enum whose permitted values
are unknown here. If it does not include these five, a mapping is required.

---

## 8. Event payload

```json
{ "schemaVersion": 1, "robotId": "robotx-pi", "type": "WARNING",
  "message": "...", "reportedAt": 1758561234.5, "taskId": "task-7" }
```

`type` is `INFO` | `WARNING` | `CRITICAL`. `taskId` appears only when the Pi
actually holds one. Messages are truncated to 500 characters, deduplicated for
30 s, and hard-limited to one per second, so a flapping subsystem cannot turn
this into a second telemetry stream.

---

## 9. Commands

Inbound, on the `command` event. Only the four values in the backend enum are
recognized; anything else is rejected, never guessed at.

```json
{ "commandId": "cmd-123", "type": "STOP", "robotId": "robotx-pi",
  "issuedAt": "2026-09-22T18:00:00Z" }
```

| Field | Required | Notes |
|---|---|---|
| `commandId` | **Yes** | Also accepted as `id`. Without it the command is rejected: there would be no row to acknowledge against |
| `type` | **Yes** | `STOP` \| `PAUSE` \| `RETURN` \| `RESUME`, case-insensitive |
| `robotId` | No | If present it **must** match, or the command is refused |
| `issuedAt` | No | Also accepted as `createdAt`. ISO-8601, epoch seconds or epoch milliseconds — which one the backend uses is unverified, so all three are parsed |

### What each command does

| Command | Effect | Refused when |
|---|---|---|
| `STOP` | Clears the route, mode → `STOPPED`, stop intent | Never — a stop must work from every state, including `ERROR` |
| `PAUSE` | Mode → `PAUSED`, **route retained**, stop intent | Never |
| `RESUME` | Mode → `AUTO` | Not currently `PAUSED`, or no route retained |
| `RETURN` | Routes to the home position, mode → `AUTO` | No home position known |

`PAUSE` and `STOP` differ deliberately: `STOP` ends the run, `PAUSE` keeps the
route so `RESUME` has something to go back to.

`RETURN` goes to `ROBOTX_HOME_LAT`/`ROBOTX_HOME_LON` if configured, otherwise
the GPS position recorded when the current mission started. **With neither, it
is refused with `FAILED`.** There is no depot coordinate anywhere in this
robot's configuration, and driving to an invented destination is the worst
thing this command could do.

### Commands cannot bypass safety

A command changes mission state. It does not reach past the agent's mission
methods — the communication layer has no import path to navigation internals,
the decision layer, GPIO or a motor driver, and a test enforces that.

After a backend `RESUME`, the robot still stops for a detected person, a
blocking obstacle, unusable perception or a lost GPS fix, exactly as before.
Verified in `tests/unit/test_agent_backend_commands.py`.

```
Backend command → Pi validates → mission state → decision layer (safety)
                → MotionIntent → [future ESP32 low-level safety] → motors
```

### Rejection reasons

Reported as `FAILED` with the reason prefixed:

| Reason | Meaning |
|---|---|
| `MALFORMED` | Not an object, or no readable `type` |
| `OVERSIZED` | Larger than 64 KiB |
| `UNKNOWN_TYPE` | Not one of the four commands |
| `MISSING_ID` | No `commandId` — **logged locally, not acknowledged**, since there is no row to fail |
| `WRONG_ROBOT` | `robotId` names a different robot |
| `STALE` | `issuedAt` older than `ROBOTX_BACKEND_COMMAND_MAX_AGE_S` (default 120 s) |

An unparseable `issuedAt` yields "no issue time" rather than "now" — treating
a bad timestamp as current would make every stale command look fresh.

---

## 10. Acknowledgement

```json
{ "schemaVersion": 1, "robotId": "robotx-pi", "commandId": "cmd-123",
  "status": "ACK", "executedAt": 1758561234.9, "reason": "mission stopped and route cleared" }
```

**`ACK` means applied, not received.** The backend enum is `SENT` | `ACK` |
`FAILED`, with no separate "executed" state — so if `ACK` meant "received",
there would be no way to ever report that the command took effect. The Pi
therefore applies the command first and acknowledges second, and `executedAt`
is the instant it was applied. Verified by test: the ordering is asserted.

The Pi **never** reports `SENT`. That is the backend's own state for "issued";
a robot claiming it would overwrite the server's record.

### Idempotency

Execution is idempotent per `commandId`, remembered for 15 minutes (256 ids
max). A redelivery, retry or post-reconnect replay **re-reports the original
outcome** — same status, same reason, same `executedAt` — without executing
again. A command cannot appear to change outcome just because it arrived
twice.

**Open question B-6:** whether the backend replays unacknowledged commands
after a reconnect. The Pi is safe either way, but a backend that does replay
should be aware the Pi will re-ack rather than re-execute.

---

## 11. Disconnection and reconnection

| Situation | Pi behaviour |
|---|---|
| Backend down at boot | Agent starts normally. The link retries with backoff; nothing blocks on it |
| Backend disappears mid-run | Detected via the socket's disconnect callback. State → `DISCONNECTED` |
| Wi-Fi drops / returns | Same path; the link reconnects and **re-registers** (the server assigns a new `socketId`, so the old association is void) |
| Auth refused | State → `REJECTED`, long backoff. Agent keeps running |
| Malformed server event | Logged and counted, never obeyed |
| Unknown event name | Counted as `unexpected_events` — evidence the provisional binding is wrong |

Backoff is exponential with jitter, capped at 60 s. The jitter matters: a
fleet that lost the same backend would otherwise reconnect in lockstep and
knock it over again the moment it returned.

### What losing the backend does to a mission

Configurable via `ROBOTX_BACKEND_LOSS_POLICY`:

- **`pause` (default)** — after `ROBOTX_BACKEND_LOSS_GRACE_S` (30 s), an
  active mission is paused. The grace period means a brief Wi-Fi blip does not
  strand the robot mid-route; the pause means an autonomous robot is not
  crossing a campus with no operator watching.
- **`continue`** — the mission runs on. The backend is a supervisor, not a
  controller.

**Neither policy can start motion.** There is no code path from link loss to
anything other than a mission pause. Enforced by test across every mode.

**Backend disconnect cannot create uncontrolled motion**, because the Pi
cannot create motion at all: it has no motor authority. Once the ESP32 exists,
its command timeout is the final low-level failsafe, independent of this link.

---

## 12. State consistency guarantees

| Guarantee | How |
|---|---|
| Never reports connected while disconnected | Link state is written only from Socket.IO's own connect/disconnect callbacks, into the one authoritative `RobotState` |
| Never asserts `isOnline` | The field is not sent. The backend derives it |
| Never sends stale GPS as current | Telemetry requires a fresh `FIX`; stale positions appear only on `status`, tagged `isFresh: false` with an `ageS` |
| Never sends a fabricated battery | `battery` is `null`; there is no sensor |
| Never reports healthy perception when the camera is down | Health is computed from a single consistent snapshot per tick |
| Never claims integration on a guessed contract | `integrated` is false while the binding is `PROVISIONAL`, and the health component reads `DEGRADED` |

---

## 13. Physical vs simulated robots

`Robot.simulated` distinguishes them; this agent sends `simulated: false`.

**No simulator exists in this repository**, so the intended "same protocol for
both" could not be verified. If one exists in the backend repo, it is the
authoritative source for the event names and should be used to build the
binding file below. The Pi's payloads are shaped around the shared `Telemetry`
and `Command` models precisely so that one protocol can serve both.

---

## 14. Supplying the real contract

Write the backend's actual event names to a JSON file and point
`ROBOTX_PROTOCOL_FILE` at it. No code change is needed.

```json
{
  "namespace": "/robot",
  "register": "robot:register",
  "telemetry": "robot:telemetry",
  "status": "robot:status",
  "event": "robot:event",
  "command_result": "robot:command_result",
  "command": "robot:command",
  "registered": "robot:registered"
}
```

Unspecified names keep their defaults. A missing or malformed file makes the
link refuse to start rather than fall back silently — falling back would let
an operator believe the real contract was in force while the Pi talked to
nobody.

Once loaded, the binding reports `source: "FILE"` and the link is eligible to
report `integrated: true` when connected.

---

## 15. Blockers — facts that still require the backend

| # | Blocker | Impact |
|---|---|---|
| **B-1** | Real Socket.IO event names and namespace unknown | Link cannot be pointed at the real backend. Everything else is ready |
| **B-2** | Handshake `auth` shape unverified (`robotId`+`token` assumed) | The server may expect a different key or a header |
| **B-3** | Whether each `telemetry` event writes a `Telemetry` row | Determines the correct rate; 1 Hz is a conservative guess |
| **B-4** | Whether `battery` is nullable | The Pi always sends `null` and will not send a number |
| **B-5** | Permitted `Robot.status` values | A mode mapping may be needed |
| **B-6** | Command replay policy after reconnect | Pi is safe either way; behaviour should be agreed |
| **B-7** | Whether the backend validates the token at all | Until it does, this channel is unauthenticated |
| **B-8** | Whether Socket.IO callback acks are used instead of an ack event | Would change how `command_result` is delivered |
| **B-9** | Task/Mission events, if any exist on the wire | The Pi carries an optional `taskId` on events but receives no task protocol. Not implemented |

---

## 16. Status of each capability

| Capability | Status |
|---|---|
| Socket.IO connection | IMPLEMENTED, TESTED (real transport) |
| Robot registration | IMPLEMENTED, TESTED (provisional event name) |
| Credential in handshake | IMPLEMENTED, TESTED — **not authentication until the backend verifies it** |
| Telemetry publishing | IMPLEMENTED, TESTED |
| Status/health publishing | IMPLEMENTED, TESTED |
| Event publishing | IMPLEMENTED, TESTED |
| Command reception | IMPLEMENTED, TESTED |
| Command validation | IMPLEMENTED, TESTED |
| ACK / FAILED | IMPLEMENTED, TESTED |
| Idempotency | IMPLEMENTED, TESTED |
| Disconnect detection | IMPLEMENTED, TESTED |
| Reconnection + backoff | IMPLEMENTED, TESTED |
| Link-loss mission policy | IMPLEMENTED, TESTED |
| Real backend validation | **BLOCKED** — no backend available |
| Dashboard live update | **NOT TESTED** — no dashboard available |
| Task / mission protocol | **NOT IMPLEMENTED** — no contract to implement |
| ESP32 link | **NOT IMPLEMENTED** — out of scope |
