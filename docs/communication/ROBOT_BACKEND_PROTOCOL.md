# RobotX Pi ↔ backend protocol (as implemented)

**Authority:** `ROBOTX_PI_P2B1_HANDOFF.md` (Dashboard repository, 2026-09-24).
This page describes what the Pi puts on the wire and why. If this page and the
handoff disagree, the handoff wins and this page is wrong.

Code: [protocol.py](../../robotx/communication/protocol.py) (plain events),
[engine.py](../../robotx/communication/engine.py) (signed engine envelopes),
[backend_link.py](../../robotx/communication/backend_link.py) (the one Socket.IO client),
[commitment_store.py](../../robotx/communication/commitment_store.py) (persisted high-water marks).

## Configuration

| Variable | Required | Meaning |
|---|---|---|
| `ROBOTX_SOCKET_ENABLED` | – | `1` to start the link |
| `ROBOTX_SOCKET_SERVER_URL` | yes | `http://<LAPTOP-LAN-IP>:<PORT>` or `https://<host>`. No default |
| `ROBOTX_ROBOT_ID` | yes | Must equal the commissioned `Robot.robotId`, exactly (case-sensitive). No default |
| `ROBOTX_PAIRING_CODE` | first boot | 6-digit code, 300 s TTL. Secret |
| `ROBOTX_BACKEND_TOKEN_PATH` | – | Where the `AUTH_SUCCESS` token is persisted (0600) |
| `ROBOTX_COMMAND_SIGNING_KEY` | for OFFERs | The backend's `COMMAND_SIGNING_KEY`. Secret. Unset: no OFFER is ever admitted |
| `ROBOTX_COMMITMENT_STATE_PATH` | – | Fence/sequence marks, tombstones and the respond-once record (0600) |

## Outbound

| Event | Payload | When |
|---|---|---|
| `AUTH` | `{robotId, token}` or `{robotId, pairingCode}` | After every connect |
| `HEARTBEAT` | `{}`, or `{commitmentId, fence}` while carrying out an accepted mission | Every 2 s, only while the agent loop is ticking |
| `TELEMETRY` | `{timestamp, sequence, status}` plus `lat, lon, speed?` for a fresh measured fix | 1 Hz |
| `COMMAND_ACK` | operator: `{commandId}`; engine: `{outboxId, fence, authorityEpoch}` | Operator: only once applied. Engine: once admitted |
| `OFFER_ACCEPT` / `OFFER_REJECT` / `OFFER_DEFER` | `{commitmentId, fence, reason?, until?}` | Exactly once per commitment |
| `CUSTODY_EVENT` | `{commitmentId, fence, kind}` | Once per genuine handover |
| `TASK_COMPLETE` | `{taskId, lat, lon}` | Once, only on L1-sufficient measured evidence |

**Missing data is omitted.** No `null`, `0`, `-1` or placeholder. The Pi never
sends `battery`, `heading`, `distanceTravelled`, `safety`, `faults`,
`localisation` or `energy`: nothing on it measures them. `lat`/`lon` are only
ever a fresh, measured GPS fix, each fix sent once. A dead-reckoned pose is
never sent, and no setting can make it be.

`timestamp` is the fix's measurement time when a fix is present, otherwise the
instant the status was read. `sequence` is seeded from the wall clock in
milliseconds and incremented per frame, so it keeps increasing across restarts.

## Inbound

| Event | Handling |
|---|---|
| `AUTH_SUCCESS` + `AUTH_OK` | Both arrive; the first authenticates, the second is ignored |
| `COMMAND {commandId, type, timestamp}` | Validated (type, robotId, age). Applied, then `COMMAND_ACK {commandId}`. Refused, unknown, stale or another robot's: **no ack** (the backend marks it FAILED) |
| `STOP {taskId?, reason, timestamp}` | Obeyed; no ack. Ignored if it names another robot |
| `command` (engine envelope) | Admission, then act; see below |
| `TASK_ASSIGN` | Recovery re-send only. Logged and correlated with the held commitment. **Never starts a mission**; no reply |
| `TASK_COMPLETE_ACK` | Logged. `verifying:true` is never retried |

## Engine admission

In this order. An envelope that fails any step is **not admitted**: no ack, no
response, no effect.

1. `agentId` equals `ROBOTX_ROBOT_ID`.
2. `notValidAfter` is in the future.
3. HMAC-SHA256 over the canonical string verifies with `ROBOTX_COMMAND_SIGNING_KEY`.
   There is no key-less mode.
4. `fence` is greater than `fenceFloor` and greater than the commitment's highest applied fence.
5. `sequence` is next in line. A duplicate is dropped; a gap is held.
6. The high-water marks are persisted, then the envelope is acted on.

One exception: an exact redelivery of an already-admitted `outboxId` gets its
`COMMAND_ACK` again (harmless per the handoff) and nothing else.

`WITHDRAW` / `RECALL` / `ABORT_MISSION` stop that mission, tombstone the
commitment and ack. Commands with no producer today are acked with no effect.

## OFFER decision

`RobotAgent.assess_offer` answers only "can this Rover physically execute
this?". It never ranks and never auto-accepts. It rejects with the first
missing capability:

`NO_EXECUTABLE_PATH` → `UNSUPPORTED_MISSION` → `ESTOP_LATCHED` → `AGENT_ERROR`
→ `MISSION_IN_PROGRESS` → `NO_MOTOR_LINK` → `NO_POSITION_FIX` → `NO_CUSTODY_SENSING`.

**Today every OFFER is rejected with `NO_MOTOR_LINK`**, because the Pi↔ESP32
link does not exist. A DEFER is sent only with a valid future `until`; an
invalid one becomes a REJECT.

## Custody and completion

An accepted mission holds at the pickup until `ACQUIRED` is recorded, and at
the drop until `RELEASED` is recorded. The only way either is recorded is
`MissionManager.record_custody`, at an arrival decided on a measured position.
Nothing on this Rover calls it today, because there is no custody sensing.

`TASK_COMPLETE` requires all of the following:

- the mission is complete;
- `RELEASED` has been sent;
- both arrivals were measured;
- the backend's five L1 checks pass on the fixes actually sent since the grant
  ([evidence.py](../../robotx/mission/evidence.py)).

Otherwise it is withheld, never sent on weaker evidence.
