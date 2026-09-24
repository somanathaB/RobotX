# P2B-1: RobotX Pi / RobotAgent backend integration, final report

**Date:** 2026-09-24 · **Authority:** `ROBOTX_PI_P2B1_HANDOFF.md` (Dashboard repo) · **Scope:** Raspberry Pi RobotAgent only · **Dashboard, ESP32, motors:** not touched · **Not committed**

**Bottom line:** the Pi now speaks the handoff protocol end to end: OFFER admission,
COMMAND_ACK, exactly one OFFER_* response, CUSTODY_EVENT, TASK_COMPLETE, HEARTBEAT
with commitment, and omit-when-missing TELEMETRY. **It rejects every OFFER today with
`NO_MOTOR_LINK`.** That is the truthful answer while the ESP32 link, the GPS fix and
custody sensing do not exist. RobotX remains unassignable by design.

---

## 1. Files changed

**New**

| File | Purpose |
|---|---|
| `robotx/communication/engine.py` | Engine envelope admission (steps 1–5), HMAC canonical signing, OFFER parsing, OFFER→mission, response, ACK and custody payload builders |
| `robotx/communication/commitment_store.py` | Persisted (0600, atomic, fsync) fence/sequence high-water marks, acked outboxIds, tombstones, respond-once record, custody/completion sent |
| `robotx/mission/evidence.py` | The backend's five L1 completion checks, same thresholds |
| `tests/fixtures/engine.py` | **SIMULATED** signed envelopes; test-only key |
| `tests/unit/test_engine_offer.py` | 75 tests, including required tests 1–16 |

**Modified**

| File | Change |
|---|---|
| `robotx/communication/backend_link.py` | `command` envelope handler; OFFER response path; custody and completion publishing; commitment heartbeat; new telemetry; operator ack only when applied; bare STOP not acked; TASK_ASSIGN never starts a mission; AUTH_SUCCESS+AUTH_OK dedupe; robot-id validation |
| `robotx/communication/protocol.py` | Binding names from the handoff; `TELEMETRY {timestamp, sequence, status, lat?, lon?, speed?}`; `HEARTBEAT {}` / `{commitmentId, fence}`; `COMMAND_ACK {commandId}`; `TASK_COMPLETE {taskId, lat, lon}`; `COMMAND.timestamp` read for staleness |
| `robotx/communication/commands.py` | `CommandTarget` gains `assess_offer`; `assign_mission(..., custody_required)` |
| `robotx/mission/mission.py`, `manager.py` | `AT_DROP`; custody-gated holds at pickup and drop; `record_custody` |
| `robotx/application/agent.py` | `assess_offer`, `custody_sensing_available() = False`, `record_custody` |
| `robotx/config/settings.py`, `.env.example` | `ROBOTX_ROBOT_ID` has no default; `ROBOTX_COMMAND_SIGNING_KEY` (secret); `ROBOTX_COMMITMENT_STATE_PATH` |
| `docs/communication/ROBOT_BACKEND_PROTOCOL.md` | Rewritten to the implemented contract |
| Tests | `test_protocol`, `test_backend_link`, `test_socketio_transport` (plus 3 new), `test_pi_backend_boundary`, `test_config`, `test_commands`, `test_local_frame`, `test_auth_and_commissioning`, and the soak script updated to the new shapes |

## 2. OFFER implementation

An OFFER arrives on the lower-case `command` event as a signed envelope.
Admission follows the handoff order:

1. `agentId` equals the robot ID.
2. `notValidAfter` is in the future.
3. HMAC-SHA256 verifies.
4. `fence` is greater than `fenceFloor` and greater than the highest applied fence.
5. `sequence` is next in line: a duplicate is dropped, a gap is held and
   released in order.
6. The high-water marks are persisted **before** acting.

The payload's `commitmentId` and `fence` must match the envelope's. An envelope
that fails any step is **not admitted**: no ack, no response, no effect.

`WITHDRAW`, `RECALL` and `ABORT_MISSION` stop that mission (only if it is the
one being driven), tombstone the commitment, and ack. Commands with no producer
today are admitted and acked with no invented effect.

## 3. ACK implementation

- **Engine:** `COMMAND_ACK {outboxId, fence, authorityEpoch}`, echoed as
  received, after admission and before the OFFER response. An exact
  redelivery of an already-admitted `outboxId` is re-acked (harmless per
  handoff §7) and nothing else happens.
- **Operator:** `COMMAND_ACK {commandId}`, **only for a command actually
  applied**. The wire has no FAILED form. A refused, unknown, stale,
  malformed or other-robot command is left unacknowledged, and the backend
  marks it FAILED after its 5/10/15 s redeliveries. A duplicate is re-acked
  and never re-applied.
- **Bare `STOP`:** obeyed, and not acked (handoff §14).

## 4. ACCEPT / REJECT / DEFER behaviour

- **Exactly one response per commitment, ever.** The response is persisted
  before it is sent. A redelivery, a later envelope for the same commitment,
  or a restart cannot produce a second one. If the record cannot be persisted,
  no response is sent (and an accepted mission is stopped).
- **The decision is `RobotAgent.assess_offer`:** "can this Rover physically
  execute it?", with no ranking and no auto-accept. It rejects with the first
  missing item, in this order:
  - `NO_EXECUTABLE_PATH` (a stop without a path; handoff-mandated)
  - `UNSUPPORTED_MISSION` (anything other than a two-stop task leg)
  - `ESTOP_LATCHED`
  - `AGENT_ERROR`
  - `MISSION_IN_PROGRESS`
  - `NO_MOTOR_LINK`
  - `NO_POSITION_FIX`
  - `NO_CUSTODY_SENSING`
- **ACCEPT** starts a custody mission from the offer's own stops and paths. If
  the agent then refuses (`MissionRejected`), the answer becomes a REJECT with
  that reason.
- **DEFER** is sent only with a valid future `until`: a number of epoch ms or
  an ISO-8601 string. A numeric string or a past time is refused, and the
  answer becomes a REJECT. **Nothing on this Rover produces a DEFER today:**
  it has no temporary condition with a knowable end time (there is no
  charging, for example).
- **Real result, verified live** (real `RobotAgent` and real Socket.IO client
  against a simulated backend): `COMMAND_ACK`, then
  `OFFER_REJECT {commitmentId, fence, reason: "NO_MOTOR_LINK"}`, persisted as
  `REJECT`.

## 5. CUSTODY implementation

- An accepted mission is a *custody mission*. It **holds at the pickup**
  (`AT_PICKUP`) until ACQUIRED is recorded, and **holds at the drop** (new
  `AT_DROP`) until RELEASED is recorded. Only then is it COMPLETE.
- Custody enters mission state only through
  `MissionManager.record_custody(kind, source)`, and only at an arrival decided
  on a **measured** position. ACQUIRED is accepted only at AT_PICKUP, RELEASED
  only at AT_DROP, each once. Arrival, a timer, a command or a simulated
  location never records it.
- The link emits `CUSTODY_EVENT {commitmentId, fence, kind}` from that state,
  once per kind (persisted), including across reconnects.
- **Nothing on this Rover calls `record_custody`.** There is no load sensor,
  compartment switch or agreed operator-confirmation path, so
  `custody_sensing_available()` is `False`. That is also why `assess_offer`
  would reject with `NO_CUSTODY_SENSING` even with motors and GPS present.

## 6. TASK_COMPLETE implementation

`{taskId, lat, lon}`, with `lat`/`lon` as JSON numbers from the **last measured
fix actually sent**. It is sent once, and only when all of these hold:

- the mission is COMPLETE;
- RELEASED has been sent;
- both arrivals were measured;
- the L1 check passes on the fixes the Pi sent since the grant:
  - within 25 m of the final stop;
  - at least 10 fixes per minute;
  - no gap over 10 s (the gaps counted include grant → first fix and last
    fix → claim);
  - at least 80% of fixes inside the ±30 m corridor of the commanded paths;
  - no implied speed over 8.33 m/s.

Otherwise the claim is **withheld**, logged, and never retried.

Dead-reckoned arrivals, no fixes, or a restart (the track is lost) all mean no
claim. `TASK_COMPLETE_ACK {verifying:true}` is logged and never retried. The
handoff's caveat (the backend completes on the claim alone when grading is off)
is exactly why this check stays on the Pi.

## 7. TASK_ASSIGN recovery behaviour

Parsed, correlated with the held commitment by `taskId`, logged
(`task_assign.not_authoritative`), and counted. It **never calls
`assign_mission`**, and nothing is sent back. Verified with the real agent: the
mission stays `None`.

## 8. Signature-key status

- **Can it be supplied by configuration?** Yes: `ROBOTX_COMMAND_SIGNING_KEY`,
  read like every other secret. It is shown as `SET`/`UNSET` in
  `public_summary()` and `/backend`, never logged, and required to be at
  least 32 bytes.
- **Can it be provisioned?** No. The backend has no mechanism. An operator
  would have to copy the backend's `COMMAND_SIGNING_KEY` onto the Pi out of
  band.
- **Blocker:**
  - The key is **fleet-wide and symmetric**. Anyone who reads it off this SD
    card can forge signed commands for every robot. Acceptable only on a
    trusted development LAN.
  - Until it is supplied, **no OFFER is admitted** (`SIGNATURE_UNVERIFIABLE`).
    Verification is never skipped; there is no bypass switch.
- **To pin before field use:** the handoff leaves three canonical-form details
  implicit. I chose one reading of each and pinned it with a hand-written test
  vector. Please confirm them against a vector generated by `commandSigning.js`:
  1. `notValidAfter` is rendered as the bare ISO string, without JSON quotes.
  2. Object keys are sorted recursively.
  3. The key is the UTF-8 bytes of the string.

## 9. Robot identity handling

- `ROBOTX_ROBOT_ID` has **no default**, in `Settings` or in `BackendConfig`.
  With the link enabled and no ID set, the link refuses to start with
  "must equal the commissioned Robot.robotId exactly".
- The value is used verbatim: case-sensitive, and not trimmed.
- It is sent only in `AUTH` and compared against `agentId` on engine
  envelopes. It is **not** in any other payload, because identity comes from
  the authenticated socket.
- For local development, set it to whatever the unit was commissioned as, for
  example `ROBOTX_ROBOT_ID=robotx-pi` (documented in `.env.example`).

## 10. Missing-data handling

Absence is always **omission**.

- **TELEMETRY** is `{timestamp, sequence, status}`. It gets
  `lat, lon, speed?` only for a fresh, measured GPS fix, and each fix is sent
  once.
- **Never sent:** `battery`, `energy`, `safety` / e-stop, `faults`,
  `localisation`, `heading`, `distanceTravelled`, and any key containing
  "capabilit".
- **No placeholders:** no `null`, `""`, `0` or `-1` stands in for a missing
  value.
- **`status`** takes only backend values: IDLE→IDLE, AUTO→ACTIVE,
  PAUSED→PAUSED, STOPPED→IDLE, ERROR→ERROR. It never sends OFFLINE.
- **`sequence`** is seeded from the wall clock in milliseconds, so it keeps
  increasing across restarts.
- **Where I followed your brief over the handoff.** Handoff §16 suggests
  `safety.estop {present:false}` and `faults: []`. Both blocks are
  **[ACCEPTED, IGNORED]** by the backend. Your brief said to omit e-stop, and
  `faults: []` would assert "measured: none" for faults nothing measures. So
  both are omitted. This is flagged in case Dashboard wants
  `{present:false}` sent later.

## 11. Tests added

- `tests/unit/test_engine_offer.py` (75 tests):
  - the canonical-signing vector;
  - required tests 1–16 as classes `T01`–`T16`;
  - admission rules: expiry, unknown command, held gap, WITHDRAW, no-effect
    commands, persist failure;
  - the real agent's rejection reasons;
  - custody genuineness on a real `MissionManager`;
  - the five L1 checks individually;
  - a restart with a persisted store;
  - a heartbeat that carries the commitment only while the mission runs.
- `tests/integration/test_socketio_transport.py`: over the real transport,
  - AUTH_SUCCESS+AUTH_OK authenticate once;
  - OFFER on `command` is acked, then rejected once, including on redelivery;
  - TASK_ASSIGN gets no reply.
- Existing tests were rewritten wherever they asserted the pre-handoff shapes.
- All fakes are labelled SIMULATED: `FakeSio`, `OfferAgent`, the
  `tests/fixtures/engine.py` envelopes and test key, and the synthetic GPS
  fixes.

## 12. Test results

```
venv/bin/python -m unittest discover -s tests -t .
Ran 708 tests in 17.840s
OK
```

(636 before this pass.) Live check with the real `RobotAgent` and real
Socket.IO client against the local contract server:

```
HEARTBEAT[0]    {}
TELEMETRY[0]    {'sequence': 1790193294411, 'status': 'IDLE', 'timestamp': 1790193294416}
COMMAND_ACK     [{'outboxId': 'ob-19', 'fence': '42', 'authorityEpoch': None}]
OFFER_REJECT    [{'commitmentId': 'c-7f3a', 'fence': '42', 'reason': 'NO_MOTOR_LINK'}]
OFFER_ACCEPT    []
mission         None            (after a TASK_ASSIGN for the same task)
status/auth     STREAMING 1     (AUTH_SUCCESS and AUTH_OK both received)
persisted       REJECT
```

**Not tested against the real Dashboard backend.**

## 13. Remaining physical hardware blockers

Each of these alone keeps RobotX unassignable, which is the intended outcome
until they exist:

1. **No Pi↔ESP32 link** → `NO_MOTOR_LINK`. The Pi cannot move the Rover or
   read ultrasonic, IR, encoder, motor or safety-stop data.
2. **No GNSS fix** (NEO-9M on the ESP32 I2C bus, not driven) →
   `NO_POSITION_FIX`, no `lat`/`lon`, and no completion evidence.
3. **No custody sensing** → `NO_CUSTODY_SENSING`. ACQUIRED and RELEASED can
   never be reported truthfully. This needs a sensor, or an agreed operator
   confirmation mechanism, which is a product decision.
4. **No signing key provisioning** → no OFFER is admitted until
   `ROBOTX_COMMAND_SIGNING_KEY` is copied in (see §8 for the fleet-key risk).
5. **No battery sensing, no e-stop circuit** → omitted. The dashboard will
   carry forward the commissioning-entered `initialBatteryPct`, which is a
   display value, not a measurement (handoff §16).

**Open items for Dashboard** (none block the Pi):

- Confirm the three canonical-signing details (§8).
- Confirm whether `safety.estop {present:false}` / `faults` should be sent
  (§10).
- Define the `stopType` vocabulary. The Pi executes only two-stop task legs,
  ordered by `sequence`, and does not interpret `stopType`.
