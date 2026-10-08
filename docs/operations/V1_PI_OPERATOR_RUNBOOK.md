# RobotX Pi — V1 operator runbook

What an operator does with a deployed RobotX Pi robot, and what to do when it
does not go as expected. Installation is in
[V1_PI_DEPLOYMENT.md](V1_PI_DEPLOYMENT.md).

Every procedure has the same five parts: **Prerequisite**, **Action**,
**Expected**, **Failure**, **Escalation**. Do not continue past a failure.

> **There is no physical emergency stop on this robot yet.** The software e-stop
> below stops motion commands leaving the Pi; it is not a substitute for a
> physical emergency disconnect. Until one is fitted, keep a person who can cut
> drive power within reach of any robot that may move.

None of these procedures has yet been exercised on the physical robot. They are
written from the software (`robotx/`, the RobotX backend source) and from
software-only end-to-end runs.

---

## Conventions

### Access (V1)

The robot's local API is **unauthenticated** and listens on the Pi's loopback
address only. Operators reach it by logging in to the Pi over SSH and calling it
there:

```bash
ssh <operator>@<pi-address>
S=http://127.0.0.1:8000
```

Use `127.0.0.1`, never a host name. Do not change `ROBOTX_API_HOST`, and do not
leave an SSH port-forward or proxy to port 8000 running: any of those exposes
the unauthenticated API beyond the Pi. Every account that can log in to the Pi
can use every route, so the Pi's accounts must be the operators' only.
(Authenticating the API, H3, is V2 work; it becomes mandatory the moment the
API is reachable from anywhere but the Pi itself.)

Routes:

| Route | Effect |
|---|---|
| `GET /health`, `/state`, `/telemetry`, `/config`, `/backend` | read only |
| `GET /camera` | MJPEG preview. **Close it before stopping the service** (see Normal shutdown). |
| `POST /mission/pause` | hold: motion stops, route and lease kept; the backend then offers the robot no new work (see PAUSE semantics) |
| `POST /mission/stop` | abandon the mission (see STOP) |
| `POST /safety/estop` | latch the software e-stop |
| `POST /safety/clear` | release the latch; resumes nothing |
| `POST /mission/custody` | operator custody confirmation |
| `POST /mission/idle` | **do not use while a mission is active** (see E-stop clear) |
| `POST /mission/start`, `POST /mission/resume` | always 409 on a V1 robot (engine mode): missions and RESUME come from RobotX |

### Quick status

```bash
curl -s $S/state | jq '{mode, safety: .safety.rule, mission: (.mission | if . then {task_id, status, segment, pickup_measured, drop_measured, custody_acquired_at} else null end), esp32: .communication.esp32, motion_ready: .controller.motion_ready, reboot_latched: .controller.reboot_latched, gps: .gps.status, position: .position.source}'
curl -s $S/backend | jq '{status, streaming, authenticated, auth_method, auth_failure, held_commitment, link_loss}'
```

| Field | Values |
|---|---|
| `mode` | `IDLE`, `AUTO` (driving a mission), `PAUSED` (held, route kept), `STOPPED`, `ERROR` |
| `safety` | `"estop"` while the software e-stop is latched |
| `mission.status` | `TO_PICKUP`, `AT_PICKUP`, `TO_DROP`, `AT_DROP`, `COMPLETE`, `ABORTED` |
| `esp32` | `UP` is required for motion; `STALE`/`DISCONNECTED` are faults |
| `gps` / `position` | `FIX` / `"GPS"` = a measured position |
| `backend.status` | `STREAMING` = authenticated and publishing |

`held_commitment` in `/backend` is the Pi's **own record**. It stays set after a
STOP or a restart even though the backend no longer considers the robot to be
carrying it. The backend database is authoritative.

### Logs

```bash
journalctl -u robotx-agent -f
journalctl -u robotx-agent --since -15min | grep -E '<event>'
```

Events used below: `agent.started`, `agent.stopped`, `backend.loss_pause`,
`backend.auth_failed`, `backend.token_discarded`, `command.applied`,
`command.rejected`, `command.refused`, `mission.resumed`, `offer.answered`,
`custody.reported`, `task.complete_reported`, `task.complete_acked`,
`safety.estop_engaged`, `esp32.reboot_detected`, `esp32.final_stop`.

### Operator command API (RESUME, PAUSE, STOP)

There are two ways to send an operator command, and in V1 they do not offer
the same commands:

| | STOP | PAUSE | RESUME |
|---|---|---|---|
| **Dashboard UI** (robot page) | button | no button | **intentionally unavailable in V1** — shown as "RESUME (N/A IN V1)" |
| **Operator command API** (below) | yes | yes | yes — **only** at step 9 of the RESUME procedure, after every check has passed |

The dashboard's RESUME is unavailable by design: a paused unit cannot return
itself to *new* assignment in V1 (see PAUSE semantics). The operator command API
is the backend's own command endpoint — the same one the dashboard's buttons
call — and is the documented V1 procedure for RESUME of the leg a robot is
already carrying. It is not a way around the dashboard, and it relaxes no check:

- the backend accepts it only from an authenticated operator session;
- the backend delivers it to the robot over the robot's authenticated link;
- the Pi applies RESUME only from `PAUSED`, only with a route still loaded, and
  only if the command is at most 120 s old by the Pi's clock; a latched e-stop
  still vetoes every motion intent after it;
- nothing on the backend checks, at that moment, that the commitment is still
  this robot's — that is exactly what the RESUME procedure's checks are for.

The Pi's own `POST /mission/resume` is not part of this: on a V1 robot it always
answers 409 (engine mode).

**Authenticate** (operator's workstation, never on the Pi). The backend's login
returns the session only as a `token` cookie; keep it in a private cookie file.
Needs `curl` and `jq` on the workstation.

```bash
COOKIES="$HOME/.robotx-operator.cookies"
( umask 077; : > "$COOKIES" )
read -r -p 'email: ' EMAIL; read -rs -p 'password: ' PW; echo
jq -n --arg email "$EMAIL" --arg password "$PW" '{email: $email, password: $password}' \
  | curl -sS -c "$COOKIES" -X POST "https://<backend-host>/api/auth/login" \
      -H 'Content-Type: application/json' --data-binary @-
unset PW
```

Expected: `{"message":"User logged in successfully", ...}`. The cookie file now
holds an operator credential valid for 7 days: delete it when finished
(`rm -f "$COOKIES"`).

**Send a command:**

```bash
curl -sS -b "$COOKIES" -X POST "https://<backend-host>/api/robots/<robot-id>/command" \
  -H 'Content-Type: application/json' \
  -d '{"type":"RESUME"}'          # or {"type":"PAUSE"} / {"type":"STOP"}
```

Expected response: `{"ok":true,"command":{..., "type":"RESUME", "status":"SENT"},"delivered":true}`.
`"delivered": false` means the robot is not connected to the backend: do not
send it again — go back to the start of the procedure you are in. `401` means
the session is missing or expired: log in again.

What happens next: the Pi acknowledges only a command it applied, and the
backend's record of it becomes `ACK`. A command the Pi refuses or finds stale is
not acknowledged; the backend redelivers it twice, 5 s apart, then marks it
`FAILED` (about 15 s). The Pi's journal is the authoritative answer:
`command.applied`, `command.refused` or `command.rejected`.

### Checking commitment ownership (backend database, read only)

The dashboard does not show Leg state. Query the backend database **read only**.

**V1 temporary operator procedure:** no dedicated read-only operator database
role exists. Until one does, every query here runs in a session forced into
read-only mode (`default_transaction_read_only=on`, below), with whatever
database credential the backend owner issues for it. A dedicated least-privilege
operator role is a future hardening item.

```bash
PGOPTIONS='-c default_transaction_read_only=on' psql "<backend-database-url>" -v robot_id=<robot-id> <<'SQL'
SELECT c."commitmentId", c."releasedAt", c."leaseExpiry", now() AS db_now,
       c."custodyState" AS commitment_custody, l."legId", l.state AS leg_state,
       l."custodyState" AS leg_custody
FROM "Commitment" c
JOIN "Agent" a ON a.id = c."agentId"
JOIN "Leg"   l ON l.id = c."legId"
WHERE a."agentId" = :'robot_id'
ORDER BY c."grantedAt" DESC
LIMIT 3;
SQL
```

How to read it:

| Observation | Meaning |
|---|---|
| newest row, `releasedAt` empty | the backend considers this robot to hold that commitment |
| `leaseExpiry` later than `db_now` | the lease is live |
| `leaseExpiry` moved later between two reads | the backend is renewing it from this robot's heartbeats now |
| `leg_state` `ACCEPTED`, `EN_ROUTE_PICKUP`, `AT_PICKUP`, `LOADED`, `EN_ROUTE_DROP`, `AT_DROP` | the Leg is still executable |
| `leg_state` `STRANDED_SAFE`, `STRANDED_OBSTRUCTING` | stranded: see Stranded goods |
| `leg_state` `REASSIGNING`, `WITHDRAWN`, `CANCELLED`, `FAILED`, `ABORTING`, `RELEASED`, `SETTLED` | not this robot's to drive any more |
| `leg_custody` / `commitment_custody` `HELD` | goods are on board as far as the backend knows |

The lease is 60 s and is renewed from the robot's heartbeat once half of it has
run out, so two reads **at least 35 s apart** show it advance while the robot is
connected and holding the commitment. Never write to the database from this
runbook.

### PAUSE semantics (V1)

PAUSE is a deliberate V1 operational state, not a light-weight "hold for a
moment". What the source does:

- **On the Pi:** the mission is held — motion stops, the route is kept, and
  while the backend link is up the commitment's lease keeps being renewed. Only a
  RESUME command returns it to `AUTO`; a restored link never does.
- **On the backend** (assignment engine on): the robot's report that it is
  `PAUSED` is accepted, but its later report that it is fit again
  (`ACTIVE`/`IDLE`) is refused — an agent may declare itself less fit, never
  fitter. `clear-fault` applies only to `ERROR`/fault, not to `PAUSED`. A
  `PAUSED` physical robot counts as under operator hold, so the engine offers it
  **no new work**.
- **The leg it is carrying is not affected by that status:** lease renewal does
  not look at it, so after the RESUME procedure the robot can still finish the
  current leg — and it then stays out of new assignment.

So:

- **Use a manual PAUSE when you intend to take the robot out of normal
  assignment operation**, for the current leg or entirely. A manually PAUSED
  robot needs escalation to the backend owner before it returns to normal fleet
  availability; V1 has no operator path for that.
- **The automatic link-loss PAUSE (Y4) is part of the safety behaviour.** It is
  expected, and it has the same backend consequence. Handle it with the Backend
  loss procedure below.
- **To continue the current leg** after any PAUSE: the RESUME procedure, all
  checks.
- **If the robot cannot safely resume:** STOP, Out of service, or Stranded
  goods, according to custody — see the decision table under PAUSE.

(Verified in the backend source — the status trust rule, `clear-fault`, the
operator-hold fact for physical robots, lease renewal. Not yet exercised end to
end with the physical robot.)

---

## Normal startup

- **Prerequisite:** installed per V1_PI_DEPLOYMENT.md; the robot is stationary
  and clear; the clock is synchronised (`timedatectl` → synchronized: yes).
- **Action:** `sudo systemctl start robotx-agent` (or power on: the unit starts
  at boot), then Quick status.
- **Expected:** `journalctl -u robotx-agent -b | grep agent.started`; `mode:
  "IDLE"`, `safety` not `"estop"`, `esp32: "UP"`, `reboot_latched: false`,
  backend `status: "STREAMING"`, `/health` camera/perception/esp32 `HEALTHY`.
  An enrolled robot reconnects on its saved session token (`/backend`
  `auth_method: "TOKEN"`). No pairing code and no dashboard step are needed
  while the backend session has not expired (V1_PI_DEPLOYMENT.md section 18).
  The dashboard then shows the robot online. Online is not "Ready for Tasks".
- **Failure:** the service is `failed` or keeps restarting
  (`systemctl status robotx-agent`); `agent.start_failed`; backend `DISABLED`
  with `enabled: true`; camera `FAILED` ("camera did not start" — it is not
  retried); `esp32` not `UP`; `esp32.simulated` in the log.
- **Escalation:** do not let the robot take work. Read the log lines named above
  and the deployment checks (sections 19–21). After fixing the cause:
  `sudo systemctl reset-failed robotx-agent && sudo systemctl start robotx-agent`.
  A camera that failed at start needs a service restart.

## Normal shutdown

- **Prerequisite:** `mode` is `IDLE`; the database shows **no unreleased
  commitment** for this robot (Checking commitment ownership); nobody is viewing
  `/camera`.
- **Action:** `sudo systemctl stop robotx-agent`
- **Expected:** stops within 15 s; the log ends with `agent.stopped` (and
  `esp32.final_stop` if motion was sent since the link connected).
- **Failure:** it takes the full 15 s and is killed — almost always an open
  `/camera` viewer holding the HTTP server. The agent's own shutdown, including
  the final STOP to the ESP32, did not run; the ESP32's 2 s command watchdog is
  then what stops the motors.
- **Escalation:** close every camera viewer and check that the rover is
  stationary. If a commitment was unreleased, a stop strands it when goods are
  on board — see Restart while carrying goods.

## Backend unavailable

The backend cannot be reached at all (since startup, or for a long time).

- **Prerequisite:** none.
- **Action:** Quick status; `curl -s $S/backend | jq '{status, detail, last_connect_error}'`.
- **Expected:** `status` cycles through `CONNECTING`/`DISCONNECTED` with a
  `last_connect_error`; the link retries on its own (backoff up to 60 s). With no
  mission the robot stays `IDLE`: no OFFER can arrive, and local
  `/mission/start` and `/mission/resume` answer 409. Nothing to do on the Pi.
- **Failure:** `status: "DISABLED"` with `enabled: true` — a backend setting was
  refused at startup (`backend.config_invalid`), so it will never connect;
  `status: "AUTH_FAILED"` — see Authentication recovery.
- **Escalation:** check the backend's health and the Pi's network/DNS/clock. Do
  not change the robot's configuration to work around a backend outage.

## Backend loss and the Y4 pause

The link drops while the robot is driving a mission.

- **Prerequisite:** a mission is active (`mode: "AUTO"`).
- **Action:** watch: `journalctl -u robotx-agent -f | grep -E 'backend\.(loss_pause|auth_failed)|mission\.paused'`.
  Prepare to go to the robot.
- **Expected:** within about 25 s of the loss (up to 15 s to detect it, then a
  10 s grace) the log shows `backend.loss_pause` and `mode` becomes `PAUSED`; the
  robot holds with its route kept. When the link returns it re-authenticates
  (`backend.status: "STREAMING"`) and **stays PAUSED** — nothing resumes it
  automatically. `/backend` `link_loss.paused` was `true` during the loss.
- **Failure:** the robot is still `AUTO` well after 25 s of loss; or it moves
  while `PAUSED`.
- **Escalation:** e-stop locally (E-stop), and be ready to cut power. Report it:
  the pause must happen before the backend can hand the Leg to another robot.
  After the link returns: this automatic PAUSE is expected safety behaviour, and
  it is handled through the documented recovery — the RESUME procedure to
  continue the current leg, or the PAUSE decision table if the robot cannot
  safely resume. Either way the robot is afterwards out of new assignment until
  escalated (PAUSE semantics).

## Authentication recovery

`/backend` shows `status: "AUTH_FAILED"` or `auth_failure` is set.

- **Prerequisite:** the backend itself is reachable (otherwise: Backend
  unavailable).
- **Action:** `curl -s $S/backend | jq '{status, auth_method, auth_failure, credential, pairing_code}'`
  and `journalctl -u robotx-agent --since -15min | grep -E 'backend\.(auth_failed|token_discarded|token_kept|pairing_code_rejected|no_credential)'`.
- **Expected / decision:**
  - `backend.token_kept` (a timeout or a disconnect with no verdict): the stored
    token is kept and retried on the normal backoff. Wait.
  - `backend.token_discarded` (the backend answered `INVALID_CREDENTIAL`), or
    `credential.token: "UNSET"`: the robot must be enrolled again with the
    dashboard-first procedure. Follow V1_PI_DEPLOYMENT.md section 17.2 from step
    2 (Generate Pairing Code, PIN/passkey, run the command the dashboard
    displays), then section 18.1, with the robot IDLE. This is expected after
    the robot has been offline longer than the backend session lifetime
    (`ROBOT_SESSION_TTL_SEC`, default 30 days, range 1 hour to 90 days; section
    18.3).
  - During an enrollment: an expired code, a refused code, a locked robot, or a
    code lost to a backend restart are covered in V1_PI_DEPLOYMENT.md section
    17.4. In each case, generate a new code in the dashboard and run the new
    command. Never re-run an old one.
- **Failure:** pairing is refused repeatedly. **Five failed pairing attempts lock
  pairing for this robot for one hour.** Do not keep retrying.
- **Escalation:** unlocking early is a backend override
  (`POST /api/robots/<robot-id>/pairing/unlock`, quarantine-override approval).
  Never put a pairing code in `/etc/robotx/robotx-agent.env`, and never set
  `ROBOTX_ROBOT_TOKEN` as a workaround. "✓ Robot Connected" or `STREAMING` after
  re-enrollment means only that the link is authenticated, not that the robot
  is ready for tasks.

## Checking commitment ownership

- **Prerequisite:** a database credential from the backend owner, used only in
  the forced read-only session shown in Conventions (V1 temporary procedure; no
  dedicated read-only operator role exists yet).
- **Action:** run the query in Conventions; compare with the Pi:
  `curl -s $S/backend | jq .held_commitment` and
  `curl -s $S/state | jq '.mission | {task_id, status}'`.
- **Expected:** for a robot carrying a mission: the newest row's `commitmentId`
  equals `held_commitment.commitmentId`, `releasedAt` is empty, the Leg state is
  executable, and `/state` `.mission.task_id` equals `held_commitment.taskId`.
  For a robot with nothing to do: no row with an empty `releasedAt`.
- **Failure:** the Pi and the database disagree.
- **Escalation:** the database wins. A Pi that believes it holds a commitment the
  backend has released must not be resumed (STOP). A commitment the backend
  holds but the Pi has no mission for (after a restart) will lapse: with goods on
  board, follow Stranded goods.

## PAUSE

A deliberate V1 operational state (read PAUSE semantics first). The robot
stops, keeps its route, and keeps renewing the current leg's lease, so that leg
stays with this robot — but the backend takes the robot out of new assignment,
and bringing it back needs escalation. Use a manual PAUSE when you intend to stop
normal assignment operation for this robot; not as a casual pause button.

- **Prerequisite:** you intend the consequence above. (For an immediate motion
  hazard use E-stop instead — it acts locally, at once.)
- **Action:** operator command API `{"type":"PAUSE"}` (Conventions; the dashboard
  has no PAUSE button); if the backend is unreachable, on the Pi:
  `curl -s -X POST $S/mission/pause`.
- **Expected:** `mode: "PAUSED"`; `command.applied type=PAUSE` (API) or
  `mission.paused` (local); the rover stops. The backend then shows the robot
  `PAUSED` and offers it nothing new.
- **Failure:** `mode` does not change; the rover keeps moving.
- **Escalation:** E-stop; cut power if it still moves. Then decide what happens to
  the current leg:

| Situation (from Checking commitment ownership) | Procedure |
|---|---|
| The leg is still this robot's and executable, and the robot can safely continue | RESUME procedure (all checks) — the robot finishes the leg and then stays out of new assignment |
| The leg was released or reassigned, no custody held | STOP, then Out of service until escalated |
| No custody held, robot cannot safely continue | STOP, then Out of service |
| Custody held (goods on board), robot cannot safely continue | E-stop and follow Stranded goods — do **not** STOP to force the issue (STOP strands the leg) |
| Leg already `STRANDED_*` | Stranded goods |

In every case, escalate to the backend owner to return the robot to fleet
availability.

## RESUME

This is a documented, manual V1 procedure for continuing the leg a paused robot
is already carrying. **The dashboard's RESUME is intentionally unavailable in
V1; RESUME is sent with the operator command API (Conventions), and only at
step 9, after every check below has passed.** This is the V1 procedure, not a
way around the dashboard: it uses the same authenticated backend endpoint, and
every check on the backend and the Pi still applies. **Nothing on the backend
checks, when RESUME arrives, that the commitment is still this robot's** — steps
6 to 8 are that check, done by the operator. Every step must pass, in order; at
the first failure, do not resume. A RESUME continues the current leg only: the
robot stays out of new assignment afterwards (PAUSE semantics).

- **Prerequisite:** `mode: "PAUSED"` (a robot in `STOPPED` cannot be resumed;
  after an e-stop, see E-stop clear first); an operator command API session
  (Conventions).
- **Action:**
  1. **Backend connectivity restored:**
     `curl -s $S/backend | jq '{status, streaming, authenticated, auth_failure}'`
     → `status: "STREAMING"`, `streaming: true`.
  2. **Pi authenticated:** `authenticated: true` in the same output, and
     `auth_failure` empty.
  3. **Safety clear:** `/state` `.safety.rule` is not `"estop"`;
     `.controller.reboot_latched` is `false`; a person has checked the robot's
     surroundings.
  4. **ESP32 up and able to move:** `.communication.esp32 == "UP"` and
     `.controller.motion_ready == true`.
  5. **Position valid:** `.gps.status == "FIX"` and `.position.source == "GPS"`.
  6. **Read the backend state:** run the commitment query (Conventions).
  7. **Still this robot's and still executable:** the newest row's
     `commitmentId` equals `/backend` `.held_commitment.commitmentId`;
     `releasedAt` is empty; `leg_state` is one of `ACCEPTED`, `EN_ROUTE_PICKUP`,
     `AT_PICKUP`, `LOADED`, `EN_ROUTE_DROP`, `AT_DROP`; `leaseExpiry` is later
     than `db_now`.
  8. **Lease advancing:** run the query again at least 35 s later; `leaseExpiry`
     must have moved later.
  9. **Only then** send RESUME with the operator command API (Conventions):

     ```bash
     curl -sS -b "$COOKIES" -X POST "https://<backend-host>/api/robots/<robot-id>/command" \
       -H 'Content-Type: application/json' -d '{"type":"RESUME"}'
     ```

     Expect `"ok": true` and `"delivered": true`; `"delivered": false` means the
     robot is not connected — start again from step 1.
  10. If step 7 or 8 shows the commitment **released, reassigned or not
      advancing**: send **STOP** instead (STOP procedure), not RESUME.
  11. If `leg_state` is `STRANDED_SAFE` or `STRANDED_OBSTRUCTING`: follow
      **Stranded goods**.
- **Expected:** within seconds, `command.applied type=RESUME` and
  `mission.resumed` in the log; `mode: "AUTO"`; the rover continues its route.
  The backend keeps showing the robot as PAUSED and offers it no new work after
  this leg (PAUSE semantics); escalate to return it to fleet availability. The
  backend's record of the command becomes `ACK`.
- **Failure:** `command.rejected ... reason=STALE` (the command was more than
  120 s old by the Pi's clock: clock skew, or a late delivery);
  `command.refused ... cannot RESUME from <mode>`; no log line at all (the
  command did not reach the robot).
- **Escalation:** do not repeat RESUME blindly — start again from step 1, since
  time has passed. Check the clock (V1_PI_DEPLOYMENT.md section 15). If the
  robot moves unexpectedly: E-stop.

## STOP

Ends the mission. **With goods on board, STOP strands the Leg**: the robot stops
renewing the lease, and once it lapses the backend marks the Leg stranded, from
which V1 has no software exit. Before a deliberate STOP, check custody and use
the PAUSE decision table: with goods on board and a robot that cannot continue,
the procedure is Stranded goods, not STOP.

- **Prerequisite:** none for safety; for a deliberate STOP, know whether
  custody is held (`leg_custody` / `/state` `.mission.custody_acquired_at`).
- **Action:** the dashboard's STOP button (or the operator command API with
  `{"type":"STOP"}`); if the backend is unreachable, on the Pi:
  `curl -s -X POST $S/mission/stop`.
- **Expected:** `mode: "STOPPED"`, the route cleared, `/state` `.mission.status:
  "ABORTED"`; the rover stops. The lease is no longer renewed: within about
  2 minutes (60 s lease plus up to 60 s for the backend's sweep) the Leg leaves
  this robot — reassigned if no custody was held, stranded if it was. On the Pi
  this reports `IDLE` to the backend (the dashboard's "goes to PAUSED" warning
  describes the simulator). Do not use RETURN in V1: it also abandons the
  mission, and drives somewhere.
- **Failure:** the rover keeps moving; `mode` does not change.
- **Escalation:** E-stop; cut power. With custody held: Stranded goods.

## Custody confirmation

The person at the stop confirms each handover. It is the only custody source in
V1 (`ROBOTX_CUSTODY_CONFIRMATION=operator`).

- **Prerequisite:**
  - ACQUIRED (pickup): `/state` `.mission.status == "AT_PICKUP"` and
    `.mission.pickup_measured == true`; the goods are physically loaded.
  - RELEASED (drop): `.mission.status == "AT_DROP"` and
    `.mission.drop_measured == true`; the goods are physically unloaded.
  - **Everyone stands clear before confirming ACQUIRED**: the robot drives off
    toward the drop on the next control tick (within 0.1 s).
- **Action:**

  ```bash
  curl -sS -X POST $S/mission/custody -H 'Content-Type: application/json' -d '{"kind":"ACQUIRED"}'
  curl -sS -X POST $S/mission/custody -H 'Content-Type: application/json' -d '{"kind":"RELEASED"}'
  ```

- **Expected:** HTTP 200 with the mission; `custody.reported ... kind=ACQUIRED`
  (or `RELEASED`) in the log. After RELEASED: `task.complete_reported`, then
  `task.complete_acked`. If the backend was briefly unreachable, the report is
  kept and sent after the next authentication (`custody.resent`,
  `task.complete_resent`).
- **Failure:** 409 "operator custody confirmation is not enabled" (configuration);
  409 "custody can be acquired/released only at a measured ... arrival" (not at
  that stop, or the arrival was not on a measured position); 422 (wrong `kind`);
  `task.complete_withheld` (the robot's own evidence check failed);
  `task.complete_verifying` (the backend found the evidence insufficient and
  passed the task to an operator).
- **Escalation:** never confirm a handover that did not physically happen, and
  never confirm early to "unstick" a robot. A withheld or verifying completion
  goes to the backend owner. A Leg already stranded ignores custody reports.

## E-stop

Software e-stop: no motion intent leaves the Pi until it is cleared. It is not a
physical disconnect.

- **Prerequisite:** none — it works from every state.
- **Action:** `curl -s -X POST $S/safety/estop`. Remotely, if SSH is not
  available: the dashboard STOP (which also ends the mission — see STOP). Last
  resort: cut drive power.
- **Expected:** `{"emergency_stopped": true}`; `/state` `.safety.rule ==
  "estop"`, `mode: "STOPPED"`; `safety.estop_engaged` in the log. **The mission
  is not abandoned**: the lease keeps being renewed while the backend link is up,
  so the Leg stays with this robot until you clear and resume, or STOP.
- **Failure:** the rover keeps moving.
- **Escalation:** cut power. Report it. Note: **a service restart (or a reboot)
  clears the software e-stop**, and an ESP32 reboot latches it automatically
  (`esp32.reboot_detected`).

## E-stop clear

- **Prerequisite:** the cause is understood and removed; a person is at the
  robot; if the latch came from an ESP32 reboot, the reason for the reboot is
  understood.
- **Action:** `curl -s -X POST $S/safety/clear`
- **Expected:** `{"emergency_stopped": false, "was_engaged": true}`. Nothing
  resumes: `mode` stays `STOPPED`. Clearing also acknowledges an ESP32 reboot; the
  ESP32 link then re-proves itself before carrying motion (`motion_ready`).
  Then decide, using the PAUSE decision table (custody matters):
  - **continue the current leg:** `curl -s -X POST $S/mission/pause` (`mode:
    "PAUSED"`, route kept), then the full **RESUME** procedure. This is a
    PAUSE, so the robot leaves new assignment afterwards (PAUSE semantics);
  - **the robot cannot safely continue:** STOP / Out of service / Stranded goods,
    according to custody, as the table says.
- **Failure:** `was_engaged: false` (nothing was latched — check you are on the
  right robot); `.safety.rule` still `"estop"`.
- **Escalation:** **never use `POST /mission/idle` while a mission is active.**
  It puts the robot in IDLE with the mission still held: the lease keeps being
  renewed, the robot never moves, and RESUME is refused because it is not
  PAUSED. If that has happened: `POST /mission/pause`, then RESUME procedure, or
  STOP.

## Stranded goods

The backend shows `leg_state` `STRANDED_SAFE` or `STRANDED_OBSTRUCTING` (goods
on board, the robot no longer holding the Leg). **V1 has no automated recovery**:
nothing produces the "goods and agent recovered" event, engine cancellation is
refused, and custody reports against the stranded Leg are ignored. The
commitment stays unreleased, so the robot will not be offered work again.

- **Prerequisite:** none — act as soon as it is seen.
- **Action:**
  1. **E-stop** the robot: `curl -s -X POST $S/safety/estop`.
  2. **Physically recover the goods** — a person goes to the robot, removes them
     and records where they went.
  3. **Take the robot out of service** (V1_PI_DEPLOYMENT.md section 25).
  4. **Escalate** to the backend owner with the robot id, `commitmentId`,
     `legId`, `leg_state`, the time and where the goods are now.
- **Expected:** goods safe and accounted for; robot out of service; the case
  with the backend owner.
- **Failure:** the goods cannot be reached or the robot is in a hazardous place.
- **Escalation:** site safety first. **Do not improvise database writes**, do not
  RESUME, do not confirm custody, do not restart the robot to "clear" it.

## Restart while carrying goods

A service restart, crash or reboot while the robot holds a commitment.

- **Prerequisite:** none — this is what to do after it happened.
- **Action:** immediately `curl -s -X POST $S/safety/estop` (the restart cleared
  any previous e-stop); then Checking commitment ownership.
- **Expected:** the Pi comes back `IDLE` with **no mission** (missions are not
  restored after a restart). It therefore stops renewing the lease, and within
  about 2 minutes the backend marks the Leg `STRANDED_*` if custody was held
  (reassigns it if not). `/backend` `held_commitment` may still name the old
  commitment: it is the Pi's stale local record.
- **Failure:** n/a — the outcome is the stranded state.
- **Escalation:** with goods on board: **Stranded goods**. Prevent it: never
  stop, restart or reboot the service while the database shows an unreleased
  commitment for the robot.

## Out of service

- **Prerequisite:** no unreleased commitment for this robot (otherwise PAUSE,
  STOP or Stranded goods first).
- **Action:** V1_PI_DEPLOYMENT.md section 25 — A (motion disabled + e-stop, still
  visible) or B (`sudo systemctl disable --now robotx-agent`).
- **Expected:** A: `/config` `esp32_motion_enabled: false`, `.safety.rule ==
  "estop"`, telemetry still flowing; every OFFER answered REJECT. B: the robot is
  offline.
- **Failure:** an OFFER is accepted (`offer.answered ... verdict=ACCEPT`).
- **Escalation:** B immediately, then investigate the configuration. Remember a
  restart clears the e-stop of option A: re-apply it.

## Rollback

- **Prerequisite:** the robot is IDLE with no unreleased commitment; no goods
  on board.
- **Action:** V1_PI_DEPLOYMENT.md section 24. Targets: the V1 candidate
  (`099e386`/`130e3f6` and its release commit); emergency software rollback to
  `db10b15` or `8073c71` **only with `ROBOTX_ESP32_MOTION_ENABLED=0`**.
  `7c58cd6`, older commits, `master` and `laptop-pi` are **not** rollback
  targets.
- **Expected:** after the restart, the deployment checks (sections 19–21) pass;
  on a non-V1 target `/config` shows `esp32_motion_enabled: false`.
- **Failure:** the service does not start (`python -m robotx.application`
  fails — the target is too old), or the checks fail.
- **Escalation:** return to the previous commit and the backed-up environment
  file; keep the robot out of service; report.
