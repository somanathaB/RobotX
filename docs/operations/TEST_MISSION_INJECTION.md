# RobotX Pi — test mission injection (test deployments only)

> **Software test success is not hardware readiness.** Passing unit,
> integration or sandbox tests, a successful injection, or an accepted test
> mission says nothing about whether this rover can move safely. Physical
> movement stays prohibited until a fresh, correctly powered non-motion
> preflight passes on the robot **and** a stopping procedure has been agreed.
> There is **no physical emergency-stop button** on this rover; the only stop
> latch is the software one (`POST /safety/estop`).

The RobotX backend's Assignment Engine cannot currently assign tasks to
physical robots, so an integration test has no way to give this rover a
mission while its backend link is up. This procedure enables one opt-in,
test-only ingress, `POST /test/mission/inject`, that hands a locally written
task to the agent's own `assess_offer` → `assign_mission(custody_required=True)`.
Every readiness and safety gate still applies; the endpoint never moves a
wheel, enables motion or talks to the ESP32. Design:
`robotx/application/injection.py`; harness: `tests/mission_injection/`.

It is **off by default** and must never be enabled on a robot in normal
service. Every step below that changes the robot (commit, checkout, environment
file, restart) needs the owner's explicit approval first.

## 1. Provenance, and what the backend sees

- Every test task id starts with `TEST-INJECTED-`; anything else is refused.
- The agent writes an append-only provenance log
  (`ROBOTX_TEST_INJECTION_LOG_PATH`, normally
  `/var/lib/robotx/test_injections.jsonl`, mode 0600). Every record carries
  `"provenance": {"source": "test_injected", "backend_assigned": false, ...}`.
  The harness keeps its own ledger the same way
  (`~/.local/state/robotx-test-injection/ledger.jsonl`).
- A test task has **no RobotX commitment**, so the Pi never sends `OFFER_*`,
  `CUSTODY_EVENT` or `TASK_COMPLETE` for it, and `HEARTBEAT` stays `{}`.
- TELEMETRY is unchanged and genuine: real positions, `status: "ACTIVE"` while
  the test mission runs, readiness blockers.
- **The backend cannot tell a test mission apart.** TELEMETRY has no task id
  and the backend API has no provenance field, so the dashboard simply shows
  the robot ACTIVE and moving (or holding). Tell the backend/dashboard owner
  before a test, record the test run id with the time window, and never report
  a test-injected run as a RobotX assignment or delivery. The provenance logs on
  the Pi are the only record of where the task came from.
- A real OFFER that arrives during a test mission is refused honestly
  (`OFFER_REJECT MISSION_IN_PROGRESS`); a test injection while RobotX holds a
  commitment is refused (`409 BACKEND_COMMITMENT_HELD`).

## 2. Tokens: two, separate, operator-held

| Token | Protects | Configured on the robot as |
|---|---|---|
| **operator token** | `POST /safety/clear`, `/mission/custody`, `/controller/reset`, `/mission/idle` | `ROBOTX_OPERATOR_TOKEN_SHA256` (digest only) |
| **injection token** | `POST /test/mission/inject` | `ROBOTX_TEST_INJECTION_TOKEN_SHA256` (digest only) |

The robot stores only SHA-256 digests; the tokens stay with the operator. The
agent refuses to enable injection unless the operator token is configured and
the two differ.

Generate both once, as the operator user on the Pi (nothing is printed except
the digests):

```bash
install -d -m 700 ~/.config/robotx
( umask 077
  openssl rand -hex 32 > ~/.config/robotx/operator-token
  openssl rand -hex 32 > ~/.config/robotx/injection-token
  printf 'Authorization: Bearer %s\n' "$(cat ~/.config/robotx/operator-token)" > ~/.config/robotx/operator.header )
ls -l ~/.config/robotx          # every file -rw------- , directory drwx------
printf %s "$(cat ~/.config/robotx/operator-token)"  | sha256sum | cut -d' ' -f1   # -> OPERATOR_DIGEST
printf %s "$(cat ~/.config/robotx/injection-token)" | sha256sum | cut -d' ' -f1   # -> INJECTION_DIGEST
```

Storage rules:

- Never put a token (only its digest) in `/etc/robotx/robotx-agent.env`, a
  ticket, a chat, a commit or a shell history line. `curl -H @file` keeps the
  token off the command line, where `ps` would show it.
- `~/.config/robotx` stays `0700` and its files `0600`, owned by the operator.
  The inject CLI refuses a token file anyone else can read.
- Rotate by generating a new token, replacing the digest and restarting.
  Rotate immediately if a token may have been seen.

## 3. Authorization on the release routes

These four routes release a stop or advance a mission:
`POST /safety/clear`, `/mission/custody`, `/controller/reset`, `/mission/idle`.

- **Always (every configuration):** a request carrying an `Origin` header is
  refused with 403 — that is every browser request, including a web page open
  on the Pi. curl never sends one.
- **With `ROBOTX_OPERATOR_TOKEN_SHA256` set:** they also require the operator
  token and a loopback `Host` (use `S=http://127.0.0.1:8000`, never a host name).

```bash
S=http://127.0.0.1:8000
OP=~/.config/robotx/operator.header
curl -sS -X POST $S/safety/clear      -H @$OP
curl -sS -X POST $S/controller/reset  -H @$OP
curl -sS -X POST $S/mission/idle      -H @$OP          # never while a mission is active (runbook)
curl -sS -X POST $S/mission/custody   -H @$OP -H 'Content-Type: application/json' -d '{"kind":"ACQUIRED"}'
curl -sS -X POST $S/mission/custody   -H @$OP -H 'Content-Type: application/json' -d '{"kind":"RELEASED"}'
```

| Answer | Meaning |
|---|---|
| 401 `a valid operator bearer token is required` | header missing or wrong token |
| 403 `browser requests are refused on this route` | an `Origin` header was sent |
| 403 `the Host header must name this Pi's loopback address` | called by host name, or through a proxy |
| 503 `operator access is misconfigured` | the digest is not 64 lowercase hex: the four routes stay refused (the robot stays stopped) until it is fixed |

`-H @$OP` needs the header file to exist (it does once section 2 has been
followed; injection cannot be enabled without the operator token). If it does
not, curl exits with code 26 and sends no request at all -- it does not skip a
missing file. On a robot with no operator token configured, use the runbook's
`AUTH=()` form instead (runbook, Access).

**Stop routes never need a token**: `POST /safety/estop`, `/mission/stop` and
`/mission/pause` work from anywhere on the Pi, always.

## 4. Configuration

Add to `/etc/robotx/robotx-agent.env` with `sudoedit` (keep a dated copy first,
deployment guide section 12):

```
ROBOTX_OPERATOR_TOKEN_SHA256=<OPERATOR_DIGEST>
ROBOTX_TEST_INJECTION_ENABLED=1
ROBOTX_TEST_INJECTION_TOKEN_SHA256=<INJECTION_DIGEST>
ROBOTX_TEST_INJECTION_LOG_PATH=/var/lib/robotx/test_injections.jsonl
```

Required as they already are on the V1 robot, and checked by the agent:
`ROBOTX_API_HOST=127.0.0.1` (an IP literal; `localhost` disables injection) and
`ROBOTX_CUSTODY_CONFIRMATION=operator`. The log must be under
`/var/lib/robotx`: the unit's `ProtectSystem=strict`/`ProtectHome=yes` make any
other location unreadable or read-only, and the agent then refuses every
injection (it still starts). Nothing else changes — backend URL, credentials,
signing key and `ROBOTX_ESP32_MOTION_ENABLED` stay as they are.

The code in `/opt/robotx/RobotX-Pi` must contain this feature. Putting it there
is a deployment, which needs its own approval: the commands below document the
procedure; nothing in preparing or reviewing the feature runs them.

The `test/mission-injection-harness` branch is **not pushed**, so the
deployment guide's `git fetch origin` (section 24, step 5) cannot retrieve it.
Fetch it from the development checkout instead, by the branch name, keeping the
per-command `safe.directory` protection (never add either path to a global git
config):

```bash
git -C /home/pi/Desktop/RobotX-Pi rev-parse test/mission-injection-harness   # -> <COMMIT>, the reviewed commit

sudo git -c safe.directory=/opt/robotx/RobotX-Pi \
  -c safe.directory=/home/pi/Desktop/RobotX-Pi \
  -C /opt/robotx/RobotX-Pi \
  fetch /home/pi/Desktop/RobotX-Pi test/mission-injection-harness
sudo git -c safe.directory=/opt/robotx/RobotX-Pi -C /opt/robotx/RobotX-Pi rev-parse FETCH_HEAD   # must print <COMMIT>
# umask 022 is REQUIRED: a checkout under a 0077 umask writes the new files
# root:root 0600, the robotx service user cannot import them, and the agent
# crash-loops (what happened to the 2026-10-19 16:30 attempt).
sudo sh -c 'umask 022 && git -c safe.directory=/opt/robotx/RobotX-Pi -C /opt/robotx/RobotX-Pi checkout --detach <COMMIT>'
# Repair anything an earlier checkout left unreadable, then prove it as the service user:
sudo find /opt/robotx/RobotX-Pi -path /opt/robotx/RobotX-Pi/venv -prune -o -path /opt/robotx/RobotX-Pi/.git -prune -o \
  \( -type f ! -perm -o=r -exec chmod a+r {} + \) -o \( -type d ! -perm -o=rx -exec chmod a+rx {} + \)
sudo find /opt/robotx/RobotX-Pi -path /opt/robotx/RobotX-Pi/venv -prune -o -path /opt/robotx/RobotX-Pi/.git -prune -o ! -perm -o=r -print
#   must print nothing
sudo -u robotx sh -c 'cd /opt/robotx/RobotX-Pi && PYTHONDONTWRITEBYTECODE=1 venv/bin/python -c "import robotx.application.main, robotx.diagnostics.faults; print(\"import ok\")"'
#   must print: import ok (before any restart)
sudo git -c safe.directory=/opt/robotx/RobotX-Pi -C /opt/robotx/RobotX-Pi rev-parse HEAD          # must print <COMMIT>
sudo git -c safe.directory=/opt/robotx/RobotX-Pi -C /opt/robotx/RobotX-Pi status --porcelain      # must print nothing
sudo git -c safe.directory=/opt/robotx/RobotX-Pi -C /opt/robotx/RobotX-Pi diff --stat 77e92fb <COMMIT> -- requirements.txt
#   must print nothing: no dependency change, so the virtual environment stays as it is
```

The second `safe.directory` entry is there because root reads a repository
owned by `pi`. If git still refuses with "dubious ownership", stop: the
alternative -- pushing the branch, then section 24, step 5 as written -- needs
its own approval. Keep the previous commit (`77e92fb`) and a dated copy of the
environment file for rollback (section 7).

Then `sudo systemctl restart robotx-agent` (close any `/camera` viewer first).

## 5. Verification (all read-only or refused, nothing moves)

```bash
journalctl -u robotx-agent -n 50 --no-pager | grep -E 'test_injection\.(enabled|unavailable)|operator_access'
#   expect: [test_injection.enabled] ... ; no test_injection.unavailable, no api.operator_access_misconfigured
curl -s $S/config | jq '{test_injection_enabled, test_injection_token_sha256, operator_token_sha256, api_host, esp32_motion_enabled}'
#   expect: true, "SET", "SET", "127.0.0.1", false (motion stays off)
curl -s $S/test/injection | jq '{enabled, available, problem, locked_out}'
#   expect: true, true, null, false
curl -s -o /dev/null -w '%{http_code}\n' -X POST $S/mission/custody -H 'Content-Type: application/json' -d '{"kind":"ACQUIRED"}'
#   expect: 401 (no token)
curl -s -X POST $S/mission/custody -H @$OP -H 'Content-Type: application/json' -d '{"kind":"ACQUIRED"}'
#   expect: 409 "no mission that requires custody is in progress" (authorized, and no effect)
curl -s $S/backend | jq '{status, streaming, authenticated, server}'
#   expect: STREAMING, true, true, the Render URL
cd <harness checkout> && /opt/robotx/RobotX-Pi/venv/bin/python -m tests.mission_injection.preflight
#   expect: mission_ingress PASS; every hardware blocker still reported honestly
```

Do not verify `/safety/clear` with the token as a "harmless" call: it also
acknowledges a latched ESP32 reboot.

## 6. Running a test (only with separate authorization for a hardware test)

1. Write a fixture for the actual site (schema: `tests/mission_injection/fixture.py`;
   example: `tests/mission_injection/fixtures/example_mission.json`, whose
   coordinates are placeholders). A new `task_id` every time: an id is injected
   once, ever.
2. Preflight with the fixture; every required check must be PASS:
   `python -m tests.mission_injection.preflight --fixture FILE`
3. Inject:
   `python -m tests.mission_injection.inject --fixture FILE --authorize-hardware-test --token-file ~/.config/robotx/injection-token --watch-s 600`
4. Custody at each stop with the operator header (section 3), only for a real
   handover, under the runbook's custody rules.
5. Stop at any time: `curl -X POST $S/safety/estop` (or `/mission/stop`). A
   paused test mission cannot be resumed locally in engine mode: stop it and
   inject a new task id.

## 7. Rollback

Disable injection only (keeps the code):

1. The robot is IDLE (`/state .mission` is null or COMPLETE/ABORTED).
2. `sudoedit /etc/robotx/robotx-agent.env`: set `ROBOTX_TEST_INJECTION_ENABLED=0`
   (or delete the three `ROBOTX_TEST_INJECTION_*` lines). Keeping
   `ROBOTX_OPERATOR_TOKEN_SHA256` is recommended; removing it returns the four
   release routes to "any non-browser local caller".
3. `sudo systemctl restart robotx-agent`.
4. Expect `GET /test/injection` → 404 and `/config .test_injection_enabled == false`.

Return to the previous release (removes the code): deployment guide section 24
with target `77e92fb` and the backed-up environment file -- with the checkout
under `umask 022` and the same permission check as section 4. That code has no
operator guard and no Origin rule: the four release routes are then open to
any local caller and browser page again.

Never delete the provenance log or its `.torn-*` files: they are the only record
of what was injected, and deleting the log makes old task ids usable again.

## 8. Troubleshooting

| Symptom | Cause / action |
|---|---|
| `/test/injection` 404 | injection disabled, or the running code predates it |
| `available: false` with `problem` | the agent states the reason (API host, digest, operator token missing or equal, log unreadable or unwritable); fix and restart |
| 429 `LOCKED_OUT` | five wrong tokens in a row; wait 5 minutes |
| 409 `DUPLICATE_TASK` | the task id was used before; write a new one |
| 409 `REJECTED` with a reason | the agent's own verdict (`NO_MOTOR_LINK`, `NO_POSITION_FIX`, ...); nothing was assigned |
| `test_injection.torn_record_quarantined` in the journal | an interrupted last record was moved to `<log>.torn-<ms>`; history before it is intact; keep the file |
| harness prints `OUTCOME UNKNOWN` | the answer was lost; check `/state .mission.task_id` and the journal before anything else |
