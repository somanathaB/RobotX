# Bench wheel-test demonstration (RBT-2000)

> **A rover on a stand, not a delivery.** RobotX assigns a real task and the Rover
> accepts the signed OFFER **without moving**. Then, as a separate operator act, one
> short, slow, forward wheel pulse is run *for that task* through the normal safety
> gate and ESP32 link. Navigation, perception-guided driving, custody and delivery are
> not demonstrated and are never started in this mode. Off by default.

Code: `RobotAgent.accept_offer`, `start_bench_motion`, `end_bench_task`,
`_check_bench_binding` (`robotx/application/agent.py`); `BackendLink._apply_envelope`
(`robotx/communication/backend_link.py`); the pulse itself is the existing
`robotx/control/bench_motion.py` (see `BENCH_MOTION_TEST.md`; every stop condition
there still applies). Tests: `tests/unit/test_bench_wheel_demo.py` (SIMULATED).

## Do not run it unless all of these are true

| Prerequisite | How it is checked | Status on record |
|---|---|---|
| An **independent motor-power cutoff** (switch or connector in the L298N motor supply, reachable by a person who does nothing else) is fitted and **tested once with the motors unpowered** | a person; then `ROBOTX_BENCH_MOTOR_CUTOFF_CONFIRMED=1` | **NOT fitted or verified** (hardware audit 2026-10-12). Every software stop goes through the ESP32. |
| Chassis in a stable stand, **all drive wheels clear of every surface**, hands, wires and clothing clear of the wheels | a person, restated in every request | physical |
| **Both** front ultrasonics valid (`front_valid` true for a minute) and nothing within 90 cm ahead of them | `GET /state` | the right-front sensor was recorded faulty (fw 8de2162); with `SAFETY_REQUIRE_BOTH_SENSORS 1` the ESP32 then refuses forward output, the Rover reports `NO_MOTOR_LINK` and **rejects the offer** |
| Camera perception usable (`perception.status` OK) | `GET /state` | needed by the pulse, not by acceptance |
| The offer-acceptance prerequisites (measured SoC, hAcc ≤ 2.5 m, pickup ≤ 300 m, ESP32 UP, latch released) | dashboard, `GET /readiness` | as for `OFFER_ACCEPTANCE_DEMO.md` |

If the first row is not true: **do not energise the motors.** Use the non-moving
fallback, `OFFER_ACCEPTANCE_DEMO.md` (motion disabled in software, accepted offer,
mode IDLE). Pulling the battery is then the only physical stop, and it also cuts
the Pi and the ESP32.

## What changes, and what does not

| | Offer-acceptance demo | Bench wheel-test demo |
|---|---|---|
| OFFER admission, HMAC, fence, route/start checks, e-stop, ERROR, position fix | unchanged | unchanged |
| Motor link (`NO_MOTOR_LINK`) | only the link UP | **full check**: motion enabled and the ESP32 motion-ready |
| Custody sensing, perception | not offer blockers | not offer blockers (the pulse checks perception itself) |
| On ACCEPT | nothing started, IDLE | nothing started, IDLE; the task is remembered as eligible |
| Wheel motion | impossible (motion disabled) | **only** `POST /test/bench-motion/run` naming the accepted `task_id` |
| Missions, local `/mission/start`, RESUME, RETURN | refused | refused (`BENCH_WHEEL_DEMO` / `MissionRefused`) |
| Lease | not renewed: RECALL after ~60 s | the same; RECALL also ends a running pulse |

A pulse is refused unless the `task_id` was accepted **by this agent process**
(a restart forgets it), its commitment is still held (not recalled, withdrawn or
ended), the backend link is authenticated, and every `BENCH_MOTION_TEST.md`
precondition holds. At most one pulse runs, then a 10 s cooldown.

## How a running pulse stops

| Event | What the Pi does | Proven in |
|---|---|---|
| Its deadline (`duration_s` ≤ `ROBOTX_BENCH_MOTION_DURATION_S`, hard cap 3 s) | STOP, then waits for TELEMETRY 0/0 | unit + host sim |
| Engine `RECALL` / `WITHDRAW` / `ABORT_MISSION` for that commitment | `TASK_CANCELLED`: STOP, COMMAND_ACK, commitment tombstoned | unit + host sim (RECALL) |
| The commitment ends by any other path | `TASK_RELEASED` on the next tick | unit |
| Backend `COMMAND` STOP (dashboard robot STOP) or bare `STOP` event | `OPERATOR_STOP` | unit + host sim (COMMAND) |
| Backend link drops | `BACKEND_LINK_LOST` | unit |
| Local `POST /mission/stop`, `/mission/pause`, `/safety/estop` | `OPERATOR_STOP` / `OPERATOR_PAUSE` / `ESTOP` | unit (existing) |
| Obstacle, person, perception, front range, controller fault, ESP32 link loss, DRIVE not ACCEPTED, applied ≠ sent | the existing bench stop conditions | unit (existing) |
| ESP32 reboot | e-stop latched; nothing restarts | unit (existing) |
| The STOP is never confirmed within 1.5 s | `STOP_UNCONFIRMED`, e-stop latched | unit |
| **The Pi dies or the UART is cut** | nothing (it is gone): the **ESP32 watchdog** zeroes the outputs 2.0 s after the last DRIVE | host sim: COMMAND_TIMEOUT 1.98 s, applied 0/0 at 2.0 s |
| **The ESP32 resets, browns out or hangs** | — | **nothing in software covers this**: the motor-power cutoff |

Nothing ever restarts a pulse: not a reconnect, a cleared obstacle, a cleared e-stop
or a reboot.

### What a dashboard "Cancel task" does

**Nothing reaches the robot.** For an engine task whose Leg is accepted,
`POST /api/tasks/:taskId/cancel` answers 409 `ENGINE_CANCELLATION_UNAVAILABLE` and
writes and sends nothing (`taskCancellation.service.js`: only QUEUED or an
exhausted REASSIGNING Leg is cancellable, and neither path emits STOP). What does
reach the Pi: the engine's RECALL when the unrenewed lease lapses (~60 s after
ACCEPT), and the dashboard's robot **STOP** (`POST /api/robots/RBT-2000/command`
`{"type":"STOP"}`, which also places a server-side PAUSED hold). Use the robot
STOP to demonstrate cancellation during a pulse.

## Configuration (separately approved; motion stays off in normal service)

Tokens and digests as in `BENCH_MOTION_TEST.md`. In `/etc/robotx/robotx-agent.env`
(dated copy first):

```
ROBOTX_BENCH_WHEEL_DEMO=1
ROBOTX_OFFER_ACCEPTANCE_DEMO=0
ROBOTX_ESP32_ENABLED=1
ROBOTX_ESP32_MOTION_ENABLED=1
ROBOTX_SOCKET_ENABLED=1
ROBOTX_OPERATOR_TOKEN_SHA256=<operator digest>
ROBOTX_BENCH_MOTION_ENABLED=1
ROBOTX_BENCH_MOTION_TOKEN_SHA256=<bench digest>
ROBOTX_BENCH_MOTOR_CUTOFF_CONFIRMED=1      # only once the cutoff is fitted and tested
ROBOTX_BENCH_MOTION_SPEED=0.20             # 51 of 255, just above the 40-unit deadband
ROBOTX_BENCH_MOTION_DURATION_S=1.0
```

`sudo systemctl restart robotx-agent`. The agent refuses to start in this mode
unless motion, the bench route and the backend are all enabled and the
offer-acceptance demo is off. Expect `agent.bench_wheel_demo` at WARNING and
`GET /test/bench-motion` → `available: true`, `wheel_demo.enabled: true`.

## Supervised procedure

Two people: the **cutoff holder** (hand on the motor-power cutoff, does nothing
else) and the **operator**.

1. Motors unpowered (cutoff open). Rover on the stand, wheels clear. Agent running.
   `GET /state`: ESP32 UP, `front_valid` true, front distances > 90 cm, perception OK,
   no e-stop, mode IDLE.
2. Close the cutoff. Watch the wheels for 10 s: **nothing may turn**. If anything
   moves, open the cutoff and stop.
3. Create the task in the dashboard. Expect `offer.accepted_without_motion` in the
   Pi log and the task ASSIGNED to RBT-2000. **Wheels still.** The lease runs out
   ~60 s after ACCEPT: do steps 4–5 inside that window.
4. `GET /test/bench-motion`: `wheel_demo.held_task_id` is the task.
5. Run one pulse:

   ```bash
   S=http://127.0.0.1:8000; B=~/.config/robotx/bench-token; T=<taskId>
   curl -s -X POST $S/test/bench-motion/run -H "Authorization: Bearer $(cat $B)" \
        -H 'Content-Type: application/json' \
        -d "{\"wheels_off_ground\": true, \"motor_cutoff_attended\": true, \"task_id\": \"$T\", \"duration_s\": 1.0}"
   ```

   To show cancellation instead, start a `duration_s` 2.0 pulse and press the robot
   **STOP** in the dashboard at once (or `curl -X POST $S/mission/stop`). A STOP
   leaves the agent STOPPED; another pulse needs `POST /mission/idle` (operator
   token) first, and the task must still be held.
6. `GET /test/bench-motion` → `last_result`. **Pass:** `outcome` COMPLETED (or ABORTED
   with `OPERATOR_STOP` / `TASK_CANCELLED`), `stop_confirmed: true`, every
   `drive_acks[].result` ACCEPTED with `req == applied`, the task id recorded, **and
   both people saw every drive wheel turn forward slowly and stop**.
7. Open the cutoff. Restore `ROBOTX_BENCH_WHEEL_DEMO=0`, `ROBOTX_ESP32_MOTION_ENABLED=0`,
   `ROBOTX_BENCH_MOTION_ENABLED=0`; restart. Clear the dashboard robot hold
   (`clear-fault`) if STOP was used.

**Abort at once** (open the cutoff) on any of these: a wheel turning before step 5
or after the stop; turning backwards; an ESP32 reboot during the pulse (brown-out
under load); `STOP_UNCONFIRMED`. Do not repeat until explained.

## What it is not evidence of

Autonomous driving, obstacle handling, navigation, custody or delivery. The
`applied` values are what the ESP32 says it drove; only the people watching are
witnesses that a wheel turned.
