# Bench motor pulse — the first physical wheel test

One short, slow, forward pulse for a rover **on blocks**, to prove the
Pi → ESP32 → motor-driver path: a DRIVE leaves the Pi, the ESP32 ACCEPTS it and
reports the output applied, the wheels turn, and a STOP stops them.

Code: `robotx/control/bench_motion.py` (the pulse), `robotx/application/bench_gate.py`
(the route), `RobotAgent.start_bench_motion` (the preconditions). Tests:
`tests/unit/test_bench_motion.py` (SIMULATED firmware; a pass is not wheel evidence).

## What it is, and is not

- **Not a mission.** No route, no taskId, no commitment; nothing is reported to the
  backend as an assignment. TELEMETRY stays `status: IDLE` and `faults[]` carries
  `BENCH_MOTION_ACTIVE` (INFO, MOTOR) while it runs. An OFFER that arrives during a
  pulse is refused (`BENCH_TEST_ACTIVE`). The one exception is the bench
  wheel-test demonstration (`BENCH_WHEEL_DEMO.md`), where a pulse is bound to an
  offer accepted without a mission.
- **Not a bypass.** The pulse proposes `forward(speed)` with equal sides. The
  camera's own stops (perception unusable, a person, an object in the corridor),
  then the same `SafetyGate` a mission uses (e-stop, perception, forward range at
  `ROBOTX_SAFETY_STOP_DISTANCE_CM`, speed ceiling), then `Esp32Link` (motion enabled,
  motion-ready controller), then the ESP32's own gates — all still apply.
- **One pulse, then nothing.** It never starts on its own; at most one runs; a 10 s
  cooldown follows each. Any stop ends it for good: **clearing an obstacle restarts
  nothing** (there is no obstacle-recovery hold for a pulse).

## It stops at once when

the deadline passes (`duration_s` ≤ `ROBOTX_BENCH_MOTION_DURATION_S` ≤ 3 s); the
camera or the gate turns it into a stop; the ESP32 link stops being motion-ready
(stale TELEMETRY, `front_valid=false`, `safety_stop`, `command_timeout`, drive
unavailable, reboot latch) or is lost and re-proven; a DRIVE ACK is not ACCEPTED, or
its `req`/`applied` differ from what was sent; no ACK arrives within 0.5 s; fresh
TELEMETRY shows applied output outside `[0, sent]`, or none 0.6 s after the ACK;
the applied-motion monitor faults; an e-stop, `/mission/stop`, `/mission/pause` or a
mission arrives.

Then the STOP must be **confirmed**: TELEMETRY received after the STOP reporting
commanded and applied 0/0. If not within 1.5 s the result is `STOP_UNCONFIRMED` and
the agent **latches its emergency stop**.

`applied` is what the firmware says it outputs. It is never proof a wheel turned;
the person watching the wheels is the only witness of that.

## Eligibility — all must be true before any configuration change

| Prerequisite | How it is checked | Kind |
|---|---|---|
| An independent motor-power cutoff is fitted and attended | **a person**, then `ROBOTX_BENCH_MOTOR_CUTOFF_CONFIRMED=1` | physical — software cannot check it |
| Drive wheels off the ground, rover secured | a person; restated in every request | physical |
| Both front ultrasonic sensors stable (`front_valid` true continuously) | `GET /state` over a minute | physical |
| Nothing within 90 cm ahead of the front sensors; nobody in the camera view | `GET /state` front distances; perception OK | physical |
| ESP32 link UP, fresh, bidirectional; drive available; no reboot latch, no `safety_stop`, no `command_timeout` | `GET /state` controller | live |
| Rover IDLE, no mission, no backend commitment, no e-stop | `GET /readiness`, `GET /backend` | live |

If the first row is not true, **stop here**: pulling the battery is then the only
physical stop, and it cuts the Pi and the ESP32 too.

## Configuration (separately approved; motion stays off in normal service)

Create two tokens (operator, bench), keep them in `~/.config/robotx/` mode 0600, and
configure only their SHA-256 digests (TEST_MISSION_INJECTION.md section 2 shows how).
Add with `sudoedit /etc/robotx/robotx-agent.env` (dated copy first):

```
ROBOTX_ESP32_MOTION_ENABLED=1
ROBOTX_OPERATOR_TOKEN_SHA256=<operator digest>
ROBOTX_BENCH_MOTION_ENABLED=1
ROBOTX_BENCH_MOTION_TOKEN_SHA256=<bench digest>
ROBOTX_BENCH_MOTOR_CUTOFF_CONFIRMED=1
```

`sudo systemctl restart robotx-agent`, then expect `GET /test/bench-motion` →
`available: true, problem: null`.

## Running it

```bash
S=http://127.0.0.1:8000; B=~/.config/robotx/bench-token
curl -s $S/test/bench-motion | python3 -m json.tool                       # available, phase IDLE
curl -s -X POST $S/test/bench-motion/run -H "Authorization: Bearer $(cat $B)" \
     -H 'Content-Type: application/json' \
     -d '{"wheels_off_ground": true, "motor_cutoff_attended": true, "duration_s": 1.0}'
#   200 phase DRIVING, or 409 {code, detail}: the precondition that failed
sleep 3; curl -s $S/test/bench-motion | python3 -m json.tool              # last_result
```

Stop at any time: `curl -X POST $S/safety/estop` (latches; clearing it is an operator
act) or `curl -X POST $S/mission/stop`. Physical stop: the motor cutoff.

**Pass:** `last_result.outcome == "COMPLETED"`, `stop_confirmed: true`, every
`drive_acks[].result == "ACCEPTED"` with `req == applied == sent_units`,
`max_telemetry_applied == sent_units`, **and the person saw every drive wheel turn
forward and stop**. Record the wheel direction per side.

**Fail (do not repeat until explained):** any other outcome; a wheel not turning or
turning backwards; motion continuing after the stop; an ESP32 reboot during the
pulse (suspect supply brown-out under motor load).

Then set `ROBOTX_ESP32_MOTION_ENABLED=0` and `ROBOTX_BENCH_MOTION_ENABLED=0` again
and restart, unless the next approved test follows immediately.
