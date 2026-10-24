# Offer-acceptance demonstration (RBT-2000)

> **Assignment only, never motion.** With this mode on, the Rover proves that
> RobotX can select it and that it can authenticate, verify and accept a real
> signed OFFER. It does not drive, and it cannot: the agent refuses to start in
> this mode unless every motor command path is disabled. Off by default.

Code: `RobotAgent.accept_offer`, `readiness_blockers`, `assess_offer`
(`robotx/application/agent.py`). Tests: `tests/unit/test_offer_acceptance_demo.py`,
and end to end against a real backend `tests/integration/offer_acceptance_demo_harness.py`
(driven by the backend's `tools/verify/gate3PiSimOffer.js --offer-acceptance-demo`).

## What changes, and what does not

| | Normal | Demonstration |
|---|---|---|
| OFFER admission (addressee, expiry, HMAC, fence, sequence, persist first) | yes | **unchanged** |
| Route consistency, start offset (25 m), pickup distance (300 m) | refused if not met | **unchanged** |
| E-stop latched, agent ERROR | refused | **unchanged** |
| ESP32 link not UP | refused (`NO_MOTOR_LINK`) | **unchanged** (the backend's F7 reads the stop latch from it) |
| No measured fix | refused (`NO_POSITION_FIX`) | **unchanged** |
| ESP32 UP but motion not enabled/ready | refused (`NO_MOTOR_LINK`) | not a reason: nothing will move |
| No custody confirmation path | refused (`NO_CUSTODY_SENSING`) | not a reason: no parcel changes hands |
| Camera/perception unusable | refused (`PERCEPTION_UNAVAILABLE`) | not a reason: nothing is driven |
| On ACCEPT | mission loaded, mode AUTO, drives | **nothing started**: mode IDLE, no route |
| HEARTBEAT after ACCEPT | renews the lease | bare: the backend recovers the Leg after `lease.duration` (60 s) |

The TELEMETRY `readiness.blockers` report follows the same rules, so the
backend is told exactly what the Rover would refuse. Any other caller of
`assign_mission` (test mission injection) is refused `OFFER_ACCEPTANCE_DEMO`.

## Enable (in `/etc/robotx/robotx-agent.env`, then restart the service)

```
ROBOTX_OFFER_ACCEPTANCE_DEMO=1
ROBOTX_ESP32_MOTION_ENABLED=0
ROBOTX_BENCH_MOTION_ENABLED=0
ROBOTX_ESP32_ENABLED=1          # the link must be UP: stop latch and GPS come through it
```

The service logs `agent.offer_acceptance_demo` at WARNING on start, and
`offer.accepted_without_motion` (with the task id) for every accepted offer.
With motion enabled the agent refuses to start with a `ValueError`.

## Disable

Remove `ROBOTX_OFFER_ACCEPTANCE_DEMO` (or set it to `0`) and restart. Normal
behaviour is exactly as before; nothing persists from the mode.

## What it is not evidence of

Navigation, obstacle handling, custody or delivery. The backend still sees a
Rover that accepted a Leg and then did not progress; cancel the task after the
demonstration, or let the lease lapse.
