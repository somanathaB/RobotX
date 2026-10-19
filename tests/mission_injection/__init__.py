"""Test-only mission injection: supply the one input RobotX cannot, nothing else.

Why this exists
---------------
The RobotX backend's Assignment Engine does not currently assign tasks to
physical robots, so the Rover never receives an OFFER. This package supplies
that missing assignment from a local fixture, for a physical integration test,
and nothing else: perception, localization, navigation, the safety gate, the
ESP32 link and the backend telemetry all stay the Rover's own.

What it is not
--------------
- Not a backend. It never signs, forges or replays an OFFER, never opens a
  socket, and never claims RobotX assigned anything.
- Not a mission engine. The agent decides: the harness posts the fixture to
  the agent's own opt-in, loopback-only, token-authorized ingress
  (`POST /test/mission/inject`, `robotx.application.injection`), which calls
  `assess_offer` -> `assign_mission(custody_required=True)` in the agent's
  process. Lifecycle is read back from the agent's state, never inferred.
- Not part of production startup. Nothing under `robotx/` imports this package.

Modules
-------
- `fixture`   -- loads a fixture file; the schema is the agent's own.
- `ledger`    -- the harness's append-only JSONL test log; one attempt per task id.
- `lifecycle` -- records the state transitions the agent actually reports.
- `preflight` -- non-motion checks of the running agent over GET-only loopback.
- `inject`    -- CLI for a physical run: preflight, then one authorized POST.
"""
