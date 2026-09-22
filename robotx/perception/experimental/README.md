# Experimental perception (not production)

## Status

**NOT WIRED INTO THE PRODUCTION AGENT.** Nothing in `robotx/application/`,
`robotx/control/decision.py`, or `robotx/perception/pipeline.py` imports this
package. The only caller is `tests/perception/test_vision.py`, a manual bench
script that requires the real Raspberry Pi camera.

## What is here

| Module | What it does |
|---|---|
| `vision_controller.py` | A full camera → detect → track → temporal-filter → decision pipeline with a continuous-turn avoidance state machine, hysteresis, and an area-trend ("is the object growing") proxy. |
| `decision_engine.py` | A 36-line fixed-threshold rule engine paired with the above. |

## Why it is kept rather than deleted

It is real, working code that encodes bench-tuning effort: hysteresis frame
counts, decision-buffer voting, and an area-growth heuristic that the
production path does not have. Deleting it would throw that away.

## Why it is not the production path

1. **Hard-coded pixel geometry.** Zone boundaries (`213` / `426`) and area
   thresholds (`30000` / `10000`) are literals tied to a 640 px frame and to one
   specific camera mounting. They were never re-derived for this robot's actual
   camera height and angle.
2. **A different safety philosophy.** It turns continuously past obstacles
   without stopping first. The production decision layer stops first. Running
   both would make "which one is actually protecting the robot" ambiguous —
   which is exactly the duplicate-pipeline problem this restructuring removed.
3. **No hardware evidence.** No bench-test record exists showing its thresholds
   are correct for this robot, so promoting it would change robot behaviour on
   an untested basis.

## If you want to promote it

Re-derive every threshold from the configured frame size (they should be
ratios in `Settings`, not literals), bench-test against the real camera at the
real mounting height, and then replace `robotx/control/decision.py` — do not
run both pipelines at once.
