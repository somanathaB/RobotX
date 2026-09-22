# RobotX Pi Agent — Architecture Refactor Report

**Date:** 2026-09-22
**Type:** Structural reorganization only (folder/file/module names, import wiring). No behavior, wiring logic, safety thresholds, GPIO pin assignments, or Socket.IO protocol were changed.
**Reference plan:** `ROBOTX_PI_ARCHITECTURE_REFACTOR_PLAN.md`
**Git history:** 4 commits on `master`, each independently revertable — `8e8438f` (checkpoint, unmodified tree) → `e41f28a` (file moves/renames) → `4ff3f69` (documentation) → `ed0b384` (.gitignore).

---

## 1. Original Structure

```
RobotX/
├── test_motor.py, test_gps.py, test_ultrasonic.py, test_controller.py, test_cv.py, test_vision.py   (repo root)
├── frame.jpg, frames/frame.jpg   (stray debug artifacts)
└── robotx/
    ├── app/            main.py, sockets.py
    ├── control/        controller.py, decision_engine.py, vision_controller.py
    ├── hardware/       motors.py, encoders.py, ultrasonic.py, ir.py
    ├── navigation/     gps.py, maps.py, planner.py
    ├── perception/     camera.py, detection.py, tracking.py, filter.py
    └── utils/          config.py
```

## 2. Final Structure

```
RobotX/
├── docs/architecture/          ROBOTX_PI_ARCHITECTURE.md, DEPENDENCY_MAP.md   (new)
├── tests/hardware/              6 hardware-exercise scripts (moved, imports updated)
└── robotx/
    ├── application/            app.py   (was app/main.py)
    ├── communication/          socket_client.py   (was app/sockets.py)
    ├── config/                 settings.py   (was utils/config.py)
    ├── control/                robot_controller.py   (was control/controller.py)
    ├── hardware/                motors.py, encoders.py, ultrasonic.py, ir.py (unchanged)
    │                            + camera.py (was perception/camera.py)
    │                            + gps.py (was navigation/gps.py)
    ├── navigation/              route_planner.py (was planner.py), directions_client.py (was maps.py)
    └── perception/              object_detector.py (was detection.py), object_tracker.py (was tracking.py),
                                  temporal_filter.py (was filter.py)
                                  + vision_controller.py, decision_engine.py (were control/*, still experimental)
```

## 3. Every Renamed File

| Old | New |
|---|---|
| `robotx/utils/config.py` | `robotx/config/settings.py` |
| `robotx/control/controller.py` | `robotx/control/robot_controller.py` |
| `robotx/app/main.py` | `robotx/application/app.py` |
| `robotx/app/sockets.py` | `robotx/communication/socket_client.py` |
| `robotx/perception/detection.py` | `robotx/perception/object_detector.py` |
| `robotx/perception/tracking.py` | `robotx/perception/object_tracker.py` |
| `robotx/perception/filter.py` | `robotx/perception/temporal_filter.py` |
| `robotx/navigation/planner.py` | `robotx/navigation/route_planner.py` |
| `robotx/navigation/maps.py` | `robotx/navigation/directions_client.py` |

## 4. Every Moved File (moved between packages, name unchanged)

| File | Old package | New package | Reason |
|---|---|---|---|
| `camera.py` | `perception/` | `hardware/` | Picamera2 device I/O is a HARDWARE-boundary concern, not perception |
| `gps.py` | `navigation/` | `hardware/` | Serial NMEA device I/O is a HARDWARE-boundary concern, not navigation |
| `vision_controller.py` | `control/` | `perception/` | Imports only `perception.*`, never `hardware`/`control`; it is vision decision-making, not motor control |
| `decision_engine.py` | `control/` | `perception/` | Same rationale — zero internal imports, pure pixel-rule logic paired with `vision_controller.py` |

## 5. Files Moved Without Renaming (package rename only)

`motors.py`, `encoders.py`, `ultrasonic.py`, `ir.py` (stayed in `hardware/`, unchanged); `test_motor.py`, `test_gps.py`, `test_ultrasonic.py`, `test_controller.py`, `test_cv.py`, `test_vision.py` (repo root → `tests/hardware/`, content unchanged apart from import lines).

## 6. Newly Created Modules

**None.** This refactor created zero new source modules with new logic. The only new files are:
- `tests/__init__.py`, `tests/hardware/__init__.py` (empty, package markers)
- `docs/architecture/ROBOTX_PI_ARCHITECTURE.md`, `docs/architecture/DEPENDENCY_MAP.md` (documentation)
- `ROBOTX_PI_ARCHITECTURE_REFACTOR_PLAN.md`, this report (documentation)

A centralized `control/safety_controller.py` was explicitly considered and **not created** — see §15.

## 7. Deleted Files

| File | Justification |
|---|---|
| `frame.jpg`, `frames/frame.jpg` | Stray debug images written by manual `test_cv.py`/`test_vision.py` runs, not source code, regenerated on demand. Removed via `git rm` (recoverable from git history). Confirmed with the user before deletion. |
| `robotx/utils/__init__.py`, `robotx/app/__init__.py` | Package directories renamed (`utils/`→`config/`, `app/`→`application/`); their `__init__.py` files moved (git tracked as renames) rather than being separately deleted+created. |

No production logic file was deleted. `decision_engine.py`/`vision_controller.py` were relocated, not removed, and remain exactly as disconnected from production as before.

## 8. Import / Dependency Changes

Every `from robotx.X import Y` referencing a moved/renamed module was updated in: `robotx/application/app.py`, `robotx/control/robot_controller.py`, `robotx/perception/vision_controller.py`, `robotx/perception/temporal_filter.py`, and all six files under `tests/hardware/`. Full before/after table is in `ROBOTX_PI_ARCHITECTURE_REFACTOR_PLAN.md` §7. No third-party import changed. No circular imports were introduced (verified — see `docs/architecture/DEPENDENCY_MAP.md`).

## 9. Responsibility of Each Major Package

See `docs/architecture/ROBOTX_PI_ARCHITECTURE.md` §3 for the full table. Summary: `config` (env-var settings), `hardware` (GPIO/camera/serial device I/O), `navigation` (route math + Directions HTTP client), `perception` (frame interpretation + experimental vision-decision pipeline), `control` (the one production control loop), `communication` (Socket.IO transport), `application` (FastAPI + composition root).

## 10. Tests Run

Only non-hardware-touching checks were run, per the task's own constraint against automated hardware tests:

| Check | Result |
|---|---|
| `python3 -m py_compile` on every `.py` file (excluding `venv/`) | **PASS** — all files compile |
| Full-repo grep for stale old-module-path references in code and docs | **PASS** — zero stale references outside the intentional "before" column of the migration plan and the banner-noted historical audit |
| `venv/bin/python -c "import robotx.application.app; print('ok')"` | **PASS** — imports cleanly, no hardware threads started (only happens under `on_startup`, which only fires under uvicorn) |

**No `tests/hardware/*.py` script was executed.** Doing so would move motors, activate the camera, or open a real serial port — explicitly out of scope for this refactor's validation per the task's hardware-safety constraints. These require manual verification — see §17.

## 11. R-00 Verification

`venv/bin/python` permission mode confirmed unchanged at `775` (executable) before and after the refactor. Nothing under `venv/` was touched by any step of this refactor. The `-m` invocation form (`venv/bin/python -m uvicorn robotx.application.app:app ...`) was updated in `README.md` for the new module path but the invocation mechanism itself is identical to before.

**Status: VERIFIED INTACT.**

## 12. R-01 Verification

`diff` between the pre-refactor checkpoint's `robotx/control/controller.py` and the post-refactor `robotx/control/robot_controller.py`, from the first line after the import block onward, is **empty** — `_safety_blocked()`, `_avoid()`, `_apply_manual()`, and `handle_command()` are byte-for-byte identical, including the MANUAL-mode safety gate (`_apply_manual` still receives and obeys the `blocked` signal for forward-class motion). Only the file's own import lines (updated to the new module paths) and its filename changed.

**Status: VERIFIED INTACT.**

## 13. R-02 Verification

`diff` between the pre-refactor checkpoint's `robotx/app/sockets.py` and the post-refactor `robotx/communication/socket_client.py` is **empty** (this file had zero internal `robotx.*` imports to update, so it is fully unchanged). The `connect()` method still builds `auth={"robot_id": ..., "token": SETTINGS.robot_token}` and still logs the "unauthenticated" warning when no token is configured.

**Status: VERIFIED INTACT.**

## 14. R-03 Verification

`diff` between the pre-refactor checkpoint's `robotx/hardware/ultrasonic.py` and the post-refactor file at the same path is **empty** (this file was never moved or edited). The `UltrasonicStatus` enum (VALID/TIMEOUT/OUT_OF_RANGE/ERROR/DISCONNECTED/STALE/UNKNOWN) and `robot_controller.py`'s "any non-VALID status blocks" logic (verified identical per §12) are unchanged.

**Status: VERIFIED INTACT.**

## 15. Remaining Technical Debt

All items below are pre-existing (documented in `ROBOTX_DEEP_CURRENT_STATE_AUDIT.md` and `ROBOTX_PI_REMEDIATION_PLAN.md` as R-04 through R-10) and were **not** addressed by this structural refactor, per its explicit scope:

- Battery telemetry is a hardcoded constant (R-04).
- No comm-loss watchdog for stale MANUAL commands after a socket disconnect (R-06).
- No schema validation on inbound Socket.IO commands (R-07).
- No process supervision / systemd unit (R-08).
- `print()` calls in `perception/object_detector.py` hot path instead of `logging` (R-09).
- No `.env.example` (R-10).
- No automated `pytest` suite for pure-logic code (`haversine_m`, `RoutePlanner`, `decode_polyline`, `ObjectTracker`, `DecisionEngine`) — `tests/unit/`/`tests/integration/` were deliberately not scaffolded empty; this is real work for a future pass.
- `perception/object_detector.py` (889 lines) still contains both the OpenCV and YOLOv8 backends in one file — splitting into `perception/detectors/opencv_detector.py` + `yolo_detector.py` was flagged and explicitly deferred (see plan §11) as a non-trivial code edit to safety-relevant, zero-test-coverage code, not appropriate for a move-only pass.
- `tests/hardware/test_controller.py` remains misleadingly named (it does not exercise `RobotController`) — deliberately not renamed here because R-05 already records a specific remedy (docstring/header clarification) for this exact issue; renaming here would have preempted that separately-tracked decision.

## 16. Remaining Architectural Risks

- **`VisionController`/`DecisionEngine` promotion decision is still open.** They are now correctly located in `perception/` (matching their actual dependencies) but remain completely unreachable from the running application — a future contributor could still be confused about which obstacle-avoidance implementation is "real" without reading `docs/architecture/ROBOTX_PI_ARCHITECTURE.md` §3's explicit callout.
- **No centralized safety-gate module.** `_safety_blocked`/`_avoid`/`_apply_manual` remain inline methods on `RobotController` rather than an independently-testable `SafetyController`. This was a deliberate choice for this pass (confirmed with the user) to avoid reopening a design decision the remediation plan already recorded as explicitly deferred — but it means the safety logic still has zero unit-test coverage and can only be verified by reading `robot_controller.py` directly.
- **No CI/automated regression protection.** Because there is still no `pytest` suite, a future refactor could silently break `RoutePlanner`, `decode_polyline`, or the tracker/filter logic with no automated signal — only manual hardware testing or code review would catch it.
- **External Socket.IO server contract is unverified.** All of `communication/socket_client.py`'s auth behavior is unilateral; there is no way to test the R-02 handshake end-to-end without the external server, which remains out of this repo's scope.

## 17. Manual Hardware Verification Required

The following **must** be performed by a human on the physical robot before trusting this refactor in the field — none of them were run automatically, and none should ever be run as part of an automated check:

1. **`venv/bin/python -m uvicorn robotx.application.app:app --host 0.0.0.0 --port 8000`** — confirm the server starts, `/health` returns `{"ok": true, ...}`, and `/camera` streams MJPEG.
2. **`tests/hardware/test_motor.py`** (wheels off the ground) — confirm forward/stop/backward/stop behaves identically to before the refactor.
3. **`tests/hardware/test_ultrasonic.py`** — confirm distance readings are unaffected; unplug the sensor mid-run and confirm it reports a non-`None`/disconnected status rather than silently going quiet.
4. **`tests/hardware/test_gps.py`** — confirm NMEA fixes are still read from the configured serial port.
5. **`tests/hardware/test_controller.py`** (wheels off the ground) — confirm STOP/FORWARD decision logic is unchanged.
6. **`tests/hardware/test_cv.py`** — confirm the Picamera2 preview/detection still works from its new `hardware/camera.py` location.
7. **`tests/hardware/test_vision.py`** — confirm the experimental `VisionController`/`DecisionEngine` pipeline still runs from its new `perception/` location (still not connected to the production controller).
8. **End-to-end MANUAL-mode safety check on real hardware**: with an obstacle within `ROBOTX_OBSTACLE_DISTANCE_CM`, send a `MANUAL forward` command over Socket.IO (requires a test server or a manual `sio` client) and confirm the robot still refuses to move forward — this re-verifies R-01 behaviorally, not just by source diff.

## 18. Recommended Next Development Phase

Per the remediation plan's own execution order (unaffected by this refactor), and now that the package structure gives each of these an obvious home:

1. **R-06** (comm-loss watchdog) — lives naturally in `communication/socket_client.py` + a small addition to `control/robot_controller.py`.
2. **R-07** (command schema validation) — a new `robotx/communication/commands.py` at the socket boundary, as already anticipated in the updated remediation plan.
3. **A `tests/unit/` pytest suite** for `route_planner.py`, `directions_client.py::decode_polyline`, `object_tracker.py`, `decision_engine.py` — the new package boundaries make it obvious which hardware-free modules are testable in isolation.
4. **R-04** (real battery telemetry) — a new `robotx/hardware/battery.py`, following the same `Config`+reader-class pattern as `ultrasonic.py`/`ir.py`.
5. Revisit the `VisionController`/`DecisionEngine` promotion decision (R-05) only after (3) gives it test coverage to promote safely.

None of R-04 through R-10 should be started without a separate, explicit go-ahead — this refactor's scope was structural only, and the code's physical behavior must remain exactly what it was before this pass until a deliberate, reviewed change is made.
