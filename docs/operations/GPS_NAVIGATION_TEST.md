# GPS navigation: pipeline, simulation and the first physical test

**Status (2026-10-20): validated in simulation only. The rover has not
demonstrated controlled movement or direction on the ground.** Nothing in this
document is hardware evidence until section 6 has been carried out and recorded.

## 1. The pipeline, task to wheel

| # | Stage | Code | Configuration | Tests |
|---|---|---|---|---|
| 1 | Signed OFFER over Socket.IO | `BackendLink._on_engine_command` → `_apply_envelope` → `_respond_to_offer` (`robotx/communication/backend_link.py`); `parse_envelope`, `verify_signature`, `parse_offer` (`robotx/communication/engine.py`) | `ROBOTX_COMMAND_SIGNING_KEY`, socket settings | `test_engine_offer`, `test_simulated_delivery` |
| 1′ | Test injection (loopback, token, off by default) | `InjectionGate.inject` (`robotx/application/injection.py`), `POST /test/mission/inject` | `ROBOTX_TEST_INJECTION_*` | `test_injection_endpoint`, `scripts.simulated_mission` |
| 2 | Validation and admission | `offer_to_mission`, `Mission.create`; `RobotAgent.assess_offer` → `readiness_blockers`, `check_route_consistency`, `check_start_position` (`robotx/mission/admission.py`); `assign_mission` | `ROBOTX_MISSION_MAX_START_OFFSET_M` (25), `_PICKUP_DISTANCE_M` (300), `_ENDPOINT_OFFSET_M` (50) | `test_mission`, `test_mission_admission`, `test_readiness_telemetry` |
| 3 | Waypoints | The backend's `stopSequence[].path`, executed as sent: `MissionManager.assign/update` (`robotx/mission/manager.py`), `Navigator.set_route`, `RoutePlanner` (`robotx/navigation/route_planner.py`) | `ROBOTX_NAV_WAYPOINT_ARRIVAL_M` (8), `_OFF_ROUTE_M` (25) | `test_navigation`, closed loop `test_waypoints_are_consumed_in_order` |
| 4 | GPS ingestion and freshness | ESP32 `GPS` frame → `Esp32Link.gps_sample` → `reading_from_frame` (`robotx/localization/esp32_gps.py`): `gps_status OK`, `fix_ok`, fix type, `hacc_mm`, age; STALE after 5 s | `ROBOTX_GPS_SOURCE=esp32`, `ROBOTX_GPS_STALE_AFTER_S` (5), `ROBOTX_NAV_MAX_H_ACC_M` (5) | `test_esp32_gps_path`, closed loop GPS faults |
| 5 | Heading and direction | `PositionEstimator.update` (`robotx/localization/position.py`); `bearing_deg`, `haversine_m`, `heading_error_deg`; `Navigator.update` | `ROBOTX_HEADING_MIN_SPEED_MPS` (0.5), `_MIN_MOVE_M` (1.5), `_HACC_FACTOR` (2) | `test_gps_and_position`, `TestDistanceAndBearing` |
| 6 | Motion intent | `DecisionMaker.decide` → `_follow_route` (`robotx/control/decision.py`); `MotionIntent.forward` | `ROBOTX_MOTION_*_SPEED`, `ROBOTX_NAV_TURN_RATE_DPS` (90), `ROBOTX_NAV_MAX_TURN_S` (2) | `test_motion_and_decision`, `test_gps_navigation_closed_loop` |
| 7 | Pi safety and transmission | `SafetyGate.evaluate` (`robotx/control/safety.py`), `front_range` (`robotx/esp32/front_range.py`), `ObstacleRecovery`, `ProgressMonitor`; `Esp32Link.submit` → `_motion_ready` → `_transmit`, `to_drive_units` (`robotx/esp32/link.py`) | `ROBOTX_ESP32_MOTION_ENABLED` (**0**), `ROBOTX_SAFETY_STOP_DISTANCE_CM` (90) | `test_safety_gate`, `test_esp32_link`, `test_obstacle_recovery` |
| 8 | ESP32 ACK and applied output | `Esp32Link._on_ack` (DRIVE ACK ACCEPTED/GATED/REJECTED), `ControllerTelemetry` `left_applied/right_applied` (`robotx/esp32/state.py`), `AppliedMotionMonitor` (`robotx/control/applied_motion.py`) | `ROBOTX_MOTION_APPLIED_TIMEOUT_S` (1.5), `_DEADBAND_UNITS` (40) | `test_applied_motion`, `simulated_mission` `applied-zero`, `gated` |
| 9 | Wheel movement and direction | **Nothing on the rover measures it.** No encoders, IMU or compass. Only GPS displacement, afterwards | — | **NOT TESTED** |
| 10 | Progress, arrival, stop | `RoutePlanner.arrived`, `MissionManager.update`, `ProgressMonitor` (`robotx/navigation/progress.py`), `stop_mission`/`pause_mission`, `emergency_stop` | `ROBOTX_NAV_HOLD_ASSIST_AFTER_S` (30), `_NO_PROGRESS_AFTER_S` (60), `_PROGRESS_MIN_M` (3) | `test_navigation_progress`, `test_simulated_delivery`, `test_mission_recovery` |

The agent runs stages 4 to 8 once per tick (`RobotAgent.tick`, 10 Hz) in this
order: GPS → position → ESP32 status → navigation → mission → decision → gate →
obstacle and progress monitors → `Esp32Link.submit` → heading invalidation.

## 2. Heading when stationary

The rover has no compass, IMU or magnetometer. **A stationary rover has no
heading** (`heading_source: NONE`). That is what `/state` reports, and it is
correct. Two GPS sources exist, and both describe the direction the rover
*moved*:

- `NMEA_TRACK`: the receiver's heading of motion, used only at ≥ 0.5 m/s.
  At the live speed reading of ~0.02 m/s it is noise, and it is ignored.
- `GPS_TRACK`: the bearing between two fixes at least `max(1.5 m, 2 × h_acc)`
  apart, counted only while the wheels were driven forward and the ESP32
  reported forward applied output. At h_acc 2 m that is 4 m. At 4.5 m it is 9 m.

So navigation starts blind. The decision layer **creeps straight** at
`motion_slow_speed` (0.25 → DRIVE 64/64) until a track exists. It then makes
**one bounded turn** toward the target: one side at the commanded speed, the
other stopped, for `error / ROBOTX_NAV_TURN_RATE_DPS` seconds (≤ 2 s). The
agent drops the heading on every steered tick, and the rover creeps straight
again to measure the result. It never pivots or reverses.

### The defect this replaced

Until this change the decision layer steered *continuously* on the held
heading, which is re-measured only every few metres. A saturated steer turns
the rover at 40–120°/s while that figure stays frozen, so the error never
shrinks and the rover circles. In the differential-drive simulation every
case circled: aligned, 30°, 90° and 180° starts, on two drivetrains. That was
4–34 full rotations, 0 of 16 arrivals, each ended by the 60 s no-progress
pause. Steering *signs* were correct throughout. The loop was unstable, not
backwards. A second defect: after a turn, the agent re-reads the last GPS frame
for up to a second, and that frame's course could immediately restore the
pre-turn heading. Headings now come only from fixes taken after the turn.

## 3. Simulation

```bash
venv/bin/python -m scripts.run_tests navigation                 # includes the closed loop
venv/bin/python -m unittest tests.unit.test_gps_navigation_closed_loop
venv/bin/python -m scripts.gps_navigation_sim                    # scenario table
venv/bin/python -m scripts.gps_navigation_sim --trace target-right-60
venv/bin/python -m scripts.gps_navigation_sim --turn-rate 50 --track-m 0.45 --h-acc 3
```

`tests/fixtures/diff_drive_sim.py` runs the real stages 4–7 and 10 in virtual
time. Wheel outputs set a hidden yaw. GPS reports a correlated position error,
the stated h_acc, and a heading of motion that is noise at a standstill. The
drivetrain numbers (side speed at DRIVE 255, track width) are **invented**
until measured.

Results with the default assumptions (0.6 m/s at 255, 0.30 m track, h_acc 2 m):

| Scenario | Outcome |
|---|---|
| aligned, target 60° right or left, 90° right | arrive; first turn on the correct side; < 0.5 rotations; no reverse or pivot DRIVE |
| target behind (180°) | paused by the no-progress monitor at 60 s (bounded, safe). It arrives on the faster drivetrain |
| L-shaped multi-waypoint route | waypoints consumed in order; arrives |
| GPS outage 20 s / inaccurate fix (6 m) 15 s | DRIVE 0/0 throughout (`no GPS position` / `GPS fix not accurate enough`), then continues and arrives |
| GPS outage > 35 s | paused: `no usable GPS position for 30s` |
| range veto 15 s (50 cm) | DRIVE 0/0 throughout, then continues |
| operator stop | DRIVE 0/0 from then on, route cleared |
| assumed turn rate 4× too low | overshoots every turn, never arrives, paused at 60 s, no reverse |

Across 3 drivetrains × 6 seeds, starts within ±45° of the route arrived every
time at h_acc 2 m. Starts ≥ 60° off depend on the turn-rate calibration. At
h_acc 4.5 m about half the non-aligned starts pause.

Mission lifecycle at the agent and backend level is covered by
`tests/integration/test_simulated_delivery.py`: genuine signed OFFER,
PAUSE/RESUME, WITHDRAW, task cancellation, backend loss and the lease,
e-stop, and ESP32 reboot. The same holds for `scripts.simulated_mission`.

## 4. Three different things, never to be confused

| Layer | Where it is visible | What it proves |
|---|---|---|
| Generated intent | `/state` `motion_intent`, `safety` (verdict/rule) | what the Pi *wants*, after its gate |
| ESP32-applied output | `/state` `controller.last_motion_sent`, `last_drive_ack` (ACCEPTED/GATED/REJECTED), `telemetry.motor.left_applied/right_applied` | that the ESP32 *received* the DRIVE and drives its PWM outputs |
| Physical wheel movement | an attending operator's eyes, and GPS displacement afterwards | that wheels turned, on the right side, in the right direction |

As of this writing, only the first layer has been exercised on the rover:
`drive_acks` 0, `last_motion_sent` null. The second and third layers have only
been exercised in simulation.

## 5. Readiness gates and where their evidence is

Blocking a physical test (live `GET /readiness`, `/state`, journal, 2026-10-20 11:16):

| Gate | Evidence | Kind of work |
|---|---|---|
| `ESTOP_LATCHED` / reboot latch | ESP32 rebooted at 11:11:01 (uptime 256968 → 320 ms). The link dropped TELEMETRY for ~1 s at 11:09:15, 11:09:56 and 11:10:57 before that | physical/firmware: the reset cause is still unknown (see the ESP32 reboot investigation). Clear only after it is understood |
| `NO_MOTOR_LINK` | `esp32_motion_enabled: false` | configuration, deliberate: enable only for the attended test |
| GPS accuracy | h_acc 4.5–5.1 m, Pi booted 8 min earlier. The navigation limit is 5.0 m, and useful headings need ≲ 2.5 m | physical: open sky, warm receiver |
| front obstacle / `front_valid` | firmware `forward_blocked` and `front_valid` are DRIVE gates (`ControllerTelemetry.safety_faults`). The right-front sensor has dropped echoes before | physical: clear area; verify the sensor map |
| rear sensor fault | not a forward-motion gate on the Pi. It is reported in `faults[]` | physical, before any reverse capability |

None of these are relaxed by code. No gate was changed to make the rover move.

## 6. The first physical GPS test — minimum ordered prerequisites

Every step needs explicit approval. Stop at the first failure.

1. **Motor power cutoff.** Fit and verify an independent motor-power cutoff
   within the operator's reach. Today one battery feeds the Pi, the ESP32 and
   the motors, and there is no physical e-stop. Verify it cuts the motors with
   the wheels off the ground.
2. **ESP32 stability.** No reboot and no TELEMETRY gap for ≥ 30 min powered
   (`scripts/esp32_reboot_watch.py`). Clear the latch only after that
   (`/safety/clear`, operator token).
3. **Sensor map.** Front left/right and rear ultrasonics each verified by
   hand (an object in front of each sensor in turn moves the right field).
   Front valid ≥ 95% over 60 s in the clear.
4. **Wheels-off-ground bench pulse** (`docs/operations/BENCH_MOTION_TEST.md`).
   ACK ACCEPTED, `applied` 64/64, every wheel turning *forward*, then STOP
   confirmed by TELEMETRY 0/0. This is the first physical evidence of layer 2
   and of drive direction.
5. **Side mapping.** Confirm that the ESP32's left channel drives the rover's
   left wheels. The bench pulse is symmetric, and no Pi tool sends an
   asymmetric DRIVE on blocks. Check the wiring against the firmware motor map
   by hand, with the power cut. The firmware's `motor_map_verified` is a
   build-time label, not a measurement. Step 9 then checks it in motion. A
   swapped side makes every turn go the wrong way, and no simulation can
   detect that.
6. **Site.** Open sky, flat, clear, bounded area of at least 40 × 20 m, with no
   people inside it. h_acc ≤ 2.5 m and steady for 5 min (`/state`
   `position.h_acc_m`).
7. **Route.** A test-injected mission (`TEST_MISSION_INJECTION.md`) with a
   pickup path of ~30 m and waypoints spaced ≥ 15 m. The arrival radius is
   8 m, and closer waypoints are consumed on sight. Place the rover **facing
   the first waypoint within ±45°**. Keep `ROBOTX_MOTION_CRUISE_SPEED` and
   `_SLOW_SPEED` at their defaults or lower.
8. **STOP first.** With motion enabled and the rover on the ground, start the
   mission and then issue `/mission/stop` within 3 s. Confirm DRIVE → STOP in
   `controller.last_motion_sent` and wheels stopped. Repeat with
   `/safety/estop`. Only then let a run continue.
9. **Turn direction and turn-rate calibration.** The first bounded turn is
   logged as `decision.changed … following route (error=Xdeg)` with its
   left/right. It lasts until the `no heading: creeping …` line that follows
   the `turning toward route …` one. A positive X must slow the right side and turn
   the rover right. If it turns the other way, stop. Measure the angle the
   rover actually turned (tape on the ground, or a photo from above) and set
   `ROBOTX_NAV_TURN_RATE_DPS` = current rate × actual / X. Too high is safe
   (slow). More than 2× too low oscillates and ends in the 60 s pause.
10. **Short autonomous run.** Repeat step 7 with the calibrated rate. Record
    the GPS track, the intents, the ACKs and the applied outputs. Success
    means arrival within 8 m, every turn on the commanded side, and STOP on
    arrival. Only after that, longer routes.

Until step 10 has been recorded, the rover's physical autonomy is unproven.
