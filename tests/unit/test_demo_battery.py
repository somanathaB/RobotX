"""DEMO battery simulation: backend-simulator drain, demo recharge, persistence, wire.

The drain expectations are the backend simulator's own numbers
(RobotX `Backend/src/simulation/energyPolicy.js` and `constants.js`), restated
here independently of the module's constants so a changed constant fails a test.
"""

import asyncio
import json
import os
import shutil
import tempfile
import time
import unittest

from robotx.application.agent import RobotAgent
from robotx.communication.backend_link import BackendConfig, BackendLink
from robotx.communication.protocol import build_telemetry_payload, demo_battery_report
from robotx.config.settings import Settings
from robotx.hardware.demo_battery import (
    BATTERY_MIN_PERCENT,
    MAX_STEP_S,
    BatteryPhase,
    DemoBatteryConfig,
    DemoBatterySimulator,
    DrainModel,
    drain_percent,
    haversine_m,
)
from robotx.state.robot_state import OperatingMode, PowerState, RobotState
from robotx.state.telemetry import build_telemetry
from tests.unit.test_backend_link import FakeAgent, FakeSio, MemoryTokenStore, state_with_fix
from tests.unit.test_protocol import make_snapshot


def make_sim(*, clock_start=0.0, **cfg):
    cfg.setdefault("state_path", None)
    cfg.setdefault("confirm_reports", False)
    return DemoBatterySimulator(DemoBatteryConfig(**cfg), clock=lambda: clock_start)


def run(sim, *, start, seconds, step=1.0, working=False, idle=True, fixes=None):
    """Advance `sim` from `start` for `seconds` in `step`s; returns the end time.

    `fixes(t)` optionally gives the (lat, lon, timestamp) measured at time t.
    """

    t = start
    sim.update(working=working, idle=idle, fix=fixes(t) if fixes else None, now=t)
    n = int(round(seconds / step))
    for _ in range(n):
        t += step
        sim.update(working=working, idle=idle, fix=fixes(t) if fixes else None, now=t)
    return t


def until(sim, phase, *, start=0.0, limit=100_000, **flags):
    """Step 1 s at a time until `sim` enters `phase`; returns that instant."""

    flags.setdefault("idle", True)
    flags.setdefault("working", False)
    t = start
    sim.update(now=t, **flags)
    for _ in range(limit):
        if sim.phase is phase:
            return t
        t += 1.0
        sim.update(now=t, **flags)
    raise AssertionError(f"never reached {phase}")


def northward(speed_mps, origin=(12.9, 77.5)):
    """Measured fixes for a rover driving due north at `speed_mps`."""

    metres_per_degree = haversine_m((0.0, 0.0), (1.0, 0.0))
    return lambda t: (origin[0] + speed_mps * t / metres_per_degree, origin[1], 1_000_000.0 + t)


def simulated_power(percent, *, charging=False, phase="DRAINING"):
    return PowerState(status="SIMULATED", percent=percent, source="demo", charging=charging, phase=phase)


# --- 1, 2: initial state ------------------------------------------------------


class TestInitialState(unittest.TestCase):
    def test_a_fresh_simulation_starts_at_100_percent(self):
        sim = make_sim()
        self.assertEqual(sim.percent, 100.0)
        self.assertEqual(sim.status_dict()["percent"], 100.0)
        self.assertEqual(sim.status_dict()["status"], "SIMULATED")

    def test_updates_never_reset_the_battery_to_100(self):
        sim = make_sim(drain_model=DrainModel.LEGACY_PERCENTAGE)
        t = run(sim, start=0.0, seconds=600)
        after_ten_minutes = sim.percent
        self.assertLess(after_ten_minutes, 100.0)
        for _ in range(50):
            t += 0.1
            sim.update(working=False, idle=True, now=t)
            self.assertLessEqual(sim.percent, after_ten_minutes)

    def test_the_first_update_drains_nothing(self):
        sim = make_sim()
        sim.update(working=False, idle=True, now=12_345.0)
        self.assertEqual(sim.percent, 100.0)

    def test_the_simulated_value_is_never_a_measurement(self):
        power = PowerState.from_status_dict(make_sim().status_dict())
        self.assertTrue(power.is_simulated)
        self.assertFalse(power.is_measured)


# --- 3, 4: the backend simulator's drain -------------------------------------


class TestDrainMatchesBackendSimulator(unittest.TestCase):
    def test_power_model_idle_is_standby_9_5_W_on_a_1000_Wh_pack(self):
        # energyPolicy.js: beta_aux = 9.5 W / 3600 -> 9.5 Wh in an hour = 0.95 % of 1000 Wh.
        sim = make_sim(drain_model=DrainModel.POWER)
        run(sim, start=0.0, seconds=3600, idle=True)
        self.assertAlmostEqual(100.0 - sim.percent, 0.95, places=9)

    def test_power_model_travel_is_30_W_at_2_m_per_s(self):
        # energyPolicy.js: P(2 m/s) = 30 W -> 30 Wh in an hour = 3 % of 1000 Wh.
        sim = make_sim(drain_model=DrainModel.POWER)
        run(sim, start=0.0, seconds=3600, working=True, idle=False, fixes=northward(2.0))
        self.assertAlmostEqual(100.0 - sim.percent, 3.0, places=6)

    def test_power_model_formula_per_interval(self):
        # Wh = 9.5/3600 * s + (30 - 9.5) / (3600 * 2) * m ; percent = 100 * Wh / 1000
        expected = (9.5 / 3600 * 10 + 20.5 / 7200 * 37) / 1000 * 100
        got = drain_percent(DrainModel.POWER, seconds=10, working=True, distance_m=37)
        self.assertAlmostEqual(got, expected, places=12)

    def test_power_model_distance_is_not_counted_when_not_on_a_mission(self):
        # Stationary GPS jitter, or a fix stream while idle, is not driving.
        sim = make_sim(drain_model=DrainModel.POWER)
        run(sim, start=0.0, seconds=3600, working=False, idle=True, fixes=northward(2.0))
        self.assertAlmostEqual(100.0 - sim.percent, 0.95, places=9)

    def test_a_gps_jump_over_500_m_is_not_distance(self):
        sim = make_sim(drain_model=DrainModel.POWER)
        sim.update(working=True, idle=False, fix=(12.9, 77.5, 1.0), now=0.0)
        sim.update(working=True, idle=False, fix=(13.0, 77.5, 2.0), now=1.0)  # ~11 km
        self.assertAlmostEqual(100.0 - sim.percent, 9.5 / 3600 / 1000 * 100, places=12)

    def test_legacy_travel_rate_is_0_0444_percent_per_2_s(self):
        # constants.js BATTERY_DRAIN_ACTIVE: "100 % -> 20 % in 1 hour" (1800 ticks).
        sim = make_sim(drain_model=DrainModel.LEGACY_PERCENTAGE)
        run(sim, start=0.0, seconds=3600, working=True, idle=False)
        self.assertAlmostEqual(100.0 - sim.percent, 0.0444 * 1800, places=6)

    def test_legacy_idle_rate_is_0_0089_percent_per_2_s(self):
        # constants.js BATTERY_DRAIN_IDLE: "100 % -> 20 % in 5 hours" (9000 ticks).
        sim = make_sim(drain_model=DrainModel.LEGACY_PERCENTAGE, recharge_threshold_percent=10)
        run(sim, start=0.0, seconds=3600, idle=True)
        self.assertAlmostEqual(100.0 - sim.percent, 0.0089 * 1800, places=6)

    def test_paused_drains_at_the_idle_rate(self):
        # VirtualRobot._drainsAsWorking: PAUSED is not working.
        sim = make_sim(drain_model=DrainModel.LEGACY_PERCENTAGE)
        run(sim, start=0.0, seconds=100, working=False, idle=False)
        self.assertAlmostEqual(100.0 - sim.percent, 0.0089 * 50, places=9)


# --- 5, 6: elapsed time, bounds, clock irregularities -------------------------


class TestElapsedTimeAndBounds(unittest.TestCase):
    def test_drain_depends_on_elapsed_time_not_loop_frequency(self):
        results = []
        for step in (0.05, 0.5, 1.0, 5.0):
            for model in DrainModel:
                sim = make_sim(drain_model=model)
                run(sim, start=0.0, seconds=600, step=step, working=True, idle=False, fixes=northward(1.0))
                results.append((model, sim.percent))
        for model in DrainModel:
            values = [p for m, p in results if m is model]
            for value in values:
                self.assertAlmostEqual(value, values[0], places=6, msg=model)

    def test_battery_never_goes_below_the_simulator_floor_or_above_100(self):
        sim = make_sim(drain_model=DrainModel.LEGACY_PERCENTAGE)
        t = 0.0
        sim.update(working=True, idle=False, now=t)
        for _ in range(20_000):
            t += 5.0
            sim.update(working=True, idle=False, now=t)
            self.assertGreaterEqual(sim.percent, BATTERY_MIN_PERCENT)
            self.assertLessEqual(sim.percent, 100.0)
        self.assertEqual(sim.percent, BATTERY_MIN_PERCENT)
        self.assertGreaterEqual(BATTERY_MIN_PERCENT, 0.0)

    def test_a_long_pause_is_capped_not_a_jump(self):
        sim = make_sim(drain_model=DrainModel.LEGACY_PERCENTAGE)
        sim.update(working=True, idle=False, now=0.0)
        sim.update(working=True, idle=False, now=10 * 3600.0)  # the loop stalled for 10 h
        self.assertAlmostEqual(100.0 - sim.percent, 0.0444 * MAX_STEP_S / 2, places=9)

    def test_a_clock_going_backwards_drains_nothing_and_raises_nothing(self):
        sim = make_sim(drain_model=DrainModel.LEGACY_PERCENTAGE)
        run(sim, start=1000.0, seconds=10)
        before = sim.percent
        sim.update(working=True, idle=False, now=500.0)
        self.assertEqual(sim.percent, before)
        sim.update(working=True, idle=False, now=501.0)
        self.assertLess(sim.percent, before)

    def test_a_non_finite_clock_reading_is_ignored(self):
        sim = make_sim()
        sim.update(working=False, idle=True, now=0.0)
        sim.update(working=False, idle=True, now=float("nan"))
        self.assertEqual(sim.percent, 100.0)


# --- 7-10: the demo recharge cycle -------------------------------------------


class TestDemoRecharge(unittest.TestCase):
    def low(self, **cfg):
        cfg.setdefault("drain_model", DrainModel.LEGACY_PERCENTAGE)
        cfg.setdefault("initial_percent", 30.02)
        return make_sim(**cfg)

    def test_at_or_below_30_percent_the_recharge_starts(self):
        sim = self.low()
        t = run(sim, start=0.0, seconds=2)
        self.assertIs(sim.phase, BatteryPhase.DRAINING)
        self.assertGreater(sim.percent, 30.0)
        t = run(sim, start=t, seconds=4)
        self.assertIs(sim.phase, BatteryPhase.RECHARGING)
        self.assertLessEqual(sim.percent, 30.0)
        self.assertTrue(sim.status_dict()["charging"])

    def test_recharge_completes_after_60_s_at_100_percent_and_holds_until_then(self):
        sim = self.low()
        t = until(sim, BatteryPhase.RECHARGING)
        held = sim.percent
        t = run(sim, start=t, seconds=59)
        self.assertIs(sim.phase, BatteryPhase.RECHARGING)
        self.assertEqual(sim.percent, held)  # no drain while recharging
        run(sim, start=t, seconds=1)
        self.assertEqual(sim.percent, 100.0)

    def test_drain_resumes_after_recharge(self):
        sim = self.low()
        t = until(sim, BatteryPhase.RECHARGING)
        t = run(sim, start=t, seconds=60)
        self.assertIs(sim.phase, BatteryPhase.DRAINING)
        self.assertEqual(sim.percent, 100.0)
        run(sim, start=t, seconds=100)
        self.assertAlmostEqual(sim.percent, 100.0 - 0.0089 * 50, places=9)

    def test_repeated_cycles(self):
        sim = make_sim(drain_model=DrainModel.LEGACY_PERCENTAGE, recharge_threshold_percent=99.5)
        t, cycles, last = 0.0, 0, sim.phase
        sim.update(working=False, idle=True, now=t)
        for _ in range(2000):
            t += 1.0
            sim.update(working=False, idle=True, now=t)
            if last is BatteryPhase.RECHARGING and sim.phase is not BatteryPhase.RECHARGING:
                cycles += 1
                self.assertEqual(sim.percent, 100.0)
            last = sim.phase
        # 0.5 % at 0.00445 %/s ~ 113 s, + 60 s recharge: ~11 cycles in 2000 s.
        self.assertGreaterEqual(cycles, 10)

    def test_threshold_and_duration_are_configurable(self):
        sim = self.low(initial_percent=50.01, recharge_threshold_percent=50, recharge_duration_s=5)
        t = until(sim, BatteryPhase.RECHARGING)
        self.assertLessEqual(sim.percent, 50.0)
        self.assertGreater(sim.percent, 49.9)
        t = run(sim, start=t, seconds=4)
        self.assertIs(sim.phase, BatteryPhase.RECHARGING)
        run(sim, start=t, seconds=1)
        self.assertEqual(sim.percent, 100.0)

    def test_no_recharge_starts_while_a_mission_is_held(self):
        sim = self.low()
        t = run(sim, start=0.0, seconds=60, working=True, idle=False)
        self.assertIs(sim.phase, BatteryPhase.DRAINING)
        self.assertLess(sim.percent, 30.0)
        run(sim, start=t, seconds=1, idle=True)
        self.assertIs(sim.phase, BatteryPhase.RECHARGING)

    def test_a_mission_cancels_a_running_recharge_without_raising_the_battery(self):
        sim = self.low()
        t = until(sim, BatteryPhase.RECHARGING)
        t = run(sim, start=t, seconds=30)
        held = sim.percent
        t = run(sim, start=t, seconds=1, working=True, idle=False)
        self.assertIs(sim.phase, BatteryPhase.DRAINING)
        self.assertLessEqual(sim.percent, held)
        # Idle again: a full minute is needed, not the remaining half.
        t = until(sim, BatteryPhase.RECHARGING, start=t)
        self.assertEqual(sim.recharge_elapsed_s, 0.0)
        run(sim, start=t, seconds=59, idle=True)
        self.assertIs(sim.phase, BatteryPhase.RECHARGING)

    def test_with_a_backend_the_recharge_waits_for_a_charging_frame(self):
        sim = self.low(confirm_reports=True)
        sim.note_reported({"status": "CHARGING", "battery": 100.0})  # fresh pack announced
        t = until(sim, BatteryPhase.RECHARGING)
        t = run(sim, start=t, seconds=60)
        self.assertIs(sim.phase, BatteryPhase.CHARGED_UNREPORTED)
        t = run(sim, start=t, seconds=600)
        self.assertEqual(sim.percent, 100.0)  # no drain until it is reported
        # A frame built before the jump does not count, nor a non-CHARGING one.
        self.assertFalse(sim.note_reported({"status": "CHARGING", "battery": 29.9}))
        self.assertFalse(sim.note_reported({"status": "IDLE", "battery": 100.0}))
        self.assertTrue(sim.note_reported({"status": "CHARGING", "battery": 100.0}))
        self.assertIs(sim.phase, BatteryPhase.DRAINING)
        run(sim, start=t, seconds=10)
        self.assertLess(sim.percent, 100.0)


# --- 11, 16: persistence, restart, invalid state ------------------------------


class TestPersistence(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="demo-battery-")
        self.path = os.path.join(self.dir, "nested", "demo_battery.json")

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def sim(self, **cfg):
        cfg.setdefault("drain_model", DrainModel.LEGACY_PERCENTAGE)
        return make_sim(state_path=self.path, **cfg)

    def test_a_restart_resumes_where_it_was_and_never_above_the_last_reported_value(self):
        sim = self.sim()
        run(sim, start=0.0, seconds=777, step=0.3)
        reported = sim.status_dict()["percent"]
        restarted = self.sim()
        self.assertEqual(restarted.status_dict()["percent"], reported)
        self.assertLessEqual(restarted.percent, sim.percent + 0.05)
        self.assertIs(restarted.phase, BatteryPhase.DRAINING)

    def test_downtime_between_runs_is_not_drained(self):
        sim = self.sim()
        run(sim, start=0.0, seconds=60)
        before = sim.status_dict()["percent"]
        restarted = self.sim()
        restarted.update(working=False, idle=True, now=9e9)  # any monotonic origin
        self.assertEqual(restarted.status_dict()["percent"], before)

    def test_a_restart_mid_recharge_keeps_its_timer(self):
        sim = self.sim(initial_percent=30.01)
        t = until(sim, BatteryPhase.RECHARGING)
        run(sim, start=t, seconds=40)
        restarted = self.sim(initial_percent=30.01)
        self.assertIs(restarted.phase, BatteryPhase.RECHARGING)
        self.assertEqual(restarted.recharge_elapsed_s, 40.0)
        t = run(restarted, start=0.0, seconds=19)
        self.assertIs(restarted.phase, BatteryPhase.RECHARGING)
        run(restarted, start=t, seconds=1)
        self.assertEqual(restarted.percent, 100.0)

    def test_a_missing_file_is_a_fresh_session_announced_as_charged(self):
        sim = self.sim(confirm_reports=True)
        self.assertEqual(sim.percent, 100.0)
        self.assertIs(sim.phase, BatteryPhase.CHARGED_UNREPORTED)
        self.assertTrue(os.path.exists(self.path))

    def test_invalid_state_files_are_replaced_by_a_fresh_session(self):
        cases = [
            "not json at all",
            json.dumps([1, 2, 3]),
            json.dumps({"version": 99, "percent": 50, "phase": "DRAINING"}),
            json.dumps({"version": 1, "percent": 150, "phase": "DRAINING"}),
            json.dumps({"version": 1, "percent": -1, "phase": "DRAINING"}),
            json.dumps({"version": 1, "percent": "50", "phase": "DRAINING"}),
            json.dumps({"version": 1, "percent": True, "phase": "DRAINING"}),
            json.dumps({"version": 1, "percent": 50, "phase": "EXPLODED"}),
            json.dumps({"version": 1, "percent": 50, "phase": "RECHARGING", "recharge_elapsed_s": -3}),
            '{"version": 1, "percent": NaN, "phase": "DRAINING"}',
            "",
        ]
        for content in cases:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with open(self.path, "w", encoding="utf-8") as fh:
                fh.write(content)
            sim = self.sim(confirm_reports=True)
            self.assertEqual(sim.percent, 100.0, content)
            # The rise to 100 is reported as a recharge, never as a plain frame.
            self.assertIs(sim.phase, BatteryPhase.CHARGED_UNREPORTED, content)
            with open(self.path, encoding="utf-8") as fh:
                self.assertEqual(json.load(fh)["percent"], 100.0, content)

    def test_a_stored_value_below_the_floor_is_raised_to_it(self):
        os.makedirs(os.path.dirname(self.path))
        with open(self.path, "w", encoding="utf-8") as fh:
            json.dump({"version": 1, "percent": 1.0, "phase": "DRAINING"}, fh)
        self.assertEqual(self.sim().percent, BATTERY_MIN_PERCENT)

    def test_an_unwritable_location_is_not_fatal(self):
        blocker = os.path.join(self.dir, "file")
        with open(blocker, "w") as fh:
            fh.write("x")
        sim = make_sim(state_path=os.path.join(blocker, "demo_battery.json"))
        sim.update(working=False, idle=True, now=0.0)
        self.assertEqual(sim.percent, 100.0)


# --- configuration ------------------------------------------------------------


class TestConfiguration(unittest.TestCase):
    def test_defaults(self):
        s = Settings.from_env({})
        self.assertFalse(s.demo_battery_enabled)
        self.assertEqual(s.demo_battery_drain_model, "power")
        self.assertEqual(s.demo_battery_initial_percent, 100.0)
        self.assertEqual(s.demo_battery_recharge_threshold_percent, 30.0)
        self.assertEqual(s.demo_battery_recharge_s, 60.0)
        self.assertEqual(s.demo_battery_state_path, "~/.robotx/demo_battery.json")

    def test_environment_overrides(self):
        s = Settings.from_env({
            "ROBOTX_DEMO_BATTERY_ENABLED": "1",
            "ROBOTX_DEMO_BATTERY_DRAIN_MODEL": "legacy_percentage",
            "ROBOTX_DEMO_BATTERY_INITIAL_PERCENT": "90",
            "ROBOTX_DEMO_BATTERY_RECHARGE_THRESHOLD_PERCENT": "40",
            "ROBOTX_DEMO_BATTERY_RECHARGE_S": "15",
            "ROBOTX_DEMO_BATTERY_STATE_PATH": "/tmp/x.json",
        })
        cfg = DemoBatteryConfig.from_settings(s, confirm_reports=True)
        self.assertEqual(
            (cfg.drain_model, cfg.initial_percent, cfg.recharge_threshold_percent, cfg.recharge_duration_s,
             cfg.state_path),
            (DrainModel.LEGACY_PERCENTAGE, 90.0, 40.0, 15.0, "/tmp/x.json"),
        )

    def test_invalid_values_are_refused(self):
        for kwargs in (
            {"recharge_threshold_percent": 5.0},
            {"recharge_threshold_percent": 100.0},
            {"initial_percent": 101.0},
            {"initial_percent": float("nan")},
            {"recharge_duration_s": 0.0},
        ):
            with self.assertRaises(ValueError, msg=kwargs):
                DemoBatteryConfig(**kwargs)
        bad = Settings.from_env({"ROBOTX_DEMO_BATTERY_DRAIN_MODEL": "turbo"})
        with self.assertRaises(ValueError):
            DemoBatteryConfig.from_settings(bad, confirm_reports=False)


# --- 12, 13: the agent ---------------------------------------------------------


def agent_settings(**extra):
    env = {
        "ROBOTX_ROBOT_ID": "test-rover",
        "ROBOTX_CAMERA_ENABLED": "0",
        "ROBOTX_PERCEPTION_ENABLED": "0",
        "ROBOTX_GPS_ENABLED": "0",
        "ROBOTX_LOG_LEVEL": "CRITICAL",
        "ROBOTX_DEMO_BATTERY_STATE_PATH": "",
    }
    env.update(extra)
    return Settings.from_env(env)


class TestAgent(unittest.TestCase):
    def test_disabled_keeps_the_hardware_path(self):
        agent = RobotAgent(agent_settings())
        self.assertIsNone(agent.demo_battery)
        snapshot = agent.tick()
        self.assertEqual(snapshot.power.status, "UNAVAILABLE")
        self.assertIsNone(snapshot.power.percent)
        self.assertNotIn("battery", build_telemetry_payload(snapshot, sequence=1, max_position_age_s=5).payload)

    def test_enabled_puts_the_simulated_value_in_state_and_local_telemetry(self):
        agent = RobotAgent(agent_settings(ROBOTX_DEMO_BATTERY_ENABLED="1"))
        self.assertIsNotNone(agent.demo_battery)
        snapshot = agent.tick()
        self.assertEqual(snapshot.power.status, "SIMULATED")
        self.assertEqual(snapshot.power.percent, 100.0)
        local = build_telemetry(snapshot)["battery"]
        wire = build_telemetry_payload(snapshot, sequence=1, max_position_age_s=5).payload
        self.assertEqual(local["percent"], wire["battery"])
        # No backend link configured: drain is not held for a report.
        self.assertIs(agent.demo_battery.phase, BatteryPhase.DRAINING)

    def test_a_bad_setting_disables_the_simulation_rather_than_the_robot(self):
        agent = RobotAgent(agent_settings(
            ROBOTX_DEMO_BATTERY_ENABLED="1", ROBOTX_DEMO_BATTERY_RECHARGE_THRESHOLD_PERCENT="2",
        ))
        self.assertIsNone(agent.demo_battery)
        self.assertEqual(agent.tick().power.status, "UNAVAILABLE")

    def test_a_mission_counts_as_working_and_not_idle(self):
        agent = RobotAgent(agent_settings(ROBOTX_DEMO_BATTERY_ENABLED="1"))
        seen = []
        real = agent.demo_battery.update
        agent.demo_battery.update = lambda **kw: seen.append(kw) or real(**kw)
        agent.tick()
        agent.state.set_mode(OperatingMode.AUTO)
        agent.tick()
        self.assertEqual((seen[0]["working"], seen[0]["idle"]), (False, True))
        self.assertEqual((seen[1]["working"], seen[1]["idle"]), (True, False))


# --- 13, 14: the TELEMETRY frame ------------------------------------------------


class TestTelemetryFrame(unittest.TestCase):
    BASE_KEYS = {"timestamp", "sequence", "status", "lat", "lon", "speed"}

    def frame(self, snapshot):
        return build_telemetry_payload(snapshot, sequence=7, max_position_age_s=5.0).payload

    def snap(self, power, mode=OperatingMode.IDLE):
        from dataclasses import replace
        return replace(make_snapshot(mode=mode), power=power)

    def test_disabled_frame_is_unchanged(self):
        payload = self.frame(self.snap(PowerState()))
        self.assertEqual(set(payload), self.BASE_KEYS)
        self.assertEqual(payload["status"], "IDLE")

    def test_simulated_value_rides_the_existing_battery_field(self):
        payload = self.frame(self.snap(simulated_power(87.26), mode=OperatingMode.AUTO))
        self.assertEqual(set(payload), self.BASE_KEYS | {"battery", "energy"})
        self.assertEqual(payload["battery"], 87.3)
        self.assertEqual(payload["status"], "ACTIVE")
        # Labelled as simulated (handoff P-2), and never a measurement basis:
        # no socMethod, so the backend must not plan with it.
        self.assertEqual(payload["energy"], {"batterySource": "DEMO_SIMULATION"})
        json.dumps(payload)

    def test_a_recharge_frame_from_an_idle_rover_says_charging(self):
        payload = self.frame(self.snap(simulated_power(100.0, charging=True, phase="CHARGED_UNREPORTED")))
        self.assertEqual(payload["status"], "CHARGING")
        self.assertEqual(payload["battery"], 100.0)
        self.assertEqual(payload["energy"], {"batterySource": "DEMO_SIMULATION"})

    def test_every_demo_frame_is_labelled_and_never_carries_a_soc_basis(self):
        """Acceptance (2): every frame with a demo `battery` says DEMO_SIMULATION, none says socMethod."""

        cases = [
            (simulated_power(87.26), OperatingMode.AUTO),
            (simulated_power(42.0), OperatingMode.IDLE),
            (simulated_power(31.0), OperatingMode.PAUSED),
            (simulated_power(100.0, charging=True, phase="CHARGED_UNREPORTED"), OperatingMode.IDLE),
        ]
        for power, mode in cases:
            payload = self.frame(self.snap(power, mode=mode))
            self.assertIn("battery", payload, mode)
            self.assertEqual(payload["energy"], {"batterySource": "DEMO_SIMULATION"}, mode)
            self.assertNotIn("socMethod", json.dumps(payload), mode)

    def test_no_label_without_a_demo_battery(self):
        """`energy` exists only to qualify a demo `battery`; with none sent, it is absent too."""

        self.assertNotIn("energy", self.frame(self.snap(PowerState())))
        for mode in (OperatingMode.AUTO, OperatingMode.PAUSED):
            payload = self.frame(self.snap(simulated_power(100.0, charging=True), mode=mode))
            self.assertNotIn("battery", payload, mode)
            self.assertNotIn("energy", payload, mode)

    def test_a_recharge_that_cannot_say_charging_omits_battery(self):
        for mode in (OperatingMode.AUTO, OperatingMode.PAUSED, OperatingMode.STOPPED, OperatingMode.ERROR):
            payload = self.frame(self.snap(simulated_power(100.0, charging=True), mode=mode))
            self.assertNotIn("battery", payload, mode)
            self.assertNotEqual(payload["status"], "CHARGING", mode)

    def test_hardware_unavailable_is_never_reported(self):
        self.assertEqual(demo_battery_report(self.snap(PowerState())), (None, False))


# --- 14, 15: the backend link ---------------------------------------------------


class TestBackendLink(unittest.TestCase):
    def link(self, state, sent):
        sio = FakeSio()
        link = BackendLink(
            BackendConfig(enabled=True, robot_id="robotx-pi", pairing_code="123456"),
            state,
            FakeAgent(),
            client_factory=lambda _cfg: sio,
            token_store=MemoryTokenStore(),
            telemetry_sent=sent.append,
        )
        return link, sio

    def test_the_battery_goes_out_on_telemetry_and_no_other_event(self):
        state = state_with_fix()
        state.update_power(simulated_power(64.0))
        sent = []
        link, sio = self.link(state, sent)

        async def scenario():
            await link._connect_once()
            await link._await_authentication()
            await link._publish_telemetry()

        asyncio.run(scenario())
        events = [event for event, _ in sio.emitted]
        self.assertEqual(events, ["AUTH", "TELEMETRY"])
        self.assertEqual(sio.events_named("TELEMETRY")[0]["battery"], 64.0)
        self.assertEqual(sent, sio.events_named("TELEMETRY"))
        for event, payload in sio.emitted:
            if event != "TELEMETRY":
                self.assertNotIn("battery", payload)

    def test_a_reported_recharge_resumes_drain(self):
        sim = make_sim(confirm_reports=True)
        state = RobotState("robotx-pi")
        state.update_power(PowerState.from_status_dict(sim.update(working=False, idle=True, now=0.0)))
        link, sio = self.link(state, [])
        link._telemetry_sent = sim.note_reported

        async def scenario():
            await link._connect_once()
            await link._await_authentication()
            await link._publish_telemetry()

        asyncio.run(scenario())
        frame = sio.events_named("TELEMETRY")[0]
        self.assertEqual((frame["status"], frame["battery"]), ("CHARGING", 100.0))
        self.assertIs(sim.phase, BatteryPhase.DRAINING)

    def test_a_frame_that_never_left_does_not_resume_drain(self):
        sim = make_sim(confirm_reports=True)
        state = RobotState("robotx-pi")
        state.update_power(PowerState.from_status_dict(sim.update(working=False, idle=True, now=0.0)))
        link, _ = self.link(state, [])
        link._telemetry_sent = sim.note_reported
        asyncio.run(link._publish_telemetry())  # not connected: nothing emitted
        self.assertIs(sim.phase, BatteryPhase.CHARGED_UNREPORTED)


if __name__ == "__main__":
    unittest.main()
