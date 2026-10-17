"""DEMO battery simulation -- a stand-in for battery sensing this robot does not have.

What this is
------------
`robotx.hardware.battery` is the honest answer to "what is the battery doing":
UNAVAILABLE, because there is no fuel gauge, ADC or divider on this robot. For
a demonstration the operator may switch on this module instead
(`ROBOTX_DEMO_BATTERY_ENABLED=1`). It produces a **simulated** percentage in
the same `{status, percent, voltage_v, source}` shape, with
`status = "SIMULATED"`, so every consumer can still tell it from a measurement.
It never reads, and never controls, any battery or charging hardware.

Off by default. With it off, nothing in this file runs and the agent keeps the
UNAVAILABLE path exactly as before.

Copied from the backend simulator (RobotX repo, `Backend/src/simulation/`)
-------------------------------------------------------------------------
Two drain models, both the backend's own arithmetic, selected by
`ROBOTX_DEMO_BATTERY_DRAIN_MODEL`:

``power`` (default) -- `energyPolicy.js` + `VirtualRobot._applyBattery`, the
path every simulated robot commissioned under the power policy takes
(`_energyBasis = "SIMULATED_POWER_WH"`)::

    beta_aux  = P_standby / 3600                       Wh per second, always
    beta_dist = (P_moving - P_standby) / (3600 * v_ref) Wh per metre driven
    Wh        = beta_aux * seconds + beta_dist * metres
    percent  -= 100 * Wh / packNominalWh,   floored at BATTERY_MIN

with P_standby = 9.5 W, P_moving = 30 W at v_ref = 2 m/s, packNominalWh =
1000 Wh, BATTERY_MIN = 5 %. (`beta_move_time` and `beta_dwell` are 0 under that
policy, so "travelling" differs from "idle" only by the distance term.)

``legacy_percentage`` -- `constants.js` + the `LEGACY_PERCENTAGE` fallback of
`_applyBattery`, for units whose energy row is incomplete::

    percent -= rate * seconds / 2 s,   floored at BATTERY_MIN
    rate = 0.0444 %/tick while working (ACTIVE / RETURNING / ISSUES on a task)
    rate = 0.0089 %/tick otherwise     (IDLE / PAUSED / ISSUES)

Both are scaled by the real elapsed time, exactly as the backend scales its
constants by the tick period: the loop frequency does not change the drain.

Where the Pi differs from the simulator, and why
------------------------------------------------
- Distance. The simulator knows the metres it stepped along its path. The Pi
  only has measured GPS fixes, so it sums the great-circle distance between
  consecutive fixes **while a mission is running** (stationary GPS jitter is
  not driving) and ignores any single jump over 500 m -- the same sanity bound
  the backend applies when it accumulates a real robot's distance
  (`telemetry.handler.js`).
- "Working" is `OperatingMode.AUTO` (reported to the backend as `ACTIVE`).
- A step longer than `MAX_STEP_S` (a stalled loop, a suspended process, a clock
  irregularity) is counted as `MAX_STEP_S`, and a negative step as zero, so a
  pause can never produce a jump. Downtime between runs is not drained at all:
  a restart resumes from the persisted value.

DEMO-SPECIFIC recharge cycle -- NOT the backend simulator's charging logic
--------------------------------------------------------------------------
The backend simulator charges by docking at a charger and integrating a
CC/CV power curve (`chargeModel.js`). None of that is used here. Instead, for
the demonstration only:

1. at or below `recharge_threshold_percent` (30 %), and only while the robot is
   idle -- no mission, mode IDLE, the backend simulator's own condition for
   starting a charge (`VirtualRobot._tick` step 4) -- a simulated recharge
   starts. No drain is applied while it runs (as the simulator applies none
   while CHARGING);
2. after `recharge_duration_s` (60 s) the percentage is set to 100 %;
3. normal drain resumes, and the cycle repeats.

A mission arriving during the minute cancels the recharge; it restarts the
next time the robot is idle.

Why a recharge has to be *reported* before drain resumes
--------------------------------------------------------
The backend's §23.5 trust boundary refuses any TELEMETRY frame whose
state of charge rose while the frame did not say `status: "CHARGING"`. Where it
is enforced it drops the whole frame, GPS included, and every later frame too,
because the reference it compares against never advances. So a rise must
reach the backend in a CHARGING frame. After the jump to 100 % the simulator
stays in `CHARGED_UNREPORTED` (no drain) until the backend link confirms that a
frame carrying the new value with `status: "CHARGING"` was sent
(`note_reported`). With no backend link (`confirm_reports=False`) there is
nobody to tell, and drain resumes at once. A fresh or unreadable state file
starts the same way, so the first report of 100 % is never a refused rise.

Persistence
-----------
One JSON file (default `~/.robotx/demo_battery.json`), written to a temp file
and renamed, like the session token and the commitment marks. It is rewritten
on every phase change, whenever the reported (0.1 %) value changes, and once a
second during a recharge. So a restart never resumes above the last value the
backend saw (which it would refuse as a rise), and a recharge resumes its
timer. Delete the file to start a fresh demo session at 100 %.
"""

from __future__ import annotations

import json
import logging
import math
import os
import tempfile
import threading
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Optional, Tuple

from robotx.config.logging_setup import log_event


logger = logging.getLogger(__name__)


# --- copied from the backend simulator ----------------------------------------
# Each value is the backend default it names; see the module docstring.

STANDBY_POWER_W = 9.5              # energyPolicy.js FIELDS.standbyPowerW.default
MOVING_POWER_W = 30.0              # energyPolicy.js FIELDS.movingPowerW.default
MOVING_POWER_REF_SPEED_MPS = 2.0   # energyPolicy.js FIELDS.movingPowerReferenceSpeedMps.default
PACK_NOMINAL_WH = 1000.0           # constants.js PACK_NOMINAL_WH
BATTERY_MIN_PERCENT = 5.0          # constants.js BATTERY_MIN
SECONDS_PER_HOUR = 3600.0
# energyPolicy.powerCoefficients()
BETA_AUX_WH_PER_S = STANDBY_POWER_W / SECONDS_PER_HOUR
BETA_DIST_WH_PER_M = (MOVING_POWER_W - STANDBY_POWER_W) / (SECONDS_PER_HOUR * MOVING_POWER_REF_SPEED_MPS)

LEGACY_TICK_S = 2.0                     # constants.js TELEMETRY_INTERVAL_MS
LEGACY_DRAIN_ACTIVE_PER_TICK = 0.0444   # constants.js BATTERY_DRAIN_ACTIVE (%/tick)
LEGACY_DRAIN_IDLE_PER_TICK = 0.0089     # constants.js BATTERY_DRAIN_IDLE (%/tick)

# telemetry.handler.js: a real robot's distance ignores GPS jumps over 500 m.
MAX_FIX_JUMP_M = 500.0

# --- demo-specific (not from the backend simulator) ---------------------------

FULL_PERCENT = 100.0
DEFAULT_INITIAL_PERCENT = 100.0
DEFAULT_RECHARGE_THRESHOLD_PERCENT = 30.0
DEFAULT_RECHARGE_DURATION_S = 60.0
# Longest single step the simulation integrates; see the module docstring.
MAX_STEP_S = 10.0
DEFAULT_STATE_PATH = "~/.robotx/demo_battery.json"
STATE_VERSION = 1

SIMULATED_STATUS = "SIMULATED"
SIMULATED_SOURCE = "DEMO SIMULATION (ROBOTX_DEMO_BATTERY_ENABLED) -- not a battery measurement"

_EARTH_RADIUS_M = 6_371_000.0


class DrainModel(str, Enum):
    """Which backend simulator drain path to reproduce."""

    POWER = "power"
    LEGACY_PERCENTAGE = "legacy_percentage"


class BatteryPhase(str, Enum):
    DRAINING = "DRAINING"
    # Demo recharge running: no drain, the percentage held.
    RECHARGING = "RECHARGING"
    # At 100 %, waiting for a CHARGING frame carrying it to leave the Pi.
    CHARGED_UNREPORTED = "CHARGED_UNREPORTED"


def drain_percent(
    model: DrainModel,
    *,
    seconds: float,
    working: bool,
    distance_m: float = 0.0,
) -> float:
    """Percentage points the backend simulator drains over one interval."""

    if model is DrainModel.POWER:
        wh = BETA_AUX_WH_PER_S * seconds + BETA_DIST_WH_PER_M * distance_m
        return wh / PACK_NOMINAL_WH * 100.0
    rate = LEGACY_DRAIN_ACTIVE_PER_TICK if working else LEGACY_DRAIN_IDLE_PER_TICK
    return rate * seconds / LEGACY_TICK_S


def haversine_m(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    h = math.sin((lat2 - lat1) / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2) ** 2
    return 2 * _EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class DemoBatteryConfig:
    drain_model: DrainModel = DrainModel.POWER
    initial_percent: float = DEFAULT_INITIAL_PERCENT
    recharge_threshold_percent: float = DEFAULT_RECHARGE_THRESHOLD_PERCENT
    recharge_duration_s: float = DEFAULT_RECHARGE_DURATION_S
    # None keeps the state in memory only (tests).
    state_path: Optional[str] = DEFAULT_STATE_PATH
    # Whether a recharge must be confirmed reported (backend link enabled).
    confirm_reports: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.drain_model, DrainModel):
            raise ValueError(f"drain_model must be one of {[m.value for m in DrainModel]}")
        if not _finite(self.initial_percent) or not BATTERY_MIN_PERCENT <= self.initial_percent <= FULL_PERCENT:
            raise ValueError(
                f"initial_percent must be in [{BATTERY_MIN_PERCENT:g}, 100], got {self.initial_percent!r}"
            )
        # At or below the simulator's floor the threshold could never be reached.
        if (
            not _finite(self.recharge_threshold_percent)
            or not BATTERY_MIN_PERCENT < self.recharge_threshold_percent < FULL_PERCENT
        ):
            raise ValueError(
                f"recharge_threshold_percent must be in ({BATTERY_MIN_PERCENT:g}, 100), "
                f"got {self.recharge_threshold_percent!r}"
            )
        if not _finite(self.recharge_duration_s) or self.recharge_duration_s <= 0:
            raise ValueError(f"recharge_duration_s must be positive, got {self.recharge_duration_s!r}")

    @classmethod
    def from_settings(cls, settings: Any, *, confirm_reports: bool) -> "DemoBatteryConfig":
        try:
            model = DrainModel(str(settings.demo_battery_drain_model).strip().lower())
        except ValueError:
            raise ValueError(
                "ROBOTX_DEMO_BATTERY_DRAIN_MODEL must be one of "
                f"{', '.join(m.value for m in DrainModel)}, got {settings.demo_battery_drain_model!r}"
            ) from None
        return cls(
            drain_model=model,
            initial_percent=settings.demo_battery_initial_percent,
            recharge_threshold_percent=settings.demo_battery_recharge_threshold_percent,
            recharge_duration_s=settings.demo_battery_recharge_s,
            state_path=settings.demo_battery_state_path or None,
            confirm_reports=confirm_reports,
        )


class DemoBatterySimulator:
    """The simulated pack. One instance per agent; thread-safe.

    `update` is the only thing that drains or recharges, and it is called once
    per agent tick, so no interval is ever applied twice.
    """

    def __init__(self, cfg: DemoBatteryConfig, *, clock: Callable[[], float] = time.monotonic) -> None:
        self.cfg = cfg
        self._clock = clock
        self._lock = threading.Lock()
        self._path: Optional[Path] = Path(cfg.state_path).expanduser() if cfg.state_path else None
        self._last_t: Optional[float] = None
        self._last_fix: Optional[Tuple[float, float, float]] = None
        self._persisted: Optional[Tuple[str, float, int]] = None
        self.percent, self.phase, self.recharge_elapsed_s = self._load()
        self._persist()

    # --- reads ---------------------------------------------------------------

    @property
    def charging(self) -> bool:
        return self.phase is not BatteryPhase.DRAINING

    def status_dict(self) -> Dict[str, Any]:
        """The `battery_status()` shape, labelled as simulated."""

        with self._lock:
            return self._status_dict()

    def _status_dict(self) -> Dict[str, Any]:
        return {
            "status": SIMULATED_STATUS,
            "percent": round(self.percent, 1),
            "voltage_v": None,
            "source": SIMULATED_SOURCE,
            "charging": self.charging,
            "phase": self.phase.value,
        }

    # --- the one writer ------------------------------------------------------

    def update(
        self,
        *,
        working: bool,
        idle: bool,
        fix: Optional[Tuple[float, float, float]] = None,
        now: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Advance the simulation to `now` and return the status dict.

        `working`: a mission is being driven (legacy drain rate, distance
        counted). `idle`: no mission at all and mode IDLE -- the only state in
        which a demo recharge may run. `fix`: the latest measured
        `(lat, lon, timestamp)`, or None.
        """

        with self._lock:
            now = self._clock() if now is None else now
            dt = 0.0 if self._last_t is None or not _finite(now) else now - self._last_t
            if _finite(now):
                self._last_t = now
            dt = min(max(dt, 0.0), MAX_STEP_S)
            distance = self._distance_since_last_fix(fix, working)

            if self.phase is BatteryPhase.DRAINING:
                if dt > 0 or distance > 0:
                    lost = drain_percent(self.cfg.drain_model, seconds=dt, working=working, distance_m=distance)
                    self.percent = max(BATTERY_MIN_PERCENT, self.percent - lost)
                if idle and self.percent <= self.cfg.recharge_threshold_percent:
                    self._enter(BatteryPhase.RECHARGING, "demo recharge started")
            elif self.phase is BatteryPhase.RECHARGING:
                if not idle:
                    # A mission takes priority; the recharge restarts when idle.
                    self._enter(BatteryPhase.DRAINING, "demo recharge cancelled: robot is no longer idle")
                else:
                    self.recharge_elapsed_s += dt
                    if self.recharge_elapsed_s >= self.cfg.recharge_duration_s:
                        self.percent = FULL_PERCENT
                        self._enter(BatteryPhase.CHARGED_UNREPORTED, "demo recharge complete")
            if self.phase is BatteryPhase.CHARGED_UNREPORTED and not self.cfg.confirm_reports:
                self._enter(BatteryPhase.DRAINING, "drain resumed (no backend link to report to)")

            self._persist()
            return self._status_dict()

    def note_reported(self, payload: Mapping[str, Any]) -> bool:
        """The backend link sent `payload` as TELEMETRY. True if that ended a recharge.

        Only a CHARGING frame carrying the post-recharge value counts: a frame
        built from a snapshot taken before the jump carries the old value and
        does not.
        """

        with self._lock:
            if self.phase is not BatteryPhase.CHARGED_UNREPORTED:
                return False
            battery = payload.get("battery")
            if payload.get("status") != "CHARGING" or not _finite(battery):
                return False
            if battery < round(self.percent, 1):
                return False
            self._enter(BatteryPhase.DRAINING, "demo recharge reported; drain resumed")
            self._persist()
            return True

    # --- internals -----------------------------------------------------------

    def _enter(self, phase: BatteryPhase, why: str) -> None:
        self.phase = phase
        self.recharge_elapsed_s = 0.0
        log_event(
            logger,
            "demo_battery.phase",
            why,
            phase=phase.value,
            percent=round(self.percent, 1),
        )

    def _distance_since_last_fix(self, fix: Optional[Tuple[float, float, float]], working: bool) -> float:
        if fix is None or not all(_finite(v) for v in fix):
            return 0.0
        previous, self._last_fix = self._last_fix, fix
        # Distance is counted only while driving a mission, between two
        # successive, newer fixes; anything else just re-anchors.
        if not working or previous is None or fix[2] <= previous[2]:
            return 0.0
        moved = haversine_m(previous[:2], fix[:2])
        return moved if moved < MAX_FIX_JUMP_M else 0.0

    def _fresh(self) -> Tuple[float, BatteryPhase, float]:
        # A fresh pack is announced in a CHARGING frame (see the module docstring).
        return float(self.cfg.initial_percent), BatteryPhase.CHARGED_UNREPORTED, 0.0

    def _load(self) -> Tuple[float, BatteryPhase, float]:
        if self._path is None:
            return self._fresh()
        try:
            raw = self._path.read_text(encoding="utf-8")
        except FileNotFoundError:
            log_event(logger, "demo_battery.fresh", "no saved demo battery; starting a fresh session",
                      percent=self.cfg.initial_percent)
            return self._fresh()
        except OSError as e:
            return self._invalid(f"unreadable: {e!r}")
        try:
            data = json.loads(raw)
        except ValueError as e:
            return self._invalid(f"not JSON: {e}")
        if not isinstance(data, dict) or data.get("version") != STATE_VERSION:
            return self._invalid("unknown shape or version")
        percent, elapsed = data.get("percent"), data.get("recharge_elapsed_s", 0.0)
        try:
            phase = BatteryPhase(data.get("phase"))
        except ValueError:
            return self._invalid(f"unknown phase {data.get('phase')!r}")
        if not _finite(percent) or not 0.0 <= percent <= FULL_PERCENT:
            return self._invalid(f"percent out of range: {percent!r}")
        if not _finite(elapsed) or elapsed < 0:
            return self._invalid(f"recharge_elapsed_s invalid: {elapsed!r}")
        percent = max(BATTERY_MIN_PERCENT, float(percent))
        elapsed = min(float(elapsed), self.cfg.recharge_duration_s) if phase is BatteryPhase.RECHARGING else 0.0
        log_event(logger, "demo_battery.resumed", "demo battery restored", percent=round(percent, 1),
                  phase=phase.value)
        return percent, phase, elapsed

    def _invalid(self, why: str) -> Tuple[float, BatteryPhase, float]:
        log_event(logger, "demo_battery.state_invalid", "saved demo battery ignored; starting fresh",
                  level=logging.WARNING, reason=why, path=str(self._path))
        return self._fresh()

    def _persist(self) -> None:
        # Whole recharge seconds too, so a restart mid-recharge resumes its timer.
        key = (self.phase.value, round(self.percent, 1), int(self.recharge_elapsed_s))
        if self._path is None or key == self._persisted:
            return
        record = {
            "version": STATE_VERSION,
            "percent": self.percent,
            "phase": self.phase.value,
            "recharge_elapsed_s": self.recharge_elapsed_s,
            "drain_model": self.cfg.drain_model.value,
            "updated_at": time.time(),
        }
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp_name = tempfile.mkstemp(dir=str(self._path.parent), prefix=".demo-battery-")
            try:
                if hasattr(os, "fchmod"):
                    os.fchmod(fd, 0o600)
                with os.fdopen(fd, "w", encoding="utf-8") as fh:
                    json.dump(record, fh)
                os.replace(tmp_name, self._path)
            except BaseException:
                try:
                    os.unlink(tmp_name)
                except OSError:
                    pass
                raise
        except OSError as e:
            log_event(logger, "demo_battery.persist_failed", "demo battery state not saved",
                      level=logging.WARNING, error=repr(e), path=str(self._path))
            return
        self._persisted = key
