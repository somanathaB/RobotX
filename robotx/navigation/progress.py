"""Navigation stalls: when a stopped or creeping rover must ask for help.

The decision layer already stops the rover for every navigation reason it
cannot drive on -- no GPS fix, a stale or inaccurate one, off route -- and the
stop lasts exactly as long as the reason does. Nothing ended it: a rover whose
receiver lost its fix, or whose route it had left, would sit holding forever
while reporting itself AUTO. Likewise a rover commanded forward that gets no
closer -- wheels slipping, a duty too low to move it, creeping the wrong way
for want of a heading -- would drive on indefinitely.

This module bounds both, the way `robotx.control.obstacle_recovery` bounds an
obstacle stop: after a limit, the agent pauses the mission with the reason and
an operator decides (PAUSED -> RESUME). It never produces motion and never
clears anything:

- one navigation hold (`NO_POSITION`, `REROUTE_NEEDED`) lasting
  `hold_assist_after_s` escalates. A hold that ends sooner -- the fix comes
  back -- is forgotten, and navigation simply resumes on the valid fix;
- `no_progress_after_s` of *commanded* motion (time spent stopped for any
  reason does not count) without the distance to the current waypoint ever
  improving by `min_progress_m` on its best so far escalates. Reaching the
  next waypoint is progress. GPS wander up and down cannot fake progress: only
  a new best by a clear margin counts.

Pure and clock-injected: no I/O, deterministic under a fake clock.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

from robotx.config.logging_setup import log_event
from robotx.navigation.navigator import NavigationState, NavigationStatus


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class ProgressConfig:
    # One navigation hold this long pauses the mission for an operator.
    hold_assist_after_s: float = 30.0
    # Commanded motion this long without getting closer pauses it too.
    no_progress_after_s: float = 60.0
    # An improvement smaller than this on the best distance is not progress.
    min_progress_m: float = 3.0

    def __post_init__(self) -> None:
        for name in ("hold_assist_after_s", "no_progress_after_s", "min_progress_m"):
            if not getattr(self, name) > 0:
                raise ValueError(f"{name} must be > 0, got {getattr(self, name)}")

    @classmethod
    def from_settings(cls, settings: Any) -> "ProgressConfig":
        return cls(
            hold_assist_after_s=settings.nav_hold_assist_after_s,
            no_progress_after_s=settings.nav_no_progress_after_s,
            min_progress_m=settings.nav_progress_min_m,
        )


@dataclass(frozen=True)
class ProgressVerdict:
    assist: bool = False   # escalate now: once per stall
    reason: str = ""


CLEAR = ProgressVerdict()

_HOLD_CAUSES = {
    NavigationStatus.NO_POSITION: "no usable GPS position",
    NavigationStatus.REROUTE_NEEDED: "off route",
}


class ProgressMonitor:
    """Times navigation holds and progress for the active mission."""

    def __init__(
        self,
        cfg: Optional[ProgressConfig] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.cfg = cfg or ProgressConfig()
        self.clock = clock
        self.reset()

    def reset(self) -> None:
        """Forget everything: a new mission, a RESUME, or none."""

        self._last_t: Optional[float] = None
        self._hold_cause: Optional[str] = None
        self._hold_s = 0.0
        self._hold_assisted = False
        self._waypoint: Optional[int] = None
        self._best_m: Optional[float] = None
        self._stalled_s = 0.0
        self._stall_assisted = False

    def update(self, *, active: bool, navigation: NavigationState, moving: bool) -> ProgressVerdict:
        """One agent tick. `moving`: this tick's gated intent drives the wheels."""

        if not active:
            # Paused or idle: no time passes for either timer.
            self._last_t = None
            return CLEAR
        now = self.clock()
        dt = 0.0 if self._last_t is None else max(0.0, now - self._last_t)
        self._last_t = now

        verdict = self._hold(navigation, dt)
        progress = self._progress(navigation, moving, dt)
        return verdict if verdict.assist else progress

    def _hold(self, navigation: NavigationState, dt: float) -> ProgressVerdict:
        cause = _HOLD_CAUSES.get(navigation.status)
        if cause is not None and navigation.position_issue:
            cause = "GPS fix not accurate enough to navigate"
        if cause is None:
            self._hold_cause, self._hold_s, self._hold_assisted = None, 0.0, False
            return CLEAR
        if cause != self._hold_cause:
            self._hold_cause, self._hold_s, self._hold_assisted = cause, 0.0, False
        else:
            self._hold_s += dt
        if self._hold_assisted or self._hold_s < self.cfg.hold_assist_after_s:
            return CLEAR
        self._hold_assisted = True
        reason = f"navigation: {cause} for {self._hold_s:.0f}s: operator assistance needed"
        log_event(logger, "navigation.assistance_needed", reason, level=logging.WARNING)
        return ProgressVerdict(assist=True, reason=reason)

    def _progress(self, navigation: NavigationState, moving: bool, dt: float) -> ProgressVerdict:
        distance = navigation.distance_to_target_m
        if navigation.status is not NavigationStatus.NAVIGATING or distance is None:
            return CLEAR
        if navigation.waypoint_index != self._waypoint or self._best_m is None:
            self._waypoint, self._best_m = navigation.waypoint_index, distance
            self._stalled_s, self._stall_assisted = 0.0, False
            return CLEAR
        if distance <= self._best_m - self.cfg.min_progress_m:
            self._best_m, self._stalled_s, self._stall_assisted = distance, 0.0, False
            return CLEAR
        if moving:
            self._stalled_s += dt
        if self._stall_assisted or self._stalled_s < self.cfg.no_progress_after_s:
            return CLEAR
        self._stall_assisted = True
        reason = (
            f"navigation: no progress toward waypoint {navigation.waypoint_index + 1}/"
            f"{navigation.waypoints_total} after {self._stalled_s:.0f}s of driving: "
            "operator assistance needed"
        )
        log_event(logger, "navigation.assistance_needed", reason, level=logging.WARNING)
        return ProgressVerdict(assist=True, reason=reason)
