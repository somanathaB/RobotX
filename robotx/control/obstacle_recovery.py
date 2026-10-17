"""Obstacle stops: when the rover may move again, and when to ask for help (RX-08).

The decision layer stops for an obstacle (or a person) on the tick it sees
one, and proposes motion again on the first tick it does not. Acting on that
directly has two problems, and this module is the policy for both:

- A detection that flickers -- present, absent, present -- would be driven
  into on its absent frames. So once an obstacle has stopped the rover, motion
  resumes only after the path has been seen clear, by USABLE perception,
  continuously for `clear_hold_s`. Unusable perception (stale, no frame,
  detector error) is not a clear path: it restarts the hold.
- An obstacle that never goes away must not hold the rover silently forever.
  After `assist_after_s` in one blocked episode the agent pauses the mission
  and asks for an operator, through the existing PAUSED -> RESUME flow.

A timer never clears an obstacle. `assist_after_s` only escalates; leaving a
blocked episode takes the clear-path evidence above, however long it has been.

This replaces feeding every obstacle tick into the route planner's blocked
counter, which turned a half-second stop into a permanent REROUTE_NEEDED on a
rover that has no rerouting.

Pure and clock-injected: no I/O, deterministic under a fake clock. It never
produces motion -- only a reason to hold, or nothing.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

from robotx.config.logging_setup import log_event


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class ObstacleRecoveryConfig:
    # Continuous clear, usable perception required before moving after an
    # obstacle stop. Several perception cycles, so one empty frame is not it.
    clear_hold_s: float = 2.0
    # One blocked episode this long pauses the mission for an operator.
    assist_after_s: float = 30.0

    def __post_init__(self) -> None:
        if not self.clear_hold_s > 0:
            raise ValueError(f"clear_hold_s must be > 0, got {self.clear_hold_s}")
        if not self.assist_after_s > self.clear_hold_s:
            raise ValueError(
                f"assist_after_s ({self.assist_after_s}) must exceed clear_hold_s ({self.clear_hold_s})"
            )

    @classmethod
    def from_settings(cls, settings: Any) -> "ObstacleRecoveryConfig":
        return cls(
            clear_hold_s=settings.obstacle_clear_hold_s,
            assist_after_s=settings.obstacle_assist_after_s,
        )


@dataclass(frozen=True)
class RecoveryVerdict:
    hold: bool = False                      # no motion this tick, whatever was proposed
    assist: bool = False                    # escalate now: once per blocked episode
    reason: str = ""
    blocked_for_s: Optional[float] = None   # age of the current episode, if any


CLEAR = RecoveryVerdict()


class ObstacleRecovery:
    """Tracks one blocked episode at a time for the active mission."""

    def __init__(
        self,
        cfg: Optional[ObstacleRecoveryConfig] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.cfg = cfg or ObstacleRecoveryConfig()
        self.clock = clock
        self._since: Optional[float] = None        # first obstacle tick of the episode
        self._clear_since: Optional[float] = None  # start of the current clear run
        self._assisted = False

    @property
    def blocked(self) -> bool:
        """Whether a blocked episode is open (obstacle seen, path not yet proven clear)."""

        return self._since is not None

    def reset(self) -> None:
        """Forget the episode: a new mission, or none."""

        self._since = None
        self._clear_since = None
        self._assisted = False

    def restart(self) -> None:
        """An operator RESUME: an open episode starts over, it is not forgotten.

        The obstacle that stopped the rover is still a fact until perception
        shows the path clear for `clear_hold_s`; resuming only re-arms the
        assistance escalation for this new attempt.
        """

        if self._since is not None:
            self._since = self.clock()
            self._clear_since = None
            self._assisted = False

    def update(self, *, active: bool, obstacle: bool, perception_usable: bool) -> RecoveryVerdict:
        """One agent tick. `obstacle`: this tick's gated intent stopped for one."""

        if not active:
            # Paused or idle: nothing moves anyway. An open episode is kept --
            # pausing proves nothing about the path -- until `restart`/`reset`.
            return CLEAR

        now = self.clock()
        if obstacle:
            if self._since is None:
                self._since = now
                log_event(logger, "obstacle.blocked", "obstacle stop: holding until the path is clear")
            self._clear_since = None
            return self._held(now, "obstacle in path")

        if self._since is None:
            return CLEAR

        if not perception_usable:
            self._clear_since = None
            return self._held(now, "path not confirmed clear: perception unusable")

        if self._clear_since is None:
            self._clear_since = now
        clear_for = now - self._clear_since
        if clear_for >= self.cfg.clear_hold_s:
            log_event(
                logger,
                "obstacle.cleared",
                f"path clear for {clear_for:.1f}s; moving again",
                blocked_for_s=round(now - self._since, 1),
            )
            self.reset()
            return CLEAR
        return self._held(
            now, f"waiting for a sustained clear path ({clear_for:.1f}s of {self.cfg.clear_hold_s:.1f}s)"
        )

    def _held(self, now: float, reason: str) -> RecoveryVerdict:
        blocked_for = now - self._since
        assist = not self._assisted and blocked_for >= self.cfg.assist_after_s
        if assist:
            self._assisted = True
            log_event(
                logger,
                "obstacle.assistance_needed",
                f"blocked for {blocked_for:.0f}s: pausing for an operator",
                level=logging.WARNING,
            )
        return RecoveryVerdict(hold=True, assist=assist, reason=reason, blocked_for_s=blocked_for)
