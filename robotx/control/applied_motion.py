"""Applied-motion monitor: the ESP32 says it is not turning the wheels the Pi drives.

The link already refuses to send DRIVE into a controller whose fresh TELEMETRY
reports a fault, and the progress monitor pauses a rover that gets no closer
after `nav_no_progress_after_s` (60 s) of driving. Between the two sat a gap:
the ESP32 answers every DRIVE with what it actually did -- `ACK GATED` when its
own gates limited the output, `REJECTED` when it refused it, and `applied_*`
for what the motor layer outputs (PROTOCOL.md sections 9 and 11) -- and nothing
on the Pi read it. A rover whose controller zeroed every command sat "driving"
for a minute.

This module reads that feedback and turns a bounded disagreement into a pause
with the reason, the way `robotx.navigation.progress` does for a stall:

- a DRIVE ACK `REJECTED` (other than the sequence resync `STALE_SEQ`) is a
  fault at once: the controller did not execute what the Pi is driving on;
- DRIVE ACKs `GATED` for `applied_timeout_s` without an ACCEPTED one between
  are a fault: the ESP32 has been limiting the output the whole time;
- `applied_timeout_s` of fresh TELEMETRY in which every side the Pi drives at
  or beyond the motor deadband reports `applied` 0 is a fault (and at least
  `MIN_ZERO_FRAMES` distinct frames, so one late frame cannot trip it).

It is only ever evaluated while the Pi is genuinely commanding motion: the
mission is AUTO and the last command the link wrote is a DRIVE with a side
at or beyond the deadband. A STOP, a pause, motion disabled (the link writes
nothing), and a DRIVE inside the deadband (`applied` is then 0 by design) all
reset it. Stale or missing TELEMETRY is no evidence either way: the timer
neither advances nor resets on it -- the link's own staleness rule stops the
rover in that case.

It never produces motion and never clears anything; resuming after its pause
is an explicit RESUME, like every other pause. Pure and clock-injected.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any, Callable, List, Optional, Tuple

from robotx.config.logging_setup import log_event
from robotx.esp32.state import ControllerState, DriveAck, SentMotion


logger = logging.getLogger(__name__)

# PROTOCOL.md section 9: |v| < 40 is output as 0 by the ESP32 motor layer.
DEFAULT_DEADBAND_UNITS = 40

# Bounds on the configurable timeout. Below the lower one, two TELEMETRY frames
# (200 ms apart) plus the ACK latency would not fit and a healthy start would
# trip it; above the upper one a rover that is not moving is reported as
# driving for longer than the ESP32's own 2 s watchdog is allowed to run.
MIN_APPLIED_TIMEOUT_S = 0.5
MAX_APPLIED_TIMEOUT_S = 5.0

# Distinct fresh TELEMETRY frames (by uptime) of zero output before it counts.
MIN_ZERO_FRAMES = 2

# REJECTED reasons that are the protocol working, not the controller refusing to
# drive: STALE_SEQ is answered by resynchronising the sequence (section 8).
TRANSIENT_REJECTIONS = frozenset({"STALE_SEQ"})


@dataclass(frozen=True)
class AppliedMotionConfig:
    applied_timeout_s: float = 1.5
    deadband_units: int = DEFAULT_DEADBAND_UNITS

    def __post_init__(self) -> None:
        t = self.applied_timeout_s
        if not (isinstance(t, (int, float)) and MIN_APPLIED_TIMEOUT_S <= t <= MAX_APPLIED_TIMEOUT_S):
            raise ValueError(
                f"applied-motion timeout must be in [{MIN_APPLIED_TIMEOUT_S:g}, "
                f"{MAX_APPLIED_TIMEOUT_S:g}] s, got {t!r}"
            )
        d = self.deadband_units
        if isinstance(d, bool) or not isinstance(d, int) or not 0 <= d <= 254:
            raise ValueError(f"motor deadband must be an integer in [0, 254] DRIVE units, got {d!r}")

    @classmethod
    def from_settings(cls, settings: Any) -> "AppliedMotionConfig":
        return cls(
            applied_timeout_s=settings.motion_applied_timeout_s,
            deadband_units=settings.motion_deadband_units,
        )


@dataclass(frozen=True)
class AppliedMotionVerdict:
    fault: bool = False   # pause now: once per episode
    kind: str = ""        # DRIVE_REJECTED | DRIVE_GATED | ZERO_APPLIED
    reason: str = ""


OK = AppliedMotionVerdict()


def driven_sides(sent: Optional[SentMotion], deadband: int) -> List[str]:
    """The sides a written DRIVE asks to turn: those at or beyond the deadband."""

    if sent is None or sent.cmd != "DRIVE":
        return []
    return [side for side, units in (("left", sent.left), ("right", sent.right)) if abs(units) >= max(1, deadband)]


class AppliedMotionMonitor:
    """Compares what the Pi is driving with what the ESP32 says it applies."""

    def __init__(
        self,
        cfg: Optional[AppliedMotionConfig] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.cfg = cfg or AppliedMotionConfig()
        self.clock = clock
        self._drive_acks_seen: Optional[int] = None
        self.reset()

    def reset(self) -> None:
        """Forget the episode: a new mission, a RESUME, a stop."""

        self._gated_since: Optional[float] = None
        self._gated_reason = ""
        # Zero output is timed only across consecutive fresh frames: a stale gap
        # pauses the count (`_zero_last` None) without discarding it.
        self._zero_s = 0.0
        self._zero_last: Optional[float] = None
        self._zero_frames: List[int] = []
        self._faulted = False

    def update(
        self,
        *,
        commanding: bool,
        controller: Optional[ControllerState],
        stale_after_s: float,
    ) -> AppliedMotionVerdict:
        """One agent tick. `commanding`: the mission is AUTO and its gated intent moves."""

        acks = None if controller is None else controller.counters.drive_acks
        new_ack = acks is not None and self._drive_acks_seen is not None and acks > self._drive_acks_seen
        if acks is not None:
            self._drive_acks_seen = acks

        sides = driven_sides(None if controller is None else controller.last_motion_sent,
                             self.cfg.deadband_units)
        if not commanding or controller is None or not sides:
            self.reset()
            return OK
        if self._faulted:
            return OK            # once per episode; the pause ends it

        now = self.clock()
        verdict = self._acks(controller.last_drive_ack, new_ack, now)
        if verdict.fault:
            return self._fault(verdict)
        verdict = self._applied(controller, sides, stale_after_s, now)
        if verdict.fault:
            return self._fault(verdict)
        return OK

    # --- evidence -------------------------------------------------------------

    def _acks(self, ack: Optional[DriveAck], new_ack: bool, now: float) -> AppliedMotionVerdict:
        if ack is not None and new_ack:
            if ack.result == "REJECTED" and ack.reason not in TRANSIENT_REJECTIONS:
                return AppliedMotionVerdict(
                    True, "DRIVE_REJECTED",
                    f"ESP32 REJECTED a DRIVE ({ack.reason}); it is not executing the Pi's commands",
                )
            if ack.result == "GATED":
                if self._gated_since is None:
                    self._gated_since = now
                self._gated_reason = ack.reason
            elif ack.result == "ACCEPTED":
                self._gated_since = None
        if self._gated_since is not None and now - self._gated_since >= self.cfg.applied_timeout_s:
            return AppliedMotionVerdict(
                True, "DRIVE_GATED",
                f"ESP32 GATED every DRIVE for {now - self._gated_since:.1f}s ({self._gated_reason}); "
                "its own gate is limiting the output",
            )
        return OK

    def _applied(
        self, controller: ControllerState, sides: List[str], stale_after_s: float, now: float
    ) -> AppliedMotionVerdict:
        tel = controller.telemetry
        age = controller.telemetry_age_s
        if tel is None or age is None or age > stale_after_s:
            self._zero_last = None   # no evidence: neither advance nor reset
            return OK
        applied = {"left": tel.left_applied, "right": tel.right_applied}
        if any(applied[side] != 0 for side in sides):
            self._zero_s, self._zero_last, self._zero_frames = 0.0, None, []
            return OK
        if self._zero_last is not None:
            self._zero_s += max(0.0, now - self._zero_last)
        self._zero_last = now
        if tel.uptime_ms not in self._zero_frames:
            self._zero_frames.append(tel.uptime_ms)
        held = self._zero_s
        if held < self.cfg.applied_timeout_s or len(self._zero_frames) < MIN_ZERO_FRAMES:
            return OK
        sent = controller.last_motion_sent
        drive: Tuple[int, int] = (sent.left, sent.right) if sent is not None else (0, 0)
        return AppliedMotionVerdict(
            True, "ZERO_APPLIED",
            f"DRIVE {drive[0]}/{drive[1]} commanded for {held:.1f}s but the ESP32 applies "
            f"{tel.left_applied}/{tel.right_applied} (state {tel.state}, block {tel.block_reason})",
        )

    def _fault(self, verdict: AppliedMotionVerdict) -> AppliedMotionVerdict:
        self._faulted = True
        log_event(logger, "motion.not_applied", verdict.reason, level=logging.WARNING, kind=verdict.kind)
        return verdict
