"""One bounded, operator-authorized forward pulse for a rover on blocks.

Purpose
-------
The first physical check of the Pi -> ESP32 -> motor-driver path: does a short,
slow, forward DRIVE leave the Pi, get ACCEPTED by the ESP32, show up as
applied output, and stop again on command? It is NOT a mission: no route, no
taskId, no commitment, nothing reported to the backend as an assignment, and it
never starts on its own (`RobotAgent.start_bench_motion` is reached only from
the guarded, disabled-by-default route in `robotx.application.bench_gate`).

What it does not bypass
-----------------------
It only *proposes* `MotionIntent.forward(speed)` -- equal sides, forward only.
The agent evaluates that proposal with the same `SafetyGate` a mission uses
(e-stop, perception, forward range, speed ceiling, intent age), and the result
goes through the same `Esp32Link.submit`, which still refuses DRIVE unless
`ROBOTX_ESP32_MOTION_ENABLED` is set and the controller is motion-ready. The
ESP32's own gates still apply on top.

When it stops
-------------
Any of these ends the pulse at once (a STOP is the very next command written):

- the hard deadline (`duration_s`, at most `MAX_DURATION_S`);
- the safety gate turning the pulse into a stop, for any reason (obstacle,
  person, perception, range, e-stop) -- and it is never resumed: there is no
  obstacle-recovery hold here, a cleared obstacle restarts nothing;
- the link losing motion readiness (stale TELEMETRY, a controller safety fault,
  drive unavailable, reboot latch), or being lost and re-proven in between;
- a DRIVE ACK that is not ACCEPTED, or ACCEPTED with `req`/`applied` other
  than exactly what was sent; no ACK within `ack_timeout_s`;
- fresh TELEMETRY reporting applied output outside [0, sent] on either side,
  or nothing applied `applied_grace_s` after the ACK;
- the agent's applied-motion monitor reporting a fault;
- the agent leaving IDLE, a mission or e-stop arriving, or an operator stop.

After the pulse, the STOP must be *confirmed*: a STOP written, then fresh
TELEMETRY received after it reporting commanded and applied output 0 on both
sides, within `stop_confirm_s`. If not, the result is STOP_UNCONFIRMED and the
agent latches its emergency stop.

The applied values are what the firmware says it outputs. They are evidence
the command reached the motor layer, never proof that a wheel turned: the
operator watching the wheels is the only witness of that.

Pure and clock-injected; no I/O.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

from robotx.config.logging_setup import log_event
from robotx.control.motion import MotionIntent

logger = logging.getLogger(__name__)

# Hard ceilings, whatever the configuration says.
MAX_DURATION_S = 3.0
MIN_DURATION_S = 0.5
MAX_SPEED = 0.35          # normalized; 0.35 -> 89 DRIVE units of 255
COOLDOWN_S = 10.0


def drive_units(value: float) -> int:
    """The link's own conversion (`robotx.esp32.link.to_drive_units`), restated to keep this module I/O-free."""

    v = max(-1.0, min(1.0, float(value)))
    return int((1 if v >= 0 else -1) * int(abs(v) * 255 + 0.5))


@dataclass(frozen=True)
class BenchMotionConfig:
    speed: float = 0.25            # the most a run may request (normalized)
    duration_s: float = 2.0        # the longest a run may last
    deadband_units: int = 40       # the ESP32's: |v| < this is output as 0
    ack_timeout_s: float = 0.5     # first DRIVE -> its ACK
    applied_grace_s: float = 0.6   # ACK -> fresh TELEMETRY showing it applied
    stop_confirm_s: float = 1.5    # end of pulse -> STOP confirmed by TELEMETRY
    stale_after_s: float = 1.0     # the link's TELEMETRY freshness limit

    def __post_init__(self) -> None:
        if not MIN_DURATION_S <= self.duration_s <= MAX_DURATION_S:
            raise ValueError(f"bench duration_s must be in [{MIN_DURATION_S}, {MAX_DURATION_S}], got {self.duration_s}")
        if not self.min_speed <= self.speed <= MAX_SPEED:
            raise ValueError(f"bench speed must be in [{self.min_speed:.3f}, {MAX_SPEED}] (above the "
                             f"{self.deadband_units}-unit deadband), got {self.speed}")

    @property
    def min_speed(self) -> float:
        """The least normalized speed whose DRIVE value clears the deadband."""

        return (self.deadband_units + 0.5) / 255.0

    @classmethod
    def from_settings(cls, settings: Any) -> "BenchMotionConfig":
        return cls(
            speed=settings.bench_motion_speed,
            duration_s=settings.bench_motion_duration_s,
            deadband_units=settings.motion_deadband_units,
            stale_after_s=settings.esp32_stale_after_s,
        )


class BenchRefused(Exception):
    """A bench run the agent will not start, with a stable code."""

    def __init__(self, code: str, detail: str) -> None:
        super().__init__(f"{code}: {detail}")
        self.code = code
        self.detail = detail


class BenchPhase(str, Enum):
    IDLE = "IDLE"
    DRIVING = "DRIVING"
    STOPPING = "STOPPING"


class BenchOutcome(str, Enum):
    COMPLETED = "COMPLETED"              # ran to its deadline; STOP confirmed
    ABORTED = "ABORTED"                  # a stop condition fired; STOP confirmed
    STOP_UNCONFIRMED = "STOP_UNCONFIRMED"  # the STOP was never confirmed: e-stop latched


@dataclass
class _Run:
    speed: float
    units: int
    duration_s: float
    source: str
    started_at: float                    # bench clock
    started_wall: float
    deadline: float
    drive_seqs: List[int] = field(default_factory=list)
    first_drive_at: Optional[float] = None
    first_ack_at: Optional[float] = None
    acks: List[Dict[str, Any]] = field(default_factory=list)
    max_applied: Tuple[int, int] = (0, 0)
    applied_seen: bool = False
    stop_reason: Optional[str] = None
    stop_code: Optional[str] = None
    stopping_since: Optional[float] = None
    stop_seq: Optional[int] = None


class BenchMotion:
    """At most one bench pulse at a time, and a record of the last one."""

    def __init__(self, cfg: Optional[BenchMotionConfig] = None, clock: Callable[[], float] = time.monotonic) -> None:
        self.cfg = cfg or BenchMotionConfig()
        self.clock = clock
        self._run: Optional[_Run] = None
        self._phase = BenchPhase.IDLE
        self._cooldown_until = 0.0
        self.last_result: Optional[Dict[str, Any]] = None

    # --- reads -------------------------------------------------------------------

    @property
    def phase(self) -> BenchPhase:
        return self._phase

    @property
    def active(self) -> bool:
        return self._phase is not BenchPhase.IDLE

    @property
    def driving(self) -> bool:
        return self._phase is BenchPhase.DRIVING

    def cooling_down(self, now: Optional[float] = None) -> float:
        """Seconds left before another run may start (0 when it may)."""

        now = self.clock() if now is None else now
        return max(0.0, self._cooldown_until - now)

    def status(self) -> Dict[str, Any]:
        run = self._run
        return {
            "phase": self._phase.value,
            "cooldown_s": round(self.cooling_down(), 1),
            "limits": {"max_speed": self.cfg.speed, "max_units": drive_units(self.cfg.speed),
                       "max_duration_s": self.cfg.duration_s},
            "running": None if run is None else {"units": run.units, "duration_s": run.duration_s,
                                                 "elapsed_s": round(self.clock() - run.started_at, 2)},
            "last_result": self.last_result,
        }

    # --- control -------------------------------------------------------------------

    def start(self, *, speed: float, duration_s: float, source: str) -> Dict[str, Any]:
        """Begin a pulse. The caller has already checked every precondition."""

        if self.active:
            raise RuntimeError("a bench run is already in progress")
        if not self.cfg.min_speed <= speed <= self.cfg.speed:
            raise ValueError(f"speed must be in [{self.cfg.min_speed:.3f}, {self.cfg.speed}]")
        if not MIN_DURATION_S <= duration_s <= self.cfg.duration_s:
            raise ValueError(f"duration_s must be in [{MIN_DURATION_S}, {self.cfg.duration_s}]")
        now = self.clock()
        self._run = _Run(speed=speed, units=drive_units(speed), duration_s=duration_s, source=source,
                         started_at=now, started_wall=time.time(), deadline=now + duration_s)
        self._phase = BenchPhase.DRIVING
        log_event(logger, "bench.started", "BENCH forward pulse: wheels must be off the ground",
                  level=logging.WARNING, units=self._run.units, duration_s=duration_s, source=source)
        return self.status()

    def intent(self, now: Optional[float] = None) -> MotionIntent:
        """This tick's proposal: forward while driving and before the deadline, else stop."""

        now = self.clock() if now is None else now
        run = self._run
        if self._phase is BenchPhase.DRIVING and run is not None and now < run.deadline:
            return MotionIntent.forward(run.speed, reason=f"bench test: forward pulse ({run.units} units)")
        reason = run.stop_reason if run is not None and run.stop_reason else "bench test: pulse ended"
        return MotionIntent.stop(f"bench test: {reason}" if not reason.startswith("bench test") else reason)

    def abort(self, code: str, reason: str, now: Optional[float] = None) -> None:
        """End the pulse now. Idempotent; a pulse already stopping keeps its first reason."""

        if self._phase is not BenchPhase.DRIVING or self._run is None:
            return
        now = self.clock() if now is None else now
        self._run.stop_code, self._run.stop_reason = code, reason
        self._run.stopping_since = now
        self._phase = BenchPhase.STOPPING
        log_event(logger, "bench.stopping", reason, level=logging.WARNING, code=code)

    def observe(
        self,
        *,
        gated: MotionIntent,
        controller: Any,
        motion_ready: bool,
        interrupted: bool,
        estop: bool,
        mode_idle: bool,
        mission_held: bool,
        applied_fault: Optional[str] = None,
        now: Optional[float] = None,
    ) -> Optional[BenchOutcome]:
        """Judge one tick. Returns the outcome on the tick the run ends, else None."""

        now = self.clock() if now is None else now
        run = self._run
        if run is None or self._phase is BenchPhase.IDLE:
            return None
        if self._phase is BenchPhase.DRIVING:
            self._record_feedback(run, controller, now)
            problem = self._stop_condition(run, now, gated=gated, controller=controller, motion_ready=motion_ready,
                                           interrupted=interrupted, estop=estop, mode_idle=mode_idle,
                                           mission_held=mission_held, applied_fault=applied_fault)
            if problem is not None:
                self.abort(*problem, now=now)
            elif now >= run.deadline:
                run.stop_code, run.stop_reason = "COMPLETED", "pulse duration reached"
                run.stopping_since = now
                self._phase = BenchPhase.STOPPING
            return None
        return self._confirm_stop(run, controller, now)

    # --- internals -----------------------------------------------------------------

    def _record_feedback(self, run: _Run, controller: Any, now: float) -> None:
        sent = getattr(controller, "last_motion_sent", None)
        if sent is not None and sent.cmd == "DRIVE" and sent.seq is not None and sent.seq not in run.drive_seqs \
                and (sent.age_s is None or sent.age_s <= now - run.started_at + 0.05):
            run.drive_seqs.append(sent.seq)
            if run.first_drive_at is None:
                run.first_drive_at = now
        ack = getattr(controller, "last_drive_ack", None)
        if ack is not None and ack.seq in run.drive_seqs and not any(a["seq"] == ack.seq for a in run.acks):
            run.acks.append({"seq": ack.seq, "result": ack.result, "reason": ack.reason,
                             "req": [ack.req_left, ack.req_right], "applied": [ack.applied_left, ack.applied_right]})
            if run.first_ack_at is None:
                run.first_ack_at = now
        tel = self._fresh_telemetry(controller)
        if tel is not None and run.first_ack_at is not None:
            left, right = tel.left_applied, tel.right_applied
            run.max_applied = (max(run.max_applied[0], left), max(run.max_applied[1], right))
            if left > 0 and right > 0:
                run.applied_seen = True

    def _stop_condition(self, run: _Run, now: float, *, gated: MotionIntent, controller: Any, motion_ready: bool,
                        interrupted: bool, estop: bool, mode_idle: bool, mission_held: bool,
                        applied_fault: Optional[str]) -> Optional[Tuple[str, str]]:
        if estop:
            return "ESTOP", "emergency stop engaged"
        if not mode_idle or mission_held:
            return "MODE_CHANGED", "the agent left IDLE or took a mission"
        # The controller's own word on the DRIVEs first: the most specific reason.
        for ack in run.acks:
            if ack["result"] != "ACCEPTED":
                return "DRIVE_NOT_ACCEPTED", f"DRIVE seq {ack['seq']} was {ack['result']} ({ack['reason']})"
            expected = [run.units, run.units]
            if ack["req"] != expected or ack["applied"] != expected:
                return "ACK_MISMATCH", (f"DRIVE seq {ack['seq']} ACK req={ack['req']} applied={ack['applied']}, "
                                        f"sent {expected}")
        if run.first_drive_at is not None and run.first_ack_at is None \
                and now - run.first_drive_at > self.cfg.ack_timeout_s:
            return "NO_ACK", f"no ACK for the DRIVE within {self.cfg.ack_timeout_s:g}s"
        tel = self._fresh_telemetry(controller)
        if tel is not None and run.first_ack_at is not None:
            for side, value in (("left", tel.left_applied), ("right", tel.right_applied)):
                if value < 0 or value > run.units:
                    return "APPLIED_UNEXPECTED", f"TELEMETRY {side}_applied={value}, sent {run.units}"
            if not run.applied_seen and now - run.first_ack_at > self.cfg.applied_grace_s:
                return "APPLIED_ZERO", (f"TELEMETRY still reports applied {tel.left_applied}/{tel.right_applied} "
                                        f"{self.cfg.applied_grace_s:g}s after the DRIVE was ACCEPTED")
        if not motion_ready:
            return "CONTROLLER_NOT_READY", "the ESP32 link can no longer carry motion (stale, fault or drive unavailable)"
        if interrupted:
            return "LINK_INTERRUPTED", "the ESP32 link was lost and re-proven during the pulse"
        if gated.is_stop and now < run.deadline:
            return "SAFETY_GATE", f"the safety gate stopped the pulse: {gated.reason}"
        if applied_fault:
            return "APPLIED_MOTION_FAULT", applied_fault
        return None

    def _confirm_stop(self, run: _Run, controller: Any, now: float) -> Optional[BenchOutcome]:
        sent = getattr(controller, "last_motion_sent", None)
        tel = self._fresh_telemetry(controller)
        age = getattr(controller, "telemetry_age_s", None)
        confirmed = (
            sent is not None and sent.cmd == "STOP" and sent.age_s is not None
            and tel is not None and age is not None and age < sent.age_s   # TELEMETRY received after the STOP
            and (tel.left_cmd, tel.right_cmd, tel.left_applied, tel.right_applied) == (0, 0, 0, 0)
        )
        if confirmed:
            if sent is not None and sent.seq is not None:
                run.stop_seq = sent.seq
            outcome = BenchOutcome.COMPLETED if run.stop_code == "COMPLETED" else BenchOutcome.ABORTED
        elif run.stopping_since is not None and now - run.stopping_since > self.cfg.stop_confirm_s:
            outcome = BenchOutcome.STOP_UNCONFIRMED
        else:
            return None
        self._finish(run, outcome, now)
        return outcome

    def _finish(self, run: _Run, outcome: BenchOutcome, now: float) -> None:
        self.last_result = {
            "outcome": outcome.value,
            "stop_code": run.stop_code,
            "stop_reason": run.stop_reason,
            "source": run.source,
            "started_at": round(run.started_wall, 3),
            "duration_s": run.duration_s,
            "ran_s": round((run.stopping_since or now) - run.started_at, 2),
            "sent_units": [run.units, run.units],
            "drive_commands": len(run.drive_seqs),
            "drive_acks": run.acks[:20],
            "max_telemetry_applied": list(run.max_applied),
            "stop_confirmed": outcome is not BenchOutcome.STOP_UNCONFIRMED,
            "stop_seq": run.stop_seq,
            "evidence": ("controller-reported only: applied output is what the ESP32 says it drove, not proof "
                         "a wheel turned -- the operator's observation is the physical evidence"),
        }
        self._phase = BenchPhase.IDLE
        self._run = None
        self._cooldown_until = now + COOLDOWN_S
        log_event(logger, "bench.finished", level=logging.WARNING if outcome is BenchOutcome.COMPLETED else logging.ERROR,
                  outcome=outcome.value, stop_code=run.stop_code, drive_acks=len(run.acks),
                  max_applied=list(run.max_applied))

    def _fresh_telemetry(self, controller: Any) -> Any:
        tel = getattr(controller, "telemetry", None)
        age = getattr(controller, "telemetry_age_s", None)
        if tel is None or age is None or age > self.cfg.stale_after_s:
            return None
        return tel
