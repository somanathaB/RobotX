"""Backend commands -> agent intents, with the safety rules that go with it.

This is the one path by which a remote party influences this robot, so it is
written defensively:

- Only the four commands in the backend enum are recognized. Anything else is
  rejected with a reason, never guessed at.
- A command is applied by calling the agent's *own* mission methods. It does
  not reach past them into navigation, the decision layer, motors or GPIO --
  there is nothing importable from here that could. A backend `RESUME` returns
  the agent to AUTO; whether the robot then actually moves is still decided by
  perception, position validity and (once it exists) the ESP32.
- Execution is idempotent per `commandId`. A backend retry, a duplicate
  delivery or a replay after reconnect re-reports the original outcome instead
  of applying the command twice.
- The acknowledgement is emitted *after* the intent is applied, so an `ACK` is
  evidence of an effect rather than of a delivered packet.

Why `PAUSE` and `STOP` differ
-----------------------------
`STOP` clears the route: it is the operator saying this run is over. `PAUSE`
keeps it and suspends motion, so `RESUME` has something to go back to.
Collapsing the two would make `RESUME` meaningless, since there would be no
retained route for it to resume -- and the backend enum clearly intends the
pair to work together.
"""

from __future__ import annotations

import logging
import time
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any, Dict, Optional, Protocol

from robotx.communication.protocol import (
    CommandRejection,
    CommandStatus,
    CommandType,
    InboundCommand,
)
from robotx.config.logging_setup import log_event
from robotx.state.robot_state import MissionRefused, OperatingMode


logger = logging.getLogger(__name__)

# How many executed command ids to remember for duplicate suppression. Bounded
# so a hostile or looping backend cannot grow this without limit.
DEFAULT_HISTORY_SIZE = 256
# Beyond this, a remembered outcome is forgotten and the command would run
# again. Long enough to cover a reconnect storm, short enough that a genuinely
# re-issued command hours later is honoured.
DEFAULT_HISTORY_TTL_S = 900.0


class CommandTarget(Protocol):
    """What the executor needs from the agent. Deliberately four methods wide.

    Narrow on purpose: the communication layer can request a mission state
    change and read the current mode, and there is no method here through which
    it could set a speed, steer, or touch hardware.
    """

    @property
    def mode(self) -> OperatingMode: ...

    def stop_mission(self, reason: str = ...) -> None: ...

    def pause_mission(self, reason: str = ...) -> None: ...

    def resume_mission(self, reason: str = ...) -> None: ...

    def return_to_base(self, reason: str = ...) -> None: ...


@dataclass(frozen=True)
class CommandOutcome:
    """The result of handling one command, ready to be acknowledged."""

    command_id: str
    status: CommandStatus
    reason: str
    executed_at: float
    duplicate: bool = False

    @property
    def succeeded(self) -> bool:
        return self.status is CommandStatus.ACK

    def to_dict(self) -> Dict[str, Any]:
        return {
            "command_id": self.command_id,
            "status": self.status.value,
            "reason": self.reason,
            "executed_at": self.executed_at,
            "duplicate": self.duplicate,
        }


class CommandExecutor:
    """Applies validated commands to the agent, once each."""

    def __init__(
        self,
        target: CommandTarget,
        *,
        history_size: int = DEFAULT_HISTORY_SIZE,
        history_ttl_s: float = DEFAULT_HISTORY_TTL_S,
    ) -> None:
        self._target = target
        self._history_size = max(1, int(history_size))
        self._history_ttl_s = float(history_ttl_s)
        self._history: "OrderedDict[str, CommandOutcome]" = OrderedDict()

    # --- duplicate suppression ------------------------------------------------

    def seen(self, command_id: str, *, now: Optional[float] = None) -> Optional[CommandOutcome]:
        """The recorded outcome for this id, if it is still remembered."""

        now = time.time() if now is None else now
        outcome = self._history.get(command_id)
        if outcome is None:
            return None
        if now - outcome.executed_at > self._history_ttl_s:
            self._history.pop(command_id, None)
            return None
        return outcome

    def _remember(self, outcome: CommandOutcome) -> None:
        self._history[outcome.command_id] = outcome
        self._history.move_to_end(outcome.command_id)
        while len(self._history) > self._history_size:
            self._history.popitem(last=False)

    @property
    def history_size(self) -> int:
        return len(self._history)

    # --- execution ------------------------------------------------------------

    def execute(self, command: InboundCommand, *, now: Optional[float] = None) -> CommandOutcome:
        """Apply one command and return the outcome to acknowledge.

        Never raises. A command handler that propagates an exception into the
        socket callback would stop the robot from processing the *next*
        command, which could be the STOP that matters.
        """

        now = time.time() if now is None else now

        previous = self.seen(command.command_id, now=now)
        if previous is not None:
            log_event(
                logger,
                "command.duplicate",
                "re-reporting the original outcome without re-executing",
                command_id=command.command_id,
                type=command.type.value,
                original_status=previous.status.value,
            )
            # Same status and reason as the first time, so the backend cannot
            # see a command flip outcome just because it was redelivered.
            return CommandOutcome(
                command_id=previous.command_id,
                status=previous.status,
                reason=previous.reason,
                executed_at=previous.executed_at,
                duplicate=True,
            )

        try:
            reason = self._apply(command)
            outcome = CommandOutcome(command.command_id, CommandStatus.ACK, reason, now)
            log_event(
                logger,
                "command.applied",
                command_id=command.command_id,
                type=command.type.value,
                mode=self._target.mode.value,
            )
        except MissionRefused as e:
            outcome = CommandOutcome(command.command_id, CommandStatus.FAILED, str(e), now)
            log_event(
                logger,
                "command.refused",
                str(e),
                level=logging.WARNING,
                command_id=command.command_id,
                type=command.type.value,
            )
        except Exception as e:  # noqa: BLE001 - a bug here must not kill the link
            outcome = CommandOutcome(
                command.command_id, CommandStatus.FAILED, f"agent error: {e!r}"[:200], now
            )
            log_event(
                logger,
                "command.failed",
                "unhandled error applying command",
                level=logging.ERROR,
                exc_info=True,
                command_id=command.command_id,
                type=command.type.value,
            )

        self._remember(outcome)
        return outcome

    def _apply(self, command: InboundCommand) -> str:
        """Turn one command into an agent mission call. Returns the ack reason."""

        mode = self._target.mode

        if command.type is CommandType.STOP:
            # Unconditional: a stop must work from any state, including ERROR,
            # and must not depend on the Pi agreeing that it was needed.
            self._target.stop_mission(f"backend STOP ({command.command_id})")
            return "mission stopped and route cleared"

        if command.type is CommandType.PAUSE:
            if mode is OperatingMode.PAUSED:
                return "already paused"
            if not mode.has_route:
                # Still honoured: pausing an idle robot is harmless and keeps
                # the backend's view of the mode correct.
                self._target.pause_mission(f"backend PAUSE ({command.command_id})")
                return "paused with no active mission"
            self._target.pause_mission(f"backend PAUSE ({command.command_id})")
            return "mission paused, route retained"

        if command.type is CommandType.RESUME:
            if mode is OperatingMode.AUTO:
                return "already running"
            if mode is not OperatingMode.PAUSED:
                # Resuming from STOPPED would have to invent a route, and
                # resuming from ERROR would discard the fault that caused it.
                raise MissionRefused(
                    f"cannot RESUME from {mode.value}; only a PAUSED mission can be resumed"
                )
            self._target.resume_mission(f"backend RESUME ({command.command_id})")
            return "mission resumed"

        if command.type is CommandType.RETURN:
            self._target.return_to_base(f"backend RETURN ({command.command_id})")
            return "returning to base"

        # Unreachable: parse_command only produces the four values above.
        raise MissionRefused(f"unhandled command type {command.type.value}")


def rejection_outcome(rejection: CommandRejection, *, now: Optional[float] = None) -> Optional[CommandOutcome]:
    """A `FAILED` outcome for a rejected command, when one can be addressed.

    Returns None when the rejection has no `commandId`: there is no backend row
    to fail, so emitting one would be shouting at nobody. Those are logged
    locally by the caller instead.
    """

    if not rejection.is_ackable or rejection.command_id is None:
        return None
    return CommandOutcome(
        command_id=rejection.command_id,
        status=CommandStatus.FAILED,
        reason=f"{rejection.reason.value}: {rejection.detail}"[:200],
        executed_at=time.time() if now is None else now,
    )
