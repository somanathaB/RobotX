"""Command execution: mode transitions, refusals, and idempotency.

The executor is the only thing a remote party can make this robot do, so the
tests cover the refusals as carefully as the successes -- particularly that a
duplicate never executes twice and that RETURN will not invent a destination.
"""

import time
import unittest

from robotx.communication.commands import (
    CommandExecutor,
    CommandOutcome,
    rejection_outcome,
)
from robotx.communication.protocol import (
    CommandRejection,
    CommandStatus,
    CommandType,
    InboundCommand,
    RejectionReason,
)
from robotx.state.robot_state import MissionRefused, OperatingMode


class FakeAgent:
    """A mission target that records calls instead of driving a robot."""

    def __init__(self, mode=OperatingMode.AUTO, *, route=True, home=True):
        self._mode = mode
        self._route = route
        self._home = home
        self.calls = []

    @property
    def mode(self):
        return self._mode

    def stop_mission(self, reason=""):
        self.calls.append(("stop", reason))
        self._route = False
        self._mode = OperatingMode.STOPPED

    def pause_mission(self, reason=""):
        self.calls.append(("pause", reason))
        self._mode = OperatingMode.PAUSED

    def resume_mission(self, reason=""):
        self.calls.append(("resume", reason))
        if not self._route:
            raise MissionRefused("no route is loaded; nothing to resume")
        self._mode = OperatingMode.AUTO

    def return_to_base(self, reason=""):
        self.calls.append(("return", reason))
        if not self._home:
            raise MissionRefused("no home position")
        self._mode = OperatingMode.AUTO


def command(type_, command_id="c1", issued_at=None):
    return InboundCommand(
        command_id=command_id,
        type=type_,
        issued_at=issued_at,
        received_at=time.time(),
        raw={},
    )


class TestStop(unittest.TestCase):
    def test_stop_acks_and_halts(self):
        agent = FakeAgent()
        outcome = CommandExecutor(agent).execute(command(CommandType.STOP))
        self.assertIs(outcome.status, CommandStatus.ACK)
        self.assertEqual(agent.calls[0][0], "stop")
        self.assertIs(agent.mode, OperatingMode.STOPPED)

    def test_stop_works_from_every_mode(self):
        # A stop that only works when the Pi agrees it is needed is not a stop.
        for mode in OperatingMode:
            agent = FakeAgent(mode=mode)
            outcome = CommandExecutor(agent).execute(command(CommandType.STOP))
            self.assertIs(outcome.status, CommandStatus.ACK, mode)

    def test_stop_carries_the_command_id_into_the_reason(self):
        agent = FakeAgent()
        CommandExecutor(agent).execute(command(CommandType.STOP, command_id="abc-123"))
        self.assertIn("abc-123", agent.calls[0][1])


class TestPause(unittest.TestCase):
    def test_pause_suspends_an_active_mission(self):
        agent = FakeAgent(mode=OperatingMode.AUTO)
        outcome = CommandExecutor(agent).execute(command(CommandType.PAUSE))
        self.assertIs(outcome.status, CommandStatus.ACK)
        self.assertIs(agent.mode, OperatingMode.PAUSED)

    def test_pausing_an_already_paused_robot_is_a_no_op_ack(self):
        agent = FakeAgent(mode=OperatingMode.PAUSED)
        outcome = CommandExecutor(agent).execute(command(CommandType.PAUSE))
        self.assertIs(outcome.status, CommandStatus.ACK)
        self.assertEqual(agent.calls, [])

    def test_pausing_an_idle_robot_is_honoured(self):
        agent = FakeAgent(mode=OperatingMode.IDLE)
        outcome = CommandExecutor(agent).execute(command(CommandType.PAUSE))
        self.assertIs(outcome.status, CommandStatus.ACK)
        self.assertIs(agent.mode, OperatingMode.PAUSED)


class TestResume(unittest.TestCase):
    def test_resume_from_paused_succeeds(self):
        agent = FakeAgent(mode=OperatingMode.PAUSED)
        outcome = CommandExecutor(agent).execute(command(CommandType.RESUME))
        self.assertIs(outcome.status, CommandStatus.ACK)
        self.assertIs(agent.mode, OperatingMode.AUTO)

    def test_resume_from_stopped_fails_rather_than_inventing_a_route(self):
        agent = FakeAgent(mode=OperatingMode.STOPPED)
        outcome = CommandExecutor(agent).execute(command(CommandType.RESUME))
        self.assertIs(outcome.status, CommandStatus.FAILED)
        self.assertIn("STOPPED", outcome.reason)
        self.assertEqual(agent.calls, [])

    def test_resume_from_error_fails(self):
        # Resuming would discard the fault that caused the error.
        agent = FakeAgent(mode=OperatingMode.ERROR)
        outcome = CommandExecutor(agent).execute(command(CommandType.RESUME))
        self.assertIs(outcome.status, CommandStatus.FAILED)

    def test_resume_while_already_running_is_an_idempotent_ack(self):
        agent = FakeAgent(mode=OperatingMode.AUTO)
        outcome = CommandExecutor(agent).execute(command(CommandType.RESUME))
        self.assertIs(outcome.status, CommandStatus.ACK)
        self.assertEqual(agent.calls, [])

    def test_agent_refusal_becomes_a_failed_with_its_reason(self):
        agent = FakeAgent(mode=OperatingMode.PAUSED, route=False)
        outcome = CommandExecutor(agent).execute(command(CommandType.RESUME))
        self.assertIs(outcome.status, CommandStatus.FAILED)
        self.assertIn("nothing to resume", outcome.reason)


class TestReturn(unittest.TestCase):
    def test_return_acks_when_a_home_is_known(self):
        agent = FakeAgent(home=True)
        outcome = CommandExecutor(agent).execute(command(CommandType.RETURN))
        self.assertIs(outcome.status, CommandStatus.ACK)
        self.assertEqual(agent.calls[0][0], "return")

    def test_return_fails_when_no_home_is_known(self):
        # The alternative -- picking a destination -- would drive the robot
        # somewhere nobody chose.
        agent = FakeAgent(home=False)
        outcome = CommandExecutor(agent).execute(command(CommandType.RETURN))
        self.assertIs(outcome.status, CommandStatus.FAILED)
        self.assertIn("home", outcome.reason)


class TestIdempotency(unittest.TestCase):
    def test_duplicate_command_is_not_executed_twice(self):
        agent = FakeAgent()
        executor = CommandExecutor(agent)
        first = executor.execute(command(CommandType.STOP, command_id="dup"))
        second = executor.execute(command(CommandType.STOP, command_id="dup"))

        self.assertEqual(len(agent.calls), 1)
        self.assertTrue(second.duplicate)
        self.assertFalse(first.duplicate)

    def test_duplicate_reports_the_original_outcome(self):
        agent = FakeAgent(mode=OperatingMode.STOPPED)
        executor = CommandExecutor(agent)
        first = executor.execute(command(CommandType.RESUME, command_id="dup"))
        second = executor.execute(command(CommandType.RESUME, command_id="dup"))

        self.assertIs(first.status, CommandStatus.FAILED)
        self.assertIs(second.status, CommandStatus.FAILED)
        self.assertEqual(first.reason, second.reason)
        self.assertEqual(first.executed_at, second.executed_at)

    def test_distinct_ids_both_execute(self):
        agent = FakeAgent()
        executor = CommandExecutor(agent)
        executor.execute(command(CommandType.PAUSE, command_id="a"))
        executor.execute(command(CommandType.STOP, command_id="b"))
        self.assertEqual([c[0] for c in agent.calls], ["pause", "stop"])

    def test_history_expires_so_a_reissued_command_runs_again(self):
        agent = FakeAgent()
        executor = CommandExecutor(agent, history_ttl_s=10.0)
        now = time.time()
        executor.execute(command(CommandType.STOP, command_id="x"), now=now)
        executor.execute(command(CommandType.STOP, command_id="x"), now=now + 60)
        self.assertEqual(len(agent.calls), 2)

    def test_history_is_bounded(self):
        executor = CommandExecutor(FakeAgent(), history_size=8)
        for index in range(50):
            executor.execute(command(CommandType.PAUSE, command_id=f"c{index}"))
        self.assertLessEqual(executor.history_size, 8)

    def test_oldest_entries_are_evicted_first(self):
        executor = CommandExecutor(FakeAgent(), history_size=3)
        for index in range(5):
            executor.execute(command(CommandType.PAUSE, command_id=f"c{index}"))
        self.assertIsNone(executor.seen("c0"))
        self.assertIsNotNone(executor.seen("c4"))


class TestFailureIsolation(unittest.TestCase):
    def test_unexpected_agent_exception_becomes_failed_not_a_raise(self):
        class Exploding(FakeAgent):
            def stop_mission(self, reason=""):
                raise RuntimeError("bang")

        outcome = CommandExecutor(Exploding()).execute(command(CommandType.STOP))
        self.assertIs(outcome.status, CommandStatus.FAILED)
        self.assertIn("agent error", outcome.reason)

    def test_a_failing_command_does_not_block_the_next_one(self):
        class ExplodingOnce(FakeAgent):
            def __init__(self):
                super().__init__()
                self.exploded = False

            def stop_mission(self, reason=""):
                if not self.exploded:
                    self.exploded = True
                    raise RuntimeError("bang")
                super().stop_mission(reason)

        agent = ExplodingOnce()
        executor = CommandExecutor(agent)
        executor.execute(command(CommandType.STOP, command_id="a"))
        outcome = executor.execute(command(CommandType.STOP, command_id="b"))
        self.assertIs(outcome.status, CommandStatus.ACK)


class TestRejectionOutcome(unittest.TestCase):
    def test_rejection_with_an_id_becomes_a_failed_ack(self):
        rejection = CommandRejection(RejectionReason.UNKNOWN_TYPE, "nope", "c1")
        outcome = rejection_outcome(rejection)
        self.assertIsInstance(outcome, CommandOutcome)
        self.assertIs(outcome.status, CommandStatus.FAILED)
        self.assertIn("UNKNOWN_TYPE", outcome.reason)

    def test_rejection_without_an_id_cannot_be_acknowledged(self):
        rejection = CommandRejection(RejectionReason.MISSING_ID, "no id")
        self.assertIsNone(rejection_outcome(rejection))


class TestSafetyBoundary(unittest.TestCase):
    def test_command_target_exposes_no_motion_control(self):
        # The executor can change mission state and nothing else. If this ever
        # grows a speed or steering method, the backend has gained a path to
        # the motors that bypasses perception.
        from robotx.communication.commands import CommandTarget

        allowed = {"mode", "stop_mission", "pause_mission", "resume_mission", "return_to_base"}
        public = {name for name in vars(CommandTarget) if not name.startswith("_")}
        self.assertEqual(public, allowed)

    def test_communication_package_imports_no_hardware(self):
        # Checked against the actual import statements rather than the file
        # text, so prose mentioning motors does not trip it and an import
        # hidden inside a function still does.
        import ast
        import pathlib

        import robotx.communication as package

        forbidden = ("RPi", "RPi.GPIO", "gpiozero", "serial", "picamera2", "cv2")
        package_dir = pathlib.Path(package.__file__).parent

        for path in sorted(package_dir.glob("*.py")):
            tree = ast.parse(path.read_text())
            imported = set()
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    imported.update(alias.name for alias in node.names)
                elif isinstance(node, ast.ImportFrom) and node.module:
                    imported.add(node.module)

            for name in imported:
                root = name.split(".")[0]
                self.assertNotIn(root, forbidden, f"{path.name} imports {name}")
                self.assertFalse(
                    name.startswith("robotx.hardware"),
                    f"{path.name} imports hardware module {name}",
                )
                self.assertFalse(
                    name.startswith("robotx.perception"),
                    f"{path.name} imports perception internals: {name}",
                )


if __name__ == "__main__":
    unittest.main()
