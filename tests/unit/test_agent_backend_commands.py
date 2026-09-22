"""Agent behaviour for the mission changes a backend command can request.

The executor is tested against a fake agent in `test_commands.py`; here the
real `RobotAgent` is driven through the same four operations, so that what a
backend STOP/PAUSE/RETURN/RESUME actually does to a running robot is pinned
down -- in particular that PAUSE stops the wheels on the next tick and that
RETURN refuses rather than inventing a destination.
"""

import logging
import unittest

from robotx.application.agent import RobotAgent
from robotx.communication.commands import CommandExecutor
from robotx.communication.protocol import CommandStatus, CommandType, InboundCommand
from robotx.hardware.gps import GpsReading, GPSStatus
from robotx.control.motion import MotionCommand
from robotx.state.robot_state import MissionRefused, OperatingMode

from tests.unit.test_agent import HERE, NORTH, FakeGPS, FakePerception, headless_settings


def setUpModule():
    # Refusal paths are exercised on purpose; their warnings are not the
    # subject under test.
    logging.getLogger("robotx").setLevel(logging.CRITICAL)


def command(type_, command_id="c1"):
    return InboundCommand(
        command_id=command_id, type=type_, issued_at=None, received_at=0.0, raw={}
    )


class AgentCommandTestCase(unittest.TestCase):
    def make_agent(self, **overrides):
        agent = RobotAgent(headless_settings(**overrides))
        agent.gps = FakeGPS()
        agent.perception = FakePerception()
        return agent

    def running_agent(self, **overrides):
        """An agent mid-mission with a clear path and a valid fix."""

        agent = self.make_agent(**overrides)
        agent.gps.set_fix(*HERE, speed=1.0, track=0.0)
        agent.perception.set_clear()
        agent.start_mission([NORTH])
        agent.tick()
        return agent


class TestPause(AgentCommandTestCase):
    def test_pause_stops_motion_on_the_next_tick(self):
        agent = self.running_agent()
        self.assertIs(agent.state.snapshot().motion_intent.command, MotionCommand.FORWARD)

        agent.pause_mission("test")
        snapshot = agent.tick()

        self.assertIs(snapshot.mode, OperatingMode.PAUSED)
        self.assertTrue(snapshot.motion_intent.is_stop)

    def test_pause_keeps_driving_impossible_even_with_a_clear_path(self):
        # The decision layer must not be able to talk the robot back into
        # moving while paused.
        agent = self.running_agent()
        agent.pause_mission("test")
        for _ in range(5):
            snapshot = agent.tick()
            self.assertTrue(snapshot.motion_intent.is_stop)

    def test_pause_retains_the_route(self):
        agent = self.running_agent()
        agent.pause_mission("test")
        self.assertGreater(agent.navigator.planner.route_length(), 0)


class TestResume(AgentCommandTestCase):
    def test_resume_returns_to_auto_and_drives_again(self):
        agent = self.running_agent()
        agent.pause_mission("test")
        agent.tick()

        agent.resume_mission("test")
        snapshot = agent.tick()

        self.assertIs(snapshot.mode, OperatingMode.AUTO)
        self.assertIs(snapshot.motion_intent.command, MotionCommand.FORWARD)

    def test_resume_without_a_route_is_refused(self):
        agent = self.make_agent()
        with self.assertRaises(MissionRefused):
            agent.resume_mission("test")

    def test_resume_after_stop_is_refused(self):
        # STOP cleared the route; there is nothing left to resume.
        agent = self.running_agent()
        agent.stop_mission("test")
        with self.assertRaises(MissionRefused):
            agent.resume_mission("test")

    def test_resume_does_not_bypass_perception_safety(self):
        # A backend can put the robot back into AUTO. It cannot make the robot
        # drive into something.
        agent = self.running_agent()
        agent.pause_mission("test")
        agent.perception.set_detection(label="person", area=5000)

        agent.resume_mission("test")
        snapshot = agent.tick()

        self.assertIs(snapshot.mode, OperatingMode.AUTO)
        self.assertTrue(snapshot.motion_intent.is_stop)
        self.assertIn("person", snapshot.motion_intent.reason)

    def test_resume_does_not_bypass_position_validity(self):
        agent = self.running_agent()
        agent.pause_mission("test")
        agent.gps.reading = GpsReading(status=GPSStatus.NO_FIX)

        agent.resume_mission("test")
        snapshot = agent.tick()
        self.assertTrue(snapshot.motion_intent.is_stop)


class TestReturn(AgentCommandTestCase):
    def test_return_uses_the_configured_home(self):
        agent = self.make_agent(ROBOTX_HOME_LAT="51.4", ROBOTX_HOME_LON="-0.2")
        agent.return_to_base("test")

        self.assertIs(agent.state.mode, OperatingMode.AUTO)
        self.assertEqual(agent.navigator.planner.destination(), (51.4, -0.2))

    def test_return_falls_back_to_the_mission_origin(self):
        agent = self.make_agent()
        agent.gps.set_fix(*HERE)
        agent.tick()  # record a position before the mission starts
        agent.perception.set_clear()
        agent.start_mission([NORTH])

        agent.return_to_base("test")
        destination = agent.navigator.planner.destination()
        self.assertAlmostEqual(destination[0], HERE[0], places=4)

    def test_return_is_refused_when_no_home_is_known(self):
        # No configured home, and the mission began without a GPS fix. The
        # only alternative would be driving somewhere nobody chose.
        agent = self.make_agent()
        agent.perception.set_clear()
        agent.start_mission([NORTH])

        with self.assertRaises(MissionRefused) as caught:
            agent.return_to_base("test")
        self.assertIn("home", str(caught.exception))

    def test_refused_return_leaves_the_route_untouched(self):
        agent = self.make_agent()
        agent.perception.set_clear()
        agent.start_mission([NORTH])

        with self.assertRaises(MissionRefused):
            agent.return_to_base("test")
        self.assertEqual(agent.navigator.planner.destination(), NORTH)

    def test_mission_origin_is_not_recorded_without_a_fix(self):
        agent = self.make_agent()
        agent.start_mission([NORTH])
        self.assertIsNone(agent._mission_origin)


class TestExecutorAgainstTheRealAgent(AgentCommandTestCase):
    """The executor and the real agent, wired exactly as the link wires them."""

    def test_stop_command_halts_the_real_agent(self):
        agent = self.running_agent()
        outcome = CommandExecutor(agent).execute(command(CommandType.STOP))

        self.assertIs(outcome.status, CommandStatus.ACK)
        self.assertIs(agent.state.mode, OperatingMode.STOPPED)
        self.assertTrue(agent.tick().motion_intent.is_stop)

    def test_pause_then_resume_round_trip(self):
        agent = self.running_agent()
        executor = CommandExecutor(agent)

        paused = executor.execute(command(CommandType.PAUSE, "c1"))
        self.assertIs(paused.status, CommandStatus.ACK)
        self.assertIs(agent.tick().mode, OperatingMode.PAUSED)

        resumed = executor.execute(command(CommandType.RESUME, "c2"))
        self.assertIs(resumed.status, CommandStatus.ACK)
        self.assertIs(agent.tick().mode, OperatingMode.AUTO)

    def test_return_without_a_home_becomes_a_failed_ack(self):
        agent = self.make_agent()
        agent.perception.set_clear()
        agent.start_mission([NORTH])

        outcome = CommandExecutor(agent).execute(command(CommandType.RETURN))
        self.assertIs(outcome.status, CommandStatus.FAILED)
        self.assertIn("home", outcome.reason)

    def test_resume_from_stopped_becomes_a_failed_ack(self):
        agent = self.running_agent()
        CommandExecutor(agent).execute(command(CommandType.STOP, "c1"))

        outcome = CommandExecutor(agent).execute(command(CommandType.RESUME, "c2"))
        self.assertIs(outcome.status, CommandStatus.FAILED)


class TestAgentWithoutABackend(AgentCommandTestCase):
    def test_no_backend_link_is_constructed_by_default(self):
        agent = self.make_agent()
        self.assertIsNone(agent.backend)

    def test_socketio_is_not_imported_when_the_link_is_disabled(self):
        # The standalone agent keeps the runtime footprint it had before this
        # integration existed.
        import subprocess
        import sys

        script = (
            "import sys;"
            "from robotx.application.agent import RobotAgent;"
            "from robotx.config.settings import Settings;"
            "RobotAgent(Settings.from_env({'ROBOTX_SOCKET_ENABLED': '0'}));"
            "print('socketio' in sys.modules)"
        )
        output = subprocess.run(
            [sys.executable, "-c", script], capture_output=True, text=True, cwd="."
        )
        self.assertEqual(output.stdout.strip(), "False", output.stderr)


if __name__ == "__main__":
    unittest.main()
