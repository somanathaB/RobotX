"""An obstacle that appears between an agent tick and the link writing its DRIVE.

The agent decides on its tick; the ESP32 link writes the decision on its next
poll. An obstacle can appear in between, after a clear-scene tick submitted a
DRIVE and before that DRIVE is on the wire. These tests fix the order of tick,
perception change and link poll by hand -- each outcome the race can have --
and pin the safety invariant for every one:

  - once a tick has the obstacle in view, no DRIVE is written for as long as
    it stays: an unsent clear-scene DRIVE is replaced by that tick's STOP;
  - the most that can reach the wire after the obstacle appeared is the one
    DRIVE already submitted by the clear-scene tick, and a STOP follows it on
    the very next tick;
  - an unsent DRIVE older than the link's `max_command_age_s` is never written.

SIMULATED: the real `RobotAgent`, `SafetyGate` and `Esp32Link` over a fake port
answered by `FirmwareModel`, on one fake clock. A pass shows the Pi's ordering
of decisions and writes; it says nothing about a physical wheel or camera.
"""

from __future__ import annotations

import logging
import unittest

from tests.unit.test_obstacle_recovery import TICK_S, Rig  # imports the agent first: robotx's import order
from robotx.state.robot_state import OperatingMode


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


def driving_rig() -> Rig:
    rig = Rig()
    rig.start()
    return rig


def clear_scene_tick(rig: Rig) -> None:
    """One tick on a clear scene whose DRIVE is submitted but not yet written."""

    rig.clock.advance(TICK_S)
    rig.esp32()                       # firmware answers; the link writes what was pending
    snap = rig.agent.tick()
    assert snap.mode is OperatingMode.AUTO and not snap.motion_intent.is_stop, snap.motion_intent


def obstacle(rig: Rig) -> None:
    rig.agent.perception.set_detection(area=40000, cx=320)


def motion(rig: Rig, since: int):
    return [c["cmd"] for c in rig.port.commands()[since:] if c["cmd"] in ("DRIVE", "STOP")]


class TestObstacleBetweenTickAndWrite(unittest.TestCase):
    def assert_no_drive_while_in_view(self, rig: Rig, seconds: float = 3.0) -> None:
        mark = rig.mark()
        rig.run(seconds)
        self.assertEqual(rig.drives(mark), [], "DRIVE written while the obstacle stayed in view")
        self.assertIn("STOP", motion(rig, mark), "the link stopped streaming STOP")

    def test_an_unsent_drive_is_replaced_by_the_stop_of_the_tick_that_sees_the_obstacle(self):
        rig = driving_rig()
        clear_scene_tick(rig)
        mark = rig.mark()
        obstacle(rig)
        snap = rig.agent.tick()           # sees it before the link ever polled
        self.assertTrue(snap.motion_intent.is_stop, snap.motion_intent)
        rig.link.poll_once()
        self.assertEqual(motion(rig, mark), ["STOP"], "the clear-scene DRIVE reached the wire after the STOP decision")
        self.assert_no_drive_while_in_view(rig)

    def test_a_drive_already_on_its_way_is_the_only_one_and_a_stop_follows_on_the_next_tick(self):
        rig = driving_rig()
        clear_scene_tick(rig)
        mark = rig.mark()
        obstacle(rig)
        rig.link.poll_once()              # the in-flight DRIVE, decided on the clear scene, goes out
        self.assertEqual(motion(rig, mark), ["DRIVE"])
        rig.step()                        # the next tick sees the obstacle
        self.assertEqual(motion(rig, mark), ["DRIVE", "STOP"], "no STOP on the first tick that saw the obstacle")
        self.assert_no_drive_while_in_view(rig)

    def test_an_unsent_drive_older_than_the_command_age_limit_is_never_written(self):
        rig = driving_rig()
        clear_scene_tick(rig)
        mark = rig.mark()
        dropped = rig.link.status().controller.counters.stale_commands_dropped
        rig.clock.advance(rig.link.cfg.max_command_age_s + TICK_S)
        rig.link.poll_once()
        self.assertEqual(rig.drives(mark), [], "a DRIVE older than max_command_age_s was written")
        self.assertEqual(rig.link.status().controller.counters.stale_commands_dropped, dropped + 1)


if __name__ == "__main__":
    unittest.main()
