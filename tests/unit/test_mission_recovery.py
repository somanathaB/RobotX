"""Interrupted-mission recovery: e-stop and ESP32 reboot during a mission.

The recovery transition under test::

    AUTO/PAUSED --emergency stop / ESP32 reboot--> STOPPED, mission held + interrupted,
                                                   no motion, lease NOT renewed
    STOPPED --clear--> PAUSED (never AUTO), lease renewed, interruption still recorded
    PAUSED --explicit RESUME--> AUTO (interruption cleared); every gate still applies
    STOPPED/PAUSED --STOP--> mission ABORTED; then IDLE is allowed

While the latch is on, RESUME, RETURN and IDLE are refused and PAUSE leaves the
mode at STOPPED.

What is real and what is simulated
----------------------------------
- REAL: `RobotAgent`, `MissionManager`, the safety gate, `CommandExecutor`,
  `Esp32Link`, `BackendLink` (OFFER verification, HEARTBEAT), the FastAPI routes.
- SIMULATED: the ESP32 (`FirmwareModel` over a fake port), GPS, perception, the
  backend socket (`FakeSio`) and the signed OFFER (test-only key).
"""

from __future__ import annotations

import asyncio
import logging
import time
import unittest

from robotx.application.agent import RobotAgent  # noqa: F401  first: robot_state import cycle (pre-existing)
from robotx.communication.commands import CommandExecutor
from robotx.communication.engine import OFFER
from robotx.communication.protocol import CommandStatus, CommandType, InboundCommand
from robotx.mission.mission import Mission, MissionStatus
from robotx.state.robot_state import Esp32LinkStatus, MissionRefused, OperatingMode
from tests.fixtures import engine as efx
from tests.fixtures.task_assign import START, leg, offset
from tests.unit.test_agent import HERE
from tests.unit.test_backend_link import FakeSio, connect_and_auth
from tests.unit.test_engine_offer import engine_link
from tests.unit.test_injection_endpoint import acall, ready_rig
from tests.unit.test_obstacle_recovery import Rig


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


PICKUP = offset(HERE, north_m=60.0)
DROP = offset(HERE, north_m=120.0)


def mission(task_id="T-RECOVERY-1"):
    return Mission.create(task_id=task_id, pickup=PICKUP, drop=DROP,
                          path_to_pickup=leg(HERE, PICKUP, points=5),
                          path_to_drop=leg(PICKUP, DROP, points=5), timestamp=time.time())


def command(kind, n=[0]):
    n[0] += 1
    return InboundCommand(command_id=f"cmd-{n[0]}", type=kind, issued_at=time.time(),
                          received_at=time.time(), raw={})


class RecoveryCase(unittest.TestCase):
    def setUp(self):
        self.rig = Rig()
        self.agent = self.rig.agent
        self.agent.assign_mission(mission(), custody_required=True)
        snaps = self.rig.run(1.0)
        self.assertIs(snaps[-1].mode, OperatingMode.AUTO)
        self.assertTrue(self.rig.drives(), "the rig never drove")

    def held(self):
        return self.agent.state.snapshot().mission


class TestEmergencyStopMidMission(RecoveryCase):
    def test_the_mission_is_held_interrupted_and_nothing_moves(self):
        self.agent.emergency_stop("operator")
        mark = self.rig.mark()
        snaps = self.rig.run(3.0)
        self.assertTrue(all(s.mode is OperatingMode.STOPPED for s in snaps))
        self.assertEqual(self.rig.drives(mark), [])
        self.assertEqual(self.rig.port.command_names()[-1], "STOP")
        held = self.held()
        self.assertTrue(held.is_active)
        self.assertIs(held.status, MissionStatus.TO_PICKUP)
        self.assertEqual(held.interrupted_by, "emergency stop: operator")
        self.assertIsNotNone(held.interrupted_at)
        self.assertEqual(self.agent.readiness_blockers()[0], "ESTOP_LATCHED")
        self.assertFalse(self.agent.mission_lease_renewable())

    def test_resume_return_and_idle_are_refused_and_pause_keeps_stopped(self):
        self.agent.emergency_stop("operator")
        for refused in (self.agent.resume_mission, self.agent.return_to_base, self.agent.resume_idle):
            with self.assertRaises(MissionRefused, msg=refused.__name__):
                refused()
            self.assertIs(self.agent.mode, OperatingMode.STOPPED)
        self.agent.pause_mission("backend PAUSE")
        self.assertIs(self.agent.mode, OperatingMode.STOPPED)

    def test_backend_commands_through_the_real_executor(self):
        executor = CommandExecutor(self.agent)
        self.agent.emergency_stop("operator")
        self.assertIs(executor.execute(command(CommandType.RESUME)).status, CommandStatus.FAILED)
        self.assertIs(executor.execute(command(CommandType.PAUSE)).status, CommandStatus.ACK)
        self.assertIs(self.agent.mode, OperatingMode.STOPPED)          # PAUSE did not demote it
        self.assertIs(executor.execute(command(CommandType.RESUME)).status, CommandStatus.FAILED)
        self.assertIs(self.agent.mode, OperatingMode.STOPPED)


class TestClearingTheStop(RecoveryCase):
    def test_clearing_pauses_the_mission_and_does_not_resume_motion(self):
        self.agent.emergency_stop("operator")
        self.rig.run(0.4)
        self.assertTrue(self.agent.clear_emergency_stop("operator"))
        mark = self.rig.mark()
        snaps = self.rig.run(5.0)
        self.assertTrue(all(s.mode is OperatingMode.PAUSED for s in snaps))
        self.assertTrue(snaps[-1].last_error.startswith("recovery: "), snaps[-1].last_error)
        self.assertEqual(self.rig.drives(mark), [], "DRIVE after clearing the e-stop without a RESUME")
        self.assertTrue(self.held().interrupted)                        # still on record
        self.assertTrue(self.agent.mission_lease_renewable())
        self.assertNotIn("ESTOP_LATCHED", self.agent.readiness_blockers())

    def test_an_explicit_resume_drives_and_clears_the_interruption(self):
        self.agent.emergency_stop("operator")
        self.agent.clear_emergency_stop("operator")
        self.rig.run(0.4)
        mark = self.rig.mark()
        self.agent.resume_mission("operator RESUME")
        snaps = self.rig.run(1.0)
        self.assertIs(snaps[-1].mode, OperatingMode.AUTO)
        self.assertTrue(self.rig.drives(mark))
        self.assertIsNone(self.held().interrupted_by)
        self.assertIsNone(snaps[-1].last_error)

    def test_stop_abandons_and_then_idle_is_allowed(self):
        self.agent.emergency_stop("operator")
        self.agent.clear_emergency_stop("operator")
        with self.assertRaises(MissionRefused):
            self.agent.resume_idle()
        self.agent.stop_mission("operator STOP")
        self.assertIs(self.held().status, MissionStatus.ABORTED)
        self.agent.resume_idle()
        self.assertIs(self.agent.mode, OperatingMode.IDLE)

    def test_the_idle_route_answers_409_while_a_mission_is_held(self):
        self.agent.emergency_stop("operator")
        self.agent.clear_emergency_stop("operator")
        status, body = asyncio.run(acall(self.agent, None, "POST", "/mission/idle", token=None))
        self.assertEqual(status, 409)
        self.assertIn("still held", body["detail"])
        self.assertIs(self.agent.mode, OperatingMode.PAUSED)

    def test_clearing_with_no_mission_is_unchanged(self):
        rig = Rig()
        rig.agent.emergency_stop("bench")
        rig.agent.clear_emergency_stop("bench")
        self.assertIs(rig.agent.mode, OperatingMode.STOPPED)


class TestEsp32RebootMidMission(RecoveryCase):
    def reboot(self):
        self.rig.uptime = 400                         # uptime goes backwards
        return self.rig.run(1.0)

    def test_a_reboot_latches_interrupts_and_recovery_needs_clear_reproof_and_resume(self):
        mark = self.rig.mark()
        snaps = self.reboot()
        self.assertTrue(self.agent.emergency_stopped)
        self.assertIs(snaps[-1].mode, OperatingMode.STOPPED)
        self.assertIn("ESP32 rebooted", self.held().interrupted_by)
        self.assertTrue(self.rig.link.status().controller.reboot_latched)
        self.assertFalse(self.agent.mission_lease_renewable())
        self.assertEqual(self.rig.drives(mark), [])

        self.assertTrue(self.agent.clear_emergency_stop("operator"))
        self.assertFalse(self.rig.link.status().motion_ready)       # must re-prove with a PING first
        mark = self.rig.mark()
        snaps = self.rig.run(2.0)
        self.assertTrue(all(s.mode is OperatingMode.PAUSED for s in snaps))
        self.assertEqual(self.rig.drives(mark), [])
        self.assertIs(snaps[-1].communication.esp32, Esp32LinkStatus.UP)   # PING answered

        self.agent.resume_mission("operator RESUME")
        snaps = self.rig.run(1.0)
        self.assertIs(snaps[-1].mode, OperatingMode.AUTO)
        self.assertTrue(self.rig.drives(mark))


class TestLeaseRenewal(unittest.TestCase):
    """HEARTBEAT renews the commitment lease only while the Rover could deliver."""

    def test_heartbeat_withholds_the_lease_while_latched_and_restores_it_after_clear(self):
        rig = ready_rig()
        rig.agent.gps.set_fix(*START)               # where the OFFER fixture's route starts
        rig.run(0.6)
        sio = FakeSio()
        link, _ = engine_link(agent=rig.agent, state=rig.agent.state, sio=sio)
        rig.agent.backend = link
        agent = rig.agent

        async def beat():
            await link._publish_heartbeat()
            return sio.events_named("HEARTBEAT")[-1]

        async def scenario():
            await connect_and_auth(link)
            await sio.fire(OFFER, efx.envelope())
            beats = {"accepted": await beat()}
            agent.emergency_stop("operator")
            beats["latched"] = await beat()
            agent.clear_emergency_stop("operator")
            beats["cleared"] = await beat()
            agent.stop_mission("operator STOP")
            beats["stopped"] = await beat()
            return beats

        beats = asyncio.run(scenario())
        self.assertEqual(len(sio.events_named("OFFER_ACCEPT")), 1)
        self.assertIn("commitmentId", beats["accepted"])
        self.assertEqual(beats["latched"], {})
        self.assertEqual(beats["cleared"], beats["accepted"])
        self.assertEqual(beats["stopped"], {})
        self.assertGreaterEqual(link.stats["lease_renewals_withheld"], 1)

    def test_error_mode_does_not_renew(self):
        rig = Rig()
        rig.agent.assign_mission(mission(), custody_required=True)
        self.assertTrue(rig.agent.mission_lease_renewable())
        rig.agent.state.set_mode(OperatingMode.ERROR, error="tick failed")
        self.assertFalse(rig.agent.mission_lease_renewable())


if __name__ == "__main__":
    unittest.main()
