"""The ESP32 link inside RobotAgent: state, health, safety and ownership.

The link is always built over a fake port here and assigned to the agent the
same way other agent tests inject fake GPS or perception. No test in this file
can open a real serial port: `ROBOTX_ESP32_ENABLED` is never set.
"""

import logging
import time
import unittest
from unittest import mock

from robotx.application.agent import RobotAgent
from robotx.config.settings import Settings
from robotx.control.motion import MotionIntent
from robotx.diagnostics.health import HealthStatus
from robotx.perception.types import PerceptionResult, PerceptionStatus
from robotx.state.robot_state import Esp32LinkStatus, OperatingMode
from robotx.state.telemetry import build_telemetry
from tests.fixtures import esp32 as fx


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


def settings(**overrides):
    env = {
        "ROBOTX_ROBOT_ID": "test-rover",
        "ROBOTX_CAMERA_ENABLED": "0",
        "ROBOTX_PERCEPTION_ENABLED": "0",
        "ROBOTX_GPS_ENABLED": "0",
        "ROBOTX_LOG_LEVEL": "CRITICAL",
    }
    env.update(overrides)
    return Settings.from_env(env)


class _UsablePerception:
    """A clear, fresh perception result, so only the ESP32 path is under test."""

    def latest(self):
        return PerceptionResult(timestamp=time.time(), status=PerceptionStatus.OK)


def agent_with_link(*, motion=False, drive_available=False, **env):
    agent = RobotAgent(settings(**env))
    port = fx.FakePort()
    link, clock = fx.make_link(port, motion_enabled=motion)
    agent.esp32 = link
    link.poll_once()
    port.feed(fx.telemetry(motor_drive_available=drive_available, last_seq=None))
    link.poll_once()
    port.feed(fx.ack(1) + fx.REAL_DIAG_SYSTEM + fx.REAL_DIAG_FRONT)   # one read
    link.poll_once()
    assert not port.inbox, "every queued chunk must have been read"
    return agent, link, port, clock


class TestDefaults(unittest.TestCase):
    def test_disabled_by_default_and_reported_as_disabled(self):
        self.assertFalse(Settings().esp32_enabled)
        self.assertFalse(Settings().esp32_motion_enabled)
        agent = RobotAgent(settings())
        agent._start_esp32()
        self.assertIsNone(agent.esp32)
        snap = agent.tick()
        self.assertIsNone(snap.controller)
        self.assertIsNone(build_telemetry(snap)["controller"])

    def test_settings_are_read_from_the_environment(self):
        s = settings(ROBOTX_ESP32_ENABLED="1", ROBOTX_ESP32_PORT="/dev/ttyAMA3",
                     ROBOTX_ESP32_TRANSMIT_ENABLED="0", ROBOTX_ESP32_STALE_AFTER_S="2.5",
                     ROBOTX_ESP32_COMMAND_MAX_AGE_S="0.2")
        self.assertTrue(s.esp32_enabled)
        self.assertEqual(s.esp32_port, "/dev/ttyAMA3")
        self.assertFalse(s.esp32_transmit_enabled)
        self.assertEqual(s.esp32_stale_after_s, 2.5)
        self.assertEqual(s.esp32_command_max_age_s, 0.2)

    def test_an_invalid_esp32_config_leaves_the_link_off_and_says_so(self):
        # Enabled, but with a command age the link refuses. It must never
        # reach a serial port -- and the robot must still come up.
        agent = RobotAgent(settings(ROBOTX_ESP32_ENABLED="1", ROBOTX_ESP32_COMMAND_MAX_AGE_S="5"))
        agent._start_esp32()
        self.assertIsNone(agent.esp32)
        agent.state.update_communication(esp32=Esp32LinkStatus.DISABLED)
        h = agent._component_health(agent.tick())["esp32"]
        self.assertIs(h.status, HealthStatus.FAILED)
        self.assertIn("command max age", h.detail)

    def test_alias_ports_are_a_config_error_not_a_crash(self):
        agent = RobotAgent(settings(ROBOTX_ESP32_ENABLED="1", ROBOTX_ESP32_PORT="/dev/serial0"))
        agent._start_esp32()
        self.assertIsNone(agent.esp32)
        self.assertIn("/dev/serial0", agent._esp32_config_error)


class TestTelemetryIntoState(unittest.TestCase):
    def test_link_status_and_controller_reach_the_snapshot(self):
        agent, link, port, clock = agent_with_link()
        snap = agent.tick()
        self.assertIs(snap.communication.esp32, Esp32LinkStatus.UP)
        self.assertIsNotNone(snap.communication.esp32_last_rx_at)
        tel = snap.controller.telemetry
        self.assertEqual(tel.uptime_ms, 3588443)
        self.assertIs(tel.motor_drive_available, False)
        self.assertEqual(snap.controller.proto, 2)

    def test_telemetry_carries_the_controller_but_not_diag(self):
        agent, *_ = agent_with_link()
        payload = build_telemetry(agent.tick())
        ctrl = payload["controller"]
        self.assertEqual(ctrl["telemetry"]["motor"]["drive_available"], False)
        self.assertEqual(ctrl["telemetry"]["front"]["left_cm"], 37.9)
        self.assertNotIn("controller_diag", payload)
        self.assertNotIn("pca_status", str(payload))

    def test_diag_is_in_the_diagnostic_snapshot(self):
        agent, *_ = agent_with_link()
        snap = agent.tick().to_dict()
        self.assertEqual(snap["controller_diag"]["SYSTEM"]["pca_status"], "ADDRESS_UNCONFIRMED")
        self.assertIn("FRONT", snap["controller_diag"])

    def test_the_snapshot_serializes(self):
        import json

        agent, *_ = agent_with_link()
        json.dumps(agent.tick().to_dict())
        json.dumps(build_telemetry(agent.tick()))


class TestHealth(unittest.TestCase):
    def health(self, agent):
        return agent._component_health(agent.tick())["esp32"]

    def test_up_but_drive_unavailable_is_degraded_with_the_reason(self):
        agent, *_ = agent_with_link()
        h = self.health(agent)
        self.assertIs(h.status, HealthStatus.DEGRADED)
        self.assertIn("motor drive unavailable (ADDRESS_UNCONFIRMED)", h.detail)

    def test_up_with_nothing_wrong_is_healthy(self):
        agent, link, port, clock = agent_with_link(drive_available=True)
        port.feed(fx.telemetry(motor_drive_available=True, front_obstacle=False,
                               forward_blocked=False, uptime_ms=3590000))
        link.poll_once()
        self.assertIs(self.health(agent).status, HealthStatus.HEALTHY)

    def test_a_quiet_esp32_fails(self):
        agent, link, port, clock = agent_with_link()
        clock.advance(2.0)
        link.poll_once()
        h = self.health(agent)
        self.assertIs(h.status, HealthStatus.FAILED)
        self.assertIs(agent.state.snapshot().communication.esp32, Esp32LinkStatus.STALE)

    def test_connecting_is_degraded_not_failed(self):
        agent = RobotAgent(settings())
        port = fx.FakePort()
        agent.esp32, _ = fx.make_link(port)
        agent.esp32.poll_once()
        h = self.health(agent)
        self.assertIs(h.status, HealthStatus.DEGRADED)
        self.assertIn("TELEMETRY", h.detail)


class TestRebootLatchesTheEmergencyStop(unittest.TestCase):
    def test_reboot_engages_estop_once_and_clearing_acknowledges(self):
        agent, link, port, clock = agent_with_link()
        agent.tick()
        port.feed(fx.telemetry(uptime_ms=500, last_seq=None))
        link.poll_once()
        snap = agent.tick()
        self.assertTrue(agent.emergency_stopped)
        self.assertIs(snap.mode, OperatingMode.STOPPED)
        self.assertIn("ESP32 rebooted", snap.last_error)
        agent.tick()                                        # latched once, not re-engaged

        self.assertTrue(agent.clear_emergency_stop("operator"))
        self.assertFalse(link.status().controller.reboot_latched)
        port.feed(fx.telemetry(uptime_ms=700, last_seq=None))
        link.poll_once()
        self.assertIs(link.status().link, Esp32LinkStatus.CONNECTING)   # must re-PING
        self.assertFalse(agent.emergency_stopped)


class TestSafetyPath(unittest.TestCase):
    def test_the_link_receives_the_gated_decision_not_the_proposal(self):
        agent, link, port, clock = agent_with_link(motion=True, drive_available=True)
        agent.start_mission([(51.5, -0.1)])
        # No perception: the decision layer and the gate both refuse to move.
        agent.tick()
        link.poll_once()
        self.assertEqual(port.command_names()[-1], "STOP")
        self.assertNotIn("DRIVE", port.command_names())

    def test_motion_disabled_never_sends_motion_whatever_the_agent_decides(self):
        agent, link, port, clock = agent_with_link(motion=False, drive_available=True)
        agent.perception = _UsablePerception()
        agent.start_mission([(51.5, -0.1)])
        for _ in range(5):
            agent.tick()
            link.poll_once()
        self.assertEqual(port.command_names(), ["PING"])
        self.assertEqual(port.writes[0], b"\n")

    def test_an_estop_sends_stop(self):
        agent, link, port, clock = agent_with_link(motion=True, drive_available=True)
        agent.emergency_stop("test")
        agent.tick()
        link.poll_once()
        self.assertEqual(port.command_names()[-1], "STOP")

    def test_losing_the_link_pauses_an_active_mission_when_motion_is_enabled(self):
        agent, link, port, clock = agent_with_link(motion=True, drive_available=True)
        agent.start_mission([(51.5, -0.1)])
        clock.advance(2.0)
        link.poll_once()
        snap = agent.tick()
        self.assertIs(snap.mode, OperatingMode.PAUSED)
        self.assertTrue(snap.motion_intent.is_stop)

    def test_with_motion_disabled_a_bench_mission_is_not_paused(self):
        agent, link, port, clock = agent_with_link(motion=False)
        agent.start_mission([(51.5, -0.1)])
        self.assertIs(agent.tick().mode, OperatingMode.AUTO)


class TestOffers(unittest.TestCase):
    def offer(self):
        from robotx.communication.engine import parse_envelope, parse_offer
        from tests.fixtures import engine as efx

        return parse_offer(parse_envelope(efx.envelope(), expected_agent_id=efx.ROBOT_ID))

    def test_an_up_link_without_motor_drive_is_not_a_motor_link(self):
        agent, link, port, clock = agent_with_link(motion=True, drive_available=False,
                                                   ROBOTX_ROBOT_ID="robotx-pi")
        agent.tick()
        self.assertIs(agent.state.snapshot().communication.esp32, Esp32LinkStatus.UP)
        self.assertEqual(agent.assess_offer(self.offer()).reason, "NO_MOTOR_LINK")

    def test_an_up_link_with_motion_disabled_is_not_a_motor_link(self):
        agent, link, port, clock = agent_with_link(motion=False, drive_available=True,
                                                   ROBOTX_ROBOT_ID="robotx-pi")
        agent.tick()
        self.assertEqual(agent.assess_offer(self.offer()).reason, "NO_MOTOR_LINK")

    def test_a_motor_capable_link_passes_to_the_next_check(self):
        agent, link, port, clock = agent_with_link(motion=True, drive_available=True,
                                                   ROBOTX_ROBOT_ID="robotx-pi")
        agent.tick()
        self.assertEqual(agent.assess_offer(self.offer()).reason, "NO_POSITION_FIX")


class TestUartOwnership(unittest.TestCase):
    def test_the_esp32_uart_is_reserved_even_with_the_link_disabled(self):
        # The default configuration: GPS enabled on /dev/ttyAMA0, ESP32 link off.
        agent = RobotAgent(settings(ROBOTX_GPS_ENABLED="1"))
        self.assertEqual(agent.settings.gps_port, agent.settings.esp32_port)
        self.assertFalse(agent.settings.esp32_enabled)
        agent._start_esp32()
        agent._start_gps()
        self.assertIsNone(agent.esp32)
        self.assertIsNone(agent.gps)
        h = agent._component_health(agent.tick())["gps"]
        self.assertIs(h.status, HealthStatus.FAILED)
        self.assertIn("reserved for the ESP32 UART", h.detail)

    def test_moving_the_esp32_port_frees_the_old_one_for_gps(self):
        agent = RobotAgent(settings(ROBOTX_GPS_ENABLED="1", ROBOTX_GPS_PORT="/dev/ttyAMA0",
                                    ROBOTX_ESP32_PORT="/dev/ttyAMA3"))
        # Never a real GPSReader in a unit test: it would open the device.
        with mock.patch("robotx.application.agent.GPSReader") as reader:
            agent._start_gps()
        reader.assert_called_once()
        self.assertIsNone(agent._gps_port_conflict)

    def test_gps_is_not_started_on_the_esp32_uart(self):
        agent = RobotAgent(settings(ROBOTX_GPS_ENABLED="1", ROBOTX_GPS_PORT="/dev/ttyAMA0"))
        agent.esp32, _ = fx.make_link(fx.FakePort(), port="/dev/ttyAMA0")
        agent._start_gps()
        self.assertIsNone(agent.gps)
        h = agent._component_health(agent.tick())["gps"]
        self.assertIs(h.status, HealthStatus.FAILED)
        self.assertIn("ESP32 UART", h.detail)

    def test_gps_on_its_own_port_is_unaffected(self):
        agent = RobotAgent(settings(ROBOTX_GPS_ENABLED="1", ROBOTX_GPS_PORT="/dev/ttyAMA3"))
        agent.esp32, _ = fx.make_link(fx.FakePort(), port="/dev/ttyAMA0")
        with mock.patch("robotx.application.agent.GPSReader") as reader:
            agent._start_gps()
        reader.assert_called_once()
        self.assertIsNone(agent._gps_port_conflict)


class TestShutdown(unittest.TestCase):
    def test_agent_stop_stops_the_link(self):
        import asyncio

        async def run():
            agent = RobotAgent(settings())
            await agent.start()
            port = fx.FakePort(block_s=0.005)
            link, _ = fx.make_link(port)
            link._clock = time.monotonic
            agent.esp32 = link
            link.start()
            await asyncio.sleep(0.05)
            await agent.stop()
            return link, port

        link, port = asyncio.run(run())
        self.assertTrue(port.closed)
        self.assertIsNone(link._thread)


if __name__ == "__main__":
    unittest.main()
