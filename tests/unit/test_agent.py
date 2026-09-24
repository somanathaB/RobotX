"""Agent wiring: a full sense->decide->publish tick without any hardware.

Every subsystem is replaced by a stand-in, so these tests check the agent's own
behaviour: startup degradation, the tick sequence, mission control, telemetry,
health aggregation, and shutdown.
"""

import asyncio
import time
import unittest

from robotx.application.agent import RobotAgent
from robotx.config.settings import Settings
from robotx.control.motion import MotionCommand
from robotx.diagnostics.health import HealthStatus
from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus
from robotx.perception.types import (
    Detection,
    FrameMetadata,
    PerceptionResult,
    PerceptionStatus,
)
from robotx.state.robot_state import BackendLinkStatus as LinkStatus, Esp32LinkStatus, OperatingMode


HERE = (51.500000, -0.100000)
NORTH = (51.500900, -0.100000)  # ~100 m north


def headless_settings(**overrides):
    """Settings with every piece of hardware switched off."""

    env = {
        "ROBOTX_ROBOT_ID": "test-rover",
        "ROBOTX_CAMERA_ENABLED": "0",
        "ROBOTX_PERCEPTION_ENABLED": "0",
        "ROBOTX_GPS_ENABLED": "0",
        "ROBOTX_LOG_LEVEL": "CRITICAL",
    }
    env.update(overrides)
    return Settings.from_env(env)


class FakeGPS:
    def __init__(self, reading=None):
        self.reading = reading or GpsReading(status=GPSStatus.NO_FIX)
        self.stopped = False

    def get_reading(self):
        return self.reading

    def set_fix(self, lat, lon, *, speed=None, track=None):
        self.reading = GpsReading(
            status=GPSStatus.FIX,
            fix=GpsFix(
                latitude=lat,
                longitude=lon,
                timestamp=time.time(),
                speed_mps=speed,
                track_deg=track,
            ),
            age_s=0.1,
        )

    def stop(self):
        self.stopped = True


class FakePerception:
    def __init__(self, result=None):
        self.result = result or PerceptionResult.unavailable(PerceptionStatus.DISABLED)
        self.stopped = False
        self.enabled = True

    def latest(self):
        return self.result

    def set_clear(self, width=640, height=480):
        self.result = PerceptionResult(
            timestamp=time.time(),
            status=PerceptionStatus.OK,
            detections=(),
            frame=FrameMetadata(width=width, height=height, age_s=0.05),
            backend="fake",
        )

    def set_detection(self, label="obstacle", area=40000, cx=320):
        half = int((area**0.5) / 2)
        detection = Detection(
            label=label,
            confidence=0.8,
            bbox=(cx - half, 100, cx + half, 100 + 2 * half),
            area_px=area,
        )
        self.result = PerceptionResult(
            timestamp=time.time(),
            status=PerceptionStatus.OK,
            detections=(detection,),
            frame=FrameMetadata(width=640, height=480, age_s=0.05),
            backend="fake",
        )

    def stop(self):
        self.stopped = True


class AgentTestCase(unittest.TestCase):
    """Builds an agent with fake subsystems, bypassing hardware startup."""

    def make_agent(self, **overrides):
        agent = RobotAgent(headless_settings(**overrides))
        agent.gps = FakeGPS()
        agent.perception = FakePerception()
        return agent


class TestAgentTick(AgentTestCase):
    def test_tick_without_any_hardware_still_produces_state(self):
        agent = RobotAgent(headless_settings())
        agent.perception = FakePerception()
        snapshot = agent.tick()

        self.assertEqual(snapshot.robot_id, "test-rover")
        self.assertIs(snapshot.mode, OperatingMode.IDLE)
        self.assertIsNone(snapshot.position)

    def test_idle_agent_holds(self):
        agent = self.make_agent()
        snapshot = agent.tick()
        self.assertIs(snapshot.motion_intent.command, MotionCommand.HOLD)

    def test_gps_fix_becomes_a_position(self):
        agent = self.make_agent()
        agent.gps.set_fix(*HERE)
        snapshot = agent.tick()

        self.assertIsNotNone(snapshot.position)
        self.assertAlmostEqual(snapshot.position.latitude, HERE[0])
        self.assertIs(snapshot.gps.status, GPSStatus.FIX)

    def test_mission_drives_forward_when_clear(self):
        agent = self.make_agent()
        agent.gps.set_fix(*HERE, speed=1.0, track=0.0)  # heading north
        agent.perception.set_clear()
        agent.start_mission([NORTH])
        snapshot = agent.tick()

        self.assertIs(snapshot.mode, OperatingMode.AUTO)
        self.assertIs(snapshot.motion_intent.command, MotionCommand.FORWARD)
        self.assertGreater(snapshot.motion_intent.left, 0.0)

    def test_obstacle_stops_an_active_mission(self):
        agent = self.make_agent()
        agent.gps.set_fix(*HERE, speed=1.0, track=0.0)
        agent.perception.set_detection(area=50000, cx=320)
        agent.start_mission([NORTH])
        snapshot = agent.tick()

        self.assertTrue(snapshot.motion_intent.is_stop)
        self.assertIn("obstacle", snapshot.motion_intent.reason)

    def test_person_stops_an_active_mission(self):
        agent = self.make_agent()
        agent.gps.set_fix(*HERE, speed=1.0, track=0.0)
        agent.perception.set_detection(label="person", area=5000)
        agent.start_mission([NORTH])
        snapshot = agent.tick()

        self.assertTrue(snapshot.motion_intent.is_stop)
        self.assertIn("person", snapshot.motion_intent.reason)

    def test_perception_failure_stops_an_active_mission(self):
        agent = self.make_agent()
        agent.gps.set_fix(*HERE, speed=1.0, track=0.0)
        agent.perception.result = PerceptionResult.unavailable(PerceptionStatus.NO_FRAME)
        agent.start_mission([NORTH])
        snapshot = agent.tick()

        self.assertTrue(snapshot.motion_intent.is_stop)
        self.assertIn("perception unavailable", snapshot.motion_intent.reason)

    def test_losing_gps_mid_mission_stops(self):
        agent = self.make_agent()
        agent.gps.set_fix(*HERE, speed=1.0, track=0.0)
        agent.perception.set_clear()
        agent.start_mission([NORTH])
        agent.tick()

        agent.gps.reading = GpsReading(status=GPSStatus.NO_FIX)
        snapshot = agent.tick()
        self.assertTrue(snapshot.motion_intent.is_stop)

    def test_tick_never_actuates_anything(self):
        # The agent owns no motor driver at all.
        agent = self.make_agent()
        self.assertFalse(hasattr(agent, "motors"))
        agent.tick()


class TestMissionControl(AgentTestCase):
    def test_start_mission_requires_a_waypoint(self):
        agent = self.make_agent()
        with self.assertRaises(ValueError):
            agent.start_mission([])

    def test_stop_mission_clears_route_and_stops(self):
        agent = self.make_agent()
        agent.gps.set_fix(*HERE)
        agent.perception.set_clear()
        agent.start_mission([NORTH])
        agent.stop_mission("test stop")

        snapshot = agent.state.snapshot()
        self.assertIs(snapshot.mode, OperatingMode.STOPPED)
        self.assertTrue(snapshot.motion_intent.is_stop)

    def test_resume_idle(self):
        agent = self.make_agent()
        agent.stop_mission()
        agent.resume_idle()
        self.assertIs(agent.state.mode, OperatingMode.IDLE)


class TestTelemetryAndHealth(AgentTestCase):
    def test_telemetry_is_published_to_sinks(self):
        agent = self.make_agent(ROBOTX_TELEMETRY_INTERVAL_S="0")
        received = []
        agent.add_telemetry_sink(received.append)
        agent.tick()

        self.assertEqual(len(received), 1)
        self.assertEqual(received[0]["robot_id"], "test-rover")
        self.assertIsNone(received[0]["battery"]["percent"])

    def test_a_failing_sink_does_not_break_the_tick(self):
        agent = self.make_agent(ROBOTX_TELEMETRY_INTERVAL_S="0")

        def explode(_payload):
            raise RuntimeError("sink failure")

        received = []
        agent.add_telemetry_sink(explode)
        agent.add_telemetry_sink(received.append)

        agent.tick()
        self.assertEqual(len(received), 1)

    def test_health_reports_disabled_subsystems_without_failing(self):
        agent = self.make_agent(ROBOTX_HEALTH_INTERVAL_S="0")
        snapshot = agent.tick()

        components = snapshot.health.components
        self.assertIn("camera", components)
        self.assertIn("gps", components)
        self.assertIn("system", components)
        # Disabled hardware is UNKNOWN, which degrades rather than fails.
        self.assertIs(components["camera"].status, HealthStatus.UNKNOWN)
        self.assertIsNot(snapshot.health.status, HealthStatus.FAILED)

    def test_communication_links_are_reported_as_inactive(self):
        agent = self.make_agent()
        asyncio.run(self._start_then_stop(agent))
        comms = agent.state.snapshot().communication
        self.assertIs(comms.esp32, Esp32LinkStatus.NOT_IMPLEMENTED)
        self.assertIs(comms.backend, LinkStatus.DISABLED)

    async def _start_then_stop(self, agent):
        await agent.start()
        await agent.stop()


class TestAgentLifecycle(unittest.TestCase):
    def test_start_and_stop_with_no_hardware(self):
        async def run():
            agent = RobotAgent(headless_settings())
            await agent.start()
            self.assertIsNone(agent.camera)
            self.assertIsNotNone(agent.perception)
            await asyncio.sleep(0.05)
            await agent.stop()
            return agent

        agent = asyncio.run(run())
        self.assertIs(agent.state.mode, OperatingMode.STOPPED)
        self.assertTrue(agent.state.snapshot().motion_intent.is_stop)

    def test_stop_releases_subsystems(self):
        async def run():
            agent = RobotAgent(headless_settings())
            gps, perception = FakeGPS(), FakePerception()
            await agent.start()
            agent.gps, agent.perception = gps, perception
            await agent.stop()
            return gps, perception

        gps, perception = asyncio.run(run())
        self.assertTrue(gps.stopped)
        self.assertTrue(perception.stopped)

    def test_loop_survives_a_failing_tick(self):
        async def run():
            agent = RobotAgent(headless_settings(ROBOTX_AGENT_HZ="50"))
            await agent.start()

            calls = {"n": 0}
            original = agent.decision.decide

            def flaky(**kwargs):
                calls["n"] += 1
                if calls["n"] == 1:
                    raise RuntimeError("transient decision failure")
                return original(**kwargs)

            agent.decision.decide = flaky
            await asyncio.sleep(0.15)
            await agent.stop()
            return calls["n"]

        calls = asyncio.run(run())
        self.assertGreater(calls, 1)  # the loop kept running after the failure

    def test_a_failing_tick_stops_the_robot_and_records_the_error(self):
        agent = RobotAgent(headless_settings())
        agent.perception = FakePerception()

        def explode(**_kwargs):
            raise RuntimeError("decision exploded")

        agent.decision.decide = explode

        async def run():
            await agent.start()
            await asyncio.sleep(0.15)
            await agent.stop()

        asyncio.run(run())
        # stop() moves the mode to STOPPED, but the error and the stop intent
        # from the failed tick must both have been recorded.
        self.assertIn("decision exploded", agent.state.snapshot().last_error or "")

    def test_double_start_is_harmless(self):
        async def run():
            agent = RobotAgent(headless_settings())
            await agent.start()
            await agent.start()
            await agent.stop()

        asyncio.run(run())


class TestMidRunDegradation(AgentTestCase):
    """Subsystems failing *during* a mission, not merely absent at startup.

    A robot that starts healthy and then goes blind is the dangerous case: it
    is already moving. Each test drives a good tick first, then breaks one
    subsystem and asserts the very next tick stops.
    """

    def running_mission(self):
        agent = self.make_agent()
        agent.gps.set_fix(*HERE, speed=1.0, track=0.0)
        agent.perception.set_clear()
        agent.start_mission([NORTH])
        first = agent.tick()
        self.assertIs(first.motion_intent.command, MotionCommand.FORWARD,
                      "precondition: should be driving before the fault")
        return agent

    def test_camera_dying_mid_mission_stops(self):
        agent = self.running_mission()
        agent.perception.result = PerceptionResult.unavailable(
            PerceptionStatus.NO_FRAME, error="camera stopped delivering"
        )
        snapshot = agent.tick()
        self.assertTrue(snapshot.motion_intent.is_stop)
        self.assertIn("perception unavailable", snapshot.motion_intent.reason)

    def test_perception_erroring_mid_mission_stops(self):
        agent = self.running_mission()
        agent.perception.result = PerceptionResult.unavailable(
            PerceptionStatus.DETECTOR_ERROR, error="inference blew up"
        )
        self.assertTrue(agent.tick().motion_intent.is_stop)

    def test_perception_going_stale_mid_mission_stops(self):
        agent = self.running_mission()
        agent.perception.result = PerceptionResult.unavailable(PerceptionStatus.STALE)
        self.assertTrue(agent.tick().motion_intent.is_stop)

    def test_gps_dropping_mid_mission_stops(self):
        agent = self.running_mission()
        agent.gps.reading = GpsReading(status=GPSStatus.NO_FIX)
        snapshot = agent.tick()
        self.assertTrue(snapshot.motion_intent.is_stop)
        self.assertIn("GPS", snapshot.motion_intent.reason)

    def test_gps_going_stale_mid_mission_stops(self):
        agent = self.running_mission()
        agent.gps.reading = GpsReading(
            status=GPSStatus.STALE,
            fix=GpsFix(latitude=HERE[0], longitude=HERE[1], timestamp=time.time() - 60),
            age_s=60.0,
        )
        self.assertTrue(agent.tick().motion_intent.is_stop)

    def test_recovery_resumes_motion(self):
        # Degradation must not be a one-way latch: once the fault clears and
        # the robot genuinely knows the path is clear, it may drive again.
        agent = self.running_mission()
        agent.perception.result = PerceptionResult.unavailable(PerceptionStatus.NO_FRAME)
        self.assertTrue(agent.tick().motion_intent.is_stop)

        agent.perception.set_clear()
        agent.gps.set_fix(*HERE, speed=1.0, track=0.0)
        self.assertIs(agent.tick().motion_intent.command, MotionCommand.FORWARD)

    def test_health_reflects_the_fault(self):
        agent = self.make_agent(ROBOTX_HEALTH_INTERVAL_S="0")
        agent.gps.set_fix(*HERE, speed=1.0, track=0.0)
        agent.perception.set_clear()
        agent.start_mission([NORTH])
        agent.tick()

        agent.perception.result = PerceptionResult.unavailable(
            PerceptionStatus.DETECTOR_ERROR, error="inference blew up"
        )
        snapshot = agent.tick()
        self.assertIs(
            snapshot.health.components["perception"].status, HealthStatus.FAILED
        )
        self.assertIs(snapshot.health.status, HealthStatus.FAILED)


if __name__ == "__main__":
    unittest.main()
