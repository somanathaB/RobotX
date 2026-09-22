"""Regression tests for defects found in the production-readiness audit.

Each test names the defect it locks down. They are grouped here rather than
scattered so the audit's findings stay traceable.
"""

import asyncio
import threading
import time
import unittest

from robotx.application.agent import RobotAgent
from robotx.config.settings import Settings
from robotx.diagnostics.health import ComponentHealth, HealthStatus
from robotx.hardware.camera import CameraConfig, CameraStatus, CameraStream
from robotx.hardware.gps import GpsReading, GPSStatus
from robotx.perception.pipeline import PerceptionPipeline, PipelineConfig
from robotx.perception.types import (
    Detection,
    FrameMetadata,
    PerceptionResult,
    PerceptionStatus,
)


def headless_settings(**overrides):
    env = {
        "ROBOTX_ROBOT_ID": "audit-rover",
        "ROBOTX_CAMERA_ENABLED": "0",
        "ROBOTX_PERCEPTION_ENABLED": "0",
        "ROBOTX_GPS_ENABLED": "0",
        "ROBOTX_LOG_LEVEL": "CRITICAL",
    }
    env.update(overrides)
    return Settings.from_env(env)


class TestPerceptionUsability(unittest.TestCase):
    """Defect: `is_usable` ignored frame metadata, bypassing the corridor check."""

    def test_ok_without_frame_is_not_usable(self):
        result = PerceptionResult(
            timestamp=time.time(),
            status=PerceptionStatus.OK,
            detections=(),
            frame=None,
            backend="test",
        )
        self.assertFalse(result.is_usable)

    def test_ok_with_frame_is_usable(self):
        result = PerceptionResult(
            timestamp=time.time(),
            status=PerceptionStatus.OK,
            detections=(),
            frame=FrameMetadata(width=640, height=480),
            backend="test",
        )
        self.assertTrue(result.is_usable)

    def test_pipeline_always_attaches_frame_metadata_to_ok_results(self):
        # The guarantee the decision layer relies on.
        class Frame:
            shape = (240, 320, 3)

        class Cam:
            def get_frame(self): return Frame()
            def last_frame_age_s(self): return 0.01

        class Det:
            backend = "fake"
            def detect_objects(self, frame): return []

        pipe = PerceptionPipeline(
            Cam(), Det(),
            PipelineConfig(inference_width=320, inference_height=240,
                           stale_after_s=60, frame_stale_after_s=60),
        )
        result = pipe.step_once()
        self.assertIs(result.status, PerceptionStatus.OK)
        self.assertIsNotNone(result.frame)
        self.assertTrue(result.is_usable)


class TestHealthEvaluation(unittest.TestCase):
    """Defects: multiple snapshots per evaluation; KeyError on unknown status."""

    def setUp(self):
        self.agent = RobotAgent(headless_settings())

    def test_health_uses_a_single_snapshot(self):
        calls = {"n": 0}
        real = self.agent.state.snapshot

        def counting():
            calls["n"] += 1
            return real()

        self.agent.state.snapshot = counting
        self.agent._component_health(self.agent.state.snapshot())
        # One explicit snapshot here, and none taken inside.
        self.assertEqual(calls["n"], 1)

    def test_unknown_camera_status_does_not_raise(self):
        class OddCamera:
            def get_status(self): return "SOMETHING_NEW"
            def last_frame_age_s(self): return None

        self.agent.camera = OddCamera()
        health = self.agent._camera_health()
        self.assertIs(health.status, HealthStatus.UNKNOWN)
        self.assertIn("unrecognized", health.detail)

    def test_unknown_gps_status_does_not_raise(self):
        self.agent.gps = object()  # presence is all that is checked
        snap = self.agent.state.snapshot()
        object.__setattr__(snap, "gps", GpsReading(status="WEIRD"))  # type: ignore[arg-type]
        health = self.agent._gps_health(snap)
        self.assertIs(health.status, HealthStatus.UNKNOWN)

    def test_component_health_covers_every_subsystem(self):
        components = self.agent._component_health(self.agent.state.snapshot())
        for name in ("camera", "perception", "gps", "navigation", "communication"):
            self.assertIn(name, components)
            self.assertIsInstance(components[name], ComponentHealth)


class TestCameraConfigReporting(unittest.TestCase):
    """Defect: a stream reported its requested size, not the device's actual one."""

    def test_unattached_stream_reports_its_own_config(self):
        cam = CameraStream(CameraConfig(width=800, height=600, fps=15))
        self.assertEqual((cam.width, cam.height), (800, 600))

    def test_attached_stream_reports_the_device_config(self):
        cam = CameraStream(CameraConfig(width=800, height=600, fps=15))

        class Manager:  # stands in for the process-wide device manager
            cfg = CameraConfig(width=640, height=480, fps=20)
            capture_failed = False
            last_error = None
            def last_frame_age_s(self): return 0.01

        cam._manager = Manager()
        cam._started = True
        # libcamera gave us 640x480; reporting 800x600 would be a lie.
        self.assertEqual((cam.width, cam.height), (640, 480))
        described = cam.describe()
        self.assertEqual(described["width"], 640)
        self.assertEqual(described["height"], 480)

    def test_status_is_stopped_before_start(self):
        self.assertIs(CameraStream(CameraConfig()).get_status(), CameraStatus.STOPPED)


class TestPipelineThreadSafety(unittest.TestCase):
    """Defect: the area-delta read-modify-write was unguarded."""

    def test_concurrent_step_once_does_not_corrupt_state(self):
        class Frame:
            shape = (240, 320, 3)

        class Cam:
            def get_frame(self): return Frame()
            def last_frame_age_s(self): return 0.01

        class Det:
            backend = "fake"
            def detect_objects(self, frame):
                return [{"label": "obstacle", "confidence": 0.7,
                         "bbox": (10, 10, 60, 60), "area": 2500}]

        pipe = PerceptionPipeline(
            Cam(), Det(),
            PipelineConfig(inference_width=320, inference_height=240,
                           stale_after_s=60, frame_stale_after_s=60),
        )
        errors = []

        def hammer():
            try:
                for _ in range(50):
                    pipe.step_once()
            except Exception as e:  # pragma: no cover - failure path
                errors.append(e)

        threads = [threading.Thread(target=hammer) for _ in range(4)]
        for t in threads: t.start()
        for t in threads: t.join(timeout=10)

        self.assertEqual(errors, [])
        self.assertFalse(any(t.is_alive() for t in threads))
        self.assertIs(pipe.latest().status, PerceptionStatus.OK)


class TestNoMotorAuthority(unittest.TestCase):
    """Architectural guarantee: the agent must never gain motor control."""

    def test_agent_has_no_motor_attribute(self):
        agent = RobotAgent(headless_settings())
        for forbidden in ("motors", "motor", "motor_driver"):
            self.assertFalse(hasattr(agent, forbidden))

    def test_agent_import_graph_excludes_gpio_and_motor_modules(self):
        import sys
        import robotx.application.main  # noqa: F401

        loaded = set(sys.modules)
        self.assertNotIn("RPi.GPIO", loaded)
        self.assertNotIn("robotx.hardware.motors", loaded)
        self.assertNotIn("robotx.hardware.encoders", loaded)
        # The old `communication.socket_client` assertion lived here. That
        # module is gone, so the check would pass vacuously. Its successor --
        # that `socketio` is not loaded when the backend link is disabled --
        # is in test_agent_backend_commands.py, where it runs in a subprocess.
        # It has to: other test modules in this suite import the link
        # deliberately, so a `sys.modules` check here would depend on test
        # ordering rather than on the agent's behaviour.

    def test_stopping_publishes_a_stop_intent_synchronously(self):
        # No waiting for the next tick: stopping must take effect at once.
        async def run():
            agent = RobotAgent(headless_settings())
            await agent.start()
            agent.start_mission([(51.5, -0.1)])
            agent.stop_mission("test")
            intent = agent.state.snapshot().motion_intent
            await agent.stop()
            return intent

        intent = asyncio.run(run())
        self.assertTrue(intent.is_stop)


class TestDeadCodeRemoved(unittest.TestCase):
    def test_record_error_is_gone(self):
        # Removed in the audit: it had no callers anywhere in the repo.
        from robotx.state.robot_state import RobotState
        self.assertFalse(hasattr(RobotState("x"), "record_error"))


if __name__ == "__main__":
    unittest.main()
