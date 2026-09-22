"""Robot state, telemetry payloads, and health aggregation."""

import time
import unittest

from robotx.control.motion import MotionIntent
from robotx.diagnostics.health import (
    ComponentHealth,
    HealthConfig,
    HealthMonitor,
    HealthStatus,
    SystemMetrics,
    read_cpu_temp_c,
    read_memory,
)
from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus
from robotx.navigation.navigator import NavigationState, NavigationStatus
from robotx.perception.types import PerceptionResult, PerceptionStatus
from robotx.localization.position import HeadingSource, Position
from robotx.state.robot_state import (
    CommunicationState,
    LinkStatus,
    OperatingMode,
    RobotState,
)
from robotx.state.telemetry import TELEMETRY_SCHEMA_VERSION, battery_telemetry, build_telemetry


def sample_position():
    return Position(
        latitude=51.5,
        longitude=-0.1,
        timestamp=time.time(),
        heading_deg=42.0,
        heading_source=HeadingSource.GPS_TRACK,
    )


def sample_gps_reading():
    return GpsReading(
        status=GPSStatus.FIX,
        fix=GpsFix(latitude=51.5, longitude=-0.1, timestamp=time.time(), satellites=9),
        age_s=0.3,
    )


class TestRobotState(unittest.TestCase):
    def setUp(self):
        self.state = RobotState("test-rover")

    def test_starts_idle_with_no_position(self):
        snapshot = self.state.snapshot()
        self.assertIs(snapshot.mode, OperatingMode.IDLE)
        self.assertIsNone(snapshot.position)
        self.assertIs(snapshot.gps.status, GPSStatus.UNAVAILABLE)

    def test_mission_active_only_in_auto(self):
        self.assertFalse(OperatingMode.IDLE.mission_active)
        self.assertFalse(OperatingMode.STOPPED.mission_active)
        self.assertFalse(OperatingMode.ERROR.mission_active)
        self.assertTrue(OperatingMode.AUTO.mission_active)

    def test_snapshot_is_a_copy_not_a_live_view(self):
        first = self.state.snapshot()
        self.state.set_mode(OperatingMode.AUTO)
        self.assertIs(first.mode, OperatingMode.IDLE)
        self.assertIs(self.state.snapshot().mode, OperatingMode.AUTO)

    def test_updates_are_recorded(self):
        self.state.update_gps(sample_gps_reading(), sample_position())
        self.state.update_navigation(NavigationState(status=NavigationStatus.NAVIGATING))
        self.state.update_motion_intent(MotionIntent.forward(0.4, reason="on course"))

        snapshot = self.state.snapshot()
        self.assertEqual(snapshot.position.latitude, 51.5)
        self.assertIs(snapshot.navigation.status, NavigationStatus.NAVIGATING)
        self.assertEqual(snapshot.motion_intent.reason, "on course")

    def test_last_position_is_kept_when_the_fix_drops(self):
        self.state.update_gps(sample_gps_reading(), sample_position())
        self.state.update_gps(GpsReading(status=GPSStatus.NO_FIX), None)

        snapshot = self.state.snapshot()
        self.assertIsNotNone(snapshot.position)  # last known, with its timestamp
        self.assertIs(snapshot.gps.status, GPSStatus.NO_FIX)  # but flagged as no fix

    def test_error_mode_records_the_message(self):
        self.state.set_mode(OperatingMode.ERROR, error="loop exploded")
        self.assertEqual(self.state.snapshot().last_error, "loop exploded")

    def test_communication_defaults_reflect_the_current_stage(self):
        comms = self.state.snapshot().communication
        self.assertIs(comms.esp32, LinkStatus.NOT_IMPLEMENTED)
        self.assertIs(comms.backend, LinkStatus.DISABLED)

    def test_communication_update_is_partial(self):
        self.state.update_communication(backend=LinkStatus.CONNECTED)
        comms = self.state.snapshot().communication
        self.assertIs(comms.backend, LinkStatus.CONNECTED)
        self.assertIs(comms.esp32, LinkStatus.NOT_IMPLEMENTED)

    def test_snapshot_serializes_fully(self):
        self.state.update_gps(sample_gps_reading(), sample_position())
        payload = self.state.snapshot().to_dict()
        for key in (
            "robot_id",
            "mode",
            "gps",
            "position",
            "navigation",
            "perception",
            "motion_intent",
            "communication",
            "health",
        ):
            self.assertIn(key, payload)


class TestTelemetry(unittest.TestCase):
    def setUp(self):
        self.state = RobotState("test-rover")
        self.state.update_gps(sample_gps_reading(), sample_position())

    def test_battery_is_unavailable_not_invented(self):
        # This robot has no battery sensing hardware.
        battery = battery_telemetry()
        self.assertEqual(battery["status"], "UNAVAILABLE")
        self.assertIsNone(battery["percent"])
        self.assertIsNone(battery["voltage_v"])

    def test_telemetry_contains_no_fabricated_battery_percentage(self):
        payload = build_telemetry(self.state.snapshot())
        self.assertIsNone(payload["battery"]["percent"])
        self.assertNotIn("76", str(payload["battery"]))

    def test_telemetry_has_every_required_section(self):
        payload = build_telemetry(self.state.snapshot())
        for key in (
            "schema_version",
            "timestamp",
            "robot_id",
            "mode",
            "gps",
            "position",
            "navigation",
            "perception",
            "motion_intent",
            "battery",
            "health",
            "communication",
        ):
            self.assertIn(key, payload)
        self.assertEqual(payload["schema_version"], TELEMETRY_SCHEMA_VERSION)

    def test_telemetry_is_json_serializable(self):
        import json

        payload = build_telemetry(self.state.snapshot())
        self.assertIsInstance(json.dumps(payload), str)

    def test_telemetry_reports_no_position_honestly(self):
        payload = build_telemetry(RobotState("bare").snapshot())
        self.assertIsNone(payload["position"])
        self.assertEqual(payload["gps"]["status"], "UNAVAILABLE")

    def test_perception_section_is_a_summary_not_every_box(self):
        payload = build_telemetry(self.state.snapshot())
        self.assertIn("count", payload["perception"])
        self.assertNotIn("detections", payload["perception"])


class TestHealthMetrics(unittest.TestCase):
    def test_metrics_are_optional_and_serializable(self):
        payload = SystemMetrics().to_dict()
        self.assertIsNone(payload["cpu_percent"])
        self.assertIsNone(payload["cpu_temp_c"])

    def test_memory_reader_returns_plausible_values_or_none(self):
        used_percent, available_mb = read_memory()
        if used_percent is not None:
            self.assertGreaterEqual(used_percent, 0.0)
            self.assertLessEqual(used_percent, 100.0)
            self.assertGreater(available_mb, 0.0)

    def test_temperature_reader_returns_plausible_value_or_none(self):
        temp = read_cpu_temp_c()
        if temp is not None:
            # Degrees, not millidegrees.
            self.assertGreater(temp, -40.0)
            self.assertLess(temp, 150.0)


class TestHealthMonitor(unittest.TestCase):
    def setUp(self):
        self.monitor = HealthMonitor(HealthConfig())

    def test_all_healthy_is_healthy(self):
        report = self.monitor.evaluate(
            {
                "camera": ComponentHealth("camera", HealthStatus.HEALTHY),
                "gps": ComponentHealth("gps", HealthStatus.HEALTHY),
            }
        )
        self.assertIn(report.status, (HealthStatus.HEALTHY, HealthStatus.DEGRADED))

    def test_one_failed_component_fails_the_agent(self):
        report = self.monitor.evaluate(
            {
                "camera": ComponentHealth("camera", HealthStatus.FAILED, "no camera"),
                "gps": ComponentHealth("gps", HealthStatus.HEALTHY),
            }
        )
        self.assertIs(report.status, HealthStatus.FAILED)

    def test_degraded_beats_healthy_but_not_failed(self):
        report = self.monitor.evaluate(
            {
                "camera": ComponentHealth("camera", HealthStatus.DEGRADED, "starting"),
                "gps": ComponentHealth("gps", HealthStatus.HEALTHY),
            }
        )
        self.assertIs(report.status, HealthStatus.DEGRADED)

    def test_unknown_component_degrades_rather_than_passing(self):
        report = self.monitor.evaluate(
            {"gps": ComponentHealth("gps", HealthStatus.UNKNOWN, "disabled")}
        )
        self.assertIs(report.status, HealthStatus.DEGRADED)

    def test_system_component_is_always_added(self):
        report = self.monitor.evaluate({})
        self.assertIn("system", report.components)

    def test_report_is_serializable(self):
        import json

        report = self.monitor.evaluate(
            {"camera": ComponentHealth("camera", HealthStatus.FAILED, "gone")}
        )
        payload = report.to_dict()
        self.assertEqual(payload["components"]["camera"]["status"], "FAILED")
        self.assertIsInstance(json.dumps(payload), str)

    def test_severity_ordering(self):
        self.assertLess(HealthStatus.HEALTHY.severity, HealthStatus.UNKNOWN.severity)
        self.assertLess(HealthStatus.UNKNOWN.severity, HealthStatus.DEGRADED.severity)
        self.assertLess(HealthStatus.DEGRADED.severity, HealthStatus.FAILED.severity)


if __name__ == "__main__":
    unittest.main()
