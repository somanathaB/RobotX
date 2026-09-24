"""Configuration: defaults, environment overrides, and secret handling."""

import unittest

from robotx.config.settings import Settings


class TestSettingsDefaults(unittest.TestCase):
    def test_defaults_do_not_require_environment(self):
        settings = Settings.from_env({})
        # No default identity: it must equal the commissioned Robot.robotId,
        # so only configuration can supply it.
        self.assertEqual(settings.robot_id, "")
        self.assertEqual(settings.gps_port, "/dev/ttyAMA0")
        self.assertEqual(settings.camera_width, 640)

    def test_no_secret_has_a_default_value(self):
        settings = Settings.from_env({})
        self.assertIsNone(settings.robot_token)
        self.assertIsNone(settings.google_maps_api_key)
        self.assertIsNone(settings.command_signing_key)

    def test_signing_key_is_never_exposed(self):
        settings = Settings.from_env({"ROBOTX_COMMAND_SIGNING_KEY": "k" * 40})
        self.assertEqual(settings.public_summary()["command_signing_key"], "SET")

    def test_backend_link_is_off_by_default(self):
        # The Pi agent must run standalone.
        self.assertFalse(Settings.from_env({}).socket_enabled)


class TestSettingsFromEnv(unittest.TestCase):
    def test_overrides_are_typed(self):
        settings = Settings.from_env(
            {
                "ROBOTX_ROBOT_ID": "rover-7",
                "ROBOTX_CAMERA_FPS": "15",
                "ROBOTX_DETECTION_MIN_CONF": "0.5",
                "ROBOTX_GPS_ENABLED": "false",
            }
        )
        self.assertEqual(settings.robot_id, "rover-7")
        self.assertEqual(settings.camera_fps, 15)
        self.assertIsInstance(settings.camera_fps, int)
        self.assertAlmostEqual(settings.detection_min_conf, 0.5)
        self.assertFalse(settings.gps_enabled)

    def test_boolean_forms(self):
        for value in ("1", "true", "TRUE", "yes", "on"):
            self.assertTrue(Settings.from_env({"ROBOTX_GPS_ENABLED": value}).gps_enabled, value)
        for value in ("0", "false", "no", "off", ""):
            self.assertFalse(Settings.from_env({"ROBOTX_GPS_ENABLED": value}).gps_enabled, value)

    def test_malformed_number_is_rejected_loudly(self):
        # Silently falling back to a default would hide a misconfigured robot.
        with self.assertRaises(ValueError) as ctx:
            Settings.from_env({"ROBOTX_CAMERA_FPS": "twenty"})
        self.assertIn("ROBOTX_CAMERA_FPS", str(ctx.exception))

    def test_settings_are_immutable(self):
        settings = Settings.from_env({})
        with self.assertRaises(Exception):
            settings.robot_id = "changed"  # type: ignore[misc]


class TestPublicSummary(unittest.TestCase):
    def test_secrets_are_never_exposed(self):
        settings = Settings.from_env(
            {"ROBOTX_ROBOT_TOKEN": "super-secret", "ROBOTX_GOOGLE_MAPS_API_KEY": "key-123"}
        )
        summary = settings.public_summary()
        self.assertEqual(summary["robot_token"], "SET")
        self.assertEqual(summary["google_maps_api_key"], "SET")
        self.assertNotIn("super-secret", str(summary))
        self.assertNotIn("key-123", str(summary))

    def test_unset_secrets_report_unset(self):
        summary = Settings.from_env({}).public_summary()
        self.assertEqual(summary["robot_token"], "UNSET")


if __name__ == "__main__":
    unittest.main()
