"""The Rover's internal architecture: state ownership and link separation.

These tests defend structural properties rather than behaviour, because the
properties are the thing that keeps breaking:

1. The **ESP32 link and the RobotX backend link are independent.** Separate
   state, separate enums, separate health. A rover whose own motor controller
   has died is in a completely different situation from one that merely cannot
   reach a dashboard, and an operator has to be able to tell them apart.

2. **`RobotState` is the single source of truth.** Producers write it;
   consumers read a snapshot. A consumer that reaches past its snapshot into
   hardware reports two instants in one frame.

3. **The Rover does not depend on RobotX.** Nothing in the robot's own state or
   telemetry may require a backend to exist.
"""

import ast
import logging
import pathlib
import unittest

from robotx.config.settings import Settings
from robotx.diagnostics.health import HealthStatus
from robotx.state.robot_state import (
    BackendLinkStatus,
    CommunicationState,
    Esp32LinkStatus,
    PowerState,
    RobotState,
)
from robotx.state.telemetry import build_telemetry


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]


def setUpModule():
    # Building an agent logs a health transition; not the subject under test.
    logging.getLogger("robotx").setLevel(logging.CRITICAL)


def headless_settings(**overrides):
    env = {
        "ROBOTX_ROBOT_ID": "test-rover",
        "ROBOTX_CAMERA_ENABLED": "0",
        "ROBOTX_PERCEPTION_ENABLED": "0",
        "ROBOTX_GPS_ENABLED": "0",
        "ROBOTX_LOG_LEVEL": "CRITICAL",
    }
    env.update(overrides)
    return Settings.from_env(env)


def imported_modules(path):
    """Every module name imported by one source file."""

    tree = ast.parse(pathlib.Path(path).read_text())
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
    return names


class TestLinkStateSeparation(unittest.TestCase):
    def test_the_two_links_use_different_enums(self):
        self.assertIsNot(BackendLinkStatus, Esp32LinkStatus)

    def test_esp32_has_no_backend_lifecycle_states(self):
        """A UART link does not authenticate, stream, or get its credential
        refused. Those states would be meaningless on it."""

        esp32_values = {s.value for s in Esp32LinkStatus}
        for backend_only in ("AUTHENTICATING", "AUTHENTICATED", "STREAMING", "AUTH_FAILED"):
            self.assertNotIn(backend_only, esp32_values)

    def test_backend_has_no_not_implemented_state(self):
        """The backend link exists; only the ESP32 one is unwritten."""

        self.assertNotIn("NOT_IMPLEMENTED", {s.value for s in BackendLinkStatus})
        self.assertIn("NOT_IMPLEMENTED", {s.value for s in Esp32LinkStatus})

    def test_defaults_state_each_link_honestly(self):
        comms = CommunicationState()
        self.assertIs(comms.esp32, Esp32LinkStatus.NOT_IMPLEMENTED)
        self.assertIs(comms.backend, BackendLinkStatus.DISABLED)

    def test_updating_the_backend_does_not_touch_the_esp32(self):
        state = RobotState("r")
        state.update_communication(
            backend=BackendLinkStatus.AUTH_FAILED, backend_detail="refused"
        )
        comms = state.snapshot().communication
        self.assertIs(comms.backend, BackendLinkStatus.AUTH_FAILED)
        self.assertIs(comms.esp32, Esp32LinkStatus.NOT_IMPLEMENTED)
        self.assertEqual(comms.esp32_detail, "")

    def test_updating_the_esp32_does_not_touch_the_backend(self):
        state = RobotState("r")
        state.update_communication(
            backend=BackendLinkStatus.STREAMING, backend_detail="streaming"
        )
        state.update_communication(
            esp32=Esp32LinkStatus.DISCONNECTED, esp32_detail="port closed"
        )
        comms = state.snapshot().communication
        self.assertIs(comms.esp32, Esp32LinkStatus.DISCONNECTED)
        self.assertIs(comms.backend, BackendLinkStatus.STREAMING)
        self.assertEqual(comms.backend_detail, "streaming")

    def test_no_backend_protocol_detail_leaks_into_domain_state(self):
        """Which event names are bound and which credential authenticated are
        facts about a Socket.IO client, not about the robot."""

        keys = set(CommunicationState().to_dict())
        for transport_trivia in (
            "backend_protocol_source",
            "backend_auth_method",
            "backend_protocol_provisional",
        ):
            self.assertNotIn(transport_trivia, keys)

    def test_esp32_link_carries_only_protocol_agnostic_observations(self):
        """Port state and rx timing are knowable without the UART contract;
        anything finer would be a guess at firmware we have not seen."""

        keys = set(CommunicationState().to_dict())
        self.assertIn("esp32_last_rx_at", keys)
        for guessed in ("esp32_battery", "esp32_encoders", "esp32_frames_parsed"):
            self.assertNotIn(guessed, keys)


class TestHealthSeparation(unittest.TestCase):
    def _components(self, **comm_updates):
        from robotx.application.agent import RobotAgent

        agent = RobotAgent(headless_settings())
        if comm_updates:
            agent.state.update_communication(**comm_updates)
        return agent._component_health(agent.state.snapshot())

    def test_both_links_are_reported_as_separate_components(self):
        components = self._components()
        self.assertIn("esp32", components)
        self.assertIn("backend", components)
        # The conflated component is gone.
        self.assertNotIn("communication", components)

    def test_backend_failure_does_not_mark_the_esp32_unhealthy(self):
        components = self._components(
            backend=BackendLinkStatus.AUTH_FAILED, backend_detail="refused"
        )
        self.assertIs(components["backend"].status, HealthStatus.FAILED)
        self.assertIsNot(components["esp32"].status, HealthStatus.FAILED)

    def test_esp32_failure_does_not_mark_the_backend_unhealthy(self):
        components = self._components(
            esp32=Esp32LinkStatus.DISCONNECTED,
            esp32_detail="port closed",
            backend=BackendLinkStatus.STREAMING,
        )
        self.assertIs(components["esp32"].status, HealthStatus.FAILED)
        self.assertIs(components["backend"].status, HealthStatus.HEALTHY)

    def test_a_quiet_esp32_is_a_failure_not_a_degradation(self):
        """An open port with nothing coming back is worse than a closed one:
        motion may still be commanded into silence."""

        components = self._components(esp32=Esp32LinkStatus.STALE)
        self.assertIs(components["esp32"].status, HealthStatus.FAILED)

    def test_an_unimplemented_esp32_is_unknown_not_failed(self):
        """Otherwise every standalone rover is permanently unhealthy for a
        component it does not have."""

        components = self._components()
        self.assertIs(components["esp32"].status, HealthStatus.UNKNOWN)

    def test_a_disabled_backend_is_unknown_not_failed(self):
        components = self._components(backend=BackendLinkStatus.DISABLED)
        self.assertIs(components["backend"].status, HealthStatus.UNKNOWN)


class TestSnapshotIsTheSingleSourceOfTruth(unittest.TestCase):
    def test_telemetry_battery_comes_from_the_snapshot(self):
        state = RobotState("r")
        state.update_power(
            PowerState(status="OK", percent=42.0, voltage_v=11.1, source="test")
        )
        battery = build_telemetry(state.snapshot())["battery"]
        self.assertEqual(battery["status"], "OK")
        self.assertEqual(battery["percent"], 42.0)
        self.assertEqual(battery["voltage_v"], 11.1)

    def test_telemetry_reflects_the_snapshot_it_was_given_not_the_live_state(self):
        """The decisive test: an older snapshot must keep reporting what was
        true when it was taken, which is impossible if telemetry reads
        hardware while building the frame."""

        state = RobotState("r")
        state.update_power(PowerState(status="OK", percent=90.0, source="test"))
        old_snapshot = state.snapshot()

        state.update_power(PowerState(status="OK", percent=10.0, source="test"))

        self.assertEqual(build_telemetry(old_snapshot)["battery"]["percent"], 90.0)
        self.assertEqual(build_telemetry(state.snapshot())["battery"]["percent"], 10.0)

    def test_default_power_is_unavailable_never_a_number(self):
        battery = build_telemetry(RobotState("r").snapshot())["battery"]
        self.assertEqual(battery["status"], "UNAVAILABLE")
        self.assertIsNone(battery["percent"])
        self.assertIsNone(battery["voltage_v"])

    def test_telemetry_module_reads_no_hardware(self):
        """Enforced structurally, because this is exactly the coupling that
        crept back in last time."""

        imports = imported_modules(REPO_ROOT / "robotx" / "state" / "telemetry.py")
        hardware = [name for name in imports if name.startswith("robotx.hardware")]
        self.assertEqual(hardware, [], f"telemetry must not import hardware: {hardware}")

    def test_power_is_part_of_the_snapshot(self):
        snapshot = RobotState("r").snapshot()
        self.assertIn("power", snapshot.to_dict())


class TestRoverDoesNotDependOnRobotX(unittest.TestCase):
    def test_state_package_does_not_import_the_backend_link(self):
        """State is the Rover's own domain. If it imported the communication
        package the robot could not be reasoned about without a backend."""

        for module in ("robot_state.py", "telemetry.py"):
            imports = imported_modules(REPO_ROOT / "robotx" / "state" / module)
            offenders = [n for n in imports if n.startswith("robotx.communication")]
            self.assertEqual(offenders, [], f"{module} imports {offenders}")

    def test_agent_ticks_with_no_backend_configured(self):
        from robotx.application.agent import RobotAgent

        agent = RobotAgent(headless_settings())
        snapshot = agent.tick()
        self.assertEqual(snapshot.robot_id, "test-rover")
        self.assertIs(snapshot.communication.backend, BackendLinkStatus.DISABLED)
        self.assertIsNone(agent.backend)

    def test_there_is_exactly_one_motor_authority_path(self):
        """The direct-GPIO controller is gone. The Pi produces MotionIntent and
        nothing else actuates; the ESP32 will own motors when it exists."""

        self.assertFalse((REPO_ROOT / "robotx" / "control" / "robot_controller.py").exists())
        # And no module outside the bench scripts drives a motor.
        for source in (REPO_ROOT / "robotx").rglob("*.py"):
            imports = imported_modules(source)
            self.assertNotIn(
                "robotx.hardware.motors", imports, f"{source} imports a motor driver"
            )


if __name__ == "__main__":
    unittest.main()
