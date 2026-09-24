"""Structural rules for the ESP32 boundary, enforced on the import graph.

1. The ESP32 package is its own boundary: it never imports the backend link,
   Socket.IO, GPIO or motor drivers, and the backend package never imports it.
2. Exactly one module outside the package wires the link in: the agent.
3. The link can only ever encode PING, STOP and DRIVE.
"""

import ast
import pathlib
import unittest

from robotx.esp32 import protocol

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
ESP32_DIR = REPO_ROOT / "robotx" / "esp32"


def imports_of(path):
    names = set()
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
    return names


class TestBoundary(unittest.TestCase):
    def test_esp32_package_imports_no_backend_gpio_or_motor_code(self):
        forbidden_roots = ("socketio", "RPi", "gpiozero", "lgpio")
        forbidden_modules = ("robotx.communication", "robotx.hardware", "robotx.application",
                             "robotx.navigation", "robotx.perception", "robotx.mission")
        for path in sorted(ESP32_DIR.glob("*.py")):
            for name in imports_of(path):
                self.assertNotIn(name.split(".")[0], forbidden_roots, f"{path.name}: {name}")
                self.assertFalse(name.startswith(forbidden_modules), f"{path.name}: {name}")

    def test_esp32_package_does_not_import_the_state_package(self):
        # robot_state imports robotx.esp32.state; the reverse would be a cycle.
        for path in sorted(ESP32_DIR.glob("*.py")):
            offenders = [n for n in imports_of(path) if n.startswith("robotx.state")]
            self.assertEqual(offenders, [], path.name)

    def test_backend_package_does_not_import_the_esp32_link(self):
        for path in sorted((REPO_ROOT / "robotx" / "communication").glob("*.py")):
            offenders = [n for n in imports_of(path) if n.startswith("robotx.esp32")]
            self.assertEqual(offenders, [], f"{path.name} imports {offenders}")

    def test_only_the_agent_wires_the_link_in(self):
        users = []
        for path in (REPO_ROOT / "robotx").rglob("*.py"):
            if ESP32_DIR in path.parents:
                continue
            if any(n.startswith("robotx.esp32.link") for n in imports_of(path)):
                users.append(path.relative_to(REPO_ROOT).as_posix())
        self.assertEqual(users, ["robotx/application/agent.py"])

    def test_serial_is_imported_only_by_the_transport(self):
        for path in sorted(ESP32_DIR.glob("*.py")):
            if path.name == "transport.py":
                continue
            self.assertNotIn("serial", {n.split(".")[0] for n in imports_of(path)}, path.name)

    def test_only_ping_stop_and_drive_can_be_encoded(self):
        self.assertEqual(set(protocol.ALLOWED_COMMANDS), {"PING", "STOP", "DRIVE"})
        for cmd in ("MOVE", "MOTORTEST", "RESET"):
            with self.assertRaises(protocol.CommandError):
                protocol.encode_command(1, cmd)


if __name__ == "__main__":
    unittest.main()
