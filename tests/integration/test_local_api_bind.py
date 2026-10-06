"""H2 -- the local API listens where ROBOTX_API_HOST / ROBOTX_API_PORT say, and
nowhere else; by default on loopback only.

Each test launches the documented command (`python -m robotx.application`) as a
real process with every piece of hardware off, waits for `/health`, and then
probes which addresses actually accept a TCP connection. A server bound to
every interface would accept on all of them; one bound as configured accepts
only on its own address.
"""

from __future__ import annotations

import contextlib
import os
import socket
import subprocess
import sys
import time
import unittest
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def accepts(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=5.0):
            return True
    except OSError:
        return False


def non_loopback_ipv4():
    """This machine's outward-facing IPv4 address, or None. Sends nothing:
    connecting a UDP socket only selects the route."""

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("192.0.2.1", 9))  # TEST-NET-1, never routed anywhere real
            address = s.getsockname()[0]
    except OSError:
        return None
    return None if address.startswith("127.") or address == "0.0.0.0" else address


def bindable(host: str) -> bool:
    try:
        with socket.socket() as s:
            s.bind((host, 0))
        return True
    except OSError:
        return False


@contextlib.contextmanager
def served(**api_env):
    env = {k: v for k, v in os.environ.items() if not k.startswith("ROBOTX_")}
    env.update(
        ROBOTX_ROBOT_ID="bind-test", ROBOTX_CAMERA_ENABLED="0", ROBOTX_PERCEPTION_ENABLED="0",
        ROBOTX_GPS_ENABLED="0", ROBOTX_LOG_LEVEL="WARNING", **api_env,
    )
    process = subprocess.Popen([sys.executable, "-m", "robotx.application"], cwd=str(REPO), env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        yield process
    finally:
        process.terminate()
        try:
            process.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()


def wait_healthy(host: str, port: int, process, limit_s: float = 30.0) -> int:
    deadline = time.monotonic() + limit_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise AssertionError(f"the API process exited early ({process.returncode})")
        try:
            return urllib.request.urlopen(f"http://{host}:{port}/health", timeout=1.0).status
        except OSError:
            time.sleep(0.2)
    raise AssertionError(f"no /health on {host}:{port} within {limit_s:.0f} s")


class TestLocalApiBind(unittest.TestCase):
    def test_by_default_it_listens_on_loopback_only_on_the_configured_port(self):
        port = free_port()
        with served(ROBOTX_API_PORT=str(port)) as process:  # ROBOTX_API_HOST unset: the default
            self.assertEqual(wait_healthy("127.0.0.1", port, process), 200)
            if bindable("127.0.0.2"):
                # Another loopback address: accepted only by a wildcard bind.
                self.assertFalse(accepts("127.0.0.2", port), "listening beyond 127.0.0.1")
            lan = non_loopback_ipv4()
            if lan is not None:
                self.assertFalse(accepts(lan, port), f"reachable on this machine's network address {lan}")

    def test_a_configured_host_and_port_are_the_ones_served(self):
        if not bindable("127.0.0.2"):
            self.skipTest("this host cannot bind 127.0.0.2")
        port = free_port()
        with served(ROBOTX_API_HOST="127.0.0.2", ROBOTX_API_PORT=str(port)) as process:
            self.assertEqual(wait_healthy("127.0.0.2", port, process), 200)
            self.assertFalse(accepts("127.0.0.1", port), "the default host was bound instead of the configured one")


if __name__ == "__main__":
    unittest.main()
