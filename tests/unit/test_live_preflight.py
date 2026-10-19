"""The read-only live preflight's added checks: reboot latch, host health, readiness, gates.

Pure: `preflight.evaluate` over observation dicts shaped like the running
agent's GET responses. Nothing here contacts the agent, a port or a service.
"""

from __future__ import annotations

import copy
import unittest

from tests.mission_injection import preflight
from tests.unit.test_mission_injection import all_green_observation, live_like_observation

R = preflight.Result


def checks(observed):
    return {c.name: c for c in preflight.evaluate(observed).checks}


class TestRebootLatch(unittest.TestCase):
    def test_a_latched_reboot_fails_and_blocks(self):
        o = all_green_observation()
        o["state"]["controller"].update(reboot_latched=True, reboot_count=2)
        report = preflight.evaluate(o)
        self.assertIs(checks(o)["esp32_reboot_latch"].result, R.FAIL)
        self.assertIn("esp32_reboot_latch", {c.name for c in report.blockers()})

    def test_unreported_is_not_tested_never_a_pass(self):
        o = all_green_observation()
        del o["state"]["controller"]["reboot_latched"]
        self.assertIs(checks(o)["esp32_reboot_latch"].result, R.NOT_TESTED)
        self.assertEqual(preflight.evaluate(o).verdict, "BLOCKED")


class TestHostHealth(unittest.TestCase):
    def with_system(self, **system):
        o = all_green_observation()
        o["health"]["health"] = {"system": {"cpu_temp_c": 45.0, "disk_used_percent": 20.0, **system}}
        return o

    def test_healthy_host_passes(self):
        self.assertIs(checks(self.with_system())["pi_host_health"].result, R.PASS)

    def test_hot_or_full_fails(self):
        self.assertIs(checks(self.with_system(cpu_temp_c=82.0))["pi_host_health"].result, R.FAIL)
        self.assertIs(checks(self.with_system(disk_used_percent=95.0))["pi_host_health"].result, R.FAIL)

    def test_throttling_now_blocks_past_throttling_is_reported(self):
        o = all_green_observation()
        o["throttled"] = "throttled=0x50005"           # under-voltage + throttled, now and since boot
        c = checks(o)["pi_power_throttling"]
        self.assertIs(c.result, R.FAIL)
        self.assertTrue(c.required)
        o["throttled"] = "throttled=0x50000"           # since boot only
        self.assertIs(checks(o)["pi_power_throttling"].result, R.PASS)
        o["throttled"] = None                          # no vcgencmd
        c = checks(o)["pi_power_throttling"]
        self.assertIs(c.result, R.NOT_TESTED)
        self.assertFalse(c.required)


class TestReadinessAndGates(unittest.TestCase):
    def test_agent_readiness_blockers_block(self):
        o = all_green_observation()
        o["readiness"] = {"blockers": ["ESTOP_LATCHED"], "estop_reason": "x", "mission_lease_renewable": False}
        self.assertIs(checks(o)["agent_readiness"].result, R.BLOCKED)
        o["readiness"]["blockers"] = []
        self.assertIs(checks(o)["agent_readiness"].result, R.PASS)

    def test_an_agent_without_the_route_is_not_tested(self):
        o = all_green_observation()
        o["readiness_http_status"] = 404
        c = checks(o)["agent_readiness"]
        self.assertIs(c.result, R.NOT_TESTED)
        self.assertFalse(c.required)

    def test_a_forward_gate_blocks(self):
        live = checks(live_like_observation())
        self.assertIs(live["obstacle_gates"].result, R.BLOCKED)
        self.assertIs(checks(all_green_observation())["obstacle_gates"].result, R.PASS)

    def test_an_interrupted_mission_is_named(self):
        o = copy.deepcopy(all_green_observation())
        o["state"]["mission"] = {"task_id": "T-1", "status": "TO_PICKUP", "interrupted_by": "emergency stop: x"}
        c = checks(o)["mission_state"]
        self.assertIs(c.result, R.BLOCKED)
        self.assertIn("emergency stop: x", c.detail)

    def test_the_client_refuses_anything_but_the_allowlist(self):
        client = preflight.LoopbackClient()
        for path in ("/safety/clear", "/mission/start", "/camera", "/test/mission/inject"):
            with self.assertRaises(ValueError):
                client.get(path)
        with self.assertRaises(ValueError):
            preflight.LoopbackClient("http://192.0.2.1:8000")


class TestReadinessRoute(unittest.TestCase):
    def test_the_route_reports_the_agents_own_checks(self):
        import asyncio

        from tests.unit.test_injection_endpoint import acall, ready_rig

        rig = ready_rig()
        rig.agent.emergency_stop("preflight test")
        status, body = asyncio.run(acall(rig.agent, None, "GET", "/readiness", token=None))
        self.assertEqual(status, 200)
        self.assertEqual(body["blockers"][0], "ESTOP_LATCHED")
        self.assertEqual(body["estop_reason"], "preflight test")
        self.assertFalse(body["mission_lease_renewable"])
        codes = [f["code"] for f in body["faults"]]
        self.assertIn("PI_EMERGENCY_STOP_LATCHED", codes)
        self.assertIsInstance(body["perception_usable"], bool)


if __name__ == "__main__":
    unittest.main()
