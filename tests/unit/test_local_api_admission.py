"""H1 + H2 -- what the local HTTP API may admit, and where it listens.

H1: with the backend link enabled (`ROBOTX_SOCKET_ENABLED=1`) RobotX admits
missions, so `POST /mission/start` and `POST /mission/resume` answer 409 before
anything changes -- including after the Y4 backend-loss pause, and while an
engine mission is held. In bench mode (link disabled) both work as before.

H2: the API is served on `ROBOTX_API_HOST`:`ROBOTX_API_PORT` by the one
documented launch path (`python -m robotx.application` -> `main.run`), loopback
by default. The real bind is exercised in tests/integration/test_local_api_bind.py.

The H1 tests call the real FastAPI handlers through httpx's in-process ASGI
transport against a real `RobotAgent`; the backend is the unit suite's FakeSio.
"""

from __future__ import annotations

import asyncio
import logging
import runpy
import time
import unittest
from pathlib import Path
from unittest import mock

import httpx

import robotx.application.main as api
from robotx.application.agent import RobotAgent
from robotx.communication.backend_link import BackendConfig, BackendLink
from robotx.communication.engine import OfferDecision, offer_to_mission
from robotx.config.settings import Settings
from robotx.state.robot_state import Esp32LinkStatus, OperatingMode
from tests.fixtures import engine as fx
from tests.unit.test_agent import HERE, NORTH, FakeGPS, FakePerception, headless_settings
from tests.unit.test_backend_link import FakeSio, MemoryTokenStore, connect_and_auth, state_with_fix
from tests.unit.test_engine_offer import engine_link, fire

REPO = Path(__file__).resolve().parents[2]
ENGINE = {"ROBOTX_SOCKET_ENABLED": "1", "ROBOTX_SOCKET_SERVER_URL": "https://backend.example"}
ROUTE = {"waypoints": [{"lat": NORTH[0], "lon": NORTH[1]}]}


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL)


async def call(agent, method, path, body=None):
    """One request through the real route handlers, in-process."""

    api.state.agent = agent
    try:
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=api.app), base_url="http://pi") as client:
            response = await client.request(method, path, json=body)
    finally:
        api.state.agent = None
    return response.status_code, response.json()


def fingerprint(agent):
    """Everything a refused request must leave exactly as it was."""

    snapshot = agent.state.snapshot()
    return {
        "mode": snapshot.mode,
        "intent": (snapshot.motion_intent.command, snapshot.motion_intent.reason),
        "mission": None if snapshot.mission is None else snapshot.mission.to_dict(),
        "navigator_route": list(agent.navigator.planner._route),
        "mission_route": list(agent._mission_route),
    }


def driving_agent(**env):
    agent = RobotAgent(headless_settings(**env))
    agent.gps = FakeGPS()
    agent.perception = FakePerception()
    agent.gps.set_fix(*HERE, speed=1.0, track=0.0)
    agent.perception.set_clear()
    agent.tick()
    return agent


class H1EngineModeRefusesLocalAdmission(unittest.TestCase):
    """Tests 1 and 2: the link enabled (configured, not merely connected)."""

    def test_1_start_is_409_and_changes_nothing(self):
        agent = driving_agent(**ENGINE)
        before = fingerprint(agent)
        status, body = asyncio.run(call(agent, "POST", "/mission/start", ROUTE))
        self.assertEqual((status, body.get("detail")), (409, api.ENGINE_MODE_REFUSAL))
        self.assertEqual(fingerprint(agent), before)
        agent.tick()
        self.assertIs(agent.state.snapshot().mode, OperatingMode.IDLE)
        self.assertTrue(agent.state.snapshot().motion_intent.command.value != "FORWARD")

    def test_2_resume_is_409_and_a_paused_mission_stays_paused(self):
        agent = driving_agent(**ENGINE)
        agent.start_mission([NORTH])  # the agent's own API: a mission to have paused
        agent.pause_mission("operator pause")
        before = fingerprint(agent)
        status, body = asyncio.run(call(agent, "POST", "/mission/resume"))
        self.assertEqual((status, body.get("detail")), (409, api.ENGINE_MODE_REFUSAL))
        self.assertEqual(fingerprint(agent), before)
        agent.tick()
        self.assertIs(agent.state.snapshot().mode, OperatingMode.PAUSED)
        self.assertTrue(agent.state.snapshot().motion_intent.is_stop)

    def test_the_refusal_does_not_depend_on_the_link_being_up(self):
        # Link enabled but never started (no BackendLink at all): still engine mode.
        agent = driving_agent(**ENGINE)
        self.assertIsNone(agent.backend)
        self.assertEqual(asyncio.run(call(agent, "POST", "/mission/start", ROUTE))[0], 409)

    def test_the_routes_that_only_reduce_motion_are_unchanged(self):
        agent = driving_agent(**ENGINE)
        agent.start_mission([NORTH])
        for path, mode in (("/mission/pause", OperatingMode.PAUSED), ("/mission/stop", OperatingMode.STOPPED),
                           ("/safety/estop", OperatingMode.STOPPED)):
            with self.subTest(path=path):
                status, _ = asyncio.run(call(agent, "POST", path))
                self.assertEqual(status, 200)
                self.assertIs(agent.state.snapshot().mode, mode)

    def test_a_malformed_start_is_still_422_not_409(self):
        # Body validation is FastAPI's, ahead of the handler: unchanged by H1.
        agent = driving_agent(**ENGINE)
        self.assertEqual(asyncio.run(call(agent, "POST", "/mission/start", {"waypoints": []}))[0], 422)


class H1Y4PauseCannotBeUndoneLocally(unittest.TestCase):
    """Tests 3 and 4 (the J1 reproduction): the real agent loop, a real
    BackendLink, a real disconnect and the real Y4 loss policy; then the local
    routes, while the backend stays unreachable."""

    GRACE_S = 0.3

    async def paused_by_y4(self):
        agent = driving_agent(**ENGINE)
        agent.start_mission([NORTH])
        agent.tick()
        sio = FakeSio()
        link = BackendLink(
            BackendConfig(enabled=True, robot_id="robotx-pi", pairing_code="123456", loss_grace_s=self.GRACE_S),
            agent.state, agent, client_factory=lambda _cfg: sio,
            token_store=MemoryTokenStore(token="tok"), agent_alive=agent.is_alive,
        )
        agent.backend = link
        self.assertTrue(await connect_and_auth(link))
        agent._running = True
        loop = asyncio.create_task(agent._loop())
        await asyncio.sleep(0.15)
        self.assertIs(agent.state.snapshot().mode, OperatingMode.AUTO)
        await sio.drop()
        deadline = time.monotonic() + self.GRACE_S + 1.0
        while time.monotonic() < deadline and agent.state.snapshot().mode is not OperatingMode.PAUSED:
            await asyncio.sleep(0.01)
        self.assertIs(agent.state.snapshot().mode, OperatingMode.PAUSED, "Y4 never paused")
        self.assertFalse(link.describe()["authenticated"])
        return agent, link, loop

    async def refused_and_still_paused(self, method, path, body=None):
        agent, link, loop = await self.paused_by_y4()
        try:
            before = fingerprint(agent)
            status, response = await call(agent, method, path, body)
            await asyncio.sleep(0.5)  # five agent ticks, the backend still down
            return status, response, before, fingerprint(agent), agent.state.snapshot(), link
        finally:
            loop.cancel()
            try:
                await loop
            except (asyncio.CancelledError, Exception):
                pass

    def check(self, result):
        status, response, before, after, snapshot, link = result
        self.assertEqual((status, response.get("detail")), (409, api.ENGINE_MODE_REFUSAL))
        self.assertEqual(after, before)
        self.assertIs(snapshot.mode, OperatingMode.PAUSED)
        self.assertTrue(snapshot.motion_intent.is_stop)
        self.assertFalse(link.describe()["authenticated"])

    def test_3_resume_after_the_y4_pause_is_409_and_the_robot_stays_paused(self):
        self.check(asyncio.run(self.refused_and_still_paused("POST", "/mission/resume")))

    def test_4_start_after_the_y4_pause_is_409_and_the_robot_stays_paused(self):
        self.check(asyncio.run(self.refused_and_still_paused("POST", "/mission/start", ROUTE)))


class H1HeldEngineMissionIsNotTakenOver(unittest.TestCase):
    """Test 5 (the J2 reproduction): a signed OFFER accepted through the real
    link puts a real engine mission on the real agent and a held commitment in
    the store; a local start must not touch either, nor the heartbeat that
    renews the lease."""

    def accepted_engine_mission(self):
        agent = RobotAgent(Settings.from_env({
            **ENGINE, "ROBOTX_ROBOT_ID": fx.ROBOT_ID, "ROBOTX_CAMERA_ENABLED": "0",
            "ROBOTX_PERCEPTION_ENABLED": "0", "ROBOTX_GPS_ENABLED": "0", "ROBOTX_LOG_LEVEL": "CRITICAL",
        }))
        # SIMULATED readiness only: the Rover's own offer assessment is not what
        # this test is about, so it accepts the offer as the real one would on a
        # ready Rover.
        agent.assess_offer = lambda offer: OfferDecision.accept(offer_to_mission(offer))
        fixed = state_with_fix(fx.ROBOT_ID).snapshot()
        agent.state.update_gps(fixed.gps, fixed.position)
        agent.state.update_communication(esp32=Esp32LinkStatus.UP)
        link, sio = engine_link(agent=agent, state=agent.state, max_position_age_s=30.0)
        agent.backend = link
        out = fire(link, sio, fx.envelope())
        self.assertIn("OFFER_ACCEPT", [name for name, _ in out])
        return agent, link, sio

    def test_5_start_over_a_held_engine_mission_is_409_and_nothing_is_taken_over(self):
        agent, link, sio = self.accepted_engine_mission()
        mission = agent.state.snapshot().mission
        self.assertIsNotNone(mission)
        self.assertIs(agent.state.snapshot().mode, OperatingMode.AUTO)
        held_before = link.commitments.held()
        asyncio.run(link._publish_heartbeat())
        heartbeat_before = sio.events_named("HEARTBEAT")[-1]
        self.assertEqual(heartbeat_before, {"commitmentId": "c-7f3a", "fence": "42"})
        before = fingerprint(agent)

        status, body = asyncio.run(call(agent, "POST", "/mission/start", ROUTE))

        self.assertEqual((status, body.get("detail")), (409, api.ENGINE_MODE_REFUSAL))
        self.assertEqual(fingerprint(agent), before)  # mission, mode and both routes
        self.assertEqual(agent.navigator.planner._route, list(mission.mission.path_to_pickup))
        self.assertEqual(link.commitments.held(), held_before)
        asyncio.run(link._publish_heartbeat())
        self.assertEqual(sio.events_named("HEARTBEAT")[-1], heartbeat_before)


class H1BenchModeUnchanged(unittest.TestCase):
    """Tests 6 and 7: with the link disabled the bench routes behave as before."""

    def test_6_start_works_in_bench_mode(self):
        agent = driving_agent()
        self.assertFalse(agent.settings.socket_enabled)
        status, body = asyncio.run(call(agent, "POST", "/mission/start", ROUTE))
        self.assertEqual((status, body), (200, {"mode": "AUTO", "waypoints": 1}))
        self.assertEqual(agent.navigator.planner._route, [NORTH])
        agent.tick()
        self.assertEqual(agent.state.snapshot().motion_intent.command.value, "FORWARD")

    def test_7_resume_works_in_bench_mode(self):
        agent = driving_agent()
        agent.start_mission([NORTH])
        agent.pause_mission("operator pause")
        status, body = asyncio.run(call(agent, "POST", "/mission/resume"))
        self.assertEqual((status, body), (200, {"mode": "AUTO"}))

    def test_7b_resume_with_nothing_to_resume_is_still_its_own_409(self):
        agent = driving_agent()
        status, body = asyncio.run(call(agent, "POST", "/mission/resume"))
        self.assertEqual(status, 409)
        self.assertNotEqual(body["detail"], api.ENGINE_MODE_REFUSAL)

    def test_bench_start_is_still_subject_to_the_emergency_stop(self):
        agent = driving_agent()
        agent.emergency_stop("test")
        self.assertEqual(asyncio.run(call(agent, "POST", "/mission/start", ROUTE))[0], 200)
        agent.tick()
        self.assertTrue(agent.state.snapshot().motion_intent.is_stop)


class H2BindConfiguration(unittest.TestCase):
    def setUp(self):
        # `run` configures root logging for the process it launches; not this one.
        patcher = mock.patch.object(api, "setup_logging")
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_the_default_host_is_loopback_and_the_port_8000(self):
        for settings in (Settings(), Settings.from_env({})):
            self.assertEqual((settings.api_host, settings.api_port), ("127.0.0.1", 8000))

    def test_run_hands_uvicorn_the_configured_host_and_port(self):
        cases = (({}, ("127.0.0.1", 8000)),
                 ({"ROBOTX_API_HOST": "127.0.0.5", "ROBOTX_API_PORT": "8123"}, ("127.0.0.5", 8123)),
                 ({"ROBOTX_API_HOST": "192.0.2.10", "ROBOTX_API_PORT": "9000"}, ("192.0.2.10", 9000)))
        for env, (host, port) in cases:
            with self.subTest(env=env), mock.patch("uvicorn.run") as served:
                api.run(Settings.from_env(env))
                served.assert_called_once_with(api.app, host=host, port=port)

    def test_the_documented_launch_module_is_main_run_with_the_process_settings(self):
        with mock.patch("uvicorn.run") as served:
            runpy.run_module("robotx.application", run_name="__main__")
        served.assert_called_once_with(api.app, host=api.SETTINGS.api_host, port=api.SETTINGS.api_port)

    def test_a_network_facing_host_is_served_but_logged_as_exposed(self):
        with mock.patch("uvicorn.run"), self.assertLogs("robotx.application.main", level="WARNING") as logs:
            api.run(Settings.from_env({"ROBOTX_API_HOST": "0.0.0.0"}))
        self.assertTrue(any("api.network_exposed" in line for line in logs.output))
        with mock.patch("uvicorn.run"), mock.patch.object(api, "log_event") as logged:
            api.run(Settings.from_env({}))
        self.assertNotIn("api.network_exposed", [c.args[1] for c in logged.call_args_list])

    def test_the_documented_launch_no_longer_forces_every_interface(self):
        for name in ("README.md", "TEST_README.md", "robotx/application/main.py"):
            with self.subTest(name=name):
                text = (REPO / name).read_text(encoding="utf-8")
                self.assertNotIn("--host 0.0.0.0", text)
                self.assertIn("python -m robotx.application", text)
        env_example = (REPO / ".env.example").read_text(encoding="utf-8")
        self.assertIn("\nROBOTX_API_HOST=127.0.0.1", env_example.replace("\r\n", "\n"))


if __name__ == "__main__":
    unittest.main()
