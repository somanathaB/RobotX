"""Opt-in operator access for the routes that release a stop or advance a mission.

Requests go through the real FastAPI route handlers (httpx's in-process ASGI
transport) against a real `RobotAgent` over the SIMULATED ESP32 port of
`test_obstacle_recovery.Rig`. No socket, serial port or hardware is touched.
"""

from __future__ import annotations

import asyncio
import dataclasses
import logging
import secrets
import unittest

import httpx

import robotx.application.main as api
from robotx.application.injection import token_digest
from robotx.application.operator_access import OperatorAccess
from robotx.config.settings import Settings
from tests.unit.test_obstacle_recovery import Rig


TOKEN = secrets.token_hex(32)
GUARDED = ("/safety/clear", "/controller/reset", "/mission/idle", "/mission/custody")
STOPS = ("/safety/estop", "/mission/stop", "/mission/pause")
CROSS_SITE = {"Origin": "http://evil.example", "Content-Type": "text/plain"}


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


def rig():
    r = Rig()
    r.agent.settings = dataclasses.replace(r.agent.settings, custody_confirmation="operator")
    return r


def call(agent, access, path, *, headers=None, json=None, host="127.0.0.1:8000"):
    async def go():
        api.state.agent, api.state.operator_access = agent, access
        try:
            transport = httpx.ASGITransport(app=api.app, client=("127.0.0.1", 40000))
            async with httpx.AsyncClient(transport=transport, base_url=f"http://{host}") as c:
                r = await c.post(path, headers=headers or {}, json=json)
                return r.status_code, r.json()
        finally:
            api.state.agent = api.state.operator_access = None

    return asyncio.run(go())


def bearer(token=TOKEN):
    return {"Authorization": f"Bearer {token}"}


BROWSER_HEADERS = (
    CROSS_SITE,                                                    # cross-site "simple" POST, no preflight
    {"Origin": "http://attacker.example:8000"},                    # DNS rebinding: same-origin to the page
    {"Origin": "null"},                                            # sandboxed iframe / file:// page
    {"Origin": "http://127.0.0.1:8000"},                           # even a page served from this Pi
)


class TestDefaultIsUnchanged(unittest.TestCase):
    def test_no_setting_means_no_token(self):
        self.assertIsNone(Settings.from_env({}).operator_token_sha256)
        self.assertFalse(OperatorAccess.from_settings(Settings.from_env({})).required)
        self.assertEqual(Settings.from_env({}).public_summary()["operator_token_sha256"], "UNSET")

    def test_unset_a_non_browser_caller_reaches_every_route_as_before(self):
        for access in (None, OperatorAccess(None)):
            r = rig()
            r.agent.emergency_stop("unit")
            status, reply = call(r.agent, access, "/safety/clear", host="pi")
            self.assertEqual((status, reply["was_engaged"]), (200, True))
            self.assertEqual(call(r.agent, access, "/mission/idle", host="pi")[0], 200)
            self.assertEqual(call(r.agent, access, "/controller/reset", host="pi")[1]["reset_status"], "REQUESTED")
            self.assertEqual(call(r.agent, access, "/mission/custody", host="pi", json={"kind": "ACQUIRED"})[0], 409)


class TestBrowserRequestsAlwaysRefused(unittest.TestCase):
    """The unconditional rule: any Origin header on a release route is a 403."""

    def assert_refused_everywhere(self, access, extra=None):
        for headers in BROWSER_HEADERS:
            r = rig()
            r.agent.emergency_stop("unit")
            for path in GUARDED:
                with self.subTest(path=path, headers=headers):
                    status, reply = call(r.agent, access, path, headers=dict(headers, **(extra or {})),
                                         json={"kind": "ACQUIRED"})
                    self.assertEqual((status, reply["detail"]), (403, api.BROWSER_REFUSAL))
            self.assertTrue(r.agent.emergency_stopped, "a browser page released the e-stop")
            self.assertEqual(r.port.command_names().count("RESET"), 0)

    def test_with_no_operator_token_configured(self):
        for access in (None, OperatorAccess(None)):
            self.assert_refused_everywhere(access)

    def test_with_the_operator_token_configured_even_carrying_the_token(self):
        self.assert_refused_everywhere(OperatorAccess(token_digest(TOKEN)), extra=bearer())

    def test_with_a_misconfigured_token(self):
        self.assert_refused_everywhere(OperatorAccess("ABC"))

    def test_stop_routes_still_accept_browsers(self):
        for access in (None, OperatorAccess(token_digest(TOKEN))):
            r = rig()
            for path in STOPS:
                with self.subTest(path=path):
                    self.assertEqual(call(r.agent, access, path, headers=CROSS_SITE)[0], 200)
            self.assertTrue(r.agent.emergency_stopped)

    def test_the_documented_curl_procedure_is_unaffected(self):
        # S=http://127.0.0.1:8000; curl -X POST $S/safety/clear -H "Authorization: Bearer $OPERATOR_TOKEN"
        r = rig()
        r.agent.emergency_stop("unit")
        curl = {"User-Agent": "curl/8.14.1", "Accept": "*/*"}
        status, reply = call(r.agent, OperatorAccess(token_digest(TOKEN)), "/safety/clear",
                             headers=dict(curl, **bearer()))
        self.assertEqual((status, reply["was_engaged"]), (200, True))
        r.agent.emergency_stop("unit")
        self.assertEqual(call(r.agent, None, "/safety/clear", headers=curl)[0], 200)


class TestGuardedRoutes(unittest.TestCase):
    def setUp(self):
        self.access = OperatorAccess(token_digest(TOKEN))

    def test_the_token_is_required(self):
        r = rig()
        for path in GUARDED:
            for headers in ({}, bearer("x" * 64), bearer(TOKEN[:20]), {"Authorization": TOKEN}):
                with self.subTest(path=path, headers=headers):
                    status, _ = call(r.agent, self.access, path, headers=headers, json={"kind": "ACQUIRED"})
                    self.assertEqual(status, 401)

    def test_a_cross_site_page_can_no_longer_clear_the_estop(self):
        r = rig()
        r.agent.emergency_stop("unit")
        for headers in (CROSS_SITE, dict(CROSS_SITE, **bearer())):
            self.assertEqual(call(r.agent, self.access, "/safety/clear", headers=headers)[0], 403)
        self.assertTrue(r.agent.emergency_stopped, "a browser page released the only stop latch")

    def test_a_rebinding_host_is_refused_even_with_the_token(self):
        r = rig()
        r.agent.emergency_stop("unit")
        self.assertEqual(call(r.agent, self.access, "/safety/clear", headers=bearer(),
                              host="attacker.example:8000")[0], 403)
        self.assertTrue(r.agent.emergency_stopped)

    def test_the_operator_with_the_token_works_as_before(self):
        for host in ("127.0.0.1:8000", "localhost:8000", "[::1]:8000"):
            with self.subTest(host=host):
                r = rig()
                r.agent.emergency_stop("unit")
                status, reply = call(r.agent, self.access, "/safety/clear", headers=bearer(), host=host)
                self.assertEqual((status, reply["was_engaged"]), (200, True))
                self.assertEqual(call(r.agent, self.access, "/mission/idle", headers=bearer(), host=host)[0], 200)
                # Custody keeps its own rules once past the guard (no mission: 409).
                self.assertEqual(call(r.agent, self.access, "/mission/custody", headers=bearer(), host=host,
                                      json={"kind": "ACQUIRED"})[0], 409)

    def test_stop_routes_are_never_guarded(self):
        for access in (self.access, OperatorAccess("not-a-digest")):
            r = rig()
            for path in STOPS:
                with self.subTest(path=path):
                    self.assertEqual(call(r.agent, access, path, headers=CROSS_SITE, host="anything")[0], 200)
            self.assertTrue(r.agent.emergency_stopped)

    def test_a_malformed_digest_fails_closed_and_the_rover_stays_stopped(self):
        r = rig()
        r.agent.emergency_stop("unit")
        access = OperatorAccess("ABC")
        self.assertIsNotNone(access.problem)
        for path in GUARDED:
            self.assertEqual(call(r.agent, access, path, headers=bearer(), json={"kind": "ACQUIRED"})[0], 503)
        self.assertTrue(r.agent.emergency_stopped)

    def test_the_token_is_never_logged_or_echoed(self):
        r = rig()
        logging.getLogger("robotx").setLevel(logging.INFO)
        self.addCleanup(logging.getLogger("robotx").setLevel, logging.CRITICAL + 1)
        with self.assertLogs("robotx.application.main", level="WARNING") as logs:
            _, reply = call(r.agent, self.access, "/safety/clear", headers=bearer("y" * 64))
        for text in ("\n".join(logs.output), str(reply)):
            self.assertNotIn(TOKEN, text)
            self.assertNotIn(token_digest(TOKEN), text)
            self.assertNotIn("y" * 64, text)

    def test_settings_read_and_hide_the_digest(self):
        s = Settings.from_env({"ROBOTX_OPERATOR_TOKEN_SHA256": token_digest(TOKEN)})
        self.assertTrue(OperatorAccess.from_settings(s).required)
        self.assertEqual(s.public_summary()["operator_token_sha256"], "SET")


if __name__ == "__main__":
    unittest.main()
