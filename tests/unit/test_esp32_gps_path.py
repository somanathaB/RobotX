"""The ESP32 GPS path end to end: frame -> decoder -> link -> GpsReading ->
position -> navigation -> decision -> health and OFFER assessment.

Receive-only and offline. Every link here is built over a `FakePort` with
`transmit_enabled=False`, and each agent test asserts that nothing at all was
written to the port: no PING, no STOP, no DRIVE.

There is no captured real GPS frame yet. The firmware the REAL_* fixtures were
captured from (82f2a8a, 2026-09-25) predates GPS frames, and the deployed
agent has received no bytes from the ESP32 since. The frames below follow the
field names and units the decoder documents (u-blox NAV-PVT: 1e-7 deg, mm,
mm/s, 1e-5 deg, ms). When a real frame is captured, add it beside the REAL_*
lines in tests/fixtures/esp32.py and to REAL_GPS_LINES below.
"""

import logging
import math
import time
import unittest

from robotx.application.agent import RobotAgent
from robotx.config.settings import Settings
from robotx.control.decision import DecisionMaker
from robotx.diagnostics.health import HealthStatus
from robotx.esp32.protocol import Frame, decode_line
from robotx.hardware.gps import GPSStatus
from robotx.localization.esp32_gps import Esp32GpsReader, reading_from_frame
from robotx.localization.position import PositionEstimator
from robotx.navigation.navigator import NavigationConfig, NavigationStatus, Navigator
from robotx.perception.types import FrameMetadata, PerceptionResult, PerceptionStatus
from robotx.state.robot_state import Esp32LinkStatus, OperatingMode
from tests.fixtures import esp32 as fx


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


HERE_E7 = (129032140, 775190770)          # 12.9032140 N, 77.5190770 E
AHEAD_E7 = (129035140, 775190770)         # ~33 m north of HERE


def gps(**overrides):
    """A gps_status OK frame dict with every field the adapter checks."""

    data = {
        "type": "GPS", "uptime_ms": 6252, "gps_status": "OK", "fix_type": 3, "fix_ok": True,
        "siv": 18, "lat_e7": HERE_E7[0], "lon_e7": HERE_E7[1], "hacc_mm": 1500,
        "speed_mm_s": 20, "head_mot_e5": 34358000, "age_ms": 180,
    }
    data.update(overrides)
    return data


def gps_line(**overrides) -> bytes:
    return fx.frame(gps(**overrides))


# Real captured GPS lines go here, byte for byte, CRC included. Empty until one
# is captured: see the module docstring.
REAL_GPS_LINES: tuple = ()


def read(frame, *, received_at=1000.0, now=None, elapsed_s=None, stale_after_s=5.0):
    return reading_from_frame(
        frame, received_at, now=received_at if now is None else now,
        stale_after_s=stale_after_s, elapsed_s=elapsed_s,
    )


class TestDecoder(unittest.TestCase):
    def test_real_captured_gps_lines_decode(self):
        if not REAL_GPS_LINES:
            self.skipTest("no real GPS frame has been captured yet")
        for line in REAL_GPS_LINES:
            self.assertIsInstance(decode_line(line), Frame, line)

    def test_every_documented_status_decodes(self):
        for status in ("NOT_STARTED", "BACKOFF", "NOT_DETECTED", "STALE", "NO_FIX", "OK"):
            result = decode_line(fx.frame({"type": "GPS", "uptime_ms": 1, "gps_status": status}))
            self.assertEqual(getattr(result, "type", None), "GPS", status)

    def test_malformed_gps_frames_are_rejected(self):
        bad = [
            fx.frame({"type": "GPS", "uptime_ms": 1}),                                  # no gps_status
            fx.frame({"type": "GPS", "gps_status": "OK"}),                              # no uptime_ms
            fx.frame({"type": "GPS", "uptime_ms": 1, "gps_status": "ok"}),              # undocumented
            gps_line(lat_e7=12.9),                                                       # float, not e7 int
            gps_line(hacc_mm="1500"),
            gps_line(fix_ok=1),                                                          # int, not bool
            gps_line(age_ms=True),                                                       # bool, not int
            gps_line()[:-6] + b"*0000\n",                                                # bad CRC
            gps_line()[:40] + b"\n",                                                     # truncated
        ]
        for line in bad:
            self.assertNotIsInstance(decode_line(line), Frame, line)

    def test_gps_does_not_disturb_the_other_frame_types_on_a_real_link(self):
        port = fx.FakePort()
        link, clock = fx.make_link(port, transmit_enabled=False)
        link.poll_once()
        port.feed(
            fx.REAL_TELEMETRY + gps_line() + fx.REAL_DIAG_FRONT + gps_line(gps_status="NO_FIX")
            + fx.REAL_DIAG_REAR + fx.REAL_DIAG_SYSTEM + fx.event("I2CSCAN") + fx.error("BAD_CRC")
            + fx.ack(7) + gps_line(lat_e7=1.5) + fx.REAL_TELEMETRY
        )
        link.poll_once()
        c = link.status().controller.counters.to_dict()
        self.assertEqual(
            (c["telemetry"], c["diag"], c["gps"], c["events"], c["esp32_errors"]), (2, 3, 2, 1, 1)
        )
        self.assertEqual(c["rejected_lines"], 1)          # only the float lat_e7 line
        frame, _ = link.gps_frame()
        self.assertEqual(frame["gps_status"], "NO_FIX")   # the latest valid GPS frame
        self.assertEqual(port.commands(), [])


class TestAdapter(unittest.TestCase):
    def test_units_are_the_documented_ones(self):
        r = read(gps(speed_mm_s=1250, head_mot_e5=9000000, siv=11, hacc_mm=2340))
        self.assertIs(r.status, GPSStatus.FIX)
        self.assertAlmostEqual(r.fix.latitude, 12.903214, places=7)
        self.assertAlmostEqual(r.fix.longitude, 77.519077, places=7)
        self.assertAlmostEqual(r.fix.speed_mps, 1.25)
        self.assertAlmostEqual(r.fix.track_deg, 90.0)
        self.assertAlmostEqual(r.fix.h_acc_m, 2.34)
        self.assertEqual((r.fix.satellites, r.fix.fix_type), (11, "3D"))
        self.assertIsNone(r.fix.altitude_m)               # not mapped: not documented
        self.assertAlmostEqual(r.fix.timestamp, 1000.0 - 0.18)

    def test_non_ok_statuses_are_no_fix_even_with_coordinates(self):
        for status in ("NOT_STARTED", "BACKOFF", "NOT_DETECTED", "STALE", "NO_FIX"):
            r = read(gps(gps_status=status))
            self.assertIs(r.status, GPSStatus.NO_FIX, status)
            self.assertIsNone(r.fix)

    def test_an_ok_frame_the_receiver_does_not_vouch_for_is_no_fix(self):
        cases = {
            "fix_ok false": gps(fix_ok=False),
            "fix_ok null": gps(fix_ok=None),
            "fix_ok absent": {k: v for k, v in gps().items() if k != "fix_ok"},
            "fix_type 0": gps(fix_type=0),
            "fix_type 1 (DR only)": gps(fix_type=1),
            "fix_type 5 (time only)": gps(fix_type=5),
            "fix_type null": gps(fix_type=None),
            "lat null": gps(lat_e7=None),
            "lat out of range": gps(lat_e7=900_000_001),
            "lon out of range": gps(lon_e7=-1_800_000_001),
            "null island": gps(lat_e7=0, lon_e7=0),
            "age_ms null": gps(age_ms=None),
            "age_ms negative": gps(age_ms=-1),
            "hacc_mm null": gps(hacc_mm=None),
            "hacc_mm zero": gps(hacc_mm=0),
        }
        for name, frame in cases.items():
            r = read(frame)
            self.assertIs(r.status, GPSStatus.NO_FIX, name)
            self.assertIsNone(r.fix, name)
            self.assertTrue(r.error, name)
            self.assertIsNone(PositionEstimator().update(r), name)

    def test_fix_age_includes_the_receivers_own_age(self):
        r = read(gps(age_ms=4500), received_at=1000.0, now=1000.6)
        self.assertIs(r.status, GPSStatus.STALE)          # 0.6 s since frame + 4.5 s old PVT

    def test_staleness_follows_the_monotonic_clock_not_the_wall(self):
        # The wall clock stepped back an hour after the frame arrived (NTP on an
        # RTC-less Pi). Six monotonic seconds have passed: the fix is stale.
        r = read(gps(), received_at=5000.0, now=5000.0 - 3600, elapsed_s=6.0)
        self.assertIs(r.status, GPSStatus.STALE)
        # And a forward wall step does not falsely stale a fresh frame.
        r = read(gps(), received_at=5000.0, now=5000.0 + 3600, elapsed_s=0.2)
        self.assertIs(r.status, GPSStatus.FIX)

    def test_reader_uses_the_links_monotonic_clock(self):
        port = fx.FakePort()
        wall = fx.FakeClock(1_800_000_000.0)
        link, clock = fx.make_link(port, wall=wall, transmit_enabled=False)
        reader = Esp32GpsReader(link, stale_after_s=5.0, wall=wall)
        self.assertIs(reader.get_reading().status, GPSStatus.STARTING)
        link.poll_once()
        port.feed(gps_line())
        link.poll_once()
        self.assertIs(reader.get_reading().status, GPSStatus.FIX)
        wall.advance(-3600)
        clock.advance(6.0)
        self.assertIs(reader.get_reading().status, GPSStatus.STALE)


class TestNavigationAccuracyGate(unittest.TestCase):
    def position(self, hacc_mm, at=HERE_E7):
        return PositionEstimator().update(read(gps(hacc_mm=hacc_mm, lat_e7=at[0], lon_e7=at[1])))

    def test_an_inaccurate_fix_is_not_navigated_on(self):
        nav = Navigator(NavigationConfig(max_h_acc_m=5.0))
        nav.set_route([(AHEAD_E7[0] / 1e7, AHEAD_E7[1] / 1e7)])
        state = nav.update(self.position(12_000))
        self.assertIs(state.status, NavigationStatus.NO_POSITION)
        self.assertIn("12.0 m", state.position_issue)
        intent = DecisionMaker().decide(mission_active=True, navigation=state, perception=clear())
        self.assertTrue(intent.is_stop)
        self.assertEqual(intent.reason, "GPS fix not accurate enough to navigate")

        state = nav.update(self.position(1_500))
        self.assertIs(state.status, NavigationStatus.NAVIGATING)
        self.assertIsNone(state.position_issue)

    def test_an_inaccurate_fix_cannot_declare_arrival(self):
        nav = Navigator(NavigationConfig(max_h_acc_m=5.0, waypoint_arrival_m=8.0))
        nav.set_route([(HERE_E7[0] / 1e7, HERE_E7[1] / 1e7)])
        self.assertIs(nav.update(self.position(20_000)).status, NavigationStatus.NO_POSITION)
        self.assertIs(nav.update(self.position(1_000)).status, NavigationStatus.ARRIVED)

    def test_a_fix_with_no_stated_accuracy_is_not_judged(self):
        # NMEA states no accuracy; the gate has nothing to compare.
        from robotx.hardware.gps import GpsFix, GpsReading

        nmea = PositionEstimator().update(
            GpsReading(GPSStatus.FIX, fix=GpsFix(12.9, 77.5, timestamp=time.time()))
        )
        self.assertIsNone(Navigator().position_issue(nmea))


def clear():
    return PerceptionResult(
        timestamp=time.time(), status=PerceptionStatus.OK,
        frame=FrameMetadata(width=640, height=480),
    )


class _ClearPerception:
    def latest(self):
        return clear()


def agent_on_esp32_gps(**env):
    base = {
        "ROBOTX_ROBOT_ID": "test-rover", "ROBOTX_CAMERA_ENABLED": "0",
        "ROBOTX_PERCEPTION_ENABLED": "0", "ROBOTX_LOG_LEVEL": "CRITICAL",
        "ROBOTX_GPS_ENABLED": "1", "ROBOTX_GPS_SOURCE": "esp32",
    }
    base.update(env)
    agent = RobotAgent(Settings.from_env(base))
    port = fx.FakePort()
    wall = fx.FakeClock(1_800_000_000.0)
    link, clock = fx.make_link(port, wall=wall, transmit_enabled=False)
    # What _start_gps does once _start_esp32 has built the link, without either
    # touching a real device.
    agent.esp32 = link
    agent.gps = Esp32GpsReader(link, stale_after_s=agent.settings.gps_stale_after_s, wall=wall)
    agent.perception = _ClearPerception()
    link.poll_once()
    return agent, link, port, clock, wall


class TestAgentGpsPath(unittest.TestCase):
    def feed(self, link, port, line):
        port.feed(line)
        link.poll_once()

    def test_frames_reach_state_navigation_and_decision_then_loss_stops(self):
        agent, link, port, clock, wall = agent_on_esp32_gps()
        self.assertIs(agent.tick().gps.status, GPSStatus.STARTING)

        self.feed(link, port, gps_line())
        snap = agent.tick()
        self.assertIs(snap.gps.status, GPSStatus.FIX)
        self.assertAlmostEqual(snap.position.latitude, 12.903214)
        self.assertEqual((snap.position.fix_type, snap.position.h_acc_m), ("3D", 1.5))

        agent.start_mission([(AHEAD_E7[0] / 1e7, AHEAD_E7[1] / 1e7)])
        snap = agent.tick()
        self.assertIs(snap.mode, OperatingMode.AUTO)
        self.assertIs(snap.navigation.status, NavigationStatus.NAVIGATING)
        self.assertAlmostEqual(snap.navigation.distance_to_target_m, 33.4, delta=0.5)
        self.assertAlmostEqual(snap.navigation.desired_heading_deg % 360, 0.0, delta=0.5)
        proposed = agent.decision.decide(mission_active=True, navigation=snap.navigation, perception=clear())
        self.assertFalse(proposed.is_stop)

        # The ESP32 goes quiet. Within the stale limit the last fix stands; past
        # it there is no position, navigation says so and the decision stops.
        clock.advance(4.0)
        wall.advance(4.0)
        self.assertIs(agent.tick().navigation.status, NavigationStatus.NAVIGATING)
        clock.advance(2.0)
        wall.advance(2.0)
        snap = agent.tick()
        self.assertIs(snap.gps.status, GPSStatus.STALE)
        # State keeps the last known position for display; nothing may act on
        # it without a fix, and navigation did not.
        self.assertFalse(snap.gps.has_fix)
        self.assertIs(snap.navigation.status, NavigationStatus.NO_POSITION)
        self.assertEqual(snap.motion_intent.reason, "no GPS position")
        self.assertTrue(snap.motion_intent.is_stop)

        # The receiver loses its fix: still stopped.
        self.feed(link, port, gps_line(gps_status="NO_FIX", fix_ok=False, lat_e7=None, lon_e7=None))
        snap = agent.tick()
        self.assertIs(snap.gps.status, GPSStatus.NO_FIX)
        self.assertTrue(snap.motion_intent.is_stop)

        # A fix that is back but poor: stopped, and health says why.
        self.feed(link, port, gps_line(hacc_mm=9_000))
        snap = agent.tick()
        self.assertIs(snap.gps.status, GPSStatus.FIX)
        self.assertIs(snap.navigation.status, NavigationStatus.NO_POSITION)
        self.assertEqual(snap.motion_intent.reason, "GPS fix not accurate enough to navigate")
        health = agent._component_health(snap)
        self.assertIs(health["gps"].status, HealthStatus.DEGRADED)
        self.assertIn("9.0 m", health["gps"].detail)
        self.assertIn("9.0 m", health["navigation"].detail)

        # Recovery: a good fix and the route resumes.
        self.feed(link, port, gps_line())
        self.assertIs(agent.tick().navigation.status, NavigationStatus.NAVIGATING)

        # Receive-only throughout: not one byte of command went to the ESP32.
        self.assertEqual(port.commands(), [])

    def test_a_wall_clock_step_back_does_not_keep_an_old_fix_alive(self):
        agent, link, port, clock, wall = agent_on_esp32_gps()
        self.feed(link, port, gps_line())
        self.assertIs(agent.tick().gps.status, GPSStatus.FIX)
        wall.advance(-3600)
        clock.advance(6.0)
        snap = agent.tick()
        self.assertIs(snap.gps.status, GPSStatus.STALE)
        self.assertIsNone(agent.position_estimator.update(snap.gps))

    def test_offer_is_rejected_on_a_fix_navigation_will_not_use(self):
        from tests.unit.test_engine_offer import parsed_offer

        agent, link, port, clock, wall = agent_on_esp32_gps(ROBOTX_ROBOT_ID="robotx-pi",
                                                            ROBOTX_CUSTODY_CONFIRMATION="operator")
        # SIMULATED: a motor link the offer checks would accept, so the
        # position check is the one under test.
        agent.state.update_communication(esp32=Esp32LinkStatus.UP)
        agent.esp32 = None
        self.feed(link, port, gps_line(hacc_mm=15_000))
        agent.tick()
        self.assertEqual(agent.assess_offer(parsed_offer()).reason, "NO_POSITION_FIX")
        self.feed(link, port, gps_line())
        agent.tick()
        agent.state.update_communication(esp32=Esp32LinkStatus.UP)
        from robotx.communication.engine import OfferVerdict

        self.assertIs(agent.assess_offer(parsed_offer()).verdict, OfferVerdict.ACCEPT)

    def test_esp32_source_without_the_link_is_a_failure_not_disabled(self):
        agent = RobotAgent(Settings.from_env({
            "ROBOTX_CAMERA_ENABLED": "0", "ROBOTX_PERCEPTION_ENABLED": "0",
            "ROBOTX_LOG_LEVEL": "CRITICAL", "ROBOTX_GPS_ENABLED": "1", "ROBOTX_GPS_SOURCE": "esp32",
        }))
        agent._start_esp32()                              # ESP32 disabled by default
        agent._start_gps()
        self.assertIsNone(agent.gps)
        h = agent._component_health(agent.tick())["gps"]
        self.assertIs(h.status, HealthStatus.FAILED)
        self.assertIn("ESP32 link is not running", h.detail)

    def test_esp32_source_never_opens_a_gps_serial_reader(self):
        from unittest import mock

        agent = RobotAgent(Settings.from_env({
            "ROBOTX_CAMERA_ENABLED": "0", "ROBOTX_PERCEPTION_ENABLED": "0",
            "ROBOTX_LOG_LEVEL": "CRITICAL", "ROBOTX_GPS_ENABLED": "1", "ROBOTX_GPS_SOURCE": "esp32",
            "ROBOTX_GPS_PORT": "/dev/ttyAMA3",
        }))
        agent.esp32, _ = fx.make_link(fx.FakePort(), transmit_enabled=False)
        with mock.patch("robotx.application.agent.GPSReader") as reader:
            agent._start_gps()
        reader.assert_not_called()
        self.assertIsInstance(agent.gps, Esp32GpsReader)


class TestSettings(unittest.TestCase):
    def test_gps_source_is_validated(self):
        self.assertEqual(Settings.from_env({"ROBOTX_GPS_SOURCE": " ESP32 "}).gps_source, "esp32")
        self.assertEqual(Settings.from_env({}).gps_source, "pi_serial")
        for bad in ("", "esp-32", "uart", "esp32,pi_serial"):
            with self.assertRaises(ValueError, msg=bad):
                Settings.from_env({"ROBOTX_GPS_SOURCE": bad})

    def test_navigation_accuracy_limit(self):
        self.assertEqual(Settings().nav_max_h_acc_m, 5.0)
        self.assertEqual(Settings.from_env({"ROBOTX_NAV_MAX_H_ACC_M": "3.5"}).nav_max_h_acc_m, 3.5)
        for bad in ("nan", "inf", "0", "-1", ""):
            with self.assertRaises(ValueError, msg=bad):
                Settings.from_env({"ROBOTX_NAV_MAX_H_ACC_M": bad})
        self.assertTrue(math.isfinite(NavigationConfig.from_settings(Settings()).max_h_acc_m))


if __name__ == "__main__":
    unittest.main()
