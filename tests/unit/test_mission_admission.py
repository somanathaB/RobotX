"""Mission admission and navigation hardening: valid and invalid routes.

- Structural validation (`Mission.create`): finite, bounded, well-formed.
- Route admission (`robotx.mission.admission`): the pickup leg starts where the
  Rover stands, the pickup is within the configured distance, and path ends
  meet their stops -- through `RobotAgent.assess_offer`, the path every engine
  OFFER and every test injection takes.
- No silent forward motion: loss of a usable position, stale perception, an
  obstacle, or a controller fault while driving stops the DRIVE stream.

What is real and what is simulated
----------------------------------
- REAL: `Mission`, the admission checks, `RobotAgent.assess_offer`, the
  navigator, decision layer, safety gate and `Esp32Link`.
- SIMULATED: GPS fixes, perception, the ESP32 (`FirmwareModel`), the OFFER.
"""

from __future__ import annotations

import logging
import math
import time
import unittest

from robotx.application.agent import RobotAgent  # noqa: F401  first: robot_state import cycle (pre-existing)
from robotx.communication.engine import OfferVerdict, parse_envelope, parse_offer
from robotx.config.settings import Settings
from robotx.hardware.gps import GPSStatus, GpsReading
from robotx.mission.admission import AdmissionConfig, check_admissible
from robotx.mission.mission import Mission, MissionRejected, MissionRejectReason
from robotx.perception.types import PerceptionResult, PerceptionStatus
from robotx.state.robot_state import OperatingMode
from tests.fixtures import engine as efx
from tests.fixtures.task_assign import DROP, PICKUP, START, leg, offset
from tests.unit.test_injection_endpoint import ready_rig
from tests.unit.test_obstacle_recovery import Rig


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


def mission(*, pickup=PICKUP, drop=DROP, to_pickup=None, to_drop=None):
    """The fixture route (START -> PICKUP -> DROP), with any field replaced."""

    return Mission.create(
        task_id="T-ADMIT", pickup=pickup, drop=drop,
        path_to_pickup=to_pickup if to_pickup is not None else leg(START, PICKUP),
        path_to_drop=to_drop if to_drop is not None else leg(PICKUP, DROP),
        timestamp=time.time(),
    )


CFG = AdmissionConfig()


class TestStructuralValidation(unittest.TestCase):
    """`Mission.create` refuses anything not finite, bounded and well-formed."""

    def test_a_valid_route_is_accepted(self):
        m = mission()
        self.assertEqual((m.path_to_pickup[0], m.path_to_pickup[-1]), (START, PICKUP))
        self.assertEqual((m.path_to_drop[0], m.path_to_drop[-1]), (PICKUP, DROP))

    def test_invalid_routes_are_refused_with_a_reason(self):
        good = leg(START, PICKUP)
        cases = {
            "nan lat": (dict(pickup=(math.nan, 77.0)), MissionRejectReason.INVALID_COORDINATE),
            "inf lon": (dict(drop=(12.9, math.inf)), MissionRejectReason.INVALID_COORDINATE),
            "lat > 90": (dict(pickup=(90.5, 77.0)), MissionRejectReason.INVALID_COORDINATE),
            "lon < -180": (dict(drop=(12.9, -180.01)), MissionRejectReason.INVALID_COORDINATE),
            "bool coordinate": (dict(pickup=(True, 77.0)), MissionRejectReason.INVALID_COORDINATE),
            "string coordinate": (dict(pickup=("12.9", 77.0)), MissionRejectReason.INVALID_COORDINATE),
            "one-point leg": (dict(to_pickup=[START]), MissionRejectReason.EMPTY_ROUTE),
            "empty leg": (dict(to_drop=[]), MissionRejectReason.EMPTY_ROUTE),
            "too many points": (dict(to_drop=[PICKUP] * 2001), MissionRejectReason.ROUTE_TOO_LONG),
            "point not a pair": (dict(to_pickup=good[:-1] + [(1.0, 2.0, 3.0)]), MissionRejectReason.MALFORMED),
            "leg is a string": (dict(to_pickup="12.9,77.5"), MissionRejectReason.MALFORMED),
            "nan in a leg": (dict(to_pickup=good[:2] + [(math.nan, 77.5)]), MissionRejectReason.INVALID_COORDINATE),
        }
        for name, (kwargs, reason) in cases.items():
            with self.subTest(name):
                with self.assertRaises(MissionRejected) as caught:
                    mission(**kwargs)
                self.assertIs(caught.exception.reason, reason)


class TestRouteAdmission(unittest.TestCase):
    def assert_refused(self, m, here, reason, text):
        with self.assertRaises(MissionRejected) as caught:
            check_admissible(m, here, CFG)
        self.assertIs(caught.exception.reason, reason)
        self.assertIn(text, caught.exception.detail)

    def test_a_route_from_here_to_a_near_pickup_is_admitted(self):
        check_admissible(mission(), START, CFG)
        check_admissible(mission(), offset(START, north_m=20.0), CFG)     # inside the 25 m road-snap

    def test_a_route_that_starts_elsewhere_is_refused(self):
        self.assert_refused(mission(), offset(START, north_m=-60.0), MissionRejectReason.ROUTE_START_TOO_FAR,
                            "pathToPickup starts 60 m from the Rover")

    def test_a_pickup_beyond_the_limit_is_refused_with_the_distance(self):
        far, beyond = offset(START, north_m=400.0), offset(START, north_m=450.0)
        route = mission(pickup=far, drop=beyond, to_pickup=leg(START, far, points=30),
                        to_drop=leg(far, beyond))
        self.assert_refused(route, START,
                            MissionRejectReason.PICKUP_TOO_FAR, "the pickup is 400 m from the Rover (limit 300 m)")

    def test_no_position_is_refused_not_guessed(self):
        self.assert_refused(mission(), None, MissionRejectReason.PICKUP_TOO_FAR, "no measured position")

    def test_path_ends_must_meet_their_stops(self):
        astray = offset(PICKUP, east_m=80.0)
        cases = (
            (dict(to_pickup=leg(START, astray)), "the end of pathToPickup and the pickup are 80 m apart"),
            (dict(to_drop=leg(astray, DROP)), "the start of pathToDrop and the pickup are 80 m apart"),
            (dict(to_drop=leg(PICKUP, offset(DROP, east_m=80.0))), "the end of pathToDrop and the drop"),
        )
        for kwargs, text in cases:
            with self.subTest(text):
                self.assert_refused(mission(**kwargs), START, MissionRejectReason.ROUTE_INCONSISTENT, text)

    def test_limits_come_from_settings_and_are_bounded(self):
        cfg = AdmissionConfig.from_settings(Settings.from_env({
            "ROBOTX_MISSION_MAX_START_OFFSET_M": "10", "ROBOTX_MISSION_MAX_PICKUP_DISTANCE_M": "1000",
            "ROBOTX_MISSION_MAX_ENDPOINT_OFFSET_M": "20"}))
        self.assertEqual((cfg.max_start_offset_m, cfg.max_pickup_distance_m, cfg.max_endpoint_offset_m),
                         (10.0, 1000.0, 20.0))
        with self.assertRaises(MissionRejected):
            check_admissible(mission(), offset(START, north_m=15.0), cfg)
        for bad in (dict(max_start_offset_m=0.0), dict(max_start_offset_m=1e9), dict(max_pickup_distance_m=math.nan),
                    dict(max_endpoint_offset_m=-1.0), dict(max_pickup_distance_m=5001.0)):
            with self.subTest(bad), self.assertRaises(ValueError):
                AdmissionConfig(**bad)


def offer():
    return parse_offer(parse_envelope(efx.envelope(), expected_agent_id=efx.ROBOT_ID))


class TestThroughAssessOffer(unittest.TestCase):
    """The real agent, ready by its own rules, deciding the fixture OFFER."""

    def rig_at(self, where):
        rig = ready_rig()
        rig.agent.gps.set_fix(*where)
        rig.run(0.6)
        return rig

    def test_accepted_when_standing_at_the_route_start(self):
        rig = self.rig_at(START)
        decision = rig.agent.assess_offer(offer())
        self.assertIs(decision.verdict, OfferVerdict.ACCEPT, decision.reason)

    def test_refused_when_the_route_does_not_start_here(self):
        rig = self.rig_at(offset(START, north_m=-200.0))
        decision = rig.agent.assess_offer(offer())
        self.assertEqual(decision.reason, "UNSUPPORTED_MISSION: ROUTE_START_TOO_FAR")
        self.assertIsNone(rig.agent.missions.active)

    def test_readiness_is_still_judged_first(self):
        rig = self.rig_at(offset(START, north_m=-200.0))
        rig.agent.emergency_stop("test")
        self.assertEqual(rig.agent.assess_offer(offer()).reason, "ESTOP_LATCHED")


# --- no silent forward motion ---------------------------------------------------------


class TestNoSilentForwardMotion(unittest.TestCase):
    """Each loss while driving ends the DRIVE stream within a tick or two."""

    def setUp(self):
        self.rig = Rig()
        self.rig.start()

    def assert_drive_stream_ends(self, settle_ticks=1, seconds=3.0):
        for _ in range(settle_ticks):
            self.rig.step()
        mark = self.rig.mark()
        snaps = self.rig.run(seconds)
        self.assertEqual(self.rig.drives(mark), [], "DRIVE after the condition")
        self.assertTrue(all(s.motion_intent.is_stop for s in snaps))
        self.assertEqual(self.rig.port.command_names()[-1], "STOP")
        return snaps

    def test_losing_the_gps_fix(self):
        self.rig.agent.gps.reading = GpsReading(status=GPSStatus.NO_FIX)
        self.assert_drive_stream_ends()

    def test_a_fix_too_inaccurate_to_navigate_on(self):
        from robotx.localization.position import Position, PositionSource

        estimator = self.rig.agent.position_estimator
        real_update = estimator.update

        def inaccurate(reading, **kwargs):
            p = real_update(reading, **kwargs)
            return None if p is None else Position(latitude=p.latitude, longitude=p.longitude,
                                                   timestamp=p.timestamp, source=PositionSource.GPS,
                                                   fix_type="3D", h_acc_m=20.0)
        estimator.update = inaccurate
        snaps = self.assert_drive_stream_ends()
        self.assertIn("not accurate enough", snaps[-1].motion_intent.reason)

    def test_stale_perception(self):
        self.rig.agent.perception.result = PerceptionResult(timestamp=time.time() - 60,
                                                            status=PerceptionStatus.STALE)
        self.assert_drive_stream_ends()

    def test_a_camera_obstacle(self):
        self.rig.agent.perception.set_detection(area=40000, cx=320)
        self.assert_drive_stream_ends()

    def test_a_range_obstacle_from_the_esp32(self):
        self.rig.fields.update(front_left_cm=20.0, front_right_cm=20.0)
        self.assert_drive_stream_ends()

    def test_a_controller_fault(self):
        self.rig.fields.update(safety_stop=True)
        snaps = self.assert_drive_stream_ends()
        self.assertIs(snaps[-1].mode, OperatingMode.PAUSED)

    def test_stale_controller_telemetry(self):
        self.rig.alive = False
        snaps = self.assert_drive_stream_ends(settle_ticks=ticks_for(1.2))
        self.assertIs(snaps[-1].mode, OperatingMode.PAUSED)


def ticks_for(seconds):
    from tests.unit.test_obstacle_recovery import ticks

    return ticks(seconds)


if __name__ == "__main__":
    unittest.main()
