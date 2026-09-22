"""Integration: localization -> navigation -> decision, on synthetic GPS.

The real GPS produces zero NMEA bytes, so no test here uses live hardware.
Everything below is driven by deterministic, hand-built NMEA sentences and
fixed coordinates. That makes these PASS-SYNTHETIC: the software chain is
validated, the hardware is not.

The fixtures feed the *real* parser (`parse_nmea_sentence`), the real
`PositionEstimator`, the real `Navigator` and the real `DecisionMaker`, so the
only thing substituted is the serial port.
"""

import time
import unittest

from robotx.control.decision import DecisionConfig, DecisionMaker
from robotx.control.motion import MotionCommand
from robotx.hardware.gps import GpsReading, GPSStatus, parse_nmea_sentence
from robotx.localization.position import PositionConfig, PositionEstimator
from robotx.navigation.navigator import NavigationConfig, Navigator, NavigationStatus
from robotx.perception.types import FrameMetadata, PerceptionResult, PerceptionStatus


# --- synthetic NMEA -----------------------------------------------------------


def nmea_checksum(body: str) -> str:
    c = 0
    for ch in body:
        c ^= ord(ch)
    return f"{c:02X}"


def gga(lat_deg: float, lon_deg: float, sats: int = 9, alt: float = 42.0) -> str:
    """Build a valid $GPGGA sentence for a decimal-degree position."""

    lat_hem = "N" if lat_deg >= 0 else "S"
    lon_hem = "E" if lon_deg >= 0 else "W"
    lat_deg, lon_deg = abs(lat_deg), abs(lon_deg)
    lat_d, lon_d = int(lat_deg), int(lon_deg)
    lat_m, lon_m = (lat_deg - lat_d) * 60, (lon_deg - lon_d) * 60
    body = (
        f"GPGGA,123519,{lat_d:02d}{lat_m:07.4f},{lat_hem},"
        f"{lon_d:03d}{lon_m:07.4f},{lon_hem},1,{sats:02d},0.9,{alt:.1f},M,46.9,M,,"
    )
    return f"${body}*{nmea_checksum(body)}"


def rmc(lat_deg: float, lon_deg: float, knots: float, track: float) -> str:
    """Build a valid $GPRMC sentence with speed and course over ground."""

    lat_hem = "N" if lat_deg >= 0 else "S"
    lon_hem = "E" if lon_deg >= 0 else "W"
    lat_deg, lon_deg = abs(lat_deg), abs(lon_deg)
    lat_d, lon_d = int(lat_deg), int(lon_deg)
    lat_m, lon_m = (lat_deg - lat_d) * 60, (lon_deg - lon_d) * 60
    body = (
        f"GPRMC,123519,A,{lat_d:02d}{lat_m:07.4f},{lat_hem},"
        f"{lon_d:03d}{lon_m:07.4f},{lon_hem},{knots:.1f},{track:.1f},230394,003.1,W"
    )
    return f"${body}*{nmea_checksum(body)}"


def reading_from(sentence: str, previous=None) -> GpsReading:
    """Parse a synthetic sentence into the reading a GPSReader would publish."""

    fix = parse_nmea_sentence(sentence, previous=previous)
    if fix is None:
        return GpsReading(status=GPSStatus.NO_FIX)
    return GpsReading(status=GPSStatus.FIX, fix=fix, age_s=0.1)


def clear_view(width: int = 640, height: int = 480) -> PerceptionResult:
    return PerceptionResult(
        timestamp=time.time(),
        status=PerceptionStatus.OK,
        detections=(),
        frame=FrameMetadata(width=width, height=height, age_s=0.05),
        backend="synthetic",
    )


# A straight 30 m course due north, in ~10 m steps.
START = (51.500000, -0.100000)
MID = (51.500090, -0.100000)
END = (51.500180, -0.100000)


class TestSyntheticFixtures(unittest.TestCase):
    """The fixtures must themselves be valid, or every test below is vacuous."""

    def test_generated_gga_parses_back_to_the_input_position(self):
        fix = parse_nmea_sentence(gga(51.5, -0.1))
        self.assertIsNotNone(fix)
        self.assertAlmostEqual(fix.latitude, 51.5, places=5)
        self.assertAlmostEqual(fix.longitude, -0.1, places=5)
        self.assertEqual(fix.satellites, 9)

    def test_generated_rmc_parses_back_speed_and_track(self):
        fix = parse_nmea_sentence(rmc(51.5, -0.1, knots=4.0, track=90.0))
        self.assertIsNotNone(fix)
        self.assertAlmostEqual(fix.track_deg, 90.0, places=1)
        self.assertAlmostEqual(fix.speed_mps, 4.0 * 0.514444, places=2)

    def test_checksums_are_well_formed(self):
        for sentence in (gga(51.5, -0.1), rmc(51.5, -0.1, 4.0, 90.0)):
            body, _, checksum = sentence[1:].partition("*")
            self.assertEqual(nmea_checksum(body), checksum)

    def test_southern_and_western_hemispheres_round_trip(self):
        fix = parse_nmea_sentence(gga(-33.8688, 151.2093))
        self.assertAlmostEqual(fix.latitude, -33.8688, places=4)
        self.assertAlmostEqual(fix.longitude, 151.2093, places=4)


class TestLocalizationChain(unittest.TestCase):
    """NMEA -> Position, through the real estimator."""

    def setUp(self):
        self.estimator = PositionEstimator(PositionConfig())

    def test_first_fix_gives_position_but_no_heading(self):
        pos = self.estimator.update(reading_from(gga(*START)))
        self.assertIsNotNone(pos)
        self.assertAlmostEqual(pos.latitude, START[0], places=5)
        self.assertIsNone(pos.heading_deg)

    def test_moving_north_derives_a_northward_heading(self):
        self.estimator.update(reading_from(gga(*START)))
        pos = self.estimator.update(reading_from(gga(*MID)))
        self.assertIsNotNone(pos.heading_deg)
        self.assertAlmostEqual(pos.heading_deg, 0.0, places=1)

    def test_rmc_track_is_preferred_while_moving(self):
        pos = self.estimator.update(reading_from(rmc(*START, knots=6.0, track=270.0)))
        self.assertAlmostEqual(pos.heading_deg, 270.0, places=1)

    def test_no_fix_sentence_yields_no_position(self):
        void = "$GPRMC,123519,V,,,,,,,,,,N*53"
        self.assertIsNone(self.estimator.update(reading_from(void)))


class TestNavigationOnSyntheticTrack(unittest.TestCase):
    """A full synthetic drive: start -> waypoint -> destination."""

    def setUp(self):
        self.estimator = PositionEstimator(PositionConfig())
        self.navigator = Navigator(NavigationConfig(waypoint_arrival_m=8.0))
        self.decider = DecisionMaker(DecisionConfig.from_settings(_settings()))

    def advance(self, coord, *, knots=4.0, track=0.0):
        reading = reading_from(rmc(*coord, knots=knots, track=track))
        position = self.estimator.update(reading)
        nav = self.navigator.update(position)
        intent = self.decider.decide(
            mission_active=True, navigation=nav, perception=clear_view()
        )
        return position, nav, intent

    def test_full_traverse_reaches_the_destination(self):
        self.navigator.set_route([MID, END])

        _, nav, intent = self.advance(START)
        self.assertIs(nav.status, NavigationStatus.NAVIGATING)
        self.assertEqual(nav.target_waypoint, MID)
        self.assertIs(intent.command, MotionCommand.FORWARD)

        _, nav, _ = self.advance(MID)
        self.assertEqual(nav.target_waypoint, END, "should advance past the first waypoint")

        _, nav, intent = self.advance(END)
        self.assertIs(nav.status, NavigationStatus.ARRIVED)
        self.assertIs(intent.command, MotionCommand.HOLD)
        self.assertIn("destination", intent.reason)

    def test_distance_shrinks_as_the_robot_closes_in(self):
        self.navigator.set_route([END])
        distances = []
        for coord in (START, MID, END):
            _, nav, _ = self.advance(coord)
            if nav.distance_to_target_m is not None:
                distances.append(nav.distance_to_target_m)
        self.assertEqual(distances, sorted(distances, reverse=True))
        self.assertLess(distances[-1], 8.0)

    def test_facing_the_wrong_way_produces_a_turn(self):
        self.navigator.set_route([END])
        # Target is due north; the robot is tracking due south.
        _, nav, intent = self.advance(START, track=180.0)
        self.assertAlmostEqual(abs(nav.heading_error_deg), 180.0, places=0)
        self.assertIn(intent.command, (MotionCommand.TURN_LEFT, MotionCommand.TURN_RIGHT))

    def test_slight_drift_steers_without_stopping(self):
        self.navigator.set_route([END])
        _, nav, intent = self.advance(START, track=20.0)
        self.assertIs(intent.command, MotionCommand.FORWARD)
        self.assertNotEqual(intent.left, intent.right, "should steer")

    def test_losing_the_fix_mid_route_stops_the_robot(self):
        self.navigator.set_route([END])
        _, _, intent = self.advance(START)
        self.assertIs(intent.command, MotionCommand.FORWARD)

        # Fix drops out: estimator returns None, navigation has no position.
        nav = self.navigator.update(None)
        intent = self.decider.decide(
            mission_active=True, navigation=nav, perception=clear_view()
        )
        self.assertIs(nav.status, NavigationStatus.NO_POSITION)
        self.assertTrue(intent.is_stop)
        self.assertIn("GPS", intent.reason)

    def test_stale_fix_is_not_reused_as_a_position(self):
        self.navigator.set_route([END])
        self.advance(START)
        stale = GpsReading(
            status=GPSStatus.STALE,
            fix=parse_nmea_sentence(gga(*START)),
            age_s=60.0,
        )
        self.assertIsNone(self.estimator.update(stale))

    def test_perception_loss_overrides_a_good_route(self):
        self.navigator.set_route([END])
        position = self.estimator.update(reading_from(rmc(*START, knots=4.0, track=0.0)))
        nav = self.navigator.update(position)
        self.assertIs(nav.status, NavigationStatus.NAVIGATING)

        blind = PerceptionResult.unavailable(PerceptionStatus.NO_FRAME)
        intent = self.decider.decide(mission_active=True, navigation=nav, perception=blind)
        self.assertTrue(intent.is_stop, "navigation being fine must not override blindness")

    def test_run_is_deterministic(self):
        def run():
            est = PositionEstimator(PositionConfig())
            nav = Navigator(NavigationConfig(waypoint_arrival_m=8.0))
            dec = DecisionMaker(DecisionConfig.from_settings(_settings()))
            nav.set_route([MID, END])
            out = []
            for coord in (START, MID, END):
                pos = est.update(reading_from(rmc(*coord, knots=4.0, track=0.0)))
                state = nav.update(pos)
                intent = dec.decide(mission_active=True, navigation=state,
                                    perception=clear_view())
                out.append((state.status.value, intent.command.value))
            return out

        self.assertEqual(run(), run(), "synthetic fixtures must be reproducible")


def _settings():
    from robotx.config.settings import Settings
    return Settings.from_env({})


if __name__ == "__main__":
    unittest.main()
