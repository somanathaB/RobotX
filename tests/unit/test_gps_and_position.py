"""GPS parsing, fix validity, staleness, and position/heading estimation."""

import time
import unittest

from robotx.hardware.gps import (
    GPSConfig,
    GPSReader,
    GPSStatus,
    GpsFix,
    GpsReading,
    parse_nmea_sentence,
)
from robotx.localization.position import (
    HeadingSource,
    PositionConfig,
    PositionEstimator,
    bearing_deg,
    haversine_m,
    heading_error_deg,
)


# Real-shaped NMEA sentences (checksums are not verified by the parser).
GGA_VALID = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
GGA_NO_FIX = "$GPGGA,123519,4807.038,N,01131.000,E,0,00,,,M,,M,,*47"
RMC_VALID = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"
RMC_INVALID = "$GPRMC,123519,V,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"
GSV_NO_POSITION = "$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74"


class TestNmeaParsing(unittest.TestCase):
    def test_valid_gga_gives_position_and_metadata(self):
        fix = parse_nmea_sentence(GGA_VALID)
        self.assertIsNotNone(fix)
        self.assertAlmostEqual(fix.latitude, 48.1173, places=3)
        self.assertAlmostEqual(fix.longitude, 11.5166, places=3)
        self.assertEqual(fix.satellites, 8)
        self.assertEqual(fix.fix_quality, 1)
        self.assertAlmostEqual(fix.altitude_m, 545.4, places=1)

    def test_gga_with_quality_zero_is_rejected(self):
        # Quality 0 means the receiver has no valid fix.
        self.assertIsNone(parse_nmea_sentence(GGA_NO_FIX))

    def test_valid_rmc_gives_speed_and_track(self):
        fix = parse_nmea_sentence(RMC_VALID)
        self.assertIsNotNone(fix)
        self.assertAlmostEqual(fix.track_deg, 84.4, places=1)
        # 22.4 knots -> m/s
        self.assertAlmostEqual(fix.speed_mps, 22.4 * 0.514444, places=2)

    def test_rmc_with_void_status_is_rejected(self):
        # 'V' is a navigation-receiver warning: the data is not valid.
        self.assertIsNone(parse_nmea_sentence(RMC_INVALID))

    def test_sentence_without_position_is_ignored(self):
        self.assertIsNone(parse_nmea_sentence(GSV_NO_POSITION))

    def test_garbage_does_not_raise(self):
        for junk in ("", "not nmea", "$GPGGA,,,,,,,,", "$$$$"):
            self.assertIsNone(parse_nmea_sentence(junk))

    def test_fields_carry_over_between_sentence_types(self):
        # GGA supplies satellites; the following RMC should keep them.
        gga = parse_nmea_sentence(GGA_VALID)
        rmc = parse_nmea_sentence(RMC_VALID, previous=gga)
        self.assertEqual(rmc.satellites, 8)
        self.assertIsNotNone(rmc.track_deg)


class TestGpsReading(unittest.TestCase):
    def test_reading_without_fix_has_no_position(self):
        reading = GpsReading(status=GPSStatus.NO_FIX)
        self.assertFalse(reading.has_fix)
        self.assertIsNone(reading.fix)

    def test_stale_status_is_not_a_fix(self):
        fix = GpsFix(latitude=1.0, longitude=2.0, timestamp=time.time() - 100)
        reading = GpsReading(status=GPSStatus.STALE, fix=fix, age_s=100.0)
        self.assertFalse(reading.has_fix)

    def test_aged_fix_is_reported_stale_by_reader(self):
        reader = GPSReader(GPSConfig(stale_after_s=1.0))
        reader._fix = GpsFix(latitude=1.0, longitude=2.0, timestamp=time.time() - 30)
        reader._status = GPSStatus.FIX

        reading = reader.get_reading()
        self.assertIs(reading.status, GPSStatus.STALE)
        self.assertFalse(reading.has_fix)
        self.assertGreater(reading.age_s, 1.0)

    def test_fresh_fix_is_reported_as_fix(self):
        reader = GPSReader(GPSConfig(stale_after_s=5.0))
        reader._fix = GpsFix(latitude=1.0, longitude=2.0, timestamp=time.time())
        reader._status = GPSStatus.FIX
        self.assertTrue(reader.get_reading().has_fix)

    def test_reading_serializes_without_a_fix(self):
        payload = GpsReading(status=GPSStatus.DISCONNECTED, error="port missing").to_dict()
        self.assertEqual(payload["status"], "DISCONNECTED")
        self.assertIsNone(payload["fix"])


class TestGeoMath(unittest.TestCase):
    def test_haversine_known_distance(self):
        # One degree of latitude is ~111.2 km.
        self.assertAlmostEqual(haversine_m((0.0, 0.0), (1.0, 0.0)) / 1000.0, 111.19, places=1)

    def test_haversine_zero_for_same_point(self):
        self.assertAlmostEqual(haversine_m((51.5, -0.1), (51.5, -0.1)), 0.0, places=6)

    def test_bearing_cardinal_directions(self):
        self.assertAlmostEqual(bearing_deg((0.0, 0.0), (1.0, 0.0)), 0.0, places=3)
        self.assertAlmostEqual(bearing_deg((0.0, 0.0), (0.0, 1.0)), 90.0, places=3)
        self.assertAlmostEqual(bearing_deg((1.0, 0.0), (0.0, 0.0)), 180.0, places=3)
        self.assertAlmostEqual(bearing_deg((0.0, 1.0), (0.0, 0.0)), 270.0, places=3)

    def test_heading_error_takes_the_short_way_round(self):
        self.assertAlmostEqual(heading_error_deg(10.0, 350.0), 20.0, places=6)
        self.assertAlmostEqual(heading_error_deg(350.0, 10.0), -20.0, places=6)
        self.assertAlmostEqual(heading_error_deg(90.0, 90.0), 0.0, places=6)

    def test_heading_error_is_bounded(self):
        for desired in range(0, 360, 17):
            for current in range(0, 360, 23):
                error = heading_error_deg(float(desired), float(current))
                self.assertGreater(error, -180.001)
                self.assertLessEqual(error, 180.001)


def fix_reading(lat, lon, *, speed=None, track=None):
    return GpsReading(
        status=GPSStatus.FIX,
        fix=GpsFix(
            latitude=lat,
            longitude=lon,
            timestamp=time.time(),
            speed_mps=speed,
            track_deg=track,
        ),
        age_s=0.1,
    )


class TestPositionEstimator(unittest.TestCase):
    def setUp(self):
        self.estimator = PositionEstimator(
            PositionConfig(heading_min_move_m=1.5, heading_min_speed_mps=0.5)
        )

    def test_no_fix_yields_no_position(self):
        self.assertIsNone(self.estimator.update(GpsReading(status=GPSStatus.NO_FIX)))
        self.assertIsNone(self.estimator.update(GpsReading(status=GPSStatus.DISCONNECTED)))

    def test_stale_reading_yields_no_position(self):
        stale = GpsReading(
            status=GPSStatus.STALE,
            fix=GpsFix(latitude=1.0, longitude=2.0, timestamp=time.time() - 60),
        )
        self.assertIsNone(self.estimator.update(stale))

    def test_first_fix_has_no_heading(self):
        # No compass: a stationary robot has no knowable heading.
        position = self.estimator.update(fix_reading(51.5, -0.1))
        self.assertIsNotNone(position)
        self.assertIsNone(position.heading_deg)
        self.assertIs(position.heading_source, HeadingSource.NONE)

    def test_nmea_track_used_when_moving(self):
        position = self.estimator.update(fix_reading(51.5, -0.1, speed=3.0, track=97.0))
        self.assertAlmostEqual(position.heading_deg, 97.0)
        self.assertIs(position.heading_source, HeadingSource.NMEA_TRACK)

    def test_nmea_track_ignored_when_stationary(self):
        # A parked receiver's course field is noise.
        position = self.estimator.update(fix_reading(51.5, -0.1, speed=0.05, track=97.0))
        self.assertIsNone(position.heading_deg)
        self.assertIs(position.heading_source, HeadingSource.NONE)

    def test_heading_derived_from_consecutive_fixes(self):
        self.estimator.update(fix_reading(51.5000000, -0.1))
        # ~5.5 m north of the previous fix.
        position = self.estimator.update(fix_reading(51.5000500, -0.1))
        self.assertIs(position.heading_source, HeadingSource.GPS_TRACK)
        self.assertAlmostEqual(position.heading_deg, 0.0, places=1)

    def test_small_movement_does_not_produce_heading(self):
        self.estimator.update(fix_reading(51.5, -0.1))
        # ~0.1 m: GPS wander, not movement.
        position = self.estimator.update(fix_reading(51.500001, -0.1))
        self.assertIsNone(position.heading_deg)

    def test_last_heading_is_held_when_movement_stops(self):
        self.estimator.update(fix_reading(51.5000000, -0.1))
        self.estimator.update(fix_reading(51.5000500, -0.1))
        held = self.estimator.update(fix_reading(51.5000501, -0.1))
        self.assertIsNotNone(held.heading_deg)
        self.assertIs(held.heading_source, HeadingSource.GPS_TRACK)

    def test_position_carries_receiver_metadata(self):
        reading = GpsReading(
            status=GPSStatus.FIX,
            fix=GpsFix(
                latitude=51.5,
                longitude=-0.1,
                timestamp=time.time(),
                altitude_m=42.0,
                satellites=9,
            ),
        )
        position = self.estimator.update(reading)
        self.assertEqual(position.altitude_m, 42.0)
        self.assertEqual(position.satellites, 9)

    def test_reset_clears_heading_history(self):
        self.estimator.update(fix_reading(51.5000000, -0.1))
        self.estimator.update(fix_reading(51.5000500, -0.1))
        self.estimator.reset()
        position = self.estimator.update(fix_reading(51.5001000, -0.1))
        self.assertIsNone(position.heading_deg)


if __name__ == "__main__":
    unittest.main()
