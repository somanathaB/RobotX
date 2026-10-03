"""Gate 3 contract pieces: what the Pi must do for the backend's physical
assignment path (RobotX Gate 1) and for the ESP32's current firmware.

- ESP32 `GPS` frames (PROTOCOL.md section 11) are decoded, not rejected, and
  become the same GpsReading the NMEA reader produces.
- TELEMETRY carries the receiver's fix quality and the software stop latch only
  when they are actually known.
- PROBE is answered on the socket with the correlation id echoed.
- A signed envelope is received under its own command name (OFFER, ...), the
  event the backend really emits -- not an event called "command".
"""

from __future__ import annotations

import time
import unittest
from dataclasses import replace

from robotx.communication.protocol import ProtocolBinding, build_telemetry_payload
from robotx.control.safety import ESTOP_RULE, SafetyDecision
from robotx.esp32.protocol import crc16, decode_line
from robotx.esp32.state import ControllerState
from robotx.hardware.gps import GPSStatus
from robotx.localization.esp32_gps import reading_from_frame
from robotx.localization.position import PositionEstimator
from tests.unit.test_backend_link import AsyncTestCase, connect_and_auth, make_link, state_with_fix


def frame_line(obj: str) -> bytes:
    payload = obj.encode("ascii")
    return payload + b"*%04X\n" % crc16(payload)


GPS_OK = (
    '{"type":"GPS","uptime_ms":6252,"gps_status":"OK","fix_type":3,"fix_ok":true,"siv":18,'
    '"lat_e7":129032140,"lon_e7":775190770,"alt_msl_mm":861690,"hacc_mm":1500,"vacc_mm":2500,'
    '"speed_mm_s":20,"head_mot_e5":34358000,"pdop_e2":120,"itow_ms":123456000,"age_ms":180,'
    '"polls":8,"pvt_ok":6,"poll_timeouts":2,"bus_errors":0,"ff_chunks":0,"max_service_us":9810,'
    '"latency_ms":40}'
)
GPS_BACKOFF = (
    '{"type":"GPS","uptime_ms":1000,"gps_status":"BACKOFF","fix_type":null,"fix_ok":null,"siv":null,'
    '"lat_e7":null,"lon_e7":null,"hacc_mm":null,"age_ms":null}'
)


class TestEsp32GpsFrames(unittest.TestCase):
    def test_a_documented_gps_frame_is_decoded_not_rejected(self):
        frame = decode_line(frame_line(GPS_OK))
        self.assertEqual(getattr(frame, "type", None), "GPS")

    def test_an_undocumented_gps_status_is_rejected(self):
        bad = decode_line(frame_line('{"type":"GPS","uptime_ms":1,"gps_status":"GREAT"}'))
        self.assertFalse(hasattr(bad, "type"))

    def test_ok_frame_becomes_a_fix_with_the_receivers_quality(self):
        frame = dict(decode_line(frame_line(GPS_OK)).data)
        now = 1_000_000.0
        reading = reading_from_frame(frame, now, now=now + 0.1, stale_after_s=5.0)
        self.assertIs(reading.status, GPSStatus.FIX)
        self.assertAlmostEqual(reading.fix.latitude, 12.903214, places=6)
        self.assertEqual(reading.fix.fix_type, "3D")
        self.assertAlmostEqual(reading.fix.h_acc_m, 1.5)
        # Stamped when the PVT arrived, not when the Pi read the frame.
        self.assertAlmostEqual(reading.fix.timestamp, now - 0.18)
        position = PositionEstimator().update(reading)
        self.assertEqual((position.fix_type, position.h_acc_m), ("3D", 1.5))

    def test_backoff_and_absent_frames_are_no_fix(self):
        frame = dict(decode_line(frame_line(GPS_BACKOFF)).data)
        self.assertIs(reading_from_frame(frame, 1.0, now=1.0, stale_after_s=5.0).status, GPSStatus.NO_FIX)
        self.assertIs(reading_from_frame(None, None, now=1.0, stale_after_s=5.0).status, GPSStatus.STARTING)

    def test_an_old_fix_is_stale(self):
        frame = dict(decode_line(frame_line(GPS_OK)).data)
        self.assertIs(reading_from_frame(frame, 100.0, now=110.0, stale_after_s=5.0).status, GPSStatus.STALE)

    def test_gnss_plus_dead_reckoning_carries_no_fix_type(self):
        frame = dict(decode_line(frame_line(GPS_OK.replace('"fix_type":3', '"fix_type":4'))).data)
        reading = reading_from_frame(frame, 1.0, now=1.0, stale_after_s=5.0)
        self.assertIs(reading.status, GPSStatus.FIX)
        self.assertIsNone(reading.fix.fix_type)


def controller_telemetry(*, safety_stop: bool):
    from robotx.esp32.state import ControllerTelemetry
    from tests.fixtures.esp32 import REAL_TELEMETRY_FIELDS

    return ControllerTelemetry.from_frame(dict(REAL_TELEMETRY_FIELDS, safety_stop=safety_stop), time.time())


class TestTelemetryBlocks(unittest.TestCase):
    def _snapshot(self, *, controller=None, safety=None, fix_type="3D", h_acc_m=1.5):
        snap = state_with_fix().snapshot()
        position = replace(snap.position, fix_type=fix_type, h_acc_m=h_acc_m)
        return replace(snap, position=position, controller=controller, safety=safety or snap.safety)

    def test_fix_quality_rides_with_a_fix_only_when_both_are_stated(self):
        snap = self._snapshot()
        frame = build_telemetry_payload(snap, sequence=1, max_position_age_s=60.0, now=snap.position.timestamp)
        self.assertEqual(frame.payload["position"], {"fixType": "3D", "hAccM": 1.5})
        snap = self._snapshot(fix_type=None)
        frame = build_telemetry_payload(snap, sequence=2, max_position_age_s=60.0, now=snap.position.timestamp)
        self.assertNotIn("position", frame.payload)

    def test_no_fresh_esp32_telemetry_means_no_stop_latch_claim(self):
        snap = self._snapshot(controller=None)
        frame = build_telemetry_payload(snap, sequence=1, max_position_age_s=60.0, now=snap.position.timestamp)
        self.assertNotIn("safety", frame.payload)

    def test_stop_latch_reports_the_esp32_latch_and_the_pi_estop(self):
        released = ControllerState(telemetry=controller_telemetry(safety_stop=False), telemetry_age_s=0.2)
        snap = self._snapshot(controller=released)
        frame = build_telemetry_payload(snap, sequence=1, max_position_age_s=60.0, now=snap.position.timestamp)
        self.assertEqual(frame.payload["safety"]["stopLatch"]["engaged"], False)

        latched = ControllerState(telemetry=controller_telemetry(safety_stop=True), telemetry_age_s=0.2)
        frame = build_telemetry_payload(self._snapshot(controller=latched), sequence=2, max_position_age_s=60.0)
        self.assertEqual(frame.payload["safety"]["stopLatch"]["engaged"], True)

        pi_estop = replace(state_with_fix().snapshot().safety, rule=ESTOP_RULE)
        frame = build_telemetry_payload(self._snapshot(controller=released, safety=pi_estop), sequence=3, max_position_age_s=60.0)
        self.assertEqual(frame.payload["safety"]["stopLatch"]["components"], {"piEmergencyStop": True, "esp32SafetyStop": False})

        stale = ControllerState(telemetry=controller_telemetry(safety_stop=False), telemetry_age_s=5.0)
        frame = build_telemetry_payload(self._snapshot(controller=stale), sequence=4, max_position_age_s=60.0)
        self.assertNotIn("safety", frame.payload)


class TestProbeAndEngineEvents(AsyncTestCase):
    def test_probe_is_answered_with_the_correlation_id(self):
        async def scenario():
            link, sio = make_link()
            await connect_and_auth(link)
            await sio.fire("PROBE", {"command": "PROBE", "correlationId": "c-1", "issuedAtMs": 1})
            return sio.events_named("PROBE_RESULT")

        sent = self.run_async(scenario())
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0]["correlationId"], "c-1")
        self.assertEqual(sent[0]["robotId"], "robotx-pi")

    def test_a_probe_before_authentication_is_not_answered(self):
        async def scenario():
            link, sio = make_link()
            link._ensure_client()  # handlers registered, never authenticated
            await sio.fire("PROBE", {"correlationId": "c-1"})
            return sio.events_named("PROBE_RESULT")

        self.assertEqual(self.run_async(scenario()), [])

    def test_every_engine_command_name_is_bound_and_command_is_not(self):
        async def scenario():
            link, sio = make_link()
            await connect_and_auth(link)
            return set(sio.handlers)

        handlers = self.run_async(scenario())
        for name in ("OFFER", "WITHDRAW", "RECALL", "ABORT_MISSION"):
            self.assertIn(name, handlers)
        self.assertNotIn("command", handlers)
        self.assertIn("command", ProtocolBinding(engine_command="command").engine_command_events())


if __name__ == "__main__":
    unittest.main()
