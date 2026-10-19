"""The temporary ESP32 reboot evidence collector (scripts/esp32_reboot_watch.py).

Pure: detection, extraction and redaction over dicts shaped like the running
agent's GET /state, and the bounded writer in a temporary directory. The HTTP
client is exercised with its opener stubbed; nothing contacts the agent.
"""

from __future__ import annotations

import copy
import json
import os
import stat
import tempfile
import unittest
from pathlib import Path

from scripts import esp32_reboot_watch as w

U = w.UNAVAILABLE


def live_like_state(uptime_ms=186099, received_at=1000.0, age=0.04, **controller):
    state = {
        "started_at": 500.0, "mode": "STOPPED", "safety": {"rule": "estop"},
        "communication": {"esp32": "DEGRADED", "esp32_detail": "ESP32 rebooted", "esp32_since": 900.0,
                          "esp32_last_rx_at": received_at},
        "health": {"components": {"esp32": {"status": "DEGRADED"}}},
        "controller": {
            "telemetry": {"uptime_ms": uptime_ms, "state": "IDLE", "block_reason": "NONE", "last_reject": "NONE",
                          "front": {"valid": True, "obstacle": True, "left_cm": 23.0, "right_cm": 141.2},
                          "rear": {"available": True, "obstacle": False, "sensor_fault": False, "mm": [438, 516, 980]},
                          "gates": {"forward_blocked": True}, "received_at": received_at},
            "telemetry_age_s": age, "reboot_count": 5, "reboot_latched": True, "ready_seen": True,
            "last_event": "I2CSCAN", "last_error": None, "bidirectional": False, "transmit_enabled": True,
            "motion_enabled": False, "motion_ready": False, "ping_rtt_ms": 43.7, "reset_status": "NONE",
            "proto": 2, "firmware": "rover-esp32",
            "counters": {"frames_ok": 53807, "telemetry": 31589, "diag": 15792, "gps": 6409, "events": 10,
                         "rejected_lines": 48, "too_long": 1, "bad_crc": 0, "commands_sent": 7, "acks": 7},
        },
        "controller_diag": {
            "SYSTEM": {"uptime_ms": uptime_ms - 300, "fw_build": "f10bb8f7208e", "proto": 2, "i2c_ready": True,
                       "tca_status": "OK", "pca_status": "OK", "motor_drive_status": "OK", "stall_count": 0,
                       "loop_max_ms": 40, "loop_max_source": "FRONT_US", "last_stall_ms": 0,
                       "last_stall_source": "NONE", "last_stall_uptime_ms": 0, "command_ever_received": False,
                       "link": {"rx_ok": 0, "rx_bad_crc": 0}, "received_at": received_at},
            "FRONT": {"front_left_valid": True, "front_right_valid": True, "front_left_health": "OK",
                      "front_right_health": "OK", "sensor_map_verified": False, "received_at": received_at},
            "REAR": {"rear_sensor_health": ["OK", "OK", "OK"], "rear_sensor_status": ["VALID"] * 3,
                     "rear_bus_err_total": [0, 0, 0], "rear_reinit_count": [0, 0, 0], "received_at": received_at},
        },
        "gps": {"fix": {"latitude": 12.9, "longitude": 77.5}},
    }
    state["controller"].update(controller)
    return state


def sample(t, state=None, error=None):
    return w.make_sample(t, state, error, U, None)


def names(events):
    return [e["event"] for e in events]


class TestExtraction(unittest.TestCase):
    def test_missing_fields_are_unavailable_not_guessed(self):
        state = live_like_state()
        del state["controller_diag"]["SYSTEM"]
        del state["controller"]["counters"]["too_long"]
        state["controller"]["telemetry"] = None
        s = w.extract_state(state)
        self.assertEqual(s["system"]["pca_status"], U)
        self.assertEqual(s["system"]["fw_link"], U)
        self.assertEqual(s["esp32_uptime_ms"], U)
        self.assertEqual(s["front"]["valid"], U)
        self.assertNotIn("too_long", s["counters"])
        self.assertIsNone(s["last_error"])  # present and null stays null

    def test_no_coordinates_are_extracted(self):
        tel = {"gps": {"status": "FIX", "age_s": 0.6, "fix": {"latitude": 12.9, "longitude": 77.5, "fix_type": "3D"}},
               "position": {"latitude": 12.9}, "health": {"status": "DEGRADED", "components": {"esp32": {"status": "X"}}}}
        blob = json.dumps([w.extract_telemetry(tel), w.extract_state(live_like_state())])
        self.assertNotIn("12.9", blob)
        self.assertNotIn("77.5", blob)


class TestRedaction(unittest.TestCase):
    def test_secret_keys_and_values(self):
        out = w.scrub({"auth_token": "abc", "detail": "token=s3cr3t and Bearer xyz.123",
                       "nested": [{"password": "p"}], "hash": "deadbeef" * 5 + "1234"})
        self.assertEqual(out["auth_token"], "[REDACTED]")
        self.assertEqual(out["nested"][0]["password"], "[REDACTED]")
        self.assertNotIn("s3cr3t", out["detail"])
        self.assertNotIn("xyz.123", out["detail"])
        self.assertEqual(out["hash"], "[REDACTED]")

    def test_ordinary_text_survives(self):
        text = "ESP32 rebooted (uptime went backwards (152832 -> 337 ms)); motion held until acknowledged"
        self.assertEqual(w.redact_text(text), text)
        self.assertEqual(w.redact_text("MOTOR_PWM_UNAVAILABLE_BECAUSE_OF_THINGS"), "MOTOR_PWM_UNAVAILABLE_BECAUSE_OF_THINGS")
        self.assertEqual(w.redact_text("f10bb8f7208e"), "f10bb8f7208e")
        self.assertTrue(w.redact_text("x" * 1000).endswith("[truncated]"))


class TestClient(unittest.TestCase):
    def test_refuses_non_loopback_and_odd_bases(self):
        for base in ("http://10.0.0.5:8000", "https://127.0.0.1:8000", "http://user:pw@127.0.0.1:8000",
                     "http://127.0.0.1:8000/state", "http://robotx-admin-dashboard.onrender.com"):
            with self.assertRaises(ValueError, msg=base):
                w.LoopbackGetClient(base)

    def test_refuses_paths_outside_the_allowlist(self):
        client = w.LoopbackGetClient()
        for path in ("/safety/clear", "/camera", "/mission/idle", "/state?x=1", "/health"):
            with self.assertRaises(ValueError, msg=path):
                client.get(path)

    def test_only_get_is_ever_sent(self):
        client = w.LoopbackGetClient()
        seen = []

        class Resp:
            def __enter__(self):
                return self

            def __exit__(self, *a):
                return False

            def read(self):
                return b'{"ok": true}'

        def fake_open(request, timeout):
            seen.append((request.get_method(), request.full_url, request.data))
            return Resp()

        client._opener.open = fake_open  # type: ignore[method-assign]
        self.assertEqual(client.get("/state"), ({"ok": True}, None))
        client.get("/telemetry")
        self.assertEqual(seen, [("GET", "http://127.0.0.1:8000/state", None),
                                ("GET", "http://127.0.0.1:8000/telemetry", None)])
        self.assertFalse(any(hasattr(client, verb) for verb in ("post", "put", "delete", "patch")))


class TestDetector(unittest.TestCase):
    def setUp(self):
        self.d = w.Detector(gap_s=1.0)
        self.assertEqual(self.d.observe(sample(0.0, live_like_state())), [])

    def test_steady_stream_is_quiet(self):
        for i in range(1, 6):
            st = live_like_state(uptime_ms=186099 + 500 * i, received_at=1000.0 + 0.5 * i)
            self.assertEqual(self.d.observe(sample(0.5 * i, st)), [])

    def test_reboot_with_a_silence(self):
        # Last frame at 1000.0 / 186099 ms; the agent goes stale ...
        ev = self.d.observe(sample(1.5, live_like_state(age=1.5)))
        self.assertEqual(names(ev), ["TELEMETRY_GAP_START"])
        self.assertEqual(ev[0]["last_uptime_ms"], 186099)
        # ... 15 s later a new boot at uptime 337 ms, counted by the agent.
        st = live_like_state(uptime_ms=337, received_at=1015.0, age=0.05, reboot_count=6,
                             link="DEGRADED", last_event="I2CSCAN")
        st["controller"]["counters"].update(events=11, rejected_lines=58)
        ev = self.d.observe(sample(16.0, st))
        self.assertEqual(names(ev), ["UPTIME_BACKWARDS", "TELEMETRY_GAP_END", "REBOOT_COUNT_CHANGED",
                                     "ESP32_EVENTS_RECEIVED", "COUNTERS_CHANGED"])
        back, gap, _, events, counters = ev
        self.assertAlmostEqual(back["est_boot_at"], 1015.0 - 0.337)
        self.assertEqual(gap["verdict"], "ESP32_RESTARTED")
        self.assertAlmostEqual(gap["silence_s"], 15.0)
        self.assertAlmostEqual(gap["silence_before_boot_s"], 14.663)
        self.assertEqual(events["events_delta"], 1)  # an I2CSCAN, no READY: READY lost
        self.assertEqual(counters["deltas"], {"rejected_lines": [48, 58]})
        self.assertFalse(counters["pi_transmitted"])

    def test_short_gap_seen_only_in_retrospect_and_ready(self):
        st = live_like_state(uptime_ms=200, received_at=1001.3, last_event="READY")
        st["controller"]["counters"]["events"] = 11
        ev = self.d.observe(sample(1.4, st))
        self.assertEqual(names(ev), ["UPTIME_BACKWARDS", "TELEMETRY_GAP_END", "READY_RECEIVED"])
        self.assertEqual(ev[1]["verdict"], "ESP32_RESTARTED")

    def test_silence_without_reset_is_kept_counting(self):
        self.d.observe(sample(1.5, live_like_state(age=1.5)))
        ev = self.d.observe(sample(3.2, live_like_state(uptime_ms=186099 + 3000, received_at=1003.0)))
        self.assertEqual(names(ev), ["TELEMETRY_GAP_END"])
        self.assertEqual(ev[0]["verdict"], "ESP32_KEPT_COUNTING")

    def test_pi_transmission_is_flagged(self):
        st = live_like_state(uptime_ms=186599, received_at=1000.5)
        st["controller"]["counters"].update(commands_sent=8)
        ev = self.d.observe(sample(0.5, st))
        self.assertEqual(names(ev), ["COUNTERS_CHANGED"])
        self.assertTrue(ev[0]["pi_transmitted"])

    def test_latch_link_error_and_i2c_changes(self):
        st = live_like_state(uptime_ms=186599, received_at=1000.5, reboot_latched=False, last_error="x")
        st["communication"]["esp32"] = "UP"
        st["controller_diag"]["SYSTEM"]["pca_status"] = "NOT_FOUND"
        ev = self.d.observe(sample(0.5, st))
        self.assertEqual(names(ev), ["REBOOT_LATCH_CHANGED", "LINK_STATUS_CHANGED", "LAST_ERROR_CHANGED",
                                     "DIAG_SYSTEM_CHANGED"])
        self.assertEqual(ev[3]["changed"], {"pca_status": ["OK", "NOT_FOUND"]})

    def test_api_outage_and_agent_restart(self):
        self.assertEqual(names(self.d.observe(sample(1.0, None, "URLError: refused"))), ["API_UNREACHABLE"])
        self.assertEqual(self.d.observe(sample(1.5, None, "URLError: refused")), [])
        st = live_like_state(uptime_ms=196099, received_at=1010.0)
        st["started_at"] = 600.0
        ev = self.d.observe(sample(10.0, st))
        self.assertEqual(names(ev)[:2], ["API_RESTORED", "AGENT_RESTARTED"])

    def test_rear_flapping_is_rate_limited(self):
        out = []
        for i in range(1, 9):
            st = live_like_state(uptime_ms=186099 + 500 * i, received_at=1000.0 + 0.5 * i)
            st["controller_diag"]["REAR"]["rear_sensor_health"] = ["OK", "FAULT" if i % 2 else "OK", "OK"]
            out += self.d.observe(sample(0.5 * i, st))
        self.assertEqual(names(out), ["REAR_CHANGED"])


class TestWriter(unittest.TestCase):
    def test_bounded_rotation_and_permissions(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / w.LOG_NAME
            writer = w.BoundedJsonlWriter(path, max_bytes=2000, backups=2)
            for i in range(200):
                writer.write({"kind": "sample", "t": i, "pad": "p" * 50, "token": "never"})
            writer.close()
            files = w.log_files(Path(tmp))
            self.assertEqual([p.name for p in files], [w.LOG_NAME + ".2", w.LOG_NAME + ".1", w.LOG_NAME])
            for p in files:
                self.assertLessEqual(p.stat().st_size, 2000)
                self.assertEqual(stat.S_IMODE(os.stat(p).st_mode), 0o600)
                self.assertNotIn("never", p.read_text())
            ts = [r["t"] for r in w.read_records(Path(tmp))]
            self.assertEqual(ts, sorted(ts))
            self.assertEqual(ts[-1], 199)

    def test_oversized_record_is_replaced_by_a_stub(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = w.BoundedJsonlWriter(Path(tmp) / w.LOG_NAME, max_bytes=1000, backups=0)
            writer.write({"kind": "pre_window", "t": 1, "samples": ["x" * 100] * 20})
            writer.close()
            rec = next(w.read_records(Path(tmp)))
            self.assertEqual(rec["kind"], "dropped")


if __name__ == "__main__":
    unittest.main()
