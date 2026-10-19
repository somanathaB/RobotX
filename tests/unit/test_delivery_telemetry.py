"""Delivery-path telemetry and the gaps closed with it (2026-10-19).

- TELEMETRY `status` for a Rover stopped with a mission held (PAUSED).
- TELEMETRY `safety`: the contract's keys, only from fresh ESP32 TELEMETRY.
- TELEMETRY `faults[]`: the complete active set, contract vocabulary, onset.
- An immediate TELEMETRY frame on a safety or fault transition.
- The OFFER signature canonical form, pinned to the backend's own vector.
- The agent: fix wander is not a heading; sensing holds are bounded.

Everything here is SIMULATED: fake backend socket (`FakeSio`), synthetic fixes,
the firmware model. Nothing is evidence about the rover's hardware.
"""

from __future__ import annotations

import asyncio
import json
import time
import unittest
from dataclasses import replace
from pathlib import Path

from robotx.communication import engine
from robotx.communication.backend_link import _safety_key
from robotx.communication.protocol import (
    backend_status,
    build_telemetry_payload,
    faults_block,
    safety_block,
)
from robotx.control.safety import ESTOP_RULE
from robotx.diagnostics.faults import Fault, FaultTracker, active_faults
from robotx.esp32.state import ControllerState, ControllerTelemetry
from robotx.hardware.gps import GpsReading, GPSStatus
from robotx.localization.position import HeadingSource
from robotx.mission.mission import ActiveMission, MissionSegment, MissionStatus
from robotx.perception.types import PerceptionResult, PerceptionStatus
from robotx.state.robot_state import Esp32LinkStatus, OperatingMode
from tests.fixtures.esp32 import CLEAR_FRONT, REAL_TELEMETRY_FIELDS
from tests.unit.test_agent import FakePerception
from tests.unit.test_backend_link import connect_and_auth, state_with_fix
from tests.unit.test_engine_offer import parsed_offer
from tests.unit.test_readiness_telemetry import link_with

VECTOR = Path(__file__).resolve().parents[1] / "fixtures" / "backend_offer_signature_vector.json"
FORBIDDEN = ("capabilit", "certification", "attestedcapabilit", "hazmatcertified", "coldchain", "clearance")


def telemetry(**overrides):
    fields = dict(REAL_TELEMETRY_FIELDS, **CLEAR_FRONT)
    fields.update(motor_drive_available=True, safety_stop=False, command_timeout=False, state="IDLE",
                  block_reason="NONE", rear_sensor_fault=False, rear_obstacle=False)
    fields.update(overrides)
    return ControllerTelemetry.from_frame(fields, time.time())


def controller(age_s=0.2, *, reboot_latched=False, motion_enabled=True, **overrides):
    return ControllerState(telemetry=telemetry(**overrides), telemetry_age_s=age_s,
                           reboot_latched=reboot_latched, motion_enabled=motion_enabled)


def clear_perception():
    fake = FakePerception()
    fake.set_clear()
    return fake.latest()


def mission(status=MissionStatus.TO_PICKUP, **kw):
    from robotx.communication.engine import offer_to_mission

    return ActiveMission(mission=offer_to_mission(parsed_offer()), status=status,
                         segment=MissionSegment.TO_PICKUP, accepted_at=time.time(), custody_required=True, **kw)


def healthy_snapshot(**changes):
    snap = state_with_fix().snapshot()
    comm = replace(snap.communication, esp32=Esp32LinkStatus.UP)
    snap = replace(snap, communication=comm, controller=controller(), perception=clear_perception())
    return replace(snap, **changes)


def faults_of(snap, **kw):
    args = dict(estop_engaged=False, esp32_attached=True, motion_enabled=True, controller_max_age_s=1.0)
    args.update(kw)
    return [f.code for f in active_faults(snap, **args)]


def all_keys(value):
    if isinstance(value, dict):
        for k, v in value.items():
            yield k
            yield from all_keys(v)
    elif isinstance(value, list):
        for v in value:
            yield from all_keys(v)


# --- status ------------------------------------------------------------------------


class TestStatus(unittest.TestCase):
    def test_stopped_with_a_mission_held_is_paused_not_idle(self):
        snap = healthy_snapshot(mode=OperatingMode.STOPPED, mission=mission(interrupted_by="emergency stop: x"))
        self.assertEqual(backend_status(snap), "PAUSED")
        self.assertEqual(build_telemetry_payload(snap, sequence=1, max_position_age_s=5.0).payload["status"], "PAUSED")

    def test_stopped_with_no_mission_or_a_finished_one_is_idle(self):
        self.assertEqual(backend_status(healthy_snapshot(mode=OperatingMode.STOPPED)), "IDLE")
        aborted = healthy_snapshot(mode=OperatingMode.STOPPED, mission=mission(MissionStatus.ABORTED))
        self.assertEqual(backend_status(aborted), "IDLE")

    def test_every_other_mode_keeps_its_mapping(self):
        for mode, wire in ((OperatingMode.IDLE, "IDLE"), (OperatingMode.AUTO, "ACTIVE"),
                           (OperatingMode.PAUSED, "PAUSED"), (OperatingMode.ERROR, "ERROR")):
            self.assertEqual(backend_status(healthy_snapshot(mode=mode, mission=mission())), wire)


# --- safety ------------------------------------------------------------------------


class TestSafetyBlock(unittest.TestCase):
    def test_contract_keys_from_fresh_controller_telemetry(self):
        snap = healthy_snapshot()
        block = safety_block(snap, 1.0)
        self.assertEqual(set(block), {"estop", "observedAtMs", "safetyStop", "motorDriveAvailable",
                                      "commandTimeout", "stopLatch"})
        self.assertEqual(block["estop"], {"present": False})
        self.assertEqual(block["safetyStop"], {"active": False, "latched": False, "reason": None})
        self.assertIs(block["motorDriveAvailable"], True)
        self.assertIs(block["commandTimeout"], False)
        self.assertEqual(block["observedAtMs"], int(round(snap.controller.telemetry.received_at * 1000)))

    def test_a_proximity_stop_is_reported_as_one_never_as_an_estop(self):
        snap = healthy_snapshot(controller=controller(front_obstacle=True, safety_stop=True))
        block = safety_block(snap, 1.0)
        self.assertEqual(block["safetyStop"], {"active": True, "latched": True, "reason": "FRONT_OBSTACLE"})
        self.assertEqual(block["estop"], {"present": False})
        self.assertTrue(block["stopLatch"]["components"]["esp32SafetyStop"])

    def test_a_latch_held_after_the_obstacle_left_is_latched_not_active(self):
        block = safety_block(healthy_snapshot(controller=controller(safety_stop=True)), 1.0)
        self.assertEqual(block["safetyStop"], {"active": False, "latched": True, "reason": None})

    def test_drive_and_watchdog_as_reported(self):
        block = safety_block(healthy_snapshot(controller=controller(motor_drive_available=False,
                                                                     command_timeout=True)), 1.0)
        self.assertIs(block["motorDriveAvailable"], False)
        self.assertIs(block["commandTimeout"], True)

    def test_stale_or_missing_controller_telemetry_sends_no_safety_block_at_all(self):
        for ctl in (None, controller(age_s=5.0), ControllerState()):
            self.assertIsNone(safety_block(healthy_snapshot(controller=ctl), 1.0))
            frame = build_telemetry_payload(healthy_snapshot(controller=ctl), sequence=1, max_position_age_s=5.0)
            self.assertNotIn("safety", frame.payload)

    def test_the_pi_latch_rides_the_stop_latch(self):
        pi = replace(healthy_snapshot().safety, rule=ESTOP_RULE)
        block = safety_block(healthy_snapshot(safety=pi), 1.0)
        self.assertTrue(block["stopLatch"]["engaged"])


# --- faults ------------------------------------------------------------------------


class TestActiveFaults(unittest.TestCase):
    def test_a_healthy_rover_has_none(self):
        self.assertEqual(faults_of(healthy_snapshot()), [])

    def test_each_condition_names_its_fault(self):
        cases = {
            "PI_EMERGENCY_STOP_LATCHED": (healthy_snapshot(), dict(estop_engaged=True)),
            "AGENT_ERROR": (healthy_snapshot(mode=OperatingMode.ERROR), {}),
            "ESP32_LINK_DISABLED": (healthy_snapshot(controller=None), dict(esp32_attached=False)),
            "ESP32_CONFIG_INVALID": (healthy_snapshot(controller=None),
                                     dict(esp32_attached=False, esp32_config_error="bad")),
            "PI_MOTION_DISABLED": (healthy_snapshot(), dict(motion_enabled=False)),
            "ESP32_LINK_STALE": (healthy_snapshot(communication=replace(
                healthy_snapshot().communication, esp32=Esp32LinkStatus.STALE), controller=controller(age_s=9)), {}),
            "ESP32_REBOOT_UNACKNOWLEDGED": (healthy_snapshot(controller=controller(reboot_latched=True)), {}),
            "ESP32_TELEMETRY_STALE": (healthy_snapshot(controller=controller(age_s=3.0)), {}),
            "MOTOR_DRIVE_UNAVAILABLE": (healthy_snapshot(controller=controller(motor_drive_available=False)), {}),
            "FRONT_SENSING_INVALID": (healthy_snapshot(controller=controller(front_valid=False)), {}),
            "ESP32_SAFETY_STOP_LATCHED": (healthy_snapshot(controller=controller(safety_stop=True)), {}),
            "ESP32_COMMAND_TIMEOUT": (healthy_snapshot(controller=controller(command_timeout=True)), {}),
            "REAR_SENSOR_FAULT": (healthy_snapshot(controller=controller(rear_sensor_fault=True)), {}),
            "GNSS_NO_FIX": (healthy_snapshot(gps=GpsReading(GPSStatus.NO_FIX)), {}),
            "GNSS_UNAVAILABLE": (healthy_snapshot(gps=GpsReading(GPSStatus.UNAVAILABLE)), {}),
            "GNSS_ACCURACY_INSUFFICIENT": (healthy_snapshot(), dict(position_issue="h_acc 9 m")),
            "PERCEPTION_NO_FRAME": (healthy_snapshot(
                perception=PerceptionResult.unavailable(PerceptionStatus.NO_FRAME)), {}),
            "MISSION_INTERRUPTED": (healthy_snapshot(mode=OperatingMode.STOPPED,
                                                     mission=mission(interrupted_by="emergency stop: x")), {}),
            "MISSION_PAUSED_OBSTACLE": (healthy_snapshot(mode=OperatingMode.PAUSED, mission=mission(),
                                                         last_error="obstacle: blocked for 30s"), {}),
            "MISSION_PAUSED_CONTROLLER": (healthy_snapshot(mode=OperatingMode.PAUSED, mission=mission(),
                                                           last_error="controller: DRIVE gated"), {}),
            "MISSION_PAUSED_BACKEND_LINK": (healthy_snapshot(mode=OperatingMode.PAUSED, mission=mission(),
                                                             last_error="backend: link lost"), {}),
            "MISSION_PAUSED": (healthy_snapshot(mode=OperatingMode.PAUSED, mission=mission()), {}),
        }
        for code, (snap, kw) in cases.items():
            self.assertIn(code, faults_of(snap, **kw), code)

    def test_stale_controller_telemetry_reports_staleness_not_old_readings(self):
        codes = faults_of(healthy_snapshot(controller=controller(age_s=3.0, safety_stop=True, front_valid=False)))
        self.assertIn("ESP32_TELEMETRY_STALE", codes)
        self.assertNotIn("ESP32_SAFETY_STOP_LATCHED", codes)
        self.assertNotIn("FRONT_SENSING_INVALID", codes)

    def test_severities_and_components_are_the_contracts(self):
        with self.assertRaises(ValueError):
            Fault("X", "WARNING", "PI")
        with self.assertRaises(ValueError):
            Fault("X", "BLOCKING", "BATTERY")
        operator_pause = active_faults(healthy_snapshot(mode=OperatingMode.PAUSED, mission=mission()),
                                       estop_engaged=False, esp32_attached=True, motion_enabled=True,
                                       controller_max_age_s=1.0)
        self.assertEqual([(f.severity, f.component) for f in operator_pause], [("INFO", "PI")])

    def test_no_power_fault_is_ever_claimed(self):
        snap = healthy_snapshot(mode=OperatingMode.ERROR, gps=GpsReading(GPSStatus.NO_FIX))
        faults = active_faults(snap, estop_engaged=True, esp32_attached=False, motion_enabled=False,
                               controller_max_age_s=1.0)
        self.assertFalse([f for f in faults if f.component == "POWER"])


class TestFaultTracker(unittest.TestCase):
    def test_since_is_the_onset_and_a_cleared_fault_starts_again(self):
        tracker, fault = FaultTracker(), Fault("GNSS_NO_FIX", "BLOCKING", "GNSS")
        first = tracker.report([fault], now=100.0)
        self.assertEqual(first, [{"code": "GNSS_NO_FIX", "severity": "BLOCKING", "component": "GNSS",
                                  "active": True, "sinceMs": 100000}])
        self.assertEqual(tracker.report([fault], now=105.0)[0]["sinceMs"], 100000)
        self.assertEqual(tracker.report([], now=106.0), [])
        self.assertEqual(tracker.report([fault], now=107.0)[0]["sinceMs"], 107000)


class TestFaultsOnTheWire(unittest.TestCase):
    def test_none_is_omitted_and_empty_is_sent_as_measured_none(self):
        snap = healthy_snapshot()
        self.assertNotIn("faults", build_telemetry_payload(snap, sequence=1, max_position_age_s=5.0).payload)
        self.assertEqual(build_telemetry_payload(snap, sequence=1, max_position_age_s=5.0, faults=[]).payload["faults"], [])

    def test_an_entry_outside_the_vocabulary_is_refused(self):
        with self.assertRaises(ValueError):
            faults_block([{"code": "X", "severity": "BAD", "component": "PI", "sinceMs": 1}])

    def test_no_forbidden_key_anywhere_in_a_full_frame(self):
        snap = healthy_snapshot(controller=controller(front_obstacle=True, safety_stop=True))
        faults = FaultTracker().report(active_faults(snap, estop_engaged=True, esp32_attached=True,
                                                     motion_enabled=False, controller_max_age_s=1.0))
        payload = build_telemetry_payload(snap, sequence=1, max_position_age_s=5.0, readiness_blockers=["ESTOP_LATCHED"],
                                          faults=faults).payload
        keys = [k.lower().replace("_", "").replace("-", "") for k in all_keys(payload)]
        self.assertFalse([k for k in keys if any(bad in k for bad in FORBIDDEN)], keys)
        self.assertNotIn(None, [payload[k] for k in payload])

    def test_the_agents_report_rides_the_frame_and_a_stalled_agent_omits_it(self):
        link, sio = link_with(agent_alive=lambda: True)
        link._faults = lambda _snap: [{"code": "GNSS_NO_FIX", "severity": "BLOCKING", "component": "GNSS",
                                       "active": True, "sinceMs": 1}]

        async def one():
            await connect_and_auth(link)
            await link._publish_telemetry()
            return [p for e, p in sio.emitted if e == "TELEMETRY"][-1]

        self.assertEqual(asyncio.run(one())["faults"][0]["code"], "GNSS_NO_FIX")
        link2, sio2 = link_with(agent_alive=lambda: False)
        link2._faults = lambda _snap: []

        async def two():
            await connect_and_auth(link2)
            await link2._publish_telemetry()
            return [p for e, p in sio2.emitted if e == "TELEMETRY"][-1]

        self.assertNotIn("faults", asyncio.run(two()))


# --- transition frames ---------------------------------------------------------------


class TestTransitionTelemetry(unittest.TestCase):
    def test_a_safety_change_is_sent_at_once_and_an_unchanged_one_is_not(self):
        faults = []
        link, sio = link_with(agent_alive=lambda: True)
        link._faults = lambda _snap: list(faults)

        async def scenario():
            await connect_and_auth(link)
            await link._publish_telemetry()
            link._last_telemetry_t = 0.0
            await link._publish_telemetry_on_transition(10.0)           # nothing changed
            unchanged = len([e for e, _ in sio.emitted if e == "TELEMETRY"])
            faults.append({"code": "PI_EMERGENCY_STOP_LATCHED", "severity": "BLOCKING", "component": "PI",
                           "active": True, "sinceMs": 1})
            await link._publish_telemetry_on_transition(10.1)           # changed: sent
            await link._publish_telemetry_on_transition(10.2)           # changed again? no; and too soon
            faults.clear()
            await link._publish_telemetry_on_transition(10.3)           # changed, but within 0.5 s
            frames = [p for e, p in sio.emitted if e == "TELEMETRY"]
            return unchanged, frames

        unchanged, frames = asyncio.run(scenario())
        self.assertEqual(unchanged, 1)
        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[1]["faults"][0]["code"], "PI_EMERGENCY_STOP_LATCHED")
        self.assertGreater(frames[1]["sequence"], frames[0]["sequence"])
        self.assertEqual(link.stats["telemetry_on_transition"], 1)

    def test_the_key_ignores_timing_and_position(self):
        a = {"status": "IDLE", "timestamp": 1, "lat": 1.0, "safety": {"observedAtMs": 1, "estop": {"present": False}}}
        b = {"status": "IDLE", "timestamp": 2, "lat": 2.0, "safety": {"observedAtMs": 2, "estop": {"present": False}}}
        self.assertEqual(_safety_key(a), _safety_key(b))
        self.assertNotEqual(_safety_key(a), _safety_key(dict(a, status="PAUSED")))


# --- signature canonical form, pinned to the backend --------------------------------


class TestBackendSignatureVector(unittest.TestCase):
    """`docs/contracts/fixtures/offer-signature-test-vector.json`, produced by the
    backend's real OFFER path (RobotX-Admin-Dashboard feature/dashboard 35092e8).
    Its key is a PUBLIC test vector, never a deployment key."""

    def setUp(self):
        self.vector = json.loads(VECTOR.read_text(encoding="utf-8"))
        self.wire = json.loads(self.vector["wire"])
        self.key = self.vector["key"].encode("utf-8")

    def test_the_canonical_string_is_byte_identical(self):
        self.assertEqual(engine.canonical_string(self.wire), self.vector["canonical"])

    def test_the_signature_matches_and_verifies(self):
        self.assertEqual(engine.sign(self.wire, self.key), self.vector["signature"])
        self.assertIsNone(engine.verify_signature(self.wire, self.key))

    def test_any_change_to_a_signed_byte_fails(self):
        tampered = json.loads(self.vector["wire"])
        tampered["payload"]["stopSequence"][0]["lat"] = 12.9082
        self.assertIsNotNone(engine.verify_signature(tampered, self.key))
        self.assertIsNotNone(engine.verify_signature(self.wire, b"another-key-of-at-least-32-bytes!!"))


# --- the agent: heading and sensing holds --------------------------------------------


class TestAgentHeadingAndSensingHolds(unittest.TestCase):
    def rig(self):
        from tests.unit.test_obstacle_recovery import Rig

        rig = Rig()
        rig.start()
        return rig

    def test_a_held_rover_never_takes_a_heading_from_fix_wander(self):
        from tests.unit.test_obstacle_recovery import HERE

        rig = self.rig()
        rig.agent.pause_mission("test: hold still")
        rig.agent.position_estimator.reset()
        lat, lon = HERE
        for i in range(30):                                   # ~0.5 m of wander per tick
            rig.agent.gps.set_fix(lat + 0.0000045 * i, lon)
            snap = rig.step()
        self.assertIsNone(snap.position.heading_deg)
        self.assertIs(snap.position.heading_source, HeadingSource.NONE)

    def test_perception_lost_mid_leg_pauses_after_the_hold_limit(self):
        rig = self.rig()
        rig.agent.progress.clock = rig.clock
        rig.agent.progress.reset()
        rig.agent.perception.result = PerceptionResult.unavailable(PerceptionStatus.NO_FRAME)
        mark = rig.mark()
        limit = rig.agent.progress.cfg.hold_assist_after_s
        early = rig.run(limit - 1.0)
        self.assertTrue(all(s.mode is OperatingMode.AUTO for s in early))
        snaps = rig.run(2.0)
        self.assertIs(snaps[-1].mode, OperatingMode.PAUSED)
        self.assertIn("perception unavailable (NO_FRAME)", snaps[-1].last_error)
        self.assertEqual(rig.drives(mark), [], "DRIVE without usable perception")


if __name__ == "__main__":
    unittest.main()
