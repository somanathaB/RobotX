"""OFFER -> assignment -> navigation -> custody -> completion, end to end, SIMULATED.

What is real and what is simulated
----------------------------------
- REAL: `RobotAgent` and its own loop, mission admission (`assess_offer`,
  route admission), `MissionManager`, navigator, decision layer, safety gate,
  obstacle/progress/applied-motion monitors, `Esp32Link`, and `BackendLink`
  with its genuine `socketio.AsyncClient`: Engine.IO handshake, AUTH, OFFER
  signature/fence/sequence verification, OFFER_ACCEPT, HEARTBEAT lease,
  CUSTODY_EVENT, TASK_COMPLETE evidence check, COMMAND handling.
- SIMULATED: the backend (`ContractServer`, a loopback `socketio.AsyncServer`
  speaking this implementation's reading of the contract -- not RobotX); the
  OFFER (signed with the test-only key); the ESP32, wheels, GPS and camera
  (`tests.fixtures.sim_rover`); and the custody handover (the operator
  confirmation the local API would record).

A pass proves the Pi's software carries a backend-style mission through every
stage against simulated feedback. It does NOT prove physical movement or a
delivery: no motor, wheel, GPS receiver, ESP32 or RobotX backend took part.
"""

from __future__ import annotations

import asyncio
import os
import tempfile
import time
import unittest

from robotx.communication.backend_link import BackendConfig, BackendLink
from robotx.communication.protocol import ProtocolBinding
from robotx.communication.token_store import TokenStore
from robotx.mission.mission import MissionStatus
from robotx.state.robot_state import OperatingMode
from tests.fixtures import engine as efx
from tests.fixtures.sim_rover import SimulatedRover, short_route
from tests.fixtures.task_assign import offset, wire_path
from tests.integration.test_socketio_transport import (  # noqa: F401
    ISSUED_TOKEN, VALID_PAIRING_CODE, ContractServer, setUpModule, wait_for,
)
from robotx.communication import engine

START = (12.9700000, 77.5900000)
COMMITMENT = "c-sim-1"


def offer_envelope(start=START, *, commitment_id=COMMITMENT, task_id="T-SIM-1"):
    pickup, drop, to_pickup, to_drop = short_route(start)
    now_ms = int(time.time() * 1000)
    stops = [
        {"sequence": 0, "stopType": "FIXTURE_STOP_A", "siteId": "sim-a", "lat": pickup[0], "lon": pickup[1],
         "projectedArrivalMs": now_ms + 30_000, "departureMs": now_ms + 60_000,
         "path": wire_path(to_pickup), "pathDistanceMeters": 24.0},
        {"sequence": 1, "stopType": "FIXTURE_STOP_B", "siteId": "sim-b", "lat": drop[0], "lon": drop[1],
         "projectedArrivalMs": now_ms + 90_000, "departureMs": now_ms + 120_000,
         "path": wire_path(to_drop), "pathDistanceMeters": 24.0},
    ]
    payload = efx.offer_payload(commitment_id=commitment_id, task_id=task_id, stop_sequence=stops)
    return efx.envelope(commitment_id=commitment_id, payload=payload)


class SimulatedDeliveryCase(unittest.TestCase):
    def run_scenario(self, scenario, *, start=START, **link_overrides):
        async def wrapper():
            # The backend accepts the session token it issued (robot.handler.js),
            # so a reconnect re-authenticates as it would against RobotX.
            server = ContractServer(accept_token=ISSUED_TOKEN)
            await server.start()
            tmp = tempfile.TemporaryDirectory()
            rover = SimulatedRover(start, robot_id=efx.ROBOT_ID)
            cfg = BackendConfig(**{**dict(
                enabled=True, server_url=server.url, robot_id=efx.ROBOT_ID, binding=ProtocolBinding(),
                pairing_code=VALID_PAIRING_CODE, command_signing_key=efx.TEST_SIGNING_KEY,
                telemetry_interval_s=0.2, heartbeat_interval_s=0.2, auth_timeout_s=3.0,
            ), **link_overrides})
            link = BackendLink(cfg, rover.agent.state, rover.agent, agent_alive=rover.agent.is_alive,
                               readiness=rover.agent.readiness_blockers, faults=rover.agent.fault_report,
                               token_store=TokenStore(os.path.join(tmp.name, "session.json")))
            rover.agent.backend = link
            try:
                await rover.start()
                await link.start()
                self.assertTrue(await wait_for(lambda: link.connected, timeout=5.0), "never authenticated")
                ready = await rover.run(5.0, until=lambda: rover.agent.readiness_blockers() == [])
                self.assertTrue(ready, rover.agent.readiness_blockers())
                return await scenario(server, rover, link)
            finally:
                await link.stop()
                await rover.stop()
                await server.stop()
                tmp.cleanup()

        return asyncio.run(wrapper())


async def accept_offer(test, server, rover):
    await server.send_engine(offer_envelope())
    accepted = await rover.run(5.0, until=lambda: bool(server.received["OFFER_ACCEPT"]))
    test.assertTrue(accepted, (server.received["OFFER_REJECT"], rover.agent.readiness_blockers()))
    test.assertIs(rover.agent.mode, OperatingMode.AUTO)


async def drive_to(test, rover, status, seconds=25.0):
    reached = await rover.run(seconds, until=lambda: rover.mission() is not None and rover.mission().status is status)
    snap = rover.agent.state.snapshot()
    test.assertTrue(reached, (status, snap.mode, snap.mission and snap.mission.status, snap.last_error,
                              snap.motion_intent.reason, snap.safety.rule, snap.navigation.status,
                              rover.firmware.applied, len(rover.drives())))


class TestSimulatedDelivery(SimulatedDeliveryCase):
    def test_offer_to_completion_with_an_estop_and_explicit_resume_on_the_way(self):
        async def scenario(server, rover, link):
            await accept_offer(self, server, rover)
            self.assertTrue(await rover.run(2.0, until=lambda: any(
                hb.get("commitmentId") == COMMITMENT for hb in server.received["HEARTBEAT"])))
            self.assertTrue(rover.drives(), "accepted but never drove")

            await drive_to(self, rover, MissionStatus.AT_PICKUP)
            rover.agent.record_custody("ACQUIRED", source="OPERATOR_CONFIRMED")
            self.assertTrue(await rover.run(3.0, until=lambda: any(
                e.get("kind") == "ACQUIRED" for e in server.received["CUSTODY_EVENT"])))
            await drive_to(self, rover, MissionStatus.TO_DROP, seconds=3.0)
            await rover.run(1.5)                                      # part-way down the drop leg

            # --- emergency stop mid-leg: no motion, no lease renewal ------------
            rover.agent.emergency_stop("operator (simulated)")
            await rover.run(0.5)
            mark, beats = rover.mark(), len(server.received["HEARTBEAT"])
            await rover.run(2.0)
            self.assertEqual(rover.drives(mark), [], "DRIVE while the e-stop was latched")
            latched = server.received["HEARTBEAT"][beats:]
            self.assertTrue(latched and all(hb == {} for hb in latched), latched)
            self.assertTrue(rover.mission().interrupted)

            # --- clear: PAUSED, still no motion; the lease is renewed again -----
            rover.agent.clear_emergency_stop("operator (simulated)")
            mark, beats = rover.mark(), len(server.received["HEARTBEAT"])
            await rover.run(2.0)
            self.assertIs(rover.agent.mode, OperatingMode.PAUSED)
            self.assertEqual(rover.drives(mark), [], "DRIVE after clearing, before any RESUME")
            self.assertTrue(any(hb.get("commitmentId") == COMMITMENT for hb in server.received["HEARTBEAT"][beats:]))

            # --- the backend's RESUME (bound to the live commitment) ------------
            await server.send_command({"commandId": "resume-1", "type": "RESUME", "robotId": efx.ROBOT_ID,
                                       "timestamp": int(time.time() * 1000), "commitmentId": COMMITMENT})
            self.assertTrue(await rover.run(3.0, until=lambda: rover.agent.mode is OperatingMode.AUTO))

            await drive_to(self, rover, MissionStatus.AT_DROP)
            rover.agent.record_custody("RELEASED", source="OPERATOR_CONFIRMED")
            done = await rover.run(5.0, until=lambda: bool(server.received["TASK_COMPLETE"]))
            self.assertTrue(done, link.describe()["stats"])
            self.assertIs(rover.mission().status, MissionStatus.COMPLETE)
            self.assertEqual([e["kind"] for e in server.received["CUSTODY_EVENT"]], ["ACQUIRED", "RELEASED"])
            self.assertEqual(len(server.received["TASK_COMPLETE"]), 1)
            self.assertGreaterEqual(rover.min_drive_units, 0, "a reverse or pivot DRIVE was written")
            self.assertGreater(link.stats["lease_renewals_withheld"], 0)
            check_telemetry_contract(self, server.received["TELEMETRY"])
            statuses = [f["status"] for f in server.received["TELEMETRY"]]
            # Driving, held by the e-stop with the Leg on board, then idle after completion.
            for expected in ("ACTIVE", "PAUSED", "IDLE"):
                self.assertIn(expected, statuses)

        self.run_scenario(scenario)

    def test_a_controller_that_applies_nothing_pauses_the_mission_with_the_reason(self):
        async def scenario(server, rover, link):
            rover.firmware.applied_zero = True
            await accept_offer(self, server, rover)
            paused = await rover.run(5.0, until=lambda: rover.agent.mode is OperatingMode.PAUSED)
            self.assertTrue(paused)
            error = rover.agent.state.snapshot().last_error
            self.assertTrue(error.startswith("controller: DRIVE"), error)
            mark = rover.mark()
            await rover.run(1.5)
            self.assertEqual(rover.drives(mark), [])
            self.assertIs(rover.mission().status, MissionStatus.TO_PICKUP)

        self.run_scenario(scenario)

    def test_an_offer_whose_route_starts_elsewhere_is_rejected(self):
        async def scenario(server, rover, link):
            await server.send_engine(offer_envelope(start=offset(START, north_m=-150.0)))
            got = await rover.run(5.0, until=lambda: bool(server.received["OFFER_REJECT"]))
            self.assertTrue(got)
            self.assertEqual(server.received["OFFER_REJECT"][-1]["reason"], "UNSUPPORTED_MISSION: ROUTE_START_TOO_FAR")
            self.assertEqual(server.received["OFFER_ACCEPT"], [])
            self.assertIsNone(rover.mission())
            self.assertEqual(rover.drives(), [])

        self.run_scenario(scenario)



# --- the TELEMETRY contract, checked on every frame the simulated backend received --

STATUSES = {"IDLE", "ACTIVE", "PAUSED", "ERROR", "ISSUES", "RETURNING", "CHARGING"}
SAFETY_KEYS = {"estop", "observedAtMs", "safetyStop", "motorDriveAvailable", "commandTimeout", "stopLatch"}
READINESS = {"ESTOP_LATCHED", "AGENT_ERROR", "NO_MOTOR_LINK", "NO_POSITION_FIX", "NO_CUSTODY_SENSING"}
FORBIDDEN = ("capabilit", "certification", "hazmatcertified", "coldchain", "clearance")


def _keys(value):
    if isinstance(value, dict):
        for k, v in value.items():
            yield k
            yield from _keys(v)
    elif isinstance(value, list):
        for v in value:
            yield from _keys(v)


def check_telemetry_contract(test, frames):
    """PHYSICAL_ROBOTX_BACKEND_CONTRACT section 4 / P2B1 handoff sections 5 and 16."""

    test.assertTrue(frames, "no TELEMETRY reached the backend")
    sequences = [f["sequence"] for f in frames]
    test.assertEqual(sequences, sorted(set(sequences)), "sequence must strictly increase")
    for f in frames:
        test.assertIsInstance(f["timestamp"], int)
        test.assertIn(f["status"], STATUSES)
        test.assertNotIn(None, f.values())
        test.assertEqual("lat" in f, "lon" in f)
        if "lat" in f:
            test.assertEqual(set(f["position"]), {"fixType", "hAccM"})
        if "safety" in f:
            test.assertLessEqual(set(f["safety"]), SAFETY_KEYS)
            test.assertEqual(f["safety"]["estop"], {"present": False})
        test.assertLessEqual(set(f["readiness"]["blockers"]), READINESS)
        for fault in f["faults"]:
            test.assertIn(fault["severity"], {"INFO", "DEGRADED", "BLOCKING", "CRITICAL"})
            test.assertIn(fault["component"], {"PI", "ESP32", "LINK", "SENSOR", "CAMERA", "MOTOR", "GNSS", "POWER"})
            test.assertIs(fault["active"], True)
        keys = [k.lower().replace("_", "") for k in _keys(f)]
        test.assertFalse([k for k in keys if any(bad in k for bad in FORBIDDEN)])


def resigned(env, **changes):
    env = dict(env, **changes)
    env["signature"] = engine.sign(env, efx.TEST_SIGNING_KEY.encode("utf-8"))
    return env


async def first_frame(server, rover, predicate, seconds=5.0):
    """The first TELEMETRY frame (index, payload) satisfying `predicate`, stepping the world."""

    found = {}

    def look():
        for i, f in enumerate(server.received["TELEMETRY"]):
            if predicate(f):
                found["hit"] = (i, f)
                return True
        return False

    await rover.run(seconds, until=look)
    return found.get("hit")


class TestGenuineOfferAdmission(SimulatedDeliveryCase):
    """A signed OFFER is admitted only through every check; nothing else moves the rover."""

    def test_tampered_expired_misaddressed_and_replayed_offers_are_never_acted_on(self):
        async def scenario(server, rover, link):
            good = offer_envelope()
            tampered = dict(good, payload=dict(good["payload"], taskId="T-FORGED"))
            expired = resigned(offer_envelope(commitment_id="c-old"), notValidAfter=efx.iso(time.time() - 5))
            elsewhere = resigned(offer_envelope(commitment_id="c-other"), agentId="robotx-someone-else")
            unsigned = dict(offer_envelope(commitment_id="c-unsigned"), signature="")
            for env in (tampered, expired, unsigned):
                await server.send_engine(env)
            await server.send_engine(elsewhere)
            await rover.run(1.5)
            self.assertEqual(server.received["COMMAND_ACK"], [], "a non-admitted envelope was acknowledged")
            self.assertEqual(server.received["OFFER_ACCEPT"] + server.received["OFFER_REJECT"], [])
            self.assertIsNone(rover.mission())
            self.assertEqual(rover.drives(), [])
            self.assertGreaterEqual(link.stats["engine_commands_not_admitted"], 4)

            # The genuine one is admitted, acknowledged and accepted -- once.
            await server.send_engine(good)
            self.assertTrue(await rover.run(5.0, until=lambda: bool(server.received["OFFER_ACCEPT"])))
            acks = len(server.received["COMMAND_ACK"])
            self.assertEqual(server.received["COMMAND_ACK"][-1],
                             {"outboxId": good["outboxId"], "fence": good["fence"], "authorityEpoch": None})
            # An exact redelivery is re-acknowledged and nothing else; a new outbox
            # replaying the same sequence is a duplicate and is not admitted at all.
            await server.send_engine(good)
            await server.send_engine(dict(good, outboxId="ob-replay"))
            await rover.run(1.0)
            self.assertEqual(len(server.received["COMMAND_ACK"]), acks + 1)
            self.assertEqual(len(server.received["OFFER_ACCEPT"]), 1)
            self.assertEqual(rover.mission().task_id, "T-SIM-1")

        self.run_scenario(scenario)

    def test_a_rover_that_cannot_see_refuses_the_offer_and_says_why(self):
        async def scenario(server, rover, link):
            rover.blind = True
            await rover.run(0.5)
            await server.send_engine(offer_envelope())
            self.assertTrue(await rover.run(5.0, until=lambda: bool(server.received["OFFER_REJECT"])))
            self.assertEqual(server.received["OFFER_REJECT"][-1]["reason"], "PERCEPTION_UNAVAILABLE: NO_FRAME")
            self.assertIsNone(rover.mission())
            self.assertEqual(rover.drives(), [])
            hit = await first_frame(server, rover, lambda f: any(
                x["code"] == "PERCEPTION_NO_FRAME" for x in f.get("faults", ())))
            self.assertIsNotNone(hit, "faults[] never named the perception fault")
            self.assertEqual(hit[1]["readiness"], {"blockers": []})   # outside the readiness vocabulary

        self.run_scenario(scenario)


class TestMissionInterruptions(SimulatedDeliveryCase):
    def test_withdraw_mid_leg_stops_the_rover_and_ends_the_commitment(self):
        async def scenario(server, rover, link):
            await accept_offer(self, server, rover)
            await rover.run(1.0)
            self.assertTrue(rover.drives())
            withdraw = efx.envelope(command="WITHDRAW", commitment_id=COMMITMENT, fence="43", sequence=1,
                                    outbox_id="ob-withdraw")
            await server.send_engine(withdraw)
            self.assertTrue(await rover.run(3.0, until=lambda: rover.mission().status is MissionStatus.ABORTED))
            self.assertIn({"outboxId": "ob-withdraw", "fence": "43", "authorityEpoch": None},
                          server.received["COMMAND_ACK"])
            mark, beats = rover.mark(), len(server.received["HEARTBEAT"])
            await rover.run(1.5)
            self.assertEqual(rover.drives(mark), [], "DRIVE after WITHDRAW")
            self.assertTrue(all(hb == {} for hb in server.received["HEARTBEAT"][beats:]))
            # The commitment is tombstoned: a late OFFER for it is not admitted.
            await server.send_engine(offer_envelope())
            await rover.run(1.0)
            self.assertEqual(len(server.received["OFFER_ACCEPT"]), 1)
            check_telemetry_contract(self, server.received["TELEMETRY"])

        self.run_scenario(scenario)

    def test_operator_pause_resume_then_task_cancellation(self):
        async def scenario(server, rover, link):
            await accept_offer(self, server, rover)
            await rover.run(1.0)
            now = int(time.time() * 1000)
            await server.send_command({"commandId": "pause-1", "type": "PAUSE", "timestamp": now})
            self.assertTrue(await rover.run(2.0, until=lambda: rover.agent.mode is OperatingMode.PAUSED))
            self.assertTrue(await rover.run(2.0, until=lambda: {"commandId": "pause-1"} in server.received["COMMAND_ACK"]))
            mark = rover.mark()
            await rover.run(1.5)
            self.assertEqual(rover.drives(mark), [], "DRIVE while paused")
            hit = await first_frame(server, rover, lambda f: f["status"] == "PAUSED" and any(
                x["code"] == "MISSION_PAUSED" for x in f["faults"]))
            self.assertIsNotNone(hit)

            await server.send_command({"commandId": "resume-1", "type": "RESUME", "timestamp": now + 1,
                                       "commitmentId": COMMITMENT})
            self.assertTrue(await rover.run(3.0, until=lambda: rover.agent.mode is OperatingMode.AUTO))
            mark = rover.mark()
            self.assertTrue(await rover.run(3.0, until=lambda: bool(rover.drives(mark))), "never drove after RESUME")

            acks = len(server.received["COMMAND_ACK"])
            await server.send_stop({"taskId": "T-SIM-1", "reason": "TASK_CANCELLED", "timestamp": now + 2})
            self.assertTrue(await rover.run(3.0, until=lambda: rover.mission().status is MissionStatus.ABORTED))
            mark = rover.mark()
            await rover.run(1.0)
            self.assertEqual(rover.drives(mark), [], "DRIVE after the task was cancelled")
            self.assertEqual(len(server.received["COMMAND_ACK"]), acks, "a bare STOP is never acknowledged")
            self.assertIsNotNone(await first_frame(server, rover, lambda f: f["status"] == "IDLE"
                                                   and f is server.received["TELEMETRY"][-1]))
            check_telemetry_contract(self, server.received["TELEMETRY"])

        self.run_scenario(scenario)

    def test_backend_loss_pauses_the_mission_and_only_a_bound_resume_continues_it(self):
        async def scenario(server, rover, link):
            await accept_offer(self, server, rover)
            await rover.run(1.0)
            # The backend stops answering AUTH and drops the socket: an outage.
            server.ignore_first = server.auth_count + 10_000
            await server.kick()
            self.assertTrue(await rover.run(6.0, until=lambda: rover.agent.mode is OperatingMode.PAUSED))
            self.assertTrue(rover.agent.state.snapshot().last_error.startswith("backend:"))
            mark = rover.mark()
            await rover.run(1.5)
            self.assertEqual(rover.drives(mark), [], "DRIVE with the backend gone")

            # It comes back: the rover re-authenticates, renews its lease, and stays PAUSED.
            server.ignore_first = 0
            self.assertTrue(await rover.run(15.0, until=lambda: link.connected), "never reconnected")
            beats = len(server.received["HEARTBEAT"])
            await rover.run(1.0)
            self.assertIs(rover.agent.mode, OperatingMode.PAUSED)
            self.assertTrue(any(hb.get("commitmentId") == COMMITMENT for hb in server.received["HEARTBEAT"][beats:]))
            hit = await first_frame(server, rover, lambda f: any(
                x["code"] == "MISSION_PAUSED_BACKEND_LINK" for x in f["faults"]))
            self.assertIsNotNone(hit)

            # A RESUME naming another commitment is refused (the Leg may have moved on) ...
            now = int(time.time() * 1000)
            await server.send_command({"commandId": "r-wrong", "type": "RESUME", "timestamp": now,
                                       "commitmentId": "c-someone-else"})
            await rover.run(1.0)
            self.assertIs(rover.agent.mode, OperatingMode.PAUSED)
            self.assertNotIn({"commandId": "r-wrong"}, server.received["COMMAND_ACK"])
            # ... and one naming this commitment continues it.
            await server.send_command({"commandId": "r-right", "type": "RESUME", "timestamp": now + 1,
                                       "commitmentId": COMMITMENT})
            self.assertTrue(await rover.run(3.0, until=lambda: rover.agent.mode is OperatingMode.AUTO))
            await drive_to(self, rover, MissionStatus.AT_PICKUP)

        self.run_scenario(scenario, loss_grace_s=1.0, backoff_initial_s=0.5, backoff_max_s=1.0)

    def test_an_esp32_reboot_mid_leg_latches_reports_at_once_and_waits_for_the_operator(self):
        async def scenario(server, rover, link):
            await accept_offer(self, server, rover)
            await rover.run(1.0)
            self.assertTrue(rover.drives())
            rover.uptime = 400                                       # SIMULATED reboot: uptime went backwards
            self.assertTrue(await rover.run(3.0, until=lambda: rover.agent.emergency_stopped))
            detected = time.monotonic()
            hit = await first_frame(server, rover, lambda f: f.get("safety", {}).get("stopLatch", {}).get("engaged"))
            self.assertIsNotNone(hit, "the latch never reached the backend")
            index, frame = hit
            # Sent on the transition, well inside the 3 s telemetry interval.
            self.assertLess(server.received_at["TELEMETRY"][index] - detected, 1.5)
            self.assertEqual(frame["status"], "PAUSED")                # stopped with the Leg on board, not IDLE
            self.assertEqual(frame["readiness"]["blockers"][0], "ESTOP_LATCHED")
            codes = {x["code"] for x in frame["faults"]}
            self.assertLessEqual({"PI_EMERGENCY_STOP_LATCHED", "ESP32_REBOOT_UNACKNOWLEDGED"}, codes)
            self.assertTrue(rover.mission().interrupted)
            self.assertGreater(link.stats["telemetry_on_transition"], 0)
            mark, beats = rover.mark(), len(server.received["HEARTBEAT"])
            await rover.run(2.0)
            self.assertEqual(rover.drives(mark), [], "DRIVE after an ESP32 reboot")
            self.assertTrue(all(hb == {} for hb in server.received["HEARTBEAT"][beats:]), "lease renewed while latched")

            # The operator clears the latch (acknowledging the reboot): PAUSED, still no motion.
            rover.agent.clear_emergency_stop("operator (simulated)")
            mark = rover.mark()
            await rover.run(2.0)
            self.assertIs(rover.agent.mode, OperatingMode.PAUSED)
            self.assertEqual(rover.drives(mark), [])
            await server.send_command({"commandId": "resume-r", "type": "RESUME",
                                       "timestamp": int(time.time() * 1000), "commitmentId": COMMITMENT})
            self.assertTrue(await rover.run(3.0, until=lambda: rover.agent.mode is OperatingMode.AUTO))
            await drive_to(self, rover, MissionStatus.AT_PICKUP)
            check_telemetry_contract(self, server.received["TELEMETRY"])

        self.run_scenario(scenario, telemetry_interval_s=3.0)


if __name__ == "__main__":
    unittest.main()
