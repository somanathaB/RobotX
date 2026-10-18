"""TELEMETRY `readiness.blockers` (ROBOTX_PI_READINESS_HANDOFF P-1, P-3) and the
older-backend compatibility of the frame that carries it.

Host-only: no serial port, no network, no motion. Where a test needs a motor
link or a fix that this host does not have, it says SIMULATED and injects the
state directly, exactly as the existing offer tests do.

The contract being pinned:
  - the blockers are the Rover-level reasons `assess_offer` would refuse any
    offer, every one that applies, in its order, in the backend's exact
    vocabulary (offerRefusal.service.js AGENT_READINESS_REFUSALS);
  - `[]` only when every check positively passed;
  - anything that cannot be evaluated omits the block -- never `[]`.
"""

import asyncio
import json
import logging
import re
import time
import unittest
from dataclasses import replace
from types import SimpleNamespace

from robotx.communication.backend_link import BackendConfig, BackendLink
from robotx.communication.engine import OfferVerdict
from robotx.communication.protocol import (
    DEMO_BATTERY_SOURCE,
    READINESS_BLOCKERS,
    build_telemetry_payload,
    readiness_block,
)
from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus
from robotx.localization.position import Position, PositionSource
from robotx.mission.mission import ActiveMission, MissionSegment, MissionStatus
from robotx.state.robot_state import Esp32LinkStatus, OperatingMode, RobotState
from tests.unit.test_backend_link import FakeAgent, FakeSio, MemoryTokenStore, connect_and_auth, state_with_fix
from tests.unit.test_demo_battery import simulated_power
from tests.unit.test_engine_offer import parsed_offer, real_agent


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


# The backend's vocabulary, restated from Backend/src/services/offerRefusal.service.js
# (AGENT_READINESS_REFUSALS, 2026-10-18). Restated rather than imported on purpose:
# if either side renames a code, this test is what notices.
BACKEND_AGENT_READINESS_REFUSALS = (
    "ESTOP_LATCHED", "AGENT_ERROR", "NO_MOTOR_LINK", "NO_POSITION_FIX", "NO_CUSTODY_SENSING",
)

# Backend attestation.findCapabilityClaims: a key matching this drops the whole frame.
BACKEND_CLAIM_KEYS = re.compile(
    r"^(capabilit(y|ies)|certifications?|attestedcapabilit(y|ies)|hazmatcertified|cold_?chain|clearance)$"
)


class _Esp32(object):
    """SIMULATED controller link: only `status().motion_ready` is consulted."""

    def __init__(self, motion_ready):
        self.motion_ready = motion_ready
        self.status_calls = 0

    def status(self):
        self.status_calls += 1
        return SimpleNamespace(motion_ready=self.motion_ready)


def give_fix(agent, *, h_acc_m=1.5, source=PositionSource.GPS, status=GPSStatus.FIX):
    """SIMULATED fix, injected into state the way the offer tests do."""

    now = time.time()
    agent.state.update_gps(
        GpsReading(status=status, fix=GpsFix(latitude=12.9, longitude=77.5, timestamp=now), age_s=0.1),
        Position(latitude=12.9, longitude=77.5, timestamp=now, source=source,
                 fix_type="3D", h_acc_m=h_acc_m),
    )


def ready_agent():
    """Every Rover-level check satisfied -- all of it SIMULATED on this host."""

    agent = real_agent(ROBOTX_CUSTODY_CONFIRMATION="operator")
    agent.state.update_communication(esp32=Esp32LinkStatus.UP)
    agent.esp32 = _Esp32(motion_ready=True)
    give_fix(agent)
    return agent


def hold_mission(agent):
    agent.state.update_mission(ActiveMission(
        mission=_offer_mission(), status=MissionStatus.TO_PICKUP,
        segment=MissionSegment.TO_PICKUP, accepted_at=time.time(), custody_required=True,
    ))


def _offer_mission():
    from robotx.communication.engine import offer_to_mission

    return offer_to_mission(parsed_offer())


# --- the agent's own answer -----------------------------------------------------


class TestAgentReadinessBlockers(unittest.TestCase):
    def test_the_rover_as_it_stands_reports_every_blocker_in_order(self):
        """No ESP32, no GPS, no custody path: all three, not only the first."""

        self.assertEqual(
            real_agent().readiness_blockers(),
            ["NO_MOTOR_LINK", "NO_POSITION_FIX", "NO_CUSTODY_SENSING"],
        )

    def test_everything_up_is_an_empty_list(self):
        self.assertEqual(ready_agent().readiness_blockers(), [])

    def test_esp32_link_down_is_no_motor_link(self):
        for link in (Esp32LinkStatus.DISABLED, Esp32LinkStatus.DISCONNECTED, Esp32LinkStatus.STALE):
            agent = ready_agent()
            agent.state.update_communication(esp32=link)
            self.assertEqual(agent.readiness_blockers(), ["NO_MOTOR_LINK"], link)

    def test_an_up_link_that_is_not_motion_ready_is_no_motor_link(self):
        """Includes ROBOTX_ESP32_MOTION_ENABLED=false: an UP link is not a motor link."""

        agent = ready_agent()
        agent.esp32 = _Esp32(motion_ready=False)
        self.assertEqual(agent.readiness_blockers(), ["NO_MOTOR_LINK"])

    def test_motion_disabled_on_a_real_link_is_no_motor_link(self):
        """The real Esp32Link over a fake port, motion disabled as deployed."""

        from tests.unit.test_esp32_agent import agent_with_link

        agent, link, port, clock = agent_with_link(motion=False, drive_available=True,
                                                   ROBOTX_ROBOT_ID="robotx-pi")
        agent.tick()
        self.assertIs(agent.state.snapshot().communication.esp32, Esp32LinkStatus.UP)
        self.assertFalse(link.cfg.motion_enabled)
        self.assertIn("NO_MOTOR_LINK", agent.readiness_blockers())
        self.assertEqual(port.command_names().count("DRIVE"), 0)

    def test_no_usable_fix_is_no_position_fix(self):
        cases = {
            "no reading at all": None,
            "receiver stale": dict(status=GPSStatus.STALE),
            "dead-reckoned, not measured": dict(source=PositionSource.DEAD_RECKONING),
            "worse than the navigation limit": dict(h_acc_m=15.0),
        }
        for name, fix in cases.items():
            agent = real_agent(ROBOTX_CUSTODY_CONFIRMATION="operator")
            agent.state.update_communication(esp32=Esp32LinkStatus.UP)
            agent.esp32 = _Esp32(motion_ready=True)
            if fix is not None:
                give_fix(agent, **fix)
            self.assertEqual(agent.readiness_blockers(), ["NO_POSITION_FIX"], name)

    def test_no_custody_path_is_no_custody_sensing(self):
        agent = ready_agent()
        agent.settings = replace(agent.settings, custody_confirmation="none")
        self.assertEqual(agent.readiness_blockers(), ["NO_CUSTODY_SENSING"])

    def test_estop_and_error_lead_the_list(self):
        agent = ready_agent()
        agent.emergency_stop("test")
        self.assertEqual(agent.readiness_blockers(), ["ESTOP_LATCHED"])
        agent.state.set_mode(OperatingMode.ERROR, error="test")
        self.assertEqual(agent.readiness_blockers(), ["ESTOP_LATCHED", "AGENT_ERROR"])

    def test_offer_specific_reasons_are_never_reported(self):
        agent = ready_agent()
        hold_mission(agent)
        self.assertEqual(agent.readiness_blockers(), [])
        self.assertEqual(agent.assess_offer(parsed_offer()).reason, "MISSION_IN_PROGRESS")

    def test_every_code_is_in_the_backend_vocabulary(self):
        self.assertEqual(READINESS_BLOCKERS, BACKEND_AGENT_READINESS_REFUSALS)
        agent = real_agent()
        agent.emergency_stop("test")
        agent.state.set_mode(OperatingMode.ERROR, error="test")
        everything = agent.readiness_blockers()
        self.assertEqual(everything, list(BACKEND_AGENT_READINESS_REFUSALS))


class TestSameChecksAsAssessOffer(unittest.TestCase):
    """The block tells the backend exactly what an offer would be refused for."""

    def scenarios(self):
        def estop(a):
            a.emergency_stop("test")

        def error(a):
            a.state.set_mode(OperatingMode.ERROR, error="test")

        def link_down(a):
            a.state.update_communication(esp32=Esp32LinkStatus.DISCONNECTED)

        def not_motion_ready(a):
            a.esp32 = _Esp32(motion_ready=False)

        def poor_fix(a):
            give_fix(a, h_acc_m=15.0)

        def no_custody(a):
            a.settings = replace(a.settings, custody_confirmation="none")

        return {
            "ready": [],
            "estop": [estop],
            "error": [error],
            "link down": [link_down],
            "not motion ready": [not_motion_ready],
            "poor fix": [poor_fix],
            "no custody": [no_custody],
            "link down + poor fix + no custody": [link_down, poor_fix, no_custody],
            "estop + not motion ready": [estop, not_motion_ready],
        }

    def test_first_blocker_is_the_offer_refusal_and_empty_means_accept(self):
        for name, steps in self.scenarios().items():
            agent = ready_agent()
            for step in steps:
                step(agent)
            blockers = agent.readiness_blockers()
            decision = agent.assess_offer(parsed_offer())
            if blockers:
                self.assertIs(decision.verdict, OfferVerdict.REJECT, name)
                self.assertEqual(decision.reason, blockers[0], name)
            else:
                self.assertIs(decision.verdict, OfferVerdict.ACCEPT, name)

    def test_a_held_mission_keeps_its_old_place_in_the_order(self):
        # After e-stop/ERROR ...
        agent = ready_agent()
        hold_mission(agent)
        agent.emergency_stop("test")
        self.assertEqual(agent.assess_offer(parsed_offer()).reason, "ESTOP_LATCHED")
        # ... before the motor link.
        agent = ready_agent()
        hold_mission(agent)
        agent.state.update_communication(esp32=Esp32LinkStatus.DISCONNECTED)
        self.assertEqual(agent.assess_offer(parsed_offer()).reason, "MISSION_IN_PROGRESS")


# --- the wire -------------------------------------------------------------------


def frame(snapshot=None, **kwargs):
    snapshot = snapshot if snapshot is not None else state_with_fix().snapshot()
    return build_telemetry_payload(snapshot, sequence=3, max_position_age_s=5.0, **kwargs).payload


class TestReadinessOnTheWire(unittest.TestCase):
    def test_not_evaluated_is_omitted(self):
        self.assertNotIn("readiness", frame())
        self.assertNotIn("readiness", frame(readiness_blockers=None))

    def test_evaluated_and_clear_is_an_empty_list(self):
        self.assertEqual(frame(readiness_blockers=[])["readiness"], {"blockers": []})

    def test_blockers_are_sent_exactly(self):
        payload = frame(readiness_blockers=["NO_MOTOR_LINK", "NO_POSITION_FIX"])
        self.assertEqual(payload["readiness"], {"blockers": ["NO_MOTOR_LINK", "NO_POSITION_FIX"]})
        self.assertIsInstance(payload["readiness"]["blockers"], list)
        json.loads(json.dumps(payload))

    def test_duplicates_are_collapsed(self):
        self.assertEqual(readiness_block(["NO_MOTOR_LINK", "NO_MOTOR_LINK"]), {"blockers": ["NO_MOTOR_LINK"]})

    def test_a_code_outside_the_vocabulary_is_refused_not_sent(self):
        for code in ("MISSION_IN_PROGRESS", "NO_EXECUTABLE_PATH", "no_motor_link", "READY"):
            with self.assertRaises(ValueError, msg=code):
                readiness_block([code])

    def test_stamped_by_the_frame_timestamp_on_both_frame_kinds(self):
        with_fix = state_with_fix().snapshot()
        payload = frame(with_fix, readiness_blockers=[])
        self.assertEqual(payload["timestamp"], int(round(with_fix.position.timestamp * 1000)))
        no_fix = RobotState("robotx-pi").snapshot()
        payload = build_telemetry_payload(no_fix, sequence=4, max_position_age_s=5.0, now=1_758_700_000.0,
                                          readiness_blockers=["NO_POSITION_FIX"]).payload
        self.assertEqual(payload["timestamp"], 1_758_700_000_000)
        self.assertNotIn("lat", payload)

    def test_no_key_is_one_the_backend_drops_a_frame_for(self):
        snap = replace(state_with_fix().snapshot(),
                       power=simulated_power(50.0))
        payload = frame(snap, readiness_blockers=["NO_MOTOR_LINK"])

        def keys(node):
            if isinstance(node, dict):
                for k, v in node.items():
                    yield k
                    yield from keys(v)
            elif isinstance(node, list):
                for v in node:
                    yield from keys(v)

        flagged = [k for k in keys(payload) if BACKEND_CLAIM_KEYS.match(re.sub(r"[^a-z_]", "", k.lower()))]
        self.assertEqual(flagged, [])


class TestOlderBackendCompatibility(unittest.TestCase):
    """The new keys are additive: nothing an older backend reads changes.

    An older backend (Render 69d5959/de03788) ignores `readiness` and reads only
    `energy.socMethod`, which is never sent -- so to it these frames are the
    frames it already accepted.
    """

    def test_existing_fields_are_unchanged_by_the_readiness_block(self):
        snap = state_with_fix().snapshot()
        before = build_telemetry_payload(snap, sequence=9, max_position_age_s=5.0).payload
        after = build_telemetry_payload(snap, sequence=9, max_position_age_s=5.0,
                                        readiness_blockers=["NO_MOTOR_LINK"]).payload
        self.assertEqual(set(after) - set(before), {"readiness"})
        self.assertEqual({k: after[k] for k in before}, before)

    def test_the_demo_label_never_carries_a_soc_basis(self):
        snap = replace(state_with_fix().snapshot(),
                       power=simulated_power(50.0))
        payload = frame(snap)
        self.assertEqual(payload["energy"], {"batterySource": DEMO_BATTERY_SOURCE})
        self.assertNotIn("socMethod", payload["energy"])


# --- the link: when readiness can and cannot be vouched for -----------------------


def link_with(readiness=None, agent_alive=None, state=None):
    sio = FakeSio()
    link = BackendLink(
        BackendConfig(enabled=True, robot_id="robotx-pi", pairing_code="123456"),
        state or state_with_fix(),
        FakeAgent(mode=OperatingMode.IDLE),
        client_factory=lambda _cfg: sio,
        token_store=MemoryTokenStore(),
        agent_alive=agent_alive,
        readiness=readiness,
    )
    return link, sio


def sent_telemetry(link, sio):
    async def scenario():
        await connect_and_auth(link)
        before = len(sio.emitted)
        await link._publish_telemetry()
        return [p for e, p in sio.emitted[before:] if e == "TELEMETRY"]

    frames = asyncio.run(scenario())
    assert len(frames) == 1, frames
    return frames[0]


class TestLinkReadiness(unittest.TestCase):
    def test_the_agents_answer_rides_the_frame(self):
        seen = []

        def readiness(snapshot):
            seen.append(snapshot)
            return ["NO_MOTOR_LINK"]

        link, sio = link_with(readiness=readiness, agent_alive=lambda: True)
        payload = sent_telemetry(link, sio)
        self.assertEqual(payload["readiness"], {"blockers": ["NO_MOTOR_LINK"]})
        # Evaluated on the very snapshot the frame was built from.
        self.assertEqual(len(seen), 1)
        self.assertEqual(seen[0].robot_id, "robotx-pi")

    def test_no_source_omits_the_block(self):
        link, sio = link_with(readiness=None)
        self.assertNotIn("readiness", sent_telemetry(link, sio))

    def test_a_stalled_agent_loop_omits_the_block_but_still_sends_the_frame(self):
        link, sio = link_with(readiness=lambda _s: [], agent_alive=lambda: False)
        payload = sent_telemetry(link, sio)
        self.assertNotIn("readiness", payload)
        self.assertIn("status", payload)

    def test_a_check_that_fails_to_evaluate_omits_the_block_never_empty(self):
        def broken(_snapshot):
            raise RuntimeError("ESP32 status unreadable")

        link, sio = link_with(readiness=broken, agent_alive=lambda: True)
        payload = sent_telemetry(link, sio)
        self.assertNotIn("readiness", payload)
        self.assertIn("lat", payload)

    def test_the_real_agent_end_to_end(self):
        """Acceptance (1), through the real agent and link: link down -> NO_MOTOR_LINK on the frame."""

        agent = real_agent()
        agent._last_tick_at = time.monotonic()        # SIMULATED: a live agent loop
        link, sio = link_with(readiness=agent.readiness_blockers, agent_alive=agent.is_alive, state=agent.state)
        payload = sent_telemetry(link, sio)
        self.assertEqual(payload["readiness"]["blockers"],
                         ["NO_MOTOR_LINK", "NO_POSITION_FIX", "NO_CUSTODY_SENSING"])

        ready = ready_agent()
        ready._last_tick_at = time.monotonic()        # SIMULATED
        link, sio = link_with(readiness=ready.readiness_blockers, agent_alive=ready.is_alive, state=ready.state)
        self.assertEqual(sent_telemetry(link, sio)["readiness"], {"blockers": []})

    def test_before_the_first_agent_tick_nothing_is_vouched_for(self):
        agent = real_agent()
        self.assertFalse(agent.is_alive())
        link, sio = link_with(readiness=agent.readiness_blockers, agent_alive=agent.is_alive, state=agent.state)
        self.assertNotIn("readiness", sent_telemetry(link, sio))


if __name__ == "__main__":
    unittest.main()
