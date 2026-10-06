"""P2B-1 (part 2): OFFER, COMMAND_ACK, OFFER_*, CUSTODY_EVENT, TASK_COMPLETE.

Against `ROBOTX_PI_P2B1_HANDOFF.md`. Required tests 1-16 of the brief are the
classes numbered `T01`..`T16`; the rest pin the admission rules they rely on.

Test doubles, labelled
----------------------
- `FakeSio` (from `test_backend_link`): a SIMULATED backend socket.
- `tests.fixtures.engine`: SIMULATED signed envelopes, signed with a test-only
  key that is not the backend's `COMMAND_SIGNING_KEY`.
- `OfferAgent`: a SIMULATED RobotAgent whose feasibility verdict is scripted.
  Where a test needs the *real* agent's verdict it builds a real `RobotAgent`
  with every hardware subsystem disabled -- and that agent rejects, because
  this Rover has no motor link, no GPS fix and no custody sensing.
- GPS fixes are SYNTHETIC. No test here is evidence about real hardware or a
  real backend.
"""

from __future__ import annotations

import hashlib
import hmac
import os
import tempfile
import time
import unittest
from dataclasses import replace

from robotx.communication.backend_link import BackendConfig, BackendLink
from robotx.communication.engine import (
    FIELD_SEPARATOR,
    EnvelopeRejection,
    EnvelopeRejectReason,
    OfferDecision,
    OfferVerdict,
    canonical_string,
    js_stringify,
    offer_to_mission,
    parse_envelope,
    parse_offer,
    validate_defer_until,
    verify_signature,
)
from robotx.communication.protocol import now_ms
from robotx.config.settings import Settings
from robotx.hardware.gps import GpsFix, GpsReading, GPSStatus
from robotx.localization.position import Position
from robotx.mission.evidence import TrackFix, assess_completion
from robotx.mission.mission import ActiveMission, MissionSegment, MissionStatus
from robotx.state.robot_state import Esp32LinkStatus, MissionRefused, OperatingMode, RobotState
from tests.fixtures import engine as fx
from tests.fixtures.task_assign import DROP, PICKUP, path_to_drop, task_assign_payload
from tests.unit.test_backend_link import (
    AsyncTestCase,
    FakeAgent,
    FakeSio,
    MemoryTokenStore,
    connect_and_auth,
    make_link,
    state_with_fix,
)


KEY = fx.TEST_SIGNING_KEY


class OfferAgent(FakeAgent):
    """SIMULATED agent: scripted feasibility; records the mission it is given."""

    def __init__(self, decision=None, state=None, **kwargs):
        super().__init__(**kwargs)
        self.decision = decision
        self.state = state

    def assess_offer(self, offer):
        self.calls.append(("assess", offer.commitment_id))
        if self.decision == "accept":
            return OfferDecision.accept(offer_to_mission(offer))
        return self.decision or OfferDecision.reject("SIMULATED_REJECT")

    def assign_mission(self, mission, custody_required=False):
        self.calls.append(("assign", mission.task_id, custody_required))
        if self.state is not None:
            self.state.update_mission(ActiveMission(
                mission=mission, status=MissionStatus.TO_PICKUP,
                segment=MissionSegment.TO_PICKUP, accepted_at=time.time(),
                custody_required=custody_required,
            ))


def engine_link(agent=None, state=None, key=KEY, **cfg):
    cfg.setdefault("command_signing_key", key)
    return make_link(state=state, agent=agent, **cfg)


def fire(link, sio, *envelopes, connect=True):
    """Deliver envelopes; return only what the Pi emitted in response."""

    async def scenario():
        if connect:
            await connect_and_auth(link)
        before = len(sio.emitted)
        for env in envelopes:
            # The backend emits each envelope under its own command name.
            await sio.fire(env.get("command") if isinstance(env, dict) and isinstance(env.get("command"), str) else "OFFER", env)
        return sio.emitted[before:]

    import asyncio

    return asyncio.run(scenario())


def names(emitted):
    return [event for event, _ in emitted]


def real_agent(**env):
    from robotx.application.agent import RobotAgent

    base = {"ROBOTX_CAMERA_ENABLED": "0", "ROBOTX_PERCEPTION_ENABLED": "0",
            "ROBOTX_GPS_ENABLED": "0", "ROBOTX_LOG_LEVEL": "CRITICAL",
            "ROBOTX_ROBOT_ID": fx.ROBOT_ID}
    base.update(env)
    return RobotAgent(Settings.from_env(base))


def parsed_offer(**kwargs):
    env = parse_envelope(fx.envelope(**kwargs), expected_agent_id=fx.ROBOT_ID)
    return parse_offer(env)


# --- the signature ------------------------------------------------------------


class TestCanonicalSigning(unittest.TestCase):
    RAW = {
        "agentId": "robotx-pi", "command": "OFFER", "commandClass": "MISSION",
        "fenceScope": "COMMITMENT", "commitmentId": "c-1", "fence": "42",
        "authorityEpoch": None, "fenceFloor": "41", "sequence": 0,
        "notValidAfter": "2026-09-24T10:00:00.000Z",
        "payload": {"b": 1.0, "a": [{"y": 2, "x": 0.5}], "s": "é"},
    }
    # Written by hand from handoff §6 step 3, not produced by the code under test.
    EXPECTED = "\u001f".join([
        'agentId="robotx-pi"', 'command="OFFER"', 'commandClass="MISSION"',
        'fenceScope="COMMITMENT"', 'commitmentId="c-1"', "fence=42",
        "authorityEpoch=null", "fenceFloor=41", "sequence=0",
        "notValidAfter=2026-09-24T10:00:00.000Z",
        'payload={"a":[{"x":0.5,"y":2}],"b":1,"s":"é"}',
    ])

    def test_canonical_string_matches_the_hand_written_vector(self):
        self.assertEqual(canonical_string(self.RAW), self.EXPECTED)

    def test_signature_is_hmac_sha256_hex_over_the_utf8_canonical_string(self):
        raw = dict(self.RAW)
        raw["signature"] = hmac.new(KEY.encode(), self.EXPECTED.encode("utf-8"),
                                    hashlib.sha256).hexdigest()
        self.assertIsNone(verify_signature(raw, KEY.encode()))

    def test_javascript_number_rendering(self):
        for value, js in ((1.0, "1"), (100.0, "100"), (0.1, "0.1"), (1e21, "1e+21"),
                          (1e-7, "1e-7"), (0.000001, "0.000001"), (-2.5, "-2.5"), (12.93541, "12.93541")):
            self.assertEqual(js_stringify(value), js, value)

    def test_separator_is_unit_separator(self):
        self.assertEqual(FIELD_SEPARATOR, "\u001f")

    def test_tampering_breaks_the_signature(self):
        env = fx.envelope()
        env["payload"]["taskId"] = "T-999"
        rejection = verify_signature(env, KEY.encode())
        self.assertIs(rejection.reason, EnvelopeRejectReason.BAD_SIGNATURE)

    def test_no_key_means_unverifiable_never_skipped(self):
        for key in (None, b"", b"too-short"):
            rejection = verify_signature(fx.envelope(), key)
            self.assertIs(rejection.reason, EnvelopeRejectReason.SIGNATURE_UNVERIFIABLE, key)


class TestSignatureAtTheLink(AsyncTestCase):
    def test_without_a_configured_key_no_offer_is_admitted(self):
        agent = OfferAgent("accept")
        link, sio = engine_link(agent=agent, key=None)
        self.assertEqual(fire(link, sio, fx.envelope()), [])
        self.assertEqual(agent.calls, [])
        self.assertEqual(link.describe()["command_signing_key"], "UNSET")

    def test_a_bad_signature_is_not_admitted(self):
        env = fx.envelope()
        env["signature"] = "0" * 64
        agent = OfferAgent("accept")
        link, sio = engine_link(agent=agent)
        self.assertEqual(fire(link, sio, env), [])
        self.assertEqual(agent.calls, [])

    def test_signing_key_comes_from_configuration_and_is_never_exposed(self):
        settings = Settings.from_env({"ROBOTX_SOCKET_ENABLED": "1", "ROBOTX_ROBOT_ID": "r",
                                      "ROBOTX_SOCKET_SERVER_URL": "http://192.0.2.1:1",
                                      "ROBOTX_COMMAND_SIGNING_KEY": KEY})
        cfg = BackendConfig.from_settings(settings)
        self.assertEqual(cfg.command_signing_key, KEY)
        link, _ = engine_link()
        self.assertNotIn(KEY, repr(link.describe()))


# --- 1 ------------------------------------------------------------------------


class T01ValidOfferIsAcked(AsyncTestCase):
    def test_ack_echoes_outbox_fence_and_epoch_before_the_response(self):
        link, sio = engine_link(agent=OfferAgent())
        out = fire(link, sio, fx.envelope(authority_epoch=7))
        self.assertEqual(out[0], ("COMMAND_ACK", {"outboxId": "ob-19", "fence": "42", "authorityEpoch": 7}))
        self.assertEqual(names(out), ["COMMAND_ACK", "OFFER_REJECT"])

    def test_null_epoch_is_echoed_as_null(self):
        link, sio = engine_link(agent=OfferAgent())
        ack = fire(link, sio, fx.envelope())[0][1]
        self.assertEqual(ack, {"outboxId": "ob-19", "fence": "42", "authorityEpoch": None})


# --- 2, 3, 4, 5 ---------------------------------------------------------------


class T02Accept(AsyncTestCase):
    def test_accept_is_sent_once_with_commitment_and_the_exact_fence(self):
        state = state_with_fix(fx.ROBOT_ID)
        agent = OfferAgent("accept", state=state)
        link, sio = engine_link(agent=agent, state=state)
        out = fire(link, sio, fx.envelope())
        self.assertEqual(out[1], ("OFFER_ACCEPT", {"commitmentId": "c-7f3a", "fence": "42"}))
        # The mission is started as a custody mission, from the offer's own stops.
        self.assertIn(("assign", "T-123", True), agent.calls)
        self.assertEqual(link.stats["offers_accepted"], 1)

    def test_accept_is_never_automatic(self):
        # The scripted default declines: arriving is not a reason to accept.
        link, sio = engine_link(agent=OfferAgent())
        self.assertNotIn("OFFER_ACCEPT", names(fire(link, sio, fx.envelope())))

    def test_fence_is_echoed_exactly_as_it_arrived(self):
        for fence in ("42", 42, "0042"):
            state = state_with_fix(fx.ROBOT_ID)
            link, sio = engine_link(agent=OfferAgent("accept", state=state), state=state)
            out = fire(link, sio, fx.envelope(fence=fence))
            self.assertEqual(out[1][1]["fence"], fence)

    def test_an_agent_that_refuses_the_mission_turns_accept_into_reject(self):
        from robotx.mission.mission import MissionRejected, MissionRejectReason

        class Refusing(OfferAgent):
            def assign_mission(self, mission, custody_required=False):
                raise MissionRejected(MissionRejectReason.ESTOP_ENGAGED, "latched")

        link, sio = engine_link(agent=Refusing("accept"))
        out = fire(link, sio, fx.envelope())
        self.assertEqual(out[1][0], "OFFER_REJECT")
        self.assertIn("ESTOP_ENGAGED", out[1][1]["reason"])


class T03Reject(AsyncTestCase):
    def test_reject_carries_an_accurate_reason(self):
        link, sio = engine_link(agent=OfferAgent(OfferDecision.reject("NO_MOTOR_LINK")))
        out = fire(link, sio, fx.envelope())
        self.assertEqual(out[1], ("OFFER_REJECT",
                                  {"commitmentId": "c-7f3a", "fence": "42", "reason": "NO_MOTOR_LINK"}))

    def test_the_real_agent_rejects_today_and_says_why(self):
        """The actual Rover, as it stands: no motor link, so it declines."""

        agent = real_agent()
        self.assertEqual(agent.assess_offer(parsed_offer()).reason, "NO_MOTOR_LINK")

    def test_the_real_agent_names_each_missing_capability_in_turn(self):
        agent = real_agent()
        # SIMULATED: pretend the ESP32 link is up, to reach the next check.
        agent.state.update_communication(esp32=Esp32LinkStatus.UP)
        self.assertEqual(agent.assess_offer(parsed_offer()).reason, "NO_POSITION_FIX")
        # SIMULATED fix, to reach the last check.
        fixed = state_with_fix(fx.ROBOT_ID).snapshot()
        agent.state.update_gps(fixed.gps, fixed.position)
        self.assertEqual(agent.assess_offer(parsed_offer()).reason, "NO_CUSTODY_SENSING")

    def test_a_stop_without_a_path_is_no_executable_path(self):
        payload = fx.offer_payload(stop_sequence=fx.stops(with_paths=False))
        offer = parsed_offer(payload=payload)
        self.assertEqual(real_agent().assess_offer(offer).reason, "NO_EXECUTABLE_PATH")

    def test_a_shape_the_rover_cannot_execute_is_rejected_not_guessed(self):
        payload = fx.offer_payload(stop_sequence=fx.stops()[:1])
        decision = real_agent().assess_offer(parsed_offer(payload=payload))
        self.assertIs(decision.verdict, OfferVerdict.REJECT)
        self.assertIn("UNSUPPORTED_MISSION", decision.reason)


class T04Defer(AsyncTestCase):
    def test_defer_with_future_epoch_ms(self):
        until = now_ms() + 300_000
        link, sio = engine_link(agent=OfferAgent(OfferDecision.defer(until, "SIMULATED_TEMPORARY")))
        out = fire(link, sio, fx.envelope())
        self.assertEqual(out[1], ("OFFER_DEFER", {"commitmentId": "c-7f3a", "fence": "42",
                                                  "until": until, "reason": "SIMULATED_TEMPORARY"}))

    def test_defer_with_future_iso_string(self):
        until = fx.iso(time.time() + 300)
        link, sio = engine_link(agent=OfferAgent(OfferDecision.defer(until, "SIMULATED_TEMPORARY")))
        self.assertEqual(fire(link, sio, fx.envelope())[1][1]["until"], until)


class T05DeferNumericStringUntil(AsyncTestCase):
    def test_numeric_string_until_is_refused(self):
        with self.assertRaises(ValueError):
            validate_defer_until(str(now_ms() + 300_000))

    def test_past_or_missing_until_is_refused(self):
        for until in (now_ms() - 1, fx.iso(time.time() - 5), None, True, "not a time"):
            with self.assertRaises(ValueError, msg=repr(until)):
                validate_defer_until(until)

    def test_an_invalid_defer_is_never_sent_and_becomes_a_reject(self):
        bad = OfferDecision.defer(str(now_ms() + 300_000), "SIMULATED_TEMPORARY")
        link, sio = engine_link(agent=OfferAgent(bad))
        out = fire(link, sio, fx.envelope())
        self.assertEqual(names(out), ["COMMAND_ACK", "OFFER_REJECT"])
        self.assertEqual(link.stats["offers_deferred"], 0)


# --- 6, 7, 8 ------------------------------------------------------------------


class T06FenceMismatch(AsyncTestCase):
    def test_payload_fence_differing_from_envelope_is_not_admitted(self):
        env = fx.envelope(payload=fx.offer_payload(fence="41"))
        agent = OfferAgent("accept")
        link, sio = engine_link(agent=agent)
        self.assertEqual(fire(link, sio, env), [])
        self.assertEqual(agent.calls, [])

    def test_fence_at_or_below_the_floor_is_not_admitted(self):
        link, sio = engine_link(agent=OfferAgent())
        self.assertEqual(fire(link, sio, fx.envelope(fence="42", fence_floor="42")), [])

    def test_a_stale_fence_for_a_known_commitment_is_not_admitted(self):
        agent = OfferAgent()
        link, sio = engine_link(agent=agent)
        first = fx.envelope()
        # A different outbox row, same commitment, fence not advanced.
        again = fx.envelope(outbox_id="ob-20", sequence=1)
        out = fire(link, sio, first, again)
        self.assertEqual(names(out), ["COMMAND_ACK", "OFFER_REJECT"])


class T07CommitmentMismatch(AsyncTestCase):
    def test_payload_commitment_differing_from_envelope_is_not_admitted(self):
        env = fx.envelope(payload=fx.offer_payload(commitment_id="c-other"))
        agent = OfferAgent("accept")
        link, sio = engine_link(agent=agent)
        self.assertEqual(fire(link, sio, env), [])
        self.assertEqual(agent.calls, [])


class T08OfferForAnotherRobot(AsyncTestCase):
    def test_nothing_is_sent_and_nothing_is_assessed(self):
        agent = OfferAgent("accept")
        link, sio = engine_link(agent=agent)
        self.assertEqual(fire(link, sio, fx.envelope(agent_id="robotx-sim-7")), [])
        self.assertEqual(agent.calls, [])
        self.assertEqual(link.stats["engine_commands_not_admitted"], 1)


# --- admission rules the above rely on -----------------------------------------


class TestAdmission(AsyncTestCase):
    def test_expired_envelope_is_not_admitted(self):
        link, sio = engine_link(agent=OfferAgent("accept"))
        self.assertEqual(fire(link, sio, fx.envelope(valid_for_s=-1)), [])

    def test_unknown_engine_command_is_not_admitted(self):
        env = fx.envelope(command="TELEPORT")
        self.assertIsInstance(parse_envelope(env, expected_agent_id=fx.ROBOT_ID), EnvelopeRejection)

    def test_expired_offer_is_acked_but_not_answered(self):
        env = fx.envelope(payload=fx.offer_payload(expiry_in_s=-1))
        link, sio = engine_link(agent=OfferAgent("accept"))
        self.assertEqual(names(fire(link, sio, env)), ["COMMAND_ACK"])

    def test_a_sequence_gap_is_held_then_applied_in_order(self):
        state = state_with_fix(fx.ROBOT_ID)
        agent = OfferAgent("accept", state=state)
        link, sio = engine_link(agent=agent, state=state)
        withdraw = fx.envelope(command="WITHDRAW", fence="43", sequence=1, outbox_id="ob-21")
        offer = fx.envelope()
        out = fire(link, sio, withdraw, offer)
        # The WITHDRAW waited for the OFFER, then applied after it.
        self.assertEqual(names(out), ["COMMAND_ACK", "OFFER_ACCEPT", "COMMAND_ACK"])
        self.assertEqual(out[2][1]["outboxId"], "ob-21")
        self.assertTrue(link.commitments.get("c-7f3a").tombstoned)

    def test_withdraw_stops_that_mission_tombstones_and_acks(self):
        state = state_with_fix(fx.ROBOT_ID)
        agent = OfferAgent("accept", state=state)
        link, sio = engine_link(agent=agent, state=state)
        out = fire(link, sio, fx.envelope(),
                   fx.envelope(command="WITHDRAW", fence="43", sequence=1, outbox_id="ob-21"),
                   fx.envelope(fence="44", sequence=2, outbox_id="ob-22"))
        self.assertIn(("stop", "engine WITHDRAW (c-7f3a)"), agent.calls)
        # The later OFFER on a tombstoned commitment is not admitted.
        self.assertEqual(names(out), ["COMMAND_ACK", "OFFER_ACCEPT", "COMMAND_ACK"])

    def test_commands_with_no_producer_are_acked_with_no_effect(self):
        state = state_with_fix(fx.ROBOT_ID)
        agent = OfferAgent("accept", state=state)
        link, sio = engine_link(agent=agent, state=state)
        out = fire(link, sio, fx.envelope(),
                   fx.envelope(command="REROUTE", fence="43", sequence=1, outbox_id="ob-21"))
        self.assertEqual(names(out), ["COMMAND_ACK", "OFFER_ACCEPT", "COMMAND_ACK"])
        self.assertNotIn("stop", [c[0] for c in agent.calls])

    def test_marks_that_cannot_be_persisted_mean_no_action(self):
        agent = OfferAgent("accept")
        link, sio = engine_link(agent=agent, commitment_path="/proc/robotx-test/commitments.json")
        self.assertEqual(fire(link, sio, fx.envelope()), [])
        self.assertEqual(agent.calls, [])


# --- 9 ------------------------------------------------------------------------


class T09TaskAssignDoesNotStartAMission(AsyncTestCase):
    def test_the_real_agent_stays_idle(self):
        agent = real_agent()
        sio = FakeSio()
        cfg = BackendConfig(enabled=True, robot_id=fx.ROBOT_ID, server_url="http://192.0.2.1:1",
                            pairing_code="123456", command_signing_key=KEY)
        link = BackendLink(cfg, agent.state, agent, client_factory=lambda _c: sio,
                           token_store=MemoryTokenStore())

        async def scenario():
            await connect_and_auth(link)
            before = len(sio.emitted)
            await sio.fire("TASK_ASSIGN", task_assign_payload(task_id="T-123"))
            return sio.emitted[before:]

        import asyncio

        self.assertEqual(asyncio.run(scenario()), [])
        self.assertIsNone(agent.state.snapshot().mission)
        self.assertEqual(link.stats["tasks_recovery_resends"], 1)

    def test_a_recovery_resend_for_the_held_task_is_correlated_not_restarted(self):
        state = state_with_fix(fx.ROBOT_ID)
        agent = OfferAgent("accept", state=state)
        link, sio = engine_link(agent=agent, state=state)
        fire(link, sio, fx.envelope())

        import asyncio

        asyncio.run(sio.fire("TASK_ASSIGN", task_assign_payload(task_id="T-123")))
        self.assertEqual([c for c in agent.calls if c[0] == "assign"], [("assign", "T-123", True)])


# --- 10, 11: custody ----------------------------------------------------------


class CustodyHarness:
    """A real MissionManager driven along SYNTHETIC measured/unmeasured poses."""

    def __init__(self):
        from robotx.communication.protocol import parse_task_assign
        from tests.unit.test_mission import MissionHarness

        self.h = MissionHarness(mission=parse_task_assign(task_assign_payload(task_id="T-123")))
        self.h.manager.assign(self.h.mission, custody_required=True)

    def tick(self, point, measured=True):
        from tests.unit.test_mission import pose

        return self.h.manager.update(self.h.navigator.update(pose(point)), position_measured=measured)

    def drive(self, points, measured=True):
        return [self.tick(p, measured) for p in points][-1]

    @property
    def active(self):
        return self.h.manager.active


class T10Acquired(unittest.TestCase):
    def test_arriving_at_the_pickup_is_not_custody(self):
        c = CustodyHarness()
        c.drive(c.h.mission.path_to_pickup)
        for _ in range(3):  # holds; never departs without the parcel
            c.tick(PICKUP)
        self.assertIs(c.active.status, MissionStatus.AT_PICKUP)
        self.assertIsNone(c.active.custody_acquired_at)

    def test_acquired_only_at_a_measured_pickup_and_only_once(self):
        c = CustodyHarness()
        with self.assertRaises(MissionRefused):  # still driving to the pickup
            c.h.manager.record_custody("ACQUIRED", source="SIMULATED")
        c.drive(c.h.mission.path_to_pickup)
        c.h.manager.record_custody("ACQUIRED", source="SIMULATED")
        with self.assertRaises(MissionRefused):
            c.h.manager.record_custody("ACQUIRED", source="SIMULATED")
        self.assertIs(c.tick(PICKUP).active.status, MissionStatus.TO_DROP)

    def test_acquired_is_refused_after_an_unmeasured_arrival(self):
        c = CustodyHarness()
        c.drive(c.h.mission.path_to_pickup, measured=False)
        with self.assertRaises(MissionRefused):
            c.h.manager.record_custody("ACQUIRED", source="SIMULATED")

    def test_the_real_rover_has_no_custody_source(self):
        self.assertFalse(real_agent().custody_sensing_available())


class T11Released(unittest.TestCase):
    def at_drop(self):
        c = CustodyHarness()
        c.drive(c.h.mission.path_to_pickup)
        c.h.manager.record_custody("ACQUIRED", source="SIMULATED")
        c.tick(PICKUP)
        c.drive(c.h.mission.path_to_drop)
        return c

    def test_arriving_at_the_drop_is_not_delivery(self):
        c = self.at_drop()
        for _ in range(3):
            c.tick(DROP)
        self.assertIs(c.active.status, MissionStatus.AT_DROP)
        self.assertFalse(c.active.is_complete)

    def test_released_only_at_a_measured_drop_then_complete(self):
        c = CustodyHarness()
        with self.assertRaises(MissionRefused):
            c.h.manager.record_custody("RELEASED", source="SIMULATED")
        c = self.at_drop()
        c.h.manager.record_custody("RELEASED", source="SIMULATED")
        update = c.tick(DROP)
        self.assertTrue(update.completed)
        self.assertEqual(update.active.completed_at, update.active.custody_released_at)


class CustodyLinkTests(AsyncTestCase):
    """The link emits CUSTODY_EVENT only from mission state, once per kind."""

    def accepted(self):
        state = state_with_fix(fx.ROBOT_ID)
        agent = OfferAgent("accept", state=state)
        link, sio = engine_link(agent=agent, state=state, max_position_age_s=30.0)
        fire(link, sio, fx.envelope())
        return link, sio, state

    def set_mission(self, state, **changes):
        state.update_mission(replace(state.snapshot().mission, **changes))

    def publish(self, link, times=1):
        import asyncio

        async def go():
            for _ in range(times):
                await link._publish_custody()
                await link._publish_task_complete()

        asyncio.run(go())

    def test_no_custody_event_without_custody_state(self):
        link, sio, state = self.accepted()
        self.set_mission(state, status=MissionStatus.AT_PICKUP, pickup_measured=True)
        self.publish(link, 3)
        self.assertEqual(sio.events_named("CUSTODY_EVENT"), [])

    def test_acquired_then_released_each_exactly_once(self):
        link, sio, state = self.accepted()
        self.set_mission(state, status=MissionStatus.AT_PICKUP, pickup_measured=True,
                         custody_acquired_at=time.time())
        self.publish(link, 3)
        self.assertEqual(sio.events_named("CUSTODY_EVENT"),
                         [{"commitmentId": "c-7f3a", "fence": "42", "kind": "ACQUIRED"}])
        self.set_mission(state, status=MissionStatus.AT_DROP, drop_measured=True,
                         custody_released_at=time.time())
        self.publish(link, 3)
        self.assertEqual([e["kind"] for e in sio.events_named("CUSTODY_EVENT")],
                         ["ACQUIRED", "RELEASED"])


# --- 12, 13, 14: completion ---------------------------------------------------


class CompletionTests(CustodyLinkTests):
    def run_mission(self, *, measured=True, fixes=True):
        """Accept, drive the drop leg on SYNTHETIC fixes, hand over, finish."""

        import asyncio

        link, sio, state = self.accepted()
        # The commitment was granted 15 s ago; the fixes below fill that span.
        link._granted_at_ms["c-7f3a"] = now_ms() - 15_000
        if fixes:
            async def send_fixes():
                points = path_to_drop()[1:]
                for i, (lat, lon) in enumerate(points):
                    t = time.time() - 12.0 + 3.0 * i if i < len(points) - 1 else time.time() - 0.5
                    state.update_gps(
                        GpsReading(status=GPSStatus.FIX,
                                   fix=GpsFix(latitude=lat, longitude=lon, timestamp=t), age_s=0.1),
                        Position(latitude=lat, longitude=lon, timestamp=t),
                    )
                    await link._publish_telemetry()

            asyncio.run(send_fixes())
        self.set_mission(state, status=MissionStatus.COMPLETE, pickup_measured=measured,
                         drop_measured=measured, custody_acquired_at=time.time() - 20,
                         custody_released_at=time.time(), completed_at=time.time())
        self.publish(link, 3)
        return link, sio

    def test_T12_completion_is_claimed_on_measured_evidence(self):
        link, sio = self.run_mission()
        reports = sio.events_named("TASK_COMPLETE")
        self.assertEqual(len(reports), 1)
        self.assertEqual(set(reports[0]), {"taskId", "lat", "lon"})
        self.assertEqual(reports[0]["taskId"], "T-123")
        self.assertAlmostEqual(reports[0]["lat"], DROP[0], places=6)
        self.assertIsInstance(reports[0]["lat"], float)

    def test_T13_dead_reckoned_arrivals_never_produce_completion(self):
        link, sio = self.run_mission(measured=False)
        self.assertEqual(sio.events_named("TASK_COMPLETE"), [])
        self.assertEqual(link.stats["task_completes_withheld"], 1)

    def test_T14_missing_gps_never_produces_completion(self):
        link, sio = self.run_mission(fixes=False)
        self.assertEqual(sio.events_named("TASK_COMPLETE"), [])
        self.assertEqual(link.stats["task_completes_withheld"], 1)

    def test_completion_waits_for_released_to_have_been_sent(self):
        link, sio, state = self.accepted()
        self.set_mission(state, status=MissionStatus.COMPLETE, pickup_measured=True,
                         drop_measured=True, completed_at=time.time())
        self.publish(link, 2)  # complete but no custody recorded
        self.assertEqual(sio.events_named("TASK_COMPLETE"), [])
        self.assertEqual(link.stats["task_completes_withheld"], 0)

    def test_verifying_reply_is_not_retried(self):
        import asyncio

        link, sio = self.run_mission()
        asyncio.run(sio.fire("TASK_COMPLETE_ACK", {"taskId": "T-123", "verifying": True}))
        self.publish(link, 3)
        self.assertEqual(len(sio.events_named("TASK_COMPLETE")), 1)
        self.assertEqual(link.stats["task_completes_verifying"], 1)


# --- Y2: a completion claim survives the loss of the socket that carried it ----


class Y2CompletionDelivery(AsyncTestCase):
    """Y2 -- TASK_COMPLETE is persisted before it is emitted and resent, unchanged,
    after each successful AUTH until the backend's TASK_COMPLETE_ACK is seen.

    Each scenario runs in ONE event loop, as the agent does: the link's asyncio
    events belong to the loop that first awaits them. The mission is the same
    one the completion tests drive -- accepted, measured fixes over the drop leg,
    ACQUIRED and RELEASED, complete -- through the real publish paths.
    """

    ACK = {"taskId": "T-123", "timestamp": 1}

    # --- one loop, real paths ---------------------------------------------------

    async def accept(self, **cfg):
        state = state_with_fix(fx.ROBOT_ID)
        agent = OfferAgent("accept", state=state)
        link, sio = engine_link(agent=agent, state=state, max_position_age_s=30.0, **cfg)
        await connect_and_auth(link)
        await sio.fire("OFFER", fx.envelope())
        return link, sio, state

    async def publish(self, link, times=1):
        for _ in range(times):
            await link._publish_custody()
            await link._publish_task_complete()

    async def complete(self, link, sio, state, *, drop_before_claim=False):
        """Drive the drop leg on SYNTHETIC measured fixes, hand over, finish."""

        link._granted_at_ms["c-7f3a"] = now_ms() - 15_000
        points = path_to_drop()[1:]
        for i, (lat, lon) in enumerate(points):
            t = time.time() - 12.0 + 3.0 * i if i < len(points) - 1 else time.time() - 0.5
            state.update_gps(
                GpsReading(status=GPSStatus.FIX, fix=GpsFix(latitude=lat, longitude=lon, timestamp=t), age_s=0.1),
                Position(latitude=lat, longitude=lon, timestamp=t),
            )
            await link._publish_telemetry()
        mission = state.snapshot().mission
        state.update_mission(replace(mission, status=MissionStatus.AT_DROP, pickup_measured=True, drop_measured=True,
                                     custody_acquired_at=time.time() - 20, custody_released_at=time.time()))
        await self.publish(link)  # ACQUIRED, RELEASED -- before any claim
        if drop_before_claim:
            await sio.drop()
        state.update_mission(replace(state.snapshot().mission, status=MissionStatus.COMPLETE, completed_at=time.time()))
        await self.publish(link, 3)

    async def reconnect(self, link, sio):
        """The socket is gone; a new session connects, authenticates and starts
        publishing -- the real `_publish_loop` entry, which ends at once here."""

        if sio.connected:
            await sio.drop()
        ok = await connect_and_auth(link)
        if ok:
            link._running = False
            await link._publish_loop()
        return ok

    async def ack(self, sio, payload=None):
        await sio.fire("TASK_COMPLETE_ACK", self.ACK if payload is None else payload)

    @staticmethod
    def record(link):
        return link.commitments.get("c-7f3a")

    # --- D, E -------------------------------------------------------------------

    def test_D_a_claim_lost_with_its_socket_is_resent_unchanged_after_reconnect_until_acked(self):
        async def scenario():
            link, sio, state = await self.accept()
            await self.complete(link, sio, state)
            first = sio.events_named("TASK_COMPLETE")
            r = self.record(link)
            persisted = (r.completion, r.completion_claim, r.completion_acked)
            reconnected = await self.reconnect(link, sio)  # the socket died before any ACK
            claims = list(sio.events_named("TASK_COMPLETE"))
            pending = self.record(link).completion_unacknowledged
            await self.ack(sio)
            acked = self.record(link).completion_acked
            await self.reconnect(link, sio)
            return first, persisted, reconnected, claims, pending, acked, sio.events_named("TASK_COMPLETE")

        first, persisted, reconnected, claims, pending, acked, final = self.run_async(scenario())
        self.assertEqual(len(first), 1)
        self.assertEqual(persisted, ("SENT", first[0], False))
        self.assertTrue(reconnected)
        self.assertEqual(claims, [first[0], first[0]])  # the same report, not a new one
        self.assertTrue(pending)  # unresolved until ACKed
        self.assertTrue(acked)
        self.assertEqual(len(final), 2)  # settled: never again

    def test_E_an_acked_claim_is_not_resent_after_reconnect(self):
        for reply in (self.ACK, {"taskId": "T-123", "verifying": True},
                      {"taskId": "T-123", "verifying": True, "reason": "CUSTODY_STILL_HELD"},
                      {"taskId": "T-123", "alreadyCompleted": True}):
            with self.subTest(reply=reply):
                async def scenario():
                    link, sio, state = await self.accept()
                    await self.complete(link, sio, state)
                    await self.ack(sio, reply)
                    acked = self.record(link).completion_acked
                    await self.reconnect(link, sio)
                    await self.reconnect(link, sio)
                    return acked, sio.events_named("TASK_COMPLETE")

                acked, claims = self.run_async(scenario())
                self.assertTrue(acked)
                self.assertEqual(len(claims), 1)

    # --- F ----------------------------------------------------------------------

    def test_F_a_restart_resends_the_persisted_claim_after_auth(self):
        from robotx.communication.commitment_store import CommitmentStore

        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "commitments.json")

            async def first_process():
                link, sio, state = await self.accept(commitment_path=path)
                await self.complete(link, sio, state)
                return sio.events_named("TASK_COMPLETE")[0]  # then the process dies, before any ACK

            claim = self.run_async(first_process())

            async def second_process():
                # A new process: marks from disk only -- no mission, no track, a new socket.
                link, sio = engine_link(state=state_with_fix(fx.ROBOT_ID), commitment_path=path)
                pending = link.commitments.get("c-7f3a").completion_unacknowledged
                ok = await self.reconnect(link, sio)
                resent = list(sio.events_named("TASK_COMPLETE"))
                await self.ack(sio)
                return pending, ok, resent

            pending, ok, resent = self.run_async(second_process())
            self.assertTrue(pending)
            self.assertTrue(ok)
            self.assertEqual(resent, [claim])
            self.assertTrue(CommitmentStore(path).get("c-7f3a").completion_acked)  # durable

            async def third_process():
                link, sio = engine_link(state=state_with_fix(fx.ROBOT_ID), commitment_path=path)
                await self.reconnect(link, sio)
                return sio.events_named("TASK_COMPLETE")

            self.assertEqual(self.run_async(third_process()), [])

    # --- G, H, I, J -----------------------------------------------------------

    def test_G_a_repeated_or_foreign_ack_corrupts_nothing(self):
        async def scenario():
            link, sio, state = await self.accept()
            await self.complete(link, sio, state)
            for reply in (self.ACK, self.ACK, {"taskId": "T-123", "verifying": True}):
                await self.ack(sio, reply)
            for junk in (None, {}, {"taskId": 5}, {"taskId": ""}, {"taskId": "T-OTHER"}, "T-123"):
                await self.ack(sio, junk)
            r = self.record(link)
            settled = (r.completion, r.completion_acked)
            await self.reconnect(link, sio)
            return settled, sio.events_named("TASK_COMPLETE")

        settled, claims = self.run_async(scenario())
        self.assertEqual(settled, ("SENT", True))
        self.assertEqual(len(claims), 1)

    def test_G_an_ack_for_another_task_does_not_settle_this_claim(self):
        async def scenario():
            link, sio, state = await self.accept()
            await self.complete(link, sio, state)
            await self.ack(sio, {"taskId": "T-OTHER"})
            return self.record(link).completion_unacknowledged

        self.assertTrue(self.run_async(scenario()))

    def test_H_repeated_reconnects_before_the_ack_keep_the_claim_and_resend_once_each(self):
        async def scenario():
            link, sio, state = await self.accept()
            await self.complete(link, sio, state)
            for _ in range(3):
                assert await self.reconnect(link, sio)
            counts = (len(sio.events_named("TASK_COMPLETE")), link.stats["task_completes_resent"],
                      self.record(link).completion_unacknowledged)
            await self.ack(sio)
            await self.reconnect(link, sio)
            return counts, len(sio.events_named("TASK_COMPLETE"))

        (sent, resent, pending), final = self.run_async(scenario())
        self.assertEqual((sent, resent, pending), (4, 3, True))  # 1 + one per authenticated session
        self.assertEqual(final, 4)

    def test_I_nothing_is_resent_before_authentication_succeeds(self):
        async def scenario():
            link, sio, state = await self.accept()
            await self.complete(link, sio, state)
            link.cfg = replace(link.cfg, auth_timeout_s=0.05)
            outcomes = []
            for mode in ("none", "silent_disconnect", "rejected"):
                if sio.connected:
                    await sio.drop()
                sio.auth_mode = mode
                ok = await connect_and_auth(link)
                if not ok:
                    await link._handle_auth_failure()
                outcomes.append((mode, ok, len(sio.events_named("TASK_COMPLETE"))))
            sio.auth_mode = "success"
            ok = await self.reconnect(link, sio)
            return outcomes, ok, len(sio.events_named("TASK_COMPLETE"))

        outcomes, ok, final = self.run_async(scenario())
        self.assertEqual(outcomes, [("none", False, 1), ("silent_disconnect", False, 1), ("rejected", False, 1)])
        self.assertTrue(ok)
        self.assertEqual(final, 2)

    def test_J_order_custody_before_the_claim_then_heartbeat_telemetry_before_the_resend(self):
        async def scenario():
            link, sio, state = await self.accept()
            await self.complete(link, sio, state)
            original = names(sio.emitted)
            before = len(sio.emitted)
            await self.reconnect(link, sio)
            return original, names(sio.emitted[before:])

        original, after = self.run_async(scenario())
        custody = [i for i, n in enumerate(original) if n == "CUSTODY_EVENT"]
        self.assertEqual(len(custody), 2)
        self.assertLess(custody[-1], original.index("TASK_COMPLETE"))  # RELEASED, then the claim
        self.assertEqual(after[:2], ["AUTH", "HEARTBEAT"])
        self.assertLess(after.index("TELEMETRY"), after.index("TASK_COMPLETE"))

    # --- persist before emit -------------------------------------------------

    def test_a_claim_made_while_the_socket_is_already_down_is_sent_after_auth(self):
        async def scenario():
            link, sio, state = await self.accept()
            await self.complete(link, sio, state, drop_before_claim=True)
            made = (sio.events_named("TASK_COMPLETE"), self.record(link).completion_unacknowledged)
            await self.reconnect(link, sio)
            return made, sio.events_named("TASK_COMPLETE")

        (sent_while_down, pending), after = self.run_async(scenario())
        self.assertEqual(sent_while_down, [])
        self.assertTrue(pending)
        self.assertEqual(len(after), 1)

    def test_a_claim_that_cannot_be_persisted_is_not_emitted(self):
        async def scenario():
            link, sio, state = await self.accept()
            update = link.commitments.update
            link.commitments.update = lambda cid, **ch: None if "completion_claim" in ch else update(cid, **ch)
            await self.complete(link, sio, state)
            return sio.events_named("TASK_COMPLETE")

        self.assertEqual(self.run_async(scenario()), [])

    def test_an_unsettled_commitment_resends_custody_before_the_claim_but_never_a_finished_accept(self):
        """No acknowledgement exists for CUSTODY_EVENT or OFFER_*: custody is resent
        until the completion is settled, before the claim; an ACCEPT is not replayed
        once the mission it accepted is no longer being carried out."""

        async def scenario():
            link, sio, state = await self.accept()
            await self.complete(link, sio, state)
            before = len(sio.emitted)
            await self.reconnect(link, sio)
            return names(sio.emitted[before:])

        resent = self.run_async(scenario())
        self.assertEqual([n for n in resent if n in ("CUSTODY_EVENT", "TASK_COMPLETE")],
                         ["CUSTODY_EVENT", "CUSTODY_EVENT", "TASK_COMPLETE"])
        for name in ("OFFER_ACCEPT", "OFFER_REJECT", "OFFER_DEFER"):
            self.assertNotIn(name, resent)


class Y2CustodyAndOfferDelivery(AsyncTestCase):
    """Y2 -- CUSTODY_EVENT and OFFER_* responses survive the loss of their socket
    and a restart: each is persisted, payload and all, before it is emitted and
    resent unchanged after each successful AUTH while it can still matter.

    The backend acknowledges neither (Y2 protocol-gap audit, Option A): it
    ignores a duplicate it already applied, so the Pi bounds the resends itself --
    tombstone, a settled completion, the offer's expiry, a DEFER's own `until`,
    and for an ACCEPT, the mission still being carried out. The helpers are
    borrowed from `Y2CompletionDelivery`, so none of its tests runs twice; each
    scenario is one event loop.
    """

    accept = Y2CompletionDelivery.accept
    publish = Y2CompletionDelivery.publish
    complete = Y2CompletionDelivery.complete
    reconnect = Y2CompletionDelivery.reconnect
    ack = Y2CompletionDelivery.ack
    record = staticmethod(Y2CompletionDelivery.record)
    ACK = Y2CompletionDelivery.ACK

    ACQUIRED = {"commitmentId": "c-7f3a", "fence": "42", "kind": "ACQUIRED"}
    RELEASED = {"commitmentId": "c-7f3a", "fence": "42", "kind": "RELEASED"}

    async def answer(self, decision, **cfg):
        """An OFFER answered with `decision` (an OfferDecision), over the link."""

        state = state_with_fix(fx.ROBOT_ID)
        agent = OfferAgent(decision, state=state)
        link, sio = engine_link(agent=agent, state=state, **cfg)
        await connect_and_auth(link)
        await sio.fire("OFFER", fx.envelope())
        return link, sio, state

    async def acquire(self, link, state):
        state.update_mission(replace(state.snapshot().mission, status=MissionStatus.AT_PICKUP,
                                     pickup_measured=True, custody_acquired_at=time.time()))
        await self.publish(link)

    async def restarted(self, path):
        """A new process: marks from disk only -- no mission, no track, a new socket."""

        link, sio = engine_link(state=state_with_fix(fx.ROBOT_ID), commitment_path=path)
        sent_at = []
        emit = sio.emit

        async def timed(event, payload, namespace=None):
            sent_at.append((event, time.monotonic()))
            return await emit(event, payload, namespace=namespace)

        sio.emit = timed
        ok = await self.reconnect(link, sio)
        return link, sio, ok, sent_at

    @staticmethod
    def custody(sio):
        return sio.events_named("CUSTODY_EVENT")

    # --- custody ------------------------------------------------------------------

    def test_C1_a_persisted_acquired_is_resent_exactly_after_restart_and_auth(self):
        from robotx.communication.commitment_store import CommitmentStore

        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "commitments.json")

            async def first():
                link, sio, state = await self.accept(commitment_path=path)
                await self.acquire(link, state)
                return self.custody(sio)  # the process dies here; the socket with it

            original = self.run_async(first())
            on_disk = CommitmentStore(path).get("c-7f3a")

            async def second():
                link, sio, ok, _ = await self.restarted(path)
                return ok, self.custody(sio)

            ok, resent = self.run_async(second())
        self.assertEqual(original, [self.ACQUIRED])
        self.assertEqual((on_disk.custody_sent, on_disk.custody_payloads), (["ACQUIRED"], {"ACQUIRED": self.ACQUIRED}))
        self.assertTrue(ok)
        self.assertEqual(resent, [self.ACQUIRED])  # the same report, identity unchanged

    def test_C2_acquired_then_released_are_resent_in_order_at_least_50ms_apart(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "commitments.json")

            async def first():
                link, sio, state = await self.accept(commitment_path=path)
                await self.acquire(link, state)
                state.update_mission(replace(state.snapshot().mission, status=MissionStatus.AT_DROP,
                                             drop_measured=True, custody_released_at=time.time()))
                await self.publish(link)

            self.run_async(first())

            async def second():
                link, sio, ok, sent_at = await self.restarted(path)
                return ok, self.custody(sio), [t for e, t in sent_at if e == "CUSTODY_EVENT"]

            ok, resent, times = self.run_async(second())
        self.assertTrue(ok)
        self.assertEqual(resent, [self.ACQUIRED, self.RELEASED])
        self.assertGreaterEqual(times[1] - times[0], 0.05)

    def test_C2_two_kinds_reported_in_one_tick_are_also_spaced_on_the_live_path(self):
        async def scenario():
            link, sio, state = await self.accept()
            state.update_mission(replace(state.snapshot().mission, status=MissionStatus.AT_DROP, pickup_measured=True,
                                         drop_measured=True, custody_acquired_at=time.time(), custody_released_at=time.time()))
            sent_at = []
            emit = sio.emit

            async def timed(event, payload, namespace=None):
                sent_at.append((event, time.monotonic()))
                return await emit(event, payload, namespace=namespace)

            sio.emit = timed
            await self.publish(link)
            return self.custody(sio), [t for e, t in sent_at if e == "CUSTODY_EVENT"]

        sent, times = self.run_async(scenario())
        self.assertEqual(sent, [self.ACQUIRED, self.RELEASED])
        self.assertGreaterEqual(times[1] - times[0], 0.05)

    def test_C3_resends_never_add_a_report_or_change_its_identity(self):
        async def scenario():
            link, sio, state = await self.accept()
            await self.complete(link, sio, state)
            for _ in range(3):
                await self.reconnect(link, sio)
            await self.publish(link, 3)  # the live path does not report again either
            r = self.record(link)
            return self.custody(sio), r.custody_sent, r.custody_payloads

        sent, custody_sent, payloads = self.run_async(scenario())
        self.assertEqual(custody_sent, ["ACQUIRED", "RELEASED"])
        self.assertEqual(payloads, {"ACQUIRED": self.ACQUIRED, "RELEASED": self.RELEASED})
        self.assertEqual(sent, [self.ACQUIRED, self.RELEASED] * 4)  # 1 original + 3 sessions, same two reports

    def test_C4_a_settled_or_already_completed_ack_ends_the_custody_resends(self):
        for reply in (self.ACK, {"taskId": "T-123", "alreadyCompleted": True}):
            with self.subTest(reply=reply):
                async def scenario():
                    link, sio, state = await self.accept()
                    await self.complete(link, sio, state)
                    await self.ack(sio, reply)
                    before = len(self.custody(sio))
                    await self.reconnect(link, sio)
                    await self.reconnect(link, sio)
                    return before, len(self.custody(sio)), self.record(link)

                before, after, r = self.run_async(scenario())
                self.assertEqual(before, after)
                self.assertTrue(r.custody_settled)

    def test_C5_verifying_custody_still_held_keeps_released_going_until_the_commitment_ends(self):
        async def scenario():
            link, sio, state = await self.accept()
            await self.complete(link, sio, state)
            await self.ack(sio, {"taskId": "T-123", "verifying": True, "reason": "CUSTODY_STILL_HELD"})
            r = self.record(link)
            recorded = (r.completion_acked, r.completion_ack_result, r.completion_ack_reason, r.custody_settled)
            before = len(self.custody(sio))
            await self.reconnect(link, sio)
            resent = self.custody(sio)[before:]
            claims = len(sio.events_named("TASK_COMPLETE"))
            # The commitment ends: the backend withdraws it.
            await sio.fire("WITHDRAW", fx.envelope(command="WITHDRAW", fence="43", sequence=1, outbox_id="ob-20"))
            ended = self.record(link).tombstoned
            before = len(self.custody(sio))
            await self.reconnect(link, sio)
            return recorded, resent, claims, ended, self.custody(sio)[before:]

        recorded, resent, claims, ended, after_end = self.run_async(scenario())
        self.assertEqual(recorded, (True, "VERIFYING", "CUSTODY_STILL_HELD", False))
        self.assertEqual(resent, [self.ACQUIRED, self.RELEASED])
        self.assertEqual(claims, 1)  # the acknowledged claim itself is not repeated
        self.assertTrue(ended)
        self.assertEqual(after_end, [])

    def test_C_custody_made_while_the_socket_is_down_is_persisted_and_sent_after_auth(self):
        async def scenario():
            link, sio, state = await self.accept()
            await sio.drop()
            await self.acquire(link, state)  # reported with no socket: persisted, not emitted
            made = (self.custody(sio), self.record(link).custody_payloads)
            await self.reconnect(link, sio)
            return made, self.custody(sio)

        (sent_while_down, persisted), after = self.run_async(scenario())
        self.assertEqual(sent_while_down, [])
        self.assertEqual(persisted, {"ACQUIRED": self.ACQUIRED})
        self.assertEqual(after, [self.ACQUIRED])

    def test_C_custody_that_cannot_be_persisted_is_not_emitted(self):
        async def scenario():
            link, sio, state = await self.accept()
            update = link.commitments.update
            link.commitments.update = lambda cid, **ch: None if "custody_payloads" in ch else update(cid, **ch)
            await self.acquire(link, state)
            return self.custody(sio)

        self.assertEqual(self.run_async(scenario()), [])

    # --- OFFER responses ----------------------------------------------------------

    def test_O1_accept_is_persisted_exactly_and_resent_while_its_mission_is_active(self):
        async def scenario():
            link, sio, state = await self.accept()
            original = sio.events_named("OFFER_ACCEPT")
            r = self.record(link)
            persisted = (r.response, r.response_payload, r.offer_expiry)
            await self.reconnect(link, sio)  # mission active, offer unexpired
            return original, persisted, sio.events_named("OFFER_ACCEPT")

        original, (response, payload, expiry), after = self.run_async(scenario())
        self.assertEqual(original, [{"commitmentId": "c-7f3a", "fence": "42"}])
        self.assertEqual((response, payload), ("ACCEPT", original[0]))
        self.assertAlmostEqual(expiry, time.time() + 20.0, delta=3.0)
        self.assertEqual(after, [original[0], original[0]])

    def test_O1_accept_is_durable_across_a_restart_but_not_replayed_without_its_mission(self):
        from robotx.communication.commitment_store import CommitmentStore

        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "commitments.json")

            async def first():
                link, sio, state = await self.accept(commitment_path=path)
                return sio.events_named("OFFER_ACCEPT")[0]

            original = self.run_async(first())
            on_disk = CommitmentStore(path).get("c-7f3a")

            async def second():
                link, sio, ok, _ = await self.restarted(path)  # the mission did not survive
                return ok, sio.events_named("OFFER_ACCEPT")

            ok, resent = self.run_async(second())
        self.assertEqual((on_disk.response, on_disk.response_payload), ("ACCEPT", original))
        self.assertTrue(ok)
        self.assertEqual(resent, [])

    def test_O1_accept_is_not_resent_once_the_mission_is_abandoned_or_the_offer_expired(self):
        for how in ("abandoned", "expired", "withdrawn"):
            with self.subTest(how=how):
                async def scenario():
                    link, sio, state = await self.accept()
                    if how == "abandoned":  # a local STOP marks the mission ABORTED
                        state.update_mission(replace(state.snapshot().mission, status=MissionStatus.ABORTED))
                    elif how == "expired":
                        link.commitments.update("c-7f3a", offer_expiry=time.time() - 1)
                    else:
                        await sio.fire("WITHDRAW", fx.envelope(command="WITHDRAW", fence="43", sequence=1, outbox_id="ob-20"))
                    await self.reconnect(link, sio)
                    return sio.events_named("OFFER_ACCEPT")

                self.assertEqual(len(self.run_async(scenario())), 1)

    def test_O2_reject_is_persisted_exactly_and_resent_after_restart(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "commitments.json")

            async def first():
                link, sio, _ = await self.answer(OfferDecision.reject("NO_MOTOR_LINK"), commitment_path=path)
                return sio.events_named("OFFER_REJECT")

            original = self.run_async(first())

            async def second():
                link, sio, ok, _ = await self.restarted(path)
                return ok, sio.events_named("OFFER_REJECT")

            ok, resent = self.run_async(second())
        self.assertEqual(original, [{"commitmentId": "c-7f3a", "fence": "42", "reason": "NO_MOTOR_LINK"}])
        self.assertTrue(ok)
        self.assertEqual(resent, original)

    def test_O3_defer_keeps_its_exact_until_and_reason_across_a_restart(self):
        from robotx.communication.commitment_store import CommitmentStore

        until_ms = int((time.time() + 600) * 1000)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "commitments.json")

            async def first():
                link, sio, _ = await self.answer(OfferDecision.defer(until_ms, "CHARGING"), commitment_path=path)
                return sio.events_named("OFFER_DEFER")

            original = self.run_async(first())
            on_disk = CommitmentStore(path).get("c-7f3a").response_payload
            time.sleep(0.05)  # a resend made now must not carry a "now"-based until

            async def second():
                link, sio, ok, _ = await self.restarted(path)
                return ok, sio.events_named("OFFER_DEFER")

            ok, resent = self.run_async(second())
        expected = {"commitmentId": "c-7f3a", "fence": "42", "until": until_ms, "reason": "CHARGING"}
        self.assertEqual(original, [expected])
        self.assertEqual(on_disk, expected)
        self.assertTrue(ok)
        self.assertEqual(resent, [expected])

    def test_O3_a_defer_whose_until_or_offer_has_passed_is_not_replayed(self):
        import asyncio

        for how in ("until_passed", "offer_expired"):
            with self.subTest(how=how):
                async def scenario():
                    until_ms = int((time.time() + 0.3) * 1000) if how == "until_passed" else int((time.time() + 600) * 1000)
                    link, sio, _ = await self.answer(OfferDecision.defer(until_ms, "CHARGING"))
                    if how == "until_passed":
                        await asyncio.sleep(0.4)
                    else:
                        link.commitments.update("c-7f3a", offer_expiry=time.time() - 1)
                    await self.reconnect(link, sio)
                    return sio.events_named("OFFER_DEFER")

                self.assertEqual(len(self.run_async(scenario())), 1)

    def test_O3_an_iso_until_is_honoured_too(self):
        async def scenario():
            later = fx.iso(time.time() + 600)
            link, sio, _ = await self.answer(OfferDecision.defer(later, "CHARGING"))
            await self.reconnect(link, sio)
            return later, sio.events_named("OFFER_DEFER")

        later, sent = self.run_async(scenario())
        self.assertEqual([p["until"] for p in sent], [later, later])

    # --- boundaries ---------------------------------------------------------------

    def test_nothing_is_resent_before_authentication_succeeds(self):
        async def scenario():
            link, sio, state = await self.accept()
            await self.acquire(link, state)
            link.cfg = replace(link.cfg, auth_timeout_s=0.05)
            counts = []
            for mode in ("none", "silent_disconnect", "rejected"):
                if sio.connected:
                    await sio.drop()
                sio.auth_mode = mode
                ok = await connect_and_auth(link)
                if not ok:
                    await link._handle_auth_failure()
                counts.append((len(self.custody(sio)), len(sio.events_named("OFFER_ACCEPT"))))
            sio.auth_mode = "success"
            await self.reconnect(link, sio)
            return counts, (len(self.custody(sio)), len(sio.events_named("OFFER_ACCEPT")))

        counts, final = self.run_async(scenario())
        self.assertEqual(counts, [(1, 1), (1, 1), (1, 1)])
        self.assertEqual(final, (2, 2))

    def test_resend_order_is_offer_response_then_custody(self):
        async def scenario():
            link, sio, state = await self.accept()
            await self.acquire(link, state)
            before = len(sio.emitted)
            await self.reconnect(link, sio)
            return [n for n in names(sio.emitted[before:]) if n in ("OFFER_ACCEPT", "CUSTODY_EVENT", "TASK_COMPLETE")]

        self.assertEqual(self.run_async(scenario()), ["OFFER_ACCEPT", "CUSTODY_EVENT"])


class TestL1Evidence(unittest.TestCase):
    """The five backend conditions, each able to fail on its own."""

    def track(self, *, spacing_s=3.0, points=None, end_offset_s=0.5):
        now = now_ms()
        points = points or path_to_drop()[1:]
        n = len(points)
        return [TrackFix(now - int((end_offset_s + spacing_s * (n - 1 - i)) * 1000), lat, lon)
                for i, (lat, lon) in enumerate(points)]

    def assess(self, track, granted_before_first_s=2.0):
        return assess_completion(
            track, granted_at_ms=track[0].t_ms - int(granted_before_first_s * 1000) if track else now_ms(),
            claim_at_ms=now_ms(), final_stop=DROP,
            commanded_path=list(path_to_drop()),
        )

    def test_a_good_track_passes(self):
        self.assertTrue(self.assess(self.track()).sufficient)

    def test_final_fix_too_far_from_the_stop(self):
        # Ends two waypoints (~34 m) short of the drop: outside the 25 m radius.
        track = self.track(points=path_to_drop()[1:-2])
        self.assertIn("from the final stop", " ".join(self.assess(track).failures))

    def test_a_gap_over_ten_seconds(self):
        self.assertIn("gap", " ".join(self.assess(self.track(spacing_s=11.0)).failures))

    def test_a_late_claim_is_a_gap(self):
        self.assertIn("gap", " ".join(self.assess(self.track(end_offset_s=11.0)).failures))

    def test_implied_speed_over_limit(self):
        self.assertIn("speed", " ".join(self.assess(self.track(spacing_s=1.0)).failures))

    def test_track_outside_the_corridor(self):
        far = [(lat + 0.001, lon) for lat, lon in path_to_drop()[1:-1]] + [DROP]
        self.assertIn("corridor", " ".join(self.assess(self.track(points=far)).failures))

    def test_fix_rate_too_low(self):
        verdict = self.assess(self.track(), granted_before_first_s=60.0)
        self.assertIn("fix rate", " ".join(verdict.failures))

    def test_no_fixes_is_insufficient(self):
        self.assertFalse(assess_completion([], granted_at_ms=now_ms(), claim_at_ms=now_ms(),
                                           final_stop=DROP, commanded_path=list(path_to_drop())).sufficient)


# --- 15, 16 -------------------------------------------------------------------


class T15DuplicateOffer(AsyncTestCase):
    def test_redelivery_is_reacked_never_answered_again(self):
        state = state_with_fix(fx.ROBOT_ID)
        agent = OfferAgent("accept", state=state)
        link, sio = engine_link(agent=agent, state=state)
        env = fx.envelope()
        out = fire(link, sio, env, env, env)
        self.assertEqual(names(out), ["COMMAND_ACK", "OFFER_ACCEPT", "COMMAND_ACK", "COMMAND_ACK"])
        self.assertEqual(len([c for c in agent.calls if c[0] == "assign"]), 1)

    def test_a_restart_does_not_answer_twice(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "commitments.json")
            state = state_with_fix(fx.ROBOT_ID)
            first, sio1 = engine_link(agent=OfferAgent("accept", state=state), state=state,
                                      commitment_path=path)
            env = fx.envelope()
            self.assertIn("OFFER_ACCEPT", names(fire(first, sio1, env)))
            self.assertEqual(os.stat(path).st_mode & 0o777, 0o600)

            # A new process: same persisted marks, fresh everything else.
            agent2 = OfferAgent("accept")
            second, sio2 = engine_link(agent=agent2, commitment_path=path)
            out = fire(second, sio2, env,
                       fx.envelope(fence="43", sequence=1, outbox_id="ob-20"))
            self.assertNotIn("OFFER_ACCEPT", names(out))
            self.assertNotIn("OFFER_REJECT", names(out))
            self.assertEqual(agent2.calls, [])


class T16ReconnectDoesNotDuplicate(CompletionTests):
    def test_nothing_is_resent_after_a_reconnect(self):
        import asyncio

        link, sio = self.run_mission()

        async def reconnect_and_publish():
            await sio.drop()
            # FakeSio answers AUTH synchronously; connecting is enough here,
            # and avoids awaiting the link's events from a second event loop.
            await link._connect_once()
            self.assertTrue(link.connected)
            for _ in range(3):
                await link._publish_custody()
                await link._publish_task_complete()
            await sio.fire("OFFER", fx.envelope())  # a redelivered OFFER

        asyncio.run(reconnect_and_publish())
        self.assertEqual(len(sio.events_named("OFFER_ACCEPT")), 1)
        self.assertEqual([e["kind"] for e in sio.events_named("CUSTODY_EVENT")],
                         ["ACQUIRED", "RELEASED"])
        self.assertEqual(len(sio.events_named("TASK_COMPLETE")), 1)


class TestHeartbeatWithCommitment(CustodyLinkTests):
    def test_heartbeat_names_the_commitment_only_while_carrying_it_out(self):
        import asyncio

        link, sio, state = self.accepted()
        asyncio.run(link._publish_heartbeat())
        self.assertEqual(sio.events_named("HEARTBEAT")[-1], {"commitmentId": "c-7f3a", "fence": "42"})

        # After a restart the record survives but the mission does not: the
        # Rover is not executing it, so it must not renew the lease.
        state.update_mission(None)
        asyncio.run(link._publish_heartbeat())
        self.assertEqual(sio.events_named("HEARTBEAT")[-1], {})


if __name__ == "__main__":
    unittest.main()
