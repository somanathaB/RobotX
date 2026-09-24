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
            await sio.fire("command", env)
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
            await sio.fire("command", fx.envelope())  # a redelivered OFFER

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
