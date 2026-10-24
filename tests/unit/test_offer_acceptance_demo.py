"""The offer-acceptance demonstration (ROBOTX_OFFER_ACCEPTANCE_DEMO, default off).

What is pinned:
  - it is opt-in, and refused at startup with any motor command path enabled;
  - in it, motion and delivery prerequisites (motor drive ready, custody
    sensing, perception) stop being reasons to refuse, while e-stop, ERROR, an
    UP ESP32 link and a measured fix still are;
  - a signed OFFER goes through the real BackendLink admission (addressee,
    expiry, HMAC, payload, route and start checks) to COMMAND_ACK +
    OFFER_ACCEPT, and nothing is started: mode IDLE, no route, no mission held,
    so no HEARTBEAT renews the lease, and the ESP32 link is never asked to move;
  - a tampered, misaddressed or malformed OFFER is still not admitted;
  - with the mode off, the same Rover state answers exactly as before.

Test doubles, labelled: `FakeSio` is a SIMULATED backend socket; envelopes are
signed with the fixtures' TEST-ONLY key; the ESP32 link and the GPS fix are
SIMULATED (`_RecordingEsp32`, `give_fix`). Nothing here is evidence about the
real Rover or a real backend.
"""

import logging
import unittest

from robotx.config.settings import Settings
from robotx.mission.mission import MissionRejected, MissionRejectReason
from robotx.state.robot_state import Esp32LinkStatus, OperatingMode
from tests.fixtures import engine as fx
from tests.unit.test_engine_offer import engine_link, fire, names, parsed_offer, real_agent
from tests.unit.test_readiness_telemetry import give_fix

DEMO = {"ROBOTX_OFFER_ACCEPTANCE_DEMO": "1"}


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


class _RecordingEsp32(object):
    """SIMULATED controller link: an UP link whose motors are disabled. Records every use."""

    def __init__(self):
        self.calls = []

    def status(self):
        from types import SimpleNamespace

        self.calls.append("status")
        return SimpleNamespace(motion_ready=False)

    def __getattr__(self, name):
        # Anything else (a DRIVE, a STOP, a reset) is recorded, so a test can say none happened.
        def record(*args, **kwargs):
            self.calls.append(name)

        return record


def demo_agent(**env):
    """The Rover as it would stand for the demonstration -- every piece SIMULATED.

    ESP32 link UP with motion disabled, a measured 3D fix, no custody path and
    no camera: all that the demonstration does not need is missing.
    """

    agent = real_agent(**{**DEMO, **env})
    agent.state.update_communication(esp32=Esp32LinkStatus.UP)
    agent.esp32 = _RecordingEsp32()
    give_fix(agent)
    return agent


def normal_agent_in_the_same_state(**env):
    agent = real_agent(**env)
    agent.state.update_communication(esp32=Esp32LinkStatus.UP)
    agent.esp32 = _RecordingEsp32()
    give_fix(agent)
    return agent


class TestConfiguration(unittest.TestCase):
    def test_off_by_default(self):
        self.assertFalse(Settings.from_env({}).offer_acceptance_demo)
        self.assertFalse(Settings().offer_acceptance_demo)

    def test_enabled_only_by_its_variable(self):
        self.assertTrue(Settings.from_env(DEMO).offer_acceptance_demo)

    def test_refused_with_motor_commands_enabled(self):
        with self.assertRaises(ValueError):
            real_agent(**DEMO, ROBOTX_ESP32_MOTION_ENABLED="1")

    def test_refused_with_the_bench_pulse_enabled(self):
        with self.assertRaises(ValueError):
            real_agent(**DEMO, ROBOTX_BENCH_MOTION_ENABLED="1")

    def test_motor_commands_alone_are_still_allowed_outside_the_demonstration(self):
        real_agent(ROBOTX_ESP32_MOTION_ENABLED="1")


class TestReadiness(unittest.TestCase):
    def test_motion_and_delivery_prerequisites_do_not_block(self):
        self.assertEqual(demo_agent().readiness_blockers(), [])

    def test_the_same_state_outside_the_demonstration_is_unchanged(self):
        self.assertEqual(
            normal_agent_in_the_same_state().readiness_blockers(),
            ["NO_MOTOR_LINK", "NO_CUSTODY_SENSING"],
        )

    def test_an_esp32_link_that_is_not_up_still_blocks(self):
        for link in (Esp32LinkStatus.DISABLED, Esp32LinkStatus.DISCONNECTED, Esp32LinkStatus.STALE):
            agent = demo_agent()
            agent.state.update_communication(esp32=link)
            self.assertEqual(agent.readiness_blockers(), ["NO_MOTOR_LINK"], link)

    def test_no_fix_still_blocks(self):
        agent = real_agent(**DEMO)
        agent.state.update_communication(esp32=Esp32LinkStatus.UP)
        agent.esp32 = _RecordingEsp32()
        self.assertEqual(agent.readiness_blockers(), ["NO_POSITION_FIX"])

    def test_an_emergency_stop_still_blocks(self):
        agent = demo_agent()
        agent.emergency_stop("test")
        self.assertEqual(agent.readiness_blockers()[0], "ESTOP_LATCHED")


class TestOfferThroughTheRealLink(unittest.TestCase):
    """The real RobotAgent behind the real BackendLink, on a SIMULATED socket."""

    def answer(self, agent, *envelopes):
        link, sio = engine_link(agent=agent, state=agent.state)
        return link, fire(link, sio, *envelopes)

    def test_a_signed_offer_is_accepted_and_nothing_is_started(self):
        agent = demo_agent()
        link, out = self.answer(agent, fx.envelope())
        self.assertEqual(names(out), ["COMMAND_ACK", "OFFER_ACCEPT"])
        self.assertEqual(out[1][1]["commitmentId"], "c-7f3a")
        # Nothing to drive, and nothing driving:
        self.assertIs(agent.state.mode, OperatingMode.IDLE)
        self.assertIsNone(agent.state.snapshot().mission)
        self.assertEqual(agent._mission_route, [])
        # The only use of the ESP32 link is none at all: no DRIVE, no STOP, no status poll.
        self.assertEqual(agent.esp32.calls, [])
        # No mission is held, so HEARTBEAT is the bare form and renews no lease.
        self.assertIsNone(link._held_mission())
        # The acceptance is the recorded answer, so a redelivery cannot produce a second one.
        self.assertEqual(link.commitments.get("c-7f3a").response, "ACCEPT")

    def test_a_redelivered_offer_is_acked_but_not_answered_twice(self):
        agent = demo_agent()
        envelope = fx.envelope()
        _, out = self.answer(agent, envelope, envelope)
        self.assertEqual(names(out), ["COMMAND_ACK", "OFFER_ACCEPT", "COMMAND_ACK"])

    def test_without_a_camera_it_is_still_accepted(self):
        agent = demo_agent()  # camera and perception disabled
        self.assertFalse(agent.state.snapshot().perception.is_usable)
        _, out = self.answer(agent, fx.envelope())
        self.assertEqual(names(out), ["COMMAND_ACK", "OFFER_ACCEPT"])

    def test_a_tampered_offer_is_not_admitted(self):
        agent = demo_agent()
        envelope = fx.envelope()
        envelope["payload"]["taskId"] = "TAMPERED"
        _, out = self.answer(agent, envelope)
        self.assertEqual(out, [])
        self.assertIs(agent.state.mode, OperatingMode.IDLE)

    def test_an_offer_signed_with_another_key_is_not_admitted(self):
        _, out = self.answer(demo_agent(), fx.envelope(key="some-other-key-0123456789abcdef"))
        self.assertEqual(out, [])

    def test_an_offer_for_another_robot_is_not_admitted(self):
        _, out = self.answer(demo_agent(), fx.envelope(agent_id="RBT-9999"))
        self.assertEqual(out, [])

    def test_a_malformed_offer_is_not_admitted(self):
        payload = fx.offer_payload(commitment_id="c-7f3a", fence="42")
        payload["stopSequence"] = "not-an-array"
        _, out = self.answer(demo_agent(), fx.envelope(payload=payload))
        self.assertNotIn("OFFER_ACCEPT", names(out))

    def test_an_expired_offer_is_not_admitted(self):
        _, out = self.answer(demo_agent(), fx.envelope(valid_for_s=-5))
        self.assertEqual(out, [])

    def test_an_emergency_stop_still_refuses(self):
        agent = demo_agent()
        agent.emergency_stop("test")
        _, out = self.answer(agent, fx.envelope())
        self.assertEqual(names(out), ["COMMAND_ACK", "OFFER_REJECT"])
        self.assertEqual(out[1][1]["reason"], "ESTOP_LATCHED")

    def test_without_a_fix_it_still_refuses(self):
        agent = real_agent(**DEMO)
        agent.state.update_communication(esp32=Esp32LinkStatus.UP)
        agent.esp32 = _RecordingEsp32()
        _, out = self.answer(agent, fx.envelope())
        self.assertEqual(names(out), ["COMMAND_ACK", "OFFER_REJECT"])
        self.assertEqual(out[1][1]["reason"], "NO_POSITION_FIX")

    def test_the_same_state_outside_the_demonstration_still_refuses(self):
        agent = normal_agent_in_the_same_state()
        _, out = self.answer(agent, fx.envelope())
        self.assertEqual(names(out), ["COMMAND_ACK", "OFFER_REJECT"])
        self.assertEqual(out[1][1]["reason"], "NO_MOTOR_LINK")
        self.assertIs(agent.state.mode, OperatingMode.IDLE)


class TestNoMissionIsEverStarted(unittest.TestCase):
    def test_assign_mission_is_refused(self):
        """The test-injection path, and any other caller of assign_mission."""

        agent = demo_agent()
        from robotx.communication.engine import offer_to_mission

        with self.assertRaises(MissionRejected) as caught:
            agent.assign_mission(offer_to_mission(parsed_offer()), custody_required=True)
        self.assertIs(caught.exception.reason, MissionRejectReason.OFFER_ACCEPTANCE_DEMO)
        self.assertIs(agent.state.mode, OperatingMode.IDLE)

    def test_accept_offer_outside_the_demonstration_still_drives(self):
        """Contrast: the normal Rover, every prerequisite SIMULATED, starts the mission."""

        from robotx.communication.engine import offer_to_mission
        from tests.unit.test_readiness_telemetry import ready_agent

        agent = ready_agent()
        agent.accept_offer(offer_to_mission(parsed_offer()))
        self.assertIs(agent.state.mode, OperatingMode.AUTO)
        self.assertIsNotNone(agent.state.snapshot().mission)


if __name__ == "__main__":
    unittest.main()
