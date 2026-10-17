"""A backend RESUME may only resume the mission of the commitment the server holds live.

The defect: the Rover drops a mission only when a RECALL / WITHDRAW / ABORT is
*admitted*. One that never arrives -- sent while the link was down, held for a
sequence gap, or escalated to FAILED by the backend after its ack deadline --
leaves the mission and its route in place while the server has released the
commitment and given the Leg to another robot. A PAUSED Rover then accepted any
operator RESUME and drove that obsolete route.

The backend now names the commitment it holds live for this robot on RESUME
(`commitmentId`, or null for none). A Rover carrying a commitment-bound
mission resumes it only when the two match; otherwise the RESUME is refused --
not applied and not acknowledged, the wire's refusal (handoff section 7). A
RESUME without the field (an older backend) and a RESUME with no engine mission
held behave exactly as before.

Test doubles, labelled: `FakeSio` is a SIMULATED backend socket, envelopes are
SIMULATED and signed with a test-only key, and `OfferAgent` is a SIMULATED agent.
"""

from __future__ import annotations

import asyncio
import time

# The link's own import order first, as the other link tests do: `robotx.state` and
# `robotx.mission` import each other, and only this order resolves the cycle.
from tests.unit.test_engine_offer import OfferAgent, engine_link, fire
from tests.unit.test_backend_link import AsyncTestCase, FakeAgent, state_with_fix
from tests.fixtures import engine as fx
from robotx.state.robot_state import OperatingMode


HELD = "c-7f3a"  # the fixture OFFER's commitment


def command(command_id, type_, **extra):
    return {"commandId": command_id, "type": type_, "timestamp": int(time.time() * 1000), **extra}


class ResumeBindingTests(AsyncTestCase):
    def paused_on_an_accepted_offer(self):
        """A Rover that accepted the fixture OFFER and was then paused by the operator."""

        state = state_with_fix(fx.ROBOT_ID)
        agent = OfferAgent("accept", state=state)
        link, sio = engine_link(agent=agent, state=state, max_position_age_s=30.0)
        fire(link, sio, fx.envelope())
        self.assertIsNotNone(link._held_mission(), "fixture: the OFFER should be held and carried out")
        self.deliver(sio, command("p-1", "PAUSE"))
        self.assertIs(agent.mode, OperatingMode.PAUSED)
        return link, sio, agent

    def deliver(self, sio, *payloads):
        async def go():
            for payload in payloads:
                await sio.fire("COMMAND", payload)

        asyncio.run(go())

    @staticmethod
    def acked(sio):
        # Operator acknowledgements only; the OFFER's engine ACK shares the event and carries
        # `outboxId` instead.
        return [ack["commandId"] for ack in sio.events_named("COMMAND_ACK") if "commandId" in ack]

    @staticmethod
    def resumes(agent):
        return [call for call in agent.calls if call[0] == "resume"]

    # --- the defect -----------------------------------------------------------

    def test_resume_naming_no_live_commitment_does_not_revive_the_obsolete_route(self):
        # The server released the commitment; its RECALL never reached the Rover.
        link, sio, agent = self.paused_on_an_accepted_offer()
        self.deliver(sio, command("r-1", "RESUME", commitmentId=None))

        self.assertEqual(self.resumes(agent), [])
        self.assertIs(agent.mode, OperatingMode.PAUSED)
        self.assertNotIn("r-1", self.acked(sio))
        self.assertEqual(link.stats["commands_refused"], 1)

    def test_resume_naming_another_commitment_does_not_revive_the_obsolete_route(self):
        # The Leg was reassigned and this robot now holds (or is offered) another commitment.
        link, sio, agent = self.paused_on_an_accepted_offer()
        self.deliver(sio, command("r-2", "RESUME", commitmentId="c-other"))

        self.assertEqual(self.resumes(agent), [])
        self.assertIs(agent.mode, OperatingMode.PAUSED)
        self.assertNotIn("r-2", self.acked(sio))

    # --- legitimate resumes are unchanged --------------------------------------

    def test_resume_naming_the_held_commitment_resumes_and_is_acknowledged(self):
        link, sio, agent = self.paused_on_an_accepted_offer()
        self.deliver(sio, command("r-3", "RESUME", commitmentId=HELD))

        self.assertEqual(len(self.resumes(agent)), 1)
        self.assertIs(agent.mode, OperatingMode.AUTO)
        self.assertIn("r-3", self.acked(sio))

    def test_resume_from_an_older_backend_without_the_field_behaves_as_before(self):
        link, sio, agent = self.paused_on_an_accepted_offer()
        self.deliver(sio, command("r-4", "RESUME"))

        self.assertEqual(len(self.resumes(agent)), 1)
        self.assertIn("r-4", self.acked(sio))

    def test_resume_with_no_engine_mission_held_is_not_affected(self):
        # An idle Rover paused by the operator: nothing to bind, so the executor decides as before.
        agent = FakeAgent(mode=OperatingMode.PAUSED)
        link, sio = engine_link(agent=agent, state=state_with_fix(fx.ROBOT_ID))
        asyncio.run(link._connect_once())
        self.deliver(sio, command("r-5", "RESUME", commitmentId=None))

        self.assertEqual(len(self.resumes(agent)), 1)
        self.assertIn("r-5", self.acked(sio))

    def test_a_refused_resume_is_not_remembered_so_a_corrected_redelivery_applies(self):
        # The backend re-reads the live commitment on every redelivery of a Command row. A
        # refusal recorded as the command's outcome would answer the corrected one from the
        # duplicate table instead of deciding it.
        link, sio, agent = self.paused_on_an_accepted_offer()
        self.deliver(sio, command("r-6", "RESUME", commitmentId="c-other"))
        self.deliver(sio, command("r-6", "RESUME", commitmentId=HELD))

        self.assertEqual(len(self.resumes(agent)), 1)
        self.assertEqual(self.acked(sio), ["p-1", "r-6"])
