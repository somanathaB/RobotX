"""The production Esp32Link against the REAL ESP32 firmware, compiled as the host simulator.

    Esp32Link -> HostSimulatorPort -> rover_sim(.exe) -> real ESP32 firmware

No fake ESP32 and no Python copy of the protocol: the bytes the link reads are
produced by the firmware's own comm.cpp / protocol.cpp, compiled unchanged by
the ESP32 repository's tests/run_host_tests.py. No serial device is opened and
no motor exists.

The simulator is optional. Point ROBOTX_ESP32_SIMULATOR_EXE at the built
executable to run these tests; without it they are skipped, not failed.
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
import threading
import time
import unittest
from typing import List

from robotx.application.agent import RobotAgent
from robotx.control.motion import MotionIntent
from robotx.esp32.link import Esp32Config, Esp32Link, to_drive_units
from robotx.esp32.protocol import PROTO_VERSION, SEQ_MAX, decode_line
from robotx.esp32.sim_transport import HostSimulatorPort
from robotx.esp32.state import Esp32LinkStatus as S
from robotx.state.robot_state import OperatingMode
from tests.fixtures import esp32 as fx
from tests.unit.test_esp32_agent import _ClearScenePerception, settings as agent_settings

SIMULATOR_EXE = os.environ.get("ROBOTX_ESP32_SIMULATOR_EXE", "")

# The simulator's own inputs (ESP32 repo, tests/host/sim_hw.cpp):
#   SIM_DRIVE_AVAILABLE=1  a verified PCA9685, so TELEMETRY reports
#                          motor_drive_available:true and DRIVE can be carried
#   SIM_ROM_NOISE=1        ESP32-boot-ROM-style plain text before setup()
SIM_ENV = {"SIM_DRIVE_AVAILABLE": "1", "SIM_ROM_NOISE": "1"}

BRING_UP_TIMEOUT_S = 5.0
DRIVE_TIMEOUT_S = 5.0
FORWARD_SPEED = 0.5

# The firmware's command watchdog: ESP32 repo config.h COMMAND_TIMEOUT_MS. It
# fires once more than this has passed since the last accepted DRIVE/MOVE/STOP.
FIRMWARE_COMMAND_TIMEOUT_MS = 2000
# Upper bound on waiting for it, measured from the TELEMETRY that confirmed the
# DRIVE (which the firmware received before that frame): the timeout, plus one
# TELEMETRY period (200 ms) and margin.
WATCHDOG_WAIT_S = FIRMWARE_COMMAND_TIMEOUT_MS / 1000.0 + 1.0
# millis() on the simulator is whole milliseconds and the Pi stamps arrivals
# with its own wall clock; this is the only slack allowed on the lower bound.
CLOCK_SLACK_S = 0.05

# A dead simulator surfaces on the link's next read (read timeout 50 ms).
LOSS_DETECT_TIMEOUT_S = 2.0
# After the loss is detected, how long to keep watching for a false recovery.
# The link retries the open at +1 s and again at +2 s after the loss (backoff
# 1 s, then 2 s), so this window always contains at least two refused reopens.
NO_RECOVERY_WINDOW_S = 3.0

# The firmware's fast TELEMETRY period (ESP32 config.h TELEMETRY_INTERVAL_MS):
# the last frame before the UART goes silent is at most this old.
TELEMETRY_PERIOD_S = 0.2
# How long to keep watching a silent link after it went STALE.
SILENT_WATCH_S = 2.0


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


class SilenceableSimulatorPort(HostSimulatorPort):
    """TEST ONLY: the production HostSimulatorPort with a switch that cuts RX.

    While `silent` is set, what the simulator sends is still drained from its
    pipe -- the process keeps running and never blocks on a full pipe -- but it
    is discarded instead of returned. read() goes on succeeding and returns
    b"", which is what a serial port on a wire that went quiet does: no
    exception, no closed port, just no bytes. Writes are untouched. No protocol
    knowledge, nothing invented.
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.silent = False
        self.discarded_bytes = 0

    def read(self, size: int) -> bytes:
        data = super().read(size)
        if self.silent:
            self.discarded_bytes += len(data)
            return b""
        return data


class RecordingSimulatorPort(HostSimulatorPort):
    """TEST ONLY: the production HostSimulatorPort, with a record of both directions.

    Every frame the link writes is kept as written, and every line the firmware
    sends is decoded with the production decoder, so a test can count commands
    by name and read ACK results the link itself does not keep. Bytes pass
    through unchanged.
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.written: List[bytes] = []
        self.received: List[dict] = []
        self._partial = b""

    def read(self, size: int) -> bytes:
        data = super().read(size)
        self._partial += data
        *lines, self._partial = self._partial.split(b"\n")
        for line in lines:
            frame = decode_line(line + b"\n")
            if hasattr(frame, "data"):
                self.received.append(dict(frame.data))
        return data

    def write(self, data: bytes) -> int:
        self.written.append(bytes(data))
        return super().write(data)

    def commands(self, name: str) -> List[dict]:
        return [json.loads(w[: w.rindex(b"*")]) for w in list(self.written)
                if w != b"\n" and json.loads(w[: w.rindex(b"*")])["cmd"] == name]

    def acks(self, cmd: str) -> List[dict]:
        return [f for f in list(self.received) if f["type"] == "ACK" and f["cmd"] == cmd]


class TestBringUpAgainstRealFirmware(unittest.TestCase):
    def setUp(self):
        if not SIMULATOR_EXE or not os.path.isfile(SIMULATOR_EXE):
            self.skipTest("ROBOTX_ESP32_SIMULATOR_EXE is not set to an existing ESP32 host simulator")

        self.cfg = Esp32Config(motion_enabled=True)
        self.ports: List[HostSimulatorPort] = []
        # A test that removes the ESP32 clears this, so the link's own reopen
        # attempts fail like a missing device instead of starting a new simulator.
        self.simulator_available = True
        # The production port unless a test needs a test-only subclass of it.
        self.port_class = HostSimulatorPort
        self.sim_env = dict(SIM_ENV)

        def factory() -> HostSimulatorPort:
            if not self.simulator_available:
                raise OSError("test: the ESP32 host simulator was removed and is not restarted")
            port = self.port_class(SIMULATOR_EXE, env=self.sim_env, read_timeout_s=self.cfg.read_timeout_s)
            self.ports.append(port)
            return port

        self.link = Esp32Link(self.cfg, port_factory=factory)
        # Registered before start(): runs even when an assertion fails.
        self.addCleanup(self._kill_leftover_simulators)
        self.addCleanup(self.link.stop)

    def _kill_leftover_simulators(self):
        """Safety net only. link.stop() closes the port, which ends the simulator."""

        for port in self.ports:
            proc = getattr(port, "_proc", None)
            if proc is not None and proc.poll() is None:
                proc.kill()
                proc.wait(timeout=5)

    def _wait_for(self, predicate, timeout_s, what):
        """Poll the link's own status until `predicate(status)` holds; bounded."""

        deadline = time.monotonic() + timeout_s
        status = self.link.status()
        while time.monotonic() < deadline:
            status = self.link.status()
            if predicate(status):
                return status
            time.sleep(0.05)
        tel = status.controller.telemetry
        self.fail(f"{what}: not reached within {timeout_s} s: {status.link.value} ({status.detail}); "
                  f"telemetry={None if tel is None else tel.to_dict()}; "
                  f"counters={status.controller.counters.to_dict()}")

    def _wait_until_up_and_motion_ready(self):
        return self._wait_for(lambda s: s.link is S.UP and s.motion_ready, BRING_UP_TIMEOUT_S,
                              "link UP and motion-ready")

    def _stop_and_assert_clean(self):
        """The link stops, the port closes, the simulator exits."""

        self.link.stop()
        self.assertIs(self.link.status().link, S.DISCONNECTED)
        for port in self.ports:
            self.assertIsNotNone(port._proc.poll(), "the simulator process is still running after stop()")

    def test_link_comes_up_bidirectional_and_motion_ready(self):
        started = time.monotonic()
        self.link.start()
        status = self._wait_until_up_and_motion_ready()
        elapsed = time.monotonic() - started

        controller = status.controller
        counters = controller.counters

        # Exactly one simulator: the link opened once and never lost it.
        self.assertEqual(len(self.ports), 1)
        self.assertEqual(counters.open_failures, 0)
        self.assertEqual(counters.disconnects, 0)

        # 1. UP
        self.assertIs(status.link, S.UP)

        # 2. READY: seen, from this firmware, and a first READY is not a reboot.
        self.assertTrue(controller.ready_seen)
        self.assertEqual(controller.reboot_count, 0)
        self.assertFalse(controller.reboot_latched)
        self.assertGreaterEqual(counters.events, 1)

        # 3. Protocol version, as the firmware reports it.
        self.assertEqual(controller.proto, PROTO_VERSION)
        self.assertEqual(controller.proto, 2)

        # TELEMETRY: decoded into the controller state, and the drive the
        # simulator was configured with is what the firmware reports.
        self.assertGreaterEqual(counters.telemetry, 1)
        self.assertIsNotNone(controller.telemetry)
        self.assertTrue(controller.telemetry.motor_drive_available)

        # 4. PING -> ACK: the Pi -> ESP32 direction is proven by a matched ACK.
        self.assertTrue(controller.bidirectional)
        self.assertGreaterEqual(counters.commands_sent, 1)
        self.assertGreaterEqual(counters.acks, 1)
        self.assertIsNotNone(controller.ping_rtt_ms)
        self.assertEqual(counters.ack_timeouts, 0)
        self.assertEqual(counters.unmatched_responses, 0)
        self.assertEqual(counters.esp32_errors, 0)

        # 5. CRC: every frame the firmware sent validated.
        self.assertEqual(counters.bad_crc, 0)

        # The ROM text was really on the line and was discarded as non-frames,
        # without breaking READY / TELEMETRY / PING / ACK above.
        self.assertGreaterEqual(counters.partial_on_open + counters.rejected_lines, 1)

        # 6. Motion readiness.
        self.assertTrue(status.motion_ready)
        self.assertTrue(controller.motion_ready)

        self.assertLess(elapsed, BRING_UP_TIMEOUT_S)

        self._stop_and_assert_clean()

    def _bring_up_and_drive_forward(self):
        """Bring the link up, then carry one forward DRIVE until TELEMETRY shows it applied.

        Returns (ready, after, drive_seq, units, submitted_at): the status just
        before the decision was submitted, the first status whose TELEMETRY
        shows the DRIVE applied, the DRIVE's seq, its DRIVE units, and the wall
        time of the submit.
        """

        self.link.start()
        up = self._wait_until_up_and_motion_ready()

        # The same verified state as the bring-up test.
        self.assertIs(up.link, S.UP)
        self.assertTrue(up.motion_ready)
        self.assertTrue(up.controller.ready_seen)
        self.assertGreaterEqual(up.controller.counters.telemetry, 1)
        self.assertTrue(up.controller.bidirectional)

        # Precondition, not a relaxation: the firmware refuses forward motion
        # until both front sensors have produced valid samples (it would answer
        # GATED FRONT_SENSOR_FAULT). With the simulator's default clear range
        # that is a few hundred ms after boot; wait for the firmware to say so.
        # Also wait for a TELEMETRY frame sent after the firmware recorded the
        # PING (last_seq no longer null): the link goes UP on the PING ACK, which
        # can overtake the next TELEMETRY, and the DRIVE's seq is correlated
        # against the last seq the firmware reports.
        ready = self._wait_for(
            lambda s: (s.motion_ready
                       and s.controller.telemetry is not None
                       and s.controller.telemetry.last_seq is not None
                       and s.controller.telemetry.front_valid
                       and not s.controller.telemetry.forward_blocked
                       and not s.controller.telemetry.safety_stop),
            DRIVE_TIMEOUT_S, "front sensing valid, forward not blocked, PING recorded by the firmware")

        last_acked = ready.controller.telemetry.last_seq   # the PING's seq
        self.assertIsNotNone(last_acked)
        drive_seq = 1 if last_acked >= SEQ_MAX else last_acked + 1   # PROTOCOL.md section 8
        units = to_drive_units(FORWARD_SPEED)

        submitted_at = time.time()
        self.link.submit(fx.decision(MotionIntent.forward(FORWARD_SPEED)))

        def drive_acked_and_applied(s):
            tel = s.controller.telemetry
            return (s.controller.counters.acks == ready.controller.counters.acks + 1
                    and tel is not None
                    and tel.received_at >= submitted_at
                    and tel.last_seq == drive_seq
                    and tel.left_applied > 0
                    and tel.right_applied > 0)

        after = self._wait_for(drive_acked_and_applied, DRIVE_TIMEOUT_S,
                               "DRIVE ACK and TELEMETRY with applied forward output")
        return ready, after, drive_seq, units, submitted_at

    def test_drive_is_accepted_and_telemetry_confirms_applied_output(self):
        ready, after, drive_seq, units, submitted_at = self._bring_up_and_drive_forward()
        before = ready.controller.counters
        counters = after.controller.counters
        tel = after.controller.telemetry

        # 1. Exactly one command went out after the decision: the DRIVE. No
        #    PING follows a bidirectional link, and nothing else submitted.
        self.assertEqual(counters.commands_sent, before.commands_sent + 1)
        self.assertEqual(counters.stale_commands_dropped, before.stale_commands_dropped)

        # 2. The DRIVE was acknowledged and the ESP32 accepted it. The link
        #    matched one more ACK to its only pending command (no timeout, no
        #    unmatched response, no ERROR frame). The firmware's own TELEMETRY
        #    names the DRIVE's seq as the last one it acknowledged, records no
        #    rejection, and holds the commanded value -- a GATED DRIVE would
        #    have been discarded (left_cmd 0) and latched safety_stop.
        self.assertEqual(counters.acks, before.acks + 1)
        self.assertEqual(counters.unmatched_responses, 0)
        self.assertEqual(counters.esp32_errors, 0)
        self.assertEqual(tel.last_seq, drive_seq)
        self.assertEqual(tel.last_reject, "NONE")
        self.assertEqual(tel.block_reason, "NONE")
        self.assertFalse(tel.safety_stop)
        self.assertEqual((tel.left_cmd, tel.right_cmd), (units, units))

        # 3. No ACK timed out, at any point in the test.
        self.assertEqual(counters.ack_timeouts, 0)
        self.assertFalse(tel.command_timeout)

        # 4. A TELEMETRY frame received after the decision shows the output
        #    the firmware applied to both sides.
        self.assertGreaterEqual(tel.received_at, submitted_at)
        self.assertGreater(tel.left_applied, 0)
        self.assertGreater(tel.right_applied, 0)

        self.assertIs(after.link, S.UP)
        self.assertEqual(counters.bad_crc, 0)

        # 5. Clean shutdown (the link's final STOP goes out as the port closes).
        self._stop_and_assert_clean()

    def test_drive_watchdog_stops_output_when_pi_goes_silent(self):
        # ── Phases 1-2: bring up, then one forward DRIVE, confirmed active ──
        ready, active, drive_seq, units, submitted_at = self._bring_up_and_drive_forward()
        before = ready.controller.counters
        on = active.controller.counters
        on_tel = active.controller.telemetry

        self.assertEqual(on.commands_sent, before.commands_sent + 1)   # the DRIVE
        self.assertEqual(on.acks, before.acks + 1)                     # its ACK
        self.assertEqual(on_tel.last_seq, drive_seq)
        self.assertEqual((on_tel.left_cmd, on_tel.right_cmd), (units, units))
        self.assertGreater(on_tel.left_applied, 0)
        self.assertGreater(on_tel.right_applied, 0)
        self.assertFalse(on_tel.command_timeout)
        self.assertFalse([e for _, e in self.link.events() if e.get("event") == "COMMAND_TIMEOUT"])

        # ── Phase 3: the Pi goes silent ──
        # Nothing below submits a decision or touches the simulator. Once UP and
        # bidirectional, Esp32Link writes nothing unprompted: a submitted motion
        # slot goes out once and is never repeated, and PING is only sent while
        # the link is not yet bidirectional (link.py _transmit). The firmware's
        # watchdog has to notice the silence on its own.
        silent_from = on_tel.received_at

        # Every TELEMETRY seen during the silence, to prove the output stayed on
        # until the watchdog -- not that something else stopped it earlier.
        seen = {}

        def watchdog_fired(s):
            tel = s.controller.telemetry
            if tel is not None and tel.received_at > silent_from:
                seen[tel.received_at] = tel
            return (tel is not None
                    and tel.received_at > silent_from
                    and tel.command_timeout
                    and tel.left_applied == 0
                    and tel.right_applied == 0
                    and any(e.get("event") == "COMMAND_TIMEOUT" for _, e in self.link.events()))

        # ── Phase 4: the watchdog removes the output ──
        off = self._wait_for(watchdog_fired, WATCHDOG_WAIT_S,
                             "COMMAND_TIMEOUT event and TELEMETRY with command_timeout and zero output")
        counters = off.controller.counters
        tel = off.controller.telemetry

        # The Pi really was silent: no command written, no ACK, nothing dropped.
        self.assertEqual(counters.commands_sent, on.commands_sent)
        self.assertEqual(counters.acks, on.acks)
        self.assertEqual(counters.stale_commands_dropped, on.stale_commands_dropped)

        # The transition, active -> safe, made by the watchdog and only by it.
        self.assertTrue(tel.command_timeout)
        self.assertEqual(tel.left_applied, 0)
        self.assertEqual(tel.right_applied, 0)
        self.assertEqual((tel.left_cmd, tel.right_cmd), (0, 0))   # the request is cleared, not just masked
        self.assertEqual(tel.state, "COMMAND_TIMEOUT")
        self.assertEqual(tel.block_reason, "NONE")                # not the obstacle gate
        self.assertFalse(tel.safety_stop)
        self.assertEqual(tel.last_seq, drive_seq)                 # no command arrived since the DRIVE
        early_stops = [t.to_dict() for _, t in sorted(seen.items())
                       if not t.command_timeout and (t.left_applied == 0 or t.right_applied == 0)]
        self.assertEqual(early_stops, [], "output dropped before the watchdog fired")
        still_on = [t for t in seen.values()
                    if not t.command_timeout and t.left_applied > 0 and t.right_applied > 0]
        self.assertGreaterEqual(len(still_on), 1, "no TELEMETRY showed the output still on during the silence")

        # ── Phase 5: the COMMAND_TIMEOUT event, as the link exposes it ──
        timeouts = [(at, e) for at, e in self.link.events() if e.get("event") == "COMMAND_TIMEOUT"]
        self.assertEqual(len(timeouts), 1, "the firmware announces the failsafe once, on the transition")
        event_at, event = timeouts[0]
        self.assertEqual(off.controller.last_event, "COMMAND_TIMEOUT")
        self.assertEqual(counters.events, on.events + 1)
        self.assertIsNone(event["seq"])
        self.assertEqual(event["timeout_ms"], FIRMWARE_COMMAND_TIMEOUT_MS)
        self.assertGreater(event["command_age_ms"], event["timeout_ms"])
        # Not early: the firmware received the DRIVE after it was submitted.
        self.assertGreaterEqual(event_at - submitted_at,
                                FIRMWARE_COMMAND_TIMEOUT_MS / 1000.0 - CLOCK_SLACK_S)

        # ── Phase 6: a watchdog stop is not a link failure ──
        self.assertIs(off.link, S.UP)
        self.assertTrue(off.controller.bidirectional)
        self.assertEqual(counters.bad_crc, 0)
        self.assertEqual(counters.esp32_errors, 0)
        self.assertEqual(counters.ack_timeouts, 0)
        self.assertEqual(counters.unmatched_responses, 0)
        self.assertEqual(counters.disconnects, 0)
        self.assertEqual(len(self.ports), 1)

        # ── Phase 7 ──
        self._stop_and_assert_clean()

    def test_active_drive_detects_esp32_loss(self):
        # ── Phases 1-2: bring up, then one forward DRIVE, confirmed active ──
        ready, active, drive_seq, units, submitted_at = self._bring_up_and_drive_forward()
        before = ready.controller.counters
        on = active.controller.counters
        on_tel = active.controller.telemetry

        self.assertEqual(on.commands_sent, before.commands_sent + 1)   # the DRIVE
        self.assertEqual(on.acks, before.acks + 1)                     # its ACK
        self.assertEqual(on_tel.last_seq, drive_seq)
        self.assertEqual((on_tel.left_cmd, on_tel.right_cmd), (units, units))
        self.assertGreater(on_tel.left_applied, 0)
        self.assertGreater(on_tel.right_applied, 0)
        self.assertIs(active.link, S.UP)
        self.assertTrue(active.motion_ready)

        # ── Phase 3: the ESP32 disappears while the Pi believes it is driving ──
        # No STOP, no further decision. The simulator process is killed outright
        # (HostSimulatorPort has no kill API; its process handle is the only
        # way to take the "ESP32" away). Reopening is refused from here on.
        self.assertEqual(len(self.ports), 1)
        sim = self.ports[0]
        self.simulator_available = False
        sim._proc.kill()
        sim._proc.wait(timeout=5)

        # ── Phase 4: the link detects the loss ──
        # Esp32Link._lost(): read raises -> port closed -> DISCONNECTED, with the
        # connection's bidirectional proof and any pending motion discarded.
        lost = self._wait_for(
            lambda s: s.link is S.DISCONNECTED and s.controller.counters.disconnects == on.disconnects + 1,
            LOSS_DETECT_TIMEOUT_S, "DISCONNECTED after the simulator was killed")
        self.assertIn("read failed", lost.detail)
        self.assertIn("simulator exited", lost.detail)
        self.assertFalse(lost.motion_ready)
        self.assertFalse(lost.controller.motion_ready)
        self.assertFalse(lost.controller.bidirectional)
        gone = lost.controller.counters

        # ── Phase 5: the last TELEMETRY is history, not live data ──
        stale = self._wait_for(
            lambda s: (s.controller.telemetry_age_s is not None
                       and s.controller.telemetry_age_s > self.cfg.stale_after_s),
            self.cfg.stale_after_s + LOSS_DETECT_TIMEOUT_S, "the last TELEMETRY aged past the freshness limit")
        self.assertIs(stale.link, S.DISCONNECTED)
        self.assertFalse(stale.motion_ready)
        # The last frame is retained -- it still says the output was applied --
        # but only with its age; nothing newer has arrived.
        self.assertIs(stale.controller.telemetry, lost.controller.telemetry)
        self.assertEqual(stale.controller.telemetry.last_seq, drive_seq)
        self.assertEqual(stale.last_rx_at, lost.last_rx_at)

        # ── Phase 6: no false recovery while the ESP32 stays gone ──
        seen_states = set()
        deadline = time.monotonic() + NO_RECOVERY_WINDOW_S
        while time.monotonic() < deadline:
            s = self.link.status()
            seen_states.add(s.link)
            self.assertFalse(s.motion_ready, f"motion-ready with the ESP32 gone: {s.link.value} ({s.detail})")
            self.assertFalse(s.controller.bidirectional)
            time.sleep(0.05)
        end = self.link.status()
        counters = end.controller.counters

        self.assertEqual(seen_states, {S.DISCONNECTED})
        self.assertIn("cannot open", end.detail)
        self.assertGreaterEqual(counters.open_failures, gone.open_failures + 2)   # it kept trying
        self.assertEqual(counters.disconnects, gone.disconnects)
        # Nothing arrived from the dead simulator, and no new one was started.
        self.assertEqual(len(self.ports), 1)
        for field in ("frames_ok", "telemetry", "events", "acks", "diag", "gps"):
            self.assertEqual(getattr(counters, field), getattr(gone, field), field)
        self.assertEqual(end.controller.reboot_count, 0)
        self.assertEqual(end.last_rx_at, lost.last_rx_at)
        self.assertGreater(end.controller.telemetry_age_s, self.cfg.stale_after_s)
        # The loss is a transport failure, not a protocol one.
        self.assertEqual(counters.bad_crc, 0)
        self.assertEqual(counters.esp32_errors, 0)

        # ── Phase 7 ──
        self._stop_and_assert_clean()
        alive = [t.name for t in threading.enumerate() if t.name in ("esp32-link", "esp32-sim-reader")]
        self.assertEqual(alive, [], "link or simulator-reader thread still running after stop()")

    def test_active_drive_enters_stale_when_uart_goes_silent(self):
        # The test-only port: identical to HostSimulatorPort until `silent` is set.
        self.port_class = SilenceableSimulatorPort

        # ── Phases 1-2: bring up, then one forward DRIVE, confirmed active ──
        ready, active, drive_seq, units, submitted_at = self._bring_up_and_drive_forward()
        before = ready.controller.counters
        on = active.controller.counters
        on_tel = active.controller.telemetry

        self.assertEqual(on.commands_sent, before.commands_sent + 1)   # the DRIVE
        self.assertEqual(on.acks, before.acks + 1)                     # its ACK
        self.assertEqual(on_tel.last_seq, drive_seq)
        self.assertEqual((on_tel.left_cmd, on_tel.right_cmd), (units, units))
        self.assertGreater(on_tel.left_applied, 0)
        self.assertGreater(on_tel.right_applied, 0)
        self.assertIs(active.link, S.UP)
        self.assertTrue(active.motion_ready)

        self.assertEqual(len(self.ports), 1)
        port = self.ports[0]
        self.assertIsInstance(port, SilenceableSimulatorPort)
        stale_after = self.cfg.stale_after_s          # production Esp32Config default
        self.assertEqual(stale_after, 1.0)

        # ── Phase 3: the UART goes silent; the simulator stays alive ──
        # No STOP, no further decision, no exception, no close. A silent link
        # never takes Esp32Link's reopen path (only a read/write exception or a
        # failed open does); should it anyway, the reopen is refused and counted
        # rather than starting a second simulator.
        self.simulator_available = False
        port.silent = True
        silenced_at = time.monotonic()

        # ── Phase 4: fresh until the threshold, then STALE -- never DISCONNECTED ──
        first = self.link.status()
        self.assertIs(first.link, S.UP)
        self.assertTrue(first.motion_ready)
        self.assertLess(first.controller.telemetry_age_s, stale_after)

        seen_states = set()

        def went_stale(s):
            seen_states.add(s.link)
            return s.link is S.STALE

        stale = self._wait_for(went_stale, stale_after + LOSS_DETECT_TIMEOUT_S, "STALE after the UART went silent")
        elapsed = time.monotonic() - silenced_at

        self.assertEqual(seen_states - {S.UP, S.STALE}, set(), f"states while silent: {seen_states}")
        self.assertIn("no TELEMETRY", stale.detail)
        self.assertFalse(stale.motion_ready)
        self.assertFalse(stale.controller.motion_ready)
        self.assertGreater(stale.controller.telemetry_age_s, stale_after)
        # The last frame before the silence was at most one TELEMETRY period old.
        self.assertGreaterEqual(elapsed, stale_after - TELEMETRY_PERIOD_S - CLOCK_SLACK_S)
        self.assertIsNone(port._proc.poll(), "the simulator must still be running")
        self.assertEqual(stale.controller.counters.disconnects, 0)
        self.assertEqual(stale.controller.counters.open_failures, 0)

        cached = stale.controller.telemetry
        frozen_rx = stale.last_rx_at
        frozen = stale.controller.counters
        discarded_at_stale = port.discarded_bytes

        # ── Phases 5-6: the cached frame is history; no false freshness, no recovery ──
        ages = []
        deadline = time.monotonic() + SILENT_WATCH_S
        while time.monotonic() < deadline:
            s = self.link.status()
            self.assertIs(s.link, S.STALE, f"left STALE while silent: {s.link.value} ({s.detail})")
            self.assertFalse(s.motion_ready)
            self.assertIs(s.controller.telemetry, cached)       # the same last-known frame
            self.assertEqual(s.last_rx_at, frozen_rx)
            ages.append(s.controller.telemetry_age_s)
            self.assertIsNone(port._proc.poll(), "the simulator must stay alive while the UART is silent")
            time.sleep(0.05)
        end = self.link.status()
        counters = end.controller.counters

        # The cached frame is still populated -- and still says the output was
        # applied -- but its arrival time is frozen and its age only grows; the
        # link does not treat it as live.
        self.assertIs(end.controller.telemetry, cached)
        self.assertEqual(cached.last_seq, drive_seq)
        self.assertGreaterEqual(cached.received_at, on_tel.received_at)
        self.assertGreater(cached.left_applied, 0)
        self.assertGreater(cached.right_applied, 0)
        self.assertEqual(ages, sorted(ages), "telemetry age went backwards")
        self.assertGreaterEqual(ages[-1] - ages[0], SILENT_WATCH_S - TELEMETRY_PERIOD_S)
        self.assertGreater(end.controller.telemetry_age_s, stale_after + SILENT_WATCH_S - TELEMETRY_PERIOD_S)
        self.assertIs(end.link, S.STALE)
        self.assertFalse(end.motion_ready)

        # Silence on the link, not a dead ESP32: the simulator kept sending and
        # every byte was dropped before the link could see it.
        self.assertGreater(port.discarded_bytes, discarded_at_stale)
        self.assertGreater(discarded_at_stale, 0)

        # Nothing reached the link while silent.
        for field in ("frames_ok", "telemetry", "events", "acks", "diag", "gps",
                      "rejected_lines", "partial_on_open", "bad_crc"):
            self.assertEqual(getattr(counters, field), getattr(frozen, field), field)
        self.assertEqual(end.controller.reboot_count, 0)
        # The transport itself never failed and was never reopened, and the Pi
        # sent nothing either.
        self.assertEqual(counters.disconnects, 0)
        self.assertEqual(counters.open_failures, 0)
        self.assertEqual(len(self.ports), 1)
        self.assertEqual(counters.commands_sent, on.commands_sent)
        self.assertEqual(counters.ack_timeouts, 0)
        self.assertEqual(counters.esp32_errors, 0)

        # ── Phase 7: the still-running simulator is ended by the link's own close ──
        self._stop_and_assert_clean()
        alive = [t.name for t in threading.enumerate() if t.name in ("esp32-link", "esp32-sim-reader")]
        self.assertEqual(alive, [], "link or simulator-reader thread still running after stop()")

    def test_operator_reset_clears_the_latch_without_reboot_or_resume(self):
        """Obstacle -> GATED -> safety_stop -> PAUSED -> RESET -> clear, still PAUSED -> RESUME -> gated again."""

        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        front_file = os.path.join(tmp.name, "front_cm.txt")

        def set_front(cm):
            """The simulator re-reads this every 20 ms: a test input for the range."""
            with open(front_file, "w") as f:
                f.write(f"{cm}\n")

        set_front(150)
        self.sim_env["SIM_FRONT_CM_FILE"] = front_file
        self.port_class = RecordingSimulatorPort

        # The production agent, driving the production link over the real
        # firmware. The agent is ticked here at its own 10 Hz while the link's
        # I/O thread runs, exactly the split the running Pi has.
        agent = RobotAgent(agent_settings(ROBOTX_DEADRECKON_ENABLED="1"))
        agent.esp32 = self.link
        agent.perception = _ClearScenePerception()

        def tick_until(predicate, timeout_s, what):
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                agent.tick()
                s = self.link.status()
                if predicate(s):
                    return s
                time.sleep(0.1)
            s = self.link.status()
            tel = s.controller.telemetry
            self.fail(f"{what}: not reached in {timeout_s} s: mode={agent.state.mode.value} "
                      f"link={s.link.value} reset={s.controller.reset_status} "
                      f"telemetry={None if tel is None else tel.to_dict()}")

        def telemetry_after(at):
            return lambda s: s.controller.telemetry is not None and s.controller.telemetry.received_at > at

        # ── 1-3: up, then a mission really driving ──
        self.link.start()
        self._wait_until_up_and_motion_ready()
        port = self.ports[0]
        agent.start_mission([(0.0, 0.0005)])
        tick_until(lambda s: (s.controller.telemetry.left_applied > 0
                              and s.controller.telemetry.right_applied > 0),
                   DRIVE_TIMEOUT_S, "mission driving with output applied")
        self.assertIs(agent.state.mode, OperatingMode.AUTO)

        # ── 4-9: obstacle -> firmware gates DRIVE -> safety_stop -> Pi pauses ──
        set_front(20)
        tick_until(lambda s: agent.state.mode is OperatingMode.PAUSED, DRIVE_TIMEOUT_S,
                   "mission paused by the ESP32 safety stop")
        gated = [a for a in port.acks("DRIVE") if a["result"] == "GATED"]
        self.assertTrue(gated, "the firmware never answered a DRIVE with GATED")
        self.assertEqual(gated[0]["reason"], "FRONT_OBSTACLE")
        latched = self.link.status()
        self.assertTrue(latched.controller.telemetry.safety_stop)
        self.assertFalse(latched.motion_ready)
        reboots = latched.controller.reboot_count
        drives_before_reset = len(port.commands("DRIVE"))
        drive_acks_before_reset = len(port.acks("DRIVE"))

        # ── 10-13: one RESET, ACCEPTED; fresh TELEMETRY shows the latch clear ──
        requested_at = time.time()
        agent.reset_controller("integration test")
        cleared = tick_until(
            lambda s: (s.controller.reset_status == "ACCEPTED"
                       and telemetry_after(requested_at)(s)
                       and not s.controller.telemetry.safety_stop),
            DRIVE_TIMEOUT_S, "RESET accepted and safety_stop clear in fresh TELEMETRY")
        tel = cleared.controller.telemetry
        self.assertEqual(tel.left_applied, 0)
        self.assertEqual(tel.right_applied, 0)
        self.assertTrue(tel.front_obstacle)                  # the obstacle is still there
        resets = port.commands("RESET")
        self.assertEqual(len(resets), 1)
        self.assertEqual(resets[0], {"type": "COMMAND", "seq": resets[0]["seq"], "cmd": "RESET"})
        self.assertEqual([(a["seq"], a["result"]) for a in port.acks("RESET")],
                         [(resets[0]["seq"], "ACCEPTED")])
        self.assertTrue(cleared.motion_ready)                # nothing the Pi treats as a fault

        # ── 14-16: not a reboot, and nothing resumed ──
        for _ in range(5):                                   # half a second of normal ticking
            agent.tick()
            time.sleep(0.1)
        after = self.link.status()
        self.assertEqual(after.controller.reboot_count, reboots)
        self.assertFalse(after.controller.reboot_latched)
        self.assertFalse([e for at, e in self.link.events() if e.get("event") == "READY" and at >= requested_at])
        self.assertFalse(agent.emergency_stopped)
        self.assertIs(agent.state.mode, OperatingMode.PAUSED)
        self.assertEqual(len(port.commands("DRIVE")), drives_before_reset)
        self.assertEqual(len(port.commands("RESET")), 1)

        # ── 17-18: explicit RESUME; the obstacle is still there, so the firmware
        #    gates the DRIVE again, re-latches, and the Pi pauses again ──
        resumed_at = time.time()
        agent.resume_mission("integration test")
        tick_until(lambda s: (agent.state.mode is OperatingMode.PAUSED
                              and telemetry_after(resumed_at)(s)
                              and s.controller.telemetry.safety_stop),
                   DRIVE_TIMEOUT_S, "DRIVE gated again and the mission re-paused")
        self.assertGreater(len(port.commands("DRIVE")), drives_before_reset)
        since_reset = port.acks("DRIVE")[drive_acks_before_reset:]
        self.assertTrue(since_reset, "no DRIVE was answered after the RESUME")
        self.assertEqual({a["result"] for a in since_reset}, {"GATED"})
        self.assertEqual({(a["applied_left"], a["applied_right"]) for a in since_reset}, {(0, 0)})
        tel = self.link.status().controller.telemetry
        self.assertEqual((tel.left_applied, tel.right_applied), (0, 0))
        self.assertEqual(len(port.commands("RESET")), 1)     # never re-sent on its own

        self._stop_and_assert_clean()


if __name__ == "__main__":
    unittest.main()
