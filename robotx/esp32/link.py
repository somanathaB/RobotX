"""The one component that owns the ESP32 UART.

A single I/O thread opens, reads, writes and closes the port. Every other
thread -- the agent loop above all -- talks to the link only through
`submit()`, `status()` and `acknowledge_reboot()`, which touch locked state and
never the port. That is what makes shutdown safe: the thread that reads is the
thread that closes, so nothing can ever read from a closed port.

What the link will transmit, and when
-------------------------------------
- A lone LF on every (re)open: the documented resynchronisation (PROTOCOL.md
  section 12). Never anything else unprompted.
- One PING per connection, once fresh TELEMETRY has arrived, to prove the
  Pi -> ESP32 direction before the link calls itself UP. Retried while
  unanswered, but only while telemetry is fresh: a PING queued into an ESP32
  blocked in its boot I2C scan would only pile up in its RX buffer.
- STOP or DRIVE, and only when `motion_enabled`, only from a `SafetyDecision`
  (the safety gate's output, never a raw intent), only while the link is UP,
  and only if the command is still fresh when the I/O thread gets to it. The
  link holds at most one pending motion command and a newer one replaces it;
  nothing is queued, nothing is retried, and nothing survives a reconnect.

With `transmit_enabled` off the link never writes a byte.

The link does not decide how the robot moves. It converts the gated intent's
normalized left/right into DRIVE units and otherwise only ever *withholds*.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

from robotx.config.logging_setup import log_event
from robotx.control.motion import age_s
from robotx.control.safety import SafetyDecision
from robotx.esp32.protocol import (
    BAUD,
    PROTO_VERSION,
    RESYNC,
    FrameError,
    Rejected,
    SequenceCounter,
    decode_line,
    encode_command,
)
from robotx.esp32.state import (
    ControllerDiag,
    ControllerState,
    ControllerTelemetry,
    Esp32LinkStatus,
    LinkCounters,
)
from robotx.esp32.transport import LineAssembler, SerialPort, open_serial


logger = logging.getLogger(__name__)

# Symlinks whose target the firmware chooses at boot. On this Pi serial0 is the
# debug connector (ttyAMA10), so accepting it would silently talk to the wrong
# UART. The ESP32 port must be named explicitly.
_ALIAS_PORTS = frozenset({"/dev/serial0", "/dev/serial1"})

_READ_SIZE = 4096

# Upper bound on how old a motion command may be when it is written. A command
# older than this describes a world that has moved on (the same 1 s the safety
# gate allows an intent by default), and it must stay far inside the ESP32's own
# 2 s command watchdog. Configuration may tighten it, never loosen it.
MAX_COMMAND_AGE_LIMIT_S = 1.0


def to_drive_units(value: float) -> int:
    """Normalized [-1, 1] -> DRIVE's integer -255..255, rounding half away from 0.

    The ESP32 owns what that number means physically (deadband, PWM, gating);
    this is only the protocol's unit.
    """

    v = max(-1.0, min(1.0, float(value)))
    return int(math.copysign(math.floor(abs(v) * 255 + 0.5), v))


@dataclass(frozen=True)
class Esp32Config:
    port: str = "/dev/ttyAMA0"
    baudrate: int = BAUD
    # Off: the link never writes a byte (receive-only commissioning).
    transmit_enabled: bool = True
    # Off: no STOP or DRIVE is ever sent, whatever the agent submits.
    motion_enabled: bool = False
    # TELEMETRY arrives every 200 ms; five missed frames is a quiet ESP32.
    stale_after_s: float = 1.0
    # Physical PING round trip is ~17-19 ms, documented worst case ~97 ms.
    ack_timeout_s: float = 0.5
    max_consecutive_ack_timeouts: int = 3
    ping_retry_s: float = 5.0
    # A motion command older than this when the I/O thread reaches it is dropped.
    max_command_age_s: float = 0.3
    reconnect_initial_s: float = 1.0
    reconnect_max_s: float = 10.0
    read_timeout_s: float = 0.05
    max_pending: int = 8

    def __post_init__(self) -> None:
        if self.port in _ALIAS_PORTS:
            raise ValueError(
                f"{self.port} is an alias whose target is chosen at boot; "
                "name the ESP32 UART explicitly (e.g. /dev/ttyAMA0)"
            )
        if self.motion_enabled and not self.transmit_enabled:
            raise ValueError("ESP32 motion requires transmit to be enabled")
        if not 0 < self.max_command_age_s <= MAX_COMMAND_AGE_LIMIT_S:
            raise ValueError(
                f"ESP32 command max age must be in (0, {MAX_COMMAND_AGE_LIMIT_S}] s, "
                f"got {self.max_command_age_s}"
            )
        for name in ("stale_after_s", "ack_timeout_s", "read_timeout_s", "reconnect_initial_s"):
            if not getattr(self, name) > 0:
                raise ValueError(f"ESP32 {name} must be positive, got {getattr(self, name)}")
        if self.reconnect_max_s < self.reconnect_initial_s:
            raise ValueError("ESP32 reconnect_max_s must be >= reconnect_initial_s")
        if self.baudrate <= 0 or self.max_pending < 1:
            raise ValueError("ESP32 baudrate and max_pending must be positive")

    @classmethod
    def from_settings(cls, settings: Any) -> "Esp32Config":
        return cls(
            port=settings.esp32_port,
            baudrate=settings.esp32_baudrate,
            transmit_enabled=settings.esp32_transmit_enabled,
            motion_enabled=settings.esp32_motion_enabled,
            stale_after_s=settings.esp32_stale_after_s,
            ack_timeout_s=settings.esp32_ack_timeout_s,
            max_command_age_s=settings.esp32_command_max_age_s,
            reconnect_max_s=settings.esp32_reconnect_max_s,
        )


@dataclass(frozen=True)
class Esp32Status:
    """One consistent reading of the link, for the agent to copy into state."""

    link: Esp32LinkStatus
    detail: str
    since: float
    last_rx_at: Optional[float]
    controller: ControllerState
    diag: ControllerDiag

    @property
    def motion_ready(self) -> bool:
        return self.controller.motion_ready


class Esp32Link:
    """Owns the ESP32 UART; see the module docstring for what it may send."""

    def __init__(
        self,
        cfg: Esp32Config,
        *,
        port_factory: Optional[Callable[[], SerialPort]] = None,
        clock: Callable[[], float] = time.monotonic,
        wall: Callable[[], float] = time.time,
    ) -> None:
        self.cfg = cfg
        self._factory = port_factory or (
            lambda: open_serial(cfg.port, cfg.baudrate, cfg.read_timeout_s)
        )
        self._clock = clock
        self._wall = wall

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # --- I/O thread only ------------------------------------------------
        self._port: Optional[SerialPort] = None
        self._assembler = LineAssembler()
        self._next_open_at = 0.0
        self._backoff = cfg.reconnect_initial_s
        self._first_line = False
        self._resync_sent = False

        # --- shared, guarded by _lock ---------------------------------------
        self._status = Esp32LinkStatus.DISCONNECTED
        self._detail = "not started"
        self._since = wall()
        self._port_open = False
        self._stopping = False
        self._last_rx_at: Optional[float] = None

        self._telemetry: Optional[ControllerTelemetry] = None
        self._telemetry_mono: Optional[float] = None
        self._telemetry_this_connection = False
        self._diag: Dict[str, Dict[str, Any]] = {}
        self._diag_at: Dict[str, float] = {}

        self._proto: Optional[int] = None
        self._proto_problem: Optional[str] = None
        self._firmware: Optional[str] = None
        self._ready_seen = False
        self._last_uptime: Optional[int] = None
        self._reboot_count = 0
        self._reboot_latched = False
        self._reboot_reason = ""
        self._last_event: Optional[str] = None
        self._last_error: Optional[str] = None
        self._events: Deque[Tuple[float, Dict[str, Any]]] = deque(maxlen=16)

        self._seq = SequenceCounter()
        self._seq_synced = False
        self._pending: Dict[int, Tuple[str, float]] = {}
        self._consecutive_timeouts = 0
        self._ping_seq: Optional[int] = None
        self._ping_next_at = 0.0
        self._ping_rtt_ms: Optional[float] = None
        self._bidirectional = False

        self._motion_slot: Optional[Tuple[str, Dict[str, int], float]] = None
        self._motion_sent = False
        self._counters: Dict[str, int] = dict(LinkCounters().__dict__)
        self._last_logged_error: Tuple[Optional[str], float] = (None, 0.0)

    # --- lifecycle -----------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        with self._lock:
            self._stopping = False
            # CONNECTING, not DISCONNECTED: nothing has failed yet, and health
            # must not flash FAILED for the moments before the first open.
            self._set_status(Esp32LinkStatus.CONNECTING, f"opening {self.cfg.port}")
        self._thread = threading.Thread(target=self._run, name="esp32-link", daemon=True)
        self._thread.start()
        log_event(logger, "esp32.starting", port=self.cfg.port, baud=self.cfg.baudrate,
                  transmit=self.cfg.transmit_enabled, motion=self.cfg.motion_enabled)

    def stop(self, timeout_s: float = 2.0) -> None:
        """Stop the I/O thread; it closes the port itself on the way out.

        With motion enabled and a motion command sent on this connection, a
        final STOP is written before the port closes. If the thread will not
        exit, the port is deliberately *not* closed from here: closing it under
        a thread that may still be reading is exactly the bug this design
        avoids.
        """

        with self._lock:
            self._stopping = True
            self._set_status(Esp32LinkStatus.STOPPING, "stopping")
        self._stop_event.set()

        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout_s)
            if thread.is_alive():
                log_event(logger, "esp32.stop_timeout", "I/O thread did not exit; port left open",
                          level=logging.ERROR, timeout_s=timeout_s)
                return
        else:
            # Never started (tests drive poll_once directly): this thread is the owner.
            self._shutdown_port()

        with self._lock:
            self._set_status(Esp32LinkStatus.DISCONNECTED, "stopped")
        log_event(logger, "esp32.stopped")

    def _run(self) -> None:
        try:
            while not self._stop_event.is_set():
                wait = self.poll_once()
                if wait > 0:
                    self._stop_event.wait(wait)
        except Exception:
            logger.exception("ESP32 I/O thread crashed")
            with self._lock:
                self._set_status(Esp32LinkStatus.DISCONNECTED, "I/O thread crashed")
        finally:
            self._shutdown_port()

    def _shutdown_port(self) -> None:
        """Close the port. Called only by its owner, after reading has stopped."""

        port, self._port = self._port, None
        if port is None:
            return
        with self._lock:
            send_stop = (
                self.cfg.motion_enabled and self._motion_sent and self._bidirectional
                and not self._reboot_latched
            )
            frame = encode_command(self._seq.next(), "STOP") if send_stop else None
            self._port_open = False
        try:
            if frame is not None:
                port.write(frame)
                with self._lock:
                    self._counters["commands_sent"] += 1
                log_event(logger, "esp32.final_stop", "STOP sent before closing the port")
        except Exception as e:
            log_event(logger, "esp32.final_stop_failed", level=logging.WARNING, error=repr(e))
        finally:
            try:
                port.close()
            except Exception:
                logger.debug("Ignoring error closing the ESP32 port", exc_info=True)

    # --- the I/O loop ----------------------------------------------------------

    def poll_once(self) -> float:
        """One I/O iteration. Returns how long the caller may wait before the next.

        Public so tests can drive the link deterministically without a thread.
        """

        now = self._clock()
        if self._port is None:
            if now < self._next_open_at or not self._open(now):
                return max(0.0, min(self._next_open_at - now, 0.2))
        port = self._port
        if port is None:
            return 0.0

        try:
            data = port.read(_READ_SIZE)
        except Exception as e:
            self._lost(f"read failed: {e!r}", now)
            return 0.0

        if data:
            before = self._assembler.overflows
            for line in self._assembler.feed(data):
                try:
                    self._handle_line(line, now)
                except Exception:
                    logger.exception("Error handling an ESP32 line; continuing")
            if self._assembler.overflows != before:
                with self._lock:
                    self._counters["too_long"] += self._assembler.overflows - before
                    self._counters["rejected_lines"] += self._assembler.overflows - before

        with self._lock:
            self._expire_pending(now)
            self._refresh_status(now)

        if self.cfg.transmit_enabled:
            try:
                self._transmit(now)
            except Exception as e:
                self._lost(f"write failed: {e!r}", now)
        return 0.0

    def _open(self, now: float) -> bool:
        try:
            port = self._factory()
        except Exception as e:
            with self._lock:
                self._counters["open_failures"] += 1
                first = self._counters["open_failures"] == 1
                self._set_status(Esp32LinkStatus.DISCONNECTED, f"cannot open {self.cfg.port}: {e}")
            if first:
                log_event(logger, "esp32.open_failed", "will retry", level=logging.ERROR,
                          port=self.cfg.port, error=repr(e))
            self._next_open_at = now + self._backoff
            self._backoff = min(self._backoff * 2, self.cfg.reconnect_max_s)
            return False

        self._port = port
        self._assembler.reset()
        self._first_line = True
        self._resync_sent = False
        self._backoff = self.cfg.reconnect_initial_s
        with self._lock:
            self._port_open = True
            self._reset_connection()
            self._set_status(Esp32LinkStatus.CONNECTING, "port open; waiting for TELEMETRY")
        log_event(logger, "esp32.connected", port=self.cfg.port)
        return True

    def _lost(self, reason: str, now: float) -> None:
        port, self._port = self._port, None
        if port is not None:
            try:
                port.close()
            except Exception:
                logger.debug("Ignoring error closing a failed ESP32 port", exc_info=True)
        with self._lock:
            self._port_open = False
            self._counters["disconnects"] += 1
            self._reset_connection()
            self._set_status(Esp32LinkStatus.DISCONNECTED, reason)
        self._next_open_at = now + self.cfg.reconnect_initial_s
        log_event(logger, "esp32.disconnected", reason, level=logging.ERROR)

    def _reset_connection(self) -> None:
        """Forget everything that belonged to the previous connection. Lock held.

        This is the no-replay guarantee: whatever was pending or unsent is
        gone, and the new connection must prove itself with a fresh PING
        before any motion command can be carried again. What is known about
        the ESP32 itself (its uptime, a latched reboot) deliberately survives.
        """

        self._pending.clear()
        self._motion_slot = None
        self._motion_sent = False
        self._bidirectional = False
        self._ping_seq = None
        self._ping_next_at = 0.0
        self._seq_synced = False
        self._consecutive_timeouts = 0
        self._telemetry_this_connection = False

    # --- receive ---------------------------------------------------------------

    def _handle_line(self, line: bytes, now: float) -> None:
        result = decode_line(line)
        if isinstance(result, Rejected):
            if result.reason is FrameError.EMPTY:
                return
            first, self._first_line = self._first_line, False
            with self._lock:
                if first:
                    # Opening mid-stream leaves a partial or bit-misaligned first
                    # line. Expected, and not evidence of a fault.
                    self._counters["partial_on_open"] += 1
                    return
                self._counters["rejected_lines"] += 1
                if result.reason is FrameError.BAD_CRC:
                    self._counters["bad_crc"] += 1
                elif result.reason is FrameError.TOO_LONG:
                    self._counters["too_long"] += 1
            logger.debug("Rejected ESP32 line: %s %s", result.reason.value, result.detail)
            return

        self._first_line = False
        wall = self._wall()
        data = result.data
        with self._lock:
            self._counters["frames_ok"] += 1
            self._last_rx_at = wall
            if result.type == "TELEMETRY":
                self._on_telemetry(data, now, wall)
            elif result.type == "DIAG":
                self._counters["diag"] += 1
                self._diag[data["section"]] = dict(data)
                self._diag_at[data["section"]] = wall
                if data["section"] == "SYSTEM":
                    self._note_proto(data.get("proto"), "DIAG SYSTEM")
            elif result.type == "EVENT":
                self._on_event(data, wall)
            elif result.type == "ACK":
                self._on_ack(data, now)
            elif result.type == "ERROR":
                self._on_error(data, now)
            self._refresh_status(now)

    def _on_telemetry(self, data, now: float, wall: float) -> None:
        self._counters["telemetry"] += 1
        tel = ControllerTelemetry.from_frame(data, wall)
        if self._last_uptime is not None and tel.uptime_ms < self._last_uptime:
            self._reboot(f"uptime went backwards ({self._last_uptime} -> {tel.uptime_ms} ms)")
        self._last_uptime = tel.uptime_ms
        self._telemetry = tel
        self._telemetry_mono = now
        self._telemetry_this_connection = True
        if not self._seq_synced:
            # Section 8: continue from the ESP32's last acknowledged seq.
            self._seq.resync(tel.last_seq)
            self._seq_synced = True

    def _on_event(self, data, wall: float) -> None:
        self._counters["events"] += 1
        name = data["event"]
        self._last_event = name
        self._events.append((wall, dict(data)))
        if name == "READY":
            known = self._last_uptime is not None
            self._ready_seen = True
            if isinstance(data.get("fw"), str):
                self._firmware = data["fw"]
            self._note_proto(data.get("proto"), "READY")
            if known:
                self._reboot("READY received from a running ESP32")
            # The new boot's baseline. uptime_ms is optional on EVENTs; without
            # it the next TELEMETRY sets the baseline, so a reboot READY already
            # reported is not counted a second time as an uptime rollback.
            self._last_uptime = data.get("uptime_ms")
            self._seq.resync(None)
            self._seq_synced = True
            log_event(logger, "esp32.ready", firmware=self._firmware, proto=data.get("proto"))
        elif name == "COMMAND_TIMEOUT":
            log_event(logger, "esp32.command_timeout", "ESP32 watchdog zeroed the motors",
                      level=logging.WARNING, command_age_ms=data.get("command_age_ms"))
        elif name == "MOTORTEST_DONE":
            # The Pi never sends MOTORTEST; if one ran, someone else commanded it.
            log_event(logger, "esp32.unexpected_motortest", level=logging.WARNING, seq=data.get("seq"))

    def _on_ack(self, data, now: float) -> None:
        self._counters["acks"] += 1
        seq = data["seq"]
        entry = self._pending.pop(seq, None)
        if entry is None:
            self._counters["unmatched_responses"] += 1
            return
        self._consecutive_timeouts = 0
        cmd, sent_at = entry
        if data["result"] == "REJECTED" and data["reason"] == "STALE_SEQ":
            self._seq.resync(data.get("last_seq"))
        if cmd == "PING" and seq == self._ping_seq:
            self._ping_seq = None
            self._ping_rtt_ms = (now - sent_at) * 1000.0
            if data["result"] == "ACCEPTED":
                self._note_proto(data.get("proto"), "PING ACK")
                self._bidirectional = self._proto_problem is None
            else:
                self._ping_next_at = now  # e.g. STALE_SEQ after resync: try again
        elif data["result"] in ("REJECTED", "DUPLICATE"):
            log_event(logger, "esp32.command_refused", level=logging.WARNING, seq=seq, cmd=cmd,
                      result=data["result"], reason=data["reason"])

    def _on_error(self, data, now: float) -> None:
        self._counters["esp32_errors"] += 1
        reason = data["reason"]
        self._last_error = reason
        seq = data["seq"]
        if seq is not None:
            if self._pending.pop(seq, None) is None:
                self._counters["unmatched_responses"] += 1
            else:
                self._consecutive_timeouts = 0
            if seq == self._ping_seq:
                self._ping_seq = None
        last_reason, last_at = self._last_logged_error
        if reason != last_reason or now - last_at >= 5.0:
            self._last_logged_error = (reason, now)
            log_event(logger, "esp32.error_frame", "ESP32 refused input from the Pi",
                      level=logging.WARNING, reason=reason, seq=seq)

    def _note_proto(self, proto: Any, source: str) -> None:
        if proto is None:
            return
        self._proto = proto
        if proto != PROTO_VERSION:
            self._proto_problem = f"{source} reports protocol {proto!r}, expected {PROTO_VERSION}"
            self._bidirectional = False

    def _reboot(self, reason: str) -> None:
        """Latch a detected ESP32 reboot. Lock held. Cleared only by acknowledge_reboot()."""

        self._reboot_count += 1
        self._reboot_latched = True
        self._reboot_reason = reason
        self._pending.clear()
        self._motion_slot = None
        self._bidirectional = False
        self._ping_seq = None
        self._seq.resync(None)
        self._seq_synced = True
        log_event(logger, "esp32.reboot_detected", reason, level=logging.CRITICAL,
                  count=self._reboot_count)

    # --- status ------------------------------------------------------------------

    def _expire_pending(self, now: float) -> None:
        for seq, (cmd, sent_at) in list(self._pending.items()):
            if now - sent_at > self.cfg.ack_timeout_s:
                del self._pending[seq]
                self._counters["ack_timeouts"] += 1
                self._consecutive_timeouts += 1
                if seq == self._ping_seq:
                    self._ping_seq = None

    def _telemetry_fresh(self, now: float) -> bool:
        return (
            self._telemetry_this_connection
            and self._telemetry_mono is not None
            and now - self._telemetry_mono <= self.cfg.stale_after_s
        )

    def _refresh_status(self, now: float) -> None:
        """Derive the link state from the facts. Lock held.

        Loss of fresh TELEMETRY is checked first: a silent ESP32 is STALE
        whatever else is latched, so a latched reboot can never make it look
        merely degraded. The latch itself is untouched by this ordering -- it
        still blocks motion and still needs acknowledging -- and it is named
        in the STALE detail so neither fact hides the other.
        """

        if not self._port_open or self._stopping:
            return
        if self._telemetry_this_connection and not self._telemetry_fresh(now):
            age = now - (self._telemetry_mono or now)
            detail = f"no TELEMETRY for {age:.1f}s"
            if self._reboot_latched:
                detail += f"; ESP32 rebooted ({self._reboot_reason}), motion held until acknowledged"
            self._set_status(Esp32LinkStatus.STALE, detail)
        elif self._reboot_latched:
            self._set_status(Esp32LinkStatus.DEGRADED,
                             f"ESP32 rebooted ({self._reboot_reason}); motion held until acknowledged")
        elif self._proto_problem is not None:
            self._set_status(Esp32LinkStatus.DEGRADED, self._proto_problem)
        elif self._consecutive_timeouts >= self.cfg.max_consecutive_ack_timeouts:
            self._set_status(Esp32LinkStatus.DEGRADED,
                             f"{self._consecutive_timeouts} consecutive commands unanswered")
        elif not self._telemetry_this_connection:
            self._set_status(Esp32LinkStatus.CONNECTING,
                             "port open; waiting for TELEMETRY (the ESP32 boot I2C scan can take ~2 min)")
        elif self.cfg.transmit_enabled and not self._bidirectional:
            self._set_status(Esp32LinkStatus.CONNECTING, "TELEMETRY arriving; waiting for PING ACK")
        else:
            self._set_status(
                Esp32LinkStatus.UP,
                "bidirectional" if self.cfg.transmit_enabled else "receive-only (transmit disabled)",
            )

    def _set_status(self, status: Esp32LinkStatus, detail: str) -> None:
        """Lock held."""

        if status is not self._status:
            expected = (
                status in (Esp32LinkStatus.UP, Esp32LinkStatus.CONNECTING, Esp32LinkStatus.STOPPING)
                or self._status is Esp32LinkStatus.STOPPING   # a deliberate stop completing
            )
            level = logging.INFO if expected else logging.WARNING
            log_event(logger, "esp32.link", detail, level=level, status=status.value,
                      previous=self._status.value)
            self._status = status
            self._since = self._wall()
        self._detail = detail

    def _motion_ready(self, now: float, cmd: str = "DRIVE") -> bool:
        """Whether the link would carry `cmd` right now. Lock held."""

        if not (self.cfg.transmit_enabled and self.cfg.motion_enabled):
            return False
        if not self._port_open or self._stopping or self._status is not Esp32LinkStatus.UP:
            return False
        if not self._bidirectional or self._reboot_latched or not self._telemetry_fresh(now):
            return False
        if cmd == "DRIVE":
            return self._telemetry is not None and self._telemetry.motor_drive_available
        return True

    def status(self) -> Esp32Status:
        now = self._clock()
        with self._lock:
            self._refresh_status(now)
            age = None if self._telemetry_mono is None else max(0.0, now - self._telemetry_mono)
            controller = ControllerState(
                telemetry=self._telemetry,
                telemetry_age_s=age,
                proto=self._proto,
                firmware=self._firmware,
                ready_seen=self._ready_seen,
                reboot_count=self._reboot_count,
                reboot_latched=self._reboot_latched,
                last_event=self._last_event,
                last_error=self._last_error,
                ping_rtt_ms=self._ping_rtt_ms,
                bidirectional=self._bidirectional,
                transmit_enabled=self.cfg.transmit_enabled,
                motion_enabled=self.cfg.motion_enabled,
                motion_ready=self._motion_ready(now),
                counters=LinkCounters(**self._counters),
            )
            diag = ControllerDiag(
                sections={k: dict(v) for k, v in self._diag.items()},
                received_at=dict(self._diag_at),
            )
            return Esp32Status(
                link=self._status,
                detail=self._detail,
                since=self._since,
                last_rx_at=self._last_rx_at,
                controller=controller,
                diag=diag,
            )

    def events(self) -> Tuple[Tuple[float, Dict[str, Any]], ...]:
        """The most recent EVENT frames (bounded), oldest first."""

        with self._lock:
            return tuple((t, dict(e)) for t, e in self._events)

    def acknowledge_reboot(self) -> bool:
        """Operator acknowledgement of a detected ESP32 reboot.

        Lifts the latch only; the link must still re-prove itself with a fresh
        PING before it will carry motion again.
        """

        with self._lock:
            if not self._reboot_latched:
                return False
            self._reboot_latched = False
            self._reboot_reason = ""
            self._bidirectional = False
            self._ping_seq = None
            self._ping_next_at = 0.0
        log_event(logger, "esp32.reboot_acknowledged", level=logging.WARNING)
        return True

    # --- transmit ------------------------------------------------------------------

    def submit(self, decision: SafetyDecision) -> None:
        """Offer the safety gate's decision for transmission. Never blocks.

        Only a `SafetyDecision` is accepted -- there is no way to hand the link
        a raw intent that has not been through the gate. Does nothing unless
        motion is enabled. A vetoed, stopping or stale decision becomes STOP; a
        DRIVE the link cannot currently carry becomes STOP as well. The link
        can only ever turn motion into less motion.
        """

        if not isinstance(decision, SafetyDecision):
            raise TypeError(f"submit() takes a SafetyDecision, not {type(decision).__name__}")
        if not (self.cfg.transmit_enabled and self.cfg.motion_enabled):
            return

        intent = decision.intent
        now = self._clock()
        if decision.blocked or intent.is_stop or age_s(intent, self._wall()) > self.cfg.max_command_age_s:
            cmd, fields = "STOP", {}
        else:
            cmd, fields = "DRIVE", {"left": to_drive_units(intent.left),
                                    "right": to_drive_units(intent.right)}
        with self._lock:
            if cmd == "DRIVE" and not self._motion_ready(now, "DRIVE"):
                cmd, fields = "STOP", {}
            self._motion_slot = (cmd, fields, now)

    def _transmit(self, now: float) -> None:
        """Write whatever is due. I/O thread only; the port is written outside the lock."""

        port = self._port
        if port is None:
            return
        frames: List[Tuple[Optional[str], bytes]] = []
        if not self._resync_sent:
            frames.append((None, RESYNC))
            self._resync_sent = True

        with self._lock:
            if (
                self._seq_synced
                and self._telemetry_fresh(now)
                and not self._bidirectional
                and self._ping_seq is None
                and not self._reboot_latched
                and self._proto_problem is None
                and now >= self._ping_next_at
                and len(self._pending) < self.cfg.max_pending
            ):
                seq = self._seq.next()
                frames.append(("PING", encode_command(seq, "PING")))
                self._pending[seq] = ("PING", now)
                self._ping_seq = seq
                self._ping_next_at = now + self.cfg.ping_retry_s

            slot, self._motion_slot = self._motion_slot, None
            if slot is not None:
                cmd, fields, created = slot
                if now - created > self.cfg.max_command_age_s:
                    self._counters["stale_commands_dropped"] += 1
                elif self._motion_ready(now, cmd) and len(self._pending) < self.cfg.max_pending:
                    seq = self._seq.next()
                    frames.append((cmd, encode_command(seq, cmd, **fields)))
                    self._pending[seq] = (cmd, now)
                    self._motion_sent = True

        for label, data in frames:
            port.write(data)
            if label is not None:
                with self._lock:
                    self._counters["commands_sent"] += 1
