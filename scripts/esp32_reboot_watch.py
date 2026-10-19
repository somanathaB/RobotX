"""TEMPORARY read-only ESP32 reboot evidence collector for the live RobotX Pi.

    python3 -m scripts.esp32_reboot_watch                           # foreground; Ctrl-C stops
    python3 -m scripts.esp32_reboot_watch --duration 60 --echo-samples
    python3 -m scripts.esp32_reboot_watch --report                  # events already logged

Why: the ESP32 has rebooted repeatedly and firmware f10bb8f reports no reset
cause. The live agent owns /dev/ttyAMA0, so this watches what the agent already
publishes -- its uptime, link state, counters and DIAG sections -- and records
the moments around a reboot: how long the ESP32 was silent, whether its uptime
restarted or kept counting through the silence, whether READY arrived, which
error counters moved, and the sensor/I2C state just before it went quiet.

What it does: GET /state every --interval s and GET /telemetry every
--telemetry-every s from the agent's loopback API. Every sample is kept in
memory for --pre-window s. A sample is written every --sample-every s; when an
anomaly is detected the pre-window is written and every sample is written for
--post-window s. Logs are JSONL, size-rotated (--max-bytes x (--backups + 1)
bytes at most), files 0600 in a 0700 directory.

What it never does: send any method but GET, call any path but /state and
/telemetry, contact anything but loopback, follow a redirect, open a serial
port or any device, read /etc/robotx, touch the service, clear a latch, or
command the ESP32. Only whitelisted fields are logged -- never a raw response,
never GPS coordinates -- and every logged string is redacted for anything
token-like. A field the agent does not publish is logged as "UNAVAILABLE".

Stdlib only: runs under the system python3 or the dev venv.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
import re
import signal
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Deque, Dict, Iterator, List, Optional, Tuple

UNAVAILABLE = "UNAVAILABLE"
DEFAULT_BASE = "http://127.0.0.1:8000"
DEFAULT_LOG_DIR = Path.home() / "robotx-esp32-watch"
LOG_NAME = "esp32_watch.jsonl"
ALLOWED_PATHS = frozenset({"/state", "/telemetry"})
LOOPBACK_HOSTS = frozenset({"127.0.0.1", "localhost", "::1"})
MAX_TEXT = 300

# Counters that grow on every healthy frame: reported in samples, never events.
STREAMING_COUNTERS = frozenset({"frames_ok", "telemetry", "diag", "gps", "events"})
# DIAG SYSTEM fields whose change is evidence (I2C bus, drive, firmware identity, stalls).
SYSTEM_WATCHED = ("i2c_ready", "tca_status", "pca_status", "motor_drive_status",
                  "stall_count", "last_stall_source", "fw_build", "proto")
REAR_WATCHED = ("available", "sensor_fault", "health", "bus_err_total", "reinit_count")
# REAR_CHANGED flaps with targets near the 2 m range limit: logged, rate limited,
# and it does not open a full-rate window.
NO_WINDOW_EVENTS = frozenset({"REAR_CHANGED", "API_RESTORED"})
RATE_LIMITED_S = {"REAR_CHANGED": 30.0}
# Uptime advance vs. wall-clock silence: agreement within this is "kept counting".
CONTINUITY_TOLERANCE_S = 0.5


# --- redaction ------------------------------------------------------------------

_SECRET_KEY = re.compile(r"token|secret|passw|api[_-]?key|authori[sz]ation|bearer|signature|cookie|"
                         r"credential|hmac", re.I)
_SECRET_ASSIGN = re.compile(r"(?i)\b(token|secret|password|passwd|api[_-]?key|authorization|signature|"
                            r"cookie|hmac)(\s*[:=]\s*)[^\s,;'\"]+")
_BEARER = re.compile(r"(?i)\bbearer\s+\S+")
_OPAQUE = re.compile(r"[A-Za-z0-9+/=_\-]{32,}")


def _opaque(match: "re.Match[str]") -> str:
    word = match.group(0)
    # A long run with both letters and digits is a key, hash or token; a long
    # all-caps status name is not.
    if re.search(r"[0-9]", word) and re.search(r"[A-Za-z]", word):
        return "[REDACTED]"
    return word


def redact_text(text: str) -> str:
    text = _SECRET_ASSIGN.sub(lambda m: f"{m.group(1)}{m.group(2)}[REDACTED]", text)
    text = _BEARER.sub("Bearer [REDACTED]", text)
    text = _OPAQUE.sub(_opaque, text)
    return text if len(text) <= MAX_TEXT else text[:MAX_TEXT] + "...[truncated]"


def scrub(value: Any) -> Any:
    """Redact a record before it is written. Secret-looking keys lose their value."""
    if isinstance(value, str):
        return redact_text(value)
    if isinstance(value, dict):
        return {str(k): ("[REDACTED]" if _SECRET_KEY.search(str(k)) else scrub(v)) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [scrub(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


# --- loopback GET client ----------------------------------------------------------

class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):  # type: ignore[no-untyped-def]
        return None  # a 3xx surfaces as an HTTPError; nothing is followed


class LoopbackGetClient:
    """GET-only, loopback-only, allowlisted-path client. It has no other verb."""

    def __init__(self, base: str = DEFAULT_BASE, timeout_s: float = 2.0) -> None:
        parts = urllib.parse.urlsplit(base)
        if (parts.scheme != "http" or parts.hostname not in LOOPBACK_HOSTS or parts.path not in ("", "/")
                or parts.query or parts.fragment or parts.username or parts.password):
            raise ValueError(f"refusing base URL {base!r}: only http://<loopback>[:port] is allowed")
        self.base = base.rstrip("/")
        self.timeout_s = timeout_s
        # No proxies (a loopback request must never leave the host), no redirects.
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())

    def get(self, path: str) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
        if path not in ALLOWED_PATHS:
            raise ValueError(f"refusing path {path!r}: only {sorted(ALLOWED_PATHS)} are read")
        request = urllib.request.Request(self.base + path, method="GET", headers={"Accept": "application/json"})
        try:
            with self._opener.open(request, timeout=self.timeout_s) as response:
                body = json.loads(response.read())
        except urllib.error.HTTPError as e:
            return None, f"HTTP {e.code}"
        except (urllib.error.URLError, OSError, ValueError) as e:
            return None, redact_text(f"{type(e).__name__}: {getattr(e, 'reason', e)}")
        if not isinstance(body, dict):
            return None, "response is not a JSON object"
        return body, None


# --- extraction (whitelist only) -----------------------------------------------------

def pick(obj: Any, *path: str) -> Any:
    """The value at `path`, or UNAVAILABLE when any step is missing. A present null stays None."""
    for key in path:
        if not isinstance(obj, dict) or key not in obj:
            return UNAVAILABLE
        obj = obj[key]
    return obj


def numeric_only(obj: Any) -> Any:
    if not isinstance(obj, dict):
        return UNAVAILABLE
    return {k: v for k, v in obj.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}


def _fields(obj: Any, mapping: Dict[str, str]) -> Dict[str, Any]:
    return {name: pick(obj, key) for name, key in mapping.items()}


def extract_state(state: Dict[str, Any]) -> Dict[str, Any]:
    c = pick(state, "controller")
    tel = pick(c, "telemetry")
    diag = pick(state, "controller_diag")
    system, front_d, rear_d = pick(diag, "SYSTEM"), pick(diag, "FRONT"), pick(diag, "REAR")
    return {
        "agent_started_at": pick(state, "started_at"),
        "mode": pick(state, "mode"),
        "safety_rule": pick(state, "safety", "rule"),
        "link": pick(state, "communication", "esp32"),
        "link_detail": pick(state, "communication", "esp32_detail"),
        "link_since": pick(state, "communication", "esp32_since"),
        "last_rx_at": pick(state, "communication", "esp32_last_rx_at"),
        "esp32_health": pick(state, "health", "components", "esp32", "status"),
        "esp32_uptime_ms": pick(tel, "uptime_ms"),
        "telemetry_received_at": pick(tel, "received_at"),
        "telemetry_age_s": pick(c, "telemetry_age_s"),
        "fw_state": pick(tel, "state"),
        "block_reason": pick(tel, "block_reason"),
        "last_reject": pick(tel, "last_reject"),
        **_fields(c, {k: k for k in ("reboot_count", "reboot_latched", "ready_seen", "last_event", "last_error",
                                     "bidirectional", "transmit_enabled", "motion_enabled", "motion_ready",
                                     "ping_rtt_ms", "reset_status", "proto", "firmware")}),
        "counters": numeric_only(pick(c, "counters")),
        "front": {
            "valid": pick(tel, "front", "valid"),
            "obstacle": pick(tel, "front", "obstacle"),
            "left_cm": pick(tel, "front", "left_cm"),
            "right_cm": pick(tel, "front", "right_cm"),
            "forward_blocked": pick(tel, "gates", "forward_blocked"),
            **_fields(front_d, {
                "left_valid": "front_left_valid", "right_valid": "front_right_valid",
                "left_health": "front_left_health", "right_health": "front_right_health",
                "left_raw_cm": "front_left_raw_cm", "right_raw_cm": "front_right_raw_cm",
                "left_timeout_streak": "front_left_timeout_streak",
                "right_timeout_streak": "front_right_timeout_streak",
                "left_samples": "front_left_samples", "right_samples": "front_right_samples",
                "sensor_map_verified": "sensor_map_verified", "diag_received_at": "received_at"}),
        },
        "rear": {
            "available": pick(tel, "rear", "available"),
            "obstacle": pick(tel, "rear", "obstacle"),
            "sensor_fault": pick(tel, "rear", "sensor_fault"),
            "mm": pick(tel, "rear", "mm"),
            **_fields(rear_d, {
                "positions": "rear_positions", "valid": "rear_sensor_valid", "status": "rear_sensor_status",
                "health": "rear_sensor_health", "fail_streak": "rear_fail_streak",
                "bus_err_total": "rear_bus_err_total", "bus_err_site": "rear_bus_err_site",
                "reinit_count": "rear_reinit_count", "orientation_verified": "rear_orientation_verified",
                "diag_received_at": "received_at"}),
        },
        "system": {
            **_fields(system, {k: k for k in ("uptime_ms", "fw_build", "proto", "i2c_ready", "tca_status",
                                              "pca_status", "motor_drive_status", "command_ever_received",
                                              "loop_max_ms", "loop_max_source", "last_stall_ms",
                                              "last_stall_source", "last_stall_uptime_ms", "stall_count")}),
            "fw_link": numeric_only(pick(system, "link")),
            "diag_received_at": pick(system, "received_at"),
        },
    }


def extract_telemetry(telemetry: Dict[str, Any]) -> Dict[str, Any]:
    """Statuses only. Never a coordinate."""
    components = pick(telemetry, "health", "components")
    return {
        "agent_uptime_s": pick(telemetry, "uptime_s"),
        "health": pick(telemetry, "health", "status"),
        "components": ({k: pick(v, "status") for k, v in components.items()}
                       if isinstance(components, dict) else UNAVAILABLE),
        "gps_status": pick(telemetry, "gps", "status"),
        "gps_age_s": pick(telemetry, "gps", "age_s"),
        "gps_fix_type": pick(telemetry, "gps", "fix", "fix_type"),
    }


def iso(t: float) -> str:
    return datetime.fromtimestamp(t).astimezone().isoformat(timespec="milliseconds")


def make_sample(t: float, state: Optional[Dict[str, Any]], error: Optional[str],
                telemetry: Any, telemetry_at: Optional[float]) -> Dict[str, Any]:
    return {
        "kind": "sample", "t": round(t, 3), "iso": iso(t),
        "api_error": error,
        "state": extract_state(state) if state is not None else UNAVAILABLE,
        "telemetry": telemetry,
        "telemetry_polled_at": round(telemetry_at, 3) if telemetry_at else UNAVAILABLE,
    }


# --- detection --------------------------------------------------------------------

def _num(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


class Detector:
    """Compares consecutive samples. Pure: no I/O, no clock of its own."""

    def __init__(self, gap_s: float = 1.0) -> None:
        self.gap_s = gap_s
        self.prev: Optional[Dict[str, Any]] = None
        self.api_down = False
        self.gap_open: Optional[Dict[str, Any]] = None
        self._last_emit: Dict[str, float] = {}
        self._suppressed: Dict[str, int] = {}

    def _event(self, out: List[Dict[str, Any]], t: float, name: str, **detail: Any) -> None:
        limit = RATE_LIMITED_S.get(name)
        if limit is not None:
            last = self._last_emit.get(name)
            if last is not None and t - last < limit:
                self._suppressed[name] = self._suppressed.get(name, 0) + 1
                return
            if self._suppressed.get(name):
                detail["suppressed_since_last"] = self._suppressed.pop(name)
        self._last_emit[name] = t
        out.append({"kind": "event", "event": name, "t": round(t, 3), "iso": iso(t), **detail})

    def observe(self, sample: Dict[str, Any]) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        t = sample["t"]
        if sample.get("api_error") or not isinstance(sample.get("state"), dict):
            if not self.api_down:
                self._event(out, t, "API_UNREACHABLE", error=sample.get("api_error"))
                self.api_down = True
            return out  # keep `prev`: compare across the outage
        if self.api_down:
            self._event(out, t, "API_RESTORED")
            self.api_down = False

        s = sample["state"]
        if self.prev is None:
            self.prev = s
            return out
        p = self.prev
        self.prev = s

        if p["agent_started_at"] != s["agent_started_at"]:
            self._event(out, t, "AGENT_RESTARTED", before=p["agent_started_at"], after=s["agent_started_at"],
                        note="counters and reboot_count restart with the agent")

        self._uptime_and_gaps(out, t, p, s)

        if p["reboot_count"] != s["reboot_count"]:
            self._event(out, t, "REBOOT_COUNT_CHANGED", before=p["reboot_count"], after=s["reboot_count"],
                        link_detail=s["link_detail"])
        if p["reboot_latched"] != s["reboot_latched"]:
            self._event(out, t, "REBOOT_LATCH_CHANGED", before=p["reboot_latched"], after=s["reboot_latched"])
        if p["link"] != s["link"]:
            self._event(out, t, "LINK_STATUS_CHANGED", before=p["link"], after=s["link"], detail=s["link_detail"])
        if p["last_error"] != s["last_error"]:
            self._event(out, t, "LAST_ERROR_CHANGED", before=p["last_error"], after=s["last_error"])

        self._counters(out, t, p, s)

        changed = {k: [p["system"][k], s["system"][k]] for k in SYSTEM_WATCHED if p["system"][k] != s["system"][k]}
        if changed:
            self._event(out, t, "DIAG_SYSTEM_CHANGED", changed=changed)
        changed = {k: [p["rear"][k], s["rear"][k]] for k in REAR_WATCHED if p["rear"][k] != s["rear"][k]}
        if changed:
            self._event(out, t, "REAR_CHANGED", changed=changed)
        return out

    def _uptime_and_gaps(self, out: List[Dict[str, Any]], t: float, p: Dict[str, Any], s: Dict[str, Any]) -> None:
        up_p, up_s = p["esp32_uptime_ms"], s["esp32_uptime_ms"]
        rx_p, rx_s = p["telemetry_received_at"], s["telemetry_received_at"]
        if _num(up_p) and _num(up_s) and up_s < up_p:
            boot_at = rx_s - up_s / 1000.0 if _num(rx_s) else None
            self._event(out, t, "UPTIME_BACKWARDS", before_ms=up_p, after_ms=up_s,
                        est_boot_at=round(boot_at, 3) if boot_at else UNAVAILABLE,
                        est_boot_iso=iso(boot_at) if boot_at else UNAVAILABLE)

        age = s["telemetry_age_s"]
        if self.gap_open is None and _num(age) and age > self.gap_s:
            self.gap_open = {"last_received_at": rx_s, "last_uptime_ms": s["esp32_uptime_ms"]}
            self._event(out, t, "TELEMETRY_GAP_START", telemetry_age_s=age, last_uptime_ms=s["esp32_uptime_ms"],
                        last_received_at=rx_s, link=s["link"],
                        front=s["front"], rear=s["rear"], system=s["system"])
            return

        if not (_num(rx_p) and _num(rx_s) and rx_s > rx_p):
            return
        before_rx = self.gap_open["last_received_at"] if self.gap_open else rx_p
        before_up = self.gap_open["last_uptime_ms"] if self.gap_open else up_p
        if not _num(before_rx):
            before_rx = rx_p
        silence = rx_s - before_rx
        if self.gap_open is None and silence <= self.gap_s:
            return
        self.gap_open = None
        self._event(out, t, "TELEMETRY_GAP_END", silence_s=round(silence, 3),
                    uptime_before_ms=before_up, uptime_after_ms=up_s, **classify_gap(silence, before_up, up_s))

    def _counters(self, out: List[Dict[str, Any]], t: float, p: Dict[str, Any], s: Dict[str, Any]) -> None:
        cp, cs = p["counters"], s["counters"]
        if not isinstance(cp, dict) or not isinstance(cs, dict):
            if cp != cs:
                self._event(out, t, "COUNTERS_AVAILABILITY_CHANGED", before=type(cp).__name__, after=type(cs).__name__)
            return
        ev_p, ev_s = cp.get("events", UNAVAILABLE), cs.get("events", UNAVAILABLE)
        if _num(ev_p) and _num(ev_s) and ev_s != ev_p:
            name = "READY_RECEIVED" if s["last_event"] == "READY" else "ESP32_EVENTS_RECEIVED"
            self._event(out, t, name, events_delta=ev_s - ev_p, last_event=s["last_event"],
                        note="only the latest event name is published; an earlier READY in this "
                             "interval would be hidden behind it")
        deltas = {k: [cp.get(k, UNAVAILABLE), cs.get(k, UNAVAILABLE)]
                  for k in sorted((set(cp) | set(cs)) - STREAMING_COUNTERS) if cp.get(k) != cs.get(k)}
        if deltas:
            self._event(out, t, "COUNTERS_CHANGED", deltas=deltas, pi_transmitted="commands_sent" in deltas,
                        last_event=s["last_event"])


def classify_gap(silence_s: float, before_ms: Any, after_ms: Any) -> Dict[str, Any]:
    """What a telemetry silence was, from the ESP32's own uptime across it."""
    if not (_num(before_ms) and _num(after_ms)):
        return {"verdict": "UNKNOWN", "why": "ESP32 uptime unavailable on one side of the gap"}
    if after_ms < before_ms:
        return {"verdict": "ESP32_RESTARTED", "silence_before_boot_s": round(silence_s - after_ms / 1000.0, 3),
                "why": "uptime restarted: a reset or power cycle during the silence"}
    advance_s = (after_ms - before_ms) / 1000.0
    if abs(advance_s - silence_s) <= max(CONTINUITY_TOLERANCE_S, 0.1 * silence_s):
        return {"verdict": "ESP32_KEPT_COUNTING", "uptime_advance_s": round(advance_s, 3),
                "why": "no reset: a firmware stall, a TX/UART path fault or a Pi-side read stall"}
    return {"verdict": "UPTIME_INCONSISTENT", "uptime_advance_s": round(advance_s, 3),
            "why": "uptime advance disagrees with the wall-clock silence (Pi clock step?)"}


# --- bounded log ------------------------------------------------------------------

class BoundedJsonlWriter:
    """Append-only JSONL, rotated at `max_bytes`, keeping `backups` old files. Files are 0600."""

    def __init__(self, path: Path, max_bytes: int, backups: int) -> None:
        self.path, self.max_bytes, self.backups = path, max_bytes, backups
        self._fd = self._open()

    def _open(self) -> int:
        fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
        os.fchmod(fd, 0o600)
        return fd

    def _rotate(self) -> None:
        os.close(self._fd)
        if self.backups == 0:
            self.path.unlink(missing_ok=True)
        else:
            # .N-1 -> .N (overwriting the oldest) ... .1 -> .2, then the live file -> .1
            for i in range(self.backups - 1, 0, -1):
                src = self.path.with_name(f"{self.path.name}.{i}")
                if src.exists():
                    os.replace(src, self.path.with_name(f"{self.path.name}.{i + 1}"))
            os.replace(self.path, self.path.with_name(f"{self.path.name}.1"))
        self._fd = self._open()

    def write(self, record: Dict[str, Any]) -> None:
        data = (json.dumps(scrub(record), separators=(",", ":"), default=str) + "\n").encode()
        if len(data) > self.max_bytes // 2:
            data = (json.dumps({"kind": "dropped", "t": record.get("t"), "record_kind": record.get("kind"),
                                "bytes": len(data)}) + "\n").encode()
        if os.fstat(self._fd).st_size + len(data) > self.max_bytes:
            self._rotate()
        os.write(self._fd, data)

    def close(self) -> None:
        os.close(self._fd)


def log_files(log_dir: Path) -> List[Path]:
    """Oldest first."""
    base = log_dir / LOG_NAME
    rotated = sorted(log_dir.glob(LOG_NAME + ".*"), key=lambda p: int(p.suffix[1:]) if p.suffix[1:].isdigit() else 0,
                     reverse=True)
    return [p for p in rotated if p.suffix[1:].isdigit()] + ([base] if base.exists() else [])


# --- output -----------------------------------------------------------------------

def _short(v: Any) -> str:
    return json.dumps(scrub(v), separators=(",", ":"), default=str) if isinstance(v, (dict, list)) else str(scrub(v))


def event_line(e: Dict[str, Any]) -> str:
    skip = {"kind", "event", "t", "iso", "note", "front", "rear", "system"}
    detail = " ".join(f"{k}={_short(v)}" for k, v in e.items() if k not in skip)
    return f"{e['iso']} {e['event']} {detail}".rstrip()


def sample_line(s: Dict[str, Any]) -> str:
    if not isinstance(s["state"], dict):
        return f"{s['iso']} sample api_error={s['api_error']}"
    st = s["state"]
    c, f, r, y = st["counters"], st["front"], st["rear"], st["system"]
    cnt = c if isinstance(c, dict) else {}
    return (f"{s['iso']} up_ms={st['esp32_uptime_ms']} age_s={st['telemetry_age_s']} link={st['link']} "
            f"reboots={st['reboot_count']} latched={st['reboot_latched']} last_event={st['last_event']} "
            f"last_error={st['last_error']} rejected={cnt.get('rejected_lines', UNAVAILABLE)} "
            f"too_long={cnt.get('too_long', UNAVAILABLE)} bad_crc={cnt.get('bad_crc', UNAVAILABLE)} "
            f"cmds_sent={cnt.get('commands_sent', UNAVAILABLE)} "
            f"front_valid={f['valid']} L={f['left_health']}/{f['left_cm']} R={f['right_health']}/{f['right_cm']} "
            f"rear={_short(r['health'])} i2c={y['i2c_ready']} pca={y['pca_status']} tca={y['tca_status']} "
            f"stalls={y['stall_count']} loop_max_ms={y['loop_max_ms']}")


class WindowStats:
    """Polls between two written samples, so a heartbeat still shows what it skipped."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.polls = self.api_errors = self.front_invalid = self.left_invalid = self.right_invalid = 0
        self.max_age: Any = None

    def add(self, sample: Dict[str, Any]) -> None:
        self.polls += 1
        st = sample["state"]
        if not isinstance(st, dict):
            self.api_errors += 1
            return
        f = st["front"]
        self.front_invalid += f["valid"] is False
        self.left_invalid += f["left_valid"] is False
        self.right_invalid += f["right_valid"] is False
        age = st["telemetry_age_s"]
        if _num(age):
            self.max_age = age if self.max_age is None else max(self.max_age, age)

    def flush(self) -> Dict[str, Any]:
        out = {"polls": self.polls, "api_errors": self.api_errors, "front_invalid": self.front_invalid,
               "front_left_invalid": self.left_invalid, "front_right_invalid": self.right_invalid,
               "max_telemetry_age_s": self.max_age if self.max_age is not None else UNAVAILABLE}
        self.reset()
        return out


# --- run --------------------------------------------------------------------------

class _Stop:
    requested = False


def _acquire_lock(log_dir: Path) -> int:
    fd = os.open(log_dir / "watch.lock", os.O_RDWR | os.O_CREAT, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(fd)
        raise SystemExit(f"another esp32_reboot_watch already writes to {log_dir} (see watch.pid)")
    return fd


def run(args: argparse.Namespace) -> int:
    if args.gap <= args.interval + 0.3:
        raise SystemExit("--gap must exceed --interval + 0.3 s, or ordinary polling jitter reads as a gap")
    client = LoopbackGetClient(args.base_url, args.timeout)
    log_dir = Path(args.log_dir).expanduser()
    log_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(log_dir, 0o700)
    lock_fd = _acquire_lock(log_dir)
    pid_file = log_dir / "watch.pid"
    pid_file.write_text(f"{os.getpid()}\n")
    os.chmod(pid_file, 0o600)

    def _request_stop(signum: int, _frame: Any) -> None:
        _Stop.requested = True

    signal.signal(signal.SIGTERM, _request_stop)
    signal.signal(signal.SIGINT, _request_stop)

    writer = BoundedJsonlWriter(log_dir / LOG_NAME, args.max_bytes, args.backups)
    detector = Detector(args.gap)
    ring: Deque[Dict[str, Any]] = deque(maxlen=max(1, int(args.pre_window / args.interval)))
    window = WindowStats()
    counts = {"samples": 0, "written": 0, "events": 0}
    started = time.time()
    writer.write({"kind": "start", "t": round(started, 3), "iso": iso(started), "pid": os.getpid(),
                  "base_url": client.base, "paths": sorted(ALLOWED_PATHS), "interval_s": args.interval,
                  "telemetry_every_s": args.telemetry_every, "sample_every_s": args.sample_every,
                  "gap_s": args.gap, "pre_window_s": args.pre_window, "post_window_s": args.post_window,
                  "max_bytes": args.max_bytes, "backups": args.backups,
                  "note": "read-only: GET /state and /telemetry on loopback only"})
    if not args.quiet:
        print(f"{iso(started)} watching {client.base} -> {log_dir / LOG_NAME} (pid {os.getpid()})", flush=True)

    mono0 = time.monotonic()
    post_until = next_beat = next_tel = 0.0
    tel_summary: Any = UNAVAILABLE
    tel_at: Optional[float] = None
    try:
        while not _Stop.requested and not (args.duration and time.monotonic() - mono0 >= args.duration):
            tick = time.monotonic()
            if tick >= next_tel:
                body, err = client.get("/telemetry")
                tel_summary = extract_telemetry(body) if body is not None else {"error": err}
                tel_at = time.time()
                next_tel = tick + args.telemetry_every
            state, err = client.get("/state")
            sample = make_sample(time.time(), state, err, tel_summary, tel_at)
            counts["samples"] += 1
            window.add(sample)

            events = detector.observe(sample)
            for e in events:
                writer.write(e)
                counts["events"] += 1
                if not args.quiet:
                    print(event_line(e), flush=True)
            if any(e["event"] not in NO_WINDOW_EVENTS for e in events):
                if ring:
                    writer.write({"kind": "pre_window", "t": sample["t"], "iso": sample["iso"],
                                  "samples": list(ring)})
                    ring.clear()
                post_until = tick + args.post_window

            in_window = tick < post_until
            if in_window or tick >= next_beat:
                sample["window"] = window.flush()
                sample["full_rate"] = in_window
                writer.write(sample)
                counts["written"] += 1
                next_beat = tick + args.sample_every
                if args.echo_samples and not args.quiet:
                    print(sample_line(sample), flush=True)
            if not in_window:
                ring.append(sample)

            delay = args.interval - (time.monotonic() - tick)
            if delay > 0 and not _Stop.requested:
                time.sleep(delay)
    finally:
        ended = time.time()
        writer.write({"kind": "stop", "t": round(ended, 3), "iso": iso(ended), "ran_s": round(ended - started, 1),
                      **counts, "reason": "signal" if _Stop.requested else "duration"})
        writer.close()
        pid_file.unlink(missing_ok=True)
        os.close(lock_fd)
        if not args.quiet:
            print(f"{iso(ended)} stopped: {counts}", flush=True)
    return 0


def read_records(log_dir: Path) -> Iterator[Dict[str, Any]]:
    for path in log_files(log_dir):
        with open(path, encoding="utf-8") as fh:
            for line in fh:
                try:
                    yield json.loads(line)
                except ValueError:
                    continue


def report(log_dir: Path) -> int:
    files = log_files(log_dir)
    if not files:
        print(f"no logs in {log_dir}")
        return 1
    samples = 0
    for rec in read_records(log_dir):
        kind = rec.get("kind")
        if kind == "event":
            print(event_line(rec))
        elif kind == "sample":
            samples += 1
        elif kind in ("start", "stop"):
            rest = {k: v for k, v in rec.items() if k not in ("kind", "iso", "t")}
            print(f"{rec.get('iso')} {kind.upper()} {_short(rest)}")
    print(f"-- {samples} samples in {len(files)} file(s), {sum(p.stat().st_size for p in files)} bytes")
    return 0


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base-url", default=DEFAULT_BASE, help="loopback agent API (default %(default)s)")
    ap.add_argument("--log-dir", default=str(DEFAULT_LOG_DIR), help="default %(default)s")
    ap.add_argument("--interval", type=float, default=0.5, help="GET /state period, s (default %(default)s)")
    ap.add_argument("--telemetry-every", type=float, default=5.0, help="GET /telemetry period, s (default %(default)s)")
    ap.add_argument("--sample-every", type=float, default=10.0, help="heartbeat sample period, s (default %(default)s)")
    ap.add_argument("--gap", type=float, default=1.0, help="telemetry silence that counts as a gap, s (default %(default)s)")
    ap.add_argument("--pre-window", type=float, default=30.0, help="samples kept before an event, s (default %(default)s)")
    ap.add_argument("--post-window", type=float, default=60.0, help="full-rate logging after an event, s (default %(default)s)")
    ap.add_argument("--max-bytes", type=int, default=5_000_000, help="per log file (default %(default)s)")
    ap.add_argument("--backups", type=int, default=4, help="rotated files kept (default %(default)s)")
    ap.add_argument("--timeout", type=float, default=2.0, help="HTTP timeout, s (default %(default)s)")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after this many s; 0 = until signalled")
    ap.add_argument("--echo-samples", action="store_true", help="also print each written sample")
    ap.add_argument("--quiet", action="store_true", help="print nothing (for nohup)")
    ap.add_argument("--report", action="store_true", help="print the logged events and exit")
    args = ap.parse_args(argv)
    if args.interval <= 0 or args.max_bytes < 100_000 or args.backups < 0:
        ap.error("--interval must be > 0, --max-bytes >= 100000, --backups >= 0")
    return args


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    if args.report:
        return report(Path(args.log_dir).expanduser())
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
