"""GPS hardware acquisition: serial port in, parsed NMEA fix out.

This module is the hardware boundary only. It does not estimate position,
smooth anything, or know what a route is. `robotx.localization.position` turns
these fixes into the robot's position estimate, and `robotx.navigation`
consumes that.

What it reports is exactly what the connected receiver sends over NMEA-0183:
- `GGA`: fix quality, satellites in use, altitude
- `RMC`: fix validity, speed over ground, track angle (course)

No receiver-specific capability is assumed. If the attached module does not emit
a field, the field stays `None` rather than being invented. The port and baud
rate come from configuration (`/dev/ttyAMA0` @ 9600 by default on this Pi).
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional

from robotx.config.logging_setup import log_event


logger = logging.getLogger(__name__)

KNOTS_TO_MPS = 0.514444


class GPSStatus(str, Enum):
    """Explicit acquisition states.

    A position may only be used when the status is `FIX`. Every other state
    means "no trustworthy position", and callers must not treat them as a
    fallback to the last known location without checking the age themselves.
    """

    FIX = "FIX"                      # valid fix received recently
    NO_FIX = "NO_FIX"                # receiver is talking but reports no valid fix
    STALE = "STALE"                  # last valid fix is older than the limit
    DISCONNECTED = "DISCONNECTED"    # serial port could not be opened
    UNAVAILABLE = "UNAVAILABLE"      # pyserial/pynmea2 missing, or GPS disabled
    STARTING = "STARTING"            # thread started, nothing parsed yet


@dataclass(frozen=True)
class GpsFix:
    """One position fix as reported by the receiver."""

    latitude: float
    longitude: float
    timestamp: float                       # unix time when this Pi parsed it
    altitude_m: Optional[float] = None     # GGA, metres above mean sea level
    satellites: Optional[int] = None       # GGA, satellites in use
    fix_quality: Optional[int] = None      # GGA, 0=invalid 1=GPS 2=DGPS ...
    speed_mps: Optional[float] = None      # RMC speed over ground
    track_deg: Optional[float] = None      # RMC course over ground (true)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "latitude": round(self.latitude, 7),
            "longitude": round(self.longitude, 7),
            "altitude_m": self.altitude_m,
            "satellites": self.satellites,
            "fix_quality": self.fix_quality,
            "speed_mps": None if self.speed_mps is None else round(self.speed_mps, 3),
            "track_deg": None if self.track_deg is None else round(self.track_deg, 1),
            "timestamp": self.timestamp,
        }


@dataclass(frozen=True)
class GpsReading:
    """Status-explicit GPS reading handed to the rest of the agent."""

    status: GPSStatus
    fix: Optional[GpsFix] = None
    age_s: Optional[float] = None
    error: Optional[str] = None
    sentences_seen: int = 0

    @property
    def has_fix(self) -> bool:
        return self.status is GPSStatus.FIX and self.fix is not None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "status": self.status.value,
            "fix": None if self.fix is None else self.fix.to_dict(),
            "age_s": None if self.age_s is None else round(self.age_s, 2),
            "error": self.error,
            "sentences_seen": self.sentences_seen,
        }


@dataclass(frozen=True)
class GPSConfig:
    port: str = "/dev/ttyAMA0"
    baudrate: int = 9600
    timeout_s: float = 1.0
    stale_after_s: float = 5.0
    reconnect_interval_s: float = 5.0

    @classmethod
    def from_settings(cls, settings: Any) -> "GPSConfig":
        return cls(
            port=settings.gps_port,
            baudrate=settings.gps_baudrate,
            timeout_s=settings.gps_timeout_s,
            stale_after_s=settings.gps_stale_after_s,
            reconnect_interval_s=settings.gps_reconnect_interval_s,
        )


def parse_nmea_sentence(sentence: str, previous: Optional[GpsFix] = None) -> Optional[GpsFix]:
    """Parse one NMEA sentence into a GpsFix, or None if it carries no valid fix.

    Fields the sentence does not carry are taken from `previous` when it is the
    same fix continuing (GGA and RMC arrive in separate sentences), so a caller
    that only sees RMC still reports the satellite count from the last GGA.
    Invalid fixes (`RMC` status != 'A', `GGA` quality 0) return None.
    """

    try:
        import pynmea2  # type: ignore
    except ImportError:  # pragma: no cover - dependency checked at startup
        return None

    try:
        msg = pynmea2.parse(sentence, check=False)
    except Exception:
        return None

    sentence_type = getattr(msg, "sentence_type", "")

    lat = getattr(msg, "latitude", None)
    lon = getattr(msg, "longitude", None)
    if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
        return None
    # pynmea2 yields 0.0/0.0 for empty fields; a literal null island fix is not
    # something this robot will ever legitimately see.
    if lat == 0.0 and lon == 0.0:
        return None

    altitude = getattr(previous, "altitude_m", None) if previous else None
    satellites = getattr(previous, "satellites", None) if previous else None
    quality = getattr(previous, "fix_quality", None) if previous else None
    speed = getattr(previous, "speed_mps", None) if previous else None
    track = getattr(previous, "track_deg", None) if previous else None

    if sentence_type == "GGA":
        try:
            quality = int(msg.gps_qual) if msg.gps_qual not in (None, "") else None
        except (TypeError, ValueError):
            quality = None
        if quality == 0:
            return None  # receiver explicitly reports no valid fix
        try:
            satellites = int(msg.num_sats) if msg.num_sats not in (None, "") else None
        except (TypeError, ValueError):
            satellites = None
        try:
            altitude = float(msg.altitude) if msg.altitude not in (None, "") else None
        except (TypeError, ValueError):
            altitude = None

    elif sentence_type == "RMC":
        status = str(getattr(msg, "status", "") or "")
        if status.upper() != "A":
            return None  # 'V' = navigation receiver warning, data not valid
        knots = getattr(msg, "spd_over_grnd", None)
        try:
            speed = float(knots) * KNOTS_TO_MPS if knots not in (None, "") else None
        except (TypeError, ValueError):
            speed = None
        course = getattr(msg, "true_course", None)
        try:
            track = float(course) if course not in (None, "") else None
        except (TypeError, ValueError):
            track = None

    elif sentence_type not in {"GLL", "GNS"}:
        # Other sentences (GSV, VTG, ...) carry no position.
        return None

    return GpsFix(
        latitude=float(lat),
        longitude=float(lon),
        timestamp=time.time(),
        altitude_m=altitude,
        satellites=satellites,
        fix_quality=quality,
        speed_mps=speed,
        track_deg=track,
    )


class GPSReader:
    """Background serial NMEA reader with explicit status and reconnect."""

    def __init__(self, cfg: GPSConfig) -> None:
        self.cfg = cfg
        self._lock = threading.Lock()

        self._fix: Optional[GpsFix] = None
        self._status = GPSStatus.STARTING
        self._error: Optional[str] = None
        self._sentences = 0
        self._last_sentence: Optional[str] = None

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._serial = None
        self._connected = False

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="gps", daemon=True)
        self._thread.start()
        log_event(logger, "gps.starting", port=self.cfg.port, baud=self.cfg.baudrate)

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False

        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(timeout=2.0)
            if thread.is_alive():
                logger.warning("GPS thread did not exit within 2s")

        self._close_serial()
        log_event(logger, "gps.stopped")

    def get_reading(self) -> GpsReading:
        """Status-explicit reading. This is the accessor the agent uses."""

        with self._lock:
            fix = self._fix
            status = self._status
            error = self._error
            sentences = self._sentences

        if fix is None:
            return GpsReading(status=status, error=error, sentences_seen=sentences)

        age = time.time() - fix.timestamp
        if status is GPSStatus.FIX and age > self.cfg.stale_after_s:
            status = GPSStatus.STALE

        return GpsReading(
            status=status, fix=fix, age_s=age, error=error, sentences_seen=sentences
        )

    def status(self) -> Dict[str, Optional[str]]:
        """Diagnostic detail for the manual bench script."""

        with self._lock:
            return {
                "status": self._status.value,
                "error": self._error,
                "last_sentence": self._last_sentence,
            }

    # --- internals -----------------------------------------------------------

    def _loop(self) -> None:
        try:
            import serial  # type: ignore  # noqa: F401
            import pynmea2  # type: ignore  # noqa: F401
        except ImportError as e:
            self._set_status(GPSStatus.UNAVAILABLE, error=f"missing GPS dependency: {e}")
            log_event(
                logger,
                "gps.unavailable",
                "pyserial/pynmea2 not installed",
                level=logging.ERROR,
                error=str(e),
            )
            return

        while self._running:
            if self._serial is None and not self._open_serial():
                # Retry rather than giving up: the receiver may be plugged in
                # after the agent starts, or the port may come back.
                self._sleep_interruptible(self.cfg.reconnect_interval_s)
                continue

            try:
                self._read_once()
            except Exception as e:
                log_event(
                    logger,
                    "gps.read_failed",
                    "serial read failed; will reconnect",
                    level=logging.WARNING,
                    error=repr(e),
                )
                self._close_serial()
                self._set_status(GPSStatus.DISCONNECTED, error=repr(e))
                self._sleep_interruptible(self.cfg.reconnect_interval_s)

    def _open_serial(self) -> bool:
        import serial  # type: ignore

        try:
            self._serial = serial.Serial(
                self.cfg.port, self.cfg.baudrate, timeout=self.cfg.timeout_s
            )
        except Exception as e:
            if self._connected or self._status is not GPSStatus.DISCONNECTED:
                log_event(
                    logger,
                    "gps.disconnected",
                    "serial port could not be opened",
                    level=logging.ERROR,
                    port=self.cfg.port,
                    error=repr(e),
                )
            self._connected = False
            self._set_status(GPSStatus.DISCONNECTED, error=f"serial open failed: {e}")
            return False

        self._connected = True
        self._set_status(GPSStatus.NO_FIX, error=None)
        log_event(logger, "gps.connected", port=self.cfg.port, baud=self.cfg.baudrate)
        return True

    def _close_serial(self) -> None:
        serial_port = self._serial
        self._serial = None
        self._connected = False
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                logger.debug("Ignoring error while closing GPS serial port", exc_info=True)

    def _read_once(self) -> None:
        raw = self._serial.readline()
        if not raw:
            return  # read timeout: receiver is quiet, not an error

        line = raw.decode("ascii", errors="ignore").strip()
        if not line.startswith("$"):
            return

        with self._lock:
            self._sentences += 1
            self._last_sentence = line
            previous = self._fix

        fix = parse_nmea_sentence(line, previous=previous)
        if fix is None:
            # Sentence carried no valid position. Only downgrade to NO_FIX if we
            # never had one; an existing fix ages out via get_reading().
            with self._lock:
                if self._fix is None:
                    self._status = GPSStatus.NO_FIX
            return

        had_fix = previous is not None
        with self._lock:
            self._fix = fix
            self._status = GPSStatus.FIX
            self._error = None

        if not had_fix:
            log_event(
                logger,
                "gps.fix_acquired",
                satellites=fix.satellites,
                quality=fix.fix_quality,
            )

    def _set_status(self, status: GPSStatus, *, error: Optional[str]) -> None:
        with self._lock:
            self._status = status
            self._error = error

    def _sleep_interruptible(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, seconds)
        while self._running and time.monotonic() < deadline:
            time.sleep(0.1)
