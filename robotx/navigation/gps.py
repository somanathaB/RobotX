import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple


@dataclass(frozen=True)
class GPSConfig:
    port: str
    baudrate: int = 9600


class GPSReader:
    """NMEA GPS reader over serial.

    Runs a background thread and keeps the latest (lat, lon) fix.
    """

    def __init__(self, cfg: GPSConfig) -> None:
        self.cfg = cfg
        self._lock = threading.Lock()

        self._location: Optional[Tuple[float, float]] = None
        self._last_fix_t: Optional[float] = None
        self._last_sentence: Optional[str] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._error: Optional[str] = None

        self._ser = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _loop(self) -> None:
        try:
            import serial  # type: ignore
            import pynmea2  # type: ignore
        except Exception as e:
            self._error = f"Missing GPS dependencies: {e}"
            return

        try:
            self._ser = serial.Serial(self.cfg.port, self.cfg.baudrate, timeout=1.0)
        except Exception as e:
            self._error = f"GPS serial open failed: {e}"
            return

        while self._running:
            try:
                line = self._ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                self._last_sentence = line

                msg = pynmea2.parse(line)

                lat = getattr(msg, "latitude", None)
                lon = getattr(msg, "longitude", None)
                if lat is None or lon is None:
                    continue
                if not (isinstance(lat, (int, float)) and isinstance(lon, (int, float))):
                    continue

                with self._lock:
                    self._location = (float(lat), float(lon))
                    self._last_fix_t = time.monotonic()
                    self._error = None
            except Exception:
                # Keep last fix
                time.sleep(0.02)

    def get_location(self) -> Dict[str, Optional[float]]:
        with self._lock:
            loc = self._location
            last_t = self._last_fix_t
        now = time.monotonic()

        if loc is None or last_t is None:
            return {"lat": None, "lon": None, "fix_age_s": None}

        return {"lat": float(loc[0]), "lon": float(loc[1]), "fix_age_s": float(now - last_t)}

    def status(self) -> Dict[str, Optional[str]]:
        with self._lock:
            sentence = self._last_sentence
        return {"error": self._error, "last_sentence": sentence}
