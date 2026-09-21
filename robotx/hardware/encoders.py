import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional


try:
    import RPi.GPIO as GPIO  # type: ignore
except Exception:  # pragma: no cover
    GPIO = None  # type: ignore


def _now() -> float:
    return time.monotonic()


@dataclass(frozen=True)
class EncoderConfig:
    left_pin: int
    right_pin: int
    pulses_per_rev: int
    wheel_diameter_m: float


class EncoderReader:
    """Quadrature-less pulse encoder reader (one channel per wheel).

    Uses GPIO interrupts to count pulses and a background sampler to compute
    speed. If GPIO isn't available, returns 0 speeds.
    """

    def __init__(self, cfg: EncoderConfig, sample_hz: float = 10.0) -> None:
        self.cfg = cfg
        self.sample_hz = max(1.0, float(sample_hz))

        self._lock = threading.Lock()
        self._left_count = 0
        self._right_count = 0
        self._left_rpm = 0.0
        self._right_rpm = 0.0
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self._last_sample_t = _now()
        self._last_left = 0
        self._last_right = 0

    def start(self) -> None:
        if self._running:
            return
        self._running = True

        if GPIO is not None:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.cfg.left_pin, GPIO.IN)
            GPIO.setup(self.cfg.right_pin, GPIO.IN)

            GPIO.add_event_detect(self.cfg.left_pin, GPIO.RISING, callback=self._on_left)
            GPIO.add_event_detect(self.cfg.right_pin, GPIO.RISING, callback=self._on_right)

        self._thread = threading.Thread(target=self._sampler_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

        if GPIO is not None:
            try:
                GPIO.remove_event_detect(self.cfg.left_pin)
            except Exception:
                pass
            try:
                GPIO.remove_event_detect(self.cfg.right_pin)
            except Exception:
                pass

    def _on_left(self, channel: int) -> None:  # pragma: no cover
        with self._lock:
            self._left_count += 1

    def _on_right(self, channel: int) -> None:  # pragma: no cover
        with self._lock:
            self._right_count += 1

    def _sampler_loop(self) -> None:
        period = 1.0 / self.sample_hz
        while self._running:
            t = _now()
            dt = t - self._last_sample_t
            if dt <= 0:
                time.sleep(period)
                continue

            with self._lock:
                left = self._left_count
                right = self._right_count

            d_left = left - self._last_left
            d_right = right - self._last_right

            # pulses per second -> revolutions per minute
            left_rps = (d_left / max(1, self.cfg.pulses_per_rev)) / dt
            right_rps = (d_right / max(1, self.cfg.pulses_per_rev)) / dt
            left_rpm = left_rps * 60.0
            right_rpm = right_rps * 60.0

            with self._lock:
                self._left_rpm = left_rpm
                self._right_rpm = right_rpm

            self._last_sample_t = t
            self._last_left = left
            self._last_right = right

            time.sleep(period)

    def get_speed(self) -> Dict[str, float]:
        """Returns wheel RPM and approximate linear speed (m/s)."""
        with self._lock:
            left_rpm = self._left_rpm
            right_rpm = self._right_rpm

        wheel_circumference = math.pi * max(1e-6, self.cfg.wheel_diameter_m)
        left_mps = (left_rpm / 60.0) * wheel_circumference
        right_mps = (right_rpm / 60.0) * wheel_circumference

        return {
            "left_rpm": float(left_rpm),
            "right_rpm": float(right_rpm),
            "left_mps": float(left_mps),
            "right_mps": float(right_mps),
            "avg_mps": float((left_mps + right_mps) / 2.0),
        }
