import enum
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple


try:
    import RPi.GPIO as GPIO  # type: ignore
except Exception:  # pragma: no cover
    GPIO = None  # type: ignore


@dataclass(frozen=True)
class UltrasonicConfig:
    trigger_pin: int
    echo_pin: int


class UltrasonicStatus(str, enum.Enum):
    """Explicit sensor health states.

    A reading is only trustworthy as "no obstacle" when status is VALID.
    Every other status must be treated as a safety-relevant failure by
    callers rather than silently read as "clear" -- collapsing
    TIMEOUT/DISCONNECTED/etc. into "no obstacle" was the exact bug this
    status model replaces.

    This property must survive the move to the ESP32: whatever the UART
    contract turns out to be, "the sensor did not answer" and "the path is
    clear" have to stay distinguishable on the wire.
    """

    VALID = "VALID"                # fresh, in-range echo measured this cycle
    TIMEOUT = "TIMEOUT"            # trigger sent, echo pulse never arrived/ended in time
    OUT_OF_RANGE = "OUT_OF_RANGE"  # measured distance <=0cm or >500cm (implausible echo)
    ERROR = "ERROR"                # unexpected exception while measuring (e.g. GPIO fault)
    DISCONNECTED = "DISCONNECTED"  # RPi.GPIO not available in this process
    STALE = "STALE"                # poll loop hasn't produced any result recently (thread stalled/dead)
    UNKNOWN = "UNKNOWN"            # no measurement has been taken yet since start()


@dataclass(frozen=True)
class UltrasonicReading:
    status: UltrasonicStatus
    distance_cm: Optional[float]
    age_s: Optional[float]


class UltrasonicSensor:
    """HC-SR04 ultrasonic distance sensor.

    Provides `get_reading()` (status + cm + age) and a backward-compatible
    `get_distance()` (cm, or None for any non-VALID status). Runs a
    background poller to keep a fresh reading without blocking the
    controller loop.
    """

    SPEED_OF_SOUND_CM_S = 34300.0

    def __init__(self, cfg: UltrasonicConfig, poll_hz: float = 10.0, stale_after_s: Optional[float] = None) -> None:
        self.cfg = cfg
        self.poll_hz = max(1.0, float(poll_hz))
        # If the poll loop hasn't produced a result in this long, the reading
        # is considered STALE regardless of what the last status/value was.
        self._stale_after_s = float(stale_after_s) if stale_after_s is not None else max(0.5, 5.0 / self.poll_hz)

        self._lock = threading.Lock()
        self._last_distance_cm: Optional[float] = None
        self._last_status: UltrasonicStatus = UltrasonicStatus.UNKNOWN
        self._last_update_t: Optional[float] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True

        if GPIO is not None:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.cfg.trigger_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.cfg.echo_pin, GPIO.IN)

        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _poll_loop(self) -> None:
        period = 1.0 / self.poll_hz
        while self._running:
            try:
                status, d = self._measure()
            except Exception:
                status, d = UltrasonicStatus.ERROR, None

            with self._lock:
                self._last_status = status
                if status == UltrasonicStatus.VALID:
                    self._last_distance_cm = d
                self._last_update_t = time.monotonic()

            time.sleep(period)

    def _measure(self, timeout_s: float = 0.03) -> Tuple[UltrasonicStatus, Optional[float]]:
        if GPIO is None:
            return UltrasonicStatus.DISCONNECTED, None

        # Ensure trigger low
        GPIO.output(self.cfg.trigger_pin, GPIO.LOW)
        time.sleep(0.0002)

        # Send 10us trigger pulse
        GPIO.output(self.cfg.trigger_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.cfg.trigger_pin, GPIO.LOW)

        start_wait = time.monotonic()
        while GPIO.input(self.cfg.echo_pin) == 0:
            if time.monotonic() - start_wait > timeout_s:
                return UltrasonicStatus.TIMEOUT, None

        pulse_start = time.monotonic()
        while GPIO.input(self.cfg.echo_pin) == 1:
            if time.monotonic() - pulse_start > timeout_s:
                return UltrasonicStatus.TIMEOUT, None

        pulse_end = time.monotonic()
        pulse_duration = pulse_end - pulse_start

        distance_cm = (pulse_duration * self.SPEED_OF_SOUND_CM_S) / 2.0
        if distance_cm <= 0 or distance_cm > 500:
            return UltrasonicStatus.OUT_OF_RANGE, None
        return UltrasonicStatus.VALID, float(distance_cm)

    def get_reading(self) -> UltrasonicReading:
        """Status-explicit reading. Callers making safety decisions must use this,
        not get_distance(), so sensor failure can't be silently read as "clear"."""
        with self._lock:
            status = self._last_status
            distance = self._last_distance_cm
            last_update_t = self._last_update_t

        if last_update_t is None:
            return UltrasonicReading(UltrasonicStatus.UNKNOWN, None, None)

        age_s = time.monotonic() - last_update_t
        if age_s > self._stale_after_s:
            return UltrasonicReading(UltrasonicStatus.STALE, distance, age_s)
        return UltrasonicReading(status, distance, age_s)

    def get_distance(self) -> Optional[float]:
        """Backward-compatible accessor: cm on a fresh valid reading, else None.
        Callers needing to distinguish failure modes must use get_reading()."""
        reading = self.get_reading()
        return reading.distance_cm if reading.status == UltrasonicStatus.VALID else None
