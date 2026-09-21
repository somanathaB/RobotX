import time
from dataclasses import dataclass
from typing import Optional, Tuple


class _MockPWM:
    def __init__(self, pin: int, hz: int):
        self.pin = pin
        self.hz = hz
        self.duty = 0.0

    def start(self, duty: float) -> None:
        self.duty = duty

    def ChangeDutyCycle(self, duty: float) -> None:
        self.duty = duty

    def stop(self) -> None:
        self.duty = 0.0


class _MockGPIO:
    BCM = "BCM"
    OUT = "OUT"
    IN = "IN"
    HIGH = 1
    LOW = 0

    def __init__(self):
        self._pins = {}

    def setmode(self, mode):
        return None

    def setwarnings(self, flag: bool):
        return None

    def setup(self, pin: int, mode, initial=None, pull_up_down=None):
        self._pins[pin] = initial if initial is not None else 0

    def output(self, pin: int, value: int):
        self._pins[pin] = value

    def PWM(self, pin: int, hz: int):
        return _MockPWM(pin, hz)

    def cleanup(self):
        return None


try:
    import RPi.GPIO as GPIO  # type: ignore
except Exception:  # pragma: no cover
    GPIO = _MockGPIO()  # type: ignore


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass(frozen=True)
class MotorPins:
    in1: int
    in2: int
    en: int
    invert: bool = False


class MotorDriver:
    """Differential drive (left/right) motor controller for L298N.

    `set_speed(left, right)` expects values in [-1.0, 1.0], where sign
    indicates direction and magnitude indicates duty cycle.
    """

    def __init__(
        self,
        left: MotorPins,
        right: MotorPins,
        pwm_hz: int = 1000,
        max_duty: float = 0.75,
    ) -> None:
        self.left = left
        self.right = right
        self.pwm_hz = pwm_hz
        self.max_duty = _clamp(max_duty, 0.0, 1.0)

        self._left_pwm = None
        self._right_pwm = None
        self._last_cmd: Tuple[float, float] = (0.0, 0.0)
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        for pin in (self.left.in1, self.left.in2, self.right.in1, self.right.in2, self.left.en, self.right.en):
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        self._left_pwm = GPIO.PWM(self.left.en, self.pwm_hz)
        self._right_pwm = GPIO.PWM(self.right.en, self.pwm_hz)
        self._left_pwm.start(0.0)
        self._right_pwm.start(0.0)
        self._started = True
        self.stop()

    def cleanup(self) -> None:
        try:
            self.stop()
        finally:
            try:
                if self._left_pwm is not None:
                    self._left_pwm.stop()
                if self._right_pwm is not None:
                    self._right_pwm.stop()
            finally:
                try:
                    GPIO.cleanup()
                except Exception:
                    pass
            self._started = False

    def _apply_motor(self, pins: MotorPins, pwm, speed: float) -> None:
        speed = _clamp(speed, -1.0, 1.0)
        if pins.invert:
            speed = -speed

        if speed > 0:
            GPIO.output(pins.in1, GPIO.HIGH)
            GPIO.output(pins.in2, GPIO.LOW)
        elif speed < 0:
            GPIO.output(pins.in1, GPIO.LOW)
            GPIO.output(pins.in2, GPIO.HIGH)
        else:
            GPIO.output(pins.in1, GPIO.LOW)
            GPIO.output(pins.in2, GPIO.LOW)

        duty = abs(speed) * self.max_duty * 100.0
        pwm.ChangeDutyCycle(_clamp(duty, 0.0, 100.0))

    def set_speed(self, left: float, right: float) -> None:
        if not self._started:
            self.start()
        assert self._left_pwm is not None and self._right_pwm is not None

        self._apply_motor(self.left, self._left_pwm, left)
        self._apply_motor(self.right, self._right_pwm, right)
        self._last_cmd = (left, right)

    def stop(self) -> None:
        if not self._started:
            return
        self.set_speed(0.0, 0.0)

    # Convenience motion primitives
    def forward(self, speed: float = 0.5) -> None:
        s = _clamp(speed, 0.0, 1.0)
        self.set_speed(s, s)

    def backward(self, speed: float = 0.5) -> None:
        s = _clamp(speed, 0.0, 1.0)
        self.set_speed(-s, -s)

    def turn_left(self, speed: float = 0.45) -> None:
        s = _clamp(speed, 0.0, 1.0)
        self.set_speed(-s, s)

    def turn_right(self, speed: float = 0.45) -> None:
        s = _clamp(speed, 0.0, 1.0)
        self.set_speed(s, -s)

    def brake(self, seconds: float = 0.05) -> None:
        """Active braking: drive both inputs HIGH briefly (optional)."""
        if not self._started:
            return
        for pins in (self.left, self.right):
            GPIO.output(pins.in1, GPIO.HIGH)
            GPIO.output(pins.in2, GPIO.HIGH)
        time.sleep(max(0.0, seconds))
        self.stop()

    @property
    def last_command(self) -> Tuple[float, float]:
        return self._last_cmd
