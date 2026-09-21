from dataclasses import dataclass
from typing import Dict, Optional


try:
    import RPi.GPIO as GPIO  # type: ignore
except Exception:  # pragma: no cover
    GPIO = None  # type: ignore


@dataclass(frozen=True)
class IRConfig:
    left_pin: int
    right_pin: int
    center_pin: int
    active_low: bool = True


class IRSensors:
    """Digital IR sensors (line/obstacle).

    Returns a dict with `left/right/center` booleans where True means
    'triggered' (line detected or obstacle depending on wiring).
    """

    def __init__(self, cfg: IRConfig) -> None:
        self.cfg = cfg
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        if GPIO is not None:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            for p in (self.cfg.left_pin, self.cfg.right_pin, self.cfg.center_pin):
                GPIO.setup(p, GPIO.IN)
        self._started = True

    def read_ir(self) -> Dict[str, Optional[bool]]:
        if not self._started:
            self.start()

        if GPIO is None:
            return {"left": None, "right": None, "center": None}

        def _read(pin: int) -> bool:
            raw = GPIO.input(pin)
            return (raw == 0) if self.cfg.active_low else (raw == 1)

        return {
            "left": _read(self.cfg.left_pin),
            "right": _read(self.cfg.right_pin),
            "center": _read(self.cfg.center_pin),
        }
