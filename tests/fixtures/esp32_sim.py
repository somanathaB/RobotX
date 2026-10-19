"""A SIMULATED ESP32 motor layer: what the firmware answers to the Pi's commands.

Not the ESP32 firmware and not its host simulator (that one needs the ESP32
repository's toolchain, which this Pi does not have). It models only what
PROTOCOL.md sections 9 and 11 say the firmware reports about motion, so the
Pi's handling of that feedback can be tested deterministically:

- every command is ACKed with its seq; DRIVE/STOP ACKs carry `req_*`, `gated_*`
  and `applied_*`;
- DRIVE is `GATED` (output 0) when the drive is unavailable
  (`MOTOR_PWM_UNAVAILABLE`) or a gate reason is injected, `REJECTED` when a
  rejection reason is injected;
- `applied` is the gated value with the +/-40 deadband applied, or 0 when
  `applied_zero` is injected (a motor layer outputting nothing);
- TELEMETRY's `left_cmd/right_cmd/left_applied/right_applied` mirror the last
  executed DRIVE/STOP.

`applied` is the PWM the firmware outputs, never wheel motion: nothing here, or
on the real rover, measures whether a wheel actually turned. A test that
passes against this model shows the Pi reacts correctly to the firmware's
report -- not that any motor moved.
"""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional, Tuple

from tests.fixtures import esp32 as fx

DEADBAND = 40


class FirmwareModel:
    """The motor-command half of the ESP32 firmware, SIMULATED."""

    def __init__(self, *, deadband: int = DEADBAND) -> None:
        self.deadband = deadband
        self.req: Tuple[int, int] = (0, 0)
        self.applied: Tuple[int, int] = (0, 0)
        # Fault injection.
        self.gate_reason: Optional[str] = None     # GATED with this reason, output 0
        self.reject_reason: Optional[str] = None   # REJECTED with this reason, not executed
        self.applied_zero = False                  # executed, but the motor layer outputs 0
        self.answer = True                         # False: commands are never ACKed
        self.applied_override: Optional[Tuple[int, int]] = None  # executed, but this is output instead
        self.executed: list = []
        # TELEMETRY `last_seq`: the last seq answered by an ACK (section 8).
        self.last_seq: Optional[int] = None

    def execute(self, command: Mapping[str, Any], fields: Optional[Mapping[str, Any]] = None) -> bytes:
        """The ACK frame for one command the Pi wrote ("" when not answering)."""

        if not self.answer:
            return b""
        fields = fields or {}
        seq, name = command["seq"], command["cmd"]
        self.last_seq = seq
        if name not in ("DRIVE", "STOP"):
            if name == "RESET":
                self.req, self.applied = (0, 0), (0, 0)
            return fx.ack(seq, cmd=name)
        if name == "DRIVE" and self.reject_reason:
            return fx.ack(seq, cmd=name, result="REJECTED", reason=self.reject_reason)

        req = (int(command.get("left", 0)), int(command.get("right", 0))) if name == "DRIVE" else (0, 0)
        result, reason, gated = "ACCEPTED", "NONE", req
        if name == "DRIVE":
            if not fields.get("motor_drive_available", True):
                result, reason, gated = "GATED", "MOTOR_PWM_UNAVAILABLE", (0, 0)
            elif fields.get("front_obstacle") and max(req) > 0:
                result, reason, gated = "GATED", "FRONT_OBSTACLE", (0, 0)
            elif self.gate_reason:
                result, reason, gated = "GATED", self.gate_reason, (0, 0)
        applied = tuple(0 if self.applied_zero or abs(v) < self.deadband else v for v in gated)
        if self.applied_override is not None and name == "DRIVE":
            applied = self.applied_override
        self.req, self.applied = req, (applied[0], applied[1])
        self.executed.append((name, req, result))
        return fx.ack(seq, cmd=name, result=result, reason=reason,
                      req_left=req[0], req_right=req[1], gated_left=gated[0], gated_right=gated[1],
                      applied_left=applied[0], applied_right=applied[1])

    def telemetry_fields(self) -> Dict[str, int]:
        return {"last_seq": self.last_seq, "left_cmd": self.req[0], "right_cmd": self.req[1],
                "left_applied": self.applied[0], "right_applied": self.applied[1]}
