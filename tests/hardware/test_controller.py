#!/usr/bin/env python3

"""Standalone controller test (no Socket.IO, no Google Maps).

This verifies the basic closed-loop behavior:
- Read ultrasonic + IR (and encoders if available)
- Decide STOP vs FORWARD
- Always stop motors on exit
"""

import time


def main() -> int:
    try:
        from robotx.hardware.motors import MotorDriver, MotorPins
        from robotx.hardware.ultrasonic import UltrasonicConfig, UltrasonicSensor
        from robotx.hardware.ir import IRConfig, IRSensors
        from robotx.hardware.encoders import EncoderConfig, EncoderReader
        from robotx.config.settings import SETTINGS
    except Exception as e:
        print(f"ERROR: imports failed: {e}")
        return 2

    motors = MotorDriver(
        left=MotorPins(
            in1=SETTINGS.motor_left_in1,
            in2=SETTINGS.motor_left_in2,
            en=SETTINGS.motor_left_ena,
            invert=SETTINGS.motor_invert_left,
        ),
        right=MotorPins(
            in1=SETTINGS.motor_right_in3,
            in2=SETTINGS.motor_right_in4,
            en=SETTINGS.motor_right_enb,
            invert=SETTINGS.motor_invert_right,
        ),
        pwm_hz=SETTINGS.motor_pwm_hz,
        max_duty=SETTINGS.max_motor_duty,
    )

    ultrasonic = UltrasonicSensor(
        UltrasonicConfig(
            trigger_pin=SETTINGS.ultrasonic_trigger_pin,
            echo_pin=SETTINGS.ultrasonic_echo_pin,
        ),
        poll_hz=SETTINGS.ultrasonic_poll_hz,
    )

    ir = IRSensors(
        IRConfig(
            left_pin=SETTINGS.ir_left_pin,
            right_pin=SETTINGS.ir_right_pin,
            center_pin=SETTINGS.ir_center_pin,
            active_low=SETTINGS.ir_active_low,
        )
    )

    enc = EncoderReader(
        EncoderConfig(
            left_pin=SETTINGS.encoder_left_pin,
            right_pin=SETTINGS.encoder_right_pin,
            pulses_per_rev=SETTINGS.encoder_pulses_per_rev,
            wheel_diameter_m=SETTINGS.wheel_diameter_m,
        ),
        sample_hz=10.0,
    )

    obstacle_cm = float(getattr(SETTINGS, "obstacle_distance_cm", 35.0))

    print("Controller test starting (simplified).")
    print("Decision logic: if ultrasonic < threshold OR any IR triggered => STOP else FORWARD")
    print(f"Obstacle threshold: {obstacle_cm:.1f} cm")
    print("SAFETY: Keep wheels off the ground for first test.")
    print("Press Ctrl+C to stop.")

    motors.start()
    ultrasonic.start()
    ir.start()
    enc.start()

    try:
        while True:
            d = ultrasonic.get_distance()
            ir_state = ir.read_ir()
            speed = enc.get_speed()

            ir_triggered = any(v is True for v in ir_state.values() if v is not None)
            too_close = (d is not None and d < obstacle_cm)

            if too_close or ir_triggered:
                decision = "STOP"
                motors.stop()
            else:
                decision = "FORWARD"
                motors.forward(0.40)

            print(
                f"dist_cm={None if d is None else round(d,1)} "
                f"ir={ir_state} "
                f"avg_mps={speed.get('avg_mps', 0.0):.3f} "
                f"decision={decision}"
            )

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("Stopping...")
    except Exception as e:
        print(f"ERROR: controller test crashed: {e}")
        return 1
    finally:
        try:
            motors.stop()
        except Exception:
            pass
        try:
            ultrasonic.stop()
        except Exception:
            pass
        try:
            enc.stop()
        except Exception:
            pass
        try:
            motors.cleanup()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
