#!/usr/bin/env python3

import time


def main() -> int:
    try:
        from robotx.hardware.motors import MotorDriver, MotorPins
        from robotx.config.settings import SETTINGS
    except Exception as e:
        print(f"ERROR: Could not import motor modules: {e}")
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

    print("Motor test starting.")
    print("SAFETY: Keep wheels off the ground for first test.")
    print("Press Ctrl+C to stop.")

    try:
        motors.start()

        print("Forward 2s...")
        motors.forward(0.45)
        time.sleep(2.0)

        print("Stop 1s...")
        motors.stop()
        time.sleep(1.0)

        print("Backward 2s...")
        motors.backward(0.45)
        time.sleep(2.0)

        print("Stop.")
        motors.stop()

    except KeyboardInterrupt:
        print("KeyboardInterrupt -> stopping motors")
        try:
            motors.stop()
        except Exception:
            pass
    except Exception as e:
        print(f"ERROR: motor test failed: {e}")
        try:
            motors.stop()
        except Exception:
            pass
        return 1
    finally:
        try:
            motors.cleanup()
        except Exception:
            pass

    print("Motor test done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
