#!/usr/bin/env python3

import time


def main() -> int:
    try:
        from robotx.hardware.ultrasonic import UltrasonicConfig, UltrasonicSensor
        from robotx.utils.config import SETTINGS
    except Exception as e:
        print(f"ERROR: Could not import ultrasonic modules: {e}")
        return 2

    sensor = UltrasonicSensor(
        UltrasonicConfig(
            trigger_pin=SETTINGS.ultrasonic_trigger_pin,
            echo_pin=SETTINGS.ultrasonic_echo_pin,
        ),
        poll_hz=SETTINGS.ultrasonic_poll_hz,
    )

    print("Ultrasonic test starting.")
    print("Move an object in front of the sensor and observe distance changes.")
    print("Press Ctrl+C to stop.")

    sensor.start()
    try:
        while True:
            d = sensor.get_distance()
            if d is None:
                print("distance_cm=None (no reading)")
            else:
                print(f"distance_cm={d:6.1f}")
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        sensor.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
