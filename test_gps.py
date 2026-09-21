#!/usr/bin/env python3

import time


def main() -> int:
    try:
        from robotx.navigation.gps import GPSConfig, GPSReader
        from robotx.utils.config import SETTINGS
    except Exception as e:
        print(f"ERROR: Could not import GPS modules: {e}")
        return 2

    gps = GPSReader(GPSConfig(port=SETTINGS.gps_port, baudrate=SETTINGS.gps_baudrate))

    print("Starting GPS reader...")
    print(f"Port: {SETTINGS.gps_port} @ {SETTINGS.gps_baudrate}")
    print("Press Ctrl+C to stop.")

    gps.start()
    try:
        while True:
            loc = gps.get_location()
            if loc.get("lat") is None or loc.get("lon") is None:
                st = gps.status()
                print("No GPS fix yet.")
                if st.get("error"):
                    print("GPS error:", st["error"])
                time.sleep(2.0)
                continue

            print(
                f"lat={loc['lat']:.6f} lon={loc['lon']:.6f} fix_age_s={loc.get('fix_age_s'):.1f}"
            )
            time.sleep(2.0)

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        gps.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
