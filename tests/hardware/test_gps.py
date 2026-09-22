#!/usr/bin/env python3

"""MANUAL hardware check: real serial GPS receiver.

Not an automated test -- it needs a GPS module wired to the configured serial
port and a view of the sky. Run it by hand:

    venv/bin/python tests/hardware/test_gps.py

Prints the status-explicit reading each second so you can tell "no fix yet"
apart from "serial port is not there at all".
"""

import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def main() -> int:
    try:
        from robotx.config.logging_setup import setup_logging
        from robotx.config.settings import SETTINGS
        from robotx.hardware.gps import GPSConfig, GPSReader
    except Exception as e:
        print(f"ERROR: could not import GPS modules: {e}")
        return 2

    setup_logging("INFO")

    gps = GPSReader(GPSConfig.from_settings(SETTINGS))

    print("Starting GPS reader...")
    print(f"Port: {SETTINGS.gps_port} @ {SETTINGS.gps_baudrate}")
    print("A cold start outdoors can take several minutes to get a first fix.")
    print("Press Ctrl+C to stop.")

    gps.start()
    try:
        while True:
            reading = gps.get_reading()

            if reading.has_fix and reading.fix is not None:
                fix = reading.fix
                print(
                    f"status={reading.status.value} "
                    f"lat={fix.latitude:.6f} lon={fix.longitude:.6f} "
                    f"alt={fix.altitude_m} sats={fix.satellites} "
                    f"speed_mps={fix.speed_mps} track={fix.track_deg} "
                    f"age={reading.age_s:.1f}s"
                )
            else:
                detail = gps.status()
                print(
                    f"status={reading.status.value} "
                    f"sentences={reading.sentences_seen} "
                    f"error={reading.error or '-'}"
                )
                if detail.get("last_sentence"):
                    print(f"  last NMEA sentence: {detail['last_sentence']}")

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        gps.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
