"""The Pi <-> ESP32 boundary: everything that knows the UART protocol exists.

    protocol.py   framing, CRC, strict decoding, command encoding (pure)
    transport.py  the serial port and line assembly
    state.py      what the ESP32 reported, as immutable values for RobotState
    link.py       the one component that owns /dev/ttyAMA0

The wire contract is PROTOCOL.md (protocol v2, firmware 82f2a8a). The ESP32
owns motor authority; this package only carries *gated* motion intent to it and
reports back what it says. It never originates motion.

This package is a separate boundary from `robotx.communication` (the backend)
and must stay one: nothing here imports the backend link, and nothing in the
backend package imports this one.
"""
