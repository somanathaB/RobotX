"""The serial port and line assembly. No protocol knowledge beyond LF.

The port is only ever touched by the link's I/O thread: opened, read, written
and closed there. That single-owner rule is what rules out the reference
client's shutdown bug, where one thread closed the port while another was
still inside `read()`.
"""

from __future__ import annotations

from typing import List, Protocol

from robotx.esp32.protocol import BAUD, INBOUND_LINE_MAX


class SerialPort(Protocol):
    """What the link needs from a port. pyserial satisfies it; so do fakes."""

    def read(self, size: int) -> bytes: ...

    def write(self, data: bytes) -> int: ...

    def close(self) -> None: ...


def open_serial(port: str, baudrate: int = BAUD, timeout_s: float = 0.05) -> SerialPort:
    """Open the ESP32 UART: 8N1, no flow control, exclusively.

    `exclusive=True` takes an advisory lock; TIOCEXCL then makes the kernel
    refuse any further open() of the tty by another process (root excepted)
    for as long as this one holds it -- so a stray GPS reader, terminal or
    second agent cannot share the line. Both end when the port is closed.

    pyserial is imported here, not at module scope, so importing RobotX does
    not require it.
    """

    import fcntl
    import termios

    import serial  # pyserial

    s = serial.Serial(
        port,
        baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout_s,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
        exclusive=True,
    )
    try:
        fcntl.ioctl(s.fileno(), termios.TIOCEXCL)
    except Exception:
        s.close()
        raise
    return PySerialPort(s)


class PySerialPort:
    """pyserial, read the way a line protocol needs.

    `serial.Serial.read(n)` with a timeout blocks until `n` bytes arrive or
    the timeout expires -- so with a large `n` and a partly idle line it waits
    the full timeout on every call, adding that much latency to every frame,
    every ACK and every command queued behind the read. This waits (up to the
    timeout) for the first byte only, then takes whatever is already buffered.
    """

    def __init__(self, serial_port) -> None:
        self._s = serial_port

    def read(self, size: int) -> bytes:
        waiting = self._s.in_waiting
        return self._s.read(max(1, min(size, waiting)))

    def write(self, data: bytes) -> int:
        return self._s.write(data)

    def close(self) -> None:
        self._s.close()


class LineAssembler:
    """Bytes in, complete lines out. Bounded.

    Partial lines are held across reads for as long as they take; several
    lines in one read come out in order. A line that grows past `max_line`
    without an LF is discarded whole (never truncated) up to its LF, and
    counted once in `overflows`.
    """

    def __init__(self, max_line: int = INBOUND_LINE_MAX) -> None:
        self.max_line = max_line
        self.overflows = 0
        self._buf = bytearray()
        self._discarding = False

    def reset(self) -> None:
        self._buf.clear()
        self._discarding = False

    def feed(self, data: bytes) -> List[bytes]:
        lines: List[bytes] = []
        self._buf += data
        while True:
            idx = self._buf.find(b"\n")
            if idx < 0:
                break
            line = bytes(self._buf[: idx + 1])
            del self._buf[: idx + 1]
            if self._discarding:
                self._discarding = False  # the tail of an overlong line
                continue
            lines.append(line)
        # +2 allows the CR/LF that may still follow a maximal line.
        if len(self._buf) > self.max_line + 2:
            self._buf.clear()
            if not self._discarding:
                self._discarding = True
                self.overflows += 1
        return lines
