"""The ESP32 **host simulator** as the link's serial port -- no UART, no motors.

The ESP32 repository (ROBOTX---ESP32) compiles its REAL sketch against a stubbed
Arduino core into a host executable (`tests/run_host_tests.py` builds
`rover_sim`), speaking protocol v2 on stdin/stdout. This wraps that process in
the `SerialPort` interface (`read` / `write` / `close`) so `Esp32Link` drives it
exactly as it drives `/dev/ttyAMA0`: same framing, CRC, sequence, ACK,
watchdog, safety gate and telemetry -- only the bytes go to a process instead
of a wire.

Selected by `ROBOTX_ESP32_SIMULATOR_EXE`. When it is set, no serial device is
opened at all; the link's port name is reported but never touched.

The simulator's own `SIM_*` variables (from the ESP32 repo's `tests/host`
`sim_hw.cpp`) configure the simulated hardware, e.g. `SIM_DRIVE_AVAILABLE=1`
for a verified PCA9685 and `SIM_FRONT_CM` for the front range sensors. They are
simulator inputs, and the link treats what comes back as it would from a real
ESP32.
"""

from __future__ import annotations

import os
import queue
import subprocess
import threading
from typing import Dict, Optional


class HostSimulatorPort:
    """`SerialPort` over the ESP32 host simulator's stdin/stdout."""

    def __init__(self, exe: str, *, env: Optional[Dict[str, str]] = None, read_timeout_s: float = 0.05) -> None:
        if not exe or not os.path.isfile(exe):
            raise FileNotFoundError(f"ESP32 host simulator not found: {exe!r}")
        full_env = dict(os.environ)
        full_env.update(env or {})
        self._timeout = read_timeout_s
        self._chunks: "queue.Queue[bytes]" = queue.Queue()
        self._buffer = bytearray()
        self._closed = False
        self._proc = subprocess.Popen(
            [exe],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            env=full_env,
            bufsize=0,
        )
        self._reader = threading.Thread(target=self._pump, name="esp32-sim-reader", daemon=True)
        self._reader.start()

    def _pump(self) -> None:
        out = self._proc.stdout
        while True:
            try:
                chunk = out.read1(4096) if hasattr(out, "read1") else out.read(1)
            except Exception:
                chunk = b""
            if not chunk:
                self._chunks.put(b"")
                return
            self._chunks.put(chunk)

    def read(self, size: int) -> bytes:
        if self._closed:
            raise OSError("ESP32 host simulator port is closed")
        if not self._buffer:
            try:
                chunk = self._chunks.get(timeout=self._timeout)
            except queue.Empty:
                return b""
            if chunk == b"":
                raise OSError("ESP32 host simulator exited")
            self._buffer += chunk
        while True:
            try:
                chunk = self._chunks.get_nowait()
            except queue.Empty:
                break
            if chunk == b"":
                self._chunks.put(b"")
                break
            self._buffer += chunk
        data = bytes(self._buffer[:size])
        del self._buffer[:size]
        return data

    def write(self, data: bytes) -> int:
        if self._closed or self._proc.poll() is not None:
            raise OSError("ESP32 host simulator is not running")
        self._proc.stdin.write(data)
        self._proc.stdin.flush()
        return len(data)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._proc.stdin.close()
        except Exception:
            pass
        try:
            self._proc.wait(timeout=3)
        except Exception:
            self._proc.kill()
