"""Receive-only ESP32 UART capture and analysis. Manual; never writes a byte.

Two ways to get the bytes, neither of which can transmit:

1. --from-strace FILE   (no service interruption)
   The running agent holds /dev/ttyAMA0 exclusively (TIOCEXCL), so nothing
   else can open it, and a second reader would steal bytes from the agent
   anyway. Instead, record what the agent itself read, with strace attached
   to its process (root needed; the agent is not paused or restarted):

     sudo timeout -s INT 60 strace -f -qq -tt -xx -s 65536 -e trace=read \
          -e signal=none -P /dev/ttyAMA0 \
          -p $(systemctl show -p MainPID --value robotx-agent) \
          -o /home/pi/esp32_rx_strace.txt
     sudo chown pi: /home/pi/esp32_rx_strace.txt

   then:  python3 -m tests.hardware.capture_esp32_rx --from-strace /home/pi/esp32_rx_strace.txt

2. --device /dev/ttyAMA0 --seconds 60   (only while the agent is STOPPED)
   Opens the tty O_RDONLY: the kernel refuses any write on that descriptor,
   so this mode cannot transmit even by mistake. With the agent running the
   open fails with EBUSY, and this script says so and exits.

Either way the bytes are split into lines and decoded with the Pi's own
decoder (robotx.esp32.protocol), and each GPS frame is run through the ESP32
GPS adapter. Raw lines are written to --out (byte for byte, CRC included) so
real frames can be added to tests/fixtures/esp32.py.
"""

from __future__ import annotations

import argparse
import collections
import errno
import os
import re
import select
import sys
import termios
import time
from typing import Dict, List, Tuple

from robotx.esp32.protocol import _OPTIONAL, _REQUIRED, Frame, decode_line
from robotx.esp32.transport import LineAssembler
from robotx.localization.esp32_gps import reading_from_frame


_READ = re.compile(rb'read\((\d+), "((?:\\x[0-9a-f]{2})*)"(\.\.\.)?, \d+\) = (\d+)')
_UNFINISHED = re.compile(rb'^(\d+)\s.*read\((\d+), <unfinished \.\.\.>')
_RESUMED = re.compile(rb'^(\d+)\s.*<\.\.\. read resumed>\s*"((?:\\x[0-9a-f]{2})*)"(\.\.\.)?, \d+\) = (\d+)')


def _unhex(escaped: bytes) -> bytes:
    return bytes.fromhex(escaped.replace(b"\\x", b"").decode("ascii"))


def bytes_from_strace(text: bytes) -> Tuple[bytes, Dict[str, int]]:
    """The bytes the agent read, in order, from an `strace -xx -s 65536` log."""

    out = bytearray()
    stats = collections.Counter()
    for line in text.splitlines():
        m = _READ.search(line)
        if m is not None:
            stats[f"fd {int(m.group(1))}"] += 1
            data, truncated, n = _unhex(m.group(2)), m.group(3), int(m.group(4))
        else:
            m = _RESUMED.search(line)
            if m is None:
                if _UNFINISHED.search(line):
                    stats["unfinished"] += 1
                continue
            data, truncated, n = _unhex(m.group(2)), m.group(3), int(m.group(4))
        if truncated or len(data) != n:
            stats["truncated_reads"] += 1   # -s too small: the capture is not byte-exact
        stats["reads"] += 1
        out += data
    return bytes(out), dict(stats)


def bytes_from_device(path: str, seconds: float) -> bytes:
    """Read `path` for `seconds`, on a descriptor that cannot be written."""

    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        if e.errno == errno.EBUSY:
            sys.exit(f"{path} is held exclusively (the agent is running). "
                     "Use --from-strace, or stop the service first.")
        raise
    try:
        attrs = termios.tcgetattr(fd)
        iflag, oflag, cflag, lflag, _, _, cc = attrs
        iflag = 0                                            # no CR/LF mangling, no XON/XOFF
        oflag = 0
        lflag = 0                                            # raw: no echo, no canonical mode
        cflag = (cflag & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB
                           | getattr(termios, "CRTSCTS", 0) | termios.HUPCL))
        cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL  # 8N1, no modem control
        cc[termios.VMIN], cc[termios.VTIME] = 0, 0
        termios.tcsetattr(fd, termios.TCSANOW,
                          [iflag, oflag, cflag, lflag, termios.B115200, termios.B115200, cc])
        out = bytearray()
        end = time.monotonic() + seconds
        while (left := end - time.monotonic()) > 0:
            ready, _, _ = select.select([fd], [], [], min(left, 0.5))
            if ready:
                out += os.read(fd, 4096)
        return bytes(out)
    finally:
        os.close(fd)


def analyse(raw: bytes) -> List[bytes]:
    """Print what the bytes contain. Returns the complete lines."""

    assembler = LineAssembler()
    lines = assembler.feed(raw)
    types = collections.Counter()
    rejected = collections.Counter()
    gps_keys = collections.Counter()
    gps_types: Dict[str, set] = collections.defaultdict(set)
    gps_verdicts = collections.Counter()
    tel_uptimes: List[int] = []
    documented_gps = {k for k, _ in _REQUIRED["GPS"]} | {k for k, _ in _OPTIONAL["GPS"]} | {"type"}

    for i, line in enumerate(lines):
        result = decode_line(line)
        if not isinstance(result, Frame):
            # The first line after attaching or opening starts mid-frame.
            label = "first line (partial on attach)" if i == 0 else result.reason.value
            rejected[f"{label} {result.detail[:60]}".strip()] += 1
            continue
        types[result.type] += 1
        data = dict(result.data)
        if result.type == "TELEMETRY":
            tel_uptimes.append(data["uptime_ms"])
        if result.type == "GPS":
            for k, v in data.items():
                gps_keys[k] += 1
                gps_types[k].add(type(v).__name__)
            # Judged as if read the instant it arrived: the frame's own content only.
            r = reading_from_frame(data, 0.0, now=0.0, stale_after_s=5.0, elapsed_s=0.0)
            gps_verdicts[f"{r.status.value}: {r.error or 'ok'}"] += 1

    print(f"bytes: {len(raw)}   complete lines: {len(lines)}   trailing partial: {len(raw) - sum(map(len, lines))} bytes")
    print("valid frames by type:", dict(types))
    print("rejected lines:", dict(rejected) or "none")
    if tel_uptimes:
        gaps = [b - a for a, b in zip(tel_uptimes, tel_uptimes[1:])]
        print(f"TELEMETRY uptime spacing ms: median {sorted(gaps)[len(gaps) // 2] if gaps else '-'}, "
              f"max {max(gaps) if gaps else '-'}, gaps > 450 ms: {sum(g > 450 for g in gaps)}")
    if types["GPS"]:
        print("GPS keys (count, JSON types):")
        for k in sorted(gps_keys):
            flag = "" if k in documented_gps else "   <- not in the decoder's documented schema"
            print(f"  {k:18s} {gps_keys[k]:5d}  {sorted(gps_types[k])}{flag}")
        missing = sorted(documented_gps - set(gps_keys))
        print("documented GPS keys never seen:", missing or "none")
        print("ESP32 GPS adapter verdicts:", dict(gps_verdicts))
    return lines


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--from-strace", metavar="FILE")
    src.add_argument("--device", metavar="TTY")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out", default="esp32_rx_lines.txt", help="raw lines, byte for byte")
    args = ap.parse_args()

    if args.from_strace:
        with open(args.from_strace, "rb") as f:
            raw, stats = bytes_from_strace(f.read())
        print("strace:", stats)
        if sum(k.startswith("fd ") for k in stats) > 1:
            print("WARNING: reads from more than one descriptor; was strace run with -P /dev/ttyAMA0?")
        if stats.get("truncated_reads"):
            print("WARNING: some reads were truncated; rerun strace with a larger -s")
    else:
        raw = bytes_from_device(args.device, args.seconds)

    lines = analyse(raw)
    with open(args.out, "wb") as f:
        f.writelines(lines)
    print(f"wrote {len(lines)} raw lines to {args.out}")


if __name__ == "__main__":
    main()
