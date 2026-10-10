#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial>=3.5"]
# ///
"""Print kv4p firmware text output from the USB serial port.

The port carries the binary KV4P KISS protocol. This prints the plain-text
boot banner as is, and decodes COMMAND_DEBUG_* vendor frames (reset reason,
BLE link changes, per-transmission TX summaries) as timestamped lines. Other
frames (audio, state, window updates) are ignored.

The port is opened with DTR and RTS held low so that opening it does not
reset the ESP32 (which would also clear the reset-reason breadcrumb).

Usage:
    uv run tools/kv4p_log.py /dev/cu.usbserial-XXXX [--raw-out capture.bin]
"""

from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import sys

import serial

FEND, FESC, TFEND, TFESC = 0xC0, 0xDB, 0xDC, 0xDD
SETHARDWARE = 0x06
DEBUG_LEVELS = {0x01: "I", 0x02: "E", 0x03: "W", 0x04: "D", 0x05: "T"}


def stamp() -> str:
    return dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def handle_frame(frame: bytes) -> None:
    if len(frame) < 7 or frame[0] & 0x0F != SETHARDWARE or frame[1:5] != b"KV4P":
        return
    command = frame[6]
    if command in DEBUG_LEVELS:
        text = frame[7:].decode("utf-8", errors="replace").rstrip()
        print(f"{stamp()} [{DEBUG_LEVELS[command]}] {text}", flush=True)


class Decoder:
    """Splits the serial byte stream into plain text lines and KISS frames."""

    def __init__(self) -> None:
        self.in_frame = False
        self.escaped = False
        self.frame = bytearray()
        self.text = bytearray()

    def feed(self, chunk: bytes) -> None:
        for b in chunk:
            if b == FEND:
                # The firmware opens and closes every frame with FEND, so a
                # FEND after frame bytes closes it and plain text may follow.
                if self.in_frame and self.frame:
                    handle_frame(bytes(self.frame))
                    self.in_frame = False
                else:
                    self.in_frame = True
                self.frame.clear()
                self.escaped = False
            elif not self.in_frame:
                # Plain text outside frames: the boot banner, ROM boot
                # messages and panic backtraces ("Guru Meditation").
                if b in (0x0A, 0x0D):
                    if self.text:
                        print(
                            f"{stamp()} {self.text.decode('utf-8', errors='replace')}",
                            flush=True,
                        )
                        self.text.clear()
                elif 0x20 <= b < 0x7F:
                    self.text.append(b)
            elif self.escaped:
                self.frame.append(FEND if b == TFEND else FESC if b == TFESC else b)
                self.escaped = False
            elif b == FESC:
                self.escaped = True
            else:
                self.frame.append(b)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--raw-out", help="also append every received byte to this file"
    )
    args = parser.parse_args()

    port = serial.Serial()
    port.port = args.port
    port.baudrate = args.baud
    port.timeout = 0.2
    port.dtr = False
    port.rts = False
    decoder = Decoder()
    with contextlib.ExitStack() as stack:
        port.open()
        stack.callback(port.close)
        raw = stack.enter_context(open(args.raw_out, "ab")) if args.raw_out else None
        try:
            while True:
                chunk = port.read(4096)
                if raw is not None and chunk:
                    raw.write(chunk)
                    raw.flush()
                decoder.feed(chunk)
        except KeyboardInterrupt:
            return 0


if __name__ == "__main__":
    sys.exit(main())
