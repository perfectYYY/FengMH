#!/usr/bin/env python3
"""USB CDC smoke test for the FengMH board.

The board must run APP_BRINGUP_STAGE_USB_CDC_TEST. The firmware sends
PROTO_FUNC_USB_CDC_STATE (0x84) frames and echoes host pings with
PROTO_FUNC_USB_CDC_PONG (0x85).
"""
from __future__ import annotations

import argparse
import struct
import sys
import time


HEAD = b"\x55\xAA"
FUNC_USB_CDC_PING = 0x21
FUNC_USB_CDC_STATE = 0x84
FUNC_USB_CDC_PONG = 0x85
STATE_STRUCT = struct.Struct("<IIIIIIIIII")
PONG_STRUCT = struct.Struct("<III")


def checksum(frame_without_sum: bytes) -> int:
    return sum(frame_without_sum) & 0xFF


def build_frame(func: int, payload: bytes) -> bytes:
    if len(payload) > 64:
        raise ValueError("payload too large")
    raw = HEAD + bytes([func, len(payload)]) + payload
    return raw + bytes([checksum(raw)])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate FengMH USB CDC link")
    parser.add_argument("port", help="USB CDC serial port, e.g. /dev/tty.usbmodemXXXX")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud placeholder")
    parser.add_argument("--timeout", type=float, default=8.0, help="seconds before failing")
    parser.add_argument("--pings", type=int, default=5, help="ping/pong exchanges required")
    parser.add_argument("--interval", type=float, default=0.2, help="seconds between pings")
    return parser.parse_args()


def read_frames(ser, deadline: float):
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(128)
        if chunk:
            buf.extend(chunk)

        while True:
            head = buf.find(HEAD)
            if head < 0:
                if len(buf) > 1:
                    del buf[:-1]
                break
            if head > 0:
                del buf[:head]
            if len(buf) < 5:
                break

            func = buf[2]
            length = buf[3]
            total = 5 + length
            if len(buf) < total:
                break

            raw = bytes(buf[:total])
            del buf[:total]
            if checksum(raw[:-1]) != raw[-1]:
                continue
            yield func, raw[4:4 + length]

        yield None, b""


def main() -> int:
    args = parse_args()
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: python3 -m pip install pyserial") from exc

    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.05)
    ser.reset_input_buffer()

    deadline = time.monotonic() + args.timeout
    next_ping_at = 0.0
    next_ping_seq = 1
    pong_seen: set[int] = set()
    state_seen = 0
    last_state = None

    for func, payload in read_frames(ser, deadline):
        now = time.monotonic()
        if next_ping_seq <= args.pings and now >= next_ping_at:
            ser.write(build_frame(FUNC_USB_CDC_PING, struct.pack("<I", next_ping_seq)))
            next_ping_at = now + args.interval
            next_ping_seq += 1

        if func is None:
            continue

        if func == FUNC_USB_CDC_STATE and len(payload) == STATE_STRUCT.size:
            state_seen += 1
            last_state = STATE_STRUCT.unpack(payload)
            seq, uptime_ms, rx_good, rx_bad, tx_ok, tx_busy, tx_err, pong_tx, last_ping_seq, last_ping_ms = last_state
            print(
                f"state seq={seq:5d} t={uptime_ms:8d}ms rx_good={rx_good:4d} rx_bad={rx_bad:4d} "
                f"tx_ok={tx_ok:4d} busy={tx_busy:4d} tx_err={tx_err:4d} "
                f"pong_tx={pong_tx:3d} last_ping={last_ping_seq:3d} last_ping_t={last_ping_ms:8d}ms"
            )
        elif func == FUNC_USB_CDC_PONG and len(payload) == PONG_STRUCT.size:
            ping_seq, uptime_ms, rx_good = PONG_STRUCT.unpack(payload)
            pong_seen.add(ping_seq)
            print(f"pong  seq={ping_seq:5d} t={uptime_ms:8d}ms rx_good={rx_good:4d}")

        if len(pong_seen) >= args.pings and state_seen > 0:
            return 0

    print(
        f"usb cdc smoke failed: states={state_seen}, pongs={sorted(pong_seen)}, last_state={last_state}",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
