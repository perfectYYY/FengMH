#!/usr/bin/env python3
"""Read BMI088 smoke-test telemetry from USB CDC.

Run tools/bringup/usb_cdc_smoke.py first. Then flash APP_BRINGUP_STAGE_IMU_TEST.
The firmware sends
PROTO_FUNC_IMU_STATE (0x83) frames with packed payload_imu_state_t.
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import time


HEAD = b"\x55\xAA"
FUNC_IMU_STATE = 0x83
PAYLOAD_STRUCT = struct.Struct("<IIhhhBBBBBBBBBfffffff")
STAGE_NAMES = {
    0: "none",
    1: "acc_initial_id",
    2: "acc_softreset",
    3: "acc_chip_id",
    4: "acc_cfg",
    5: "gyro_initial_id",
    6: "gyro_softreset",
    7: "gyro_chip_id",
    8: "gyro_cfg",
    9: "ready",
    10: "read",
}


def checksum(frame_without_sum: bytes) -> int:
    return sum(frame_without_sum) & 0xFF


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read FengMH BMI088 IMU telemetry")
    parser.add_argument("port", help="USB CDC serial port, e.g. /dev/tty.usbmodemXXXX")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud placeholder")
    parser.add_argument("--timeout", type=float, default=8.0, help="seconds before failing")
    parser.add_argument("--samples", type=int, default=20, help="valid samples required")
    parser.add_argument("--allow-motion", action="store_true", help="do not check static accel norm")
    return parser.parse_args()


def read_frames(port: str, baud: int, timeout_s: float):
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: python3 -m pip install pyserial") from exc

    ser = serial.Serial(port, baudrate=baud, timeout=0.1)
    buf = bytearray()
    deadline = time.monotonic() + timeout_s

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


def main() -> int:
    args = parse_args()
    valid = 0
    last = None
    accel_norm_bad = 0

    for func, payload in read_frames(args.port, args.baud, args.timeout):
        if func != FUNC_IMU_STATE or len(payload) != PAYLOAD_STRUCT.size:
            continue

        (
            seq,
            uptime_ms,
            err,
            diag_err,
            diag_io_err,
            ready,
            status,
            diag_stage,
            diag_reg,
            diag_val,
            acc_initial_id,
            acc_chip_id,
            gyro_initial_id,
            gyro_chip_id,
            ax,
            ay,
            az,
            gx,
            gy,
            gz,
            temp_c,
        ) = PAYLOAD_STRUCT.unpack(payload)
        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        gyro_norm = math.sqrt(gx * gx + gy * gy + gz * gz)
        stage_name = STAGE_NAMES.get(diag_stage, f"stage_{diag_stage}")
        last = (
            seq,
            uptime_ms,
            err,
            ready,
            status,
            stage_name,
            diag_err,
            diag_io_err,
            diag_reg,
            diag_val,
            acc_initial_id,
            acc_chip_id,
            gyro_initial_id,
            gyro_chip_id,
            accel_norm,
            gyro_norm,
            temp_c,
        )

        print(
            f"seq={seq:6d} t={uptime_ms:8d}ms err={err:3d} ready={ready} status={status} "
            f"diag={stage_name} derr={diag_err:3d} io={diag_io_err:3d} "
            f"reg=0x{diag_reg:02X} val=0x{diag_val:02X} "
            f"ids=acc0:0x{acc_initial_id:02X}/acc:0x{acc_chip_id:02X} "
            f"gyro0:0x{gyro_initial_id:02X}/gyro:0x{gyro_chip_id:02X} "
            f"acc=[{ax:+7.3f} {ay:+7.3f} {az:+7.3f}] |a|={accel_norm:6.3f} "
            f"gyro=[{gx:+8.4f} {gy:+8.4f} {gz:+8.4f}] |g|={gyro_norm:7.4f} "
            f"temp={temp_c:6.2f}C"
        )

        if err != 0 or ready != 1 or status != 0:
            return 2

        if not args.allow_motion and not (7.0 <= accel_norm <= 12.5):
            accel_norm_bad += 1
        else:
            valid += 1

        if valid >= args.samples:
            return 0

    if last:
        print(f"not enough valid samples, last={last}", file=sys.stderr)
        return 3
    print("no IMU telemetry received", file=sys.stderr)
    return 4


if __name__ == "__main__":
    raise SystemExit(main())
