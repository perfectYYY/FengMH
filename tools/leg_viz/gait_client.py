"""Send gait commands to the FengMH USB CDC protocol."""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]

PROTO_HEAD1 = 0x55
PROTO_HEAD2 = 0xAA
PROTO_FUNC_CHASSIS_CMD = 0x10
PROTO_FUNC_GAIT_CMD = 0x12

GAIT_ACTION_STAND = 0
GAIT_ACTION_TROT = 1
GAIT_ACTION_SET_TROT_PARAMS = 2


def build_frame(func_id: int, payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("payload too large")
    head = bytes((PROTO_HEAD1, PROTO_HEAD2, func_id & 0xFF, len(payload) & 0xFF))
    checksum = (sum(head) + sum(payload)) & 0xFF
    return head + payload + bytes((checksum,))


def gait_payload(action: int, args) -> bytes:
    phase = args.phase if args.phase is not None else (0.0, 0.5, 0.5, 0.0)
    if len(phase) != 4:
        raise ValueError("--phase needs exactly four values: FL FR RL RR")
    return struct.pack(
        "<B3x11f",
        action,
        float(args.height),
        float(args.step_length),
        float(args.step_height),
        float(args.period),
        float(args.duty),
        float(phase[0]),
        float(phase[1]),
        float(phase[2]),
        float(phase[3]),
        float(args.touchdown),
        float(args.blend),
    )


def chassis_payload(args) -> bytes:
    return struct.pack("<3f", float(args.vx), float(args.vy), float(args.wz))


def write_frame(port: str, baud: int, frame: bytes) -> None:
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: pip install -r tools/leg_viz/requirements.txt") from exc

    with serial.Serial(port, baudrate=baud, timeout=0.2, write_timeout=0.2) as ser:
        ser.write(frame)
        ser.flush()


def add_gait_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--height", type=float, default=0.18, help="body height in m")
    p.add_argument("--step-length", type=float, default=0.06, help="trot step length in m")
    p.add_argument("--step-height", type=float, default=0.04, help="swing foot height in m")
    p.add_argument("--period", type=float, default=0.4, help="gait period in s")
    p.add_argument("--duty", type=float, default=0.5, help="stance duty ratio")
    p.add_argument("--phase", type=float, nargs=4, metavar=("FL", "FR", "RL", "RR"))
    p.add_argument("--touchdown", type=float, default=0.0)
    p.add_argument("--blend", type=float, default=0.0)


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", help="serial device, for example /dev/tty.usbmodemXXX")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--hex", action="store_true", help="print frame hex instead of sending")

    sub = p.add_subparsers(dest="cmd", required=True)

    trot = sub.add_parser("trot", help="set trot params and start trot")
    add_gait_args(trot)

    set_trot = sub.add_parser("set-trot", help="set trot params without starting")
    add_gait_args(set_trot)

    stand = sub.add_parser("stand", help="switch to stand")
    stand.set_defaults(height=0.18, step_length=0.0, step_height=0.0, period=1.0,
                       duty=0.5, phase=(0.0, 0.0, 0.0, 0.0), touchdown=0.0, blend=0.0)
    stand.add_argument("--blend", type=float, default=0.0)

    vel = sub.add_parser("velocity", help="send chassis velocity command")
    vel.add_argument("--vx", type=float, default=0.0)
    vel.add_argument("--vy", type=float, default=0.0)
    vel.add_argument("--wz", type=float, default=0.0)

    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    if args.cmd == "velocity":
        frame = build_frame(PROTO_FUNC_CHASSIS_CMD, chassis_payload(args))
    else:
        action = {
            "stand": GAIT_ACTION_STAND,
            "trot": GAIT_ACTION_TROT,
            "set-trot": GAIT_ACTION_SET_TROT_PARAMS,
        }[args.cmd]
        frame = build_frame(PROTO_FUNC_GAIT_CMD, gait_payload(action, args))

    if args.hex or not args.port:
        print(frame.hex(" "))
        return 0

    write_frame(args.port, args.baud, frame)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
