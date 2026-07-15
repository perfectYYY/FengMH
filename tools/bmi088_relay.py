#!/usr/bin/env python3
"""Cross-platform BMI088 v2 relay and FengMH receive diagnostics.

The relay owns two CDC serial ports:
  BMI/Damiao port -> source_port
  FengMH main-controller port -> main_port

Only complete, CRC-valid A5 5A v2 type-0x01 frames are written to main_port.
This program never creates or sends a 55 AA motion command.

Examples:
  Windows: python tools/bmi088_relay.py --source COM29 --main COM27
  Ubuntu:  python3 tools/bmi088_relay.py --source /dev/ttyACM0 --main /dev/ttyACM1
  Listen only (no writes): python3 tools/bmi088_relay.py --source /dev/ttyACM0 --main /dev/ttyACM1 --listen-only
"""

from __future__ import annotations

import argparse
import csv
import queue
import struct
import sys
import threading
import time
from pathlib import Path

try:
    import serial
    from serial import SerialException
except ImportError as exc:  # pragma: no cover - depends on target machine
    raise SystemExit("pyserial is required: python -m pip install pyserial") from exc


BMI_MAGIC = b"\xA5\x5A"
BMI_VERSION = 2
BMI_TYPE = 1
BMI_PAYLOAD_LEN = 188
BMI_FRAME_LEN = 198
BMI_CRC_OFFSET = 194
BMI_OUTPUT_PERIOD_S = 0.005  # Damiao output is approximately 200 Hz.
LEGACY_MAGIC = b"\x55\xAA"
CHASSIS_FUNC = 0x10
IMU_DIAG_FUNC = 0x83
IMU_DIAG_LEN = 52


def crc32_iso_hdlc(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return (~crc) & 0xFFFFFFFF


def le_u32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def le_f32(data: bytes, offset: int) -> float:
    return struct.unpack_from("<f", data, offset)[0]


def build_chassis_command(vx: float, vy: float, wz: float) -> bytes:
    payload = struct.pack("<fff", vx, vy, wz)
    frame = bytearray(LEGACY_MAGIC + bytes((CHASSIS_FUNC, len(payload))) + payload)
    frame.append(sum(frame) & 0xFF)
    return bytes(frame)


class BmiParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.valid = 0
        self.crc_errors = 0
        self.header_errors = 0
        self.discarded = 0
        self.queue_drops = 0

    def feed(self, data: bytes) -> list[bytes]:
        self.buffer.extend(data)
        frames: list[bytes] = []
        while True:
            start = self.buffer.find(BMI_MAGIC)
            if start < 0:
                # Preserve a possible first magic byte split across reads.
                keep = 1 if self.buffer and self.buffer[-1] == BMI_MAGIC[0] else 0
                self.discarded += len(self.buffer) - keep
                if keep:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            if start:
                self.discarded += start
                del self.buffer[:start]
            if len(self.buffer) < BMI_FRAME_LEN:
                break
            frame = bytes(self.buffer[:BMI_FRAME_LEN])
            if (frame[2] != BMI_VERSION or frame[3] != BMI_TYPE or
                    frame[4:6] != struct.pack("<H", BMI_PAYLOAD_LEN)):
                self.header_errors += 1
                del self.buffer[0]
                continue
            expected = le_u32(frame, BMI_CRC_OFFSET)
            actual = crc32_iso_hdlc(frame[2:194])
            del self.buffer[:BMI_FRAME_LEN]
            if actual != expected:
                self.crc_errors += 1
                continue
            self.valid += 1
            frames.append(frame)
        return frames


class MainParser:
    """Parser for legacy 55 AA frames, used only for diagnostics."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.diag_frames = 0
        self.bad_checksum = 0

    def feed(self, data: bytes) -> list[dict[str, int | float]]:
        self.buffer.extend(data)
        result: list[dict[str, int | float]] = []
        while True:
            start = self.buffer.find(LEGACY_MAGIC)
            if start < 0:
                if self.buffer and self.buffer[-1] == LEGACY_MAGIC[0]:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 5:
                break
            length = self.buffer[3]
            total = 5 + length
            if len(self.buffer) < total:
                break
            frame = bytes(self.buffer[:total])
            del self.buffer[:total]
            checksum = sum(frame[:4 + length]) & 0xFF
            if checksum != frame[4 + length]:
                self.bad_checksum += 1
                continue
            if frame[2] != IMU_DIAG_FUNC or length != IMU_DIAG_LEN:
                continue
            self.diag_frames += 1
            # payload_imu_state_t offsets inside the 55 AA frame.
            result.append({
                "accepted": le_u32(frame, 40),
                "crc_errors": le_u32(frame, 44),
                "skipped_samples": le_u32(frame, 48),
                "sequence": le_u32(frame, 8),
                "yaw_rad": le_f32(frame, 20),
                "gyro_z_rad_s": le_f32(frame, 24),
                "age_ms": struct.unpack_from("<H", frame, 52)[0],
                "valid": frame[54],
            })
        return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, help="Damiao BMI CDC port")
    parser.add_argument("--main", required=True, help="FengMH main-controller CDC port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=0.0,
                        help="seconds; 0 means run until Ctrl-C")
    parser.add_argument("--log", type=Path, help="optional CSV diagnostic log")
    parser.add_argument("--listen-only", action="store_true",
                        help="validate BMI frames but do not write to main port")
    parser.add_argument("--command-vx", type=float,
                        help="optional test vx (m/s); explicitly enables 0x10 output")
    parser.add_argument("--command-vy", type=float, default=0.0,
                        help="test vy (m/s), used with --command-vx")
    parser.add_argument("--command-wz", type=float, default=0.0,
                        help="test wz (rad/s), used with --command-vx")
    parser.add_argument("--command-duration", type=float, default=0.0,
                        help="seconds to repeat test command, then send zero")
    parser.add_argument("--command-period", type=float, default=0.05,
                        help="test command refresh period in seconds")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.command_vx is not None and args.command_duration <= 0:
        raise SystemExit("--command-duration must be positive when --command-vx is used")
    if args.command_vx is not None and args.listen_only:
        raise SystemExit("--command-vx cannot be combined with --listen-only")
    source = None
    controller = None
    log_file = None
    writer = None
    try:
        source = serial.Serial(args.source, args.baud, timeout=0.05,
                               write_timeout=0.5)
        controller = serial.Serial(args.main, args.baud, timeout=0.01,
                                   write_timeout=0.5)
        if args.log:
            log_file = args.log.open("w", newline="", encoding="utf-8")
            fields = ["host_time", "accepted", "crc_errors", "skipped_samples",
                      "sequence", "yaw_rad", "gyro_z_rad_s", "age_ms", "valid"]
            writer = csv.DictWriter(log_file, fieldnames=fields)
            writer.writeheader()

        bmi = BmiParser()
        main_parser = MainParser()
        forwarded = [0]
        started = time.monotonic()
        print(f"BMI source={args.source}; main={args.main}; "
              f"listen_only={args.listen_only}", flush=True)
        command_frames = [0]
        write_lock = threading.Lock()
        if args.command_vx is None:
            print("No 55 AA motion commands are generated by this program.", flush=True)
        else:
            print(f"TEST COMMAND vx={args.command_vx} vy={args.command_vy} "
                  f"wz={args.command_wz} duration={args.command_duration}s",
                  flush=True)

        frame_queue: queue.Queue[bytes] = queue.Queue(maxsize=512)
        stop_reader = threading.Event()
        reader_error: list[BaseException] = []

        def read_bmi() -> None:
            while not stop_reader.is_set():
                try:
                    incoming = source.read(4096)
                except (SerialException, OSError) as exc:
                    reader_error.append(exc)
                    stop_reader.set()
                    return
                for frame in bmi.feed(incoming):
                    try:
                        frame_queue.put(frame, timeout=0.1)
                    except queue.Full:
                        bmi.queue_drops += 1

        reader = threading.Thread(target=read_bmi, name="bmi-reader", daemon=True)
        reader.start()
        stop_writer = threading.Event()
        writer_error: list[BaseException] = []

        def write_bmi() -> None:
            next_write = time.monotonic()
            while not stop_writer.is_set():
                delay = next_write - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                try:
                    frame = frame_queue.get(timeout=0.05)
                    with write_lock:
                        controller.write(frame)
                    forwarded[0] += 1
                    next_write = max(next_write + BMI_OUTPUT_PERIOD_S,
                                     time.monotonic())
                except queue.Empty:
                    continue
                except (SerialException, OSError) as exc:
                    writer_error.append(exc)
                    stop_writer.set()
                    return

        writer_thread = None
        if not args.listen_only:
            writer_thread = threading.Thread(target=write_bmi,
                                              name="bmi-writer", daemon=True)
            writer_thread.start()
        command_thread = None
        stop_command = threading.Event()
        command_error: list[BaseException] = []
        if args.command_vx is not None:
            command_frame = build_chassis_command(args.command_vx,
                                                   args.command_vy,
                                                   args.command_wz)
            zero_frame = build_chassis_command(0.0, 0.0, 0.0)

            def send_test_command() -> None:
                deadline = time.monotonic() + args.command_duration
                try:
                    while time.monotonic() < deadline and not stop_command.is_set():
                        with write_lock:
                            controller.write(command_frame)
                        command_frames[0] += 1
                        time.sleep(max(0.001, args.command_period))
                    with write_lock:
                        controller.write(zero_frame)
                    command_frames[0] += 1
                except (SerialException, OSError) as exc:
                    command_error.append(exc)
                    stop_command.set()

            command_thread = threading.Thread(target=send_test_command,
                                               name="chassis-command", daemon=True)
            command_thread.start()
        try:
            while args.duration <= 0 or time.monotonic() - started < args.duration:
                if reader_error:
                    raise reader_error[0]
                if writer_error:
                    raise writer_error[0]
                if command_error:
                    raise command_error[0]

                diagnostics = main_parser.feed(controller.read(4096))
                for diag in diagnostics:
                    now = time.time()
                    print("IMU " + " ".join(f"{key}={value}" for key, value in diag.items()),
                          flush=True)
                    if writer:
                        writer.writerow({"host_time": now, **diag})
                        log_file.flush()
        finally:
            stop_writer.set()
            stop_reader.set()
            stop_command.set()
            reader.join(timeout=1.0)
            if writer_thread:
                writer_thread.join(timeout=1.0)
            if command_thread:
                command_thread.join(timeout=1.0)

        elapsed = time.monotonic() - started
        print("SUMMARY " + " ".join([
            f"seconds={elapsed:.2f}", f"bmi_valid={bmi.valid}",
            f"forwarded={forwarded[0]}", f"bmi_crc_errors={bmi.crc_errors}",
            f"bmi_header_errors={bmi.header_errors}",
            f"bmi_discarded={bmi.discarded}",
            f"bmi_queue_drops={bmi.queue_drops}",
            f"imu_diag_frames={main_parser.diag_frames}",
            f"command_frames={command_frames[0]}",
            f"main_bad_checksum={main_parser.bad_checksum}",
        ]), flush=True)
        return 0
    except KeyboardInterrupt:
        print("Stopped by user; no motion command was sent.", flush=True)
        return 0
    except (SerialException, OSError) as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 2
    finally:
        if log_file:
            log_file.close()
        if source and source.is_open:
            source.close()
        if controller and controller.is_open:
            controller.close()


if __name__ == "__main__":
    raise SystemExit(main())
