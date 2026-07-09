#!/usr/bin/env python3
"""
walk.py — FengMH 四足机器人行走控制

通过 USB CDC 发送 chassis_cmd (0x10) 帧控制机器人行走。
默认 50Hz 持续发送，维持心跳（超时 500ms 则机器人回 stand）。

用法:
  python3 tools/walk.py                      # 键盘交互控制
  python3 tools/walk.py --vx 0.1             # 前进 0.1 m/s，10 秒后停
  python3 tools/walk.py --vx 0.1 --vy 0.05   # 前进 + 侧移
  python3 tools/walk.py --port /dev/tty.usbmodemXXX  # 指定端口
  python3 tools/walk.py --list               # 列出可用串口

键盘控制:
  W/S    前进/后退 (±0.1 m/s)
  A/D    左移/右移 (±0.1 m/s)
  Q/E    左转/右转 (±0.5 rad/s)
  F      加速 (步长 ×2)
  空格   急停 (归零)
  ESC    退出
"""

import struct
import sys
import time
import argparse
import threading
import select
import termios
import tty
import os

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("需要 pyserial: pip3 install pyserial")
    sys.exit(1)

# ─── 协议常量 ────────────────────────────────────────────
PROTO_HEAD1 = 0x55
PROTO_HEAD2 = 0xAA
PROTO_FUNC_CHASSIS_CMD = 0x10

# ─── 步态参数 ────────────────────────────────────────────
# 与 MCU GAIT_PARAMS_TROT_DEFAULT 对应
STEP_LENGTH_DEFAULT = 0.06   # m
PERIOD_DEFAULT      = 0.4    # s
DUTY_DEFAULT        = 0.5

# 前向速度→步长的映射 (vx 在此范围线性映射到[0, step_length])
VX_MAX_FOR_STEP = 0.5  # m/s: 达到 step_length 的速度


def build_chassis_frame(vx: float, vy: float, wz: float) -> bytes:
    """构造 chassis_cmd 帧 (FuncID=0x10)"""
    payload = struct.pack("<fff", vx, vy, wz)
    length = len(payload)  # 12

    cksum = (PROTO_HEAD1 + PROTO_HEAD2 + PROTO_FUNC_CHASSIS_CMD + length) & 0xFF
    for b in payload:
        cksum = (cksum + b) & 0xFF

    return (
        bytes([PROTO_HEAD1, PROTO_HEAD2, PROTO_FUNC_CHASSIS_CMD, length])
        + payload
        + bytes([cksum])
    )


def find_port():
    """自动发现 STM32 USB CDC 端口"""
    ports = list(serial.tools.list_ports.comports())

    keywords = ["STM32", "STMicroelectronics", "usbmodem", "ttyACM",
                 "ttyUSB", "USB Serial", "CDC", "FengMH"]

    for port in ports:
        desc = f"{port.description or ''} {port.hwid or ''}".lower()
        for kw in keywords:
            if kw.lower() in desc:
                return port.device

    # 没有匹配的，列出所有让用户选择
    if not ports:
        print("[!] 未发现任何串口设备", file=sys.stderr)
        return None

    print("可用端口:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} — {p.description or '?'}")
    try:
        idx = input(f"选择端口 [0-{len(ports)-1}]: ").strip()
        return ports[int(idx)].device
    except (ValueError, IndexError):
        return None


class WalkController:
    """行走控制器：持续发送 chassis_cmd，维持心跳"""

    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.1, write_timeout=0.1)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.running = threading.Event()
        self.running.set()
        self._tx_thread: threading.Thread | None = None
        self._tx_rate = 50  # Hz

    def _tx_loop(self):
        interval = 1.0 / self._tx_rate
        while self.running.is_set():
            try:
                frame = build_chassis_frame(self.vx, self.vy, self.wz)
                self.ser.write(frame)
            except serial.SerialException as e:
                print(f"\n[!] 串口错误: {e}", file=sys.stderr)
                self.running.clear()
                break
            time.sleep(interval)

    def start(self):
        """启动发送线程"""
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._tx_thread.start()
        return self

    def set_vel(self, vx=0.0, vy=0.0, wz=0.0):
        self.vx = vx
        self.vy = vy
        self.wz = wz

    def stop(self):
        """平滑停止"""
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        time.sleep(0.15)
        self.running.clear()
        if self._tx_thread:
            self._tx_thread.join(timeout=1.0)
        self.ser.close()


# ─── 键盘交互 ────────────────────────────────────────────

def get_key():
    """非阻塞读取单个按键 (Unix)"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], 0.02)
        if r:
            ch = sys.stdin.read(1)
            if ch == "\x1b":  # ESC 序列（方向键等）
                r2, _, _ = select.select([sys.stdin], [], [], 0.01)
                if r2:
                    ch += sys.stdin.read(2)
            return ch
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def interactive_loop(ctrl: WalkController):
    """键盘交互主循环"""
    speed = 0.10  # m/s 基准步进
    turn  = 0.50  # rad/s 基准步进

    print("\n" + "=" * 50)
    print("  FengMH 行走控制 — 键盘交互模式")
    print("=" * 50)
    print("  W/S  前进/后退   A/D  左移/右移   Q/E  左转/右转")
    print("  F    加速×2     空格 急停         ESC  退出")
    print("=" * 50)
    print(f"  当前: vx=0 vy=0 wz=0  speed={speed:.2f}")
    print()

    last_state = (0.0, 0.0, 0.0)

    try:
        while ctrl.running.is_set():
            key = get_key()
            if key is None:
                continue

            vx, vy, wz = ctrl.vx, ctrl.vy, ctrl.wz

            if key in ("w", "W"):
                vx = speed
            elif key in ("s", "S"):
                vx = -speed
            elif key in ("a", "A"):
                vy = speed
            elif key in ("d", "D"):
                vy = -speed
            elif key in ("q", "Q"):
                wz = turn
            elif key in ("e", "E"):
                wz = -turn
            elif key == " ":  # 空格急停
                vx = 0.0
                vy = 0.0
                wz = 0.0
            elif key in ("f", "F"):  # 加速
                speed = min(speed * 2.0, 0.8)
                if vx != 0:
                    vx = speed if vx > 0 else -speed
                if vy != 0:
                    vy = speed if vy > 0 else -speed
            elif key in ("\x1b", "\x03"):  # ESC / Ctrl-C
                print("\n退出...")
                break
            else:
                # 无匹配按键 → 保持当前速度（持续发送维持心跳）
                pass

            ctrl.set_vel(vx, vy, wz)

            # 只在状态变化时打印
            new_state = (vx, vy, wz)
            if new_state != last_state:
                last_state = new_state
                parts = []
                if vx != 0:
                    parts.append(f"vx={vx:+.2f}")
                if vy != 0:
                    parts.append(f"vy={vy:+.2f}")
                if wz != 0:
                    parts.append(f"wz={wz:+.2f}")
                if not parts:
                    parts.append("STOP")
                print(f"\r  {' | '.join(parts)}    ", end="", flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        print("\n\n停止...")


def cli_mode(ctrl: WalkController, args):
    """命令行模式：发送指定速度，持续一段时间后停止"""
    ctrl.set_vel(args.vx, args.vy, args.wz)
    dur = args.duration or 10.0

    vx, vy, wz = ctrl.vx, ctrl.vy, ctrl.wz
    n = abs(vx) + abs(vy) + abs(wz)
    if n < 0.05:
        print("[!] 速度太小 (<0.05)，无法触发 trot，请增大 --vx/--vy/--wz")
        ctrl.stop()
        return

    print(f"行走中: vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}  持续 {dur:.0f}s ...")
    print("  (按 Ctrl-C 提前停止)")

    try:
        time.sleep(dur)
    except KeyboardInterrupt:
        pass
    finally:
        print("停止.")


# ─── 主入口 ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="FengMH 四足机器人行走控制",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                       键盘交互控制
  %(prog)s --vx 0.1              前进 0.1 m/s，10s 后停
  %(prog)s --vx 0.15 --duration 5 前进 0.15 m/s，5s 后停
  %(prog)s --vx 0.1 --wz 0.5     前进 + 左转
  %(prog)s --list                 列出可用端口
  %(prog)s --port /dev/tty.usbmodem123 指定端口
        """,
    )
    parser.add_argument("--port", "-p", help="串口设备路径（自动检测）")
    parser.add_argument("--baud", "-b", type=int, default=115200,
                        help="波特率（USB CDC 可忽略，默认 115200）")
    parser.add_argument("--list", action="store_true", help="列出所有可用串口")
    parser.add_argument("--vx", type=float, default=0.0,
                        help="前后速度 m/s (+前 -后)")
    parser.add_argument("--vy", type=float, default=0.0,
                        help="侧向速度 m/s (+左 -右)")
    parser.add_argument("--wz", type=float, default=0.0,
                        help="偏航角速度 rad/s (+左转 -右转)")
    parser.add_argument("--duration", "-t", type=float, default=None,
                        help="行走持续时间 (秒)，默认 10s (仅 CLI 模式)")
    args = parser.parse_args()

    if args.list:
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("未发现任何串口设备")
        else:
            for p in ports:
                print(f"  {p.device:<30} {p.description or '?'}")
        return

    # 找端口
    port = args.port or find_port()
    if not port:
        print("[!] 找不到 STM32 USB CDC 设备。用 --port 指定。", file=sys.stderr)
        sys.exit(1)

    print(f"[*] 连接 {port} ...")
    try:
        ctrl = WalkController(port, args.baud)
        ctrl.start()
    except serial.SerialException as e:
        print(f"[!] 无法打开 {port}: {e}", file=sys.stderr)
        sys.exit(1)

    # 判断模式
    has_cmd = abs(args.vx) + abs(args.vy) + abs(args.wz) > 0.001

    if has_cmd:
        cli_mode(ctrl, args)
    else:
        interactive_loop(ctrl)

    ctrl.stop()
    print("完成.")


if __name__ == "__main__":
    main()
