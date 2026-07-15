#!/usr/bin/env python3
"""FengMH Ubuntu 上位机流程控制器.

配置在同目录 mission_config.json；串口默认为自动识别，也可用 --port 指定。
"""
import argparse
import json
import math
import struct
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial：sudo apt install python3-pyserial", file=sys.stderr)
    raise SystemExit(1)

H1, H2 = 0x55, 0xAA
F_CHASSIS, F_TARGET, F_PUMP, F_MODE, F_AUX = 0x10, 0x11, 0x14, 0x15, 0x17
F_ARM_FB, F_CHASSIS_DIAG, F_MODE_FB = 0x86, 0x89, 0x8B
IDLE, NAV, ARM, PLACE = 0, 1, 2, 5
GRASP, PUT = 0, 1
PC8, PA8 = 0, 2
ARM_MOVING, ARM_REACHED, ARM_ERROR = 1, 2, 3


def frame(func, payload=b""):
    body = bytes((H1, H2, func, len(payload))) + payload
    return body + bytes((sum(body) & 0xFF,))


def load_config(path):
    cfg = json.loads(Path(path).read_text(encoding="utf-8"))
    for key in ("FirstNavDistanceM", "SecondNavDistanceM"):
        value = float(cfg[key])
        if not 0 < value <= 20:
            raise ValueError(f"{key} 必须在 0~20 m 范围内")
    for key in ("ArmGrasp1", "ArmPlace1", "ArmGrasp2", "ArmPlace2",
                "PlaceGrasp1", "PlacePlace1", "PlaceGrasp2", "PlacePlace2"):
        point = cfg[key]
        if len(point) != 3:
            raise ValueError(f"{key} 必须是 [x, y, z]")
        point = tuple(float(x) for x in point)
        radius = math.sqrt(sum(x * x for x in point))
        if not 0.045 <= radius <= 0.655:
            raise ValueError(f"{key} 超出下位机工作空间：半径 {radius:.4f} m")
        cfg[key] = point
    release_tolerance = float(cfg.get("PlacePumpReleaseToleranceM", 0.08))
    if not 0.04 <= release_tolerance <= 0.20:
        raise ValueError("PlacePumpReleaseToleranceM 必须在 0.04~0.20 m 范围内")
    cfg["PlacePumpReleaseToleranceM"] = release_tolerance
    return cfg


def find_port():
    ports = list(list_ports.comports())
    candidates = [p for p in ports if p.device.startswith(("/dev/ttyACM", "/dev/ttyUSB"))]
    if len(candidates) == 1:
        return candidates[0].device
    if not candidates:
        print("未找到 /dev/ttyACM* 或 /dev/ttyUSB* 串口。")
        for p in ports:
            print(f"  {p.device}: {p.description}")
        raise RuntimeError("没有可用串口")
    print("检测到多个候选串口：")
    for i, p in enumerate(candidates):
        print(f"  [{i}] {p.device}: {p.description}")
    return candidates[int(input("选择串口编号：").strip())].device


class MissionHost:
    def __init__(self, port, cfg, speed=0.5, timeout=60.0, tolerance=0.04):
        self.port_name, self.cfg = port, cfg
        self.speed, self.timeout, self.tolerance = speed, timeout, tolerance
        self.ser = serial.Serial(port, 115200, timeout=0.05, write_timeout=0.5)
        self.rx = bytearray()
        self.mode = -1
        self.safety = 0
        self.arm_state = -1
        self.arm_xyz = (math.nan, math.nan, math.nan)
        self.diag_target = None
        self.diag_filtered = None
        self.diag_at = 0.0
        self.pumps = {"PD11": False, "PA8": False, "PC8": False}
        self.stage = "连接"
        self.log(f"已连接 {port}")

    def log(self, text):
        print(f"[{time.strftime('%H:%M:%S')}] {text}", flush=True)

    def status(self):
        pumps = " ".join(f"{k}={'ON' if v else 'OFF'}" for k, v in self.pumps.items())
        print(f"    阶段: {self.stage} | 模式={self.mode} ARM={self.arm_state} | 气泵: {pumps}", flush=True)

    def send(self, func, payload=b""):
        self.ser.write(frame(func, payload))

    def send_mode(self, mode):
        self.send(F_MODE, bytes((mode,)))

    def send_chassis(self, vx=0.0):
        self.send(F_CHASSIS, struct.pack("<fff", vx, 0.0, 0.0))

    def send_target(self, typ, point):
        self.send(F_TARGET, struct.pack("<Bfff", typ, *point))

    def main_pump(self, on):
        self.pumps["PD11"] = bool(on)
        for _ in range(3):
            self.send(F_PUMP, bytes((int(on),)))
            time.sleep(0.02)
        self.log(f"PD11 主气泵 {'开启' if on else '关闭'}")
        self.status()

    def aux_pump(self, channel, name, on):
        self.pumps[name] = bool(on)
        for _ in range(3):
            self.send(F_AUX, bytes((channel, int(on))))
            time.sleep(0.02)
        self.log(f"{name} 辅助气泵 {'开启' if on else '关闭'}")
        self.status()

    def read_feedback(self):
        n = self.ser.in_waiting
        if n:
            self.rx.extend(self.ser.read(n))
        while len(self.rx) >= 5:
            if self.rx[:2] != bytes((H1, H2)):
                del self.rx[0]
                continue
            length = self.rx[3]
            total = length + 5
            if len(self.rx) < total:
                return
            raw = self.rx[:total]
            if (sum(raw[:-1]) & 0xFF) != raw[-1]:
                del self.rx[0]
                continue
            func, payload = raw[2], raw[4:4 + length]
            del self.rx[:total]
            if func == F_MODE_FB and len(payload) == 2:
                self.mode, self.safety = payload
            elif func == F_ARM_FB and len(payload) == 17:
                self.arm_state = payload[0]
                self.arm_xyz = struct.unpack_from("<fff", payload, 1)
            elif func == F_CHASSIS_DIAG and len(payload) == 48:
                self.diag_target = struct.unpack_from("<4h", payload, 4)
                self.diag_filtered = struct.unpack_from("<4h", payload, 12)
                self.diag_at = time.monotonic()

    def set_mode(self, mode, name):
        self.stage = f"切换 {name}"
        self.log(f"进入 {name} 模式")
        start = time.monotonic()
        while time.monotonic() - start < 4:
            self.send_mode(mode)
            self.read_feedback()
            if self.mode == mode and not self.safety:
                self.log(f"{name} 模式已确认")
                self.status()
                return
            time.sleep(0.1)
        raise RuntimeError(f"未收到 {name} 模式反馈")

    def stop_chassis(self, mode=NAV):
        for _ in range(10):
            self.send_mode(mode)
            self.send_chassis(0.0)
            self.read_feedback()
            time.sleep(0.02)

    def wait_chassis_stopped(self):
        self.stage = "NAV 停稳并切回 stand"
        self.log("持续发送零速度，等待四轮速度归零")
        start = time.monotonic()
        stable = 0
        while time.monotonic() - start < 5.0:
            self.send_mode(NAV)
            self.send_chassis(0.0)
            self.read_feedback()
            fresh = time.monotonic() - self.diag_at < 0.25
            if fresh and self.diag_target is not None and self.diag_filtered is not None:
                stopped = (max(abs(v) for v in self.diag_target) <= 100 and
                           max(abs(v) for v in self.diag_filtered) <= 100)
                stable = stable + 1 if stopped else 0
                if stable >= 5:
                    blend_end = time.monotonic() + 0.5
                    while time.monotonic() < blend_end:
                        self.send_mode(NAV)
                        self.send_chassis(0.0)
                        self.read_feedback()
                        time.sleep(0.02)
                    self.log("底盘轮速已归零，stand 过渡完成")
                    self.status()
                    return
            time.sleep(0.02)
        raise TimeoutError(
            f"底盘停止确认超时，target={self.diag_target}, filtered={self.diag_filtered}"
        )

    def drive(self, distance):
        duration = distance / self.speed
        self.stage = f"NAV 行走 {distance:.3f} m"
        self.log(f"NAV: vx={self.speed:.3f} m/s，行走 {distance:.3f} m，约 {duration:.3f} s")
        end = time.monotonic() + duration
        next_mode = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_mode:
                self.send_mode(NAV)
                next_mode = time.monotonic() + 0.1
            self.send_chassis(self.speed)
            self.read_feedback()
            time.sleep(0.02)
        self.wait_chassis_stopped()

    def hold(self, mode, seconds, text):
        self.stage = text
        self.log(f"等待 {seconds:.1f} s：{text}")
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send_mode(mode)
            self.send_chassis(0.0)
            self.read_feedback()
            time.sleep(0.08)

    def return_to_initial(self):
        self.stage = "中止任务并返回上电状态"
        self.log("收到 Ctrl+C：发送 IDLE，底盘归零，机械臂返回 PARK")
        start = time.monotonic()
        stable_reached = 0
        last_status = 0.0
        while time.monotonic() - start < self.timeout:
            self.send_mode(IDLE)
            self.send_chassis(0.0)
            self.read_feedback()
            if self.mode == IDLE:
                self.pumps.update(PD11=False, PA8=False, PC8=False)
                if self.arm_state == ARM_ERROR:
                    raise RuntimeError("下位机在 IDLE/PARK 回位过程中报告 ARM_ERROR")
                if self.arm_state == ARM_REACHED:
                    stable_reached += 1
                    if stable_reached >= 5:
                        self.log("已恢复上电初始状态：IDLE、底盘停止、机械臂 PARK、全部气泵关闭")
                        self.status()
                        return
                else:
                    stable_reached = 0
            if time.monotonic() - last_status >= 1.0:
                self.status()
                last_status = time.monotonic()
            time.sleep(0.08)
        raise TimeoutError("等待机械臂返回 PARK 超时")

    def move_wait(self, mode, typ, point, label,
                  on_near=None, near_tolerance=None):
        self.stage = label
        self.log(f"{label}: ({point[0]:.4f}, {point[1]:.4f}, {point[2]:.4f}) m")
        start = time.monotonic()
        next_target = 0.0
        stable = 0
        near_fired = False
        while time.monotonic() - start < self.timeout:
            now = time.monotonic()
            if now >= next_target:
                self.send_mode(mode)
                self.send_target(typ, point)
                next_target = now + 1.0
            self.read_feedback()
            if self.safety or self.arm_state == ARM_ERROR:
                raise RuntimeError(f"{label}：下位机报告错误")
            error = math.sqrt(sum((a - b) ** 2 for a, b in zip(self.arm_xyz, point)))
            if (on_near is not None and not near_fired and
                    near_tolerance is not None and error <= near_tolerance):
                self.log(
                    f"{label} 进入提前动作范围，误差 {error:.4f} m "
                    f"(阈值 {near_tolerance:.4f} m)"
                )
                on_near()
                near_fired = True
            if self.arm_state == ARM_REACHED:
                stable = stable + 1 if error <= self.tolerance else 0
                if stable >= 3:
                    self.log(f"{label} 到位，误差 {error:.4f} m")
                    return
            else:
                stable = 0
            time.sleep(0.02)
        raise TimeoutError(f"{label} 超时，最后坐标={self.arm_xyz}")

    def run(self):
        c = self.cfg
        place_release_tolerance = float(c["PlacePumpReleaseToleranceM"])
        self.set_mode(NAV, "NAV")
        self.drive(float(c["FirstNavDistanceM"]))
        self.set_mode(ARM, "ARM")

        self.main_pump(True)
        self.move_wait(ARM, GRASP, c["ArmGrasp1"], "ARM 一号箱抓取")
        self.aux_pump(PA8, "PA8", True)
        self.move_wait(ARM, PUT, c["ArmPlace1"], "ARM 一号箱放置")
        self.main_pump(False)
        self.hold(ARM, 3, "一号箱交接后等待二号观察位")

        self.main_pump(True)
        self.move_wait(ARM, GRASP, c["ArmGrasp2"], "ARM 二号箱抓取")
        self.aux_pump(PC8, "PC8", True)
        self.move_wait(ARM, PUT, c["ArmPlace2"], "ARM 二号箱放置")
        self.main_pump(False)

        self.set_mode(NAV, "NAV")
        self.hold(NAV, 5, "两个箱子由 PA8/PC8 保持")
        self.drive(float(c["SecondNavDistanceM"]))

        self.set_mode(PLACE, "PLACE")
        # 一号箱在进入 PLACE 后立即由主气泵接管，先释放车身槽位的 PA8。
        self.aux_pump(PA8, "PA8", False)
        self.main_pump(True)
        self.move_wait(PLACE, GRASP, c["PlaceGrasp1"], "PLACE 一号箱车身抓取")
        self.move_wait(PLACE, PUT, c["PlacePlace1"], "PLACE 一号箱地面放置")
        self.main_pump(False)
        self.hold(PLACE, 3, "一号箱放置后等待二号箱")

        self.main_pump(True)
        self.move_wait(
            PLACE, GRASP, c["PlaceGrasp2"], "PLACE 二号箱车身抓取",
            on_near=lambda: self.aux_pump(PC8, "PC8", False),
            near_tolerance=place_release_tolerance,
        )
        self.move_wait(PLACE, PUT, c["PlacePlace2"], "PLACE 二号箱地面放置")
        self.main_pump(False)
        self.hold(PLACE, 0.5, "最终稳定")
        self.log("MISSION COMPLETE")

    def run_nav_only(self):
        """仅测试两段 NAV，适用于机械臂未连接的情况。"""
        c = self.cfg
        self.set_mode(NAV, "NAV")
        self.drive(float(c["FirstNavDistanceM"]))
        self.hold(NAV, 3, "两段 NAV 之间延时")
        self.drive(float(c["SecondNavDistanceM"]))
        self.log("NAV ONLY TEST COMPLETE")


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="FengMH Ubuntu 上位机流程控制")
    parser.add_argument("--port", help="串口，例如 /dev/ttyACM0；不填则自动识别")
    parser.add_argument("--config", default=str(here / "mission_config.json"))
    parser.add_argument("--speed", type=float, default=0.5)
    parser.add_argument(
        "--nav-only", action="store_true",
        help="仅执行两段 NAV，中间延时 3 秒，不读取或控制机械臂",
    )
    args = parser.parse_args()
    try:
        cfg = load_config(args.config)
        port = args.port or find_port()
        host = MissionHost(port, cfg, args.speed)
        try:
            if args.nav_only:
                host.run_nav_only()
            else:
                host.run()
        except KeyboardInterrupt:
            print()
            if args.nav_only:
                host.stop_chassis(NAV)
                host.log("NAV 测试已中止：底盘已发送停止指令")
            else:
                host.return_to_initial()
        finally:
            if host.ser.is_open:
                if host.mode != IDLE:
                    host.stop_chassis(host.mode if host.mode in (NAV, ARM, PLACE) else NAV)
                host.ser.close()
    except Exception as exc:
        print(f"执行失败：{exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
