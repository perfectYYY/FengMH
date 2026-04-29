# leg_viz — 3D 整车腿部调试 GUI

Windows / macOS / Linux 上的纯 PC 调试工具。主视图是一个 3D 底盘：机体、四个髋关节、四条腿、
足端世界坐标和足端轨迹都画在同一坐标系里。

GUI 不再自己计算脚本/IK。它通过 ctypes 启动 `host_sim`：在电脑上绑定 12 个
虚拟电机，然后运行真实的 `task_chassis -> gait/script -> leg_controller ->
motor_registry` 控制链。3D 视图只读取虚拟电机最终收到的命令和足端姿态。
这让 PC 端尽量接近“烧录到板子上的行为”。

## 安装与构建

需要 Python 3.10+、CMake，以及一个 C 编译器。Windows 推荐安装 Visual Studio Build Tools
或 MinGW-w64/Ninja。

macOS / Linux:

```bash
python3 -m pip install -r tools/leg_viz/requirements.txt
cmake -S App/test -B build_host
cmake --build build_host --target fengmh_sim
```

Windows PowerShell:

```powershell
py -3 -m pip install -r tools/leg_viz/requirements.txt
cmake -S App/test -B build_host
cmake --build build_host --target fengmh_sim
```

共享库产物会按平台生成在 `build_host` 下：
`fengmh_sim.dll` / `libfengmh_sim.dylib` / `libfengmh_sim.so`。若使用 Visual Studio
这类多配置生成器，`.dll` 可能在 `build_host\Debug` 或 `build_host\Release` 下；
`sim_bridge.py` 会自动搜索这些位置。

固件里 `App/service/kinematics/leg_ik.c`、`App/service/leg/leg_config.c`、
`App/device/motor_registry.c` 等配置或公式有改动时，重跑第二步即可，GUI 立即同步。

## 运行

```bash
python tools/leg_viz/leg_viz.py
```

Windows 也可以用：

```powershell
py -3 tools\leg_viz\leg_viz.py
```

## 操作说明

- **3D 视图**：X 向前、Y 向左、Z 向上；半透明矩形是底盘，四条彩色腿按真实连杆画出 FL / FR / RL / RR。
- **腿方向**：IK/电机命令已同步 `firmware-dev`，腿型、限幅、减速比、零位/启动姿态来自 C 侧配置；`leg_config.c` 里的 `xdir` 是真实装配的 x 镜像符号，IK 和 FK/可视化都会使用，当前 FL/RL 为 `-1`，FR/RR 为 `+1`。
- **播放**：按空格开始/暂停；按 `n` 在 `stand_hold` / `wave_up_down` / `trot_step` 间切换。
- **自定义小跑**：按 `g` 启停当前 GUI 参数下的 trot；`[`/`]` 调步长，`;`/`'` 调步高，`,`/`.` 调周期，`h`/`y` 调 duty。
- **目标选择**：按 `0` 选择 ALL 整车一起调；按 `1/2/3/4` 分别选择 FL/FR/RL/RR。
- **足端调节**：方向键左右调足端 x，方向键上下调抬腿 dz。
- **底盘调节**：`w/s` 调站立高度，`q/e` 调 pitch，`a/d` 调 roll。
- **视角/复位**：`v` 切换常用 3D 观察角，`r` 复位。
- **右侧面板**：显示每条腿的世界足端坐标、hip/knee 电机命令角、`xdir`、真实腿型和电机映射。
- **host 执行器**：右侧 `host` 行显示当前实际 C 侧步态；手动足端目标和脚本播放都会经过真实 `leg_controller` 后再显示。

## 下发到板子

`gait_client.py` 会按 `App/service/protocol/proto_defs.h` 里的 `PROTO_FUNC_GAIT_CMD`
构帧，可直接通过 USB CDC 设置固件里的 trot 参数并启动/停止步态：

```bash
python tools/leg_viz/gait_client.py --port /dev/tty.usbmodemXXXX trot \
  --height 0.18 --step-length 0.04 --step-height 0.02 --period 0.6 --duty 0.65

python tools/leg_viz/gait_client.py --port /dev/tty.usbmodemXXXX set-trot \
  --height 0.18 --step-length 0.03 --step-height 0.015 --period 0.8 --duty 0.7

python tools/leg_viz/gait_client.py --port /dev/tty.usbmodemXXXX stand
```

Windows 串口示例：

```powershell
py -3 tools\leg_viz\gait_client.py --port COM4 trot `
  --height 0.18 --step-length 0.04 --step-height 0.02 --period 0.6 --duty 0.65
```

不传 `--port` 或加 `--hex` 时只打印待发送帧，便于和串口助手或抓包结果对照。

## 机械模型说明

`leg_ik.c` 现在按真实平行连杆展开关键点：

- 髋轴到上膝点：`thigh_length = 0.100 m`
- 膝电机 40mm 摇臂：`link_length = 0.040 m`
- 摇臂末端到小腿安装点的从动连杆：`thigh_length = 0.100 m`
- 上膝点到小腿安装点：`link_length = 0.040 m`
- 上膝点到轮轴/足端：`shin_length = 0.150 m`

由于图纸里的 100/40/100/40 闭环是平行四边形，足端 IK 仍等效为
100mm 上连杆 + 150mm 小腿输出杆的闭式解；GUI 读取 C 侧
`leg_linkage_solve()` 的真实连杆点来显示 40mm 摇臂、100mm 从动杆和
150mm 小腿，不再自己画简化二连杆。

## 文件结构

- `sim_bridge.py` — ctypes 加载层，定义结构体和函数原型，暴露 host 执行器、配置表和诊断函数
- `host_sim.c/.h` — PC 端虚拟电机和真实控制链执行器
- `leg_viz.py` — matplotlib GUI 主程序，只显示 host 执行器状态

## 故障排查

- **找不到 `fengmh_sim` 共享库**：去运行 `cmake --build build_host --target fengmh_sim`；Windows 下确认 `build_host`、`build_host\Debug` 或 `build_host\Release` 里有 `fengmh_sim.dll`
- **Windows 提示缺少编译器**：安装 Visual Studio Build Tools 的 C++ 工作负载，或安装 MinGW-w64/Ninja 后重新运行 CMake
- **窗口空白/不响应**：确保 matplotlib 用的是支持 GUI 的 backend；Windows 通常用 `TkAgg`，macOS 默认 `MacOSX` 即可
- **按键无响应**：先用鼠标点一下 3D 窗口让它获得焦点，再按键；旧版按钮/滑条已移除以避开 macOS Matplotlib 3D backend 的 widget 卡死问题
- **窗口一直抢焦点**：确认运行的是最新版；新版使用后端 timer，不再用 `plt.pause()` 循环抢占前台窗口
