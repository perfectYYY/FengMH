# leg_viz — 3D 整车腿部调试 GUI

Mac 上的纯 PC 调试工具。主视图是一个 3D 底盘：机体、四个髋关节、四条腿、
足端世界坐标和足端轨迹都画在同一坐标系里。IK、腿型、电机对应、方向、
CAN bus/id 都通过 ctypes 从 `libfengmh_sim.dylib` 读取同一份 C 代码和
`leg_config.c` 配置表，所以这里看到的就是固件实际使用的配置。

## 安装与构建

```bash
# 1. 装 Python 依赖
pip install -r tools/leg_viz/requirements.txt

# 2. 构建共享库（产物：build_host/libfengmh_sim.dylib）
cmake -S App/test -B build_host
cmake --build build_host --target fengmh_sim
```

固件里 `App/service/kinematics/leg_ik.c`、`App/service/leg/leg_config.c`、
`App/device/motor_registry.c` 等配置或公式有改动时，重跑第二步即可，GUI 立即同步。

## 运行

```bash
python tools/leg_viz/leg_viz.py
```

## 操作说明

- **3D 视图**：X 向前、Y 向左、Z 向上；半透明矩形是底盘，四条彩色连杆是 FL / FR / RL / RR。
- **腿方向**：`leg_config.c` 里的 `foot_x_dir` 把机体前向足端位移转换到单腿 IK 局部 x 轴；当前 FL/RL 为 `+1`，FR/RR 为 `-1`。
- **播放**：按空格开始/暂停；按 `n` 在 `stand_hold` / `wave_up_down` / `trot_step` 间切换。
- **目标选择**：按 `0` 选择 ALL 整车一起调；按 `1/2/3/4` 分别选择 FL/FR/RL/RR。
- **足端调节**：方向键左右调足端 x，方向键上下调抬腿 dz。
- **底盘调节**：`w/s` 调站立高度，`q/e` 调 pitch，`a/d` 调 roll。
- **视角/复位**：`v` 切换常用 3D 观察角，`r` 复位。
- **右侧面板**：显示每条腿的世界足端坐标、hip/knee 电机命令角、`xdir`、真实腿型和电机映射。

## 文件结构

- `sim_bridge.py` — ctypes 加载层，定义结构体和函数原型，暴露 `solve_ik / solve_fk / sample_script / list_leg_configs`
- `leg_viz.py` — matplotlib GUI 主程序

## 故障排查

- **`libfengmh_sim.dylib not found`**：去运行 `cmake --build build_host --target fengmh_sim`
- **窗口空白/不响应**：确保 matplotlib 用的是支持 GUI 的 backend，Mac 默认 `MacOSX` 即可
- **按键无响应**：先用鼠标点一下 3D 窗口让它获得焦点，再按键；旧版按钮/滑条已移除以避开 macOS Matplotlib 3D backend 的 widget 卡死问题
- **窗口一直抢焦点**：确认运行的是最新版；新版使用后端 timer，不再用 `plt.pause()` 循环抢占前台窗口
