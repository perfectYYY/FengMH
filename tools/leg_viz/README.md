# leg_viz — 单腿可视化调试 GUI

Mac 上的纯 PC 调试工具。鼠标拖动足端 → 实时 IK → 用 FK 还原腿姿态画出来。
所有数学都通过 ctypes 调 `libfengmh_sim.dylib` 里的同一份 C 代码（也就是上板子的代码），
所以这里看到的就是固件实际计算的结果。

## 安装与构建

```bash
# 1. 装 Python 依赖
pip install -r tools/leg_viz/requirements.txt

# 2. 构建共享库（产物：build_host/libfengmh_sim.dylib）
cmake -S App/test -B build_host
cmake --build build_host --target fengmh_sim
```

固件里 `App/service/kinematics/leg_ik.c` 等公式有改动时，重跑第二步即可，GUI 立即同步。

## 运行

```bash
python tools/leg_viz/leg_viz.py
```

## 操作说明

- **左侧腿视图**：原点是髋关节，向前 +x，向上 +y。两个灰色虚线圆是工作空间外/内边界。
- **拖动红色足端点**：左键按下并拖动，腿姿态实时跟随。拖到工作空间外足端变深红色，姿态冻结在最近一次合法解。
- **滑条**：
  - `thigh L1 / shin L2`：腿长，立刻反映在工作空间圆和姿态上
  - `stand height`：站立高度偏移，IK 把它加到 z 上
- **leg_type**：`ORIGINAL`（RL/FR）vs `MIRROR`（FL/RR），同一足端坐标会得到不同的 knee 弯曲方向
- **script**：选中一个内置脚本（`stand_hold` / `wave_up_down` / `trot_step`）后点 **Play**，足端按脚本关键帧线性插值移动，橙色尾迹是足端轨迹
- **Reset**：清掉尾迹和时序曲线，回到默认姿态
- **右侧**：上面是 hip / knee 关节角的实时时序曲线；下面是当前所有参数文字面板

## 文件结构

- `sim_bridge.py` — ctypes 加载层，定义结构体和函数原型，暴露 `solve_ik / solve_fk / sample_script / list_builtin_scripts`
- `leg_viz.py` — matplotlib GUI 主程序

## 故障排查

- **`libfengmh_sim.dylib not found`**：去运行 `cmake --build build_host --target fengmh_sim`
- **窗口空白/不响应**：确保 matplotlib 用的是支持 GUI 的 backend，Mac 默认 `MacOSX` 即可
- **拖动无响应**：必须按住左键拖，不是单击
