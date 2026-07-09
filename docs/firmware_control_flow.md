# 固件控制链路

本文档说明当前“上位机命令到电机输出”的实际路径。这里描述的是已经实现的行为，不是未来目标。

## 一个控制周期

MCU 底盘任务以 500 Hz 运行。一个周期的路径如下：

```text
task_comm 缓存的命令
  -> task_chassis read_chassis_input()
  -> chassis_control_tick()
  -> attitude_estimator_update()
  -> chassis_planner_update()
  -> gait_machine_update()
  -> apply_attitude_compensation()
  -> leg_controller_apply_dt()
  -> motor vtable 命令
  -> motor_go_send_all() / motor_m3508_send_all()
```

## 输入阶段

`task_comm` 解析 USB CDC 协议帧。

- 功能号 `0x10`：底盘命令
- 基础 payload：`vx`、`vy`、`wz`
- 扩展 payload：`target_yaw`、`steer_mode`

`task_comm` 只缓存最新命令和心跳时间，不决定步态行为。

`task_chassis` 将缓存命令复制到 `chassis_control_input_t`，然后调用 `chassis_control_tick()`。

## 底盘控制阶段

`chassis_control_tick()` 负责行为流水线：

1. 在 MCU 构建中运行 GO 预标定窗口。
2. 输入为空时生成零命令快照。
3. 读取 IMU，更新 yaw/roll/pitch 姿态估计；需要闭环航向时得到实际使用的 `wz`。
4. 调用 planner。
5. 判断在线/离线状态。
6. 选择 stand、trot 或 script gait。
7. 更新 gait machine。
8. 在线时只给支撑相腿应用 planner 的每轮局部滚动速度。
9. 可选地将 roll/pitch 转成机身虚拟力矩，并写入腿部 `tau_ff` 前馈分配层。
10. 派发腿部命令。
11. 在 MCU 构建中刷新电机输出。

当前转向行为：

- `steer_mode == 0`：直接使用上位机给的 `wz`。
- `steer_mode == 1`：使用 BMI088 yaw 估计和 steering controller 计算实际 `wz`。

当前姿态行为：

- `attitude_estimator` 使用陀螺仪积分 yaw，同时用加速度计估计 roll/pitch。
- `g_chassis_attitude_comp.enable == 0` 为默认状态，不写入姿态平衡前馈。
- 打开后，`chassis_control` 根据 roll/pitch 和 roll/pitch rate 生成虚拟机身力矩 `Mx/My`，写入 `g_leg_gravity_comp.balance_mx_nm / balance_my_nm`。
- `half_track_m` 默认 `0.15 m`，对应 `0.30 m` 左右轮距；`half_length_m` 默认 `0.25 m`，对应 `0.50 m` 前后轮距。
- `scale` 可用于逐步放大或反向验证符号，`max_moment_nm` 和 `max_leg_force_n` 分别限制姿态力矩和单腿补偿力。

## Planner 阶段

`chassis_planner_update()` 将速度命令转换为：

- `moving` 标志
- walk/trot 通用步态参数
- 四个轮子的局部滚动速度目标

当前 yaw 转向使用 2DOF 轮腿可实现的刚体前后速度分量：

- `y_leg = 0.15 m`，表示腿/轮接触点到机体中心线的横向距离。
- 每条腿的局部前后速度按 `v_leg_x = vx - wz * y_leg` 计算。
- `gait_params.leg_step_length_m[i]` 保存每条腿实际步长。
- `step_length_m` 和 `turn_step_m` 只作为诊断摘要：共同步长均值、右左步长差的一半。
- 轮毂速度也来自同一个 `v_leg_x / wheel_radius`，用于匹配腿端接触速度。这里不是独立的轮子差速转向控制，而是让支撑轮圆周速度和同一条腿的步态位移保持一致。

当前步频/步幅调度：

- 在线行走采用轮足分工：轮子按 `v_leg_x / wheel_radius` 跟踪地面速度，腿保持 walk gait，不能退化成纯轮模式。
- 轮速只在支撑相写入；摆动相轮速清零，避免空中轮子继续按地面速度转。
- `g_chassis_stride_cfg.enable == 1` 时，planner 只在 `slow_period_s` 到 `fast_period_s` 之间小范围调度 gait 周期；默认是 `0.60 s -> 0.50 s`。
- 每腿步长由 `v_leg_x * period_s * duty` 计算并受 `max_leg_step_m` 限制，因此速度变化主要体现在轮速和步长上，步频只轻微变化。
- 低速原地/小半径转向仍使用 `g_chassis_turn_cfg` 的转向周期、抬脚高度和支撑占空比；默认转向抬腿高度也是 `0.055 m`。

当前限制：`vy` 会传入 planner，并参与 moving 判断，但 2DOF 腿不能生成真实横向足端轨迹，因此暂不进入运动学控制。

## 步态阶段

`gait_machine` 管理当前步态、目标步态和过渡混合。

- Stand 输出固定站立目标和零轮速。
- Trot 输出显式足端目标：`foot_x_m`、`foot_z_m`、`in_stance`。
- Trot 优先使用 `leg_step_length_m[i]` 生成每条腿的足端轨迹；未写入每腿步长时才回退到 `step_length_m ± turn_step_m`。
- Script gait 通过脚本播放器包装成 gait 接口。

在线移动时，planner 的每轮局部滚动速度只应用到支撑相腿；摆动相腿轮速置零。

姿态补偿不会改变 gait 相位、planner 决策或 `foot_z_m`，只在腿控制器中按当前支撑腿集合分配垂向力并转换成髋/膝 `tau_ff`。

## 腿和电机阶段

`leg_controller_apply_dt()` 的流程：

1. 调用 `leg_ik_solve_all()`。
2. 给 GO 髋/膝电机发送位置命令。
3. 给 M3508 轮毂电机发送 MIT 位置/速度参考命令。

轮子默认全面使用 MIT 模式，不再走速度模式。支撑相按轮速积分目标角度，摆动相用 MIT 保持当前轮角；`g_leg_wheel_mit` 只保留默认增益、限幅、参考角和诊断状态。

电机驱动负责把 vtable 命令转换成总线帧：

- GO 髋/膝电机通过 `motor_go_send_all()` 发送 RS485/RIS 帧。
- M3508 轮毂电机通过 `motor_m3508_send_all()` 发送 FDCAN/C620 电流帧。

## 安全相关

- 离线模式默认回到保守站立，除非当前处于手动保持。
- 急停由 safety task 处理，会在正常底盘输出前禁用电机。
- M3508 温度降额在 M3508 驱动内部处理。

## 验证命令

控制链路改动后执行：

```sh
cmake --build build_arm
cmake --build build_host_tests
ctest --test-dir build_host_tests --output-on-failure
git diff --check
```
