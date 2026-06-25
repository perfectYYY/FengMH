# 固件控制链路

本文档说明当前“上位机命令到电机输出”的实际路径。这里描述的是已经实现的行为，不是未来目标。

## 一个控制周期

MCU 底盘任务以 500 Hz 运行。一个周期的路径如下：

```text
task_comm 缓存的命令
  -> task_chassis read_chassis_input()
  -> chassis_control_tick()
  -> chassis_planner_update()
  -> gait_machine_update()
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
3. 更新 yaw 转向，得到实际使用的 `wz`。
4. 调用 planner。
5. 判断在线/离线状态。
6. 选择 stand、trot 或 script gait。
7. 更新 gait machine。
8. 在线时只给支撑相腿应用 planner 的共同前进轮速。
9. 派发腿部命令。
10. 在 MCU 构建中刷新电机输出。

当前转向行为：

- `steer_mode == 0`：直接使用上位机给的 `wz`。
- `steer_mode == 1`：使用 BMI088 yaw 估计和 steering controller 计算实际 `wz`。

## Planner 阶段

`chassis_planner_update()` 将速度命令转换为：

- `moving` 标志
- trot 步态参数
- 四个轮子的共同前进速度目标

当前 yaw 转向不再使用左右轮差速。`wz` 会被转换成 `gait_params.turn_step_m`：

- `step_length_m` 表示共同前后步长。
- `turn_step_m` 表示左右侧步长差；正 `wz` 时右侧腿步长增大、左侧腿步长减小。
- 原地/低速转向时 `step_length_m = 0`，`turn_step_m != 0`，轮子共同前进速度为 0。

当前限制：`vy` 会传入 planner，并参与 moving 判断，但还不是完整横向运动控制。

## 步态阶段

`gait_machine` 管理当前步态、目标步态和过渡混合。

- Stand 输出固定站立目标和零轮速。
- Trot 输出显式足端目标：`foot_x_m`、`foot_z_m`、`in_stance`。
- Trot 使用 `step_length_m ± turn_step_m` 生成左右侧足端轨迹，从步态层完成 yaw 转向。
- Script gait 通过脚本播放器包装成 gait 接口。

在线移动时，planner 的共同前进轮速只应用到支撑相腿；摆动相腿轮速置零。

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
