# 固件控制链路

本文档说明当前“USB 命令到电机/气泵输出”的实际路径。这里描述的是已实现行为，不是未来目标。

## 总图

```mermaid
flowchart TD
    MAIN["Core/Src/main.c\napp_init before osKernelInitialize"] --> TASKS["app_tasks_create"]
    TASKS --> COMM["task_comm\nUSB RX parser + command cache"]
    TASKS --> CH_TASK["task_chassis\n500 Hz wrapper"]
    TASKS --> ARM_TASK["task_arm\n1 ms wrapper, 10 ms arm control"]
    TASKS --> SAFETY["task_safety\nestop/offline/temp monitor"]

    USB["USB CDC bytes"] --> COMM
    COMM --> FRAME["proto_frame_feed"]
    FRAME --> DISP["proto_dispatch_on_frame"]
    DISP --> H10["0x10 chassis"]
    DISP --> H11["0x11 arm target"]
    DISP --> H12["0x12 gait debug"]
    DISP --> H13["0x13 MIT debug"]
    DISP --> H14["0x14 pump"]
    DISP --> H15["0x15 mode"]
    DISP --> H16["0x16 direct wheel test"]

    H10 --> CH_CACHE["s_chassis"]
    H11 --> ARM_CACHE["s_arm_target"]
    H14 --> PUMP_CACHE["s_arm_pump"]
    H15 --> MODE_CACHE["s_mode_cmd + task_safety_estop_set"]

    CH_CACHE --> CH_TASK
    MODE_CACHE --> CH_TASK
    CH_TASK --> CH_CTRL["chassis_control_tick"]
    CH_CTRL --> STEER["BMI088 attitude + steer_controller"]
    CH_CTRL --> PLAN["slew_plan_command + chassis_planner_update"]
    PLAN --> GAIT["stand / trot / walk / script via gait_machine"]
    GAIT --> LEG["leg_controller_apply_dt"]
    LEG --> IK["leg_ik_solve_all"]
    LEG --> GO["GO hip/knee position"]
    LEG --> WHEEL["M3508 bounded-integral MIT"]
    GO --> UART["bsp_uart"]
    WHEEL --> FDCAN12["bsp_fdcan CAN1/CAN2"]

    ARM_CACHE --> ARM_TASK
    PUMP_CACHE --> ARM_TASK
    MODE_CACHE --> ARM_TASK
    ARM_TASK --> FIXED["fixed wait / park profile"]
    FIXED --> LEGACYQ["Arm_Serial_Protocol queue"]
    LEGACYQ --> ARMCTRL["arm_control_tick"]
    ARMCTRL --> ARMIK["arm_kinematics"]
    ARMCTRL --> ARMMOTION["arm_motion quintic"]
    ARMCTRL --> ARMGC["arm_gravity_comp"]
    ARMCTRL --> DAMIAO["motor_damiao J1-J4 MIT"]
    ARMCTRL --> PUMP["arm_pump GPIO PD11"]
    DAMIAO --> FDCAN3["bsp_fdcan CAN3"]
    ARM_TASK --> FB["task_comm_send_arm_feedback 0x86"]
```

## 启动阶段

1. `Core/Src/main.c` 默认 `USE_LEGACY_MAIN=0`，只保留 HAL/时钟/外设启动。
2. `app_init()` 在 `osKernelInitialize()` 前运行，初始化 time、FDCAN、UART、USB CDC、SPI、GPIO、气泵、motor registry、GO/M3508/达妙、BMI088。
3. `MX_FREERTOS_Init()` 创建 CubeMX `defaultTask`，再调用 `app_tasks_create()`。
4. `defaultTask` 初始化 USB device；`app_tasks_create()` 初始化 `task_comm`、`task_chassis`、`task_arm` 并创建 app 任务。

## 通信输入

`task_comm` 是整机 USB CDC 唯一下行入口。

| FuncID | 当前处理 |
|---:|---|
| `0x10 CHASSIS_CMD` | 接受 12B `vx/vy/wz` 或 17B `target_yaw/steer_mode` 扩展帧，写 `s_chassis`。 |
| `0x11 ARM_TARGET` | 校验 `target_type`、有限值和 0.045-0.655 m 可达距离，写 `s_arm_target`。 |
| `0x12 GAIT_CMD` | stand/trot/walk/set-param 调试命令，走 `task_chassis_*` 包装。 |
| `0x13 MIT_CMD` | 单电机 MIT 调试；拒绝 ARM_J1-J6 旁路，防止抢机械臂 task。 |
| `0x14 ARM_PUMP` | 校验 `pump_on` 后写 `s_arm_pump`。 |
| `0x15 MODE_CMD` | 写 `s_mode_cmd`；`ESTOP/ERROR` 同步置位 `task_safety` 急停。 |
| `0x16 WHEEL_TEST` | 测试专用：模式 1 固定腿轮驱，模式 2 轮毂零电流自由推动，模式 3 按 NAV 参数原地踏步并独立轮驱；500 ms 超时锁轮。 |

上行：

- `0x80 STATE`：`task_comm_entry()` 10 Hz 发送。
- `0x81 MOTOR_STATE`：`task_comm_entry()` 5 Hz 轮询发送单电机状态。
- `0x86 ARM_FEEDBACK`：`task_arm` 50 Hz 发送机械臂末端反馈。

## 底盘控制周期

`task_chassis_entry()` 在 MCU 上以 500 Hz 调用：

```text
task_comm_get_chassis/mode
  -> mode gate: no mode frame allows legacy debug; NAV allows motion; others zero command
  -> chassis_control_tick()
```

`chassis_control_tick()` 的当前顺序：

1. MCU 构建先跑 GO 预标定窗口，采集反馈后 `motor_go_calibrate_all()`。
2. 上电 stand height ramp 从 `0.12 m` 平滑到目标站高。
3. 读取 BMI088，更新 yaw/roll/pitch；`steer_mode=1` 时用目标 yaw 生成实际 `wz`。
4. 对 `vx/vy/wz` 做斜率限制：`0.80 m/s^2`、`0.60 m/s^2`、`1.50 rad/s^2`。
5. `chassis_planner_update()` 生成 moving、walk/trot 参数、足端转向步长和每轮局部滚动速度。
6. 根据心跳和 mode 判断 online/offline。
7. online 且普通移动选择 `trot`；低速/原地 yaw 转向选择 `walk`；停止选择 `stand`。
8. `gait_machine_update()` 输出足端目标，过渡时混合输出。
9. 运动 gait 的四轮跨支撑/摆动相连续应用 planner 轮速；腿相位不参与轮毂驱动控制。
10. 姿态补偿和机械臂载荷补偿按编译期开关和 debug 开关写入 `g_leg_gravity_comp`。
11. `leg_controller_apply_dt()` 做 IK、支撑腿载荷分配、髋/膝位置命令，以及轮毂 DRIVE/HOLD MIT 命令。
12. MCU 构建 `motor_go_send_all()`、`motor_m3508_send_all()` 刷新总线输出。

当前 planner 约束：

- 轮速使用完整局部速度：`v_wheel_x = vx - wz * y_leg`，默认 `|y_leg| = 0.15 m`。
- 普通行驶默认启用 `wheel_only_travel`：足端不使用 `vx`，直行时三个步长字段全部为 0；边行驶边转向时只保留 `-wz * y_leg` 的左右步差。
- `vy` 只参与 moving 判断，不进入 2DOF 足端横向轨迹。
- `g_chassis_stride_cfg` 默认把普通行驶周期从 `0.25 s` 调到 `0.20 s`，固定抬脚高度 `0.055 m`、duty `0.60`；可用 `wheel_only_travel=0` 回退旧平移步长。
- `g_chassis_turn_cfg` 默认让低速 `walk` 转向复用普通行进的 `0.25 s -> 0.20 s` 周期调度；轮差速保持完整 `wz`，腿部 yaw 步长默认缩放为旧值的 `25%`。`match_travel_period` 和 `turn_leg_scale` 可用于现场回退或调参。

## 腿和轮输出

`leg_controller_apply_dt()`：

```text
gait_output
  -> leg_ik_solve_all()
  -> gravity_comp_prepare_payload_shares()
  -> gravity_comp_prepare_balance_forces()
  -> hip/knee motor->ops->set_position(... tau_ff)
  -> wheel bounded-integral MIT or stand MIT hold
```

轮毂 M3508 统一使用 MIT 路径：

- `GAIT_WHEEL_DRIVE`：所有轮子不受腿相位门控；`velocity_i += (wheel_rads - measured_rads) * dt`，再令 `theta_ref = theta_actual + velocity_i`。
- `velocity_i` 同时受位置误差/力矩上限约束；力矩饱和且误差继续推向饱和时暂停积分，误差反向时允许退饱和。
- `GAIT_WHEEL_HOLD`：进入 stand/保守保持时清 DRIVE 积分、一次锁存实际轮角，随后发送固定位置和零速度。
- `g_leg_wheel_mit` 提供 drive/hold mask、速度积分、增益、力矩限幅、位置误差限幅和 debug reset。

`0x16 WHEEL_TEST` payload 为 `mode:u8, wheel_mask:u8, reserved[2], wheel_rads[4]:f32`，轮序固定 `FL/FR/RL/RR`。`mode=0` 退出并锁轮，`1` 为固定腿纯轮驱，`2` 为轮毂零电流自由推动，`3` 为四腿按 NAV 行走参数仅上下踏步（步长 0）并使用四个独立轮速。模式 3 默认步高 `0.055 m`、周期 `0.20 s`（`5 Hz`）、占空比 `0.60`，并读取当前 `g_chassis_stride_cfg` 对应值。测试状态绕过正式 planner/gait，但仍由 500 Hz chassis task 下发；命令必须在 500 ms 内持续刷新，模式 2 退出时会在当前位置重新锁轮。

`0x89` 的 `flags` 位 12..13 同步输出当前测试模式，因此 payload 最后两个字节在无其他标志时分别为：普通状态 `00 00`、模式 1 `00 10`、模式 2 `00 20`、模式 3 `00 30`。

```text
自由推动（每 50 ms 循环）：
55 AA 16 14 02 0F 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 3A

原地踏步轮驱，FL/FR/RL=4.00 rad/s，RR=4.16 rad/s：
55 AA 16 14 03 0F 00 00 00 00 80 40 00 00 80 40 00 00 80 40 B8 1E 85 40 16

退出测试并锁轮：
55 AA 16 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 29
```

## 机械臂控制周期

`task_arm_entry()` 每 1 ms 调用 `task_arm_step_for_test()`；`arm_control_tick()` 内部按 `APP_ARM_CONTROL_PERIOD_MS=10` 控制达妙输出。

当前机械臂 task 不是由 mode 启停，而是常驻：

1. `ESTOP/ERROR` 急停时禁用达妙输出和控制器，不进入 `Arm_Control_Process()`，但保留气泵处理和反馈发送。
2. `APP_ARM_FORCE_GRAVITY_ONLY` 或 Live Expressions `debug_arm_force_gravity_only` 非零时，清空旧协议队列，关全部泵，只跑纯重力保持。
3. 非 ARM 模式默认进入 `PARK` 固定姿态；ARM 模式进入第一箱等待姿态。
4. 等待姿态到位后，只有新的合法 `GRASP` 目标会释放给 host 抓放流程。
5. `task_comm` 中缓存的 `0x11/0x14` 只有在 `ROBOT_MODE_ARM` 且固定姿态已释放后才排进 `Arm_Serial_Protocol_QueueTarget/QueuePump()`。
6. `Arm_Serial_Protocol_Process()` 保留旧 target/pump/place-cycle 语义：PLACE 到位后延迟关泵、切重力模式、等待下一次 GRASP。
7. `Arm_Control_Process()` 包装到 `arm_control_tick()`，执行 IK、三段式安全运动、fine tracking、五次轨迹、重力补偿、达妙 MIT 输出。
8. PLACE cycle 完成后，`task_arm` 等待 `APP_ARM_PLACE_HOLD_AFTER_PUMP_OFF_MS=3000 ms`，再进入第二箱等待姿态序列。

机械臂 target 校验：

- `task_comm` 先拦截错类型、非有限值和明显错单位/错字节序。
- `arm_control_validate_host_target()` 再运行 IK；`GRASP` 额外应用 J1 周期性禁区，`PLACE` 不应用该禁区。

## 安全链路

- `task_safety_estop_set(true)` 会遍历 registry 并调用每个电机的 `disable()`。
- `task_safety_entry()` 200 Hz 扫描电机反馈超时和温度；达妙超时默认跟机械臂运动反馈 abort 阈值 `250 ms`。
- M3508 温度警告和降额主要在 M3508 驱动内执行，超过 stop 阈值由 safety 禁用电机。
- `task_chassis_entry()` 急停时跳过底盘控制 tick。
- `task_arm_step_for_test()` 急停时跳过达妙自动 enable，避免急停中重新使能。

## 验证命令

```sh
cmake --build build_arm
cmake --build build_host_tests
ctest --test-dir build_host_tests --output-on-failure
git diff --check
```
