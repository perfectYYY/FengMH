# 固件控制链路深度导读

这份文档按“上机测试时怎么从现象追到函数和变量”的粒度写。事实来源是当前 FengMH 仓库，不引用工作区外 ROS 文件作为代码索引。

## 0. 安全默认值

- `USE_LEGACY_MAIN=0`：默认不走旧 `main.c` 业务死循环。
- `APP_CHASSIS_ENABLE=1`：默认启用底盘初始化和 `t_chassis`。
- `APP_ARM_MOTOR_OUTPUT_DEFAULT_ENABLE=(!APP_TARGET_HOST)`：MCU 上默认允许达妙输出，host 单测默认关闭。
- `APP_CHASSIS_COMP_FORCE_DISABLE=(!APP_TARGET_HOST)`：MCU 上默认强制关闭姿态补偿、机械臂载荷补偿和腿部 `tau_ff` 补偿；host 单测仍验证算法。
- `APP_ARM_FORCE_GRAVITY_ONLY=0`：默认不强制纯重力模式；现场可用 `debug_arm_force_gravity_only=1` 进入。
- `APP_ARM_POWER_ON_FIXED_GRASP_TEST=0`：默认走正式 ARM 等待策略，而不是上电固定抓取测试策略。

## 1. 启动排查

### 1.1 入口顺序

```text
Core/Src/main.c
  -> HAL_Init()
  -> SystemClock_Config()
  -> MX_GPIO/FDCAN/SPI/TIM/UART/DMA init
  -> app_init()
  -> osKernelInitialize()
  -> MX_FREERTOS_Init()
      -> defaultTask
      -> app_tasks_create()
  -> osKernelStart()
```

`app_init()` 必须在 `osKernelInitialize()` 前运行，因为 BMI088 等初始化可能使用 HAL timeout/delay。USB device 初始化在 `defaultTask` 中执行，`task_comm_init()` 先安装 CDC RX 回调，USB 真正枚举后即可收帧。

### 1.2 `app_init()` 初始化顺序

1. `log_init()`、`bsp_time_init()`。
2. `bsp_fdcan_init()`。
3. 底盘启用时初始化 UART2/UART3/UART4/UART7。
4. `bsp_usb_cdc_init()`。
5. 底盘启用时初始化 BMI088 的 SPI2。
6. 初始化主气泵 GPIO 和 `arm_pump`，默认关闭。
7. `motor_registry_init()`。
8. 底盘启用时 `motor_go_init_all()`、`motor_m3508_init_all()`。
9. `motor_damiao_init_all()`，机械臂 J1-J4 绑定到 FDCAN3。
10. 底盘启用时 `imu_bmi088_init()`；失败只降级姿态/航向相关功能。

### 1.3 `app_tasks_create()` 创建任务

| 任务 | 入口 | 周期/职责 |
|---|---|---|
| `t_log` | `task_log_entry()` | 低优先级日志占位 |
| `t_safety` | `task_safety_entry()` | 200 Hz 电机离线/温度/急停 |
| `t_comm` | `task_comm_entry()` | 上行状态 10 Hz、电机状态 5 Hz；下行在 USB RX 回调解析 |
| `t_chassis` | `task_chassis_entry()` | 500 Hz 底盘控制，`APP_CHASSIS_ENABLE` 时创建 |
| `t_arm` | `task_arm_entry()` | 1 ms 包装循环；`arm_control` 内部 10 ms 控制节拍 |

RTOS 诊断变量在 `app_tasks.c`：`debug_rtos_stack_overflow_count`、`debug_rtos_assert_count`、`debug_rtos_assert_file/line`。

## 2. 协议排查

帧格式：

```text
55 AA | func_id(1) | len(1) | payload(len) | checksum(1)
checksum = 从 0x55 到 payload 末尾累加低 8 位
```

解析链路：

```text
bsp_usb_cdc_on_rx()
  -> task_comm.on_usb_rx()
  -> proto_frame_feed()
  -> proto_dispatch_on_frame()
  -> handle_*
```

关键计数：

- `task_comm_good_cnt()`：校验正确帧数量。
- `task_comm_bad_cnt()`：帧头/长度/checksum 错误数量。
- `task_comm_dispatch_hit()`：FuncID 命中 handler 数量。
- `task_comm_dispatch_miss()`：未知 FuncID 数量。
- `task_comm_last_rx_ms()`：最近有效命令时间。

FuncID：

| FuncID | Payload | 下位机入口 | 结果 |
|---:|---|---|---|
| `0x10` | `<fff>` 或 `<fff f B>` | `handle_chassis()` | 写 `s_chassis`，刷新心跳 |
| `0x11` | `<Bfff>` | `handle_arm_target()` | 校验后写 `s_arm_target` |
| `0x12` | `payload_gait_cmd_t` | `handle_gait()` | stand/trot/walk/set-param |
| `0x13` | `payload_mit_cmd_t` | `handle_mit_cmd()` | 单电机 MIT 调试，拒绝 ARM_J1-J6 |
| `0x14` | `<B>` | `handle_arm_pump()` | 写 `s_arm_pump` |
| `0x15` | `<B>` | `handle_mode_cmd()` | 写 `s_mode_cmd`，ESTOP/ERROR 置急停 |
| `0x16` | `<BB2x4f>` | `handle_wheel_test()` | 腿固定 stand，直接测试 FL/FR/RL/RR 轮毂 MIT |
| `0x80` | state | `send_state_frame()` | MCU 上行 10 Hz |
| `0x81` | motor state | `send_motor_frame()` | MCU 上行 5 Hz 轮询 |
| `0x86` | arm feedback | `task_comm_send_arm_feedback()` | `task_arm` 上行 50 Hz |

机械臂 target 在通信入口有第一道过滤：`target_type` 必须是 `GRASP/PLACE`，`x/y/z` 必须 finite，距离必须在 `0.045 m` 到 `0.655 m` 之间。现场如果怀疑字节序或单位错误，先看 `debug_comm_arm_target_*` 和 accept/reject 计数。

## 3. 底盘链路排查

### 3.1 mode gate

`task_chassis.c` 的 `chassis_mode_allows_motion()`：

- `mode.seq == 0`：允许，兼容旧调试工具。
- 收到过 mode：只有 `PROTO_ROBOT_MODE_NAV` 允许底盘速度进入控制器。

`read_chassis_input()` 会在非 NAV 时把 `vx/vy/wz/target_yaw/steer_mode` 全部清零，但保留 command `seq` 和通信统计。

### 3.2 一个 500 Hz 周期

```text
task_chassis_step_for_test()
  -> chassis_control_tick()
      -> go_pre_calibration_ready()
      -> update_boot_stand()
      -> update_steering()
      -> slew_plan_command()
      -> chassis_planner_update()
      -> update_online_state()
      -> decide_gait_for_link_state()
      -> gait_machine_update()
      -> apply_plan_wheel_speed()
      -> apply_attitude_compensation()
      -> update_arm_load_compensation()
      -> leg_controller_apply_dt()
      -> flush_motor_outputs()
```

如果底盘完全不动，按这个顺序查：

1. `task_safety_estop_active()` 是否为 true。
2. `task_comm_dispatch_hit()` 是否增加，`task_comm_last_rx_ms()` 是否更新。
3. mode 是否为 NAV；如果不是，`read_chassis_input()` 会清零。
4. 是否仍在 GO 预标定或 boot stand ramp。
5. `chassis_control_get_status()` 中 `online/moving/effective_wz/gait_params/wheel_rads`。
6. `leg_controller` 是否绑定到 12 个执行器，`send_cnt/miss_cnt` 是否变化。
7. GO/M3508 是否有新反馈并在线。

### 3.3 planner 行为

普通移动：

- `moving=1`。
- 基础 gait 使用 `s_trot_params`。
- `online_decide()` 选择 `trot`。
- 默认使用现场标定的固定 `0.2525 s` 周期（约 `3.96 Hz`），抬脚高度固定 `0.055 m`，duty 为 `0.60`。
- `wheel_only_travel=1` 时，`vx` 不生成足端前后步长，直行足端只做高频原地抬落。

低速/原地 yaw 转向：

- `chassis_planner_is_low_speed_turn()` 为 true。
- 基础 gait 使用 `s_walk_params`。
- `planner_apply_turn_gait()` 覆盖转向抬脚高度和 duty。
- 默认复用普通行进的固定 `0.2525 s` 周期。
- 四轮仍使用完整 yaw 差速；腿部 yaw 步长默认乘 `turn_leg_scale=0.25`，作为辅助转向。
- `online_decide()` 选择 `walk`。

局部腿速度：

```text
left legs:  y_leg = +half_track
right legs: y_leg = -half_track
wheel_vx = vx - wz * y_leg
wheel_rads = wheel_vx / wheel_radius

普通行驶:
  foot_vx = -wz * y_leg
  leg_step_length_m[i] = foot_vx * period_s * duty, then clamped

低速/原地 yaw 转向:
  wheel_vx = vx - wz * y_leg
  foot_vx = vx - turn_leg_scale * wz * y_leg
  walk 周期默认与普通 trot 行进使用同一套速度调度
```

`vy` 当前只参与 moving 判断，不进入足端横向轨迹。

### 3.4 gait 到电机

`gait_machine_update()` 输出 `foot_x_m/foot_z_m/in_stance/wheel_mode`。`apply_plan_wheel_speed()` 在 online motion gait 中给四轮持续写入 planner 轮速，不再按 `in_stance` 清零；`trot <-> walk` 过渡的轮速 scale 保持为 1。

`leg_controller_apply_dt()`：

1. `leg_ik_solve_all()` 将足端目标转换为髋/膝角。
2. `gravity_comp_prepare_payload_shares()` 处理机械臂载荷。
3. `gravity_comp_prepare_balance_forces()` 处理姿态虚拟力矩。
4. `send_joint_targets()` 给 GO 髋/膝下发位置和 `tau_ff`。
5. `send_wheel_target()` 在 DRIVE 中下发有界积分 MIT，在 HOLD 中下发 MIT 锁轮参考。

轮毂 MIT 控制：

- DRIVE 模式跨支撑/摆动相持续积分实际速度误差，并用 `theta_actual + velocity_i` 作为 MIT 位置参考。
- 积分受 `min(pos_err_limit, tau_limit / kp)` 限制，并根据未限幅力矩做条件积分抗饱和。
- HOLD 模式只在进入时清积分并锁存一次实际轮角，stand 中持续保持该参考角和零速度。
- 默认增益、滚阻前馈、积分和限幅看 `g_leg_wheel_mit`。

独立轮驱测试使用 `0x16`，不经过 planner、步态轨迹或轮速 blend。控制任务固定输出 stand 足端目标，按 `wheel_mask` 应用 `wheel_rads[FL,FR,RL,RR]`；未选轮速度为 0。进入时清空并开启 M3508 trace，退出或 500 ms 超时时停止 trace 并切回 HOLD。

常用小端测试帧：

```text
# 四轮 +0.5 rad/s；需以 10 Hz 左右持续发送
55 AA 16 14 01 0F 00 00 00 00 00 3F 00 00 00 3F 00 00 00 3F 00 00 00 3F 35

# 仅 FL +0.5 rad/s
55 AA 16 14 01 01 00 00 00 00 00 3F 00 00 00 00 00 00 00 00 00 00 00 00 6A

# 退出测试并锁轮
55 AA 16 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 29
```

## 4. 机械臂链路排查

### 4.1 task_arm 状态

关键变量：

- `debug_arm_fixed_state`：`WAIT_FEEDBACK/MOVING/HOLDING/ERROR/RELEASED_TO_HOST/GRAVITY_ONLY`。
- `debug_arm_fixed_profile`：`PARK/FIRST_LIFT/FIRST_J1/SECOND_SAFE_LIFT/SECOND_J1/SECOND_FIXED_LIFT`。
- `debug_arm_fixed_last_result`：最近固定姿态或校验结果。
- `debug_arm_box_position_select`：现场手动选择第一/第二箱等待角。
- `g_arm_debug_snapshot`：100 ms 刷新的常用机械臂快照。

正式策略：

```text
非 ARM -> PARK
进入 ARM -> FIRST_LIFT -> FIRST_J1 -> HOLDING
HOLDING + 新合法 GRASP -> RELEASED_TO_HOST
PLACE cycle 完成 -> hold 3000 ms -> SECOND_SAFE_LIFT -> SECOND_J1 -> SECOND_FIXED_LIFT
```

因此 ARM 模式刚进入但还没收到合法 GRASP 时，机械臂保持等待姿态是预期行为。

### 4.2 USB target 如何真正生效

```text
handle_arm_target()
  -> s_arm_target.seq++
  -> task_arm.host_has_new_valid_grasp_target()
  -> release_fixed_pose_to_host()
  -> consume_new_commands(mode==ARM)
  -> Arm_Serial_Protocol_QueueTarget()
  -> Arm_Serial_Protocol_Process()
  -> Arm_Control_MoveToTypedPose()
  -> arm_control_set_target()
```

如果 target 被收到了但机械臂不动，依次查：

1. `debug_comm_arm_target_accept_count/reject_count`。
2. 当前 mode 是否为 `ROBOT_MODE_ARM`。
3. `debug_arm_fixed_state` 是否已经 `HOLDING` 或 `RELEASED_TO_HOST`。
4. target 是否为新的 `GRASP`；PLACE 不会释放等待姿态。
5. `debug_arm_grasp_j1_reject_count` 是否增加。
6. `debug_serial_rejected_command_count` 是否增加。
7. `arm_control_status_t.last_result/safe_move_stage/target_valid/target_pending`。

### 4.3 arm_control 内部

`arm_control_set_target()`：

- 检查 target type 和 finite。
- `arm_kinematics_inverse()` 求 J1-J4。
- 将 J1 归一到离当前参考最近的等效角。
- 新目标或 pending 目标进入 `plan_pending`。
- 运动中按当前阶段处理更新：收臂阶段更新后续目标，旋转阶段可重规划底座，展开/fine tracking 阶段大变化退回收臂，小变化在线重规划。

`arm_control_tick()`：

1. 绑定 registry 中的达妙电机。
2. 刷新 measured feedback 和 FK。
3. 输出 gate 打开时运行 `motor_damiao_process()` 自动 enable。
4. 每 `APP_ARM_CONTROL_PERIOD_MS` 执行一次控制。
5. pending 时规划安全阶段。
6. `arm_motion_update()` 采样五次轨迹。
7. `update_motion_gravity()` 计算重力补偿。
8. `command_arm_motors()` 给 J1-J4 达妙发送 MIT。
9. `update_settle_state()` 根据真实反馈误差、速度和超时判断 MOVING/REACHED/ERROR。

反馈新鲜度很重要：输出 gate 打开且 J1-J4 反馈不新鲜时，目标不会正常输出，运动中反馈超过 `APP_ARM_MOTION_FEEDBACK_ABORT_MS=250 ms` 会报错。

### 4.4 气泵和 PLACE cycle

`Arm_Serial_Protocol_Process()` 保留旧抓放语义：

- pump on：立即 `Pump_Control_Set(1)`，同步 payload loaded。
- pump off 且当前 target 是 PLACE 但还没 reached：延迟关泵。
- PLACE reached 后执行 `finish_place_cycle()`：关泵、切重力模式、等待下一次 GRASP、递增 place cycle sequence。
- `task_arm` 看到 place cycle sequence 增加后，进入 PLACE 后等待和第二箱固定姿态序列。

## 5. 达妙链路排查

注册和输出：

```text
motor_registry: ARM_J1..J4 -> MOTOR_DAMIAO, FDCAN3, CAN ID 1..4
motor_damiao_init_all()
  -> motor_registry_bind()
  -> bsp_fdcan_attach_rx(BSP_FDCAN_3, motor_damiao_fdcan_rx_cb)
arm_control_tick()
  -> motor_damiao_process()
  -> motor->ops->set_position()
```

自动 enable：

- `motor_damiao_process(now_ms)` 周期检查 J1-J4。
- 无反馈、未使能、反馈长时间沉默会发送 enable 特殊帧。
- 计数看 `motor_damiao_auto_enable_count()`。

反馈路由：

- `motor_damiao_fdcan_rx_cb()` 使用 payload 内 motor id 路由，避免只看 shifted CAN ID。
- 诊断变量：`debug_damiao_last_rx_can_id`、`debug_damiao_last_payload_motor_id`、`debug_damiao_feedback_route_count[]`、`debug_damiao_feedback_invalid_motor_id_count`。

## 6. 补偿链路

### 6.1 姿态补偿

`apply_attitude_compensation()` 默认在 MCU 被 `APP_CHASSIS_COMP_FORCE_DISABLE` 关闭。打开后：

```text
roll/pitch + roll_rate/pitch_rate
  -> deadband
  -> virtual Mx/My
  -> low-pass filter
  -> g_leg_gravity_comp.balance_*
  -> leg_controller gravity_comp_prepare_balance_forces()
  -> hip/knee tau_ff
```

### 6.2 机械臂载荷补偿

`update_arm_load_compensation()` 默认在 MCU 同样被强制关闭。打开后：

```text
arm_control_get_status()
  -> measured end pose preferred, planned pose fallback
  -> arm_gravity_comp_get_active_end_mass()
  -> total mass + estimated COM
  -> g_leg_gravity_comp.payload_*
  -> support leg payload distribution
  -> hip/knee tau_ff
```

详细见 [`arm_chassis_load_compensation.md`](arm_chassis_load_compensation.md)。

## 7. Host 测试覆盖

`host_tests/test_main.c` 覆盖当前关键链路：

- planner：轮驱零平移步长、前进/转向、`vy`、死区、高频周期调度和旧步长回退。
- gait：trot 原地抬腿峰值、每腿转向步长、walk 三支撑、足端字段。
- 底盘：端到端、`vx=0.1 m/s` 四轮有界积分 MIT 连续驱动、stand ramp/MIT 锁轮、独立轮驱/超时、motion gait 切换、轮速 blend、mode gate、姿态补偿、机械臂载荷补偿。
- 协议：FuncID 分区、USB 到缓存、ARM feedback、MIT 拒绝机械臂旁路。
- 机械臂：IK/FK、旧兼容层、轨迹、重补、payload 过渡、反馈 freshness、三段式、fine tracking、固定等待姿态、纯重补、ESTOP。
- 设备：气泵、达妙 MIT、达妙反馈、FDCAN3 路由、自动 enable。

建议验证：

```sh
cmake --build build_host_tests
ctest --test-dir build_host_tests --output-on-failure
cmake --build build_arm
git diff --check
```
