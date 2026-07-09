# App 函数索引

本文档记录 `remake_original` 分支当前 App 层重要文件和函数。它是查表用文档：改代码前先用它定位函数，再去读源码。

## `App/app`

### `app_init.c`
- `app_init()`: 初始化日志、时间、FDCAN、UART、USB、SPI、电机注册表、GO 电机、M3508 电机和 BMI088；这是板级启动入口。

### `app_tasks.c`
- `app_tasks_create()`: 初始化通信和底盘控制，并在 MCU 构建中启动 FreeRTOS 任务。

### `task_chassis.c`
- `chassis_mode_allows_motion()`: 根据 `ROBOT_MODE_CMD` 判断底盘速度是否允许输出；无 mode 帧时保留旧调试兼容。
- `read_chassis_input()`: 将 `task_comm` 缓存的命令复制为 `chassis_control_input_t`，并按整机模式清零被 gate 的底盘速度。
- `task_chassis_init()`: 初始化底盘行为模块。
- `task_chassis_set_mode()`: 将底盘模式修改转发给 `chassis_control`。
- `task_chassis_get_mode()`: 读取当前底盘模式。
- `task_chassis_get_gait_active()`: 读取当前激活步态。
- `task_chassis_set_online_timeout_ms()`: 配置心跳超时时间。
- `task_chassis_get_online_timeout_ms()`: 读取心跳超时时间。
- `task_chassis_play_script()`: 请求播放脚本步态。
- `task_chassis_stop_script()`: 停止脚本步态并回到 stand。
- `task_chassis_start_stand()`: 手动请求 stand 步态。
- `task_chassis_set_trot_params()`: 更新默认 trot 参数。
- `task_chassis_get_trot_params()`: 读取当前 trot 参数。
- `task_chassis_start_trot()`: 手动请求 trot 步态。
- `task_chassis_active_gait_name()`: 返回当前步态实现名称。
- `task_chassis_reset_yaw()`: 重置 yaw 估计状态。
- `task_chassis_get_yaw()`: 读取 yaw 估计值。
- `task_chassis_get_effective_wz()`: 读取转向修正后的上一拍 yaw-rate 命令。
- `task_chassis_step_for_test()`: 运行一个底盘控制周期。
- `task_chassis_entry()`: 500 Hz MCU 任务包装，包含急停保护。

### `task_arm.c`
- `arm_usb_commands_allowed()`: 判断是否允许消费新的机械臂 USB 目标/气泵命令；只有 `ROBOT_MODE_ARM` 允许。
- `consume_new_commands()`: 读取机械臂目标和气泵缓存；非 ARM 模式同步 seq 并丢弃，ARM 模式才排进 `Arm_Serial_Protocol_QueueTarget()` / `Arm_Serial_Protocol_QueuePump()`。
- `build_integrated_feedback()`: 用旧机械臂 API 读取当前关节、末端位姿和运动状态，构造新整机协议反馈。
- `send_feedback_if_due()`: 按 50 Hz 节流发送 `0x86 ARM_FEEDBACK`。
- `task_arm_init()`: 按原工程顺序初始化 `Pump_Control`、`Arm_Control`、`Arm_Serial_Protocol`。
- `task_arm_step_for_test()`: 运行一个机械臂任务周期：USB gate -> 旧工程三连 process -> 新协议反馈。
- `task_arm_entry()`: 机械臂 MCU 任务 1ms 循环。

### `task_comm.c`
- `mark_valid_rx()`: 更新最近一次有效接收时间戳。
- `cache_chassis_base_command()`: 缓存底盘命令中的 `vx/vy/wz`。
- `cache_chassis_steering_extension()`: 缓存可选 target-yaw 转向扩展字段。
- `mark_chassis_command_rx()`: 增加底盘命令序号并刷新心跳时间。
- `handle_chassis()`: 解码 `0x10` 底盘速度命令 payload。
- `gait_params_from_payload()`: 将协议中的 gait payload 转换为 `gait_params_t`。
- `handle_gait()`: 解码 `0x12` stand/trot/set-params 命令。
- `handle_mit_cmd()`: 解码 `0x13` 单电机 MIT 调试命令。
- `handle_arm_target()`: 解码 `0x11` 机械臂 `target_type + arm_base xyz` 目标命令。
- `handle_arm_pump()`: 解码 `0x14` 机械臂气泵开关命令。
- `handle_mode_cmd()`: 解码 `0x15` 上位机请求的整机模式命令。
- `on_usb_rx()`: 将 USB 字节流送入协议帧解析器。
- `task_comm_init()`: 重置命令状态并安装协议分发表。
- `task_comm_good_cnt()`: 返回有效帧计数。
- `task_comm_bad_cnt()`: 返回无效帧计数。
- `task_comm_dispatch_hit()`: 返回命中分发表的计数。
- `task_comm_dispatch_miss()`: 返回未命中分发表的计数。
- `task_comm_last_rx_ms()`: 返回最近一次有效命令时间戳。
- `task_comm_get_chassis()`: 复制最近一次底盘命令。
- `task_comm_get_arm_target()`: 复制最近一次机械臂目标命令。
- `task_comm_get_arm_pump()`: 复制最近一次机械臂气泵命令。
- `task_comm_get_mode_cmd()`: 复制最近一次上位机模式命令。
- `task_comm_send_arm_feedback()`: 将机械臂反馈封装为 `0x86` 上行帧并通过 USB CDC 发送。
- `build_state_payload()`: 构造 `0x80` 整机状态遥测 payload。
- `send_proto_payload()`: 将一个 payload 封装成协议帧并通过 USB CDC 发送。
- `send_state_frame()`: 构造并发送 `0x80` 整机状态遥测帧。
- `build_motor_payload()`: 从电机注册表状态构造一个电机遥测 payload。
- `next_motor_tx_index()`: 推进轮询发送的电机遥测索引。
- `send_motor_payload_if_present()`: 电机句柄存在时发送其遥测。
- `send_motor_frame()`: 构造并发送一个轮询的 `0x81` 电机遥测帧。
- `task_comm_entry()`: MCU 遥测任务循环。

### `task_log.c`
- `task_log_entry()`: MCU 日志心跳任务占位。

### `task_safety.c`
- `task_safety_estop_active()`: 返回急停状态。
- `task_safety_event_count()`: 返回安全事件计数。
- `task_safety_estop_set()`: 修改急停状态；进入急停时禁用电机。
- `task_safety_entry()`: MCU 安全监控任务循环，检查超时和温度。

## `App/bsp`

### `bsp_gpio.c`
- `enable_port_clock()`: MCU 构建中按 GPIO port 打开外设时钟。
- `bsp_gpio_output_init()`: 初始化指定 App GPIO 输出，默认拉低。
- `bsp_gpio_write()`: 写指定 GPIO 输出 latch，并在 MCU 上同步 HAL GPIO。
- `bsp_gpio_read_latch()`: 读取 host/MCU 共用的软件 latch。
- `bsp_gpio_test_reset()`: host 测试辅助，清空 GPIO latch 和初始化状态。

### `bsp_fdcan.c`
- `fdcan_dwt_enable()`: 在 MCU 上启用 DWT cycle counter。
- `fdcan_cycles_per_us()`: 将 MCU 时钟换算为 cycles/us。
- `fdcan_wait_free_level()`: 等待 FDCAN TX FIFO 空位。
- `fdcan_warn_throttled()`: 对 FDCAN 警告日志做限频。
- `bsp_fdcan_init()`: 初始化 host mock 队列或 MCU FDCAN 滤波/中断。
- `bsp_fdcan_attach_rx()`: 注册某条总线的 RX 回调。
- `bsp_fdcan_send()`: 发送一帧 CAN，或在 host 中压入 TX 队列。
- `bsp_fdcan_get_bus_err_cnt()`: 返回指定 FDCAN 总线累计错误计数。
- `bsp_fdcan_hal_rxfifo0_cb()`: 将 HAL RX FIFO 回调桥接到 BSP 回调。
- `bsp_fdcan_hal_error_cb()`: 记录 FDCAN 错误中断状态，并在 `BUS_OFF` 时触发恢复和通知重注册。
- `bsp_fdcan_test_tx_count()`: host 测试辅助，返回 TX 队列帧数。
- `bsp_fdcan_test_pop_tx()`: host 测试辅助，弹出一帧 TX。
- `bsp_fdcan_test_inject_rx()`: host 测试辅助，注入一帧 RX。
- `bsp_fdcan_test_reset()`: host 测试辅助，清空 mock 队列。

### `bsp_spi.c`
- `bsp_spi_init()`: 初始化 SPI 总线或 host mock。
- `bsp_spi_set_cs()`: 控制片选线或 host mock CS 状态。
- `bsp_spi_transfer_byte()`: 传输一个 SPI 字节。
- `bsp_spi_test_attach_xfer()`: host 测试辅助，安装传输回调。
- `bsp_spi_test_reset()`: host 测试辅助，清空 SPI mock 状态。

### `bsp_time.c`
- `now_ns_raw()`: host 辅助，返回单调纳秒时间。
- `bsp_time_init()`: 初始化 host 起始时间；MCU 依赖 HAL SysTick。
- `bsp_time_now_ms()`: 返回当前毫秒时间。
- `bsp_time_now_us()`: 返回当前微秒时间。
- `bsp_time_delay_ms()`: 毫秒延时。
- `bsp_time_delay_us()`: 微秒延时。
- `bsp_time_test_advance_ms()`: host 测试辅助，推进模拟时间。

### `bsp_uart.c`
- `uart_dwt_enable()`: 启用 MCU UART 等待用 DWT 计时。
- `uart_cycles_per_us()`: 将 MCU 时钟换算为 cycles/us。
- `bsp_uart_init()`: 初始化 UART/RS485 总线或 host TX 队列。
- `bsp_uart_attach_rx()`: 注册 UART 总线 RX 回调。
- `bsp_uart_send()`: 发送 UART 字节，或在 host 中压入 TX 队列。
- `bsp_uart_wait_tx_done()`: 等待 RS485 TX 窗口完成。
- `bsp_uart_on_rx()`: 将底层 RX 桥接到注册回调。
- `bsp_uart_on_tx_done()`: 标记 MCU DMA TX 完成。
- `bsp_uart_test_tx_count()`: host 测试辅助，返回 TX 包数量。
- `bsp_uart_test_pop_tx()`: host 测试辅助，弹出一个 TX 包。
- `bsp_uart_test_inject_rx()`: host 测试辅助，注入 UART RX 字节。
- `bsp_uart_test_reset()`: host 测试辅助，清空 UART mock 队列。
- `bsp_uart_hal_rx_event()`: 将 HAL UART RX 事件映射到 BSP 总线回调。
- `bsp_uart_hal_tx_done()`: 将 HAL UART TX 完成回调映射到 BSP 总线状态。

### `bsp_usb_cdc.c`
- `bsp_usb_cdc_init()`: 初始化 USB CDC 桥或 host TX 缓冲。
- `bsp_usb_cdc_attach_rx()`: 注册 USB RX 回调。
- `bsp_usb_cdc_send()`: 发送 USB 字节，或写入 host TX 缓冲。
- `bsp_usb_cdc_on_rx()`: 将收到的 USB 字节送入注册回调。
- `bsp_usb_cdc_test_inject_rx()`: host 测试辅助，注入 USB RX 字节。
- `bsp_usb_cdc_test_tx_size()`: host 测试辅助，读取 TX 缓冲长度。
- `bsp_usb_cdc_test_read_tx()`: host 测试辅助，复制 TX 缓冲内容。
- `bsp_usb_cdc_test_reset()`: host 测试辅助，清空 USB mock TX 缓冲。

## `App/common`

### `log.c`
- `stdio_backend()`: 可选 host stdout 日志后端。
- `lookup_tag_level()`: 查找 tag 专用日志等级或全局默认等级。
- `log_init()`: 重置日志后端和等级状态。
- `log_register_backend()`: 添加日志后端回调。
- `log_set_global_level()`: 设置全局日志阈值。
- `log_get_global_level()`: 读取全局日志阈值。
- `log_set_level()`: 设置指定 tag 的日志阈值。
- `log_emit()`: 格式化并分发一行日志。
- `log_last_line()`: 返回最近格式化的一行日志。

## `App/control/arm`

### `arm_control.c`
- `enforce_t4_coupling()`: 按 J3/J4 固定和关系修正 4 号关节位置、速度和加速度。
- `init_planned_home_sample()`: 初始化 dry-run 阶段使用的 planned home sample。
- `reset_feedback()`: 清空协议 feedback 并回到 `IDLE`。
- `sample_to_angles()`: 将轨迹 sample 转换为几何角和电机角。
- `angles_to_positions()`: 将关节角转换为轨迹生成器使用的四轴位置数组。
- `reset_settle_tracking()`: 重置 settling 计时、误差和 reached 状态。
- `current_reference_angles()`: 优先取真实反馈作为当前参考角，反馈缺失时回退 planned sample。
- `target_delta_inside_fine_window()`: 判断新目标相对参考姿态是否可直接小范围微调。
- `target_is_inside_fine_window()`: 用当前反馈/规划参考判断目标是否可进入 fine tracking。
- `joint_targets_match()`: 以前三轴为准判断两个关节目标是否落在更新死区内。
- `normalize_theta1_to_reference()`: 将 1 号轴目标角归一到离当前参考最近的等效角。
- `make_safe_joint_target()`: 构造三段式使用的安全收臂/旋转姿态。
- `max_joint_error_to_target()`: 计算真实反馈相对当前阶段目标的最大关节误差。
- `max_measured_speed()`: 计算真实反馈前三轴最大速度。
- `settle_tolerance_for_stage()`: 根据当前安全阶段选择 settling 位置阈值。
- `bind_arm_motors_from_registry()`: 从 `motor_registry` 绑定 ARM_J1-J4 达妙电机句柄。
- `motor_state_is_fresh()`: 根据 online、rx 计数和反馈时间戳判断单个电机反馈是否新鲜。
- `read_measured_angles()`: 读取 J1-J4 电机状态，按 J2/J3 方向映射生成 measured angles/sample。
- `refresh_measured_feedback()`: 刷新真实末端位姿和 feedback 新鲜度诊断。
- `update_pose_feedback_from_sample()`: 根据指定 sample 计算末端位姿；planned sample 同时更新重力补偿力矩，measured sample 更新真实 feedback。
- `update_feedback()`: 优先使用 measured sample 生成 `0x86 ARM_FEEDBACK`；反馈缺失时回退 planned sample。
- `set_stage_target()`: 设置当前三段式阶段目标并标记待启动轨迹。
- `start_safe_retract_stage()`: 从当前参考角启动安全收臂阶段，并退出 fine tracking。
- `start_safe_rotate_stage()`: 使用最新 active target 的底座角启动安全旋转阶段。
- `start_safe_extend_stage()`: 使用最新 active target 启动展开阶段。
- `plan_final_target()`: 根据最新反馈判断直接微调或进入 `RETRACT` 阶段。
- `advance_safe_stage()`: 在阶段 reached 后推进到 `ROTATE_BASE`、`EXTEND` 或最终 `REACHED`。
- `recover_from_settle_timeout()`: 按旧工程策略处理安全阶段/fine tracking 超时恢复。
- `update_settle_state()`: 根据轨迹状态、真实反馈误差、速度和 fine tracking 阈值判定 `MOVING/REACHED/ERROR`。
- `stop_motion_and_target()`: 停止轨迹并清空当前目标。
- `start_pending_target()`: 用最新 IK 目标启动五次轨迹；运动中重规划会先采样当前轨迹，输出 gate 打开时要求 J1-J4 反馈新鲜并以真实反馈作为轨迹起点。
- `command_joint()`: 调用单个达妙电机 vtable 的 `set_position()`。
- `command_arm_motors()`: 在输出 gate 打开后发送 J1-J4 位置、速度、增益和重力前馈命令。
- `command_idle_gravity_hold_motors()`: 无目标时使用当前反馈角和重力补偿保持机械臂，对齐原工程 gravity 模式。
- `arm_control_init()`: 初始化机械臂控制门面、planned home sample、轨迹生成器、feedback 状态和输出 gate 默认值；MCU 固件默认启用控制器。
- `arm_control_set_enabled()`: 设置机械臂硬运行开关；正常 `IDLE/NAV/ARM` 不由 mode 调用。
- `arm_control_set_motor_output_enabled()`: 显式打开或关闭达妙输出 gate；默认由 `APP_ARM_MOTOR_OUTPUT_DEFAULT_ENABLE` 控制。
- `arm_control_set_gravity_mode()`: 停止当前目标/轨迹并回到当前位置重力保持，不关泵。
- `arm_control_set_target()`: 校验 `ARM_TARGET`，运行 IK，选择底座最近等效角，并按当前阶段执行目标更新、重触发或 fine tracking 重规划。
- `arm_control_set_pump()`: 缓存合法气泵命令，驱动主气泵 `PD11`，并同步空载/带载重力补偿质量。
- `arm_control_tick()`: 运行一个机械臂控制周期；刷新达妙反馈、在输出 gate 打开时运行达妙自动 enable、启动/更新五次轨迹、刷新 feedback，并发送达妙命令。
- `arm_control_get_feedback()`: 复制当前 `0x86 ARM_FEEDBACK` payload。
- `arm_control_get_status()`: 复制 host test 和诊断用的机械臂状态。

### `arm_kinematics.c`
- `arm_clampf()`: 限幅 float。
- `wrap_to_2pi()`: 将角度包裹到 `[0, 2pi)`。
- `normalize_theta1_motor()`: 归一化 1 号底座电机角。
- `arm_kinematics_compute_t4_from_t3()`: 根据 J3/J4 固定和关系计算 4 号关节角。
- `arm_kinematics_forward()`: 根据几何关节角计算末端 `xyz` 位姿。
- `arm_kinematics_inverse()`: 根据末端 `xyz` 目标求 J1-J4 目标角，并检查 J2/J3 实测物理限位。

### `arm_motion.c`
- `motion_clampf()`: 限幅 float。
- `calculate_coefficients()`: 根据起点、目标和时长计算五次多项式系数。
- `evaluate()`: 在指定轨迹时间采样位置、速度和加速度。
- `initial_duration()`: 根据每轴速度/加速度限制估算同步轨迹初始时长。
- `trajectory_within_limits()`: 离散采样检查轨迹是否超过速度/加速度限制。
- `arm_motion_init()`: 初始化轨迹生成器和默认运动限制。
- `arm_motion_set_limits()`: 更新有效的速度、加速度和时长限制。
- `arm_motion_start()`: 启动四关节同步五次轨迹。
- `arm_motion_update()`: 根据 `now_ms` 更新并返回当前轨迹 sample。
- `arm_motion_stop()`: 停止轨迹并保持指定 sample。
- `arm_motion_get_state()`: 返回当前轨迹状态。
- `arm_motion_get_sample()`: 复制当前轨迹 sample。
- `arm_motion_get_duration()`: 返回当前轨迹总时长。
- `arm_motion_get_progress()`: 返回当前轨迹进度 `[0, 1]`。

### `arm_gravity_comp.c`
- `update_mass_transition()`: 将 active end mass 平滑逼近当前 payload 目标质量。
- `arm_gravity_comp_init()`: 初始化 payload 状态和末端 active/target 质量。
- `arm_gravity_comp_set_payload_state()`: 切换空载/带载目标质量。
- `arm_gravity_comp_get_payload_state()`: 返回当前 payload 状态。
- `arm_gravity_comp_get_active_end_mass()`: 返回当前用于计算的末端 active 质量。
- `arm_gravity_comp_get_target_end_mass()`: 返回当前 payload 对应的目标末端质量。
- `arm_gravity_comp_calculate()`: 根据 J2/J3/J4 几何角计算 tau2/tau3/tau4 重力补偿力矩。

## `App/control/attitude`

### `attitude_estimator.c`
- `attitude_estimator_init()`: 重置 yaw 和估计器状态。
- `attitude_estimator_update()`: 积分 gyro Z 得到 yaw 估计。
- `attitude_estimator_get_yaw()`: 返回当前 yaw。
- `attitude_estimator_reset_yaw()`: 将 yaw 重置为零。
- `attitude_estimator_get_state()`: 返回估计器状态指针。

### `steer_controller.c`
- `wrap_pi()`: 将角度误差包裹到 `[-pi, pi]`。
- `steer_controller_init()`: 重置转向 PID 状态。
- `steer_controller_set_mode()`: 修改转向模式。
- `steer_controller_set_target_yaw()`: 设置 yaw 目标。
- `steer_controller_update()`: 根据 yaw 误差计算 yaw-rate 命令。
- `steer_controller_get_cfg()`: 返回转向增益/配置。

## `App/control/chassis`

### `chassis_control.c`
- `controller_height_from_body()`: 将机身高度转换为 IK 使用的高度约定。
- `apply_controller_height()`: 将 gait 机身高度同步到腿控制器。
- `validate_trot_params()`: 检查 gait 参数是否合法。
- `request_gait()`: 封装 gait-machine 的 set/request 行为。
- `is_offline()`: 判断当前在线/离线状态。
- `online_decide()`: 在线时根据 planner 输出选择 stand 或 trot。
- `apply_plan_wheel_speed()`: 只给支撑相腿应用 planner 轮速。
- `apply_rl_single_leg_debug()`: 可选右后单腿调试输出路径。
- `update_steering()`: 更新 yaw 估计并计算实际使用的 `wz`。
- `offline_decide()`: 选择保守离线行为。
- `go_pre_calibration_ready()`: MCU 上 GO 闭环前的反馈采集窗口。
- `flush_motor_outputs()`: MCU 构建中发送已暂存的电机命令。
- `safe_input_or_zero()`: 复制输入；输入为空时生成零命令快照。
- `make_plan_command()`: 根据输入和实际 `wz` 构造 planner 命令。
- `update_plan_from_input()`: 运行 chassis planner 并保存最新 plan。
- `update_online_state()`: 更新缓存的在线/离线状态。
- `decide_gait_for_link_state()`: 根据在线/离线状态应用步态策略。
- `apply_gait_output_to_motors()`: 更新 gait 输出并派发腿部命令。
- `chassis_control_init()`: 初始化 gait、leg、planner 和 steering 状态。
- `chassis_control_tick()`: 运行一个完整的命令到电机控制周期。
- `chassis_clampf()`: 底盘控制内部 float 限幅。
- `attitude_comp_valid_or_default()`: 读取姿态补偿尺寸/限幅参数，非法时使用默认值。
- `attitude_comp_param_or_default()`: 读取允许为 0 的姿态补偿增益/滤波参数。
- `attitude_comp_scale()`: 读取姿态补偿比例，非法时回退为 0。
- `attitude_comp_apply_deadband()`: 对 roll/pitch 姿态误差应用死区。
- `attitude_comp_limit_moment()`: 限制姿态虚拟力矩。
- `attitude_comp_filter_update()`: 对姿态虚拟力矩做一阶滤波。
- `attitude_comp_write_leg_balance()`: 将姿态虚拟力矩写入 `g_leg_gravity_comp.balance_*`。
- `attitude_comp_clear_debug()`: 清空姿态补偿诊断输出。
- `apply_attitude_compensation()`: 将 roll/pitch 姿态误差转换为支撑腿 `tau_ff` 前馈补偿。
- `chassis_control_set_mode()`: 设置底盘模式。
- `chassis_control_get_mode()`: 读取底盘模式。
- `chassis_control_get_gait_active()`: 读取当前激活步态枚举。
- `chassis_control_set_online_timeout_ms()`: 设置心跳超时。
- `chassis_control_get_online_timeout_ms()`: 读取心跳超时。
- `chassis_control_play_script()`: 启动脚本步态播放。
- `chassis_control_stop_script()`: 停止脚本步态。
- `chassis_control_start_stand()`: 启动 stand 步态。
- `chassis_control_set_trot_params()`: 更新 trot 参数。
- `chassis_control_get_trot_params()`: 读取 trot 参数。
- `chassis_control_start_trot()`: 启动 trot 步态。
- `chassis_control_active_gait_name()`: 返回当前 gait 名称。
- `chassis_control_reset_yaw()`: 重置 yaw 估计。
- `chassis_control_get_yaw()`: 读取 yaw 估计。
- `chassis_control_get_effective_wz()`: 读取上一拍实际 yaw-rate。
- `chassis_control_get_status()`: 复制诊断状态快照。

### `chassis_planner.c`
- `clampf_local()`: 限幅 float。
- `chassis_planner_init()`: planner 初始化保留钩子。
- `planner_limit_wheel()`: 应用配置的轮速上限。
- `planner_is_low_speed_turn()`: 判断低速 yaw 转向场景。
- `planner_configured_half_track()`: 读取腿/轮接触点到机体中心线的横向距离，非法时使用默认 `0.15 m`。
- `planner_configured_max_leg_step()`: 读取单腿步长限幅，非法时使用默认值并限制到全局最大步长内。
- `planner_leg_y_m()`: 返回单腿横向位置；左侧为 `+half_track`，右侧为 `-half_track`。
- `planner_leg_local_vx()`: 按 `v_leg_x = vx - wz * y_leg` 计算单腿局部前后速度。
- `planner_apply_turn_gait()`: 覆盖转向场景的 gait 参数。
- `planner_safe_duty()`: 清理 duty，避免步长/周期计算除以零。
- `planner_configured_slow_period()`: 读取低速步态周期配置，非法时使用默认值。
- `planner_configured_fast_period()`: 读取高速步态周期配置，非法时使用默认值。
- `planner_configured_fast_speed()`: 读取进入高速周期的速度阈值，非法时使用默认值。
- `planner_period_from_speed()`: 根据运动速度在低速周期和高速周期之间插值。
- `planner_apply_stride_schedule()`: 轮足行走周期调度；周期只小范围变化，速度主要进入轮速和每腿步长。
- `planner_step_from_vx()`: 将单腿局部前后速度转换成单腿步长，并应用步长限幅。
- `planner_mean_step()`: 计算四腿步长均值，写入 `step_length_m` 作为诊断摘要。
- `planner_right_left_turn_step()`: 计算右侧均值和左侧均值的半差，写入 `turn_step_m` 作为诊断摘要。
- `planner_fill_leg_steps()`: 为四条腿写入 `leg_step_length_m[]`，并刷新摘要字段。
- `chassis_planner_update()`: 将 `vx/vy/wz` 命令转换为 moving、每腿步长和每轮局部滚动速度；`vy` 只参与 moving 判断。

## `App/control/gait`

### `gait_if.c`
- `gait_wrap01()`: 将相位包裹到 `[0, 1)`。

### `gait_machine.c`
- `lerp()`: 线性插值辅助。
- `blend_outputs()`: 混合两个 gait 输出。
- `gait_machine_init()`: 清空 gait machine 状态。
- `gait_machine_set()`: 立即设置当前 gait。
- `gait_machine_request()`: 请求目标 gait，可带过渡混合。
- `gait_machine_update()`: 更新当前 gait 或过渡状态。
- `gait_machine_state()`: 读取 gait machine 状态。

### `gait_params.c`
- `GAIT_PARAMS_TROT_DEFAULT`: 默认 trot 参数。
- `GAIT_PARAMS_STAND_DEFAULT`: 默认 stand 参数。

### `gait_stand.c`
- `stand_init()`: stand gait 初始化钩子。
- `stand_set_param()`: 保存 stand gait 参数。
- `stand_update()`: 输出零足端位移、支撑相和零轮速。
- `stand_exit()`: stand gait 退出钩子。
- `stand_name()`: 返回 `"stand"`。
- `gait_stand_create()`: 返回 stand gait 单例。

### `gait_trot.c`
- `trot_init()`: 重置 trot 相位。
- `trot_set_param()`: 保存已校验的 trot 参数。
- `gait_trot_foot_traj()`: 计算支撑/摆动足端轨迹。
- `trot_leg_side_sign()`: 返回单腿用于 yaw 步态转向的左右侧符号。
- `trot_has_per_leg_steps()`: 判断当前参数是否启用了每腿步长；允许某条腿的步长为 0。
- `trot_leg_step_length()`: 优先读取 `leg_step_length_m[]`；没有每腿步长时回退到 `step_length_m ± turn_step_m`。
- `trot_update()`: 推进 trot 相位并写入足端目标。
- `trot_exit()`: trot gait 退出钩子。
- `trot_name()`: 返回 `"trot"`。
- `gait_trot_create()`: 返回 trot gait 单例。

## `App/control/kinematics`

### `leg_ik.c`
- `leg_ik_clampf()`: 限幅 float。
- `leg_ik_get_joint_limit()`: 根据腿型获取关节限位。
- `leg_ik_update_best()`: 选择最接近参考的合法关节解。
- `leg_ik_constrain_joint_target()`: 施加关节和大腿/小腿相对关系限位。
- `leg_ik_solve()`: 求解单腿逆运动学。
- `leg_fk_solve()`: 求解单腿正运动学。
- `leg_ik_solve_all()`: 将四条腿足端目标转换为髋/膝关节目标。

### `leg_params.c`
- `LEG_DIM_DEFAULT`: 默认腿部几何和质量参数。

## `App/control/leg`

### `leg_controller.c`
- `leg_controller_init()`: 清空控制器状态。
- `leg_controller_bind_from_registry()`: 按 logical id 绑定髋/膝/轮电机句柄。
- `leg_controller_set_stand_height()`: 设置 IK 站立高度。
- `leg_controller_set_output_options()`: 配置启用腿、关节输出和轮子输出。
- `try_set_pos()`: 电机支持位置接口时发送位置命令。
- `try_set_wheel_mit()`: 发送 M3508 轮子 MIT 命令和安全限幅。
- `wheel_mit_reset_refs()`: 清空轮子 MIT 积分参考位置。
- `wheel_mit_read_cfg()`: 读取轮子 MIT 默认/调试增益和安全限幅。
- `wheel_mit_limit_dt()`: 限制轮子 MIT 积分步长。
- `wheel_mit_latch_ref_if_needed()`: MIT 使用前锁存轮子参考角。
- `wheel_mit_command()`: 发送一个轮子 MIT 位置/速度命令。
- `wheel_mit_apply_stance()`: 支撑相积分并发送轮子 MIT 命令。
- `wheel_mit_apply_swing()`: 摆动相使用 MIT 保持当前轮角。
- `try_set_wheel()`: 对轮毂电机发送默认 MIT 命令。
- `leg_output_enabled()`: 检查腿输出 mask 是否启用。
- `leg_has_required_actuators()`: 检查启用的关节/轮子句柄是否存在。
- `send_joint_targets()`: 发送髋/膝位置目标。
- `send_wheel_target()`: 发送一个轮子的 MIT 目标。
- `send_leg_targets()`: 派发单腿所有启用执行器命令。
- `leg_controller_apply_dt()`: 运行 IK 并发送关节/轮子命令。
- `leg_controller_apply()`: 使用默认 2 ms 周期调用控制器。

## `App/control/script`

### `gait_script.c`
- `gs_init()`: 初始化或重启脚本播放器。
- `gs_set_param()`: 保存脚本 gait 包装参数。
- `gs_update()`: 更新脚本播放器，或输出安全 stand-hold。
- `gs_exit()`: script gait 退出钩子。
- `gs_name()`: 返回 `"script"`。
- `gait_script_create()`: 返回 script gait 单例。
- `gait_script_set_script()`: 将脚本加载到播放器。
- `gait_script_rewind()`: 将当前脚本回到起点。
- `gait_script_state()`: 读取播放器状态。

### `script_builtin.c`
- `script_builtin_find()`: 按名称查找内置脚本。
- `SCRIPT_BUILTIN_STAND_HOLD`: 静态 stand-hold 脚本。
- `SCRIPT_BUILTIN_WAVE_UP_DOWN`: 静态同步抬腿脚本。
- `SCRIPT_BUILTIN_TROT_STEP`: 静态四关键帧 trot 脚本。

### `script_player.c`
- `lerp()`: 线性插值辅助。
- `normalize_legacy_foot_fields()`: 将旧脚本 hip/knee foot 数据转换为显式足端字段。
- `blend_kf()`: 插值两个脚本关键帧。
- `script_sample()`: 在任意时间采样脚本输出。
- `script_player_init()`: 清空播放器状态。
- `script_player_load()`: 加载脚本并开始运行。
- `script_player_reset()`: 将已加载脚本回到起点。
- `script_player_update()`: 推进时间并采样当前脚本。
- `script_player_state()`: 读取播放器状态。

## `App/device`

### `arm_pump.c`
- `aux_to_gpio()`: 将气泵辅助通道枚举映射到 BSP GPIO 输出。
- `arm_pump_init()`: 初始化主气泵输出并默认关闭。
- `arm_pump_set()`: 打开或关闭主气泵。
- `arm_pump_is_enabled()`: 返回主气泵当前状态。
- `arm_pump_set_aux()`: 打开或关闭一个保留辅助通道。
- `arm_pump_aux_is_enabled()`: 返回保留辅助通道当前状态。

### `imu_bmi088.c`
- `make_i16()`: 将 LSB/MSB 合成为有符号 16 位值。
- `diag_reset()`: 清空 IMU 诊断状态。
- `diag_mark()`: 保存最近诊断阶段和状态。
- `acc_write_reg()`: 写加速度计寄存器。
- `acc_read_reg()`: 读加速度计寄存器。
- `acc_read_multi()`: 连续读取多个加速度计寄存器。
- `gyro_write_reg()`: 写陀螺仪寄存器。
- `gyro_read_reg()`: 读陀螺仪寄存器。
- `gyro_read_multi()`: 连续读取多个陀螺仪寄存器。
- `accel_init()`: 初始化 BMI088 加速度计。
- `gyro_init()`: 初始化 BMI088 陀螺仪。
- `read_temperature()`: 读取并转换加速度计温度。
- `imu_bmi088_init()`: 初始化 BMI088 或 host mock。
- `imu_bmi088_read()`: 读取加速度、陀螺仪和温度样本。
- `imu_bmi088_set_accel_range()`: 修改加速度计量程。
- `imu_bmi088_is_ready()`: 返回 IMU 是否就绪。
- `imu_bmi088_get_diag()`: 复制诊断状态。
- `imu_bmi088_set_gyro_range()`: 修改陀螺仪量程。

### `motor_go.c`
- `go_crc16()`: 计算 RIS 帧 CRC。
- `go_cfg()`: 查找电机配置。
- `go_cfg_sign()`: 返回配置的关节方向。
- `go_cfg_gear()`: 返回配置的减速比。
- `go_motor_pos_to_joint()`: 将电机侧位置转换为关节侧位置。
- `go_joint_pos_to_motor()`: 将关节侧位置转换为电机侧位置。
- `go_motor_vel_to_joint()`: 将电机侧速度转换为关节侧速度。
- `go_joint_vel_to_motor()`: 将关节侧速度转换为电机侧速度。
- `go_motor_tau_to_joint()`: 将电机侧力矩转换为关节侧力矩。
- `go_joint_tau_to_motor()`: 将关节侧力矩转换为电机侧力矩。
- `go_encode_cmd()`: 编码 RIS 命令帧。
- `go_decode_fbk()`: 解码 RIS 反馈帧。
- `go_set_current()`: 将电流请求映射成类似力矩的命令。
- `go_set_torque()`: 设置 GO 力矩命令。
- `go_set_position()`: 设置 GO 位置/速度/kp/kd/tau 命令。
- `go_set_velocity()`: 设置 GO 速度阻尼命令。
- `go_calibrate()`: 从最近反馈锁存零偏。
- `go_enable()`: 启用闭环模式。
- `go_disable()`: 命令零力矩模式。
- `go_reset_fault()`: 清除错误计数并进入零力矩模式。
- `go_feed_rx()`: 将解码后的 RX payload 写入状态。
- `go_find_index_by_logical_id()`: 按 logical motor id 查找 GO 实例。
- `go_init_ctx()`: 根据总线映射初始化一个 GO 驱动上下文。
- `go_init_dev()`: 初始化一个 GO 电机的 `motor_dev_t` 包装。
- `go_init_instance()`: 初始化并注册绑定一个 GO 实例。
- `motor_go_init_all()`: 创建 GO 实例并绑定 registry。
- `motor_go_uart_rx_cb()`: 将 UART 反馈路由到匹配的 GO 实例。
- `go_tx_bus_count()`: 返回 GO UART TX 总线数量。
- `go_send_one_on_bus()`: 编码并发送一个 GO 命令帧。
- `go_send_bus()`: 发送某条 UART 总线上的所有 GO 电机。
- `motor_go_send_all()`: 发送全部 GO 命令帧。
- `motor_go_calibrate_all()`: 根据反馈校准所有 GO 电机。
- `motor_go_debug_get_state()`: 复制一个 GO 电机的调试状态。

### `motor_m3508.c`
- `m3508_limit_current()`: 限幅 raw 电流命令。
- `m3508_power_limit()`: 根据功率估计限制电流。
- `m3508_rpm_to_rads()`: 将 rpm 转换为 rad/s。
- `m3508_encoder_to_rad()`: 将编码器计数转换为弧度。
- `m3508_raw_to_ampere()`: 将 raw 电流转换为安培。
- `m3508_ampere_to_raw()`: 将安培转换为 raw 电流。
- `m3508_ampere_to_raw_f()`: ampere-to-raw 的 float 版本。
- `m3508_cfg()`: 查找电机配置。
- `m3508_cfg_sign()`: 返回配置的关节方向。
- `m3508_clampf()`: 限幅 float。
- `m3508_output_torque_to_raw()`: 将输出轴力矩转换为电机 raw 电流。
- `m3508_update_angle()`: 维护多圈编码器状态。
- `m3508_set_current()`: 设置直接电流模式。
- `m3508_set_torque()`: 设置力矩模式。
- `m3508_set_position()`: 设置位置模式或 MIT 模式。
- `m3508_set_velocity()`: 设置速度模式。
- `m3508_enable()`: 标记电机启用并重置控制环。
- `m3508_disable()`: 清零命令并禁用电机。
- `m3508_reset_fault()`: 清除错误计数并启用电机。
- `m3508_feed_rx()`: 解码 C620 反馈帧。
- `m3508_mit_limit_tau()`: 限幅 MIT 力矩上限。
- `m3508_mit_limit_pos_err()`: 限幅 MIT 位置误差上限。
- `motor_m3508_trace_reset()`: 重置 trace 环形缓冲。
- `motor_m3508_trace_enable()`: 启用或禁用 trace 记录。
- `motor_m3508_set_mit_limits()`: 应用单次命令的 MIT 安全限幅。
- `m3508_trace_float_to_i16()`: 将 trace float 压缩为整数。
- `m3508_trace_record()`: 记录一个 trace 样本。
- `m3508_speed_pid_set_tunings()`: 更新速度 PID 增益。
- `m3508_speed_pid_decay_integral()`: 衰减 PID 积分状态。
- `m3508_speed_pid_update()`: 带抗积分饱和行为更新速度 PID。
- `m3508_slew_current()`: 限制电流命令变化率。
- `m3508_velocity_hold_target_rpm()`: 将零速保持误差转换为 rpm 目标。
- `m3508_apply_static_ff()`: 应用可选静摩擦前馈。
- `m3508_stop_ctx()`: 将一个上下文停止为零电流。
- `m3508_hold_ctx_mit()`: 使用 MIT 模式保持当前角度。
- `m3508_mit_ramp_update()`: 调试 MIT ramp 状态机。
- `motor_m3508_init_all()`: 创建 M3508 实例并绑定 registry。
- `motor_m3508_fdcan_rx_cb()`: 将 CAN 反馈路由到匹配的 M3508 实例。
- `m3508_apply_temperature_derate()`: 应用 M3508 过温半电流降额。
- `m3508_run_position_control()`: 运行位置到速度到电流的级联控制。
- `m3508_run_torque_control()`: 运行输出轴力矩前馈加电流校正。
- `m3508_run_mit_control()`: 运行轮子 MIT 阻抗控制和安全限幅。
- `m3508_run_current_control()`: 在直接电流模式下应用温度降额。
- `m3508_velocity_target_to_rpm()`: 将输出轴速度目标转换为转子 rpm。
- `m3508_pid_needs_tuning_update()`: 检测速度 PID 增益是否变化。
- `m3508_prepare_velocity_target_rpm()`: 选择移动或零速保持的速度目标。
- `m3508_run_velocity_control()`: 运行速度 PI、前馈、斜率限制和功率限制。
- `m3508_run_control_mode()`: 将一个电机上下文分发到当前控制模式。
- `m3508_pack_current_slot()`: 将 raw 电流命令打包到 DJI CAN slot。
- `m3508_log_tx_failure()`: 对 `0x200` 帧发送失败做限频日志。
- `m3508_log_probe_tx_failure()`: 对 `0x1FF` 探测帧发送失败做限频日志。
- `m3508_send_current_frame()`: 发送当前总线的 `0x200` 电流帧。
- `m3508_send_probe_frame()`: 发送零电流 `0x1FF` 探测帧。
- `m3508_run_bus_controls()`: 运行分配到一条 FDCAN 总线上的所有电机。
- `motor_m3508_send_all()`: 运行控制环并发送 CAN 电流帧。

### `motor_damiao.c`
- `damiao_clampf()`: 限幅 float。
- `damiao_float_to_uint()`: 将物理量按达妙协议量程压缩为无符号整数。
- `damiao_uint_to_centered_float()`: 将反馈 raw 值按中心零点解码为物理量。
- `damiao_model_ranges()`: 根据 DM4310/DM4340 选择速度和力矩量程。
- `damiao_model_v_abs()`: 返回模型反馈速度绝对量程。
- `damiao_model_t_abs()`: 返回模型反馈力矩绝对量程。
- `motor_damiao_pack_mit()`: 打包达妙 MIT 位置、速度、刚度、阻尼和前馈力矩帧。
- `motor_damiao_parse_feedback()`: 解码达妙 8 字节反馈并更新 `motor_state_t` 和驱动上下文。
- `damiao_send()`: 通过 `bsp_fdcan_send()` 在目标 FDCAN 总线上发送一帧达妙命令。
- `damiao_set_position()`: 通过 MIT 帧发送位置/速度/增益/力矩命令。
- `damiao_set_torque()`: 以当前角度为保持点发送力矩前馈命令。
- `damiao_set_velocity()`: 以当前角度为保持点发送速度命令。
- `damiao_enable()`: 发送达妙 enable 特殊帧。
- `damiao_disable()`: 发送达妙 disable 特殊帧并清除在线状态。
- `damiao_reset_fault()`: 清除错误计数并发送 enable 特殊帧。
- `damiao_feed_rx()`: 将 RX payload 交给达妙反馈解析器。
- `damiao_feedback_needs_enable()`: 判断无反馈、未使能或长时间无反馈的达妙关节是否需要重发 enable。
- `motor_damiao_process()`: 按 200ms 周期和 500ms 单电机间隔运行达妙自动 enable 恢复。
- `motor_damiao_auto_enable_count()`: 返回自动 enable 成功发送计数。
- `damiao_index_for_can_id()`: 按 FDCAN bus 和 CAN ID 查找 J1-J4 驱动实例。
- `motor_damiao_fdcan_rx_cb()`: 将 FDCAN3 反馈路由到匹配的达妙电机实例。
- `motor_damiao_init_all()`: 创建 J1-J4 达妙实例、绑定 registry，并注册 FDCAN3 RX 回调。

### `motor_registry.c`
- `motor_registry_init()`: 清空 logical motor registry。
- `motor_get()`: 按 logical id 返回电机句柄。
- `motor_get_cfg()`: 按 logical id 返回只读电机配置。
- `motor_registry_count()`: 返回 registry 槽位数量。
- `motor_registry_bind()`: 将 logical motor id 绑定到运行期设备。

## `App/service`

### `pid.c`
- `clamp_f()`: 限幅 float。
- `sort_limits()`: 排列 high/low 限幅指针。
- `normalize_dt()`: 提供安全的 timestep fallback。
- `dt_from_ms()`: 根据毫秒时间戳计算 timestep。
- `app_pid_reset()`: 清空 PID 动态状态。
- `app_pid_set_tunings()`: 设置 PID 增益。
- `app_pid_set_output_limits()`: 设置输出限幅。
- `app_pid_set_integral_limits()`: 设置积分限幅。
- `app_pid_update_dt()`: 使用显式 timestep 更新位置式 PID。
- `app_pid_update_ms()`: 根据时间戳更新位置式 PID。
- `app_pid_update_inc_dt()`: 使用显式 timestep 更新增量式 PID。
- `app_pid_update_inc_ms()`: 根据时间戳更新增量式 PID。

### `proto_dispatch.c`
- `proto_dispatcher_init()`: 安装功能号分发表。
- `proto_dispatch_on_frame()`: 调用匹配的协议 handler，或记录 miss。

### `proto_frame.c`
- `proto_frame_init()`: 初始化解析器状态和回调。
- `proto_frame_reset()`: 清空解析器状态。
- `fail_and_reset()`: 统计坏帧并重置解析器。
- `proto_frame_feed()`: 增量解析带帧格式的字节流。
- `proto_frame_build()`: 构造一帧带校验的协议包。
