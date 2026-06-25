# App 函数索引

本文档记录 `remake_original` 分支当前 App 层重要文件和函数。它是查表用文档：改代码前先用它定位函数，再去读源码。

## `App/app`

### `app_init.c`
- `app_init()`: 初始化日志、时间、FDCAN、UART、USB、SPI、电机注册表、GO 电机、M3508 电机和 BMI088；这是板级启动入口。

### `app_tasks.c`
- `app_tasks_create()`: 初始化通信和底盘控制，并在 MCU 构建中启动 FreeRTOS 任务。

### `task_chassis.c`
- `read_chassis_input()`: 将 `task_comm` 缓存的命令复制为 `chassis_control_input_t`。
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

### `task_comm.c`
- `mark_valid_rx()`: 更新最近一次有效接收时间戳。
- `cache_chassis_base_command()`: 缓存底盘命令中的 `vx/vy/wz`。
- `cache_chassis_steering_extension()`: 缓存可选 target-yaw 转向扩展字段。
- `mark_chassis_command_rx()`: 增加底盘命令序号并刷新心跳时间。
- `handle_chassis()`: 解码 `0x10` 底盘速度命令 payload。
- `gait_params_from_payload()`: 将协议中的 gait payload 转换为 `gait_params_t`。
- `handle_gait()`: 解码 `0x12` stand/trot/set-params 命令。
- `handle_mit_cmd()`: 解码 `0x13` 单电机 MIT 调试命令。
- `on_usb_rx()`: 将 USB 字节流送入协议帧解析器。
- `task_comm_init()`: 重置命令状态并安装协议分发表。
- `task_comm_good_cnt()`: 返回有效帧计数。
- `task_comm_bad_cnt()`: 返回无效帧计数。
- `task_comm_dispatch_hit()`: 返回命中分发表的计数。
- `task_comm_dispatch_miss()`: 返回未命中分发表的计数。
- `task_comm_last_rx_ms()`: 返回最近一次有效命令时间戳。
- `task_comm_get_chassis()`: 复制最近一次底盘命令。
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

### `bsp_fdcan.c`
- `fdcan_dwt_enable()`: 在 MCU 上启用 DWT cycle counter。
- `fdcan_cycles_per_us()`: 将 MCU 时钟换算为 cycles/us。
- `fdcan_wait_free_level()`: 等待 FDCAN TX FIFO 空位。
- `fdcan_warn_throttled()`: 对 FDCAN 警告日志做限频。
- `bsp_fdcan_init()`: 初始化 host mock 队列或 MCU FDCAN 滤波/中断。
- `bsp_fdcan_attach_rx()`: 注册某条总线的 RX 回调。
- `bsp_fdcan_send()`: 发送一帧 CAN，或在 host 中压入 TX 队列。
- `bsp_fdcan_hal_rxfifo0_cb()`: 将 HAL RX FIFO 回调桥接到 BSP 回调。
- `bsp_fdcan_hal_error_cb()`: 记录 FDCAN 错误中断状态。
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
- `signf_local()`: 返回 float 符号。
- `chassis_planner_init()`: planner 初始化保留钩子。
- `planner_limit_wheel()`: 应用配置的轮速上限。
- `planner_is_low_speed_turn()`: 判断低速 yaw 转向场景。
- `planner_configured_turn_step_gain()`: 读取 yaw 到步态转向步长差的比例，非法时使用默认值。
- `planner_configured_turn_step_limit()`: 读取步态转向步长差限幅，非法时使用默认值。
- `planner_limit_turn_step_for_base()`: 根据共同前进步长限制左右侧合成步长。
- `planner_turn_step()`: 将 `wz` 转换成 `gait_params.turn_step_m`。
- `planner_apply_turn_gait()`: 覆盖转向场景的 gait 参数。
- `chassis_planner_update()`: 将命令转换为 gait 参数和共同前进轮速；yaw 转向写入 `turn_step_m`。

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
- `trot_leg_step_length()`: 合成单腿实际步长 `step_length_m ± turn_step_m`。
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
