# App 函数索引

本文档记录当前 App/Control/Device/BSP/Service 层的重要函数。它是查表用文档：改代码前先用它定位函数，再去读源码。

## 启动入口

### `Core/Src/main.c`
- `main()`: HAL、时钟和外设初始化；默认调用 `app_init()` 后启动 FreeRTOS。
- `MX_FREERTOS_Init()`: 由 CubeMX 生成并在 `main()` 中调用；实际定义在 `Core/Src/freertos.c`。

### `Core/Src/freertos.c`
- `MX_FREERTOS_Init()`: 创建 USB `defaultTask`，再调用 `app_tasks_create()`。
- `StartDefaultTask()`: 初始化 USB device 并保持 1 ms delay loop。

## `App/app`

### `app_init.c`
- `app_init()`: 初始化 log、time、FDCAN/UART/USB/SPI/GPIO、气泵、motor registry、GO、M3508、达妙和 BMI088。

### `app_tasks.c`
- `vApplicationStackOverflowHook()`: 记录 RTOS stack overflow 诊断并停机。
- `app_freertos_assert_failed()`: 记录 `configASSERT` 诊断并停机。
- `app_tasks_create()`: 初始化 `task_comm/task_chassis/task_arm`，MCU 上创建 `t_log/t_safety/t_comm/t_chassis/t_arm`。

### `task_comm.c`
- `mark_valid_rx()`: 更新最近一次有效接收时间戳。
- `cache_chassis_base_command()`: 缓存 `vx/vy/wz`。
- `cache_chassis_steering_extension()`: 兼容 17B 扩展帧中的 `target_yaw/steer_mode`。
- `handle_chassis()`: 解码 `0x10 CHASSIS_CMD`，接受 12B/17B。
- `gait_params_from_payload()`: 将协议 gait payload 转成 `gait_params_t`。
- `handle_gait()`: 解码 `0x12 GAIT_CMD`，支持 stand/trot/walk 和参数设置。
- `handle_mit_cmd()`: 解码 `0x13 MIT_CMD`；拒绝 ARM_J1-J6 旁路写入。
- `handle_arm_target()`: 解码和预校验 `0x11 ARM_TARGET`。
- `handle_arm_pump()`: 解码 `0x14 ARM_PUMP`。
- `handle_mode_cmd()`: 解码 `0x15 MODE_CMD`，同步 ESTOP/ERROR 到 safety。
- `handle_wheel_test()`: 解码 `0x16 WHEEL_TEST`，清普通底盘速度并更新独立轮驱测试。
- `handle_trot_test_config()`: 解码 `0x18 TROT_TEST_CONFIG`，更新模式 3 独立步态参数。
- `on_usb_rx()`: USB RX 回调，喂给 `proto_frame_feed()`。
- `task_comm_init()`: 重置缓存、安装分发表、注册 USB RX 回调。
- `task_comm_good_cnt()` / `task_comm_bad_cnt()`: 返回协议 parser 统计。
- `task_comm_dispatch_hit()` / `task_comm_dispatch_miss()`: 返回 dispatcher 统计。
- `task_comm_last_rx_ms()`: 返回最近有效 RX 时间。
- `task_comm_get_chassis()`: 复制底盘命令缓存。
- `task_comm_get_arm_target()`: 复制机械臂目标缓存。
- `task_comm_get_arm_pump()`: 复制气泵命令缓存。
- `task_comm_get_mode_cmd()`: 复制 mode 缓存。
- `task_comm_send_arm_feedback()`: 封装并发送 `0x86 ARM_FEEDBACK`。
- `task_comm_send_trot_test_diag()`: 封装模式 3 的 `0x8A TROT_TEST_DIAG`。
- `build_state_payload()`: 构造 `0x80 STATE` payload。
- `send_proto_payload()`: 封装协议帧并通过 USB CDC 发送。
- `send_state_frame()`: 发送整机状态。
- `build_motor_payload()`: 从 registry 构造单电机状态。
- `send_motor_frame()`: 轮询发送 `0x81 MOTOR_STATE`。
- `task_comm_entry()`: MCU 上行遥测任务循环。

### `task_chassis.c`
- `chassis_mode_allows_motion()`: mode gate；无 mode 帧允许旧调试，收到 mode 后仅 NAV 允许底盘运动。
- `read_chassis_input()`: 复制底盘缓存并按 mode gate 清零速度、target yaw 和 steer mode。
- `task_chassis_init()`: 初始化底盘控制模块。
- `task_chassis_set_mode()` / `task_chassis_get_mode()`: 读写底盘控制模式。
- `task_chassis_get_gait_active()`: 读取当前 gait。
- `task_chassis_set_online_timeout_ms()` / `task_chassis_get_online_timeout_ms()`: 配置心跳超时。
- `task_chassis_play_script()` / `task_chassis_stop_script()`: 启停脚本步态。
- `task_chassis_start_stand()`: 手动请求 stand。
- `task_chassis_set_trot_params()` / `task_chassis_get_trot_params()` / `task_chassis_start_trot()`: trot 参数和启动。
- `task_chassis_set_walk_params()` / `task_chassis_get_walk_params()` / `task_chassis_start_walk()`: walk 参数和启动。
- `task_chassis_active_gait_name()`: 返回当前 gait 名称。
- `task_chassis_reset_yaw()` / `task_chassis_get_yaw()` / `task_chassis_get_effective_wz()`: yaw 诊断入口。
- `task_chassis_step_for_test()`: host/任务共用的单周期入口。
- `task_chassis_entry()`: MCU 500 Hz 底盘任务；急停时跳过控制。

### `task_arm.c`
- `nearest_equivalent_angle_rad()`: 将等待 J1 角归一到离当前反馈最近的一圈。
- `selected_box_position()` / `selected_box_j1_motor_deg()`: 读取现场选择的第一/第二箱等待角。
- `reset_fixed_hold()`: 重置固定姿态状态机并切重力模式。
- `command_fixed_pose()`: 生成当前固定 profile 的关节目标并调用内部 hold target。
- `staged_profile_ready()`: 判断分阶段等待姿态是否到位。
- `process_power_on_fixed_grasp_test()`: 编译期开启时处理上电固定抓取测试。
- `advance_staged_profile()`: 推进第一箱/第二箱等待 profile。
- `process_fixed_hold()`: 固定姿态等待、移动、保持、错误重试主逻辑。
- `host_has_new_valid_grasp_target()`: 只有新的合法 GRASP 可以释放等待姿态。
- `release_fixed_pose_to_host()`: 将固定姿态状态切到上位机接管。
- `current_robot_mode()`: 读取 mode 缓存，无 mode 时视为 IDLE。
- `arm_usb_commands_allowed()`: 只有 ARM 模式允许消费机械臂 USB 命令。
- `consume_new_commands()`: 非 ARM 同步并丢弃 seq；ARM 且已释放时排入旧协议队列。
- `legacy_feedback_state()`: 将兼容层运动状态映射成 `PROTO_ARM_STATE_*`。
- `build_integrated_feedback()`: 用当前关节/FK 构造 `0x86 ARM_FEEDBACK`。
- `send_feedback_if_due()`: 50 Hz 节流发送机械臂反馈。
- `update_debug_snapshot_if_due()`: 100 ms 更新 `g_arm_debug_snapshot`。
- `force_all_pump_outputs_off()`: 关闭主泵和辅助泵输出。
- `enter_gravity_only_override()` / `process_gravity_only_override()` / `leave_gravity_only_override()`: 纯重力模式切换。
- `task_arm_init()`: 初始化 Pump、Arm_Control、旧协议和固定姿态状态。
- `task_arm_step_for_test()`: 机械臂常驻 task 的完整单周期。
- `task_arm_entry()`: MCU 1 ms 机械臂任务。

### `task_safety.c`
- `task_safety_estop_active()`: 返回急停状态。
- `task_safety_event_count()`: 返回安全事件计数。
- `task_safety_estop_set()`: 修改急停；进入急停时 disable registry 中所有电机。
- `motor_timeout_ms()`: 达妙和其他电机使用不同超时阈值。
- `task_safety_entry()`: 200 Hz 扫描电机离线和过温。

### `task_log.c`
- `task_log_entry()`: MCU 日志心跳任务占位。

## `App/control/arm`

### `arm_control.c`
- `enforce_t4_coupling()`: 按 J3/J4 固定关系修正轨迹 sample。
- `init_planned_home_sample()`: 初始化 planned home sample。
- `reset_feedback()`: 清空 `0x86` feedback 并回到 IDLE。
- `control_period_due()`: 将 1 ms task 包装降为 10 ms 控制周期。
- `current_reference_angles()`: 优先实测反馈，否则 planned sample。
- `target_delta_inside_fine_window()` / `target_is_inside_fine_window()`: 判断是否允许 fine tracking。
- `joint_target_is_valid()`: 检查内部关节目标合法性。
- `arm_control_grasp_j1_angle_forbidden()`: 周期性检查 GRASP J1 禁区。
- `arm_control_validate_host_target()`: host target 预检，GRASP 额外应用 J1 禁区。
- `make_safe_joint_target()`: 构造安全收臂/转底座姿态。
- `max_joint_error_to_target()` / `max_measured_speed()`: settling 判定辅助。
- `settle_tolerance_for_stage()`: 根据安全阶段选择到位容差。
- `bind_arm_motors_from_registry()`: 绑定 ARM_J1-J4 达妙句柄。
- `motor_state_is_fresh()`: 判断单电机反馈是否新鲜。
- `read_measured_angles()`: 从 registry 读取 J1-J4 并映射成机械臂角。
- `refresh_measured_feedback()`: 刷新实测 FK、online mask、fresh 标志。
- `update_pose_feedback_from_sample()`: 用 sample 更新 planned/measured 末端和 feedback。
- `update_feedback()`: 优先 measured sample 生成 `0x86` 状态。
- `set_stage_target()`: 设置当前安全阶段目标并标记待启动轨迹。
- `start_safe_retract_stage()` / `start_safe_rotate_stage()` / `start_safe_extend_stage()`: 三段式阶段入口。
- `plan_final_target()`: 根据当前参考决定直接 fine tracking 或先收臂。
- `advance_safe_stage()`: 到位后推进阶段或进入 REACHED。
- `recover_from_settle_timeout()`: 阶段超时恢复策略。
- `update_settle_state()`: 根据轨迹、真实误差、速度和超时输出 MOVING/REACHED/ERROR。
- `stop_motion_and_target()`: 停止轨迹并清目标。
- `arm_control_set_gravity_mode()`: 停止目标并回到当前位置重力保持。
- `start_pending_target()`: 从当前 sample 或实测反馈启动五次轨迹。
- `command_joint()` / `command_arm_motors()`: 达妙 MIT 输出。
- `command_idle_gravity_hold_motors()`: 无目标时按当前反馈做重力保持。
- `update_motion_gravity()`: 计算并缓存 J2/J3/J4 重力前馈。
- `arm_control_init()`: 初始化控制器、轨迹、重补、反馈和输出 gate。
- `arm_control_set_enabled()`: 硬运行开关。
- `arm_control_set_motor_output_enabled()`: 达妙输出 gate。
- `arm_control_set_j1_free_mode()`: 等待/重补场景下配置 J1 自由模式。
- `arm_control_set_target()`: host target 入口，处理 IK、重规划和三段式。
- `arm_control_set_joint_target()`: 内部关节目标入口，用于固定等待姿态。
- `arm_control_set_pump()`: 控制气泵并同步 payload 状态。
- `arm_control_tick()`: 一个机械臂控制周期。
- `arm_control_get_feedback()`: 复制 `0x86` payload。
- `arm_control_get_status()`: 复制诊断状态。

### `arm_kinematics.c`
- `arm_kinematics_compute_t4_from_t3()`: 根据 J3/J4 固定关系计算 J4。
- `arm_kinematics_forward()`: FK，关节角到末端 pose。
- `arm_kinematics_inverse()`: IK，末端 pose 到 J1-J4，并检查限位。

### `arm_motion.c`
- `arm_motion_init()`: 初始化轨迹和默认限制。
- `arm_motion_set_limits()`: 更新速度、加速度和时长限制。
- `arm_motion_start()`: 启动四轴同步五次轨迹。
- `arm_motion_update()`: 按 `now_ms` 采样轨迹。
- `arm_motion_stop()`: 停止并保持 sample。
- `arm_motion_get_state()` / `arm_motion_get_sample()` / `arm_motion_get_duration()` / `arm_motion_get_progress()`: 轨迹诊断。

### `arm_gravity_comp.c`
- `arm_gravity_comp_init()`: 初始化 payload 状态。
- `arm_gravity_comp_set_payload_state()`: 切换空载/带载目标质量。
- `arm_gravity_comp_get_payload_state()`: 返回 payload 状态。
- `arm_gravity_comp_get_active_end_mass()` / `arm_gravity_comp_get_target_end_mass()`: 返回 active/target 末端质量。
- `arm_gravity_comp_calculate()`: 计算 J2/J3/J4 重力补偿力矩。

### `arm_legacy_compat.c`
- `Arm_Control_Init()` / `Arm_Control_Process()`: 旧 API 包装到新 `arm_control`。
- `Arm_Control_MoveToPose()` / `Arm_Control_MoveToTypedPose()` / `Arm_Control_MoveToJointTarget()`: 旧 mm/关节目标入口包装。
- `Arm_Control_GetCurrentAngles()` / `Arm_Control_GetMoveStatus()` / `Arm_Control_IsFeedbackFresh()`: 旧诊断入口。
- `Arm_Motion_*`: 旧轨迹 API 包装到 `arm_motion_*`。
- `Gravity_Comp_*` / `Calculate_Gravity_Compensation()`: 旧重补 API 包装。
- `Arm_Forward_Kinematics()` / `Arm_Inverse_Kinematics()` / `Arm_Compute_T4_From_T3()`: 旧运动学 API 包装。

### `arm_vision_transform.c`
- `arm_vision_transform_compute()`: 新米制视觉坐标转换 API。
- `arm_vision_transform_update()`: 更新全局米制视觉转换结果。
- `Perform_Vision_Coordinate_Transform()`: 旧 API，输入 m、输出 mm。

## `App/control/chassis`

### `chassis_control.c`
- `controller_height_from_body()` / `set_leg_controller_height()`: 机身高度到腿 IK 高度约定。
- `reset_stand_height_ramp()` / `update_stand_height_ramp()`: 上电/站立高度平滑。
- `validate_gait_params()`: gait 参数合法性检查。
- `request_gait()`: 封装 gait machine set/request。
- `is_offline()`: AUTO/ONLINE/STANDALONE 心跳判定。
- `online_decide()`: online 下 moving 走 trot，低速 yaw 走 walk，停止走 stand。
- `gait_motion_scale()`: 计算 stand/motion blend 的轮速缩放；motion gait 之间切换保持 1。
- `apply_plan_wheel_speed()`: 给运动 gait 的四轮连续应用 planner 轮速并设置 DRIVE/HOLD。
- `apply_attitude_compensation()`: roll/pitch 转支撑腿 balance `tau_ff`。
- `update_arm_load_compensation()`: 机械臂姿态和 payload 转底盘载荷补偿。
- `update_steering()`: IMU 更新和目标 yaw 转实际 `wz`。
- `offline_decide()`: 离线保守 stand 或可选 auto march。
- `go_pre_calibration_ready()`: MCU GO 上电预标定窗口。
- `flush_motor_outputs()`: MCU 上发送 GO/M3508 总线输出。
- `make_plan_command()`: 构造 planner 命令。
- `slew_plan_command()`: 对 `vx/vy/wz` 做斜率限制。
- `update_plan_from_input()`: 运行 planner 并缓存 plan。
- `update_online_state()`: 更新 online 缓存。
- `decide_gait_for_link_state()`: online/offline gait 策略分发。
- `apply_gait_output_to_motors()`: gait、轮速、补偿、腿控制器整体派发。
- `wheel_test_active_at()`: 检查独立轮驱测试心跳并在 500 ms 超时后退出。
- `apply_direct_wheel_test()`: 固定 stand 足端目标并绕过 planner/gait 应用四轮测试速度。
- `wheel_test_build_vertical_output()`: 生成模式 3 对角 TROT 平滑垂向轨迹并叠加逐腿高度微调。
- `wheel_test_update_joint_errors()`: 计算模式 3 八个腿关节的目标减实际跟踪误差。
- `update_boot_stand()`: 上电站起阶段。
- `chassis_control_init()`: 初始化 gait、leg、planner、姿态和补偿默认值。
- `chassis_control_set_wheel_test()`: 校验/缓存 `0x16` 测试目标并控制 M3508 trace。
- `chassis_control_set_trot_test_config()`: 校验并更新模式 3 步高、周期、占空比和逐腿高度微调。
- `chassis_control_get_trot_test_debug()`: 复制模式 3 配置与诊断快照。
- `chassis_control_tick()`: 底盘 500 Hz 完整周期。
- `chassis_control_set_mode()` / `chassis_control_get_mode()`: 读写底盘模式。
- `chassis_control_get_gait_active()`: 当前 gait 枚举。
- `chassis_control_*trot*()` / `chassis_control_*walk*()` / `chassis_control_start_stand()`: gait 参数和启动 API。
- `chassis_control_play_script()` / `chassis_control_stop_script()`: 脚本步态 API。
- `chassis_control_reset_yaw()` / `chassis_control_get_yaw()` / `chassis_control_get_effective_wz()`: yaw 诊断。
- `chassis_control_get_status()`: 复制底盘状态快照。

### `chassis_planner.c`
- `chassis_planner_init()`: planner 初始化钩子。
- `chassis_planner_is_low_speed_turn()`: 判断低速/原地 yaw 转向。
- `planner_leg_local_vx()`: 按 `v_leg_x = vx - wz * y_leg` 计算单腿局部速度。
- `planner_leg_turn_vx()`: 计算不含 `vx` 的单腿 yaw 足端速度。
- `planner_apply_turn_gait()`: 低速转向 gait 参数覆盖。
- `planner_period_from_speed()`: 根据速度插值周期。
- `planner_configured_travel_step_height()` / `planner_configured_travel_duty()`: 读取普通行驶原地踏步高度和 duty。
- `planner_apply_stride_schedule()`: 应用现场标定的固定 `0.2525 s` 普通行驶周期、`0.055 m` 抬脚高度和 `0.60` 占空比。
- `planner_fill_leg_steps()`: 按轮驱模式或旧模式填充每腿步长和摘要字段。
- `chassis_planner_update()`: 输出 moving、low_speed_turn、每腿步长和轮速。

## `App/control/gait`

### `gait_machine.c`
- `gait_machine_init()`: 初始化状态机。
- `gait_machine_set()`: 立即设置当前 gait。
- `gait_machine_request()`: 请求目标 gait 和 blend。
- `gait_machine_update()`: 更新当前或 blend 输出。
- `gait_machine_state()`: 读取状态。

### `gait_stand.c`
- `gait_stand_create()`: 返回 stand gait 单例；输出全腿支撑、零足端位移和零轮速。

### `gait_trot.c`
- `gait_trot_foot_traj()`: trot 足端轨迹。
- `trot_update()`: 对角小跑相位更新，输出每腿足端和 stance。
- `gait_trot_create()`: 返回 trot gait 单例。

### `gait_walk.c`
- `gait_walk_foot_traj()`: walk 足端轨迹。
- `walk_update()`: 四拍三支撑 walk 更新。
- `gait_walk_create()`: 返回 walk gait 单例。

### `gait_trajectory.c`
- `gait_wrap01()`: 相位包裹到 `[0,1)`。
- `gait_resolve_leg_step_length()`: 优先每腿步长，否则回退 `step_length ± turn_step`。
- `gait_cycloid_foot_traj()`: 通用支撑/摆动摆线足端轨迹。

## `App/control/leg` 和 `kinematics`

### `leg_ik.c`
- `leg_ik_solve()`: 单腿逆运动学。
- `leg_fk_solve()`: 单腿正运动学。
- `leg_ik_solve_all()`: 四腿足端目标到髋/膝目标。

### `leg_controller.c`
- `leg_controller_init()`: 初始化腿控制器和 debug 补偿入口。
- `leg_controller_bind_from_registry()`: 绑定髋/膝/轮电机。
- `leg_controller_set_stand_height()`: 设置 IK 站立高度。
- `leg_controller_set_output_options()`: 配置启用腿、关节、轮子、关节增益。
- `try_set_pos()`: 髋/膝位置命令。
- `try_set_wheel_mit()`: M3508 DRIVE/HOLD MIT 命令和限幅。
- `wheel_mit_should_integrate()`: 根据未限幅力矩和速度误差执行条件积分抗饱和。
- `wheel_mit_apply_drive()`: 跨支撑/摆动相维护有界速度误差积分并发送 MIT。
- `wheel_mit_apply_hold()`: 进入 HOLD 时一次锁存实际轮角，随后通过 MIT 保持固定位置和零速。
- `gravity_comp_prepare_payload_shares()`: 按支撑腿分配 payload 质量。
- `gravity_comp_prepare_balance_forces()`: 按支撑腿分配姿态虚拟力。
- `gravity_comp_compute()`: 计算单腿髋/膝 `tau_ff`。
- `leg_controller_apply_dt()`: IK、补偿和电机派发主入口。
- `leg_controller_apply()`: 默认 2 ms 周期包装。

## `App/control/attitude`

### `attitude_estimator.c`
- `attitude_estimator_init()`: 重置姿态状态。
- `attitude_estimator_update()`: gyro 积分 yaw，accel 估计 roll/pitch。
- `attitude_estimator_get_yaw()`: 返回 yaw。
- `attitude_estimator_reset_yaw()`: yaw 清零。
- `attitude_estimator_get_state()`: 返回姿态状态指针。

### `steer_controller.c`
- `steer_controller_init()`: 初始化 yaw PID。
- `steer_controller_set_mode()`: 设置 OFF/YAW 模式。
- `steer_controller_set_target_yaw()`: 设置目标 yaw。
- `steer_controller_update()`: 输出 yaw-rate 命令。
- `steer_controller_get_cfg()`: 返回配置指针。

## `App/control/script`

### `gait_script.c`
- `gait_script_create()`: 返回 script gait 单例。
- `gait_script_set_script()`: 加载脚本。
- `gait_script_rewind()`: 回到脚本起点。
- `gait_script_state()`: 读取脚本播放器状态。

### `script_player.c`
- `script_player_init()`: 初始化播放器。
- `script_player_load()`: 加载并开始脚本。
- `script_player_reset()`: 重置脚本时间。
- `script_player_update()`: 推进脚本并采样输出。
- `script_player_state()`: 读取播放状态。

### `script_builtin.c`
- `script_builtin_find()`: 按名称查找内置脚本。

## `App/device`

### `motor_registry.c`
- `motor_registry_init()`: 清空 registry。
- `motor_get()`: 按 logical id 返回电机句柄。
- `motor_get_cfg()`: 返回硬件映射配置。
- `motor_registry_count()`: 返回 registry 槽数。
- `motor_registry_bind()`: 绑定运行期电机设备。

### `motor_go.c`
- `motor_go_init_all()`: 创建 GO 实例、注册 UART RX、绑定 registry。
- `motor_go_send_all()`: 按 UART 总线发送 GO 命令。
- `motor_go_calibrate_all()`: 根据反馈锁存零偏。
- `motor_go_debug_get_state()`: 复制 GO debug 状态。

### `motor_m3508.c`
- `motor_m3508_init_all()`: 创建 M3508 实例、注册 FDCAN RX、绑定 registry。
- `motor_m3508_set_mit_limits()`: 设置本次 MIT 命令限幅。
- `motor_m3508_trace_reset()` / `motor_m3508_trace_enable()`: trace 调试。
- `motor_m3508_send_all()`: 运行控制环并发送 DJI 电流帧。

### `motor_damiao.c`
- `motor_damiao_pack_mit()`: 打包达妙 MIT 帧。
- `motor_damiao_parse_feedback()`: 解码反馈并更新状态。
- `motor_damiao_process()`: 自动 enable/re-enable 维护。
- `motor_damiao_auto_enable_count()`: 自动 enable 发送计数。
- `motor_damiao_init_all()`: 创建 J1-J4 达妙、绑定 registry、注册 FDCAN3 RX。

### `arm_pump.c`
- `arm_pump_init()`: 初始化主气泵并默认关闭。
- `arm_pump_set()`: 打开/关闭主气泵。
- `arm_pump_is_enabled()`: 返回主气泵状态。
- `arm_pump_set_aux()` / `arm_pump_aux_is_enabled()`: 辅助通道接口。

### `pump_control.c`
- `Pump_Control_Init()`: 旧 API 初始化，关闭全部输出。
- `Pump_Control_Process()`: 应用旧 debug 变量。
- `Pump_Control_Set()`: 包装主气泵并同步 payload 状态。
- `Pump_Control_SetPC8/PC9/PA8/PA9()`: 辅助通道旧 API。

### `dm4310_posvel.c`
- `DM_PosVel_Init()` / `DM_CAN_Filter_Init()`: 兼容 no-op。
- `DM_Parse_Feedback()`: 包装达妙反馈解析并更新旧 debug 结构。
- `DM_Motor_Enable()` / `DM_PosVel_Control()` / `DM_MIT_Control_4310()` / `DM_MIT_Control_4340()`: 旧达妙控制 API 包装。

### `imu_bmi088.c`
- `imu_bmi088_init()`: 初始化 BMI088。
- `imu_bmi088_read()`: 读取 accel/gyro/temp。
- `imu_bmi088_set_accel_range()` / `imu_bmi088_set_gyro_range()`: 设置量程。
- `imu_bmi088_is_ready()`: 返回 IMU ready。
- `imu_bmi088_get_diag()`: 复制 IMU 诊断。

## `App/bsp`

- `bsp_fdcan_init()` / `bsp_fdcan_attach_rx()` / `bsp_fdcan_send()`: FDCAN 初始化、RX 回调和发送。
- `bsp_fdcan_hal_rxfifo0_cb()` / `bsp_fdcan_hal_error_cb()`: HAL 回调桥接。
- `bsp_uart_init()` / `bsp_uart_attach_rx()` / `bsp_uart_send()` / `bsp_uart_wait_tx_done()`: UART/RS485 BSP。
- `bsp_uart_hal_rx_event()` / `bsp_uart_hal_tx_done()`: HAL UART 回调桥接。
- `bsp_usb_cdc_init()` / `bsp_usb_cdc_attach_rx()` / `bsp_usb_cdc_send()` / `bsp_usb_cdc_on_rx()`: USB CDC BSP。
- `bsp_gpio_output_init()` / `bsp_gpio_write()` / `bsp_gpio_read_latch()`: GPIO 输出和 latch。
- `bsp_spi_init()` / `bsp_spi_set_cs()` / `bsp_spi_transfer_byte()`: SPI BSP。
- `bsp_time_init()` / `bsp_time_now_ms()` / `bsp_time_now_us()` / `bsp_time_delay_ms()` / `bsp_time_delay_us()`: 时间 BSP。

## `App/service`

### `proto_frame.c`
- `proto_frame_init()`: 初始化 parser。
- `proto_frame_reset()`: 重置 parser。
- `proto_frame_feed()`: 增量解析整机协议帧。
- `proto_frame_build()`: 构造整机协议帧。

### `proto_dispatch.c`
- `proto_dispatcher_init()`: 安装分发表。
- `proto_dispatch_on_frame()`: 按 FuncID 调 handler，记录 hit/miss/bad length。

### `arm_serial_protocol.c`
- `Arm_Serial_Protocol_Init()`: 初始化旧机械臂协议兼容状态。
- `Arm_Serial_Protocol_QueueTarget()`: 整机协议经 gate 后排入旧 target 队列。
- `Arm_Serial_Protocol_QueuePump()`: 整机协议经 gate 后排入旧 pump 队列。
- `Arm_Serial_Protocol_Receive()`: 手动喂旧单机械臂串口帧。
- `Arm_Serial_Protocol_Process()`: 处理 target/pump/place-cycle/旧反馈兼容。
- `Arm_Serial_Protocol_PlaceCycleSequence()`: 返回 PLACE 完成序号。

### `pid.c`
- `app_pid_reset()`: 清空 PID 状态。
- `app_pid_set_tunings()`: 设置 PID 参数。
- `app_pid_set_output_limits()` / `app_pid_set_integral_limits()`: 设置限幅。
- `app_pid_update_dt()` / `app_pid_update_ms()`: 位置式 PID。
- `app_pid_update_inc_dt()` / `app_pid_update_inc_ms()`: 增量式 PID。

## `App/common`

### `log.c`
- `log_init()`: 初始化日志状态。
- `log_register_backend()`: 注册日志后端。
- `log_set_global_level()` / `log_get_global_level()`: 全局日志等级。
- `log_set_level()`: tag 日志等级。
- `log_emit()`: 格式化并分发日志。
- `log_last_line()`: 最近一行日志。
