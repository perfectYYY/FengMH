# damiao_new1 机械臂函数迁移对账

目的：保留旧机械臂工程的主要业务函数和调试入口，方便后续负责机械臂的人在 FengMH 集成工程里继续开发。当前策略是：

- 主控制链仍使用新工程的 `task_arm -> arm_control -> motor_damiao/arm_pump`。
- 旧工程 CamelCase API 以兼容层方式保留，内部转到新抽象。
- 旧 USB/CAN/HAL 直连逻辑不抢主链路；需要单独调试时可手动调用。

## 1. 新增兼容文件

| 新文件 | 来源 | 作用 |
|---|---|---|
| `App/control/include/arm/arm_legacy_compat.h` | `Core/Inc/arm_control.h`、`arm_kinematics.h`、`arm_motion.h`、`gravity_comp.h` | 保留旧类型和旧函数名。 |
| `App/control/src/arm/arm_legacy_compat.c` | `Core/Src/arm_control.c`、`arm_kinematics.c`、`arm_motion.c`、`gravity_comp.c` | 旧 API 包装到新 `arm_control/arm_motion/arm_gravity_comp`。 |
| `App/control/include/arm/arm_vision_transform.h` | `Core/Inc/vision_transform.h` | 保留视觉转换接口，并提供新米制 API。 |
| `App/control/src/arm/arm_vision_transform.c` | `Core/Src/vision_transform.c` | 迁入相机坐标到机械臂基座坐标转换。 |
| `App/device/include/pump_control.h` | `Core/Inc/pump_control.h` | 保留旧气泵函数名。 |
| `App/device/src/pump_control.c` | `Core/Src/pump_control.c` | 旧气泵函数包装到 `arm_pump`，同步 payload 状态。 |
| `App/device/include/dm4310_posvel.h` | `Core/Inc/dm4310_posvel.h` | 保留旧达妙控制/反馈结构和函数名。 |
| `App/device/src/dm4310_posvel.c` | `Core/Src/dm4310_posvel.c` | 旧达妙函数包装到 `motor_damiao/motor_registry`。 |
| `App/service/include/protocol/arm_serial_protocol.h` | `Core/Inc/arm_serial_protocol.h` | 保留旧单机械臂串口协议。 |
| `App/service/src/protocol/arm_serial_protocol.c` | `Core/Src/arm_serial_protocol.c` | 保留旧 target/pump parser、延迟关泵、feedback 打包。 |

## 2. 函数级覆盖

### 2.1 运动学 `arm_kinematics`

| 旧函数 | 当前状态 | 说明 |
|---|---|---|
| `Arm_Forward_Kinematics()` | 已保留 | 在 `arm_legacy_compat.c` 中保留旧 mm API。 |
| `Arm_Inverse_Kinematics()` | 已保留 | 旧 mm API，关节限位和 t4 耦合逻辑保留。 |
| `Arm_Compute_T4_From_T3()` | 已保留 | 保留旧固定和关系。 |

新主链对应函数仍是 `arm_kinematics_forward()`、`arm_kinematics_inverse()`、`arm_kinematics_compute_t4_from_t3()`，单位为 m/rad。

### 2.2 轨迹 `arm_motion`

| 旧函数 | 当前状态 | 说明 |
|---|---|---|
| `Arm_Motion_Init()` | 已保留 | 包装到 `arm_motion_init()`。 |
| `Arm_Motion_SetLimits()` | 已保留 | 包装到 `arm_motion_set_limits()`。 |
| `Arm_Motion_Start()` | 已保留 | 包装到 `arm_motion_start()`。 |
| `Arm_Motion_Update()` | 已保留 | 包装到 `arm_motion_update()`。 |
| `Arm_Motion_Stop()` | 已保留 | 包装到 `arm_motion_stop()`。 |
| `Arm_Motion_GetState()` | 已保留 | 包装到 `arm_motion_get_state()`。 |
| `Arm_Motion_GetSample()` | 已保留 | 包装到 `arm_motion_get_sample()`。 |
| `Arm_Motion_GetDuration()` | 已保留 | 包装到 `arm_motion_get_duration()`。 |
| `Arm_Motion_GetProgress()` | 已保留 | 包装到 `arm_motion_get_progress()`。 |

### 2.3 重力补偿 `gravity_comp`

| 旧函数 | 当前状态 | 说明 |
|---|---|---|
| `Gravity_Comp_Init()` | 已保留 | 包装到 `arm_gravity_comp_init()`。 |
| `Gravity_Comp_SetPayloadState()` | 已保留 | 包装到 `arm_gravity_comp_set_payload_state()`。 |
| `Gravity_Comp_GetPayloadState()` | 已保留 | 包装到 `arm_gravity_comp_get_payload_state()`。 |
| `Gravity_Comp_GetActiveEndMass()` | 已保留 | 包装到 `arm_gravity_comp_get_active_end_mass()`。 |
| `Gravity_Comp_GetTargetEndMass()` | 已保留 | 包装到 `arm_gravity_comp_get_target_end_mass()`。 |
| `Calculate_Gravity_Compensation()` | 已保留 | 包装到 `arm_gravity_comp_calculate()`，并保留旧 `dbg_gravity_scale_*` 比例。 |

旧调试变量 `dbg_gc_mass_*`、`dbg_gc_com_*` 也保留；调用兼容重补函数时会同步到新 `g_arm_gc_*`。

### 2.4 机械臂控制 `arm_control`

| 旧函数 | 当前状态 | 说明 |
|---|---|---|
| `Arm_Control_Init()` | 已保留 | 包装到 `arm_control_init()` 并使能兼容层。 |
| `Arm_Control_Process()` | 已保留 | 包装到 `arm_control_tick()`；旧 `debug_motion_trigger`/A-B 循环写电机入口已删除，避免与正式状态机抢控制权。 |
| `Arm_Control_SetGravityMode()` | 已保留 | 当前实现为安全停止/清目标；不再直接抢主控制任务。 |
| `Arm_Control_SetGravityOnlyMode()` | 已保留 | 兼容层保留 flag，并阻止旧 API 继续接收目标。 |
| `Arm_Control_IsGravityOnlyMode()` | 已保留 | 返回兼容层 flag。 |
| `Arm_Control_SetJointTarget()` | 已保留 | 包装到 `Arm_Control_MoveToJointTarget()`。 |
| `Arm_Control_MoveToJointTarget()` | 已保留 | 通过 FK 转末端 pose，再进入新目标入口。 |
| `Arm_Control_MoveToPose()` | 已保留 | 旧 mm pose 转 m 后调用 `arm_control_set_target()`。 |
| `Arm_Control_GetMode()` | 已保留 | 根据新状态映射旧 enum。 |
| `Arm_Control_GetMoveStatus()` | 已保留 | 根据新状态映射旧 enum。 |
| `Arm_Control_GetCurrentAngles()` | 已保留 | 优先读 motor_registry 中四个达妙反馈；无反馈时用规划末端反解。 |
| `Arm_Control_IsFeedbackFresh()` | 已保留 | 包装到 `arm_control_status_t.motor_feedback_fresh`。 |

注意：旧工程的重力保持模式会持续给达妙发重补力矩；新主链在上电反馈就绪后进入固定等待姿态，ARM 模式的新 GRASP 才释放给抓放流程。兼容层保留函数入口和安全语义，但不会绕过新 `task_arm` gate 去抢电机输出。

### 2.5 视觉转换 `vision_transform`

| 旧函数/变量 | 当前状态 | 说明 |
|---|---|---|
| `Perform_Vision_Coordinate_Transform()` | 已保留 | 旧输入 `vision_camera_data` 为 m，输出 `vision_world_data` 为 mm。 |
| `vision_camera_data` | 已保留 | 旧上位机相机输入变量。 |
| `vision_world_data` | 已保留 | 旧世界坐标输出变量。 |
| `debug_last_camera_data` | 已保留 | 调试快照。 |
| `debug_last_world_data` | 已保留 | 调试快照。 |
| `debug_vision_rx_count` | 已保留 | 转换计数。 |
| `arm_vision_transform_compute()` | 新增 | 新工程米制 API。 |
| `arm_vision_transform_update()` | 新增 | 用全局米制相机数据更新全局米制世界坐标。 |

### 2.6 气泵 `pump_control`

| 旧函数 | 当前状态 | 说明 |
|---|---|---|
| `Pump_Control_Init()` | 已保留 | 包装到 `arm_pump_init()` 并关闭全部输出。 |
| `Pump_Control_Process()` | 已保留 | 应用旧 debug 变量到新 GPIO 抽象。 |
| `Pump_Control_Set()` | 已保留 | 包装到 `arm_pump_set()`，并同步 `debug_payload_loaded`/重补 payload。 |
| `Pump_Control_IsEnabled()` | 已保留 | 包装到 `arm_pump_is_enabled()`。 |
| `Pump_Control_SetPC8()` | 已保留 | 包装到 `arm_pump_set_aux(ARM_PUMP_AUX_PC8)`。 |
| `Pump_Control_SetPC9()` | 已保留 | 包装到 `arm_pump_set_aux(ARM_PUMP_AUX_PC9)`。 |
| `Pump_Control_SetPA8()` | 已保留 | 包装到 `arm_pump_set_aux(ARM_PUMP_AUX_PA8)`。 |
| `Pump_Control_SetPA9()` | 已保留 | 包装到 `arm_pump_set_aux(ARM_PUMP_AUX_PA9)`。 |

### 2.7 达妙 `dm4310_posvel`

| 旧函数 | 当前状态 | 说明 |
|---|---|---|
| `DM_PosVel_Init()` | 已保留 | 兼容 no-op；FDCAN 初始化由 `bsp_fdcan/motor_damiao_init_all()` 管。 |
| `DM_CAN_Filter_Init()` | 已保留 | 兼容 no-op；过滤/回调由 BSP 管。 |
| `DM_Parse_Feedback()` | 已保留 | 调用 `motor_damiao_parse_feedback()` 并更新旧 `dm_feedback_motor*`。 |
| `DM_Motor_Enable()` | 已保留 | 通过 `motor_registry` 找达妙电机并调用 `ops->enable()`。 |
| `DM_PosVel_Control()` | 已保留 | 包装到 `ops->set_position()`。 |
| `DM_MIT_Control_4310()` | 已保留 | 包装到 `ops->set_position()`。 |
| `DM_MIT_Control_4340()` | 已保留 | 包装到 `ops->set_position()`。 |

底层冲突处理：旧代码直接持有 `FDCAN_HandleTypeDef`、配置滤波器并注册 FIFO 中断；新工程统一由 BSP 和 motor driver 管。兼容函数保留旧签名，但不会重新配置 FDCAN。

### 2.8 旧串口协议 `arm_serial_protocol`

| 旧函数 | 当前状态 | 说明 |
|---|---|---|
| `Arm_Serial_Protocol_Init()` | 已保留 | 初始化旧协议 parser/debug 状态。 |
| `Arm_Serial_Protocol_Receive()` | 已保留 | 可手动喂旧 `0x12/0x13` 帧。 |
| `Arm_Serial_Protocol_QueueTarget()` / `Arm_Serial_Protocol_QueuePump()` | 新增薄适配 | 整机协议 `0x11/0x14` 经 mode gate 后排入旧协议待处理队列，避免 task 直接改机械臂状态。 |
| `Arm_Serial_Protocol_Process()` | 已保留 | 保留 target/pump 消费、延迟关泵、等待下一次抓取等逻辑；旧 `0x21` feedback 打包保留但整合固件默认不发送。 |

底层冲突处理：旧协议不自动 attach 到 `bsp_usb_cdc_attach_rx()`，因为当前主链由 `task_comm` 独占 USB CDC。如果需要单机械臂调试，可以在专门测试入口里手动调用 `Arm_Serial_Protocol_Receive()`。

## 3. 未迁移/不应迁移项

| 旧文件/函数 | 处理方式 | 原因 |
|---|---|---|
| `Core/Src/main.c` | 不迁入主链 | 旧阻塞主循环与 FreeRTOS app 架构冲突。 |
| `Core/Src/fdcan.c` / `gpio.c` | 不迁入主链 | CubeMX/HAL 外设初始化已由 FengMH BSP 接管。 |
| `USB_DEVICE/*` / `usbd_cdc_if.*` | 不迁入主链 | USB CDC 由 `bsp_usb_cdc` 和 `task_comm` 统一管理。 |
| `stm32h7xx_it.c` / MSP / system / startup | 不迁入 | 底层启动和中断文件不能混入两个工程版本。 |
| `Debug/`、`build/` 产物 | 不迁入 | 编译产物，不是源码。 |

## 4. 回归覆盖

| 测试 | 覆盖点 |
|---|---|
| `test_arm_legacy_kinematics_and_gravity_wrappers()` | 旧 IK/FK 和旧重补包装函数。 |
| `test_arm_legacy_motion_wrappers_share_quintic_engine()` | 旧轨迹函数名可用，底层复用新五次轨迹。 |
| `test_arm_vision_transform_preserves_legacy_units()` | 旧视觉转换的 m 输入、mm 输出语义。 |
| `test_pump_control_legacy_wrappers_update_payload_state()` | 旧气泵 API 同步新气泵和 payload 状态。 |
| `test_dm4310_legacy_mit_wrapper_uses_registry_motor()` | 旧达妙 MIT 函数通过 registry 下发。 |
| `test_dm4310_legacy_parse_feedback_updates_debug_pose()` | 旧达妙反馈结构更新。 |
| `test_arm_serial_protocol_legacy_parser_caches_target_and_pump()` | 旧串口协议 parser 可解析 target/pump。 |

验证命令：

```bash
cmake --build build_host_tests
ctest --test-dir build_host_tests --output-on-failure
cmake --build build_arm
```
