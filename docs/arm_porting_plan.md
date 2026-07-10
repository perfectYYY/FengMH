# 机械臂下位机移植状态

本文档原来是从 `damiao_new1` 移植到 FengMH 的执行计划；当前改为“已迁入能力 + 仍需确认项”。

## 当前结论

机械臂下位机能力已经进入 FengMH 主链：

```text
task_arm
  -> Arm_Serial_Protocol queue/process
  -> arm_control_tick()
      -> arm_kinematics
      -> arm_motion
      -> arm_gravity_comp
      -> motor_damiao
      -> arm_pump
  -> task_comm_send_arm_feedback(0x86)
```

旧工程的裸循环 `main.c` 不迁入；旧 CamelCase API 通过兼容层保留，主控制权仍由 `task_arm` 管。

## 迁入目录

| 能力 | 当前文件 |
|---|---|
| 达妙 DM4340/DM4310 驱动 | `App/device/src/motor_damiao.c` |
| 旧达妙 API 兼容 | `App/device/src/dm4310_posvel.c` |
| 主气泵/辅助气泵 | `App/device/src/arm_pump.c`、`App/device/src/pump_control.c` |
| 机械臂控制门面 | `App/control/src/arm/arm_control.c` |
| IK/FK | `App/control/src/arm/arm_kinematics.c` |
| 五次轨迹 | `App/control/src/arm/arm_motion.c` |
| 重力补偿 | `App/control/src/arm/arm_gravity_comp.c` |
| 旧 API 兼容 | `App/control/src/arm/arm_legacy_compat.c` |
| 视觉坐标转换 | `App/control/src/arm/arm_vision_transform.c` |
| 旧机械臂协议兼容 | `App/service/src/protocol/arm_serial_protocol.c` |
| 机械臂 FreeRTOS wrapper | `App/app/src/task_arm.c` |

## 当前协议

| FuncID | 名称 | 状态 |
|---:|---|---|
| `0x11` | `ARM_TARGET` | 主入口，`target_type + x/y/z`，单位 m。 |
| `0x14` | `ARM_PUMP` | 主气泵命令。 |
| `0x15` | `ROBOT_MODE_CMD` | ARM mode gate。 |
| `0x86` | `ARM_FEEDBACK` | `arm_state + end_x/y/z + theta1_rad`，50 Hz。 |

旧 `0x12/0x13/0x21` 机械臂协议只保留手动兼容入口，默认不接 USB。

## task_arm 当前行为

默认正式策略：

```text
非 ARM -> PARK 固定姿态
进入 ARM -> FIRST_LIFT -> FIRST_J1 -> HOLDING
HOLDING + 新合法 GRASP -> RELEASED_TO_HOST
PLACE 完成并关泵 -> hold 3000 ms -> SECOND_SAFE_LIFT -> SECOND_J1 -> SECOND_FIXED_LIFT
```

关键点：

- 机械臂 task 常驻，不随 mode 停止。
- 非 ARM 模式的新 target/pump 会同步 seq 并丢弃，不会切回 ARM 后补执行旧命令。
- ARM 模式也不是立即执行 target：必须等待固定姿态到位，并且收到新的合法 GRASP。
- PLACE target 不会释放固定等待姿态。
- 急停时禁用达妙输出和控制器，不运行自动 enable。
- `debug_arm_force_gravity_only=1` 会清队列、关泵并强制纯重力保持。

## arm_control 当前能力

已实现：

- host target 类型、finite、IK 校验。
- GRASP J1 周期性禁区；PLACE 不应用该禁区。
- J1 最近等效角选择，避免绕远圈。
- 安全三段式：`RETRACT -> ROTATE_BASE -> EXTEND`。
- fine tracking：小范围视觉更新直接重规划，大变化回收臂。
- 四关节同步五次轨迹。
- 基于真实反馈的 settling/REACHED。
- 运动中反馈超过 `APP_ARM_MOTION_FEEDBACK_ABORT_MS=250 ms` 报错。
- 无目标且反馈新鲜时当前位置重力保持。
- J2/J3/J4 重力前馈。
- 输出 gate：MCU 默认开，host 默认关；输出 gate 开时必须反馈新鲜。

## 达妙驱动当前能力

已实现：

- J1/J2/J3 = DM4340，J4 = DM4310。
- FDCAN3 MIT 控制帧打包。
- enable/disable/reset fault 特殊帧。
- 8B feedback 解码到 `motor_state_t` 和 driver context。
- 按 payload motor id 路由反馈。
- 自动 enable/re-enable：无反馈、未使能或反馈沉默时周期重发 enable。
- host tests 覆盖 MIT 打包、FDCAN3 TX、反馈解析、路由和自动 enable。

仍需上板确认：

- CAN ID、反馈 ID/payload id 与实际达妙固件一致。
- J1-J4 方向、零位、软限位。
- MIT kp/kd/tau_ff 默认值和力矩限幅。
- bus-off 恢复在实机上是否稳定。

## 气泵当前能力

已实现：

- 主气泵 `PD11` 上电默认关闭。
- `Pump_Control_Set()` 包装到 `arm_pump_set()`。
- pump on/off 同步 `arm_gravity_comp` payload 空载/带载状态。
- `PC8/PC9/PA8/PA9` 作为辅助通道兼容接口保留，但默认不主动初始化。
- ESTOP 下当前 `task_arm` 不会强制关主泵；纯重力 override 会关全部泵。

需要现场确认气泵急停策略：保持吸附还是立即释放。

## 兼容层边界

保留旧 API 的目的：

- 方便旧测试和现场 Live Expressions。
- 保留旧单位语义：旧 pose 多为 mm，新主链为 m/rad。
- 保留旧抓放协议中的 PLACE 延迟关泵和等待下一次 GRASP 语义。

不允许兼容层做的事：

- 自动抢 USB CDC 主入口。
- 绕过 `task_arm` mode gate 消费上位机命令。
- 重新配置 FDCAN/HAL。
- 与 `arm_control` 主链同时直接竞写达妙。

## 测试覆盖

host tests 已覆盖：

- IK/FK roundtrip。
- 旧运动学/轨迹/重补包装。
- 五次轨迹到达目标。
- 末端 payload 质量平滑过渡。
- 视觉转换单位语义。
- 气泵 GPIO 和 payload 状态。
- arm_control target、反馈、输出 gate、重力保持、反馈过期、settling。
- 三段式安全运动、阶段中目标更新、fine tracking。
- task_arm 固定等待姿态、ARM/NAV gate、纯重补、ESTOP。
- 达妙打包、反馈、FDCAN3 路由、自动 enable。

## 后续工作

1. 上板确认达妙方向/零位/限位。
2. 上板确认固定等待姿态的机械安全空间。
3. 确认气泵急停策略。
4. 将达妙自动 enable 计数、FDCAN 错误计数、机械臂固定姿态状态接入更完整遥测。
5. 上位机按 `0x15` mode gate 做 NAV/ARM 编排。
