# 底盘、机械臂和上位机整合状态

本文档原来是初始迁移方案；当前改为“迁移后状态 + 仍需联调项”。代码事实以 `/Users/code/FengMH` 当前仓库为准。

## 当前结论

FengMH 已经成为统一 STM32 下位机主工程：

- 单一 USB CDC 下行入口：`task_comm`。
- 单一 motor registry：GO、M3508、达妙都按 logical id 绑定。
- 底盘和机械臂任务并行常驻：`t_chassis` + `t_arm`。
- MCU 本地 mode gate 已实现：NAV 释放底盘，ARM 释放机械臂，ESTOP/ERROR 禁用输出。
- 机械臂旧工程核心能力已迁入或包装：达妙、气泵、IK/FK、轨迹、重力补偿、旧协议兼容层。

## 当前固件架构

```text
Core/Src/main.c
  -> app_init()
  -> osKernelInitialize()
  -> MX_FREERTOS_Init()
      -> app_tasks_create()
          -> t_comm
          -> t_chassis
          -> t_arm
          -> t_safety
          -> t_log
```

## 协议现状

| FuncID | 方向 | 名称 | 当前状态 |
|---:|---|---|---|
| `0x10` | Host -> MCU | `CHASSIS_CMD` | 已实现，兼容 12B 和 17B 扩展帧。 |
| `0x11` | Host -> MCU | `ARM_TARGET` | 已实现，通信入口有 reach/finite/type 预检。 |
| `0x12` | Host -> MCU | `GAIT_CMD` | 已实现，底盘 stand/trot/walk 调试。 |
| `0x13` | Host -> MCU | `MIT_CMD` | 已实现，拒绝 ARM_J1-J6 旁路。 |
| `0x14` | Host -> MCU | `ARM_PUMP` | 已实现。 |
| `0x15` | Host -> MCU | `ROBOT_MODE_CMD` | 已实现，ESTOP/ERROR 同步 safety。 |
| `0x80` | MCU -> Host | `STATE` | 已实现，10 Hz。 |
| `0x81` | MCU -> Host | `MOTOR_STATE` | 已实现，5 Hz 单电机轮询。 |
| `0x86` | MCU -> Host | `ARM_FEEDBACK` | 已实现，`task_arm` 50 Hz。 |

旧机械臂 `0x12/0x13/0x21` 不作为整机主协议使用；旧单机械臂协议只保留在 `arm_serial_protocol` 兼容层，且默认不接管 USB。

## 模式互锁现状

| Mode | 当前固件行为 |
|---:|---|
| `IDLE=0` | 底盘速度清零；机械臂不消费新 target/pump，保持 park。 |
| `NAV=1` | 底盘允许速度进入 planner；机械臂不消费新 target/pump。 |
| `ARM=2` | 底盘速度清零；机械臂进入等待姿态，合法 GRASP 到来后释放给抓放流程。 |
| `ESTOP=3` | safety 置急停，disable 全部电机；底盘跳过 tick；机械臂不自动 enable。 |
| `ERROR=4` | 同急停路径。 |

## 已完成迁移项

- `task_comm`：整机协议解析、缓存、上行状态/电机/机械臂反馈。
- `task_chassis`：mode gate、500 Hz 控制包装、walk/trot/stand/script API。
- `task_arm`：常驻机械臂任务、固定等待姿态、ARM gate、纯重补调试、0x86 feedback。
- `motor_damiao`：J1-J4 达妙 MIT、反馈解析、FDCAN3 路由、自动 enable。
- `arm_pump/pump_control`：主气泵 PD11、辅助通道兼容、payload 状态同步。
- `arm_control`：IK/FK、安全三段式、fine tracking、五次轨迹、重力补偿、真实反馈优先、输出 gate。
- `arm_legacy_compat`：旧 CamelCase API 保留，主链不由它抢 USB/CAN。
- `chassis_control`：命令斜率限制、walk 低速转向、轮足 planner、姿态和机械臂载荷补偿入口。
- host tests：覆盖协议、planner、gait、底盘端到端、机械臂控制、兼容层、气泵和达妙。

## 仍需上机确认

- 达妙 J1-J4 实际方向、零位、软限位和力矩上限。
- FDCAN3 波特率、滤波配置、bus-off 恢复在实机上的表现。
- 主气泵 PD11 与辅助通道接线是否与现场硬件一致。
- 底盘 GO 预标定和 stand height ramp 在实机上的时间和姿态是否合适。
- `APP_CHASSIS_COMP_FORCE_DISABLE` 关闭补偿后的基础行走，再逐步打开姿态/载荷补偿。
- 上位机是否严格按 NAV/ARM 发送 mode；MCU 本地 gate 已兜底，但上位机仍应减少无效命令。

## 上位机接口要求

上位机侧只需要遵守协议和模式：

```text
NAV 阶段:
  周期发送 0x15 NAV
  发送 0x10 CHASSIS_CMD
  不发送新的 0x11/0x14

ARM 阶段:
  周期发送 0x15 ARM
  底盘速度发零或不发
  发送 0x11 ARM_TARGET 和 0x14 ARM_PUMP
  订阅/解析 0x86 ARM_FEEDBACK

急停/错误:
  发送 0x15 ESTOP 或 ERROR
```

## 验证顺序

1. `cmake --build build_host_tests && ctest --test-dir build_host_tests --output-on-failure`。
2. `cmake --build build_arm`。
3. 上板只看 USB 协议和 `0x80/0x81/0x86` 上行。
4. 单独确认达妙 enable/feedback，不发运动目标。
5. ARM 模式固定等待姿态，确认 J1/J2/J3/J4 方向。
6. 单点 GRASP/PLACE，不带底盘运动。
7. NAV 模式底盘基础 stand/trot/walk。
8. 最后做 NAV -> ARM -> NAV 整机联调。

## 相关文档

- 当前主链路：[`firmware_control_flow.md`](firmware_control_flow.md)
- 代码图：[`unified_control_code_map.md`](unified_control_code_map.md)
- 机械臂兼容层：[`arm_legacy_migration_audit.md`](arm_legacy_migration_audit.md)
- 机械臂载荷补偿：[`arm_chassis_load_compensation.md`](arm_chassis_load_compensation.md)
