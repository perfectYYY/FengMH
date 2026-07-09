# 底盘、机械臂和 ROS 系统整合迁移方案

本文档是四足机器狗底盘、机械臂、识别和导航工程的初始整合方案。目标是把两个 STM32 下位机工程合并成一个 FreeRTOS 固件，把机械臂识别和底盘导航合入同一套 ROS2 系统，并用明确的模式状态机保证行走和抓取不会同时输出。

## 当前工程边界

| 工程 | 当前职责 | 整合后定位 |
| --- | --- | --- |
| `/Users/code/FengMH` | 底盘 STM32 固件，FreeRTOS 架构，分层清晰 | 统一下位机主工程 |
| `/Users/leon/Desktop/merge/damiao_new1` | 机械臂 STM32 固件，裸循环，直接 HAL 调用 | 机械臂控制、达妙电机、气泵逻辑来源 |
| `/Users/leon/Desktop/merge/GSing_dipan` | 底盘导航定位 ROS2 上位机 | 统一 ROS2 工作空间中的导航模块 |
| `/Users/leon/Desktop/merge/gsing_dog-main` | 机械臂识别和抓放 ROS2 上位机 | 统一 ROS2 工作空间中的识别和抓放模块 |

下位机统一采用 `/Users/code/FengMH` 的 `remake_original` 分支作为基线。机械臂代码不直接塞进 `main.c`，而是按现有 `App/app`、`App/control`、`App/device`、`App/bsp`、`App/service` 分层迁移。

## 总体原则

- 一个 STM32 工程、一个 USB CDC 协议入口、一个电机/设备注册体系。
- 一个 ROS2 串口 bridge 独占 STM32 串口，导航和机械臂节点只发布意图，不直接抢串口。
- MCU 和 ROS2 双侧都要有模式状态机；即使上位机逻辑异常，MCU 本地也必须禁止底盘和机械臂同时输出。
- 先统一协议，再迁移驱动，再接入控制任务，最后合并 ROS2 编排。
- 每个阶段都有 host 单测或上机可观测遥测，避免一次性大合并后难定位。

## 统一协议

底盘现有协议帧结构保留：

```text
55 AA | func_id(1) | len(1) | payload(len) | checksum(1)
checksum = 从 0x55 到 payload 最后一个字节的累加和低 8 位
```

建议功能号如下：

| FuncID | 方向 | 名称 | Payload |
| --- | --- | --- | --- |
| `0x10` | Host -> MCU | `CHASSIS_CMD` | `f32 vx + f32 vy + f32 wz`，可兼容扩展 `target_yaw + steer_mode` |
| `0x11` | Host -> MCU | `ARM_TARGET` | `u8 target_type + f32 x + f32 y + f32 z`，坐标系 `arm_base`，单位 m |
| `0x12` | Host -> MCU | `GAIT_CMD` | 底盘步态调试命令，保留给 FengMH |
| `0x13` | Host -> MCU | `MIT_CMD` | 单电机 MIT 调试命令，保留给 FengMH |
| `0x14` | Host -> MCU | `ARM_PUMP` | `u8 pump_on`，`1` 吸附，`0` 释放 |
| `0x15` | Host -> MCU | `ROBOT_MODE_CMD` | `u8 mode`，`0` idle，`1` nav，`2` arm，`3` estop |
| `0x20` | Host -> MCU | `STATUS_REQ` | 状态请求 |
| `0x21` | Host -> MCU | `USB_CDC_PING` | CDC ping，保留给 FengMH |
| `0x80` | MCU -> Host | `STATE` | 整机状态 |
| `0x81` | MCU -> Host | `MOTOR_STATE` | 单电机轮询状态 |
| `0x82` | MCU -> Host | `LOG_MIRROR` | 日志镜像 |
| `0x83` | MCU -> Host | `IMU_STATE` | IMU 状态 |
| `0x84` | MCU -> Host | `USB_CDC_STATE` | CDC 状态 |
| `0x85` | MCU -> Host | `USB_CDC_PONG` | CDC pong |
| `0x86` | MCU -> Host | `ARM_FEEDBACK` | `u8 arm_state + f32 end_x + f32 end_y + f32 end_z + f32 theta1_rad` |
| `0x8F` | MCU -> Host | `EVENT` | 异常、模式切换、保护触发等事件 |

机械臂旧协议中的 `0x12 ARM_TARGET`、`0x13 ARM_PUMP`、`0x21 ARM_FEEDBACK` 与底盘现有 `GAIT_CMD`、`MIT_CMD`、`USB_CDC_PING` 冲突，迁移时必须同步修改 ROS2 上位机打包/解析。

## MCU 侧目标架构

```text
main.c
  -> app_init()
  -> app_tasks_create()
      -> t_safety
      -> t_comm
      -> t_chassis
      -> t_arm
      -> t_log
```

### 任务职责

| 任务 | 周期/优先级建议 | 职责 |
| --- | --- | --- |
| `t_safety` | 高优先级，快速轮询 | 急停、模式互锁、设备超时、错误降级 |
| `t_chassis` | 500 Hz | 只在 `ROBOT_MODE_NAV` 或底盘允许模式下输出底盘电机 |
| `t_arm` | 100 Hz 控制，50 Hz 反馈 | 常驻运行机械臂控制；只在 `ROBOT_MODE_ARM` 下消费新的 USB 目标/气泵 |
| `t_comm` | 解析回调 + 低频上报 | 统一协议解析、命令缓存、遥测发送 |
| `t_log` | 低优先级 | 日志输出 |

### 模式状态机

建议 MCU 本地维护：

```text
ROBOT_MODE_IDLE
ROBOT_MODE_NAV
ROBOT_MODE_ARM
ROBOT_MODE_ESTOP
ROBOT_MODE_ERROR
```

核心互锁规则：

- `NAV` 模式下接收并执行底盘速度，机械臂保持安全姿态或不输出新目标。
- `ARM` 模式下底盘速度强制清零，底盘控制维持 stand/hold，机械臂允许执行抓放。
- `ESTOP` 下底盘和机械臂均禁能或进入力矩安全态，气泵按现场安全策略处理。
- 任一关键设备反馈超时进入 `ERROR` 或降级到 `IDLE`，并通过 `EVENT` 上报。

模式切换建议由 ROS2 mission manager 发起，但 MCU 必须本地二次校验。

## MCU 侧模块迁移

### Device 层

新增或迁移：

```text
App/device/include/motor_damiao.h
App/device/src/motor_damiao.c
```

职责：

- 使用 `bsp_fdcan_send(BSP_FDCAN_3, ...)` 和 `bsp_fdcan_attach_rx(BSP_FDCAN_3, ...)`，不在 device/control 层直接依赖 HAL 句柄。
- 适配 DM4340、DM4310 MIT 控制帧、反馈解析、使能、失能、故障恢复。
- 与 `motor_registry` 绑定 `MOTOR_ID_ARM_J1..J4`。
- 明确达妙 CAN ID、方向、零偏、限位和力矩上限。

### Control 层

新增：

```text
App/control/src/arm/arm_control.c
App/control/src/arm/arm_motion.c
App/control/src/arm/arm_kinematics.c
App/control/src/arm/arm_gravity_comp.c
App/control/include/arm/*.h
```

保留机械臂原工程中的核心价值：

- 4 个达妙电机：J1/J2/J3 为 DM4340，J4 为 DM4310。
- 100 Hz 控制周期、50 Hz feedback。
- 分阶段安全运动：收臂、转底座、伸臂。
- 末端 fine tracking window。
- 轨迹限速、重力补偿、反馈新鲜度检查、bus-off 恢复。

需要从 HAL/裸循环调用改为纯控制层接口：

```text
arm_control_init()
arm_control_set_target(target_type, x, y, z)
arm_control_set_pump(pump_on)
arm_control_tick(dt_s, now_ms)
arm_control_get_feedback()
```

### App 层

新增：

```text
App/app/include/task_arm.h
App/app/src/task_arm.c
```

职责：

- 从 `task_comm` 读取最新机械臂目标和气泵命令。
- 常驻调用 `arm_control_tick()`；`ROBOT_MODE_ARM` 只 gate 新目标/气泵命令。
- 定时通过 `task_comm` 或公共发送接口输出 `ARM_FEEDBACK`。
- 暴露 host test 可调用的 step API。

## ROS2 侧目标架构

统一 ROS2 工作空间中保留导航、定位、识别、抓取模块，但串口只由一个节点持有。

```text
Nav2 / slot_nav_dispatcher
  -> /cmd_vel
  -> robot_serial_bridge
  -> MCU 0x10

detection_3d / arm_grasp_manager
  -> /arm/target, /arm/pump_cmd
  -> robot_serial_bridge
  -> MCU 0x11 / 0x14

MCU 0x80 / 0x81 / 0x86 / 0x8F
  -> robot_serial_bridge
  -> /robot/state, /motor/state, /arm/feedback, /robot/event
```

### `robot_serial_bridge`

建议新建统一串口桥节点：

- 独占打开 STM32 CDC 设备。
- 订阅 `/cmd_vel`、`/robot/mode_cmd`、`/arm/target`、`/arm/pump_cmd`。
- 发布 `/robot/state`、`/arm/feedback`、`/robot/event`。
- 负责帧打包、解析、超时、重连和发送节流。
- 按当前模式做上位机侧 gate：`NAV` 只发底盘，`ARM` 只发机械臂。

### `mission_manager`

建议新增任务编排节点，不让 navigation 和 arm bridge 互相直接依赖：

```text
IDLE
  -> NAV_TO_TARGET
  -> WAIT_NAV_RESULT
  -> SWITCH_TO_ARM
  -> ARM_GRASP
  -> ARM_DONE
  -> SWITCH_TO_NAV
  -> NEXT_TARGET / FINISH
  -> ERROR / ESTOP
```

职责：

- 调用 Nav2 `NavigateToPose` action。
- 到达目标后发布模式切换到 `ARM`。
- 允许机械臂识别和抓放状态机开始工作。
- 抓取完成后切回 `NAV` 或结束任务。
- 统一处理超时、取消、急停和恢复策略。

## 分阶段实施计划

### 阶段 1：协议修正和接口冻结

目标：

- 在 FengMH 中固化新的 FuncID 和 payload 结构。
- 机械臂旧协议功能号迁移到 `0x11/0x14/0x86`。
- 增加 host 单测验证帧打包、长度、解析和缓存。

验收：

- 旧底盘 `0x10/0x12/0x13` 单测不回退。
- 新 `ARM_TARGET`、`ARM_PUMP` 帧能被解析并缓存。
- 无未知功能号冲突，`git diff --check` 通过。

### 阶段 2：达妙电机驱动迁移

目标：

- 从 `damiao_new1` 迁移 DM4340/DM4310 协议打包和反馈解析。
- 改为基于 FengMH BSP FDCAN3。
- 接入 `motor_registry`。

验收：

- host 单测覆盖达妙 MIT 帧打包/反馈解析。
- 上板能单独使能/失能 J1-J4，能读到反馈。

### 阶段 3：机械臂控制层迁移

目标：

- 迁移 IK、轨迹、重力补偿、安全分阶段运动。
- 建立 `arm_control_tick()` 控制入口。
- 气泵控制从裸 GPIO 调用迁移到 device/BSP 适配。

验收：

- host 单测覆盖目标合法性、限位、模式切换、反馈超时。
- 上板能执行固定抓取/放置目标，不依赖 ROS2。

### 阶段 4：FreeRTOS 任务接入和模式互锁

目标：

- 新增 `t_arm`。
- 增加 MCU 本地 `robot_mode`。
- 底盘和机械臂输出互斥。

验收：

- `NAV` 模式下机械臂目标不触发输出。
- `ARM` 模式下底盘速度清零并保持站立。
- 急停能同时压住底盘和机械臂。

### 阶段 5：ROS2 串口桥统一

目标：

- 新建 `robot_serial_bridge`。
- 重构底盘 `cmd_vel_chassis_serial.py`，移除串口所有权。
- 重构机械臂 `arm_serial_bridge_node.py`，保留抓放状态机，改为 topic 输入输出。

验收：

- 单串口节点可同时解析底盘状态和机械臂反馈。
- 导航阶段只发送 `0x10`。
- 抓取阶段只发送 `0x11/0x14`。

### 阶段 6：任务编排和整机联调

目标：

- 新增或重构 `mission_manager`。
- 串联导航到点、切机械臂、识别抓取、更新 flag、回导航。
- 增加错误恢复和日志可观测性。

验收：

- 可以完整执行“导航到目标点 -> 抓取 -> 更新状态 -> 继续/结束”。
- 任一步失败都能停在明确状态，并可从日志判断原因。

## 关键待确认参数

- 机械臂实际电机数量、型号和 CAN ID。
- FDCAN3 波特率、滤波配置和总线接线。
- 气泵 GPIO、默认安全态和断电策略。
- J1-J4 方向、零位、软限位、最大速度、最大力矩。
- `arm_base` 坐标系方向是否与 ROS2 `vision_transform.py` 完全一致。
- 底盘和机械臂模式切换时，机械臂默认收纳姿态和底盘站立姿态。

## 主要风险和对策

| 风险 | 影响 | 对策 |
| --- | --- | --- |
| 功能号冲突 | 上位机命令被误解释，可能触发错误动作 | 先修协议并测试；ROS2 同步更新常量 |
| 串口多节点抢占 | 导航和机械臂随机丢包或打不开设备 | 单 `robot_serial_bridge` 独占串口 |
| 底盘和机械臂同时输出 | 机械结构和重心风险 | ROS2 gate + MCU 本地互锁双保险 |
| 达妙反馈超时或 bus-off | 机械臂失控或停止 | 驱动层反馈新鲜度、bus-off 恢复和错误上报 |
| 坐标系不一致 | 抓取偏差大 | 统一 `arm_base` 定义，feedback 中持续上报末端和 `theta1` |
| 一次性大合并 | 问题难定位 | 分阶段迁移，每阶段保留 host 单测和上板最小验收 |

## 近期工作顺序

1. 修正 FengMH 协议定义：`ARM_TARGET=0x11`、`ARM_PUMP=0x14`、`ROBOT_MODE_CMD=0x15`、`ARM_FEEDBACK=0x86`。
2. 在 `task_comm` 中增加机械臂目标/气泵命令缓存接口。
3. 增加 host 单测，验证新协议帧解析不影响底盘旧路径。
4. 更新 ROS2 机械臂协议常量，消除旧 `0x12/0x13/0x21`。
5. 开始迁移达妙电机驱动。
