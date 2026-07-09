# 机械臂下位机移植计划

本文档只描述机械臂从 `damiao_new1` 移植到 `/Users/code/FengMH` 的执行计划。整机系统的总整合方案见 [`integration_migration_plan.md`](integration_migration_plan.md)。

## 移植目标

把 `damiao_new1` 中已经验证过的机械臂能力迁移到 FengMH FreeRTOS 工程内，并满足：

- 下位机只有一个固件工程、一个 USB CDC 协议入口。
- 机械臂控制作为独立 `t_arm` 任务运行，不进入 `main.c` 裸循环。
- 机械臂和底盘由整机模式互锁，禁止行走时机械臂输出。
- 机械臂 ROS 上位机协议号与 MCU 保持一致。
- 每个阶段都可编译、可测试、可回退定位。

## 源工程能力盘点

`damiao_new1` 当前运行链路：

```text
main.c
  -> Pump_Control_Init()
  -> Arm_Control_Init(&hfdcan3)
  -> Arm_Serial_Protocol_Init()
  -> while(1)
       Arm_Control_Process()
       Pump_Control_Process()
       Arm_Serial_Protocol_Process()
       HAL_Delay(1)
```

需要保留的能力：

- 4 个达妙电机：J1/J2/J3 为 DM4340，J4 为 DM4310。
- FDCAN3 总线通信。
- 100 Hz 左右控制周期，50 Hz 左右反馈周期。
- `target_type + arm_base xyz` 抓取/放置目标。
- 末端实时反馈：`arm_state + end_xyz + theta1_rad`。
- 气泵开关控制。
- 安全三段式运动：收臂、旋转底座、展开。
- fine tracking、轨迹限速、重力补偿、反馈新鲜度检查、bus-off 恢复。

## 目标目录映射

| 源文件 | 目标位置 | 处理方式 |
| --- | --- | --- |
| `Core/Src/dm4310_posvel.c` | `App/device/src/motor_damiao.c` | 改成 `bsp_fdcan` 驱动，不直接依赖 HAL handle |
| `Core/Inc/dm4310_posvel.h` | `App/device/include/motor_damiao.h` | 改成 device 层接口 |
| `Core/Src/arm_kinematics.c` | `App/control/src/arm/arm_kinematics.c` | 保留算法，统一单位和命名 |
| `Core/Src/arm_motion.c` | `App/control/src/arm/arm_motion.c` | 保留轨迹规划，去掉裸循环依赖 |
| `Core/Src/gravity_comp.c` | `App/control/src/arm/arm_gravity_comp.c` | 保留重力补偿，参数集中配置 |
| `Core/Src/arm_control.c` | `App/control/src/arm/arm_control.c` | 拆出 tick 接口，调用 device 层电机 |
| `Core/Src/pump_control.c` | `App/device/src/arm_pump.c` | 主气泵封装为 device，GPIO 经 `bsp_gpio` 输出 |
| `Core/Src/arm_serial_protocol.c` | `App/app/src/task_comm.c` + `task_arm` | 不移植独立解析器，统一走 FengMH 协议层 |
| `Core/Src/main.c` | 不移植 | 生命周期由 `app_init/app_tasks` 接管 |

## 阶段 1：协议同步与通信边界

状态：已完成。

目标：

- MCU 统一功能号：
  - `0x11 ARM_TARGET`
  - `0x14 ARM_PUMP`
  - `0x15 ROBOT_MODE_CMD`
  - `0x86 ARM_FEEDBACK`
- `task_comm` 缓存机械臂目标、气泵和模式命令。
- ROS 机械臂上位机同步功能号、日志和测试。
- 禁止继续使用旧机械臂 `0x12/0x13/0x21`。

验收：

- FengMH host test 通过。
- FengMH MCU build 通过。
- `gsing_dog-main/src/detection_3d` 的协议测试通过。
- `rg "0x12|0x13|0x21"` 只在历史记录或底盘保留项中出现，不作为机械臂现行协议出现。

## 阶段 2：机械臂 App 接口骨架

状态：已完成安全空骨架。

目标：

- 新增 `task_arm.h/.c`。
- 新增 `arm_control.h/.c` 的最小接口，不立即输出电机。
- 暴露可测试接口：

```c
void task_arm_init(void);
void task_arm_entry(void* arg);
void task_arm_step_for_test(float dt_s, uint32_t now_ms);

app_err_t arm_control_init(void);
app_err_t arm_control_set_target(uint8_t target_type, float x_m, float y_m, float z_m);
app_err_t arm_control_set_pump(uint8_t pump_on);
app_err_t arm_control_tick(float dt_s, uint32_t now_ms);
void arm_control_get_feedback(payload_arm_feedback_t* out);
```

第一版行为：

- 能接收并锁存 `ARM_TARGET`。
- 能接收并锁存 `ARM_PUMP`。
- 能产生 `ARM_FEEDBACK`，状态先按 `IDLE/MOVING/REACHED/ERROR` 框架返回。
- 不发送任何达妙电机帧。

验收：

- host test 覆盖 `task_comm -> task_arm -> arm_control` 缓存链路。
- MCU build 通过。
- 上位机能收到合法 `0x86` feedback 框架。

## 阶段 3：达妙电机驱动移植

状态：驱动骨架已完成，硬件参数和上板动作验证待确认。

目标：

- 新增 `motor_damiao` device 驱动。
- 使用 `bsp_fdcan_send(BSP_FDCAN_3, ...)`。
- 使用 `bsp_fdcan_attach_rx(BSP_FDCAN_3, ...)` 接收反馈。
- 支持 DM4340/DM4310 MIT 控制、使能、失能、反馈解析、bus-off/error 状态。
- 绑定 `MOTOR_ID_ARM_J1..J4`。

已完成：

- 新增 `App/device/include/motor_damiao.h` 和 `App/device/src/motor_damiao.c`。
- J1/J2/J3 按 DM4340、J4 按 DM4310 初始化。
- 默认 CAN ID 沿用旧工程：J1=1、J2=2、J3=3、J4=4。
- `motor_registry.c` 已将 `MOTOR_ID_ARM_J1..J4` 绑定为 `MOTOR_DAMIAO`，J5/J6 暂保留为未使用占位。
- `app_init()` 已在 GO、M3508 初始化后调用 `motor_damiao_init_all()`。
- host stub 已支持 FDCAN TX 记录和 RX 注入，便于不接硬件回归达妙帧。
- FDCAN BSP 已在板上错误回调中累计错误计数，并对 `BUS_OFF` 调用 `HAL_FDCAN_Start()` 和重新注册通知完成恢复。
- `motor_damiao_process()` 已迁移旧工程自动 enable 策略：无反馈、反馈未使能且非硬错误、或反馈超过 2s 未更新时，按 200ms 检查周期和 500ms 单电机重试间隔发送 enable。
- 自动 enable 由常驻 `arm_control` 在 `motor_output_enabled` 打开时调用；MCU 固件默认打开，host 单测默认 dry-run。
- host test 已覆盖：
  - DM4340 MIT 中心值打包。
  - registry 绑定和 FDCAN3 发送。
  - 达妙反馈解析并更新 `motor_state_t`。
  - 达妙自动 enable 的周期和重试节流。
  - `arm_control` 输出 gate 对自动 enable 的保护。

需要确认：

- 达妙 CAN ID。
- FDCAN3 波特率和滤波配置。
- J1-J4 方向、零位、软限位、力矩上限。
- J5/J6 是否实际不存在；若不存在，registry 中保留但不绑定。
- 上板前确认 enable/disable/reset_fault 帧与当前达妙固件版本一致。
- 上板前确认反馈 ID 是否等于命令 ID；若实际反馈使用 master ID 或偏移 ID，需要调整 `motor_damiao_fdcan_rx_cb()` 的匹配逻辑。

验收：

- host test 覆盖达妙 MIT 帧打包、registry 绑定、FDCAN3 TX 和反馈解析。
- 上板能单独使能/失能 J1-J4，并读到实时反馈。
- 没有目标时不输出运动命令。

## 阶段 4：运动学、轨迹和重力补偿移植

状态：固件侧移植完成，待上板验证。`arm_kinematics`、`arm_motion`、`arm_gravity_comp` 和 `arm_control` 已迁入 FengMH，`arm_control` 已能完成 `xyz -> safe-stage/fine target -> quintic trajectory -> measured/planned feedback -> gravity tau`，并具备受 gate 保护的 J1-J4 达妙 MIT 输出路径。

补充：为方便后续机械臂开发，旧 `damiao_new1` 的 CamelCase API、视觉转换、旧气泵接口、旧达妙接口、旧单机械臂串口协议已作为兼容层迁入。对账表见 [/Users/code/FengMH/docs/arm_legacy_migration_audit.md](/Users/code/FengMH/docs/arm_legacy_migration_audit.md)。

目标：

- 迁移 `arm_kinematics`，统一所有外部接口使用 m/rad。
- 迁移 `arm_motion`，把原先过程式状态改成 `tick(dt_s, now_ms)`。
- 迁移 `arm_gravity_comp`，保留现场调参入口但收敛命名。
- 移植并验证安全三段式运动、settling/REACHED 和 fine tracking。

已完成：

- 新增 `App/control/include/arm/arm_kinematics.h` 和 `App/control/src/arm/arm_kinematics.c`。
- 新增 `App/control/include/arm/arm_motion.h` 和 `App/control/src/arm/arm_motion.c`。
- 新增 `App/control/include/arm/arm_gravity_comp.h` 和 `App/control/src/arm/arm_gravity_comp.c`。
- 机械臂几何参数使用 m/rad：L2=0.35 m，L3=0.30 m。
- 保留旧工程 J2/J3 物理限位、J3/J4 固定和关系、J2/J3 电机方向映射。
- 保留旧工程大臂/小臂/末端质量、质心参数和载荷质量平滑过渡。
- `arm_control_set_target()` 已做 IK、不可达/超限目标拦截、底座最近等效角选择。
- `arm_control_tick()` 已运行五次轨迹，并更新内部 planned end pose 和 gravity tau。
- `arm_control` 已从 `motor_registry` 绑定 ARM_J1-J4，并能读取达妙反馈生成真实末端 feedback；无目标但反馈新鲜时，会发送真实 `IDLE` 位姿并按原工程 gravity 模式做当前位置重力保持。
- 新增 `APP_ARM_MOTOR_OUTPUT_DEFAULT_ENABLE` 编译期开关；MCU 固件默认 `1`，host 单测默认 `0`。
- 输出 gate 打开时，必须 J1-J4 都已绑定且反馈新鲜，否则进入 `ERROR` 并返回 `APP_ERR_OFFLINE`，不发送 MIT 命令。
- 输出 gate 打开且反馈新鲜时，轨迹起点使用真实反馈，J2/J3 按旧工程方向映射，J2/J3/J4 带重力补偿前馈。
- 已接入安全三段式：大范围目标按 `RETRACT -> ROTATE_BASE -> EXTEND` 推进，小范围目标直接 fine move。
- 已迁移旧工程连续目标更新逻辑：收臂阶段只更新后续目标，旋转阶段跟随最新底座角，展开阶段大变化退回收臂，小变化直接重规划。
- 已迁移 fine tracking：三段式完成后进入 fine tracking，视觉小修正直接在线重规划，重复目标或死区内微小抖动不重启轨迹。
- 已迁移阶段超时恢复策略：`RETRACT/ROTATE_BASE` 超时重启当前安全阶段，`EXTEND` 超时根据当前误差选择 fine tracking 或重新收臂。
- 已接入基于真实反馈的 settling/REACHED：只有轨迹完成、反馈新鲜、关节误差和速度稳定达到阈值后才上报 `REACHED`。

待完成：

- 上板确认后打开 `APP_ARM_MOTOR_OUTPUT_DEFAULT_ENABLE` 或增加调试协议入口。
- 上板验证安全三段式实际方向、零位、限位和阈值。
- 将 FDCAN 错误计数、达妙 enable 重试计数接入后续诊断遥测或调试命令。

验收：

- host test 覆盖典型 `arm_base xyz -> joint target -> forward pose`。
- host test 覆盖五次轨迹生成和最终目标一致性。
- host test 覆盖重力补偿力矩方向/数量级和载荷质量平滑过渡。
- host test 覆盖 `arm_control` 不输出电机时只上报 `MOVING`、不上报 `REACHED`。
- host test 覆盖真实反馈优先的 `IDLE` feedback。
- host test 覆盖 host dry-run 输出 gate 默认关闭时不发电机命令。
- host test 覆盖输出 gate 打开且反馈新鲜时发送 J1-J4 位置/速度/前馈命令。
- host test 覆盖输出 gate 打开但反馈缺失时进入 `ERROR` 且不发命令。
- host test 覆盖真实反馈稳定后才上报 `REACHED`。
- host test 覆盖 `RETRACT -> ROTATE_BASE -> EXTEND -> REACHED` 三段式推进。
- host test 覆盖三段式过程中目标更新：收臂更新后续旋转目标、旋转阶段重规划底座角、展开阶段大变化退回收臂。
- host test 覆盖 fine tracking：重复目标不重规划，小范围视觉修正不进入安全三段式并能重新 `REACHED`。
- 初始非法/超限目标和反馈过期会进入错误状态并不上电机；连续跟踪中的偶发不可逆解会被拒绝但不打断正在执行的安全阶段或 fine tracking。
- 固定点抓取/放置可在无 ROS 情况下通过协议触发。

## 阶段 5：气泵控制移植

状态：主气泵已完成。旧工程确认的 `PD11` 作为主气泵输出，默认上电关闭；`PC8/PC9/PA8/PA9` 保留为辅助通道接口，但不会主动初始化，避免误占现有外设引脚。

目标：

- 把 `pump_control` 从裸 GPIO 改为 FengMH BSP/device 风格。
- 明确默认安全态。
- `ARM_PUMP` 只在 `ROBOT_MODE_ARM` 下生效。
- `ESTOP/ERROR` 下按安全策略关闭或保持气泵。

已完成：

- 新增 `App/bsp/include/bsp_gpio.h` 和 `App/bsp/src/bsp_gpio.c`。
- 新增 `App/device/include/arm_pump.h` 和 `App/device/src/arm_pump.c`。
- `app_init()` 上电初始化主气泵 GPIO 并默认关闭。
- `arm_control_set_pump()` 已调用 `arm_pump_set()` 控制主气泵，并同步 `arm_gravity_comp` 的空载/带载末端质量。
- `arm_control_set_enabled(0)` 会关闭主气泵并恢复空载重补；正常 `IDLE/NAV/ARM` 不会调用关闭，只用于 ESTOP/ERROR。
- `task_arm` 仍只在 `ROBOT_MODE_ARM` 下消费 `ARM_PUMP`；切出 ARM 模式后保持当前泵状态，新的非 ARM 泵命令会被同步 seq 并丢弃。

验收：

- host test 覆盖 pump device 默认关闭、GPIO latch 输出、payload mass 切换、协议命令缓存和模式 gate。
- 上板能通过 `0x14` 开/关气泵。

## 阶段 6：模式互锁

状态：固件侧完成，待 ROS mission manager 接入后做整机联调。

目标：

- MCU 本地维护整机模式：

```text
IDLE -> NAV -> ARM -> NAV/IDLE
任意 -> ESTOP
错误 -> ERROR
```

- `NAV`：只允许底盘速度输出，机械臂不执行新目标。
- `ARM`：底盘速度强制清零并保持站立，机械臂允许执行。
- `ESTOP/ERROR`：底盘和机械臂都进入安全输出。

已完成：

- `task_comm` 已接受 `IDLE/NAV/ARM/ESTOP/ERROR` 全部模式枚举。
- `task_arm` 常驻运行机械臂控制器，只在 `ROBOT_MODE_ARM` 下消费新的目标和气泵命令；切出 ARM 后机械臂保持当前状态，非 ARM 期间的新机械臂 USB 命令会被丢弃。
- `task_chassis` 读取 `ROBOT_MODE_CMD` 并做本地 gate：无 mode 帧时保持旧调试兼容；`NAV` 允许底盘速度；`ARM/IDLE/ESTOP/ERROR` 将底盘速度、目标 yaw 和转向模式清零，使底盘保持 stand/hold。

验收：

- host test 覆盖 `NAV` 模式下机械臂命令不执行。
- host test 覆盖 `ARM` 模式下底盘命令被 gate。
- 上位机误发混合命令时 MCU 本地仍安全。

## 阶段 7：ROS 串口桥重构

目标：

- 当前先同步 `detection_3d` 的机械臂功能号。
- 后续把机械臂 `arm_serial_bridge_node.py` 的串口所有权抽出，改成发布 `/arm/target`、`/arm/pump_cmd`，订阅 `/arm/feedback`。
- 新建统一 `robot_serial_bridge` 独占 STM32 CDC。
- 导航 `/cmd_vel` 和机械臂抓放都通过统一 bridge，根据 `/robot/mode` gate。

验收：

- 导航阶段只发 `0x10`。
- 抓取阶段只发 `0x11/0x14`。
- MCU feedback `0x86` 能被解析到 `/arm/feedback`。

## 阶段 8：整机任务状态机

目标：

在 ROS2 侧新增或重构 mission manager：

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

验收：

- 完整执行“导航到点 -> 切机械臂 -> 识别抓取 -> 更新 flag -> 回导航”。
- 任一步超时或失败都进入可观测错误状态。

## 当前执行顺序

1. 同步上位机机械臂协议功能号和测试。
2. 补齐下位机 `task_comm` 对 `ARM_FEEDBACK` 的发送接口。
3. 新增 `task_arm` 和 `arm_control` 空实现骨架。
4. 迁移达妙驱动。
5. 迁移机械臂运动学和控制。
   - 已完成 IK/FK、五次轨迹、重力补偿、真实反馈优先、受 gate 保护的达妙输出路径、安全三段式、fine tracking、阶段超时恢复和 settling/REACHED 判定。
   - 已完成主气泵 `PD11` device/BSP 接入。
   - 已完成达妙自动 enable 恢复和 FDCAN bus-off 错误计数/恢复接口。
6. 已完成模式互锁。
7. 下一步重构 ROS 统一串口桥和 mission manager。
