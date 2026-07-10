# App 代码地图

本文档按文件说明当前 App 固件的主要职责，用来快速判断“应该先看哪个文件”。逐函数查找见 [`app_function_index.md`](app_function_index.md)。

## 启动和任务层

### `Core/Src/main.c`

CubeMX/HAL 主入口。默认 `USE_LEGACY_MAIN=0`，旧底盘死循环业务不参与启动；`main()` 完成 HAL、时钟、外设初始化后，在 RTOS kernel 初始化前调用 `app_init()`。

### `Core/Src/freertos.c`

CubeMX FreeRTOS 入口。`MX_FREERTOS_Init()` 创建 USB `defaultTask`，再调用 `app_tasks_create()` 创建 App 任务。

### `App/app/src/app_init.c`

板级应用初始化入口。初始化日志、时间、FDCAN、UART、USB CDC、SPI、GPIO、气泵、motor registry、GO/M3508/达妙电机和 BMI088；BMI088 初始化失败时降级运行。

### `App/app/src/app_tasks.c`

创建 App 任务。在 MCU 构建中创建 `t_log`、`t_safety`、`t_comm`、`t_chassis`、`t_arm`；host 构建只初始化模块，不创建线程。这里也保存 RTOS stack overflow 和 assert 诊断变量。

### `App/app/src/task_comm.c`

USB CDC 协议入口和遥测发送。

主要职责：

- `proto_frame` 增量解析 `55 AA | func | len | payload | checksum`。
- `proto_dispatch` 分发 `0x10/0x11/0x12/0x13/0x14/0x15`。
- 缓存底盘速度、机械臂目标、气泵命令、整机 mode 和最近有效接收时间。
- 拒绝机械臂 J1-J6 的 `0x13 MIT_CMD` 旁路写入。
- 在通信入口过滤机械臂 target 类型、有限值和明显单位/字节序错误。
- MCU 上发送 `0x80 STATE`、`0x81 MOTOR_STATE`；被 `task_arm` 调用时发送 `0x86 ARM_FEEDBACK`。

### `App/app/src/task_chassis.c`

底盘 RTOS 包装层。

主要职责：

- 读取 `task_comm` 底盘缓存。
- 本地二次执行 mode gate：无 mode 帧保留旧调试兼容；收到 mode 后仅 `ROBOT_MODE_NAV` 允许底盘速度、目标 yaw 和 steer mode 进入控制器。
- 以 500 Hz 调用 `chassis_control_tick()`。
- 保留 `task_chassis_*` 公开 API，转发到 `chassis_control`。

### `App/app/src/task_arm.c`

机械臂 RTOS 包装层。

主要职责：

- 机械臂 task 常驻，不随 mode 启停。
- 处理上电/非 ARM `PARK` 姿态、ARM 第一箱等待姿态、PLACE 完成后的第二箱等待姿态。
- 只有等待姿态到位并收到新的合法 `GRASP` 后，才释放固定姿态给上位机抓放流程。
- 只有 `ROBOT_MODE_ARM` 且已释放时，才把新的 USB target/pump 排进 `Arm_Serial_Protocol_QueueTarget/QueuePump()`；其他模式同步 seq 并丢弃旧命令。
- 支持编译期 `APP_ARM_FORCE_GRAVITY_ONLY` 和 Live Expressions `debug_arm_force_gravity_only` 纯重力保持。
- 急停时禁用控制器和达妙输出，不运行自动 enable。
- 50 Hz 发送 `0x86 ARM_FEEDBACK`，100 ms 更新一次 `g_arm_debug_snapshot`。

### `App/app/src/task_safety.c`

急停和安全监控任务。`MODE_CMD` 为 `ESTOP/ERROR` 时会置位急停并 disable 全部电机；任务本身 200 Hz 扫描电机反馈超时和温度。

### `App/app/src/task_log.c`

低优先级日志心跳任务占位。

## Control 控制层

### `App/control/src/chassis/chassis_control.c`

底盘主行为流水线。改底盘行为优先读这个文件。

当前职责：

- GO 上电预标定窗口。
- stand height ramp。
- BMI088 姿态估计和 yaw 闭环转向。
- `vx/vy/wz` 命令斜率限制。
- planner 调用、online/offline 判断、stand/trot/walk/script gait 选择。
- 支撑相轮速应用、姿态补偿、机械臂载荷补偿。
- 腿控制器派发和 MCU 电机输出 flush。

### `App/control/src/chassis/chassis_planner.c`

将速度命令转换成 planner 输出：

- `moving` 标志。
- `low_speed_turn` 标志。
- walk/trot 通用 gait 参数。
- `leg_step_length_m[4]` 每腿步长。
- `wheel_rads[4]` 每轮局部滚动速度。

`wz` 通过 `v_leg_x = vx - wz * y_leg` 映射到每条腿，默认 `|y_leg| = 0.15 m`。`vy` 目前只参与 moving 判断。

### `App/control/src/gait`

步态接口、默认参数和步态实现。

- `gait_machine.c`：当前 gait、目标 gait、过渡混合。
- `gait_stand.c`：静态站立。
- `gait_trot.c`：对角小跑，支持每腿步长。
- `gait_walk.c`：四拍三支撑 walk，当前用于低速/原地 yaw 转向。
- `gait_trajectory.c`：通用摆线足端轨迹。

### `App/control/src/kinematics/leg_ik.c`

二连杆腿部 IK/FK 和四腿镜像映射。输入是 gait 输出的 `foot_x_m/foot_z_m`，输出髋/膝关节角。

### `App/control/src/leg/leg_controller.c`

将 gait 输出转换为电机 vtable 调用：

- 调用 `leg_ik_solve_all()`。
- 根据支撑腿集合分配 payload 和 balance 前馈。
- 给 GO 髋/膝发送位置命令。
- 给 M3508 轮毂发送 MIT 位置/速度参考命令。

### `App/control/src/attitude`

姿态估计和 yaw 转向控制器。

- `attitude_estimator.c`：用 gyro 积分 yaw，用 accel 估计 roll/pitch，输出角速度。
- `steer_controller.c`：目标 yaw 到 yaw-rate 的 PID 控制。

### `App/control/src/arm/arm_control.c`

机械臂控制门面。

当前职责：

- 校验 target 类型、IK 和 GRASP J1 周期性禁区。
- 管理安全三段式 `RETRACT -> ROTATE_BASE -> EXTEND`。
- 支持 fine tracking 小范围在线重规划。
- 使用四关节同步五次轨迹。
- 优先用真实达妙反馈生成 measured FK；反馈缺失时按场景回退 planned sample 或报错。
- 计算 J2/J3/J4 重力补偿并输出达妙 MIT 命令。
- 无目标时用当前反馈角做重力保持。
- 维护 `arm_control_status_t`，供 `task_arm` 和底盘载荷补偿读取。

### `App/control/src/arm/arm_kinematics.c`

机械臂四关节 IK/FK，单位为 m/rad。保留 J2/J3 物理限位、J3/J4 固定关系和旧工程零偏。

### `App/control/src/arm/arm_motion.c`

四关节同步五次轨迹生成器。按速度/加速度限制估算总时长，支持运动中从当前 sample 连续重规划。

### `App/control/src/arm/arm_gravity_comp.c`

机械臂重力补偿模型。维护 L2/L3/末端/载荷质量和 COM 参数，泵开关会平滑切换末端 active mass。

### `App/control/src/arm/arm_legacy_compat.c`

旧 `damiao_new1` CamelCase API 兼容层。主链仍走 `task_arm -> arm_control`；兼容层用于旧测试、调试入口和旧单位转换。

### `App/control/src/arm/arm_vision_transform.c`

保留旧视觉坐标转换和新的米制 API。

### `App/control/src/script`

脚本步态播放器和 gait 包装。脚本可以通过 gait 接口输出关键帧步态。

## Device 设备层

### `App/device/src/motor_registry.c`

logical motor id 表和运行期设备绑定。硬件映射集中在这里：四腿 GO/M3508 和机械臂 J1-J4 达妙。

### `App/device/src/motor_go.c`

GO-8010 髋/膝电机驱动，使用 RS485/RIS 协议和 UART2/UART3/UART4/UART7。

### `App/device/src/motor_m3508.c`

M3508/C620 轮毂电机驱动，使用 FDCAN1/FDCAN2。支持电流、速度、位置、力矩和 MIT 模式；轮足主链默认走 MIT。

### `App/device/src/motor_damiao.c`

达妙 DM4340/DM4310 机械臂关节电机驱动，使用 FDCAN3。负责 MIT 打包、反馈解析、特殊 enable/disable/reset 帧、RX 路由和自动 enable 恢复。

### `App/device/src/arm_pump.c`

机械臂主气泵 device wrapper。主气泵为 `PD11`，上电默认关闭；辅助 `PC8/PC9/PA8/PA9` 只保留接口。

### `App/device/src/pump_control.c`

旧气泵 API 兼容层，包装到 `arm_pump` 并同步机械臂 payload 重力状态。

### `App/device/src/dm4310_posvel.c`

旧达妙 API 兼容层，包装到 `motor_damiao` 和 `motor_registry`。

### `App/device/src/imu_bmi088.c`

BMI088 初始化、寄存器读写、温度读取、量程配置和诊断状态。

## BSP 层

`App/bsp/src` 是 MCU 外设适配和 host mock：

- `bsp_fdcan.c`
- `bsp_uart.c`
- `bsp_spi.c`
- `bsp_usb_cdc.c`
- `bsp_gpio.c`
- `bsp_time.c`

## Service 层

### `App/service/src/protocol`

- `proto_frame.c`：整机协议帧解析/构造。
- `proto_dispatch.c`：FuncID 查表分发。
- `arm_serial_protocol.c`：旧机械臂 target/pump/place-cycle 协议兼容，不自动占用 USB。

### `App/service/src/pid`

通用 PID 实现。

## 测试层

### `host_tests`

host 侧回归测试覆盖 planner、walk/trot、IK、轮毂 MIT、姿态/载荷前馈、USB 协议、模式 gate、机械臂 IK/轨迹/重补/兼容层、task_arm 固定姿态、达妙打包/反馈/自动 enable、气泵和端到端任务链路。
