# App 代码地图

本文档按文件说明当前 App 固件的主要职责，用来快速判断“应该先看哪个文件”。逐函数查找见 [`app_function_index.md`](app_function_index.md)。

## App 任务层

### `App/app/src/app_init.c`

板级启动入口。初始化日志、时间、总线、电机注册表、电机和 IMU。

### `App/app/src/app_tasks.c`

创建应用任务。在 MCU 构建中，先初始化通信和底盘控制，再启动 FreeRTOS 任务。

### `App/app/src/task_comm.c`

USB CDC 协议输入和遥测输出。

主要职责：

- 解析协议帧
- 缓存最新底盘命令
- 缓存最新机械臂目标、气泵命令和整机模式命令
- 更新心跳时间
- 分发步态命令、底盘 MIT 调试命令、机械臂命令和模式命令；机械臂 J1-J6 拒绝 MIT 旁路写入
- 在 MCU 构建中发送状态和电机遥测

### `App/app/src/task_arm.c`

机械臂控制任务包装层。

主要职责：

- 读取 `task_comm` 缓存的机械臂目标、气泵和模式命令
- 机械臂按原工程三连调用常驻运行：`Arm_Control_Process()`、`Pump_Control_Process()`、`Arm_Serial_Protocol_Process()`
- J1-J4 反馈就绪后立即进入固定等待姿态，ARM 模式收到新 `GRASP` 后同拍交给原抓放流程
- 固定等待姿态不会因普通 4 秒 settling 超时清目标或退回重补；PLACE 完成后重新进入固定等待姿态
- 只在上位机请求 `ROBOT_MODE_ARM` 后把新的 USB 目标/气泵命令排进旧机械臂协议队列
- 唯一保留的机械臂测试态是纯重力补偿：可由编译期开关 `APP_ARM_FORCE_GRAVITY_ONLY` 固定开启，或在 Live Expressions 手动将 `debug_arm_force_gravity_only` 置 `1`
- 以 50 Hz 发送 `0x86 ARM_FEEDBACK`
- 每 100 ms 更新一次 `g_arm_debug_snapshot`，供 Live Expressions 单表达式查看常用机械臂数据
- MCU 固件达妙输出默认开启；只有等待首帧反馈、反馈丢失恢复和纯重补测试态使用重力保持

### `App/app/src/task_chassis.c`

`chassis_control` 的 RTOS 包装层。

主要职责：

- 将 `task_comm` 状态转换成 `chassis_control_input_t`
- 读取 `ROBOT_MODE_CMD`：无 mode 帧时兼容旧调试路径，`NAV` 才允许底盘速度输出
- 在 `ARM/IDLE/ESTOP/ERROR` 下本地清零底盘速度、目标 yaw 和转向模式
- 运行一个 500 Hz 控制周期
- 保留旧 `task_chassis_*` 公开 API 的转发包装

### `App/app/src/task_safety.c`

急停和安全监控任务。

## Control 控制层

### `App/control/src/arm/arm_control.c`

机械臂控制门面。当前阶段接收目标和气泵状态，运行 IK/FK、安全三段式、fine tracking、五次轨迹、重力补偿和基于真实反馈的 settling/REACHED；优先使用真实达妙反馈生成 `0x86`，并具备 J1-J4 达妙 MIT 输出路径。MCU 固件默认自动 enable 达妙；无目标时用当前反馈角做重力保持，USB 目标/泵命令只在 ARM 模式被消费。

### `App/control/src/arm/arm_kinematics.c`

机械臂四关节运动学。

主要职责：

- 维护 L2/L3 几何参数和旧工程零偏配置
- 将末端 `arm_base xyz` 目标转换为 J1-J4 关节角
- 检查 J2/J3 实测物理限位
- 计算 J3/J4 固定和关系
- 根据关节角计算末端位姿

### `App/control/src/arm/arm_motion.c`

机械臂四关节同步五次轨迹生成器。

主要职责：

- 根据每轴速度/加速度限制估算同步运动时长
- 生成四关节五次多项式轨迹
- 在 `tick(now_ms)` 风格调用中输出位置、速度和加速度
- 支持运动中重规划时传入当前 sample 保持连续

### `App/control/src/arm/arm_gravity_comp.c`

机械臂重力补偿模型。

主要职责：

- 维护旧工程大臂、小臂、末端和货物质量参数
- 维护旧工程质心参数和末端等效质心距离
- 根据 J2/J3/J4 几何角计算 tau2/tau3/tau4 重力前馈
- 抓取载荷切换时平滑过渡 active end mass

### `App/control/src/chassis/chassis_control.c`

主行为流水线。在线/离线策略、转向、步态选择、步态更新和电机派发都在这里分阶段执行。

改行为时优先读这个文件。

### `App/control/src/chassis/chassis_planner.c`

将速度命令转换成 planner 输出：

- `moving`
- 每腿 gait 参数
- 每轮局部滚动速度目标

`wz` 通过 `y_leg = 0.15 m` 映射成每条腿的局部前后速度：`v_leg_x = vx - wz * y_leg`。planner 将这个局部速度同时写入每腿步长和每轮滚动速度；轮速只在支撑相应用，用于匹配接触速度，不作为独立的纯轮差速转向方案。

`g_chassis_stride_cfg` 控制轮足步态周期的小范围调度：默认周期只从 `0.60 s` 缩短到 `0.50 s`，速度变化主要进入轮速和每腿步长；`g_chassis_turn_cfg` 继续控制低速转向的步态参数。

当前注意点：`vy` 参与 moving 判断，但还不是完整横向控制链路。

### `App/control/src/attitude`

姿态估计和 yaw 转向控制器。

- `attitude_estimator.c`：用 BMI088 gyro/accel 更新 yaw、roll、pitch 和角速度。yaw 仍用于航向闭环，roll/pitch 提供给底盘姿态力矩补偿。
- `steer_controller.c`：目标 yaw 到实际 `wz` 的 PID 控制器。

`chassis_control.c` 中的 `g_chassis_attitude_comp` 是姿态补偿调试入口，默认关闭；打开后把 roll/pitch 姿态误差转换成 `Mx/My`，再由腿控制器分配到支撑腿 `tau_ff`。

### `App/control/src/gait`

步态实现和步态切换：

- `gait_machine.c`：当前/目标步态和过渡混合
- `gait_stand.c`：静态站立
- `gait_trot.c`：trot 相位和足端轨迹

### `App/control/src/kinematics/leg_ik.c`

二连杆腿部 IK/FK 和四腿映射。

### `App/control/src/leg/leg_controller.c`

将 gait 输出转换成电机 vtable 调用：

- GO 髋/膝位置命令
- M3508 轮毂 MIT 位置/速度参考命令

轮毂默认全面使用 MIT 模式：支撑相积分轮角参考，摆动相保持当前轮角。

### `App/control/src/script`

脚本步态播放器和 gait 包装。脚本步态是当前维护功能，不是开发日志；脚本可以通过 gait 接口输出关键帧步态。

## Device 设备层

### `App/device/src/motor_registry.c`

logical motor id 表和运行期电机句柄绑定。

### `App/device/src/motor_go.c`

GO-8010 髋/膝电机驱动，使用 RS485/RIS 协议。

主要职责：

- 创建并绑定 GO 电机实例
- 将关节侧命令转换成电机协议单位
- 根据反馈锁存零偏
- 按 UART 总线发送命令帧

### `App/device/src/motor_m3508.c`

M3508/C620 轮毂电机驱动，使用 FDCAN。

主要职责：

- 创建并绑定轮毂电机实例
- 解码 C620 反馈
- 运行速度、位置、力矩、电流和 MIT 控制模式
- 打包并发送 DJI 电流帧

### `App/device/src/motor_damiao.c`

达妙 DM43xx 机械臂关节电机驱动，使用 FDCAN3。

主要职责：

- 创建并绑定 J1-J4 达妙电机实例
- 打包 DM4340/DM4310 MIT 控制帧
- 发送 enable、disable、reset fault 特殊帧
- 解析达妙反馈并更新统一 `motor_state_t`
- 将 FDCAN3 RX 帧路由到匹配的机械臂关节
- 按旧工程策略周期性重发 enable，恢复未使能或长时间无反馈的关节

### `App/device/src/arm_pump.c`

机械臂气泵 device wrapper。

主要职责：

- 上电默认关闭主气泵
- 通过 `bsp_gpio` 控制旧工程确认的主气泵 `PD11`
- 保留 `PC8/PC9/PA8/PA9` 辅助通道接口，但不主动初始化这些通道
- 给 `arm_control` 提供气泵状态查询和控制入口

### `App/device/src/imu_bmi088.c`

BMI088 初始化和传感器读取。

## BSP 层

### `App/bsp/src`

硬件适配和 host mock：

- `bsp_gpio.c`
- `bsp_fdcan.c`
- `bsp_uart.c`
- `bsp_usb_cdc.c`
- `bsp_spi.c`
- `bsp_time.c`

## Service 工具层

### `App/service/src/protocol`

协议帧解析器和功能号分发器。

### `App/service/src/pid`

可复用 PID 实现。

## 测试层

### `host_tests`

host 侧回归测试，覆盖当前可读控制链路。现有覆盖包括 planner 行为、gait 输出、IK、命令解析、`chassis_control_tick()` 到 stub 电机命令的直接链路、达妙 MIT 打包、FDCAN3 发送和反馈解析，以及主气泵 GPIO latch/mode gate。
