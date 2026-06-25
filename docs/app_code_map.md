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
- 更新心跳时间
- 分发步态命令和 MIT 调试命令
- 在 MCU 构建中发送状态和电机遥测

### `App/app/src/task_chassis.c`

`chassis_control` 的 RTOS 包装层。

主要职责：

- 将 `task_comm` 状态转换成 `chassis_control_input_t`
- 运行一个 500 Hz 控制周期
- 保留旧 `task_chassis_*` 公开 API 的转发包装

### `App/app/src/task_safety.c`

急停和安全监控任务。

## Control 控制层

### `App/control/src/chassis/chassis_control.c`

主行为流水线。在线/离线策略、转向、步态选择、步态更新和电机派发都在这里分阶段执行。

改行为时优先读这个文件。

### `App/control/src/chassis/chassis_planner.c`

将速度命令转换成 planner 输出：

- `moving`
- trot 参数
- 共同前进轮速目标

`wz` 会转换成 trot 的 `turn_step_m`，由步态左右侧步长差完成 yaw 转向；轮子不再做差速转向。

当前注意点：`vy` 参与 moving 判断，但还不是完整横向控制链路。

### `App/control/src/attitude`

yaw 估计和 yaw 转向控制器。

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

### `App/device/src/imu_bmi088.c`

BMI088 初始化和传感器读取。

## BSP 层

### `App/bsp/src`

硬件适配和 host mock：

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

host 侧回归测试，覆盖当前可读控制链路。现有覆盖包括 planner 行为、gait 输出、IK、命令解析，以及 `chassis_control_tick()` 到 stub 电机命令的直接链路。
