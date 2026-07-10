# 固件架构

本文档说明当前固件代码的启动路径、分层职责和依赖方向。新增代码时先按这里判断应该放在哪一层。

## 启动形态

默认构建不走旧业务主循环：

```text
Core/Src/main.c
  HAL_Init / SystemClock_Config / MX_GPIO/FDCAN/SPI/TIM/UART
  -> app_init()
  -> osKernelInitialize()
  -> MX_FREERTOS_Init()
       -> app_tasks_create()
  -> osKernelStart()

Core/Src/freertos.c
  defaultTask -> MX_USB_DEVICE_Init()
  app_tasks_create() -> 创建 App 任务
```

`USE_LEGACY_MAIN=1` 只作为旧底盘代码回退入口；默认 `0`，避免旧 GO/M3508/PID 代码与 App 层任务竞写电机。

## 分层结构

```text
Core/             CubeMX/HAL 启动、外设实例、中断桥接
USB_DEVICE/       ST USB device 栈
App/app/          FreeRTOS 任务入口、命令缓存消费、应用生命周期
App/control/      机器人行为、运动学、步态、轨迹、补偿、输出派发
App/device/       电机、IMU、气泵、设备注册表
App/bsp/          UART/FDCAN/SPI/USB/GPIO/time 适配和 host mock
App/service/      协议、PID、旧机械臂协议兼容工具
App/common/       配置、错误码、日志、公共类型
host_tests/       host 侧回归测试
```

## 各层职责

### `Core/`

保留 CubeMX 生成的底层入口和 HAL handle。主链路只在这里完成 MCU 初始化、`app_init()` 调用、RTOS 启动和 USB device 初始化，不再放业务控制逻辑。

### `App/app`

任务包装层，保持薄：

- `app_init()` 初始化 BSP、设备、registry、IMU、气泵。
- `app_tasks_create()` 初始化通信、底盘、机械臂，并创建 FreeRTOS 任务。
- `task_comm` 解析 USB 协议并缓存最新命令。
- `task_chassis` 将通信缓存和 mode gate 转成 `chassis_control_input_t`。
- `task_arm` 维护机械臂常驻 task、固定等待姿态、ARM 模式命令释放、0x86 反馈。
- `task_safety` 维护急停、电机离线和过温保护。

不要把步态数学、IK、电机协议或 HAL 细节放进这一层。

### `App/control`

控制和算法层：

- 底盘：在线/离线策略、命令斜率限制、planner、walk/trot/stand/script、姿态补偿、机械臂载荷补偿。
- 腿：足端目标到二连杆 IK、支撑腿前馈力矩、M3508 轮毂 MIT 参考。
- 机械臂：host target 校验、J1 禁区、IK/FK、三段式安全运动、fine tracking、五次轨迹、重力补偿、达妙输出 gate。
- 姿态：BMI088 yaw/roll/pitch 估计和目标 yaw 转 `wz`。

控制层可以调用 device 层 vtable，但不直接调用 HAL。

### `App/device`

具体设备和电机协议：

- `motor_registry` 维护 logical motor id 到运行期设备的绑定和硬件映射。
- `motor_go` 负责 GO-8010 RS485/RIS 编解码和 UART 总线发送。
- `motor_m3508` 负责 DJI C620/M3508 反馈、速度/位置/力矩/电流/MIT 控制和 FDCAN 电流帧。
- `motor_damiao` 负责 J1-J4 达妙 MIT 帧、反馈、enable/disable/reset fault、FDCAN3 路由和自动 enable 恢复。
- `imu_bmi088`、`arm_pump` 提供传感器和气泵设备抽象。

设备层可以做协议量和物理量转换，但不决定高层步态或任务模式。

### `App/bsp`

硬件适配和 host mock：

- UART/RS485、FDCAN、SPI、USB CDC、GPIO、time。
- MCU 构建桥接 HAL；host 构建提供可注入的测试队列和 latch。

BSP 以上代码调用稳定 BSP API，不直接碰 HAL handle。

### `App/service`

不持有机器人行为状态的工具：

- `proto_frame` 和 `proto_dispatch` 是整机协议 parser/dispatcher。
- `arm_serial_protocol` 保留旧机械臂 target/pump/place-cycle 语义，但不接管 USB 主入口。
- `pid` 是通用 PID。

## 依赖方向

```text
Core -> App/app
App/app -> App/control + App/device + App/service + App/bsp
App/control -> App/device + App/service + App/common
App/device -> App/bsp + App/common
App/service -> App/common
```

禁止反向依赖：`device/` 不调用任务层，`bsp/` 不知道步态、电机 registry 策略或机械臂状态机。

## 主运行链路

```text
USB CDC
  -> task_comm
  -> task_chassis / task_arm
  -> chassis_control / arm_control
  -> leg_controller / motor_damiao / arm_pump
  -> motor vtable
  -> BSP UART/FDCAN/GPIO
```

更详细路径见 [`firmware_control_flow.md`](firmware_control_flow.md)。
