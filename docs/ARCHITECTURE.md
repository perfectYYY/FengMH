# 固件架构

本文档说明当前固件代码的分层、职责边界和依赖方向。新增代码或做重构前，先用这里判断代码应该放在哪一层。

## 分层结构

```text
App/app/          FreeRTOS 任务入口和应用生命周期
App/control/      机器人行为、步态、运动学、转向、腿部派发
App/device/       电机、IMU、设备注册表等设备驱动
App/bsp/          MCU 外设适配和 host mock
App/service/      协议、PID 等可复用工具
App/common/       配置、错误码、日志和公共基础类型
```

## 各层职责

### `App/app`

负责任务入口、协议缓存和应用生命周期。这里应保持很薄：

- 初始化模块
- 读取通信状态
- 调用控制周期
- 保留旧公开 API 的兼容包装

不要把步态数学、IK、电机协议细节或板级驱动逻辑放在这一层。

### `App/control`

负责机器人行为：

- 在线/离线决策
- 转向和 planner 状态
- 步态选择和步态更新
- 足端目标到关节目标的转换
- 腿和轮的命令派发

新的运动行为优先放在这一层，除非它只是某个具体设备协议的细节。

### `App/device`

负责具体设备和电机 vtable：

- logical motor id 到运行期设备的绑定
- GO-8010 RS485/RIS 协议
- M3508/C620 FDCAN 协议
- BMI088 设备行为

设备层可以做物理量和协议量之间的转换，但不应该决定高层步态策略。

### `App/bsp`

负责 MCU 外设访问和 host 侧 mock：

- UART / RS485
- FDCAN
- SPI
- USB CDC
- 时间

BSP 以上的代码应调用稳定的 BSP API，不直接依赖 HAL。

### `App/service`

负责不持有机器人行为状态的通用工具：

- 协议帧解析和分发
- PID 工具

## 依赖方向

推荐依赖方向如下：

```text
app -> control -> device -> bsp
app -> service
control -> service
device -> service/common
```

避免反向依赖。例如 `device/` 不应该调用 `task_chassis`，`bsp/` 不应该知道步态或电机注册表策略。

## 当前运行形态

```text
USB 命令
  -> task_comm
  -> task_chassis
  -> chassis_control
  -> chassis_planner
  -> gait_machine + 具体 gait
  -> leg_ik + leg_controller
  -> motor registry vtable
  -> GO / M3508 驱动
  -> UART / FDCAN BSP
```

更详细的运行链路见 [`firmware_control_flow.md`](firmware_control_flow.md)。
