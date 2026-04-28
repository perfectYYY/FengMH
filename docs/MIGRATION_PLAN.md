# FengMH 迁移计划

> **版本**: 1.0 · **日期**: 2026-04-25  
> **目标**: 将旧业务代码 (Core/Src/) 完整迁移至新 App/ 分层架构，消除 USE_LEGACY 依赖

---

## 迁移差距总览

通过对 `Core/Src/` 旧代码与 `App/` 新代码的逐文件对比，识别出以下未迁移项目：

### 🔴 P0 — 阻塞项（电机无法运转）

| # | 旧文件 | 新目标 | 说明 | 状态 |
|---|--------|--------|------|------|
| 1 | `Core/Src/GO-motor.c` | `App/device/motor_go.c/h` | GO-8010 电机 vtable 实现：RIS 协议编解码 (modify_data/extract_data)、CRC16-CCITT、UART DMA 收发、MotorController_SetCommand/SendCommand、MotorInstance_Init/Update | ❌ 未迁移 |
| 2 | `Core/Src/M3508.c` | `App/device/motor_m3508.c/h` | M3508/C620 vtable 实现：0x200 控制帧、0x201-0x208 反馈帧、速度 PID 闭环、编码器多圈解算、温度保护、FDCAN 收发 | ❌ 未迁移 |
| 3 | `App/bsp/bsp_fdcan.c` (MCU端) | 同文件 | `bsp_fdcan_send()` MCU 端需要桥接 `HAL_FDCAN_AddMessageToTxFifoQ()`，当前返回 `APP_ERR_UNSUPPORTED`；RX 需要 `HAL_FDCAN_RxFifo0Callback` → `bsp_fdcan_on_rx()` | ❌ 未实现 |
| 4 | — | `App/bsp/bsp_uart.c/h` (新增) | GO 电机使用 UART/RS485 (USART2/USART3, 4Mbps)，当前 App 中无任何 UART BSP；需要 bsp_uart_init/send/attach_rx + RS485 DE 引脚控制 | ❌ 不存在 |

### 🟡 P1 — 功能缺失项（核心特性不完整）

| # | 旧文件 | 新目标 | 说明 | 状态 |
|---|--------|--------|------|------|
| 5 | `Core/Src/gait_plan.c` → `inverse_kinematics_position()` | `App/service/kinematics/leg_ik.c/h` | 2 连杆逆运动学：ORIGINAL/MIRROR 腿型区分、atan2 + acos 解算；**目录存在但为空** | ❌ 未迁移 |
| 6 | `Core/Src/gait_plan.c` → `forward_kinematics_position()` | `App/service/kinematics/leg_ik.c/h` | 正运动学：关节角→足端坐标 | ❌ 未迁移 |
| 7 | `Core/Src/gait_plan.c` → `params_init()` / `leg_size` | `App/service/kinematics/leg_params.c/h` | 腿尺寸/质量/质心/角度范围参数初始化；App 中无等价物 | ❌ 未迁移 |
| 8 | `Core/Src/main.c` → `GO_begin[12]` 零位校准 | `App/device/motor_go.c` 或独立模块 | GO 电机上电零位标定序列；App 中无等价物 | ❌ 未迁移 |
| 9 | — | `App/service/protocol/proto_defs.h` | 上行帧 0x80(整机状态)/0x81(电机状态) 定义存在但实现缺失；task_comm 中无构造和发送逻辑 | ❌ 未实现 |
| 10 | — | `App/app/task_safety.c` | 自动电机超时扫描(last_rx_tick > 100ms → 离线)、温度扫描(>80°C → 限功率, >85°C → 停机) 未实现 | ❌ 未实现 |
| 11 | `Core/Src/GO-motor.c` → `footForce` | `App/device/motor_go.c` | GO 电机足底力传感器数据提取；motor_state_t 中无 force 字段 | ❌ 未迁移 |

### 🟢 P2 — 增强项（后续里程碑）

| # | 说明 | 目标里程碑 |
|---|------|-----------|
| 12 | 达妙电机驱动 `motor_damiao.c/h` | M5 |
| 13 | 机械臂控制任务 `task_arm.c/h` | M5 |
| 14 | 机械臂逆运动学 `arm_ik.c/h` | M5 |
| 15 | LQR 平衡控制器 | M4 |
| 16 | 重力补偿 | M3/M4 |
| 17 | IMU 接口 `imu_if.h` + 状态估计 | M4 |
| 18 | 低通滤波器/卡尔曼滤波器 `filter/lpf.c, kalman.c` | M4 |
| 19 | 里程计 `state_est/odom.c` | M4 |
| 20 | CAN Bus-Off 自恢复 | M6 |

---

## 分阶段迁移计划

### 阶段一：底层打通（让电机转起来）

**目标**：P0 全部完成，实现 "上位机指令 → 步态输出 → 电机运动" 的闭环

**预计工作量**：3-5 天

#### Step 1.1: 实现 bsp_fdcan MCU 端绑定

**文件**：`App/bsp/bsp_fdcan.c`

**修改内容**：
```c
// 1. bsp_fdcan_send() MCU 分支：
//    - 根据 bus 索引获取 hfdcan1/hfdcan2 句柄
//    - 构造 FDCAN_TxHeaderTypeDef
//    - 调用 HAL_FDCAN_AddMessageToTxFifoQ()

// 2. 新增 bsp_fdcan_on_rx() 供 HAL 回调调用：
//    - 从 FDCAN_RxHeaderTypeDef + RxBuffer 构造 bsp_fdcan_frame_t
//    - 调用已注册的 s_rx[bus].cb()

// 3. bsp_fdcan_init() MCU 分支：
//    - 注册 HAL FDCAN 接收滤波器
//    - 启动 HAL_FDCAN_Start()
```

**依赖**：`Core/Src/fdcan.c` 中的 `hfdcan1`/`hfdcan2` 句柄需 extern 引用或通过 init 传入

**测试**：host 单测不变；MCU 端用 CAN 分析仪验证收发

#### Step 1.2: 新增 bsp_uart 模块

**新文件**：`App/bsp/bsp_uart.h` + `App/bsp/bsp_uart.c`

**接口设计**：
```c
typedef enum {
    BSP_UART_1 = 0,  // 调试串口 115200
    BSP_UART_2 = 1,  // RS485-1 (GO电机) 4Mbps, DE=PD4
    BSP_UART_3 = 2,  // RS485-2 (GO电机) 4Mbps, DE=PB14
    BSP_UART_BUS_MAX
} bsp_uart_bus_t;

typedef void (*bsp_uart_rx_cb_t)(bsp_uart_bus_t bus,
                                  const uint8_t* data, uint32_t len,
                                  void* user);

app_err_t bsp_uart_init(bsp_uart_bus_t bus);
app_err_t bsp_uart_attach_rx(bsp_uart_bus_t bus, bsp_uart_rx_cb_t cb, void* user);
app_err_t bsp_uart_send(bsp_uart_bus_t bus, const uint8_t* data, uint32_t len);
```

**MCU 实现**：
- `send()`: RS485 DE 拉高 → `HAL_UART_Transmit_DMA()` → TX完成回调 DE 拉低
- RX: `HAL_UARTEx_ReceiveToIdle_DMA()` + 回调 → `bsp_uart_on_rx()`
- Host: mock 实现（类似 bsp_fdcan 的 TX 队列 + RX 注入）

**测试**：新增 `test_bsp_uart.c`

#### Step 1.3: 实现 motor_go.c — GO-8010 vtable

**新文件**：`App/device/motor_go.h` + `App/device/motor_go.c`

**从 `Core/Src/GO-motor.c` 迁移并重构**：

| 旧函数 | 新映射 | 说明 |
|--------|--------|------|
| `modify_data()` | 内部 `go_encode_cmd()` | RIS 协议编码：0xFE 0xEE 帧头 + 位置/速度/力矩/kp/kd 量化 + CRC16 |
| `extract_data()` | 内部 `go_decode_fbk()` | RIS 协议解码：0xFD 0xEE 帧头 + CRC 校验 + 位置/速度/力矩/温度/力传感器反量化 |
| `MotorController_SetCommand()` | → `go_set_position()` vtable | 映射到 `motor_ops_t::set_position` |
| `MotorController_SendCommand()` | → `go_send_all()` | 遍历同总线 GO 电机，拼接 TX 帧，通过 bsp_uart_send 发送 |
| `MotorInstance_Init()` | → `go_enable()` vtable | 使能电机：mode=0 进入闭环 |
| `MotorInstance_Update()` | → `go_feed_rx()` vtable | UART 回调中解码反馈帧，更新 motor_state_t |
| `crc_ccitt()` | 内部 `go_crc16()` | CRC16-CCITT 计算 |
| — | → `go_disable()` vtable | mode=0 退出闭环 |
| — | → `go_reset_fault()` vtable | 清除故障 |

**关键设计决策**：
1. 每条 RS485 总线对应一个 `go_bus_ctx_t`：管理 TX 缓冲和 DMA 状态
2. 12 个 GO 电机共享 2 个 RS485 总线，通过 motor_registry 的 can_bus 字段区分
3. `feed_rx()` 在 UART 回调上下文被调用，只做解码+更新 state，不触发重发
4. TX 采用周期性轮询发送（1kHz，由 task_chassis 或独立 task_motor_tx 驱动）

**零位校准**：从旧 `GO_begin[12]` 迁移标定逻辑，在 `go_enable()` 后自动记录当前位置为零位

#### Step 1.4: 实现 motor_m3508.c — M3508/C620 vtable

**新文件**：`App/device/motor_m3508.h` + `App/device/motor_m3508.c`

**从 `Core/Src/M3508.c` 迁移并重构**：

| 旧函数 | 新映射 | 说明 |
|--------|--------|------|
| `M3508_SendCurrent()` | → `m3508_send_current()` | 构造 0x200 控制帧，通过 bsp_fdcan_send 发送 |
| `M3508_ConfigOneCan()` | → `m3508_init()` 内部 | FDCAN 滤波器配置（已由 bsp_fdcan_init 覆盖，此处只初始化电机状态） |
| `M3508_SpeedPidCalc()` | → `m3508_set_velocity()` vtable | 速度 PID 闭环，使用 app_pid_t |
| `get_motor_measure()` | → `m3508_feed_rx()` vtable | CAN RX 回调：解析 0x201-0x208 反馈帧，更新 motor_state_t |
| `set_motor_current_can1/can2()` | → `m3508_send_current()` | 统一为 bsp_fdcan_send 调用 |
| 编码器多圈解算 | 内部 `m3508_update_angle()` | 8191→连续角度 + 圈数累计 |

**关键设计决策**：
1. 4 个轮毂 M3508 分属 Bus1/Bus2，每条总线最多 4 个电机
2. 速度 PID 使用 `app_pid_t`（替代旧 `PID_Controller`），默认 Kp=3.0/Ki=0.3/Kd=0.0
3. 减速比 187:1 自动在 vtable 中处理：对外角度/速度已除以减速比
4. 温度保护：>80°C 限功率 50%，>85°C 调用 disable
5. `set_current()` 直接映射电流指令（无 PID），`set_velocity()` 走速度 PID

**测试**：新增 `test_motor_m3508.c`（host mock FDCAN 收发）

#### Step 1.5: 绑定 motor_registry + 端到端验证

**修改文件**：`App/app/app_init.c`

```c
app_err_t app_init(void) {
    // ... 现有初始化 ...
    bsp_uart_init(BSP_UART_2);
    bsp_uart_init(BSP_UART_3);

    // 创建电机实例并绑定到 registry
    motor_go_init_all();      // 初始化 GO 电机，注册 feed_rx 到 bsp_uart
    motor_m3508_init_all();   // 初始化 M3508，注册 feed_rx 到 bsp_fdcan

    // 绑定 motor_dev_t 到 registry 槽位
    for (int i = 0; i < MOTOR_ID_MAX; i++) {
        motor_dev_t* dev = motor_create_by_type(motor_get_cfg(i)->type);
        motor_registry_bind(i, dev);
    }
}
```

**端到端验证**：
1. 上位机发送 `0x55 0xAA 0x10 0x0C [vx=0.1, vy=0, wz=0] [checksum]`
2. task_comm 解析 → chassis_cmd → task_chassis 切入 trot
3. gait_machine 输出 gait_output_t → leg_controller_apply
4. motor_dev_t->ops->set_position (GO) / set_velocity (M3508)
5. bsp_uart_send / bsp_fdcan_send → 物理总线 → 电机响应

---

### 阶段二：运动学闭环（让腿动得对）

**目标**：P1 全部完成，步态输出经 IK 转换为正确的关节角度

**预计工作量**：2-3 天

#### Step 2.1: 迁移逆运动学/正运动学

**新文件**：`App/service/kinematics/leg_ik.h` + `App/service/kinematics/leg_ik.c`

**从 `Core/Src/gait_plan.c` 迁移**：

| 旧函数 | 新函数 | 说明 |
|--------|--------|------|
| `inverse_kinematics_position()` | `leg_ik_solve()` | 2 连杆 IK：输入足端(x,z)+腿参数+腿型 → 输出(θ1,θ2) |
| `forward_kinematics_position()` | `leg_fk_solve()` | 正运动学：输入(θ1,θ2)+腿参数 → 输出足端(x,z) |
| — | `leg_ik_solve_all()` | 批量对4条腿做 IK，输入 gait_output_t(足端位移) → 输出 gait_output_t(关节角) |

**重构要点**：
1. 去除 arm_math 依赖（旧代码用 arm_matrix），改用纯 float 运算
2. `leg_type_t` 枚举：LEG_TYPE_ORIGINAL / LEG_TYPE_MIRROR，与 motor_registry 中的 dir 对应
3. 输入/输出用简单的 float[2] 或 struct，不用 vector 矩阵

**测试**：新增 `test_leg_ik.c`

#### Step 2.2: 迁移腿尺寸参数

**新文件**：`App/service/kinematics/leg_params.h` + `App/service/kinematics/leg_params.c`

**从 `Core/Src/gait_plan.c` → `params_init()` 迁移**：

```c
typedef struct {
    float thigh_length;     // 大腿长度
    float shin_length;      // 小腿长度
    float link_length;      // 连杆长度
    float wheel_diameter;   // 轮直径
    float thigh_mass_1;     // 大腿质量1
    float thigh_mass_2;     // 大腿质量2
    float shin_mass;        // 小腿质量
    float link_mass;        // 连杆质量
    float wheel_mass;       // 轮质量
    float lc_t_m_1;         // 大腿质心距离1
    float lc_t_m_2;         // 大腿质心距离2
    float lc_s_m;           // 小腿质心距离
    float lc_l_m;           // 连杆质心距离
    float g;                // 重力加速度
    float angle1_max/min;   // 髋关节角度范围
    float angle2_max/min;   // 膝关节角度范围
} leg_dim_t;

extern const leg_dim_t LEG_DIM_DEFAULT;
```

#### Step 2.3: leg_controller 接入 IK

**修改文件**：`App/service/leg/leg_controller.c`

```c
app_err_t leg_controller_apply(leg_controller_t* lc, const gait_output_t* o) {
    gait_output_t ik_out;
    leg_ik_solve_all(o, &LEG_DIM_DEFAULT, &ik_out);  // 足端位移 → 关节角
    // ... 用 ik_out 替代 o 推送给电机 ...
}
```

此时 `gait_trot.c` 中 hip_rad/knee_rad 临时携带的 (dx, dz) 会被 IK 正确转换为关节角。

#### Step 2.4: GO 电机零位标定

**迁移位置**：`App/device/motor_go.c` 内部

逻辑：上电后使能电机 → 读取当前位置 → 记录为 zero_offset → 后续 set_position 的 pos 参数减去 zero_offset

---

### 阶段三：安全与上行（让系统可靠）

**目标**：补齐 P1 中剩余的安全/上行功能

**预计工作量**：2-3 天

#### Step 3.1: task_safety 自动扫描

**修改文件**：`App/app/task_safety.c`

```c
void task_safety_entry(void* arg) {
    for (;;) {
        uint32_t now = bsp_time_now_ms();
        for (uint32_t i = 0; i < motor_registry_count(); i++) {
            motor_dev_t* d = motor_get(i);
            if (!d || !d->state.online) continue;
            // 超时检测
            if (now - d->state.last_rx_tick > 100) {
                d->state.online = 0;
                LOGW("motor %u offline", i);
            }
            // 温度保护
            if (d->state.temperature_c > 85.0f) {
                if (d->ops->disable) d->ops->disable(d);
                LOGE("motor %u overtemp %.1fC", i, d->state.temperature_c);
            }
        }
        osDelay(5);  // 200Hz
    }
}
```

#### Step 3.2: 上行帧 0x80/0x81 实现

**修改文件**：`App/app/task_comm.c`

- `0x80` 整机状态：姿态/速度/电量/步态模式，100Hz 周期发送
- `0x81` 电机状态列表：18 个电机的 angle/velocity/torque/temperature/online，20Hz 发送
- 在 task_comm_entry 的循环中定时构造并发送

#### Step 3.3: motor_state_t 增加 force 字段

**修改文件**：`App/device/motor_if.h`

```c
typedef struct {
    // ... 现有字段 ...
    float    foot_force_n;    // GO 电机足底力传感器 (N)
} motor_state_t;
```

同步修改 `motor_go.c` 的 `go_decode_fbk()` 提取 `footForce`。

---

### 阶段四：清理与收尾

**目标**：关闭 USE_LEGACY，删除旧代码

**预计工作量**：1-2 天

#### Step 4.1: 关闭 USE_LEGACY

1. `config.h` 中 `USE_LEGACY` 改为 0
2. 确认 `Core/Src/main.c` 中 `#if USE_LEGACY_MAIN` 包裹的旧代码不再被编译
3. 移除 `Core/Src/` 中已迁移的旧文件引用

#### Step 4.2: 更新 CMakeLists

1. `cmake/firmware.cmake` 的 APP_SOURCES 列表添加新文件
2. Host test CMakeLists 添加新测试文件

#### Step 4.3: 删除旧文件

确认所有功能在新架构中正常工作后：
- 删除 `Core/Src/GO-motor.c/h`
- 删除 `Core/Src/M3508.c/h`
- 删除 `Core/Src/gait_plan.c/h`
- 删除 `Core/Src/pid.c/h`（已被 `App/service/pid/` 替代）

---

## 迁移依赖关系图

```
Step 1.1 (bsp_fdcan MCU) ──┐
Step 1.2 (bsp_uart)     ──┤
                            ├── Step 1.3 (motor_go) ──┐
                            ├── Step 1.4 (motor_m3508)─┤
                            │                          ├── Step 1.5 (registry bind + e2e)
                            │                          │
Step 2.1 (leg_ik)      ──────┤                          │
Step 2.2 (leg_params)  ──────┤── Step 2.3 (leg_ctrl+IK) │
                            │                          │
                            ├── Step 2.4 (零位标定)    │
                            │                          │
Step 3.1 (safety scan) ─────┤                          │
Step 3.2 (上行帧)       ─────┤                          │
Step 3.3 (force字段)   ─────┤                          │
                            │                          │
                            └── Step 4.x (清理收尾) ────┘
```

**关键路径**：Step 1.1 + 1.2 → 1.3 + 1.4 → 1.5 → 2.1 → 2.3 → 4.x

---

## 旧代码与新代码映射速查

| 旧文件 | 旧函数 | 新文件 | 新函数/接口 |
|--------|--------|--------|-------------|
| `GO-motor.c` | `modify_data()` | `motor_go.c` | `go_encode_cmd()` |
| `GO-motor.c` | `extract_data()` | `motor_go.c` | `go_decode_fbk()` |
| `GO-motor.c` | `MotorController_SetCommand()` | `motor_go.c` | `motor_ops_t::set_position` |
| `GO-motor.c` | `MotorController_SendCommand()` | `motor_go.c` | `go_send_all()` |
| `GO-motor.c` | `MotorInstance_Init()` | `motor_go.c` | `motor_ops_t::enable` |
| `GO-motor.c` | `MotorInstance_Update()` | `motor_go.c` | `motor_ops_t::feed_rx` |
| `GO-motor.c` | `crc_ccitt()` | `motor_go.c` | `go_crc16()` |
| `M3508.c` | `M3508_SendCurrent()` | `motor_m3508.c` | `m3508_send_current()` |
| `M3508.c` | `M3508_ConfigOneCan()` | `motor_m3508.c` | `m3508_init()` |
| `M3508.c` | `M3508_SpeedPidCalc()` | `motor_m3508.c` | `motor_ops_t::set_velocity` |
| `M3508.c` | `get_motor_measure()` | `motor_m3508.c` | `motor_ops_t::feed_rx` |
| `M3508.c` | `set_motor_current_can1/can2()` | `motor_m3508.c` | `m3508_send_current()` |
| `gait_plan.c` | `params_init()` | `leg_params.c` | `LEG_DIM_DEFAULT` 常量 |
| `gait_plan.c` | `inverse_kinematics_position()` | `leg_ik.c` | `leg_ik_solve()` |
| `gait_plan.c` | `forward_kinematics_position()` | `leg_ik.c` | `leg_fk_solve()` |
| `gait_plan.c` | `compete_gait()` | — | 已被 `gait_trot.c` 的相位发生器替代 |
| `gait_plan.c` | `gait_action_init()` | — | 已被 `gait_params_t` 替代 |
| `pid.c` | `PID_Update()` | `pid.c` | `app_pid_update_dt()` |
| `pid.c` | `PID_UpdateIncremental()` | `pid.c` | `app_pid_update_inc_dt()` |
| `main.c` | `HAL_UART_TxCpltCallback()` | `bsp_uart.c` | TX 完成 DMA 回调 |
| `main.c` | `HAL_UART_RxCpltCallback()` | `bsp_uart.c` | RX DMA 回调 |
| `main.c` | `Init_up_down()` | — | 已被 `script_builtin.c` 替代 |
| `main.c` | while(1) 主循环 | `task_chassis.c` | `task_chassis_entry()` |

---

## 注意事项

1. **RS485 半双工时序**：GO 电机 UART 为半双工，DE 引脚控制必须在 TX 前拉高、TX 完成后拉低，时序窗口约 10μs
2. **CAN 总线拥塞**：16 个电机 1kHz 全双工会达到 CAN 极限（1Mbps × 8B × 16 = 128KB/s），需要群发帧优化（0x200 一次控4个M3508）
3. **DMA 冲突**：USART2/USART3 的 DMA 通道不能与 ADC/SPI 等冲突，需在 CubeMX 中确认
4. **FreeRTOS 优先级**：CAN RX 回调优先级必须低于 `configMAX_SYSCALL_INTERRUPT_PRIORITY`，否则不能调用 FreeRTOS API
5. **测试先行**：每个 Step 完成后先跑 host 单测，再烧录 MCU 验证

---

*本文档由 AI 辅助生成，与源码同步于 2026-04-25。*
