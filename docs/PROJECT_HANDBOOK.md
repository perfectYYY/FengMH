# FengMH 项目解释书

> **版本**: 1.0 · **日期**: 2026-04-25  
> **MCU**: STM32H723VGTX · **RTOS**: FreeRTOS (CMSIS-RTOS v2)  
> **项目**: 四组轮腿机器狗 + 机械臂 下位机控制系统

---

## 目录

1. [项目概述](#1-项目概述)
2. [架构总览](#2-架构总览)
3. [App/common — 通用基础设施](#3-appcommon--通用基础设施)
4. [App/bsp — 板级支持包层](#4-appbsp--板级支持包层)
5. [App/device — 设备抽象层](#5-appdevice--设备抽象层)
6. [App/service — 服务/算法层](#6-appservice--服务算法层)
7. [App/app — 应用任务层](#7-appapp--应用任务层)
8. [App/test — 单元测试](#8-apptest--单元测试)
9. [附录：关键数据流](#9-附录关键数据流)

---

## 1. 项目概述

FengMH 是一款基于 STM32H723VGTX 的四足轮腿机器狗下位机控制系统，搭载 6 自由度机械臂。项目采用 **分层架构**，将硬件驱动与业务逻辑完全解耦，使算法层可在 PC 上独立编译与单测。

### 核心设计原则

| 原则 | 实现方式 |
|------|---------|
| 驱动/应用完全解耦 | 分层目录 + 接口头文件 + 不允许反向依赖 |
| 可替换/可单测 | 所有外设访问通过 `bsp_*` 接口；`APP_TARGET_HOST` 宏切换 PC 编译 |
| 调试可观测 | 统一 `log.h` 宏 + tag 分级 + 可注册后端 |
| 步态可扩展 | 步态=策略模式 (`gait_if_t`) + 状态机调度 (`gait_machine_t`) |
| 上位机对接 | USB CDC + 0x55 0xAA 状态机协议 + FuncID 分发表 |

### 电机配置

| 类型 | 数量 | 用途 | 通信方式 |
|------|------|------|---------|
| 宇树 GO-8010 | 8 | 腿部髋/膝关节 | UART/RS485 (4Mbps) |
| DJI M3508 (C620) | 4 | 轮毂电机 | FDCAN (1Mbps) |
| 达妙 (DAMIAO) | 4 | 机械臂 J1-J4 | FDCAN3 (预留) |
| M3508 | 2 | 机械臂 J5-J6 | FDCAN3 (预留) |

---

## 2. 架构总览

```
┌─────────────────────────────────────────────────────────────┐
│ App Layer (tasks)                                           │
│  task_chassis  task_comm  task_safety  task_log             │
│  (task_arm 预留)                                            │
├─────────────────────────────────────────────────────────────┤
│ Service Layer (无硬件依赖, 可在 PC 上单测)                    │
│  gait/*  script/*  kinematics/*(空)  pid/*  protocol/*      │
├─────────────────────────────────────────────────────────────┤
│ Device Layer (电机/上位机 抽象接口)                           │
│  motor_if.h  motor_registry.c/h  (motor_m3508/go 待实现)    │
├─────────────────────────────────────────────────────────────┤
│ BSP / Port Layer (HAL 包装, 可替换/可 mock)                  │
│  bsp_fdcan.c  bsp_usb_cdc.c  bsp_time.c                    │
│  (bsp_uart 待新增)                                          │
├─────────────────────────────────────────────────────────────┤
│ ST HAL / CMSIS / FreeRTOS                                   │
└─────────────────────────────────────────────────────────────┘
```

**强约束**：上层头文件可被下层 include；下层严禁 include 上层；Service 层只 include 标准库 + 自己头文件（保证 PC 编译可行）。

### 编译目标

| 宏 | 含义 | 用途 |
|----|------|------|
| `APP_TARGET_HOST=1` | PC 主机构建 | 跑 Unity 单测，BSP 为 mock 实现 |
| `APP_TARGET_MCU=1` | STM32 固件构建 | 接入真实 HAL，FreeRTOS 多线程 |
| `USE_LEGACY=1` | 旧代码并行编译 | 过渡期保留旧 Core/Src 路径 |

---

## 3. App/common — 通用基础设施

### 3.1 config.h — 编译期开关

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `APP_TARGET_HOST` | 0 | 1=PC构建, 0=MCU构建 |
| `APP_TARGET_MCU` | `!APP_TARGET_HOST` | 自动取反 |
| `USE_LEGACY` | 1 | 旧代码并行编译开关 |
| `LOG_BACKEND_RTT` | 0 | SEGGER RTT 后端（暂未接入源码） |
| `LOG_BACKEND_STDIO` | `APP_TARGET_HOST` | host 构建自动启用 stdio 后端 |
| `LOG_DEFAULT_LEVEL` | 3 (INFO) | 日志默认级别 |
| `PROTO_MAX_PAYLOAD` | 64 | 协议帧最大 payload 字节数 |

### 3.2 err.h — 统一错误码

```c
typedef enum {
    APP_OK              = 0,
    APP_ERR_GENERIC     = -1,   // 通用错误
    APP_ERR_NULL_PTR    = -2,   // 空指针
    APP_ERR_INVALID_ARG = -3,   // 参数非法
    APP_ERR_TIMEOUT     = -4,   // 超时
    APP_ERR_NOT_FOUND   = -5,   // 未找到
    APP_ERR_BUSY        = -6,   // 忙
    APP_ERR_NO_MEM      = -7,   // 内存不足
    APP_ERR_CHECKSUM    = -8,   // 校验和错误
    APP_ERR_PROTO       = -9,   // 协议错误
    APP_ERR_OFFLINE     = -10,  // 离线
    APP_ERR_OVERFLOW    = -11,  // 溢出
    APP_ERR_UNSUPPORTED = -12,  // 不支持
    APP_ERR_UNINIT      = -13   // 未初始化
} app_err_t;
```

### 3.3 types.h — 通用基础类型

| 类型/宏 | 定义 | 说明 |
|---------|------|------|
| `app_tick_t` | `uint32_t` | 统一时间戳类型（ms） |
| `ARRAY_SIZE(a)` | `sizeof(a)/sizeof(a[0])` | 数组元素计数 |
| `UNUSED_ARG(x)` | `((void)(x))` | 消除未使用参数警告 |

### 3.4 log.h / log.c — 统一日志系统

**设计目标**：tag 分级 + 多后端注册 + 运行期可调级别，M1 阶段的简化实现（非线程安全，后续由 task_log 接管）。

#### 类型

| 类型 | 说明 |
|------|------|
| `log_lvl_t` | 日志级别枚举：ERR(1)/WARN(2)/INFO(3)/DBG(4)/TRACE(5) |
| `log_backend_fn` | 后端回调函数指针 `void (*)(log_lvl_t, const char* tag, const char* msg)` |

#### 函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `log_init` | `void log_init(void)` | 初始化日志系统：清零后端列表、tag规则表、全局级别设为 LOG_DEFAULT_LEVEL；若 `LOG_BACKEND_STDIO=1` 则自动注册 stdio 后端 |
| `log_register_backend` | `void log_register_backend(log_backend_fn fn)` | 注册后端回调（最多4个），每条日志分发到所有已注册后端 |
| `log_set_global_level` | `void log_set_global_level(log_lvl_t lvl)` | 设置全局过滤级别 |
| `log_get_global_level` | `log_lvl_t log_get_global_level(void)` | 获取当前全局级别 |
| `log_set_level` | `void log_set_level(const char* tag, log_lvl_t lvl)` | 为指定 tag 设定独立过滤级别（最多16条规则），未设定则用全局级别 |
| `log_emit` | `void log_emit(log_lvl_t lvl, const char* tag, const char* fmt, ...)` | 内部分发入口：先查级别过滤，再 vsnprintf 到 256B 栈缓冲，保存副本到 `s_last_line`，最后分发到所有后端 |
| `log_last_line` | `const char* log_last_line(void)` | 返回最后一条日志内容副本，供 host 单测断言 |

#### 日志宏

```c
#define LOGE(fmt, ...) log_emit(LOG_LVL_ERR,   TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) log_emit(LOG_LVL_WARN,  TAG, fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) log_emit(LOG_LVL_INFO,  TAG, fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) log_emit(LOG_LVL_DBG,   TAG, fmt, ##__VA_ARGS__)
#define LOGT(fmt, ...) log_emit(LOG_LVL_TRACE, TAG, fmt, ##__VA_ARGS__)
```

#### 内部实现细节

- `stdio_backend()`: host 默认后端，`fprintf(stdout, "[%s][%s] %s\n")` 格式输出
- `lookup_tag_level()`: 遍历 `s_tag_rules[]` 查找 tag 对应级别，未找到返回全局级别
- `s_last_line[]`: 256B 静态缓冲，保存最近一条日志（用于 host 测试 `log_last_line()`）

---

## 4. App/bsp — 板级支持包层

BSP 层将 HAL 与上层隔离，host 构建提供 mock 实现，MCU 构建桥接到真实 HAL。

### 4.1 bsp_time.h / bsp_time.c — 系统时间抽象

| 函数 | 签名 | 说明 |
|------|------|------|
| `bsp_time_init` | `void bsp_time_init(void)` | Host: 记录 CLOCK_MONOTONIC 基准时间戳；MCU: 空操作（HAL_Init 已初始化 SysTick） |
| `bsp_time_now_ms` | `app_tick_t bsp_time_now_ms(void)` | Host: `(now_ns - base_ns)/1e6 + fake_offset`；MCU: `HAL_GetTick()` |
| `bsp_time_now_us` | `uint64_t bsp_time_now_us(void)` | Host: ns级精度；MCU: `HAL_GetTick() * 1000`（暂为 ms 精度，后续接 DWT） |
| `bsp_time_delay_ms` | `void bsp_time_delay_ms(uint32_t ms)` | Host: `nanosleep()`；MCU: `HAL_Delay()` |
| `bsp_time_test_advance_ms` | `void bsp_time_test_advance_ms(uint32_t ms)` | Host 专用：手动推进 fake_offset_ms，用于单测控制时间流逝；MCU: 空操作 |

### 4.2 bsp_fdcan.h / bsp_fdcan.c — FDCAN 抽象接口

#### 类型

| 类型 | 说明 |
|------|------|
| `bsp_fdcan_bus_t` | 总线枚举：`BSP_FDCAN_1`/`BSP_FDCAN_2`/`BSP_FDCAN_BUS_MAX` |
| `bsp_fdcan_frame_t` | CAN 帧结构：`can_id`(标准帧ID) + `dlc`(0..8) + `data[8]` + `rx_tick`(接收时间戳) |
| `bsp_fdcan_rx_cb_t` | RX 回调：`void (*)(bsp_fdcan_bus_t, const bsp_fdcan_frame_t*, void* user)` |

#### 函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `bsp_fdcan_init` | `app_err_t bsp_fdcan_init(void)` | 初始化：清零 RX 回调槽 + Host TX 队列 |
| `bsp_fdcan_attach_rx` | `app_err_t bsp_fdcan_attach_rx(bsp_fdcan_bus_t bus, bsp_fdcan_rx_cb_t cb, void* user)` | 为指定总线注册 RX 回调 |
| `bsp_fdcan_send` | `app_err_t bsp_fdcan_send(bsp_fdcan_bus_t bus, const bsp_fdcan_frame_t* f)` | **⚠️ MCU 端返回 `APP_ERR_UNSUPPORTED`**（HAL 绑定未实现）；Host: 推入 64 深度环形 TX 队列 |
| `bsp_fdcan_test_tx_count` | `uint32_t bsp_fdcan_test_tx_count(bsp_fdcan_bus_t bus)` | Host: 返回 TX 队列中待读帧数 |
| `bsp_fdcan_test_pop_tx` | `app_err_t bsp_fdcan_test_pop_tx(bsp_fdcan_bus_t bus, bsp_fdcan_frame_t* out)` | Host: 从 TX 队列弹出一帧 |
| `bsp_fdcan_test_inject_rx` | `void bsp_fdcan_test_inject_rx(bsp_fdcan_bus_t bus, const bsp_fdcan_frame_t* f)` | Host: 模拟接收到一帧，触发 RX 回调 |
| `bsp_fdcan_test_reset` | `void bsp_fdcan_test_reset(void)` | Host: 清空 TX 队列 |

**关键缺口**：MCU 端 `bsp_fdcan_send()` 需要桥接到 `HAL_FDCAN_AddMessageToTxFifoQ()`，这是电机驱动的阻塞项。

### 4.3 bsp_usb_cdc.h / bsp_usb_cdc.c — USB CDC 抽象

#### 函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `bsp_usb_cdc_init` | `app_err_t bsp_usb_cdc_init(void)` | 初始化：清零 RX 回调 + Host TX 缓冲 |
| `bsp_usb_cdc_attach_rx` | `app_err_t bsp_usb_cdc_attach_rx(bsp_usb_rx_cb_t cb, void* user)` | 注册 RX 回调 |
| `bsp_usb_cdc_send` | `app_err_t bsp_usb_cdc_send(const uint8_t* data, uint32_t len)` | MCU: 调用 `CDC_Transmit_HS()`；Host: 追加到 512B TX 缓冲 |
| `bsp_usb_cdc_on_rx` | `void bsp_usb_cdc_on_rx(const uint8_t* data, uint32_t len)` | 由 `usbd_cdc_if.c` 在 `CDC_Receive_HS` 中调用，将字节灌入 RX 回调 |
| `bsp_usb_cdc_test_inject_rx` | `void bsp_usb_cdc_test_inject_rx(const uint8_t* data, uint32_t len)` | Host: 模拟 USB 接收，直接调用 `bsp_usb_cdc_on_rx` |
| `bsp_usb_cdc_test_tx_size` | `uint32_t bsp_usb_cdc_test_tx_size(void)` | Host: 返回 TX 缓冲已用字节数 |
| `bsp_usb_cdc_test_read_tx` | `uint32_t bsp_usb_cdc_test_read_tx(uint8_t* out, uint32_t max_len)` | Host: 读取 TX 缓冲内容 |
| `bsp_usb_cdc_test_reset` | `void bsp_usb_cdc_test_reset(void)` | Host: 清空 TX 缓冲 |

---

## 5. App/device — 设备抽象层

### 5.1 motor_if.h — 统一电机抽象（vtable）

所有电机类型对上层只暴露一个统一接口，通过 vtable 实现多态。

#### 类型

| 类型 | 说明 |
|------|------|
| `motor_type_t` | 电机类型枚举：`MOTOR_UNKNOWN`/`MOTOR_M3508`/`MOTOR_GO`/`MOTOR_DAMIAO` |
| `motor_state_t` | 电机运行状态：id/type/can_bus/can_id/online/angle_rad/velocity_rads/torque_nm/temperature_c/last_rx_tick/rx_cnt/err_cnt |
| `motor_ops_t` | vtable 操作集：set_current/set_torque/set_position/set_velocity/enable/disable/reset_fault/feed_rx |
| `motor_dev_t` | 电机设备：state + ops指针 + drv_ctx(驱动私有上下文) |

#### vtable 函数说明

| 操作 | 签名 | 说明 |
|------|------|------|
| `set_current` | `int (*)(motor_dev_t*, float iq_a)` | 电流控制（M3508 用） |
| `set_torque` | `int (*)(motor_dev_t*, float tau_nm)` | 力矩控制（GO/达妙用） |
| `set_position` | `int (*)(motor_dev_t*, float pos, float vel, float kp, float kd, float tau_ff)` | 阻抗控制（位置+速度+刚度+阻尼+前馈力矩） |
| `set_velocity` | `int (*)(motor_dev_t*, float vel_rads)` | 速度控制 |
| `enable` | `int (*)(motor_dev_t*)` | 使能电机 |
| `disable` | `int (*)(motor_dev_t*)` | 失能电机 |
| `reset_fault` | `int (*)(motor_dev_t*)` | 清除故障 |
| `feed_rx` | `int (*)(motor_dev_t*, const uint8_t* data, uint8_t dlc)` | 喂入接收数据（ISR/回调上下文调用） |

#### 辅助函数

| 函数 | 说明 |
|------|------|
| `motor_op_unsupported(motor_dev_t* dev, ...)` | 默认"不支持"实现，避免 vtable NULL 解引用 |

### 5.2 motor_registry.h / motor_registry.c — 电机注册表

通过"逻辑名 → 物理总线/CAN ID/类型"映射，应用层只拿 `motor_dev_t*` 操作。

#### 类型

| 类型 | 说明 |
|------|------|
| `motor_logical_id_t` | 逻辑ID枚举：FL_HIP/FL_KNEE/FL_WHEEL/FR_HIP/.../ARM_J1..J6，共18个槽位 |
| `motor_cfg_t` | 配置项：logical/type/can_bus/can_id/dir/zero_offset/limit_min/limit_max/name |

#### 函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `motor_registry_init` | `app_err_t motor_registry_init(void)` | 初始化：清零所有 `s_devs[]` 槽位（所有 motor_dev_t* 设为 NULL） |
| `motor_get` | `motor_dev_t* motor_get(motor_logical_id_t id)` | 按逻辑ID获取已绑定的 motor_dev_t 指针（未绑定为 NULL） |
| `motor_get_cfg` | `const motor_cfg_t* motor_get_cfg(motor_logical_id_t id)` | 按逻辑ID获取编译期配置项（总线/CAN ID/方向/限位等） |
| `motor_registry_count` | `uint32_t motor_registry_count(void)` | 返回总槽位数（18） |
| `motor_registry_bind` | `app_err_t motor_registry_bind(motor_logical_id_t id, motor_dev_t* dev)` | 运行期绑定 motor_dev_t 到指定逻辑ID，同时从配置项填充 state 元数据 |

#### 静态配置表 `s_cfg[18]`

| 腿 | 髋 | 膝 | 轮 |
|----|-----|-----|-----|
| FL | GO, Bus1, dir=+1 | GO, Bus1, dir=+1 | M3508, Bus1, 0x201, dir=+1 |
| FR | GO, Bus1, dir=-1 | GO, Bus1, dir=-1 | M3508, Bus1, 0x202, dir=-1 |
| RL | GO, Bus2, dir=+1 | GO, Bus2, dir=+1 | M3508, Bus2, 0x203, dir=+1 |
| RR | GO, Bus2, dir=-1 | GO, Bus2, dir=-1 | M3508, Bus2, 0x204, dir=-1 |
| ARM | DAMIAO J1-J4, Bus3 | M3508 J5-J6, Bus3 | — |

> ⚠️ 当前所有 `s_devs[]` 为 NULL，尚无 motor_dev_t 实例被绑定。

---

## 6. App/service — 服务/算法层

### 6.1 service/pid — PID 控制器

#### pid.h / pid.c — 位置式 + 增量式 PID

**类型 `app_pid_t`**:

| 字段 | 类型 | 说明 |
|------|------|------|
| Kp, Ki, Kd | float | PID 增益 |
| integral | float | 积分累计 |
| prev_error | float | 上次误差 |
| prev_prev_error | float | 上上次误差（增量式用） |
| prev_measurement | float | 上次测量值（d-on-measurement） |
| output | float | 当前输出 |
| output_max, output_min | float | 输出限幅 |
| integral_max, integral_min | float | 积分限幅（anti-windup） |
| last_time_ms | uint32_t | 上次计算时间戳 |
| initialized | uint8_t | 首帧初始化标志 |

#### 函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `app_pid_init` | `void app_pid_init(p, Kp, Ki, Kd, out_max, out_min, i_max, i_min)` | 初始化增益和限幅，内部调用 reset |
| `app_pid_reset` | `void app_pid_reset(app_pid_t* p)` | 清零 integral/prev_error/prev_prev_error/prev_measurement/output/last_time_ms |
| `app_pid_set_tunings` | `void app_pid_set_tunings(p, Kp, Ki, Kd)` | 运行期修改增益 |
| `app_pid_set_output_limits` | `void app_pid_set_output_limits(p, max, min)` | 修改输出限幅并 clamp 当前输出 |
| `app_pid_set_integral_limits` | `void app_pid_set_integral_limits(p, max, min)` | 修改积分限幅并 clamp 当前积分 |
| `app_pid_update_dt` | `float app_pid_update_dt(p, setpoint, measurement, dt_s)` | 位置式 PID：d-on-measurement，dt 由调用者提供；自动 clamp dt 到 [1μs, 500ms] |
| `app_pid_update_ms` | `float app_pid_update_ms(p, setpoint, measurement, now_ms)` | 位置式 PID：自动从时间戳差计算 dt |
| `app_pid_update_inc_dt` | `float app_pid_update_inc_dt(p, setpoint, measurement, dt_s)` | 增量式 PID：Δoutput = Kp·Δe + Ki·e·dt + Kd·Δ²e/dt |
| `app_pid_update_inc_ms` | `float app_pid_update_inc_ms(p, setpoint, measurement, now_ms)` | 增量式 PID：自动从时间戳差计算 dt |

#### 内部辅助函数

| 函数 | 说明 |
|------|------|
| `clamp_f(v, lo, hi)` | 浮点限幅 |
| `sort_limits(hi, lo)` | 确保 hi ≥ lo，否则交换 |
| `normalize_dt(dt_s)` | 将 dt 限制在合理范围 [1μs, 500ms]，异常值回退到 10ms |
| `dt_from_ms(p, now_ms)` | 从毫秒时间戳计算 dt，首帧回退到默认 10ms |

---

### 6.2 service/protocol — 通信协议栈

#### proto_defs.h — 协议常量与 Payload 结构体

帧格式: `0x55 0xAA | FuncID(1) | Len(1) | Payload(Len) | Checksum(1)`

| 常量 | 值 | 说明 |
|------|-----|------|
| `PROTO_HEAD1` | 0x55 | 帧头字节1 |
| `PROTO_HEAD2` | 0xAA | 帧头字节2 |
| `PROTO_FUNC_CHASSIS_CMD` | 0x10 | 下行：底盘速度指令 |
| `PROTO_FUNC_ARM_CMD` | 0x11 | 下行：机械臂指令（预留） |
| `PROTO_FUNC_GAIT_CMD` | 0x12 | 下行：步态指令（预留） |
| `PROTO_FUNC_STATUS_REQ` | 0x20 | 下行：状态请求（预留） |
| `PROTO_FUNC_STATE` | 0x80 | 上行：整机状态（预留） |
| `PROTO_FUNC_MOTOR_STATE` | 0x81 | 上行：电机状态列表（预留） |
| `PROTO_FUNC_LOG_MIRROR` | 0x82 | 上行：日志镜像（预留） |
| `PROTO_FUNC_EVENT` | 0x8F | 上行：错误/事件（预留） |

| 结构体 | 字段 | 说明 |
|--------|------|------|
| `payload_chassis_cmd_t` | vx, vy, wz (float) | FuncID 0x10, len=12 |
| `payload_status_req_t` | req_kind (uint8_t) | FuncID 0x20, len=1 |

#### proto_frame.h / proto_frame.c — 帧解析状态机

状态机：`IDLE → HEAD2 → FUNC → LEN → PAYLOAD → CKSUM`，任意步骤失败回 IDLE。

##### 类型

| 类型 | 说明 |
|------|------|
| `proto_frame_state_t` | 状态枚举：PF_ST_IDLE/HEAD2/FUNC/LEN/PAYLOAD/CKSUM |
| `proto_frame_t` | 解析结果帧：func_id + len + payload[64] |
| `proto_frame_cb_t` | 完整帧回调：`void (*)(const proto_frame_t*, void* user)` |
| `proto_frame_parser_t` | 解析器实例：state + f + payload_idx + sum + good/bad/overflow_cnt + cb + user |

##### 函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `proto_frame_init` | `void proto_frame_init(p, cb, user)` | 初始化解析器：清零、设置回调 |
| `proto_frame_reset` | `void proto_frame_reset(p)` | 重置到 IDLE，清零在途帧和校验和 |
| `proto_frame_feed` | `void proto_frame_feed(p, data, len)` | 喂入字节流：逐字节状态机推进；校验通过则回调；失败则 bad_cnt++ 并 reset |
| `proto_frame_build` | `int proto_frame_build(func_id, payload, len, out, out_cap)` | 构造帧到 out 缓冲：填充头部+payload+累加校验和；返回总长度 |

#### proto_dispatch.h / proto_dispatch.c — FuncID 分发表

##### 类型

| 类型 | 说明 |
|------|------|
| `proto_handler_fn` | 处理函数指针：`int (*)(const uint8_t* payload, uint8_t len)` |
| `proto_entry_t` | 分发表条目：func_id + expect_len(0=不校验) + handler + name |
| `proto_dispatcher_t` | 分发器：table + count + hit/miss/bad_len_cnt |

##### 函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `proto_dispatcher_init` | `void proto_dispatcher_init(d, table, count)` | 初始化分发器：绑定分发表 |
| `proto_dispatch_on_frame` | `void proto_dispatch_on_frame(f, user)` | 帧回调入口：遍历分发表匹配 FuncID；长度不匹配则 bad_len_cnt++；未匹配则 miss_cnt++ |

---

### 6.3 service/gait — 步态引擎

#### gait_if.h / gait_if.c — 步态统一接口（策略模式）

##### 类型

| 类型 | 说明 |
|------|------|
| `gait_leg_t` | 腿枚举：GAIT_LEG_FL/FR/RL/RR/NUM |
| `gait_leg_target_t` | 单腿目标：hip_rad/knee_rad/wheel_rads/in_stance |
| `gait_output_t` | 步态输出：leg[4] + phase[0,1) + tick_count |
| `gait_params_t` | 公共参数：body_height/step_length/step_height/period/duty/phase_offset[4]/touchdown_thresh |
| `gait_ops_t` | vtable：init/set_param/update/exit/name |
| `gait_if_t` | 步态实例：ops + params + ctx(私有上下文) |

##### 函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `gait_wrap01` | `float gait_wrap01(float x)` | 工具函数：把任意浮点数折回 [0,1)，使用 floorf |

#### gait_machine.h / gait_machine.c — 步态切换状态机

##### 类型

| 类型 | 说明 |
|------|------|
| `gait_machine_state_t` | GM_STATE_IDLE/RUN/BLEND |
| `gait_machine_t` | 状态机实例：current + target + blend_dur_s + blend_t_s + state + buf_a/b(双缓冲) |

##### 函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `gait_machine_init` | `void gait_machine_init(m)` | 初始化：清零，设 IDLE |
| `gait_machine_set` | `app_err_t gait_machine_set(m, g, p)` | 立即设置步态：调用 init+set_param，切到 RUN |
| `gait_machine_request` | `app_err_t gait_machine_request(m, g, p, blend_dur_s)` | 请求切换：若 blend_dur=0 立即切；否则进入 BLEND，同时跑当前+目标步态，lerp 输出 |
| `gait_machine_update` | `app_err_t gait_machine_update(m, dt_s, out)` | 推进时间：RUN 直接输出 current；BLEND 双缓冲 lerp，t≥1 时切到目标 |
| `gait_machine_state` | `gait_machine_state_t gait_machine_state(m)` | 查询当前状态 |

##### 内部函数

| 函数 | 说明 |
|------|------|
| `lerp(a, b, t)` | 线性插值 |
| `blend_outputs(a, b, t, o)` | 混合两个 gait_output_t：浮点字段 lerp，in_stance 取目标值，phase 取目标值 |

#### gait_stand.h / gait_stand.c — 站立步态

| 函数 | 签名 | 说明 |
|------|------|------|
| `gait_stand_create` | `gait_if_t* gait_stand_create(void)` | 创建站立步态单例：所有腿 in_stance=1，hip/knee/wheel=0 |

vtable 实现：
- `stand_init`: 空操作
- `stand_set_param`: 复制参数
- `stand_update`: 输出全 0 + in_stance=1
- `stand_exit`: 空操作
- `stand_name`: 返回 "stand"

#### gait_trot.h / gait_trot.c — 对角小跑步态

| 函数 | 签名 | 说明 |
|------|------|------|
| `gait_trot_create` | `gait_if_t* gait_trot_create(void)` | 创建 trot 步态单例 |
| `gait_trot_foot_traj` | `void gait_trot_foot_traj(leg_phase, duty, step_len, step_height, dx_m, dz_m, in_stance)` | 纯函数：根据腿相位计算足端 (dx, dz) 摆动量。支撑相：x 线性后退，z=0；摆动相：x 线性前进，z=sin(πt)·height |

vtable 实现：
- `trot_init`: phase 清零
- `trot_set_param`: 校验 period>0, duty∈[0,1]，复制参数
- `trot_update`: 累加主相位 phase += dt/period，wrap [0,1)；对4腿计算 leg_phase = phase + offset，调用 `gait_trot_foot_traj`；**注意**：hip_rad/knee_rad 临时携带 (dx, dz) 足端位移而非关节角，等 IK 接入后替换
- `trot_exit`: 空操作
- `trot_name`: 返回 "trot"

#### gait_params.h / gait_params.c — 出厂默认步态参数

| 常量 | 值 | 说明 |
|------|-----|------|
| `GAIT_PARAMS_TROT_DEFAULT` | period=0.4s, duty=0.5, step_len=0.06m, step_height=0.04m, height=0.20m, offset={0,0.5,0.5,0} | FL+RR同相, FR+RL反相 |
| `GAIT_PARAMS_STAND_DEFAULT` | period=1.0s, duty=1.0, step_len=0, step_height=0, height=0.20m | 全腿恒支撑 |

---

### 6.4 service/leg — 腿控制器

#### leg_controller.h / leg_controller.c

##### 类型

| 类型 | 说明 |
|------|------|
| `leg_actuators_t` | 单腿执行器：hip/knee/wheel 三个 motor_dev_t* |
| `leg_controller_t` | 腿控制器：leg[4] + send_cnt + miss_cnt |

##### 函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `leg_controller_init` | `void leg_controller_init(lc)` | 清零所有执行器指针 |
| `leg_controller_bind_from_registry` | `app_err_t leg_controller_bind_from_registry(lc)` | 从 motor_registry 按约定 logical ID 装配 4 腿 × 3 执行器，日志输出已绑定数量 |
| `leg_controller_apply` | `app_err_t leg_controller_apply(lc, o)` | 将 gait_output 推送给各腿电机：hip/knee 用 `set_position`（低增益 kp=1.5, kd=0.1），wheel 用 `set_velocity`；电机未绑定时 miss_cnt++ |

##### 内部函数

| 函数 | 说明 |
|------|------|
| `try_set_pos(d, rad)` | 安全调用 set_position，空指针/ops 空返回 UNSUPPORTED |
| `try_set_vel(d, v)` | 安全调用 set_velocity |

> ⚠️ **关键缺口**：当前 motor_registry 中所有 motor_dev_t* 为 NULL，apply 实际不会操作任何电机。

---

### 6.5 service/script — 脚本化步态

#### script_if.h — 脚本接口定义

| 类型 | 说明 |
|------|------|
| `script_keyframe_t` | 关键帧：t_s(时间戳) + leg[4] |
| `script_t` | 脚本：name + frames指针 + n_frames + loop标志 |

#### script_player.h / script_player.c — 脚本播放器

##### 类型

| 类型 | 说明 |
|------|------|
| `script_player_state_t` | SP_STATE_IDLE/RUNNING/DONE |
| `script_player_t` | 播放器：s(脚本) + t_s(累计时间) + tick_count + state |

##### 函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `script_player_init` | `void script_player_init(p)` | 清零 |
| `script_player_load` | `app_err_t script_player_load(p, s)` | 加载脚本，t_s=0，设 RUNNING |
| `script_player_reset` | `app_err_t script_player_reset(p)` | 回到脚本起始，设 RUNNING |
| `script_player_update` | `app_err_t script_player_update(p, dt_s, out)` | 累加 t_s；非 loop 脚本到末尾设 DONE；调用 script_sample 插值 |
| `script_player_state` | `script_player_state_t script_player_state(p)` | 查询当前状态 |
| `script_sample` | `app_err_t script_sample(s, t_s, out)` | **纯函数**：根据时间戳在关键帧间线性插值。t<首帧→首帧，t>末帧→末帧，loop→取模回放；alpha = (t - k_i.t) / (k_{i+1}.t - k_i.t)，浮点 lerp，in_stance 取较新帧 |

#### gait_script.h / gait_script.c — 脚本步态适配器

把 script_player 包装成 gait_if_t，可无缝接入 gait_machine 调度。

| 函数 | 签名 | 说明 |
|------|------|------|
| `gait_script_create` | `gait_if_t* gait_script_create(void)` | 创建脚本步态单例，内含 script_player |
| `gait_script_set_script` | `app_err_t gait_script_set_script(self, s)` | 装入指定脚本到 player |
| `gait_script_rewind` | `app_err_t gait_script_rewind(self)` | 重播脚本 |
| `gait_script_state` | `script_player_state_t gait_script_state(self)` | 查询播放状态 |

vtable 实现：
- `gs_init`: 若已加载脚本则 reset，否则维持 IDLE
- `gs_set_param`: 复制参数
- `gs_update`: 未装脚本→输出全0+in_stance=1；已装→调 player.update
- `gs_exit`: 空操作
- `gs_name`: "script"

#### script_builtin.h / script_builtin.c — 内置脚本集合

| 脚本 | 关键帧数 | 周期 | loop | 说明 |
|------|---------|------|------|------|
| `SCRIPT_BUILTIN_STAND_HOLD` | 2 | 1.0s | ✓ | 全腿支撑、零目标 |
| `SCRIPT_BUILTIN_WAVE_UP_DOWN` | 10 | 0.9s | ✓ | 四腿同时按正弦抬放（knee_rad 通道承载 z），等价旧 up_down[9] |
| `SCRIPT_BUILTIN_TROT_STEP` | 4 | 0.6s | ✓ | 对角 trot（FL+RR 同相支撑，FR+RL 反相摆动），hip_rad 承载 x，knee_rad 承载 z |

| 函数 | 签名 | 说明 |
|------|------|------|
| `script_builtin_find` | `const script_t* script_builtin_find(name)` | 按名字查找内置脚本，未找到返回 NULL |

---

### 6.6 service/kinematics — 运动学（⚠️ 空目录）

`App/service/kinematics/` 目录存在但为空。需要从 `Core/Src/gait_plan.c` 迁移：
- `inverse_kinematics_position()` — 2 连杆逆运动学（ORIGINAL/MIRROR 腿型）
- `forward_kinematics_position()` — 正运动学
- `params_init()` / `leg_size` 结构体 — 腿尺寸/质量/质心参数

---

## 7. App/app — 应用任务层

### 7.1 app_init.h / app_init.c — 全局初始化汇总

| 函数 | 签名 | 说明 |
|------|------|------|
| `app_init` | `app_err_t app_init(void)` | 依次调用：log_init → bsp_time_init → bsp_fdcan_init → bsp_usb_cdc_init → motor_registry_init |
| `app_init_bring_up` | `app_err_t app_init_bring_up(void)` | M1 过渡期入口，等价 app_init |

### 7.2 app_tasks.h / app_tasks.c — RTOS 任务创建

| 函数 | 签名 | 说明 |
|------|------|------|
| `app_tasks_create` | `void app_tasks_create(void)` | 先调用 task_comm_init + task_chassis_init，然后 MCU 端创建 4 个 osThread：t_log(512B, low)、t_safety(512B, realtime)、t_comm(1024B, above_normal)、t_chassis(2048B, high) |

### 7.3 task_chassis.h / task_chassis.c — 底盘步态控制任务

##### 模式

| 模式 | 说明 |
|------|------|
| `CHASSIS_MODE_AUTO` | 上电默认；心跳活→ONLINE，超时→STANDALONE |
| `CHASSIS_MODE_ONLINE` | 严格按 USB CDC 指令走 stand/trot |
| `CHASSIS_MODE_STANDALONE` | 完全脱机，跑固定脚本 |

##### 内部状态

| 状态 | 说明 |
|------|------|
| `ACT_STAND` | 站立（gait_stand） |
| `ACT_TROT` | 小跑（gait_trot） |
| `ACT_SCRIPT` | 脚本播放（gait_script） |

##### 函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `task_chassis_init` | `void task_chassis_init(void)` | 创建 stand/trot/script 步态实例，初始化 gait_machine 并设为 stand，初始化 leg_controller |
| `task_chassis_entry` | `void task_chassis_entry(void* arg)` | MCU: 500Hz 主循环；estop 时跳过；否则调用 step_for_test(2ms, now) |
| `task_chassis_set_mode` | `void task_chassis_set_mode(m)` | 切换模式 |
| `task_chassis_get_mode` | `chassis_mode_t task_chassis_get_mode(void)` | 查询当前模式 |
| `task_chassis_play_script` | `int task_chassis_play_script(s, blend_dur_s)` | 切到 SCRIPT 子状态，装入指定脚本 |
| `task_chassis_stop_script` | `int task_chassis_stop_script(blend_dur_s)` | 停止脚本，回 stand |
| `task_chassis_set_online_timeout_ms` | `void task_chassis_set_online_timeout_ms(ms)` | 设置心跳超时阈值（默认 500ms） |
| `task_chassis_get_online_timeout_ms` | `uint32_t task_chassis_get_online_timeout_ms(void)` | 查询超时阈值 |
| `task_chassis_step_for_test` | `void task_chassis_step_for_test(dt_s, now_ms)` | 单步推进：离线→offline_decide，在线→online_decide；然后 gait_machine_update + leg_controller_apply |
| `task_chassis_active_gait_name` | `const char* task_chassis_active_gait_name(void)` | 查询当前活跃步态名称 |

##### 内部函数

| 函数 | 说明 |
|------|------|
| `request_gait(g, p, blend)` | 请求步态切换：当前已在则跳过；BLEND 中强切 |
| `is_offline(now_ms)` | STANDALONE→离线，ONLINE→在线，AUTO→根据 dispatch_hit 和 last_rx_ms 判定 |
| `online_decide()` | 在线决策：\|v\|>0.05→trot，否则→stand |
| `offline_decide()` | 离线决策：脚本播完→回 stand；否则保持当前脚本 |

### 7.4 task_comm.h / task_comm.c — USB CDC 通信任务

##### 类型

| 类型 | 说明 |
|------|------|
| `task_comm_chassis_cmd_t` | 当前底盘指令：vx/vy/wz + seq(序列号) |

##### 函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `task_comm_init` | `void task_comm_init(void)` | 初始化：清零 chassis、初始化分发表(1条: 0x10→handle_chassis)、初始化帧解析器、注册 USB RX 回调 |
| `task_comm_entry` | `void task_comm_entry(void* arg)` | MCU: 空循环 osDelay(50)（解析在 USB 回调中完成） |
| `task_comm_good_cnt` | `uint32_t task_comm_good_cnt(void)` | 帧解析器 good_cnt |
| `task_comm_bad_cnt` | `uint32_t task_comm_bad_cnt(void)` | 帧解析器 bad_cnt |
| `task_comm_dispatch_hit` | `uint32_t task_comm_dispatch_hit(void)` | 分发器 hit_cnt |
| `task_comm_dispatch_miss` | `uint32_t task_comm_dispatch_miss(void)` | 分发器 miss_cnt |
| `task_comm_last_rx_ms` | `uint32_t task_comm_last_rx_ms(void)` | 最近一次有效帧的时间戳 |
| `task_comm_get_chassis` | `void task_comm_get_chassis(out)` | 获取当前底盘指令快照 |

##### 内部函数

| 函数 | 说明 |
|------|------|
| `handle_chassis(p, len)` | 解析 payload_chassis_cmd_t，更新 s_chassis.vx/vy/wz + seq + last_rx_ms |
| `on_usb_rx(d, n, user)` | USB RX 回调：直接喂给 proto_frame_feed |

### 7.5 task_log.h / task_log.c — 日志任务

| 函数 | 签名 | 说明 |
|------|------|------|
| `task_log_entry` | `void task_log_entry(void* arg)` | 占位实现：MCU 端 1Hz 心跳 TRACE 日志 |

### 7.6 task_safety.h / task_safety.c — 安全监控任务

| 函数 | 签名 | 说明 |
|------|------|------|
| `task_safety_estop_active` | `bool task_safety_estop_active(void)` | 查询急停状态 |
| `task_safety_estop_set` | `void task_safety_estop_set(bool on)` | 设置急停：on=true 时遍历所有 motor_registry 电机调用 disable |
| `task_safety_event_count` | `uint32_t task_safety_event_count(void)` | 急停事件计数 |
| `task_safety_entry` | `void task_safety_entry(void* arg)` | 占位：MCU 端 5ms 周期空循环（未来扫描电机超时/温度） |

> ⚠️ **缺口**：尚未实现自动电机超时/温度扫描逻辑。

---

## 8. App/test — 单元测试

| 测试文件 | 覆盖模块 | 说明 |
|---------|---------|------|
| `test_pid.c` | service/pid | PID 增益/限幅/增量式 |
| `test_proto_frame.c` | service/protocol/proto_frame | 帧解析状态机：完整帧/残帧/校验错 |
| `test_log.c` | common/log | 级别过滤/tag规则/后端注册 |
| `test_motor_registry.c` | device/motor_registry | 逻辑ID绑定/查询 |
| `test_gait.c` | service/gait | 相位发生器/wrap01/足端轨迹 |
| `test_leg_controller.c` | service/leg/leg_controller | 电机绑定/apply |
| `test_script.c` | service/script | 关键帧插值/loop/播放状态 |
| `test_task_chassis.c` | app/task_chassis | 三模式切换/心跳超时 |
| `test_task_comm.c` | app/task_comm | USB 注入→帧解析→分发 |
| `test_bsp_fdcan.c` | bsp/bsp_fdcan | Host mock TX队列/RX注入 |

构建方式: `cmake -S App/test -B build_host && ctest`

---

## 9. 附录：关键数据流

### 9.1 上位机指令流

```
PC USB → usbd_cdc_if.c → bsp_usb_cdc_on_rx() → on_usb_rx()
    → proto_frame_feed() [状态机解析]
    → proto_dispatch_on_frame() [FuncID 分发]
    → handle_chassis() [0x10]
    → s_chassis.vx/vy/wz + s_last_rx_ms 更新

task_chassis_entry (500Hz):
    → is_offline(now) → online_decide()
    → gait_machine_update(dt, &out) → gait_*.ops->update()
    → leg_controller_apply(&lc, &out)
        → motor_dev_t->ops->set_position / set_velocity [⚠️ vtable 未实现]
```

### 9.2 步态切换流

```
task_chassis request_gait():
    → gait_machine_request(&s_gm, g, p, blend)
    → GM_STATE_BLEND:
        current->update → buf_a
        target->update  → buf_b
        blend_outputs(a, b, t, out)   // t = blend_t / blend_dur
    → t >= 1.0: current = target, GM_STATE_RUN
```

### 9.3 脚本步态流

```
task_chassis_play_script(script):
    → gait_script_set_script(s_script, script)
    → request_gait(s_script, &STAND_DEFAULT, blend_dur)
    → gait_script.update(dt, out):
        → script_player_update(player, dt, out)
        → script_sample(script, t_s, out)  // 关键帧线性插值
```

---

*本文档由 AI 辅助生成，与源码同步于 2026-04-25。*
