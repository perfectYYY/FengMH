# FengMH 四足轮腿 + 机械臂 下位机 系统设计计划书

Version: 0.1 (2026-04-21)
MCU: STM32H723VGTX · RTOS: FreeRTOS (CMSIS-RTOS v2) · 日志: SEGGER RTT · 测试: Unity + Host-PC Mock

---

## 1. 设计目标

| 目标 | 达成手段 |
|---|---|
| 驱动层 / 应用层完全解耦 | 分层目录 + 接口头文件 (HAL Port) + 不允许反向依赖 |
| 可替换 / 可单测 | 所有外设访问通过 `bsp_*` 接口，应用层只依赖 `app_*` / `algo_*` |
| 调试可观测 | 统一 `log.h` 宏 + tag 分级 + RTT 通道 + 周期性健康上报 |
| 冗余 & 安全 | 电机通讯超时看门狗、过流/过温/掉线事件总线、软急停、CAN Bus-Off 自恢复 |
| 易扩展步态 | 步态= Trajectory Generator + Leg Controller 的可插拔策略模式 |
| 与上位机对接 | USB CDC + 状态机协议解析 + FuncID 分发表 |

---

## 2. 总体分层

```
+-----------------------------------------------------------+
| App Layer (tasks)                                         |
|  task_chassis  task_arm  task_comm  task_safety  task_log |
+-----------------------------------------------------------+
| Service Layer (无硬件依赖, 可在 PC 上单测)                 |
|  gait/*  kinematics/*  gravity_comp/*  lqr/*  protocol/*  |
|  pid/*   filter/*      state_estimator/*                  |
+-----------------------------------------------------------+
| Device Layer (电机/IMU/上位机 抽象接口)                    |
|  motor_if.h  motor_m3508.c  motor_go.c  motor_damiao.c    |
|  imu_if.h    host_if.h (USB CDC)                          |
+-----------------------------------------------------------+
| BSP / Port Layer (HAL 包装, 可替换/可 mock)                |
|  bsp_fdcan.c  bsp_uart.c  bsp_usb_cdc.c  bsp_tim.c        |
|  bsp_gpio.c   bsp_time.c                                   |
+-----------------------------------------------------------+
| ST HAL / CMSIS / FreeRTOS                                  |
+-----------------------------------------------------------+
```

**强约束**：上层头文件可被下层 include；下层严禁 include 上层；Service 层只 include 标准库 + 自己头文件（保证 PC 编译可行）。

---

## 3. 目录结构（重构后）

```
FengMH/
├─ Core/                      # CubeMX 生成, 只保留 msp/it/system/main_cube.c
├─ Drivers/                   # HAL & CMSIS (不动)
├─ Middlewares/               # FreeRTOS, USB_Device, DSP
├─ App/                       # ← 新增，应用代码全部在这里
│  ├─ bsp/
│  │  ├─ bsp_fdcan.{c,h}
│  │  ├─ bsp_usb_cdc.{c,h}
│  │  ├─ bsp_time.{c,h}
│  │  └─ bsp_gpio.{c,h}
│  ├─ device/
│  │  ├─ motor_if.h           # 统一电机接口 (抽象)
│  │  ├─ motor_m3508.{c,h}    # C620+M3508 (轮毂 & 机械臂末端)
│  │  ├─ motor_go.{c,h}       # 宇树 GO 关节电机 (腿)
│  │  ├─ motor_damiao.{c,h}   # 达妙 (机械臂, M5 阶段移植)
│  │  └─ motor_registry.{c,h} # id→(bus,canid,type) 映射表
│  ├─ service/
│  │  ├─ pid/pid.{c,h}
│  │  ├─ filter/lpf.{c,h}  filter/kalman.{c,h}
│  │  ├─ kinematics/leg_ik.{c,h}  kinematics/arm_ik.{c,h}
│  │  ├─ gait/gait_if.h   gait/gait_trot.{c,h}  gait/gait_stand.{c,h}
│  │  ├─ gravity_comp/gravity_comp.{c,h}
│  │  ├─ lqr/lqr.{c,h}
│  │  ├─ state_est/odom.{c,h}
│  │  └─ protocol/
│  │     ├─ proto_frame.{c,h}      # 0x55 0xAA 状态机
│  │     ├─ proto_dispatch.{c,h}   # FuncID 分发
│  │     └─ proto_defs.h           # FuncID / Payload 结构体
│  ├─ app/
│  │  ├─ task_chassis.{c,h}
│  │  ├─ task_arm.{c,h}
│  │  ├─ task_comm.{c,h}
│  │  ├─ task_safety.{c,h}
│  │  ├─ task_log.{c,h}
│  │  └─ app_init.{c,h}       # 替代 main 里所有业务初始化
│  ├─ common/
│  │  ├─ log.h   log.c        # tag 宏 + RTT
│  │  ├─ err.h                # 统一错误码
│  │  ├─ config.h             # 编译期开关
│  │  └─ types.h
│  └─ test/
│     ├─ unity/               # Unity sources
│     ├─ mocks/               # HAL/FDCAN mock
│     ├─ test_pid.c
│     ├─ test_proto_frame.c
│     ├─ test_leg_ik.c
│     └─ CMakeLists.txt       # host-pc build
├─ docs/
│  ├─ SYSTEM_DESIGN.md (本文件)
│  └─ PROTOCOL.md
└─ CMakeLists.txt
```

> 现有 `Core/Src/{M3508,GO-motor,fdcan,pid,gait_plan}.{c,h}` 将按职责搬迁并重命名至 `App/device/` 与 `App/service/`，原文件在 M1 完成后删除。

---

## 4. 电机抽象 (`motor_if.h`)

所有电机对上层**只暴露一个接口**，便于步态/机械臂算法复用：

```c
typedef enum { MOTOR_M3508, MOTOR_GO, MOTOR_DAMIAO } motor_type_t;

typedef struct {
    uint16_t id;              // 逻辑 id (0..N)
    motor_type_t type;
    uint8_t  can_bus;         // 1/2/3
    uint32_t can_id;          // 物理 CAN ID
    uint8_t  online;
    float    angle_rad;       // 多圈后
    float    velocity_rads;
    float    torque_nm;       // 估算或反馈
    float    temperature_c;
    uint32_t last_rx_tick;
    uint32_t rx_cnt, err_cnt;
} motor_state_t;

typedef struct motor_dev_s {
    motor_state_t state;
    /* vtable */
    int (*set_current)(struct motor_dev_s*, float iq_a);      // C620 是电流
    int (*set_torque) (struct motor_dev_s*, float tau_nm);    // GO/达妙
    int (*set_position)(struct motor_dev_s*, float pos, float vel, float kp, float kd, float tau_ff);
    int (*set_velocity)(struct motor_dev_s*, float vel_rads);
    int (*enable)     (struct motor_dev_s*);
    int (*disable)    (struct motor_dev_s*);
    int (*reset_fault)(struct motor_dev_s*);
    int (*feed_rx)    (struct motor_dev_s*, const uint8_t* data, uint8_t dlc); // ISR 调用
} motor_dev_t;
```

### 4.1 C620 + M3508 功能覆盖（依据 PDF V1.01 / M3508 V1.0）
驱动层需实现全部协议功能：
- 电流控制帧 `0x200 / 0x1FF / 0x2FF`（1~8号电机群发）
- 反馈帧 `0x201~0x208`：机械角度 (0..8191) / 转速 rpm / 转矩电流 / 温度
- 机械角度→连续多圈 `int32_t turns + angle` 解算
- 温度保护 (>80°C) 软限功率、>85°C 立即停机
- 堵转检测（电流持续 > 阈值 且 速度 ≈ 0 超时）
- CAN 心跳丢失 > 100ms → 置离线 + 事件广播
- 电流闭环 PID(位置/速度/电流 三环) 可选装配
- 减速比 187:1 已知量：对外角度/速度自动除以减速比
- 参数化 helper：`m3508_rpm_to_rads`, `m3508_iq_to_current_a`, `m3508_ecd_to_rad`

### 4.2 GO 电机 / 达妙
保留既有协议，迁入 `motor_go.c` / `motor_damiao.c`，统一套用 vtable。

### 4.3 电机注册表 `motor_registry`
- 编译期数组 `motor_cfg_t g_motor_table[]`，字段：逻辑名 / 类型 / 总线 / canid / 方向 / 零位偏置 / 限位
- 运行期 `motor_get(MOTOR_LEG_FL_HIP)` 返回 `motor_dev_t*`
- **关键**：硬件映射改动只改表，不动算法

---

## 5. RTOS 任务划分

| 任务 | 周期 | 优先级 | 说明 |
|---|---|---|---|
| `task_motor_tx_leg`   | 1 kHz | 很高 | 读取各腿目标指令→下发 CAN |
| `task_motor_tx_arm`   | 1 kHz | 很高 | 机械臂 CAN 下发（M5 阶段） |
| `task_chassis_ctrl`   | 500 Hz | 高 | IK / 步态相位 / 关节目标计算 |
| `task_arm_ctrl`       | 500 Hz | 高 | 机械臂 IK + 轨迹 |
| `task_state_est`      | 500 Hz | 高 | IMU 融合 / 里程计 |
| `task_gravity_comp`   | 200 Hz | 中 | 整机重力/动力学补偿 |
| `task_comm_rx`        | 事件 | 中 | USB CDC 流→帧 |
| `task_comm_tx`        | 100 Hz | 中 | 上传姿态 / 电机状态 |
| `task_safety`         | 200 Hz | 最高 | 超时 / 过温 / 急停 |
| `task_log`            | 低 | 最低 | 异步 log 刷出 |

CAN RX 走 **ISR → 环形缓冲 → notify task**；不在 ISR 里算浮点。
所有共享状态放 `static` + `osMutex` 或 `osMessageQueue`。

---

## 6. 日志系统 (`log.h`)

```c
typedef enum { LOG_LVL_ERR, LOG_LVL_WARN, LOG_LVL_INFO, LOG_LVL_DBG, LOG_LVL_TRACE } log_lvl_t;

// tag 在每个 .c 文件顶部定义: static const char* TAG = "M3508";
#define LOGE(fmt,...) log_emit(LOG_LVL_ERR , TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt,...) log_emit(LOG_LVL_WARN, TAG, fmt, ##__VA_ARGS__)
#define LOGI(fmt,...) log_emit(LOG_LVL_INFO, TAG, fmt, ##__VA_ARGS__)
#define LOGD(fmt,...) log_emit(LOG_LVL_DBG , TAG, fmt, ##__VA_ARGS__)
#define LOGT(fmt,...) log_emit(LOG_LVL_TRACE,TAG, fmt, ##__VA_ARGS__)
```

特性：
- 后端：SEGGER RTT (channel 0) + 可选 USB CDC 分包（`log_backend_t` 接口）
- 运行期可调 tag 级别：`log_set_level("M3508", LOG_LVL_WARN);`
- `ASSERT(cond, ...)` 失败 → LOGE + 触发 safety 急停（debug 模式）
- 内部使用 lock-free ring buffer (ISR 安全) + `task_log` 消费

**必须打 TAG 的模块**（M1 要求）：
`M3508` `GO` `FDCAN` `IK_LEG` `GAIT` `PID` `COMM` `SAFETY` `STATE` `INIT`

---

## 7. 上位机 USB CDC 协议

### 7.1 解析状态机 (`proto_frame.c`)
```
IDLE → HEAD1(0x55) → HEAD2(0xAA) → FUNCID → LEN → PAYLOAD(N) → CHECKSUM
```
任意一步失败回 `IDLE`；校验和= Byte0..Byte(4+N-1) 累加低8位。

### 7.2 Payload 结构体（小端，`__attribute__((packed))`）
```c
typedef struct __attribute__((packed)) {
    float vx; float vy; float wz;
} payload_chassis_cmd_t;  // FuncID 0x10, Len=12
```

### 7.3 分发表
```c
typedef int (*proto_handler_t)(const uint8_t* payload, uint8_t len);
typedef struct { uint8_t func_id; uint8_t expect_len; proto_handler_t h; } proto_entry_t;
static const proto_entry_t g_proto_tbl[] = {
    { 0x10, 12, on_chassis_cmd },
    // 预留 0x11 arm_cmd / 0x12 gait_cmd / 0x20 status_req ...
};
```
未知 FuncID → `LOGW` 并计入统计寄存器（冗余：可被上位机 `status_req` 查询）。

### 7.4 上行帧（下位机 → 上位机）预留
- `0x80` 整机状态（姿态/速度/电量）100Hz
- `0x81` 电机状态列表 20Hz
- `0x82` 日志镜像（level ≥ WARN）事件触发
- `0x8F` 错误/事件

---

## 8. 冗余与安全 (`task_safety`)

| 事件源 | 检测 | 响应 |
|---|---|---|
| 电机超时 (>100ms 无反馈) | RX 时戳 | 置离线；同腿置零力矩；报 0x8F |
| 过温 / 过流 | 反馈字段 | 软限 → 停机 |
| CAN Bus-Off | HAL 回调 | 自动复位 FDCAN 外设，重注册滤波器 |
| 上位机心跳丢失 >500ms | `task_comm` | 进入 `STAND_HOLD` 安全姿态 |
| ASSERT 失败 | 任意任务 | 急停广播，所有电机 disable |
| 看门狗 | IWDG 1s | 硬复位 |

全局状态机：`BOOT → SELF_TEST → IDLE → READY → RUN → FAULT`。

---

## 9. 单元测试策略

### 9.1 Host-PC 测试（`App/test/CMakeLists.txt`）
- Toolchain：gcc + Unity
- 覆盖：`pid`, `filter`, `leg_ik`, `arm_ik`, `proto_frame`, `proto_dispatch`, `gait_trot` 相位发生器, `motor_registry`
- Mock：`bsp_fdcan_mock.c` 捕获发送帧供断言
- CI-friendly：`cmake -S App/test -B build_host && ctest`

### 9.2 板上自检
- 上电 `SELF_TEST`：
  - FDCAN 回环帧
  - 每路电机心跳探测
  - IMU 可读
- 失败则进 `FAULT`，RTT 打印报告

---

## 10. 里程碑与交付物

### M1 — FreeRTOS 架构 + 底层解耦（当前）
- [ ] 新建 `App/` 目录树，迁移并重命名现有文件
- [ ] 实现 `bsp_fdcan`、`bsp_usb_cdc`、`bsp_time`
- [ ] 定义 `motor_if.h`，把 M3508 / GO 改为 vtable 实现
- [ ] 实现 `motor_registry` 与空映射表（留给你后期填）
- [ ] `log.{c,h}` + SEGGER RTT 后端，所有关键模块打 TAG
- [ ] CubeMX 生成 FreeRTOS (CMSIS v2)，创建骨架任务（先打印心跳）
- [ ] `app_init()` 汇总初始化，`main.c` 只调用 `app_init(); osKernelStart();`
- [ ] Host-PC 测试工程跑通 `test_pid` + `test_proto_frame`

### M2 — 应用层步态封装
- [ ] `gait_if.h` 统一接口 (`init / set_param / update(phase,dt) / exit`)
- [ ] `gait_stand` / `gait_trot`（从现有 `gait_plan.c` 抽离）
- [ ] `leg_controller`：关节电机 + 轮毂电机统一接口
- [ ] 参数文件 `gait_params.h`（步高/步长/周期/触地阈值/占空比）
- [ ] 步态切换状态机 + 平滑过渡
- [ ] 单测：相位发生器波形、落地时序

### M3 — 底盘控制 + 协议对接
- [ ] 处理 `0x10` 底盘指令 → 步态参数解算
- [ ] 重力补偿（腿静态 + 机械臂动态）
- [ ] 整机状态上报帧 0x80/0x81

### M4 — LQR
- [ ] 线性化模型 + 离线增益表 / 在线解
- [ ] 与重力补偿叠加

### M5 — 机械臂移植
- [ ] `motor_damiao.c` 接入 vtable
- [ ] 移植 `damiao_new1` 轨迹/IK 到 `App/service/kinematics/arm_ik.c`
- [ ] 新建 `task_arm_ctrl`
- [ ] 底盘对机械臂做动态重力补偿闭环联调

### M6 — 整机联调与优化
- [ ] 运行期性能测量（任务占用 / CAN 利用率 / 丢帧率）
- [ ] 参数整定工具（上位机帧驱动）
- [ ] 压力测试脚本

---

## 11. 风险与对策

| 风险 | 对策 |
|---|---|
| FDCAN 总线拥塞（16+ 电机 1kHz） | 群发帧优先、分桶下发、监测 Tx FIFO |
| Host-PC 测试覆盖浮点 IK 的数值误差 | Unity `TEST_ASSERT_FLOAT_WITHIN` 容差 1e-4 |
| 任务周期抖动 | 使用 `osDelayUntil`，关键任务锁核 |
| RTT 在 Release 被优化掉 | `log.c` 关键符号 `__attribute__((used))` |
| 迁移老代码破坏当前可用功能 | M1 保留旧目录并行编译，通过宏 `USE_LEGACY` 逐步切换 |

---

## 12. 立即可启动的 M1 第一步

1. 新建 `App/` 目录骨架 + `App/common/{log.h,log.c,err.h,config.h}`
2. 打通 SEGGER RTT（添加 `SEGGER_RTT.c/.h` 到 `Middlewares/Third_Party/SEGGER`）
3. 写 `motor_if.h` + 把 `M3508.c` 重构为实现体
4. Host-PC `test/` CMake 跑通第一个 `test_pid.c`

等你确认后我将按此推进 M1 的代码改造，并把每次改动控制在少量请求内完成。
