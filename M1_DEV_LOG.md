# M1 开发日志 — FreeRTOS 架构 + 底层解耦（骨架落地）

Version: M1-skeleton
Date: 2026-04-21
Target: STM32H723VGTX / FreeRTOS (CMSIS v2) / host-PC test runner
相关设计文档: `docs/SYSTEM_DESIGN.md` §2~§9, §12

---

## 1. 本次目标

对齐 `SYSTEM_DESIGN.md §10 M1` 清单，落地以下骨架：

1. 新建 `App/` 分层目录树，并把所有新代码放入；
2. 实现 `bsp_time / bsp_fdcan / bsp_usb_cdc` 接口（板上壳 + host mock）；
3. 定义 `motor_if.h` vtable 与 `motor_registry` 逻辑→物理映射表；
4. 落地 `log.{c,h}`（TAG + 级别 + 多后端 + host 可抓取末行）；
5. 落地 USB CDC 协议：`proto_frame`（0x55 0xAA 状态机）+ `proto_dispatch`（FuncID 分发）；
6. Host-PC 单测工程（CMake + 内置极简 Unity 风格断言）；
7. 初始化汇总入口 `app_init()`，M1 阶段与旧 `main.c` 并行不冲突。

**策略**：`USE_LEGACY=1`，旧 `Core/Src/*.c` 原封保留并继续参与固件构建；新代码只在 `App/` 下独立演进。这样 ARM 固件编译不受影响，host 单测独立跑通。

---

## 2. 新增目录

```
App/
├─ common/        log.{c,h}  err.h  config.h  types.h
├─ bsp/           bsp_time.{c,h}  bsp_fdcan.{c,h}  bsp_usb_cdc.{c,h}
├─ device/        motor_if.h  motor_registry.{c,h}
├─ service/
│  ├─ pid/        pid.{c,h}
│  └─ protocol/   proto_defs.h  proto_frame.{c,h}  proto_dispatch.{c,h}
├─ app/           app_init.{c,h}
└─ test/
   ├─ unity/      test_util.{c,h}
   ├─ CMakeLists.txt
   ├─ test_pid.c
   ├─ test_proto_frame.c
   ├─ test_motor_registry.c
   ├─ test_log.c
   └─ test_bsp_fdcan.c
```

---

## 3. 新增文件清单（逐项说明）

| 文件 | 说明 |
|---|---|
| `App/common/types.h` | `app_tick_t`、`ARRAY_SIZE`、`UNUSED_ARG` 等通用类型/宏。 |
| `App/common/err.h` | `app_err_t` 统一错误码（NULL/TIMEOUT/OFFLINE/…）。 |
| `App/common/config.h` | 编译期开关：`APP_TARGET_HOST`、`USE_LEGACY`、`LOG_BACKEND_*`、`PROTO_MAX_PAYLOAD`。 |
| `App/common/log.{c,h}` | `LOG{E,W,I,D,T}` 宏；全局级别 + 每 tag 级别（上限 16）+ 多后端注册（上限 4）；host 下默认注册 stdio 后端；`log_last_line()` 供测试断言。 |
| `App/bsp/bsp_time.{c,h}` | `bsp_time_now_ms/us`、`bsp_time_delay_ms`、host 下 `bsp_time_test_advance_ms()`。板上包 `HAL_GetTick`，host 用 `CLOCK_MONOTONIC + 手动偏移`。 |
| `App/bsp/bsp_fdcan.{c,h}` | 抽象的 FDCAN 发送/接收注册接口；host 下内存 TX 环形队列（容量 64）+ RX 注入；板上 HAL 绑定 M1 末期接入（不破坏旧 `Core/Src/M3508.c` 初始化路径）。 |
| `App/bsp/bsp_usb_cdc.{c,h}` | USB CDC 发送/接收注册；host 下内存缓存（512B）+ `test_inject_rx`；板上绑定留给 M2。 |
| `App/device/motor_if.h` | `motor_type_t`、`motor_state_t`、`motor_ops_t`（vtable）、`motor_dev_t`。 |
| `App/device/motor_registry.{c,h}` | `motor_logical_id_t`（四足 12 个 + 机械臂 6 个占位）、编译期 `motor_cfg_t s_cfg[]` 空映射表、`motor_registry_bind()` 运行期挂载。 |
| `App/service/pid/pid.{c,h}` | host 可编译的 PID（位置式 + 增量式），d-on-measurement；与 `Core/Inc/pid.h` 语义等价、函数前缀 `app_pid_*` 以避免符号冲突。 |
| `App/service/protocol/proto_defs.h` | 帧常量、FuncID 下/上行列表、`payload_chassis_cmd_t`、`payload_status_req_t`。 |
| `App/service/protocol/proto_frame.{c,h}` | 状态机解析器（IDLE → HEAD1 → HEAD2 → FUNC → LEN → PAYLOAD → CKSUM）；`good_cnt/bad_cnt/overflow_cnt` 诊断计数；`proto_frame_build()` 构造器。 |
| `App/service/protocol/proto_dispatch.{c,h}` | 基于 `proto_entry_t` 表的分发器（hit/miss/bad_len 计数）。 |
| `App/app/app_init.{c,h}` | `app_init()` 汇总 `log_init` + 所有 bsp init + `motor_registry_init`；`app_init_bring_up()` 是 M1 过渡期的显式别名，方便后续切换。 |
| `App/test/unity/test_util.{c,h}` | 内置极简 Unity 风格宏：`TEST_ASSERT*`、`TEST_ASSERT_FLOAT_WITHIN`、`TU_RUN`、`TU_MAIN_EPILOGUE`。M2 接入完整 Unity 后可原地替换。 |
| `App/test/CMakeLists.txt` | host-pc 构建工程（独立于固件 `CMakeLists.txt`）；`cmake -S App/test -B build_host && ctest`。 |
| `App/test/test_pid.c` | 5 用例。 |
| `App/test/test_proto_frame.c` | 7 用例（含构造-解析闭环、逐字节送入、校验失败、前导垃圾、超长 LEN、分发器）。 |
| `App/test/test_motor_registry.c` | 4 用例。 |
| `App/test/test_log.c` | 3 用例（全局级别过滤 / tag 级别 / 格式化）。 |
| `App/test/test_bsp_fdcan.c` | 3 用例（发-取、RX 注入、参数校验）。 |
| `docs/M1_DEV_LOG.md` | 本文件。 |

---

## 4. 改动（修改）清单

本次 M1 阶段 **不修改** 以下任何现有文件：
- `Core/Src/*.c` / `Core/Inc/*.h` 保留原样
- 顶层 `CMakeLists.txt` 不改（仍只编译固件，不触达 `App/`）
- `FengMH.ioc` / linker script / OpenOCD cfg 不改

保证"旧固件编译链路 0 回归"。`App/` 的代码仅由 host-pc 工程 `App/test/CMakeLists.txt` 构建。

---

## 5. 测试结果

执行：

```bash
cmake -S App/test -B build_host
cmake --build build_host
ctest --test-dir build_host --output-on-failure
```

最终结果：

```
Test project /Users/code/FengMH/build_host
    Start 1: test_pid              ... Passed
    Start 2: test_proto_frame      ... Passed
    Start 3: test_motor_registry   ... Passed
    Start 4: test_log              ... Passed
    Start 5: test_bsp_fdcan        ... Passed

100% tests passed, 0 tests failed out of 5
```

共 **5 个可执行 / 22 个用例全部通过**。

### 5.1 过程中定位到的两个问题（已修复）

- `test_pid.c :: t_ms_dt_auto`：误以为首帧不积分；实际实现首帧用 `PID_DEFAULT_DT_S=0.01s` 累积，期望值从 `0.1` 修正为 `0.11`。
- `test_proto_frame.c :: t_garbage_then_good`：原构造的 junk 字节恰好能触发头序+合法 LEN，导致被当半帧吞了。换为不含 `0x55` 的纯噪声。

---

## 6. 遗漏 / 未覆盖的测试

M1 骨架阶段，以下点"接口已留但暂无覆盖"，将在对应里程碑补齐：

1. **板上 HAL 绑定**：`bsp_fdcan` 板上发送/接收尚未接 `HAL_FDCAN_*`，`bsp_usb_cdc` 板上也只是桩。相关路径在 `APP_TARGET_MCU=1` 下返回 `APP_ERR_UNSUPPORTED`，意图明确。
2. **FreeRTOS 任务心跳**：设计书 §5 的 `task_*` 骨架尚未建（等 CubeMX 开启 FreeRTOS 后一并做）。本次未创建 `task_*.c`。
3. **SEGGER RTT 后端**：`LOG_BACKEND_RTT` 宏预留，源码未引入（`Middlewares/Third_Party/SEGGER/` 未动）。host 端已有 stdio 后端。
4. **motor vtable 实现**：`motor_if.h` 已定义，但尚未创建 `motor_m3508.c / motor_go.c`，旧版 `Core/Src/M3508.c / GO-motor.c` 暂保留。迁移拆在 M1 的后续迭代里。
5. **PID 并行实现**：`App/service/pid/pid.c` 与 `Core/Src/pid.c` 并存（符号不冲突：前缀不同）。M2 起让 Core 版本逐步下线。
6. **协议上行帧**：只定义了 FuncID 常量（0x80/0x81/0x82/0x8F），尚无具体 payload 结构体和发送 helper。
7. **CAN Bus-Off 自恢复 / 电机超时看门狗 / 过温保护**：`task_safety` 尚未建，只是在设计书里写明。
8. **数值测试容差策略**：浮点断言统一用 `TEST_ASSERT_FLOAT_WITHIN`，PID 一阶系统收敛测试用 1e-2；未对 IK 做单测（`App/service/kinematics/` 本次未建）。

---

## 7. 项目使用讲解

### 7.1 固件构建（无变化，老流程）

照旧：CLion 打开本工程直接 Build；或命令行：

```bash
cmake -S . -B Debug -DCMAKE_BUILD_TYPE=Debug
cmake --build Debug
# flashing
cmake --build Debug --target flash
```

`App/` 目录**不会被固件 CMake 递归**，因此 M1 对板上二进制零改动、零风险。

### 7.2 Host-PC 单测构建与运行

```bash
# 首次配置
cmake -S App/test -B build_host
# 编译
cmake --build build_host
# 跑全部用例
ctest --test-dir build_host --output-on-failure
# 或跑单个
./build_host/test_proto_frame
```

只要本机有任意 C11 编译器（macOS clang / Linux gcc）即可编译，不需要 arm-none-eabi 工具链。

### 7.3 在代码里打日志

```c
#include "log.h"
static const char* TAG = "M3508";
void foo(void) {
    LOGI("online, rx_cnt=%u", rx_cnt);
    LOGW("temp=%d over soft limit", t);
    LOGE("offline, force torque=0");
}
```

运行期调级：

```c
log_set_global_level(LOG_LVL_WARN);
log_set_level("M3508", LOG_LVL_DBG);   // 只把 M3508 模块开到 debug
```

host 单测里抓末行：

```c
LOGI("hello %d", 42);
TEST_ASSERT(strstr(log_last_line(), "hello 42") != NULL);
```

### 7.4 电机接入约定（当前仅骨架）

1. 在 `motor_registry.c::s_cfg[]` 里把逻辑名对齐到实际 `bus + can_id + dir + zero_offset`；
2. 实现具体驱动（例如 `motor_m3508.c`）并填 `motor_ops_t`；
3. 启动时调用：

```c
static motor_dev_t s_fl_wheel;
motor_m3508_create(&s_fl_wheel, /* can_bus, id, ...*/);
motor_registry_bind(MOTOR_ID_FL_WHEEL, &s_fl_wheel);
```

之后上层（步态/IK/安全任务）统一：

```c
motor_dev_t* d = motor_get(MOTOR_ID_FL_WHEEL);
d->ops->set_current(d, 2.5f);   // 2.5 A
```

### 7.5 协议接入示例

```c
#include "proto_frame.h"
#include "proto_dispatch.h"
#include "proto_defs.h"

static int on_chassis(const uint8_t* p, uint8_t len) {
    payload_chassis_cmd_t cmd;
    memcpy(&cmd, p, sizeof cmd);
    /* cmd.vx, cmd.vy, cmd.wz → 传给 task_chassis */
    return 0;
}

static const proto_entry_t tbl[] = {
    { PROTO_FUNC_CHASSIS_CMD, sizeof(payload_chassis_cmd_t), on_chassis, "chassis" },
};

static proto_frame_parser_t parser;
static proto_dispatcher_t   disp;

void comm_init(void) {
    proto_dispatcher_init(&disp, tbl, sizeof(tbl)/sizeof(tbl[0]));
    proto_frame_init(&parser, proto_dispatch_on_frame, &disp);
    bsp_usb_cdc_attach_rx(on_usb_bytes, &parser);
}

void on_usb_bytes(const uint8_t* d, uint32_t n, void* user) {
    proto_frame_feed((proto_frame_parser_t*)user, d, n);
}
```

---

## 8. 下一步（M1 剩余 + M2 起点）

- [ ] 在 CubeMX 里启用 FreeRTOS（CMSIS v2），生成任务骨架，并把 `app_init()` 接进 `main.c` 的 `USER CODE` 区（仍 `USE_LEGACY=1`）。
- [ ] 创建 `motor_m3508.c`（vtable 实现），把 `Core/Src/M3508.c` 的功能迁入；老文件先保留直至 host 单测 + 板上对比验证通过再删除。
- [ ] 引入 `SEGGER_RTT.c/.h` 并打开 `LOG_BACKEND_RTT`。
- [ ] 接板上 HAL：`bsp_fdcan` 板上 send/rx → HAL、`bsp_usb_cdc` 板上 → USBD_CDC。
- [ ] M2：`gait_if.h` + `gait_trot`、`leg_controller`。

---

本次改动**对现有固件零破坏**，所有新能力以 host-PC 单测为验收手段已 100% 通过。
