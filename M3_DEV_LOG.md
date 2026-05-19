# M3 开发日志 — 脚本化死程序步态 + 脱机/在线双模 + 旧 main.c 隔离

Version: M3
Date: 2026-04-21
Target: STM32H723VGTX / FreeRTOS(CMSIS v2) / host-PC 单测
相关设计: `docs/SYSTEM_DESIGN.md` §5/§7/§10 M3（在 M3 原规划之外额外落地了"脚本步态"需求）

---

## 1. 本次目标

1. **隔离旧 `Core/Src/main.c`**：旧的 GO 电机直发 / PID_M3508_CAN_Init / 抬腿 while(1) 与 App/RTOS
   任务会互相写同一组电机，容易撞车。旧代码全部用 `USE_LEGACY_MAIN` 宏包起，默认 **0**（不参与编译），
   源码保留以便对照。
2. **新增"脚本步态" (`script_*`)**：像旧 main.c 那样，**完全脱离上位机**手写关键帧步态。
   脚本层是纯函数 + vtable 适配，既可 host 单测，也能直接喂给 `gait_machine` 播放。
3. **`task_chassis` 三模式 + 心跳回退**：
   - `ONLINE`     — 严格按 USB CDC 下发的 chassis_cmd 走 `stand/trot`
   - `STANDALONE` — 完全忽略 USB，只跑脚本（默认 `SCRIPT_BUILTIN_STAND_HOLD`）
   - `AUTO`       — 上电默认；心跳活按 ONLINE 行为；超时 (`online_timeout_ms`，默认 500ms) 自动回退
     STANDALONE 行为，防止有线断联就失控。
4. **一次任务最多 3 次请求**：本次 coding → 测试 → 总结+commit。

---

## 2. 新增目录

```
App/control/include/script/   （脚本化步态对外接口）
App/control/src/script/       （脚本化步态实现）
```

## 3. 新增文件清单

### 3.1 服务层 — 脚本步态
| 文件 | 作用 |
|---|---|
| `App/control/include/script/script_if.h` | 关键帧结构：`script_keyframe_t { t_s, leg[4] }`；脚本描述 `script_t { name, frames, n_frames, loop }`。 |
| `App/control/include/script/script_player.h` / `App/control/src/script/script_player.c` | 关键帧线性插值播放器。纯函数 `script_sample(s, t, out)` host 可测；状态机 `SP_STATE_IDLE/RUNNING/DONE`；loop 脚本自动回放。 |
| `App/control/include/script/gait_script.h` / `App/control/src/script/gait_script.c` | 把 `script_player` 包装成 `gait_if_t`；`gait_script_set_script()` / `gait_script_rewind()` 暴露出来；`init` 时自动 reset。 |
| `App/control/include/script/script_builtin.h` / `App/control/src/script/script_builtin.c` | 三份内置脚本：`STAND_HOLD` / `WAVE_UP_DOWN`（等价旧 main.c up_down[9]）/ `TROT_STEP`（4 帧对角 trot loop）；`script_builtin_find(name)` 查表。**用户自己增补死程序步态只需要在这个文件里加 keyframe 表。** |

### 3.2 测试
| 文件 | 覆盖 |
|---|---|
| `App/test/test_script.c` | 7 用例：端点/插值/非 loop DONE/loop 永 RUNNING/内置查表/内置 WAVE 峰值/gait_script vtable 驱动。 |
| `App/test/test_task_chassis.c` | 8 用例：默认 stand、ONLINE 收到 cmd 切 trot、AUTO 静默回退、AUTO 收到 cmd 切 trot、AUTO 超时回 stand、脚本播放+停止、STANDALONE 忽略 USB、脚本下电机被调用。 |

### 3.3 文档
- 本文件（`M3_DEV_LOG.md`）。

---

## 4. 修改文件清单

| 文件 | 改动 |
|---|---|
| `Core/Src/main.c` | 全部旧业务代码（含 `extract_data` UART 回调、up_down[9] 初始化、PID_M3508_CAN_Init、所有 `MotorController_Set/Send` 初始化序列、`while(1)` 抬腿 + `set_motor_current_can*` 循环）包进 `#if USE_LEGACY_MAIN ... #endif`。默认宏值 0，旧代码不参与链接，避免与 App 层 RTOS 任务对同一电机并发写。 |
| `App/app/include/task_chassis.h` / `App/app/src/task_chassis.c` | 重写：引入 `chassis_mode_t{AUTO/ONLINE/STANDALONE}`；新增 `task_chassis_play_script / stop_script / set_mode / set_online_timeout_ms / active_gait_name / step_for_test`；内部三状态 `ACT_STAND / ACT_TROT / ACT_SCRIPT`；`is_offline()` 以 `task_comm_dispatch_hit()==0` 作"从没收过帧"的判据（避开 `bsp_time_now_ms` 起点为 0 的误判）；host 单测 entry 点 `task_chassis_step_for_test(dt, now_ms)`。`task_chassis_init` 中重置 `s_mode` 和 `s_online_timeout_ms`，允许 host 单测反复 init。 |
| `App/app/include/task_comm.h` / `App/app/src/task_comm.c` | 新增 `s_last_rx_ms`（成功 dispatch 一帧时更新）+ `task_comm_last_rx_ms()`；`task_comm_init` 里额外清零 `s_last_rx_ms`。 |
| `App/test/CMakeLists.txt` | include 追加 `App/control/include/script`；源码清单加 `App/control/src/script` + `task_safety.c`/`task_chassis.c`；新增 `test_script`/`test_task_chassis` 两个 target。 |
| `CMakeLists.txt` | `include_directories` 追加 `App/control/include/script`。 |
| `cmake/firmware.cmake` | `APP_INCLUDE_DIRS`/`APP_SOURCE_DIRS` 追加 script include/src，保证 CubeMX 重新生成顶层 `CMakeLists.txt` 也能自动把新模块挂上。 |

---

## 5. 测试结果

### 5.1 Host-PC 单测

```bash
cmake -S App/test -B build_host
cmake --build build_host
ctest --test-dir build_host --output-on-failure
```

```
 1/10 Test #1:  test_pid             ... Passed
 2/10 Test #2:  test_proto_frame     ... Passed
 3/10 Test #3:  test_motor_registry  ... Passed
 4/10 Test #4:  test_log             ... Passed
 5/10 Test #5:  test_bsp_fdcan       ... Passed
 6/10 Test #6:  test_gait            ... Passed   (6 cases)
 7/10 Test #7:  test_task_comm       ... Passed   (2 cases)
 8/10 Test #8:  test_leg_controller  ... Passed   (3 cases)
 9/10 Test #9:  test_script          ... Passed   (7 cases  ← 新增)
10/10 Test #10: test_task_chassis    ... Passed   (8 cases  ← 新增)
100% tests passed, 0 tests failed out of 10
```

累计 **10 个 bin / 48 个用例全部通过**（M2 收官为 33，本次新增 15）。

### 5.2 ARM 固件编译

```bash
cmake -S . -B build_arm -DCMAKE_BUILD_TYPE=Debug
cmake --build build_arm
```

```
Memory region         Used Size  Region Size  %age Used
         ITCMRAM:           0 B        64 KB      0.00%
         DTCMRAM:           0 B       128 KB      0.00%
           FLASH:       56684 B         1 MB      5.41%   (M2: 64472 B → 下降 ~7.8 KB)
          RAM_D1:       83240 B       320 KB     25.40%
          RAM_D2:           0 B        32 KB      0.00%
          RAM_D3:           0 B        16 KB      0.00%
```

旧 main.c 被宏裁掉后 FLASH 占用下降；新 script 模块链接无告警。

### 5.3 过程中定位到的两个问题（已修复）

- 心跳判据一开始用"`last_rx_ms==0` 视为从未收过帧"，但在 host 上 `bsp_time_now_ms()`
  启动时可能返回 0，导致刚刚喂了一帧但 last=0 → 错误地进了离线分支。改为
  `task_comm_dispatch_hit()==0` 作为"从没收过帧"判据，`last_rx_ms` 只用于超时比较。
- `task_chassis_init` 原先不重置 `s_mode`，host 单测里一旦某个用例把模式设为 ONLINE，
  后续用例 setup_all 仍保留 ONLINE，导致"AUTO 静默回退"测例在模式被污染时判错。
  init 里显式复位 `s_mode = AUTO`、`s_online_timeout_ms = 500`。

---

## 6. 遗漏 / 未覆盖的测试

1. **真机脱机测试**：`task_chassis_play_script(&SCRIPT_BUILTIN_WAVE_UP_DOWN, 0.3f)` 已在 host
   打通插值输出与 leg_controller 派发；板上真机尚未跑（需要先做 M3/M4 的 IK + 电机 vtable 才能看到腿真的动）。
2. **`USE_LEGACY_MAIN=1` 的向后兼容性**：宏打开后代码还是能编的（全部包在 `#if` 里原样保留），
   但没有进行专门回归。若哪天要对照旧行为测试，需要 `-DUSE_LEGACY_MAIN=1` 重新编译并**停掉 app 层任务**，
   否则两边都会写 GO 电机 → 不能共存；这个约束已写入 main.c 文件顶部注释。
3. **脚本热切换**：目前 `gait_script_set_script()` 不支持"运行中无缝换脚本"；调用后下一次 `update()`
   会从新脚本的 t=0 播。需要"先 rewind 再换"，API 自己已按这个语义实现，但无单测。
4. **脚本插值在 in_stance 字段的语义**：插值期间 `in_stance = 目标帧的值`，跨摆动↔支撑边界时可能"提前"
   通知 safety。M3 真机调电机前建议把 in_stance 改成"在关键帧之间按时间切"而不是"取目标帧"。
5. **真机 USB CDC 长时间丢帧自动回退到 stand 的端到端**：host 侧已经模拟超时，板上未测。
6. **stand ↔ trot ↔ script 的三向切换矩阵**：只测了 stand→trot、script→stand 两条路径，
   trot→script、script→trot 没专项测。
7. **上行帧 0x80/0x81 仍未实现**（仍在 M3 TODO）。
8. **`motor_m3508.c / motor_go.c` vtable 仍未实现**；leg_controller 下发的 set_position
   目前在真机上还是空操作。

---

## 7. 项目使用讲解

### 7.1 构建

```bash
# 固件（默认：旧 main.c 已隔离，App 层正常工作）
cmake -S . -B build_arm -DCMAKE_BUILD_TYPE=Debug
cmake --build build_arm

# host 单测
cmake -S App/test -B build_host
cmake --build build_host
ctest --test-dir build_host --output-on-failure

# 如果想临时跑旧 main.c 抬腿循环（不推荐，仅对照用）：
cmake -S . -B build_arm_legacy -DCMAKE_BUILD_TYPE=Debug -DUSE_LEGACY_MAIN=1
# 并自行把 Core/Src/freertos.c 里 app_tasks_create() 注释掉
```

### 7.2 三种运行模式

```c
#include "task_chassis.h"
task_chassis_set_mode(CHASSIS_MODE_AUTO);        /* 默认 */
task_chassis_set_mode(CHASSIS_MODE_ONLINE);      /* 强制在线 */
task_chassis_set_mode(CHASSIS_MODE_STANDALONE);  /* 强制脱机 */
task_chassis_set_online_timeout_ms(500);         /* 心跳超时门限 */
```

### 7.3 手工写"死程序"步态（脱离上位机）

```c
#include "script_if.h"
#include "task_chassis.h"

/* 1) 直接写关键帧（t 为 0-起始的秒） */
static const script_keyframe_t MY_KF[] = {
    { 0.0f, {{0,0,0,1},{0,0,0,1},{0,0,0,1},{0,0,0,1}} },
    { 0.5f, {{0.2f,-0.3f,0,1},{0.2f,-0.3f,0,1},{0.2f,-0.3f,0,1},{0.2f,-0.3f,0,1}} },
    { 1.0f, {{0.0f, 0.0f,0,1},{0.0f, 0.0f,0,1},{0.0f, 0.0f,0,1},{0.0f, 0.0f,0,1}} },
};
static const script_t MY_SCRIPT = {
    .name = "squat", .frames = MY_KF,
    .n_frames = sizeof(MY_KF)/sizeof(MY_KF[0]),
    .loop = 1,   /* 0=播一遍自动回 stand；1=无限循环 */
};

/* 2) 播：在 task_chassis_init 之后任何时候都能调 */
task_chassis_play_script(&MY_SCRIPT, 0.3f);   /* blend 0.3s 过渡进来 */

/* 3) 停：回到 stand */
task_chassis_stop_script(0.3f);
```

内置脚本（在 `script_builtin.c`）：
- `SCRIPT_BUILTIN_STAND_HOLD` — STANDALONE 默认兜底
- `SCRIPT_BUILTIN_WAVE_UP_DOWN` — 等价旧 main.c 的 up_down[9] 抬腿循环
- `SCRIPT_BUILTIN_TROT_STEP` — 对角 trot 4 关键帧 loop

### 7.4 运行时数据流

```
=== ONLINE 分支 ===
USB CDC → task_comm → chassis_cmd
           ↓
   task_chassis: |v|>eps? → trot : stand
           ↓ gait_machine (BLEND/RUN)
       gait_output → leg_controller → motor vtable

=== STANDALONE / AUTO 超时分支 ===
(无 USB)
   task_chassis_play_script(&SCRIPT) 或默认 STAND_HOLD
           ↓ gait_script (包装 script_player)
           ↓ gait_machine (BLEND/RUN)
       gait_output → leg_controller → motor vtable
```

### 7.5 急停仍沿用 M2

```c
task_safety_estop_set(true);   /* chassis 挂起，所有 motor disable */
task_safety_estop_set(false);  /* 解除 */
```

---

## 8. 下一步（M4 / M5 起点）

- [ ] `kinematics/leg_ik.c`：脚本现在把目标当关节角或足端位移混用（跟 M2 trot 一致），
      接 IK 后改为统一"足端 (x,y,z)" → 关节角，再让 `gait_trot` / 脚本都喂 (x,y,z)。
- [ ] 真机 `motor_m3508.c / motor_go.c` vtable：让 leg_controller 真正把力下去。
- [ ] `task_safety` 扫描 `motor_state.last_rx_tick > 100ms` 自动 estop。
- [ ] 上行帧 0x80 / 0x81。
- [ ] 把本次 `is_offline` 判据也接到 `task_log` 里做"离线/在线"事件日志，方便真机排障。

本次 M3 的核心价值：**上位机可以有，也可以没有**——没上位机时，下位机用脚本跑死程序步态；
有上位机时，按协议跑 stand/trot；上位机断联，自动退回脚本；旧 main.c 不再与新架构竞写电机。
下一里程碑的重点就转向"让电机真的动起来"（IK + vtable）。
