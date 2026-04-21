# M2 开发日志 — 应用层步态封装 + FreeRTOS 任务骨架 + USB CDC 端到端

Version: M2
Date: 2026-04-21
Target: STM32H723VGTX / FreeRTOS(CMSIS v2, CM4F 移植) / USB HS CDC / host-PC 单测
相关设计: `docs/SYSTEM_DESIGN.md` §5 RTOS 任务、§7 协议、§10 M2

---

## 1. 本次目标（对齐 SYSTEM_DESIGN §10 M2）

1. 步态统一接口 `gait_if.h` + `gait_stand` + `gait_trot`；
2. 步态状态机 `gait_machine`（RUN/BLEND 平滑过渡）；
3. `leg_controller`：腿 = 髋 + 膝 + 轮，统一向 motor_registry 取设备并下发；
4. 步态参数 `gait_params.h`（trot / stand 出厂默认）；
5. 单测：相位发生器波形、落地时序、状态机切换；
6. FreeRTOS 任务骨架：`task_log / task_safety / task_comm / task_chassis`；
7. USB CDC 端到端：`CDC_Receive_HS → bsp_usb_cdc_on_rx → proto_frame → proto_dispatch → task_chassis`；
8. 顶层 CMake 纳入 `App/` 全部源码与 USB/CDC、FreeRTOS include path；同时指定 `fpv5-d16 hard` FPU 与 `nano/nosys specs`，使 CubeMX 生成的骨架在命令行下可完整编译。

**策略**：保持旧 `Core/Src/M3508.c / GO-motor.c / pid.c / gait_plan.c` 在链接目标里共存（`USE_LEGACY=1`），M2 的新业务走 `App/` 路径；老代码逐步在后续里程碑下线。

---

## 2. 新增目录

```
App/service/gait/
App/service/leg/
App/app/          （此前 M1 已存在，本次大量充实：4 个 task_*.c + app_tasks.c）
```

---

## 3. 新增文件清单

### 3.1 服务层 — 步态
| 文件 | 作用 |
|---|---|
| `App/service/gait/gait_if.{c,h}` | 策略模式：`gait_ops_t` vtable + `gait_leg_target_t`（hip/knee/wheel + in_stance）+ `gait_output_t`（4 条腿 + phase）。工具函数 `gait_wrap01`。 |
| `App/service/gait/gait_params.{c,h}` | `GAIT_PARAMS_TROT_DEFAULT`（period=0.4s, duty=0.5, phase_offset=FL/RR 同相, FR/RL 反相）；`GAIT_PARAMS_STAND_DEFAULT`（duty=1）。 |
| `App/service/gait/gait_stand.{c,h}` | 全 in_stance=1、wheel=0；IK 接入留 M3。 |
| `App/service/gait/gait_trot.{c,h}` | 主相位推进 + 逐腿 `leg_phase = wrap(phase + offset)`；支撑相 x 线性扫 +step/2→-step/2，摆动相 x 反向 + z 走 `sin(π·t)·step_height` 椭圆。`gait_trot_foot_traj()` 单独导出供单测。 |
| `App/service/gait/gait_machine.{c,h}` | 三状态机 `IDLE / RUN / BLEND`；`request()` + `blend_dur_s`；BLEND 期间同时跑 cur/tgt，输出做 `lerp(a, b, t/dur)`。 |

### 3.2 服务层 — 腿控制器
| 文件 | 作用 |
|---|---|
| `App/service/leg/leg_controller.{c,h}` | 4 条腿 × 3 个 `motor_dev_t*`（hip/knee/wheel）；`bind_from_registry()` 一次性从 `motor_registry` 拿齐；`apply(out)` 调用 `set_position / set_velocity`；带 `send_cnt / miss_cnt`。 |

### 3.3 应用层 — RTOS 任务
| 文件 | 作用 |
|---|---|
| `App/app/task_log.{c,h}` | 低优先级心跳任务（占位；后续接 lock-free ring）。 |
| `App/app/task_safety.{c,h}` | `Realtime` 优先级；`task_safety_estop_set(true)` 遍历 motor_registry 调用 `disable()`；全局 `s_estop` + 事件计数。 |
| `App/app/task_comm.{c,h}` | `task_comm_init()` 注册 `proto_frame_parser` + `proto_dispatcher`（当前只有 `PROTO_FUNC_CHASSIS_CMD`），把 `bsp_usb_cdc_attach_rx` 的字节灌进解析器；对外 `task_comm_get_chassis(&cmd)`、`good/bad/hit/miss` 计数。 |
| `App/app/task_chassis.{c,h}` | `High` 优先级，500Hz：读 chassis_cmd → |v|>0.05 切 trot，否则切 stand（blend 0.3s）→ `gait_machine_update` → `leg_controller_apply`。estop 激活时挂起。 |
| `App/app/app_tasks.{c,h}` | `app_tasks_create()`：依次 `task_comm_init / task_chassis_init`，再 `osThreadNew` 4 个任务（栈 512~2048、优先级 Low→Realtime）。 |

### 3.4 测试
| 文件 | 覆盖 |
|---|---|
| `App/test/test_gait.c` | 6 用例：`gait_wrap01` 边界、stand 输出、trot 足端 4 点、相位偏移、周期归零（带浮点容差）、状态机 set+request+blend。 |
| `App/test/test_task_comm.c` | 2 用例：完整 chassis 帧 → `task_comm_get_chassis` 值匹配 + 计数；未知 FuncID → miss 计数。 |
| `App/test/test_leg_controller.c` | 3 用例：全 bind 后 12 个 actuator 到位、apply 实际派发到 stub vtable 并值正确、未 bind 时 miss_cnt 累加。 |

### 3.5 文档
- `M2_DEV_LOG.md`（本文件）。

---

## 4. 修改文件清单

| 文件 | 修改 |
|---|---|
| `App/bsp/bsp_usb_cdc.h` | 新增 `bsp_usb_cdc_on_rx()` 公开 hook，供板上 `usbd_cdc_if.c` 直接灌数据。 |
| `App/bsp/bsp_usb_cdc.c` | 板上 send 桥接到 `CDC_Transmit_HS`（由 `usb_cdc_if.c` 外部声明，不反向依赖 USB 中间层头）；`bsp_usb_cdc_on_rx()` 统一给 host/MCU 共用。 |
| `App/test/CMakeLists.txt` | `include_directories` 加 `service/gait`、`service/leg`；`APP_SRCS` 纳入 gait 5 个 + leg_controller + task_comm；新增 3 个 test target。 |
| `App/test/test_gait.c` | `t_trot_periodic` 断言放宽（1 周期累加误差 phase≈1 等价 0，用 `min(ph, 1-ph)<5e-3`）。 |
| `Core/Src/freertos.c` | `USER CODE BEGIN Includes` 加 `#include "app_init.h"` / `"app_tasks.h"`；`USER CODE BEGIN RTOS_THREADS` 调用 `app_init(); app_tasks_create();`。 |
| `USB_DEVICE/App/usbd_cdc_if.c` | `CDC_Receive_HS` 里 `extern void bsp_usb_cdc_on_rx()` 并调用，把 USB RX 字节直通 App 解析流水线。 |
| `CMakeLists.txt` | 追加 FPU 选项 `-mfloat-abi=hard -mfpu=fpv5-d16`（FreeRTOS 的 `vstmdbeq s16-s31` 要求，修复 Assembler Error）；`-specs=nano.specs -specs=nosys.specs`；末尾 `App/ 分层` 节把 App 全部 `*.c` 与 include 目录加到 `FengMH.elf` target；并定义 `APP_TARGET_HOST=0`。注：CubeMX 将来若重新生成 CMakeLists，需要把这节末尾内容原样追加回去（文件里已用中文注释标识该边界）。 |
| `App/device/motor_registry.c` | （外部编辑）`RR_KNEE` 一行格式细微调整，功能无变化。 |

---

## 5. 测试结果

### 5.1 Host-PC 单测

```
cmake -S App/test -B build_host
cmake --build build_host
ctest --test-dir build_host
```

```
1/8 Test #1: test_pid              ... Passed
2/8 Test #2: test_proto_frame      ... Passed
3/8 Test #3: test_motor_registry   ... Passed
4/8 Test #4: test_log              ... Passed
5/8 Test #5: test_bsp_fdcan        ... Passed
6/8 Test #6: test_gait             ... Passed   (6 cases)
7/8 Test #7: test_task_comm        ... Passed   (2 cases)
8/8 Test #8: test_leg_controller   ... Passed   (3 cases)
100% tests passed, 0 tests failed out of 8
```

累计 **8 个 bin / 33 个用例全部通过**（M1 22 + M2 新增 11）。

### 5.2 ARM 固件编译

```
cmake -S . -B build_arm -DCMAKE_BUILD_TYPE=Debug
cmake --build build_arm
```

```
[100%] Linking C executable FengMH.elf
Memory region         Used Size  Region Size  %age Used
         ITCMRAM:           0 B        64 KB      0.00%
         DTCMRAM:           0 B       128 KB      0.00%
           FLASH:       64472 B         1 MB      6.15%
          RAM_D1:       87048 B       320 KB     26.56%
          RAM_D2:           0 B        32 KB      0.00%
          RAM_D3:           0 B        16 KB      0.00%
Building FengMH.hex
Building FengMH.bin
```

固件与 App 层一体化构建通过，生成 `.elf / .hex / .bin`。

### 5.3 过程中定位到的两个问题（已修复）

- `port.c` 编译时 Assembler 报错 `selected FPU does not support instruction -- vstmdbeq r0!,{s16-s31}`：CubeMX 重新生成的 `CMakeLists.txt` 丢了 FPU 选项。已加 `-mfloat-abi=hard -mfpu=fpv5-d16` 到 compile/link 两端。
- `test_gait.c :: t_trot_periodic`：200 次 0.002s 累加后 phase≈0.999999（浮点单精度误差）。改成容忍 0 或 1 两个等价终点。

---

## 6. 遗漏 / 未覆盖的测试

1. **真机 USB CDC 联通**：`usbd_cdc_if.c` 改动在板上未实测；host 侧已通过 `bsp_usb_cdc_test_inject_rx → task_comm_get_chassis` 端到端验证逻辑正确性。
2. **真机 FreeRTOS 任务运行顺序**：任务栈大小（512/512/1024/2048）是经验值，暂未打开 `configUSE_TRACE_FACILITY` 做栈水位检测。
3. **步态切换时关节角的连续性**：当前 BLEND 只做 `lerp(hip/knee/wheel)`；由于 `hip_rad/knee_rad` 在 M2 被 trot 临时占用为 `(dx, dz)` 摆幅，M3 接 IK 后需要重新审视过渡曲线。
4. **motor vtable 具体驱动**：`motor_m3508.c / motor_go.c` 仍未新建；`leg_controller_apply` 走 `set_position/set_velocity`，旧 `Core/Src/M3508.c` 的 `set_motor_current_can*` 没被新路径调用，板上真机需要先把电机的 vtable 实现做出来再触发任务。
5. **task_safety 真实事件源**：目前只支持 `task_safety_estop_set()` 手动置位；`motor_state.last_rx_tick / temperature_c` 扫描等 M3 再补。
6. **CubeMX 重新生成对 CMakeLists 的影响**：本次已在文件中加中文注释提示；还未做出"模板外挂 include 文件"的结构化隔离，下次重新生成要手工再贴回。
7. **上行帧 `0x80/0x81`**：仍未实现。
8. **stand 的 hip/knee 目标**：本次 M2 让 stand 输出 0（待 M3 接 IK 后用 body_height 反解）。板上把 0 推给电机会"摊平"，真机上电不能直接开 stand 转电机——这是已知风险，板上第一次测试必须先禁用 chassis 任务或把 s_estop 置 true。

---

## 7. 项目使用讲解

### 7.1 构建

固件（ARM）：
```bash
cmake -S . -B build_arm -DCMAKE_BUILD_TYPE=Debug
cmake --build build_arm               # 生成 FengMH.elf / .hex / .bin
cmake --build build_arm --target flash
```

Host 单测：
```bash
cmake -S App/test -B build_host
cmake --build build_host
ctest --test-dir build_host --output-on-failure
```

### 7.2 运行时数据流（板上）

```
上位机  ─USB CDC─►  usbd_cdc_if.c CDC_Receive_HS
                          │
                          ▼  bsp_usb_cdc_on_rx()
                          │
                          ▼  proto_frame_parser（0x55 0xAA 状态机 + 校验）
                          │
                          ▼  proto_dispatcher（FuncID=0x10 → handle_chassis）
                          │
                          ▼  task_comm 的 s_chassis.{vx,vy,wz}
                          │
  task_chassis (500Hz)  ─读─►  gait_machine.update(dt)
                                │
                                ▼
                         gait_output_t（4 条腿）
                                │
                                ▼
                         leg_controller_apply
                                │
                                ▼
                   motor_dev_t->ops->set_position / set_velocity
                                │
                                ▼
                     （M3 阶段补）真实 CAN 发送
```

### 7.3 上位机怎么发一帧 chassis 指令

```python
import struct, serial
vx, vy, wz = 0.3, 0.0, 0.5
payload = struct.pack('<fff', vx, vy, wz)  # 12B
head    = bytes([0x55, 0xAA, 0x10, 12])
pkt     = head + payload
cksum   = sum(pkt) & 0xFF
pkt    += bytes([cksum])
serial.Serial('/dev/tty.usbmodemXXXX').write(pkt)
```

### 7.4 切换步态 / 手动急停（代码内示例）

```c
/* 手动切到 trot（blend 0.5s） */
gait_machine_request(&s_gm, s_trot, &GAIT_PARAMS_TROT_DEFAULT, 0.5f);

/* 软急停：所有电机 disable，chassis 任务挂起 */
task_safety_estop_set(true);
```

### 7.5 连新电机驱动（未来）

1. 实现 `motor_m3508.c`，填充 `motor_ops_t` 并封装一个 `motor_m3508_create(motor_dev_t*, bus, can_id, ...)`；
2. 在 `app_init()` 或 `task_chassis_init()` 里创建实例并 `motor_registry_bind()`；
3. 其它层（gait / leg / safety）不用改。

---

## 8. 下一步（M3 起点）

- [ ] `kinematics/leg_ik.c`：把 `(dx, dz, body_height)` 解成 hip/knee 关节角；`gait_stand / gait_trot` 的 `leg_target` 填实。
- [ ] `motor_m3508.c` vtable 实现（CAN1/CAN2 群发帧聚合）、`motor_go.c`（走 USARTx DMA，复用旧 `GO-motor.c` 的协议解析）。
- [ ] `task_safety` 扫描 `motor_state.last_rx_tick > 100ms` 自动急停。
- [ ] 上行帧 `0x80 整机状态` / `0x81 电机状态` 100Hz / 20Hz。
- [ ] 板上真机 USB CDC 联通测试：上位机 loopback + 电机不通电空跑 chassis 任务观察 CAN TX 帧。
- [ ] SEGGER RTT 后端接入（`Middlewares/Third_Party/SEGGER/`）。

本次推进对旧固件零回归，新路径已完整打通从 USB 到步态到电机接口，只差"电机 vtable + IK"就能驱动真实硬件。
