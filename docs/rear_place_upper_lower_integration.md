# 后放置上下位机 V2 联调与验收

本文是当前任务赛自动控制系统的上下位机联调基准。现场行为以 STM32 的
`task_comm`、`task_arm` 和上位机的 `robot_serial_bridge`、
`competition_mission_manager`、`rear_place_planner` 实现为准。旧协议仅保留
兼容入口，比赛流程必须使用带 `command_seq` 的 V2 命令。

## 1. 职责边界

| 上位机 | STM32 |
| --- | --- |
| 管理比赛路线、物资类别、A/B 后槽库存和归还侧别 | 执行机械臂运动学、轨迹、限位和本地安全保护 |
| 将相机坐标转换为机械臂基座坐标，单位 m | 执行目标和主泵命令，维护 A/B 槽保持输出 |
| 按 V2 `command_seq` 重发并等待明确终态 | 按序列幂等执行并通过 `0x88` 报告命令生命周期 |
| 离开机械臂模式前完成主泵关闭互锁 | 模式边沿停止旧轨迹、关闭主泵并接管到固定姿态 |

只有 `competition_mission_manager` 发布整机模式意图。串口桥负责可靠投递、
主泵关闭互锁、重连恢复和 MCU 状态确认；Action 节点不得绕过串口桥直接打开
USB CDC，也不得自行切换整机模式。

## 2. 物理帧和 V2 载荷

链路为 STM32 USB CDC，配置值 `115200 8N1`，原始字节流只能由
`robot_serial_bridge` 打开。帧格式：

```text
0x55 | 0xAA | func_id | payload_len | payload | checksum
```

`checksum` 是前面所有字节之和的低 8 位。整数和 IEEE-754 `float32` 均为
小端序；结构体无填充。

| 方向 | 功能号 | Python 格式 | 长度 | 载荷 |
| --- | ---: | --- | ---: | --- |
| 上位机 -> STM32 | `0x10` | `<fff` | 12 | `vx, vy, wz` |
| 上位机 -> STM32 | `0x11` | `<BfffI` | 17 | `target_type, x_m, y_m, z_m, command_seq` |
| 上位机 -> STM32 | `0x14` | `<BI` | 5 | `pump_on, command_seq` |
| 上位机 -> STM32 | `0x15` | `<BI` | 5 | `mode, command_seq` |
| STM32 -> 上位机 | `0x86` | `<Bffff` | 17 | `arm_state, end_x_m, end_y_m, end_z_m, theta1_rad` |
| STM32 -> 上位机 | `0x88` | `<IIBBBB` | 12 | `boot_id, command_seq, command_func, stage, result, actual_mode` |
| STM32 -> 上位机 | `0x89` | `<IIBBBB` | 12 | `boot_id, uptime_ms, actual_mode, safety_flags, main_pump_on, reserved` |
| STM32 -> 上位机 | `0x8A` | `<IffffffHBB` | 32 | `timestamp_ms, x_m, y_m, yaw_rad, vx_m_s, yaw_rate_rad_s, gyro_z_bias_rad_s, quality_flags, stance_mask, wheel_online_mask` |
| STM32 -> 上位机 | `0x8B` | `<IffffB` | 21 | `timestamp_ms, FL/FR/RL/RR_velocity_rad_s, online_mask` |
| STM32 -> 上位机 | `0x8C` | `<I20h2H` | 48 | 轮速目标/滤波值、电流、yaw、gyro、有效 wz、滑移残差、速度比例和诊断标志 |

固件仍能解析旧版 `0x11/0x14/0x15` 的 13/1/1 字节载荷，但旧帧没有可靠终态
关联能力，禁止用于比赛自动流程。`0x86` 是连续姿态反馈，不是 V2 命令完成
凭据；上位机只按匹配的 `0x88` 推进动作。

固件发送队列总深度为 32 帧，按 `0x88` 命令回执（8 帧）、`0x89` 系统状态
（4 帧）和其他遥测（20 帧）分成三条 FIFO。调度优先级依次为命令回执、系统
状态、普通遥测；已经交给 USB 的帧只在对应发送完成回调中出队，USB busy 时保留
队首等待重试，断开连接时清空全部队列和在途状态。上位机不能依靠普通遥测到达
顺序判断命令完成，仍须匹配 `0x88` 的会话与序列字段。

## 3. `command_seq` 和 `0x88` 完成语义

`0x11`、`0x14`、`0x15` 各自维护独立的非零 `uint32` 序列水位：

1. 新序列被接收后，STM32 先保留该序列，再校验和执行命令。
2. 同一功能号、同一 `command_seq` 的重发只返回缓存的 `0x88`，不会重复
   启动轨迹、切模式或抖动气泵；即使重发时载荷被改动，也不会二次执行。
3. 旧序列或乱序序列返回 `REJECTED/STALE_SEQUENCE`。显式 V2 `IDLE` 是上位机
   重启后开启新会话的唯一安全入口，可重置旧水位。
4. 串口写成功只代表字节已交给 USB，不代表命令成功。上位机必须同时匹配
   `boot_id + command_func + command_seq`。

`stage`：

| 值 | 名称 | 是否终态 | 含义 |
| ---: | --- | --- | --- |
| 0 | `RECEIVED` | 否 | 通信入口接受并缓存，尚未完成动作 |
| 1 | `EXECUTING` | 否 | 对应任务已开始实际执行 |
| 2 | `COMPLETED` | 是 | 对应动作的真实完成条件已经满足 |
| 3 | `REJECTED` | 是 | 命令未执行，参数、模式或序列不合法 |
| 4 | `ERROR` | 是 | 命令执行中因 watchdog、ESTOP 或内部错误终止 |

`result`：`0=OK`、`1=BAD_LENGTH`、`2=BAD_VALUE`、`3=WRONG_MODE`、
`4=INVALID_TARGET`、`5=BUSY`、`6=WATCHDOG`、`7=ESTOP`、`8=INTERNAL`、
`9=STALE_SEQUENCE`。只有 `COMPLETED + OK` 是成功，其他终态都必须终止当前
动作并进入关主泵/IDLE 恢复。

各命令的 `COMPLETED` 条件：

| 命令 | 完成条件 |
| --- | --- |
| `0x11 GRASP/PLACE` | 机械臂控制器真实到达并稳定在目标，不是刚收到坐标 |
| `0x11 STOW` | MCU 原生 PARK 固定姿态进入 `HOLDING` |
| `0x14 pump_on=1` | 主泵控制输出已经为开 |
| `0x14 pump_on=0` | 主泵已在通信入口关闭；在 ERROR/ESTOP 中也立即执行并回 `COMPLETED` |
| `0x15 mode` | STM32 已应用 `actual_mode`、底盘门控和本地安全边沿 |

`0x88.actual_mode` 是命令回执时的快照。上位机对外发布的确认模式只取周期
`0x89.actual_mode`，不把模式命令回执单独当作整机状态确认。

## 4. 模式门控和 stop-before-switch

| mode | 名称 | STM32 行为 |
| ---: | --- | --- |
| 0 | `IDLE` | 不接受新机械臂目标；主泵关闭 |
| 1 | `NAV` | 允许底盘导航；不接受新机械臂目标；主泵关闭 |
| 2 | `ARM` | 允许 GRASP、PLACE、原生 STOW/PARK 和主泵命令 |
| 3 | `ESTOP` | 停止运动输出；只允许关主泵恢复命令 |
| 4 | `ERROR` | watchdog/错误安全态；只允许关主泵和 IDLE 恢复 |
| 5 | `REAR_PLACE` | 允许后槽取箱、视觉归还目标和主泵命令；不接受 STOW |

离开 `ARM/REAR_PLACE` 或在两者之间切换时，必须执行以下顺序：

```text
上位机: 0x14 pump_off
       -> 等同序列 0x88 COMPLETED/OK
       -> 等 0x89 main_pump_on=0
       -> 0x15 new_mode
       -> 等 0x88 COMPLETED/OK 和 0x89 actual_mode=new_mode

STM32: 模式值发生变化
       -> 立即关闭主泵
       -> 停止并清除旧 Arm_Control 轨迹/目标
       -> 清除旧 V2 target/pump 活动状态
       -> 由新模式固定姿态或新目标接管
```

重复发送相同模式和相同序列必须幂等，不得重置正在执行的动作。
`ARM -> REAR_PLACE` 与 `REAR_PLACE -> ARM` 都是完整模式边沿，旧轨迹不能继续
穿过边界。A/B 后槽保持输出不属于主泵，模式边沿、通信 watchdog 和普通 ESTOP
均不得把已携带物资释放。

## 5. 机械臂目标和 MCU 原生 PARK

`target_type`：

| 值 | 名称 | 模式 | 坐标语义 |
| ---: | --- | --- | --- |
| 0 | `GRASP` | ARM / REAR_PLACE | `arm_base` 中的 X/Y/Z，单位 m |
| 1 | `PLACE` | ARM / REAR_PLACE | `arm_base` 中的 X/Y/Z，单位 m |
| 2 | `STOW` | 仅 ARM | 请求 MCU 原生 PARK；XYZ 保留，建议全部发 0，固件不使用 |

GRASP/PLACE 坐标必须有限，通信入口允许的三维距离为 `0.045-0.655 m`，之后仍
须通过机械臂逆解、限位和安全轨迹检查。后放置视觉 PLACE 的 Z 为 `0.02 m`。

`target_type=2` 不是上位机伪造的笛卡尔坐标。STM32 会终止当前抓放协议、关闭
主泵并直接执行 `config.h` 中的 `APP_ARM_PARK_J1/J2/J3_MOTOR_DEG` 固定关节
姿态；只有 PARK 到位并保持后才回该序列 `COMPLETED/OK`。上位机必须等到这个
终态后才能切 NAV，不能用固定延时替代。

`0x86` 仍需连续发送真实末端位置和 `theta1_rad`，供视觉坐标转换与诊断使用；
逆解、限位、碰撞或轨迹执行失败时应报告 `arm_state=ERROR`，同时对应 V2 目标
必须收到 `0x88 ERROR`。

## 6. A/B 后槽保持和 `0x89`

`0x89` 周期报告 MCU 会话、真实模式、主泵和安全/携带状态：

| `safety_flags` | 含义 |
| --- | --- |
| bit 0 | ESTOP active |
| bit 1 | communication watchdog active |
| bit 2 | arm command error |
| bit 3 | chassis/host communication offline |
| bit 4 | `A_HELD`：PA8 保持输出已通电 |
| bit 5 | `B_HELD`：PC8 或 PC9 保持输出已通电 |

槽位映射：

| carrier_slot | 后槽 | 固定 GRASP `(x_m, y_m, z_m)` | `0x89` 标志 |
| ---: | --- | --- | --- |
| 0 | B / 右后 / 负 Y | `(-0.2170, -0.2200, 0.3635)` | bit 5 |
| 1 | A / 左后 / 正 Y | `(-0.2170, 0.2200, 0.3635)` | bit 4 |

在 REAR_PLACE 取后槽箱时，STM32 先让末端到达对应固定 GRASP，再打开主泵，
之后才释放匹配的 A/B 保持输出，避免接近过程中掉箱。`main_pump_on` 与 A/B
标志相互独立。

这些 held 位表示下位机保持输出状态，不是负压或物体存在传感器。当前比赛
按既定假设处理：检测完成且命令成功后视为抓取/放置成功，不额外等待不存在的
传感器；任务管理器只用 held 位校验软件库存和下位机输出是否一致。

`boot_id` 非零并用于隔离 MCU 会话；上位机还会监控同一 `boot_id` 的
`uptime_ms`。若 uptime 明显回退，也按 MCU 重启处理：废止复位前在途命令，
重新执行关主泵和 IDLE 握手。正常 `uint32` uptime 回绕不会触发误判。

## 7. 一次自动后放置的实际时序

1. 管理器导航到 `place_12/place_23/place_34`，底盘停稳并保持零速度。
2. 串口桥按 stop-before-switch 完成互锁，确认 `mode=REAR_PLACE(5)`。
3. Action 校验软件库存、物资类别、carrier_slot 和 LEFT/RIGHT 归还侧。
4. 发送对应 A/B 固定 GRASP 的 V2 `0x11`，等同序列 `COMPLETED/OK`。
5. 发送 V2 `0x14 pump_on=1`；完成后，STM32 释放匹配的 A/B 保持输出。
6. 发送视觉转换后的 PLACE 目标，等同序列 `COMPLETED/OK`。
7. 发送 V2 `0x14 pump_on=0`，等 `0x88 COMPLETED/OK` 和 `0x89` 主泵关闭。
8. Action 完成并删除准确槽位库存。同一归还间隙还有箱子时保持 mode 5，继续
   下一件；没有时切 ARM，发送 `target_type=2` 原生 PARK。
9. PARK 的 `0x88 COMPLETED/OK` 到达后才切 NAV，继续下一导航段。

典型成功日志的功能顺序：

```text
0x15 REAR_PLACE(seq=M)       -> 0x88 COMPLETED/OK
0x11 GRASP(seq=T1)           -> 0x88 EXECUTING -> COMPLETED/OK
0x14 pump_on(seq=P1)         -> 0x88 EXECUTING -> COMPLETED/OK
0x11 PLACE(seq=T2)           -> 0x88 EXECUTING -> COMPLETED/OK
0x14 pump_off(seq=P2)        -> 0x88 COMPLETED/OK
0x15 ARM(seq=M2)             -> 0x88 COMPLETED/OK
0x11 STOW/PARK(seq=T3)       -> 0x88 EXECUTING -> COMPLETED/OK
0x15 NAV(seq=M3)             -> 0x88 COMPLETED/OK
```

## 8. 联调验收

- 校验所有 V2 载荷长度、字节序、非零序列和 `0x88/0x89` 字段。
- 校验同序列重复帧只重发缓存状态，不重复运动、切模式或抖动主泵。
- 校验目标、泵和模式三条序列通道相互独立，旧/乱序命令明确拒绝。
- 校验 GRASP/PLACE/PARK 都只在真实完成条件满足后回 `COMPLETED/OK`。
- 校验 pump off 在 ARM、REAR_PLACE、ERROR、ESTOP 中都可完成。
- 双向中途切换 ARM 与 REAR_PLACE，旧轨迹停止，新固定姿态接管，A/B held 位不丢。
- 断开并重连 USB、MCU 更换 `boot_id`、同 boot_id 的 uptime 回退，均废止旧命令并
  走 pump-off -> IDLE 恢复。
- 完成两箱同类、相邻类、非相邻类和单箱路线；最后一箱后必须 PARK 完成再 NAV。
