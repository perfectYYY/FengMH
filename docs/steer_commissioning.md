# Steer 直行闭环现场调试

## 上电与安全检查

1. 将机器人架空，保持机体静止至少 1 秒，等待 BMI088 零偏标定完成。
2. 确认 `0x89.flags` 的 `bit0` 为 1；否则航向闭环会自动降级为原始 `wz` 直通。
3. 架空发送低速直行，确认四轮方向一致，`0x88` 和 `0x89` 的 FL/FR/RL/RR 顺序正确。
4. 手动让机体产生小角度偏航，确认 `effective_wz` 的方向会把 yaw 拉回锁存值。

## 下行命令

- 旧 `0x10`、12 字节 payload：`vx, vy, wz` 三个 little-endian float，默认 `AUTO_HOLD`。
- 扩展 `0x10`、17 字节 payload：前三个 float 后追加 `target_yaw` float 和 `steer_mode` byte。
- `steer_mode=0`：OFF；`1`：ABSOLUTE；`2`：AUTO_HOLD。
- AUTO_HOLD 在 `|vx| > 0.05 m/s` 且 `|wz| < 0.05 rad/s` 时锁存当前 yaw。手动转弯或停车会释放锁存，下次直行重新锁存。

## 上行诊断

`0x88` 保持原格式，25 Hz 输出四轮实际速度 float，单位 rad/s。

`0x89` 为 48 字节 little-endian payload，25 Hz：

| 偏移 | 字段 | 类型 | 换算 |
| ---: | --- | --- | --- |
| 0 | timestamp_ms | uint32 | ms |
| 4 | target[4] | int16[4] | /1000 = rad/s |
| 12 | filtered[4] | int16[4] | /1000 = rad/s |
| 20 | cmd_current[4] | int16[4] | C620 raw |
| 28 | actual_current[4] | int16[4] | C620 raw |
| 36 | yaw | int16 | /1000 = rad |
| 38 | gyro_z | int16 | /1000 = rad/s |
| 40 | effective_wz | int16 | /1000 = rad/s |
| 42 | slip_residual | int16 | /1000 = rad/s |
| 44 | speed_scale | uint16 | /1000 |
| 46 | flags | uint16 | 状态位 |

flags：`bit0 IMU_READY`、`bit1 HEADING_HOLD`、`bit2 WHEEL_SATURATED`、`bit3 SLIP_WARNING`、`bit4 SLIP_ACTIVE`、`bit5 CURRENT_LIMITED`。

## 分级路试

每种地面按 `0.15 -> 0.30 -> 0.50 -> 0.70 m/s` 逐级测试，每级先走 2 m，确认无振荡后再走 5 m。记录完整的 `0x88/0x89` 数据，并测量终点航向和横向偏移。

优先按现象调参：

- 缓慢单向偏移且 `SLIP_*` 未触发：先检查轮径、安装阻力和载荷，再在 `g_chassis_wheel_scale` 内做 0.97 到 1.03 的小幅轮速标定。
- yaw 左右快速摆动：降低 `Kp` 或增大 `Kd`；回正太慢：小幅提高 `Kp`。
- 长时间有固定残差：最后再小幅提高 `Ki`，同时观察积分引起的摆动。
- 高摩擦地面出现 `CURRENT_LIMITED`：检查轴承、轮胎干涉和左右载荷，不要用轮速补偿掩盖机械阻力。
- 低摩擦地面出现 `SLIP_ACTIVE`：系统会自动降至 0.6 倍；严重残差降至 0.4 倍，残差稳定 1 秒后缓慢恢复。

验收标准：均匀地面直行 5 m，航向误差不超过 2 度，横向偏移不超过 10 cm；至少在两种摩擦系数明显不同的地面重复验证。
