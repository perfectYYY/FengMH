# IMU-wheel relative odometry protocol

## Scope

The MCU publishes a planar **relative** odometry estimate. It fuses BMI088 yaw-rate
with M3508 wheel feedback at the chassis 500 Hz control cadence. The MCU does not
integrate linear acceleration and does not use `foot_force_n` or any sole-force
sensor. Map localization, global planning, obstacle avoidance, and absolute pose
correction remain upper-computer responsibilities.

The transport frame is shared by all USB CDC messages:

```text
0x55 0xAA | FuncID:u8 | Len:u8 | Payload:Len bytes | checksum:u8
checksum = sum of all preceding bytes, modulo 256
```

All multi-byte fields are little-endian IEEE-754 `float32` unless noted otherwise.

## Coordinate system

- Origin: the pose at MCU boot, or the last accepted `0x18 ODOM_RESET` command.
- `x_m`: forward from the robot initial body heading.
- `y_m`: left from the robot initial body heading.
- `yaw_rad`: counter-clockwise positive about the upward axis, normalized to `[-pi, pi]`.
- `vx_m_s`: forward body-frame speed estimate.
- `yaw_rate_rad_s`: fused local yaw rate.

This is dead reckoning. Its `x/y/yaw` values must not be treated as a map-frame
absolute pose without an upper-computer localization correction.

## 0x8A ODOMETRY (MCU -> upper computer)

The MCU emits this frame at 50 Hz. Payload length is 32 bytes.

| Offset | Type | Field | Meaning |
|---:|---|---|---|
| 0 | u32 | `timestamp_ms` | MCU monotonic millisecond timestamp |
| 4 | f32 | `x_m` | Relative forward position |
| 8 | f32 | `y_m` | Relative left position |
| 12 | f32 | `yaw_rad` | Relative heading |
| 16 | f32 | `vx_m_s` | Body-forward speed |
| 20 | f32 | `yaw_rate_rad_s` | Fused yaw rate |
| 24 | f32 | `gyro_z_bias_rad_s` | Residual yaw-rate bias estimated by wheel/IMU fusion |
| 28 | u16 | `quality_flags` | Validity and gating bits below |
| 30 | u8 | `stance_mask` | `bit0..3 = FL/FR/RL/RR`; 1 means current gait output is stance |
| 31 | u8 | `wheel_online_mask` | `bit0..3 = FL/FR/RL/RR`; 1 means fresh wheel feedback |

`quality_flags`:

| Bit | Name | Meaning |
|---:|---|---|
| 0 | `INITIALIZED` | Odometry state exists |
| 1 | `IMU_READY` | BMI088 attitude estimator completed its boot calibration |
| 2 | `WHEEL_VALID` | At least one fresh stance wheel exists on both left and right sides |
| 3 | `WHEEL_CORRECTED` | A wheel speed or yaw-rate observation was accepted this cycle |
| 4 | `ZERO_VELOCITY` | State-driven zero-velocity correction is active |
| 5 | `TRANSITION_GATED` | Wheel observation rejected because gait state is not trusted |
| 6 | `SLIP_GATED` | Wheel observation rejected because slip/current-limit protection is active |

When `IMU_READY` is clear, upper-computer navigation must not initialize heading from
this frame. When `WHEEL_CORRECTED` is clear, position continues only from the last
velocity estimate and heading propagation; the upper computer should increase its
odometry uncertainty.

## Observation gate used by the MCU

No contact-force data participates in this decision. The gate uses only existing
control states and measured wheel feedback:

| Situation | Wheel speed treatment |
|---|---|
| `ACTIVE_STAND` and `GM_STATE_RUN` | All fresh wheels may correct forward speed and yaw rate; static detection may apply zero-velocity correction |
| `ACTIVE_TROT` or `ACTIVE_WALK`, `GM_STATE_RUN` | Only wheels whose `gait_output.leg[i].in_stance` is 1 are considered |
| Low-speed gait turn | Correct yaw-rate only; do not use wheel speed as translational velocity |
| `GM_STATE_BLEND`, `GM_STATE_IDLE`, script gait, direct wheel-test | Reject wheel observations; retain IMU prediction only |
| Missing stale wheel feedback, or no stance wheel on either side | Reject the unavailable observation |
| Existing slip warning | Retain wheel observation but use four times its nominal measurement variance |
| Existing slip-active or current-limit state | Reject wheel observations |

Zero-velocity correction requires all of the following for at least 0.5 s:
`ACTIVE_STAND`, `GM_STATE_RUN`, four fresh wheel feedback values, all wheel linear
speeds below 0.02 m/s, and IMU yaw rate below 0.03 rad/s.

## 0x18 ODOM_RESET (upper computer -> MCU)

Payload length is 12 bytes.

| Offset | Type | Field |
|---:|---|---|
| 0 | f32 | `x_m` |
| 4 | f32 | `y_m` |
| 8 | f32 | `yaw_rad` |

The MCU accepts the command only when all safety conditions hold: active gait is
stand, gait machine is `GM_STATE_RUN`, wheel-test mode is inactive, planner is not
moving, IMU calibration is complete, four wheel feedback values are fresh, all
wheel linear speeds are below 0.02 m/s, and yaw rate is below 0.03 rad/s. Invalid
or rejected commands leave odometry unchanged. `task_chassis_reset_yaw()` only resets
the attitude-estimator heading and does not change this odometry frame. The protocol has no separate ACK;
the upper computer confirms acceptance by observing the requested pose in a later
`0x8A` frame.

## Upper-computer integration

Use `timestamp_ms` to reject reordered or duplicate frames. Start relative navigation
only after `IMU_READY` is set. Preserve the latest pose through a brief gait blend,
but enlarge covariance while `TRANSITION_GATED` or `SLIP_GATED` is set. Fuse absolute
sources such as visual localization, UWB, GNSS, or map matching in the upper
computer; apply a new relative origin through `0x18` only while the robot is still.
