# Firmware Control Flow

This document describes the current readable control path on branch
`remake_original`. It is the map to use before editing motion code.

## Runtime Path

```text
USB / host command
  -> task_comm
  -> task_chassis
  -> chassis_control
  -> chassis_planner
  -> gait_machine + gait implementation
  -> leg_controller + leg_ik
  -> motor registry vtable
  -> GO / M3508 drivers
  -> UART / FDCAN BSP
```

## Step By Step

1. `task_comm` parses protocol frames and stores the latest chassis command.
   The speed command is `vx`, `vy`, `wz`; the extended command also carries
   `target_yaw` and `steer_mode`.

2. `task_chassis` is now a thin app wrapper. It reads the cached command from
   `task_comm`, copies it into `chassis_control_input_t`, and calls
   `chassis_control_tick()` at 500 Hz.

3. `chassis_control_tick()` owns the behavior pipeline:
   - runs the GO pre-calibration window on MCU builds
   - resolves online/offline mode
   - updates yaw steering when `steer_mode == 1`
   - calls `chassis_planner_update()`
   - switches stand/trot/script gait
   - updates the gait machine
   - applies planned wheel speeds only to stance legs
   - calls `leg_controller_apply_dt()`
   - flushes motor commands on MCU builds

4. `chassis_planner_update()` converts velocity command to:
   - dynamic trot parameters
   - four wheel speeds
   - `moving` flag

5. `gait_trot` outputs explicit foot targets:
   - `foot_x_m`
   - `foot_z_m`
   - `in_stance`
   - local wheel speed placeholder

6. `leg_controller_apply_dt()` calls `leg_ik_solve_all()`, then sends:
   - GO hip/knee: `set_position(pos, vel, kp, kd, tau_ff)`
   - M3508 wheel: default `set_velocity(wheel_rads)`
   - optional debug MIT wheel path through `g_leg_wheel_mit`

## Important Current Behaviors

- `vy` currently participates in the `moving` decision only. It is not yet a
  real lateral gait or wheel allocation command.
- During gait motion, stance wheels receive planner wheel speeds; swing wheels
  receive zero speed.
- M3508 wheel MIT exists as a debug path but is not the default wheel policy.
- `task_chassis` no longer owns behavior state; use `chassis_control.c` for
  behavior changes.

## Verification

Use both checks after control changes:

```sh
cmake --build build_arm
cmake -S host_tests -B build_host_tests
cmake --build build_host_tests
ctest --test-dir build_host_tests --output-on-failure
```
