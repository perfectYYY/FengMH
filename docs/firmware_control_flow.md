# Firmware Control Flow

This document explains the current command-to-motor path. It describes behavior
as implemented now, not desired future behavior.

## One Tick

The MCU chassis task runs at 500 Hz. One tick follows this path:

```text
task_comm cached command
  -> task_chassis read_chassis_input()
  -> chassis_control_tick()
  -> chassis_planner_update()
  -> gait_machine_update()
  -> leg_controller_apply_dt()
  -> motor vtable commands
  -> motor_go_send_all() / motor_m3508_send_all()
```

## Input Stage

`task_comm` parses USB CDC protocol frames.

- Function `0x10`: chassis command
- Base payload: `vx`, `vy`, `wz`
- Extended payload: `target_yaw`, `steer_mode`

`task_comm` only stores the latest command and heartbeat timestamp. It does not
decide gait behavior.

`task_chassis` copies the cached command into `chassis_control_input_t` and
calls `chassis_control_tick()`.

## Chassis Stage

`chassis_control_tick()` owns the behavior sequence:

1. Run the GO pre-calibration window on MCU builds.
2. Normalize missing input to a zero command.
3. Update yaw steering and compute effective `wz`.
4. Run the planner.
5. Resolve online/offline state.
6. Select stand, trot, or script gait.
7. Update the gait machine.
8. Apply wheel speeds to stance legs when online.
9. Dispatch leg commands.
10. Flush staged motor commands on MCU builds.

Current steering behavior:

- `steer_mode == 0`: use raw `wz`.
- `steer_mode == 1`: use BMI088 yaw estimate and steering controller to produce
  effective `wz`.

## Planner Stage

`chassis_planner_update()` converts the command into:

- a `moving` flag
- trot gait parameters
- four wheel speed targets

Important current limitation: `vy` is passed into the planner and participates
in the moving decision, but it is not a complete lateral motion controller.

## Gait Stage

The gait machine owns current gait and blend transitions.

- Stand outputs fixed stance targets and zero wheel speed.
- Trot outputs explicit foot targets: `foot_x_m`, `foot_z_m`, `in_stance`.
- Script gait is available through the script player wrapper.

During online movement, planner wheel speeds are applied only to stance legs.
Swing legs receive zero wheel speed.

## Leg And Motor Stage

`leg_controller_apply_dt()`:

1. Calls `leg_ik_solve_all()`.
2. Sends GO hip/knee position commands.
3. Sends M3508 wheel velocity commands.
4. Optionally uses the M3508 wheel MIT debug path when
   `g_leg_wheel_mit.enable` is set.

Default wheel behavior is velocity control, not MIT.

Motor drivers translate vtable commands into bus frames:

- GO hip/knee motors use RS485/RIS frames through `motor_go_send_all()`.
- M3508 wheel motors use FDCAN/C620 current frames through
  `motor_m3508_send_all()`.

## Safety Notes

- Offline mode falls back to conservative stand behavior unless manual hold is
  active.
- Emergency stop is handled by the safety task and disables motors before the
  chassis task sends normal outputs.
- M3508 thermal derating happens inside the M3508 driver.

## Verification

Run these checks after control-path changes:

```sh
cmake --build build_arm
cmake --build build_host_tests
ctest --test-dir build_host_tests --output-on-failure
git diff --check
```
