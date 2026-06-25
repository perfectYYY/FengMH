# App Code Map

This file is the human-facing map for the firmware App layer. It lists each
important file, its responsibility, and the functions that matter when changing
behavior.

## App Task Layer

### `App/app/src/task_comm.c`

Responsibility: USB CDC protocol ingestion and telemetry output.

Key functions:
- `task_comm_init()`: installs protocol parser and USB RX callback.
- `handle_chassis()`: decodes `0x10` speed command into cached chassis command.
- `handle_gait()`: decodes `0x12` stand/trot/parameter command.
- `handle_mit_cmd()`: decodes `0x13` single-motor MIT debug command.
- `task_comm_get_chassis()`: exposes the latest speed command to the chassis task.
- `task_comm_entry()`: MCU telemetry loop for state and motor reports.

### `App/app/src/task_chassis.c`

Responsibility: RTOS wrapper for chassis control.

Key functions:
- `read_chassis_input()`: adapts `task_comm` state to `chassis_control_input_t`.
- `task_chassis_step_for_test()`: single control tick used by host tests and MCU loop.
- `task_chassis_entry()`: 500 Hz MCU FreeRTOS task.
- `task_chassis_*`: compatibility wrappers that forward old public API calls to
  `chassis_control`.

## Chassis Control Layer

### `App/control/src/chassis/chassis_control.c`

Responsibility: behavior pipeline for the wheeled-leg chassis.

Key functions:
- `chassis_control_init()`: creates gait instances, binds leg motors, initializes
  steering/planner state.
- `chassis_control_tick()`: one full control cycle from command input to motor output.
- `update_steering()`: converts target yaw mode into effective `wz`.
- `online_decide()`: chooses stand or trot from planner output.
- `offline_decide()`: conservative offline fallback.
- `apply_plan_wheel_speed()`: applies wheel speeds only to stance legs.
- `chassis_control_get_status()`: diagnostics/test snapshot.

### `App/control/src/chassis/chassis_planner.c`

Responsibility: velocity command to gait parameters and wheel speeds.

Key functions:
- `chassis_planner_update()`: computes `moving`, dynamic trot params, and four
  wheel speeds.
- `planner_is_low_speed_turn()`: detects low-speed yaw turns.
- `planner_apply_turn_gait()`: overrides gait parameters for turn-in-place style motion.

## Gait Layer

### `App/control/src/gait/gait_machine.c`

Responsibility: current/target gait state and blend transitions.

Key functions:
- `gait_machine_set()`: hard switch to a gait.
- `gait_machine_request()`: request switch with optional blend.
- `gait_machine_update()`: update current gait or blend current/target outputs.

### `App/control/src/gait/gait_stand.c`

Responsibility: stationary stance target.

Key functions:
- `stand_update()`: outputs zero foot displacement, stance=true, wheel speed zero.

### `App/control/src/gait/gait_trot.c`

Responsibility: trot phase oscillator and foot trajectory.

Key functions:
- `gait_trot_foot_traj()`: stance line and swing cycloid trajectory.
- `trot_update()`: updates phase and writes explicit `foot_x_m` / `foot_z_m` targets.

## Leg And Kinematics

### `App/control/src/leg/leg_controller.c`

Responsibility: convert gait targets into motor vtable commands.

Key functions:
- `leg_controller_bind_from_registry()`: maps FL/FR/RL/RR hip/knee/wheel handles.
- `leg_controller_set_output_options()`: enables leg masks and joint/wheel outputs.
- `try_set_wheel()`: default velocity wheel path or optional debug MIT path.
- `leg_controller_apply_dt()`: IK plus hip/knee/wheel command dispatch.

### `App/control/src/kinematics/leg_ik.c`

Responsibility: two-link leg IK/FK and four-leg mapping.

Key functions:
- `leg_ik_solve()`: single-leg inverse kinematics.
- `leg_fk_solve()`: single-leg forward kinematics.
- `leg_ik_solve_all()`: maps gait foot targets to per-leg joint angles.

## Device Layer

### `App/device/src/motor_registry.c`

Responsibility: logical motor IDs to concrete motor device handles.

Key functions:
- `motor_registry_init()`: clears the registry.
- `motor_registry_bind()`: binds a logical ID to a `motor_dev_t`.
- `motor_get()`: returns the motor handle used by control code.

### `App/device/src/motor_go.c`

Responsibility: GO-8010 RS485/RIS protocol driver for hip/knee joints.

Key functions:
- `motor_go_init_all()`: creates GO instances and binds them to registry.
- `go_set_position()`: converts joint-side position command to motor-side RIS command.
- `motor_go_calibrate_all()`: latches boot zero offsets from feedback.
- `motor_go_send_all()`: sends all GO command frames over four RS485 buses.

### `App/device/src/motor_m3508.c`

Responsibility: M3508/C620 wheel driver and control loops.

Key functions:
- `motor_m3508_init_all()`: creates wheel instances and binds them to registry.
- `m3508_set_velocity()`: default wheel speed command entry.
- `m3508_set_position()`: position or MIT command entry depending on kp/kd/vel/tau.
- `motor_m3508_send_all()`: runs selected control loop and sends CAN current frame.

## Test Layer

### `host_tests/test_main.c`

Responsibility: host-side regression tests for the readable control path.

Covered tests:
- planner forward/yaw behavior
- trot explicit foot target output
- IK reading `foot_x_m` / `foot_z_m`
- end-to-end `chassis_control_tick()` to stub motor commands

### `host_tests/stubs.c`

Responsibility: host replacements for hardware drivers and MCU-only functions.

The stubs keep host tests focused on control behavior rather than HAL/FDCAN/UART
availability.
