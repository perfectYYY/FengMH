# App Code Map

This is the file-level map for the maintained App firmware. It tells you where
to look first. For every function, use [`app_function_index.md`](app_function_index.md).

## App Task Layer

### `App/app/src/app_init.c`

Board bring-up entry. Initializes log, time, buses, motor registry, motors, and
IMU.

### `App/app/src/app_tasks.c`

Creates application tasks. It initializes communication and chassis control
before starting FreeRTOS tasks on MCU builds.

### `App/app/src/task_comm.c`

USB CDC protocol ingress and telemetry egress.

Important responsibilities:

- parse protocol frames
- cache the latest chassis command
- update heartbeat time
- dispatch gait and MIT debug commands
- send state and motor telemetry on MCU builds

### `App/app/src/task_chassis.c`

Thin RTOS wrapper around `chassis_control`.

Important responsibilities:

- adapt `task_comm` state into `chassis_control_input_t`
- run one 500 Hz control tick
- keep old `task_chassis_*` public API calls as wrappers

### `App/app/src/task_safety.c`

Emergency stop and safety monitor task.

## Control Layer

### `App/control/src/chassis/chassis_control.c`

Main behavior pipeline. This is where online/offline policy, steering, gait
selection, gait update, and motor dispatch are staged.

Read this file first when changing behavior.

### `App/control/src/chassis/chassis_planner.c`

Converts velocity command into planner output:

- `moving`
- trot parameters
- wheel speed targets

Current note: `vy` participates in the moving decision but is not a complete
lateral control path.

### `App/control/src/attitude`

Yaw estimate and yaw steering controller.

### `App/control/src/gait`

Gait implementations and gait transitions:

- `gait_machine.c`: current/target gait and blend transitions
- `gait_stand.c`: stationary stance
- `gait_trot.c`: trot phase and foot trajectory

### `App/control/src/kinematics/leg_ik.c`

Two-link leg IK/FK and four-leg mapping.

### `App/control/src/leg/leg_controller.c`

Converts gait output into motor vtable calls:

- GO hip/knee position commands
- M3508 wheel velocity commands
- optional M3508 wheel MIT debug path

### `App/control/src/script`

Script gait player and wrapper. This is a maintained feature, not a dev log:
scripts can provide keyframed gait outputs through the gait interface.

## Device Layer

### `App/device/src/motor_registry.c`

Logical motor ID table and runtime motor handle binding.

### `App/device/src/motor_go.c`

GO-8010 hip/knee motor driver over RS485/RIS.

Important responsibilities:

- create and bind GO motor instances
- convert joint commands to motor-side protocol units
- latch zero offsets from feedback
- send command frames by UART bus

### `App/device/src/motor_m3508.c`

M3508/C620 wheel driver over FDCAN.

Important responsibilities:

- create and bind wheel motor instances
- decode C620 feedback
- run velocity, position, torque, current, and MIT modes
- pack and send DJI current frames

### `App/device/src/imu_bmi088.c`

BMI088 initialization and sensor readout.

## BSP Layer

### `App/bsp/src`

Hardware adapters and host mocks:

- `bsp_fdcan.c`
- `bsp_uart.c`
- `bsp_usb_cdc.c`
- `bsp_spi.c`
- `bsp_time.c`

## Service Layer

### `App/service/src/protocol`

Protocol frame parser and function dispatcher.

### `App/service/src/pid`

Reusable PID implementation.

## Test Layer

### `host_tests`

Host-side regression tests for the readable control path. Current coverage
includes planner behavior, gait output, IK, command parsing, and direct
`chassis_control_tick()` to stub motor commands.
