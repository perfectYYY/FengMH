# Firmware Architecture

This document defines the ownership boundaries for the maintained firmware
code. Use it before deciding where new code or refactors belong.

## Layers

```text
App/app/          FreeRTOS task entry points and app lifecycle
App/control/      robot behavior, gait, kinematics, steering, leg dispatch
App/device/       motor, IMU, and registry drivers
App/bsp/          MCU bus adapters and host mocks
App/service/      reusable protocol and control utilities
App/common/       config, errors, logging, shared basics
```

## Ownership Rules

### `App/app`

Owns task entry points, protocol-facing caches, and application lifecycle.
It should stay thin:

- initialize modules
- read communication state
- call control ticks
- expose compatibility wrappers used by older code

Do not put gait math, IK, motor protocol details, or board-driver logic here.

### `App/control`

Owns robot behavior:

- online/offline decision policy
- steering and planner state
- gait selection and gait updates
- foot target to joint target conversion
- leg and wheel command dispatch

New locomotion behavior belongs here unless it is a device protocol detail.

### `App/device`

Owns concrete devices and motor vtables:

- logical motor registry binding
- GO-8010 RS485/RIS protocol
- M3508/C620 FDCAN protocol
- BMI088 device behavior

Device code may translate between physical units and protocol units, but it
should not decide high-level gait policy.

### `App/bsp`

Owns MCU peripheral access and host-side mocks:

- UART / RS485
- FDCAN
- SPI
- USB CDC
- time

Code above BSP should call stable BSP APIs rather than HAL APIs directly.

### `App/service`

Owns reusable utilities that are not robot state machines:

- protocol frame parsing and dispatch
- PID utilities

## Direction Of Dependency

Higher layers may depend on lower layers:

```text
app -> control -> device -> bsp
app -> service
control -> service
device -> service/common
```

Avoid reverse dependencies. For example, `device/` should not call
`task_chassis`, and `bsp/` should not know about gait or motor registry policy.

## Current Runtime Shape

```text
USB command
  -> task_comm
  -> task_chassis
  -> chassis_control
  -> chassis_planner
  -> gait_machine + gait implementation
  -> leg_ik + leg_controller
  -> motor registry vtable
  -> GO / M3508 drivers
  -> UART / FDCAN BSP
```

For the detailed runtime path, read
[`firmware_control_flow.md`](firmware_control_flow.md).
