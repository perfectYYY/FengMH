# FengMH App Architecture

This layout follows the same separation used by `cyberdog_locomotion-master`,
but keeps the embedded firmware surface small:

```text
App/app/          FreeRTOS task entry points and app lifecycle
  include/        task/app public headers
  src/            task/app implementations

App/control/      locomotion pipeline: gait, kinematics, chassis, attitude
  include/<mod>/  controller-facing headers
  src/<mod>/      controller implementations

App/service/      reusable utilities that are not robot-control state
  include/<mod>/  protocol/PID headers
  src/<mod>/      protocol/PID implementations

App/device/       device drivers and motor registry
  include/        device-driver headers
  src/            device-driver implementations

App/bsp/          MCU bus adapters and host mocks
  include/        BSP headers
  src/            BSP implementations

App/common/       shared config, error codes, logging, basic types
  include/        common headers
  src/            common implementations
```

The intended data flow is:

```text
USB CDC frame
  -> app/src/task_comm.c
  -> app/src/task_chassis.c
  -> control/src/chassis planner
  -> control/src/gait + control/src/kinematics
  -> control/src/leg controller
  -> device motors
  -> bsp buses
```

`app/` should stay thin: create tasks, collect commands, call control steps, and
push commands to devices. New locomotion math belongs in `control/`; new board
drivers belong in `device/` or `bsp/`; new wire protocols belong in `service/`.
