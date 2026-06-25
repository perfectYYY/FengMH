# FengMH Firmware Docs

This directory contains the current, maintained documentation for the firmware
on branch `remake_original`. The goal is to explain how the code is organized
and how commands move through the robot, without keeping development logs in the
main reading path.

## Read First

1. [`ARCHITECTURE.md`](ARCHITECTURE.md): layer responsibilities and ownership
   rules.
2. [`firmware_control_flow.md`](firmware_control_flow.md): command-to-motor
   runtime path.
3. [`app_code_map.md`](app_code_map.md): file-level map of the App layer.
4. [`app_function_index.md`](app_function_index.md): function lookup index.

## Documentation Rules

- Keep current behavior docs here.
- Keep temporary plans, session notes, and dev logs out of this directory.
- Prefer small documents with one job over long mixed-purpose notes.
- When a refactor adds or renames important functions, update
  `app_function_index.md`.
- When responsibilities move between files, update `app_code_map.md` and, if the
  runtime path changed, `firmware_control_flow.md`.

## Current Scope

The maintained docs describe the readable firmware control path:

```text
USB command
  -> task_comm
  -> task_chassis
  -> chassis_control
  -> chassis_planner
  -> gait + IK
  -> leg_controller
  -> GO / M3508 motor drivers
  -> BSP buses
```

Important behavior note: `vy` is currently passed through to the planner and
participates in the moving decision, but it is not yet a complete lateral
control implementation.

## Verification

Use these checks after code or documentation-adjacent refactors:

```sh
cmake --build build_arm
cmake --build build_host_tests
ctest --test-dir build_host_tests --output-on-failure
git diff --check
```
