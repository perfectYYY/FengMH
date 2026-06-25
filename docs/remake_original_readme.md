# Remake Original Refactor Notes

This branch is a readability refactor of the firmware control path. The first
goal is to make the existing behavior understandable and testable before
changing motion strategy.

Read in this order:

1. `docs/firmware_control_flow.md` - command-to-motor runtime path.
2. `docs/app_code_map.md` - file-level architecture map.
3. `docs/app_function_index.md` - file/function lookup index.

Verification commands:

```sh
cmake --build build_arm
cmake -S host_tests -B build_host_tests
cmake --build build_host_tests
ctest --test-dir build_host_tests --output-on-failure
```

Current host coverage:

- planner forward/yaw behavior
- trot explicit foot-target output
- IK reading `foot_x_m` / `foot_z_m`
- direct `chassis_control_tick()` to stub motor commands
- USB protocol frame -> `task_comm` -> `task_chassis` -> `chassis_control` -> stub motor commands
