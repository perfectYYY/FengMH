# FengMH 固件文档

本目录只保留当前维护中的固件文档。文档目标是说明代码结构、模块职责，以及上位机命令如何一路变成电机指令；开发日志、临时方案和会话笔记不放在这里。

## 阅读顺序

1. [`ARCHITECTURE.md`](ARCHITECTURE.md)：固件分层、职责边界和依赖方向。
2. [`firmware_control_flow.md`](firmware_control_flow.md)：从上位机命令到电机输出的运行链路。
3. [`app_code_map.md`](app_code_map.md)：App 层重要文件职责地图。
4. [`app_function_index.md`](app_function_index.md)：函数查找索引。

## 文档维护规则

- 这里只写当前行为，不写过期方案和调试流水账。
- 临时计划、会话记录、开发日志不要放进 `docs/`。
- 一个文档只承担一个职责，避免把架构、日志、TODO、验证记录混在一起。
- 重构新增或重命名重要函数时，同步更新 `app_function_index.md`。
- 文件职责移动时，同步更新 `app_code_map.md`；运行链路变化时，同步更新 `firmware_control_flow.md`。

## 当前文档覆盖范围

维护文档描述的是当前可读化后的固件控制链路：

```text
USB 命令
  -> task_comm
  -> task_chassis
  -> chassis_control
  -> chassis_planner
  -> gait + IK
  -> leg_controller
  -> GO / M3508 电机驱动
  -> BSP 总线
```

当前行为边界：`vy` 会被传给 planner，并参与 moving 判断，但还不是完整的横向控制实现。

## 常用验证

代码重构或文档相关整理后，建议执行：

```sh
cmake --build build_arm
cmake --build build_host_tests
ctest --test-dir build_host_tests --output-on-failure
git diff --check
```
