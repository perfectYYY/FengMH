# FengMH 固件文档

本目录描述当前 `/Users/code/FengMH` 仓库里的 STM32H723 + FreeRTOS 固件。文档按“实际代码现在怎么跑”维护；旧迁移计划只保留当前状态和仍有用的对账信息。

## 阅读顺序

1. [`ARCHITECTURE.md`](ARCHITECTURE.md)：启动入口、分层边界和依赖方向。
2. [`firmware_control_flow.md`](firmware_control_flow.md)：从 USB 命令到 GO/M3508/达妙/气泵输出的当前控制链路。
3. [`app_code_map.md`](app_code_map.md)：App、control、device、BSP、service 主要文件职责。
4. [`app_function_index.md`](app_function_index.md)：按文件定位重要函数。
5. [`full_control_deep_dive.md`](full_control_deep_dive.md)：上机排查粒度的代码级导读。
6. [`unified_control_code_map.md`](unified_control_code_map.md)：整机模式 gate 和上下行协议代码图。
7. [`arm_legacy_migration_audit.md`](arm_legacy_migration_audit.md)：机械臂旧 API 兼容层对账。
8. [`arm_chassis_load_compensation.md`](arm_chassis_load_compensation.md)：机械臂载荷到底盘腿部前馈补偿。
9. [`integration_migration_plan.md`](integration_migration_plan.md)、[`arm_porting_plan.md`](arm_porting_plan.md)：历史迁移计划的当前状态版。

## 当前固件总链路

```text
Core/Src/main.c
  -> app_init()
  -> osKernelInitialize()
  -> MX_FREERTOS_Init()
      -> app_tasks_create()
          -> task_comm
          -> task_chassis      [APP_CHASSIS_ENABLE]
          -> task_arm
          -> task_safety
          -> task_log
```

运行时主要控制链路：

```text
USB CDC bytes
  -> task_comm / proto_frame / proto_dispatch
  -> cached chassis, arm, pump, mode commands

mode=NAV:
  task_chassis -> chassis_control -> planner -> gait -> leg IK
    -> GO hip/knee + M3508 wheel MIT -> UART/FDCAN BSP

mode=ARM:
  task_arm -> fixed wait pose -> Arm_Serial_Protocol queue
    -> arm_control -> arm_motion/IK/gravity -> Damiao MIT + pump GPIO
    -> task_comm 0x86 ARM_FEEDBACK
```

## 当前行为边界

- 默认 `USE_LEGACY_MAIN=0`，旧 `Core/Src/main.c` 死循环业务不参与主链路。
- `APP_CHASSIS_ENABLE=1` 时创建底盘任务；关闭后仍可保留机械臂任务。
- `task_comm` 是唯一 USB CDC 下行解析入口，旧机械臂串口协议不自动接管 USB。
- `ROBOT_MODE_NAV` 才允许底盘速度进入 `chassis_control`；没有 mode 帧时保留旧调试兼容。
- `ROBOT_MODE_ARM` 只允许机械臂消费新的 USB 目标/泵命令；机械臂 task 本身常驻保持姿态。
- 底盘普通移动使用 `trot`，低速/原地 yaw 转向使用 `walk`。
- `vy` 参与 moving 判断，但当前 2DOF 腿不生成真实横向足端轨迹。
- 轮毂默认走 M3508 MIT 位置/速度参考：支撑相积分轮角，摆动相保持当前轮角。
- MCU 上 `APP_CHASSIS_COMP_FORCE_DISABLE` 默认会关闭底盘姿态和机械臂载荷前馈；host 单测仍覆盖这些算法路径。
- 机械臂达妙输出在 MCU 默认开启，host 单测默认关闭。

## 文档维护规则

- 只写当前行为；历史计划必须标清当前完成状态。
- 新增、删除、重命名重要函数时同步 [`app_function_index.md`](app_function_index.md)。
- 文件职责移动时同步 [`app_code_map.md`](app_code_map.md)。
- 控制链路变化时同步 [`firmware_control_flow.md`](firmware_control_flow.md) 和 [`unified_control_code_map.md`](unified_control_code_map.md)。

## 常用验证

```sh
cmake --build build_arm
cmake --build build_host_tests
ctest --test-dir build_host_tests --output-on-failure
git diff --check
```
