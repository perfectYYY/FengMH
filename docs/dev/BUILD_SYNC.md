# 双工具链构建一致性 (CMake/CLion ⇄ STM32CubeIDE)

本工程同时被两套构建系统消费：

| 工具链        | 入口                 | CubeMX 是否会覆写 |
|---------------|----------------------|-------------------|
| CMake (CLion) | `CMakeLists.txt`     | **会** (来自 `CMakeLists_template.txt`) |
| CMake 工程定制 | `cmake/firmware.cmake` | 不会 |
| STM32CubeIDE  | `.cproject` / `.project` | 不会 (但 CubeMX 动 MX 配置时会改) |

## 反覆写架构

CubeMX 会从模板重新生成根 `CMakeLists.txt`，导致手写内容（App 源、`-mfpu=fpv5-d16 -mfloat-abi=hard`、`-specs=nano.specs` 等）丢失。为此：

- 所有"会丢失"的定制代码都搬到 **`cmake/firmware.cmake`**（不在 CubeMX 覆写范围内）。
- 根 `CMakeLists.txt` 与 `CMakeLists_template.txt` 末尾都通过 `include(...)` 引用它。
- 模板文件（`CMakeLists_template.txt`）里预埋同一行，这样 CubeMX 下次重生成 `CMakeLists.txt` 时仍然会保留 include 行。
- 即使 CubeMX 升级把模板也重置了，跑 `python3 tools/patch_cmakelists.py` 会 **幂等地** 把 include 行补回来。

## `firmware.cmake` 里放了什么

1. 硬件浮点：`target_compile_options / target_link_options(... -mfloat-abi=hard -mfpu=fpv5-d16)`
   （必须用 `target_*` 而不是 `add_compile_options`，因为根 CMakeLists 里 `add_executable` 已经执行过了）
2. `-specs=nano.specs -specs=nosys.specs`
3. `App/` 的 include 路径与 `file(GLOB *.c)` 源
4. `APP_TARGET_HOST=0` 宏

CubeIDE 端这些参数由 `.cproject` 控制（MCU 选型 + `App/` 目录条目）。

## 添加新模块的标准流程

1. 在 `App/<layer>/<module>/` 下放 `*.c` / `*.h`。
2. 跑同步脚本：
   ```sh
   python3 tools/sync_app_sources.py            # 打印两端应有的片段
   python3 tools/sync_app_sources.py --check    # 校验两端是否一致
   python3 tools/patch_cmakelists.py --check    # 校验 include(firmware.cmake) 行健在
   ```
3. 如果 `--check` 报缺：
   - CMake 端：编辑 **`cmake/firmware.cmake`** 里的 `APP_DIRS` 列表（不是根 CMakeLists！），加一行模块相对路径。
   - CubeIDE 端：打开 `.cproject`，在 Debug `1349390055` / Release `1640766556` 两个 cconfiguration 的 `assembler.option.includepaths` 和 `c.compiler.option.includepaths` 里各加一行 `<listOptionValue builtIn="false" value="../App/<layer>/<module>"/>`。
   - `.cproject` 的 `<sourceEntries>` 已声明 `<entry name="App" excluding="test"/>`，新增 `.c` 会被 CubeIDE 自动扫到，**不需要再手动登记源文件**。
4. host 单测：在 `App/test/CMakeLists.txt` 的 `APP_SRCS` 列表追加新 `.c`（host 端没用 GLOB）。

## 平台相关代码的写法

固件路径定义 `APP_TARGET_HOST=0`，host 测试定义 `APP_TARGET_HOST=1`。涉及 HAL / FreeRTOS 的代码用：

```c
#if APP_TARGET_HOST
    /* host stub */
#else
    HAL_xxx(...);
#endif
```

## CubeMX 重新生成时的恢复

CubeMX 会覆写 `CMakeLists.txt`（从 `CMakeLists_template.txt` 渲染），有时也会把模板本身重置。执行：

```sh
python3 tools/patch_cmakelists.py     # 把 include(cmake/firmware.cmake) 补回
python3 tools/patch_cproject.py       # 把 .cproject 的 App/include 与 APP_TARGET_HOST=0 补回
python3 tools/sync_app_sources.py --check   # 验证
```

全部幂等，重复跑没有副作用。

## 更新 `APP_DIRS` 时需要同步两端

- `cmake/firmware.cmake` 里的 `APP_DIRS`（CMake 端唯一真源）
- `.cproject` 的两份 includepaths（CubeIDE 端）

这也是 `tools/sync_app_sources.py --check` 要做的事。建议接入 pre-commit / CI。
