# cmake/firmware.cmake
#
# 本文件是"CLion/CMake 构建端"的唯一真源 (single source of truth):
#   - App/ 分层的 include 与 sources
#   - 硬浮点 (fpv5-d16 / hard-ABI) 编译与链接选项
#   - newlib-nano / nosys specs
#   - APP_TARGET_HOST=0 平台宏
#   - 强制从 GLOB_RECURSE SOURCES 里排除 App/ 后再显式纳入固件源码
#

# -----------------------------------------------------------------------------
# 1. 清理根 CMake 的 GLOB_RECURSE 历史污染
# -----------------------------------------------------------------------------
# 根 CMakeLists.txt 的 file(GLOB_RECURSE SOURCES ...) 只收集 CubeMX 生成层。
# App/ 统一由下面的 target_sources 显式纳入, 避免意外把非固件文件卷进来。
if(DEFINED SOURCES)
    set(_filtered "")
    foreach(_s IN LISTS SOURCES)
        # 绝对路径/相对路径都覆盖
        file(RELATIVE_PATH _rel "${CMAKE_SOURCE_DIR}" "${_s}")
        if(_rel MATCHES "^App/")
            # drop
        else()
            list(APPEND _filtered "${_s}")
        endif()
    endforeach()
    set(SOURCES "${_filtered}")
endif()

# -----------------------------------------------------------------------------
# 2. 硬浮点: Cortex-M7 fpv5-d16 hard-ABI
# -----------------------------------------------------------------------------
# 必须编译与链接都声明, 否则 libc / libm 选错变体会链接失败。
# 用 target_* 作用到 executable, 避免 add_compile_options 的 "只影响之后创建
# 的 target" 陷阱; 若未来新增 add_library, 子库也要把下面这几行加到自己的 target。
target_compile_options(${PROJECT_NAME}.elf PRIVATE -mfloat-abi=hard -mfpu=fpv5-d16)
target_link_options   (${PROJECT_NAME}.elf PRIVATE -mfloat-abi=hard -mfpu=fpv5-d16)

# -----------------------------------------------------------------------------
# 3. newlib-nano / nosys
# -----------------------------------------------------------------------------
target_link_options(${PROJECT_NAME}.elf PRIVATE -specs=nano.specs -specs=nosys.specs)

# -----------------------------------------------------------------------------
# 4. App/ 分层源与 include
# -----------------------------------------------------------------------------
option(USE_LEGACY_MAIN "Build the old blocking Core/Src/main.c path instead of the FreeRTOS App tasks" OFF)

if(USE_LEGACY_MAIN)
    set(_USE_LEGACY_MAIN_DEFINE 1)
else()
    set(_USE_LEGACY_MAIN_DEFINE 0)
endif()

# APP_INCLUDE_DIRS / APP_SOURCE_DIRS 是固件 App 层的模块清单：
#   app      - RTOS task wrappers and app lifecycle
#   control  - locomotion/control pipeline
#   service  - reusable protocol/PID utilities
#   device/bsp - hardware-facing drivers and bus adapters
set(APP_INCLUDE_DIRS
        common/include
        bsp/include
        device/include
        control/include
        control/include/attitude
        control/include/chassis
        control/include/gait
        control/include/kinematics
        control/include/leg
        control/include/script
        service/include
        service/include/pid
        service/include/protocol
        app/include
)

set(APP_SOURCE_DIRS
        common/src
        bsp/src
        device/src
        control/src/attitude
        control/src/chassis
        control/src/gait
        control/src/kinematics
        control/src/leg
        control/src/script
        service/src/pid
        service/src/protocol
        app/src
)

foreach(_d IN LISTS APP_INCLUDE_DIRS)
    target_include_directories(${PROJECT_NAME}.elf PRIVATE
            ${CMAKE_SOURCE_DIR}/App/${_d})
endforeach()

# App/ 根目录 (支持未来使用 #include "control/.../x.h" 的模块路径引用)
target_include_directories(${PROJECT_NAME}.elf PRIVATE
        ${CMAKE_SOURCE_DIR}/App)

set(_app_globs "")
foreach(_d IN LISTS APP_SOURCE_DIRS)
    list(APPEND _app_globs ${CMAKE_SOURCE_DIR}/App/${_d}/*.c)
endforeach()

# CONFIGURE_DEPENDS: 新增 .c 时, 下次 build 会自动 reconfigure 感知到。
file(GLOB APP_SOURCES CONFIGURE_DEPENDS ${_app_globs})

target_sources             (${PROJECT_NAME}.elf PRIVATE ${APP_SOURCES})
target_compile_definitions (${PROJECT_NAME}.elf PRIVATE
        APP_TARGET_HOST=0
        USE_LEGACY_MAIN=${_USE_LEGACY_MAIN_DEFINE}
)

message(STATUS "USE_LEGACY_MAIN=${_USE_LEGACY_MAIN_DEFINE}")

# -----------------------------------------------------------------------------
# 5. 健康自检
# -----------------------------------------------------------------------------
# 如果清单指示要存在某个模块但实际目录缺失, 在 configure 时就报警,
# 避免到链接阶段才发现。
foreach(_d IN LISTS APP_INCLUDE_DIRS APP_SOURCE_DIRS)
    if(NOT IS_DIRECTORY ${CMAKE_SOURCE_DIR}/App/${_d})
        message(WARNING "firmware.cmake: expected App/${_d} is missing; update APP include/source dirs")
    endif()
endforeach()
