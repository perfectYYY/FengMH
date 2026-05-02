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
set(APP_BRINGUP_STAGE "100" CACHE STRING "Firmware bring-up stage; 100 means normal firmware")
set(APP_BRINGUP_LEG_MASK "0x01" CACHE STRING "Bring-up leg bitmask: bit0 FL, bit1 FR, bit2 RL, bit3 RR")
set(APP_BRINGUP_WHEEL_MASK "0x01" CACHE STRING "Bring-up wheel bitmask: bit0 FL, bit1 FR, bit2 RL, bit3 RR")
set(APP_BRINGUP_WHEEL_JOG_RAD_S "0.5f" CACHE STRING "Bring-up wheel jog speed in output rad/s")

#  APP_DIRS 是固件 App 层的模块清单。
set(APP_DIRS
        common
        bsp
        device
        service/pid
        service/protocol
        service/gait
        service/leg
        service/kinematics
        service/script
        service/attitude
        app
)

foreach(_d IN LISTS APP_DIRS)
    target_include_directories(${PROJECT_NAME}.elf PRIVATE
            ${CMAKE_SOURCE_DIR}/App/${_d})
endforeach()

# App/ 根目录 (支持 #include "service/pid/pid.h" 带路径引用)
target_include_directories(${PROJECT_NAME}.elf PRIVATE
        ${CMAKE_SOURCE_DIR}/App)

set(_app_globs "")
foreach(_d IN LISTS APP_DIRS)
    list(APPEND _app_globs ${CMAKE_SOURCE_DIR}/App/${_d}/*.c)
endforeach()

# CONFIGURE_DEPENDS: 新增 .c 时, 下次 build 会自动 reconfigure 感知到。
file(GLOB APP_SOURCES CONFIGURE_DEPENDS ${_app_globs})

target_sources             (${PROJECT_NAME}.elf PRIVATE ${APP_SOURCES})
target_compile_definitions (${PROJECT_NAME}.elf PRIVATE
        APP_TARGET_HOST=0
        APP_BRINGUP_STAGE=${APP_BRINGUP_STAGE}
        APP_BRINGUP_LEG_MASK=${APP_BRINGUP_LEG_MASK}
        APP_BRINGUP_WHEEL_MASK=${APP_BRINGUP_WHEEL_MASK}
        APP_BRINGUP_WHEEL_JOG_RAD_S=${APP_BRINGUP_WHEEL_JOG_RAD_S}
)

message(STATUS "APP_BRINGUP_STAGE=${APP_BRINGUP_STAGE}, LEG_MASK=${APP_BRINGUP_LEG_MASK}, WHEEL_MASK=${APP_BRINGUP_WHEEL_MASK}, WHEEL_JOG=${APP_BRINGUP_WHEEL_JOG_RAD_S}")

# -----------------------------------------------------------------------------
# 5. 健康自检
# -----------------------------------------------------------------------------
# 如果 APP_DIRS 指示要存在某个模块但实际目录缺失, 在 configure 时就报警,
# 避免到链接阶段才发现。
foreach(_d IN LISTS APP_DIRS)
    if(NOT IS_DIRECTORY ${CMAKE_SOURCE_DIR}/App/${_d})
        message(WARNING "firmware.cmake: expected App/${_d} is missing; update APP_DIRS")
    endif()
endforeach()
