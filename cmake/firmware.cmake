# cmake/firmware.cmake
#
# 本文件不会被 CubeMX 覆盖。
# 根 CMakeLists.txt 与 CMakeLists_template.txt 末尾各有一行:
#     include(${CMAKE_SOURCE_DIR}/cmake/firmware.cmake)
# 由 tools/patch_cmakelists.py 保证这行一直存在。
#
# 本文件是"CLion/CMake 构建端"的唯一真源 (single source of truth):
#   - App/ 分层的 include 与 sources (列表由 tools/sync_app_sources.py 校验)
#   - 硬浮点 (fpv5-d16 / hard-ABI) 编译与链接选项
#   - newlib-nano / nosys specs
#   - APP_TARGET_HOST=0 平台宏
#   - 强制从 GLOB_RECURSE SOURCES 里排除 App/ 与 App/test/
#
# CubeIDE 端由 .cproject 的 sourceEntries + listOptionValue 独立维护,
# 双端的 App/ 清单都由 tools/sync_app_sources.py --check 做一致性校验。
#

# -----------------------------------------------------------------------------
# 1. 清理根 CMake 的 GLOB_RECURSE 历史污染
# -----------------------------------------------------------------------------
# 历史上根 CMakeLists.txt 的 file(GLOB_RECURSE SOURCES ...) 可能把 "App/*.*"
# 卷进来, 结果 App/test/*.c (host-only, 依赖 unity) 被一起塞进固件构建。
# 在这里做一次防御性过滤: 把 SOURCES 里所有落在 App/ 下的文件剔掉,
# App/ 统一由下面的 target_sources 显式纳入。
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
    set(SOURCES "${_filtered}" PARENT_SCOPE)
    # 本作用域里也更新一下, 让紧随其后的 add_executable 看到新列表
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
#  APP_DIRS 是双工具链共享的"模块清单";
#  CubeIDE 端的 .cproject 需要包含同样的目录集合;
#  增删一项 → tools/sync_app_sources.py --check 会立刻 fail。
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
target_compile_definitions (${PROJECT_NAME}.elf PRIVATE APP_TARGET_HOST=0)

# -----------------------------------------------------------------------------
# 5. 健康自检
# -----------------------------------------------------------------------------
# 如果 tools/sync_app_sources.py 指示要存在 service/kinematics 但实际目录缺失,
# 在 configure 时就报错, 避免到链接阶段才发现。
foreach(_d IN LISTS APP_DIRS)
    if(NOT IS_DIRECTORY ${CMAKE_SOURCE_DIR}/App/${_d})
        message(WARNING "firmware.cmake: expected App/${_d} is missing; "
                        "run tools/sync_app_sources.py to update APP_DIRS")
    endif()
endforeach()
