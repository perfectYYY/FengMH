# cmake/firmware.cmake
#
# ⚠️  本文件不会被 CubeMX 覆盖。
#    根 CMakeLists.txt / CMakeLists_template.txt 的末尾通过
#    `include(${CMAKE_SOURCE_DIR}/cmake/firmware.cmake)` 拉进来。
#    所有"CubeMX 模板生成时会丢失"的工程定制都应放在这里:
#      - 硬件浮点 (fpu / float-abi) 编译与链接选项
#      - newlib-nano / nosys specs
#      - App/ 分层的 include 与 sources
#      - APP_TARGET_HOST=0 平台宏
#

# === 硬件浮点: Cortex-M7 fpv5-d16 hard-ABI ===
#  (必须编译与链接都声明, 否则 libc / libm 选错变体会链接失败)
#  注意: 使用 target_* 作用到已创建的 executable,
#       避免 add_compile_options 这种 "只影响之后创建的 target" 的陷阱。
target_compile_options(${PROJECT_NAME}.elf PRIVATE -mfloat-abi=hard -mfpu=fpv5-d16)
target_link_options(${PROJECT_NAME}.elf PRIVATE -mfloat-abi=hard -mfpu=fpv5-d16)

# === newlib-nano / nosys ===
target_link_options(${PROJECT_NAME}.elf PRIVATE -specs=nano.specs -specs=nosys.specs)

# === App/ 分层源与 include ===
#  目录清单由 tools/sync_app_sources.py --check 自动校验一致性。
set(APP_DIRS
        common
        bsp
        device
        service/pid
        service/protocol
        service/gait
        service/leg
        service/kinematics
        app
)

foreach(_d IN LISTS APP_DIRS)
    target_include_directories(${PROJECT_NAME}.elf PRIVATE
            ${CMAKE_SOURCE_DIR}/App/${_d})
endforeach()

set(APP_GLOBS "")
foreach(_d IN LISTS APP_DIRS)
    list(APPEND APP_GLOBS ${CMAKE_SOURCE_DIR}/App/${_d}/*.c)
endforeach()

file(GLOB APP_SOURCES ${APP_GLOBS})
target_sources(${PROJECT_NAME}.elf PRIVATE ${APP_SOURCES})
target_compile_definitions(${PROJECT_NAME}.elf PRIVATE APP_TARGET_HOST=0)
