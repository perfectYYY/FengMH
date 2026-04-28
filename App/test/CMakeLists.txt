#
# Host-PC unit tests for FengMH App layer
#
# Build:
#   cmake -S App/test -B build_host
#   cmake --build build_host
#   ctest --test-dir build_host --output-on-failure
#
cmake_minimum_required(VERSION 3.16)
project(fengmh_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-unused-function -g)
add_compile_definitions(APP_TARGET_HOST=1)

set(APP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

include_directories(
    ${APP_ROOT}
    ${APP_ROOT}/common
    ${APP_ROOT}/bsp
    ${APP_ROOT}/device
    ${APP_ROOT}/service/pid
    ${APP_ROOT}/service/protocol
    ${APP_ROOT}/service/gait
    ${APP_ROOT}/service/leg
    ${APP_ROOT}/service/kinematics
    ${APP_ROOT}/service/script
    ${APP_ROOT}/app
    ${APP_ROOT}/test/unity
)

# 共享的 App 源（host 侧能编译的部分）
set(APP_SRCS
    ${APP_ROOT}/common/log.c
    ${APP_ROOT}/bsp/bsp_time.c
    ${APP_ROOT}/bsp/bsp_fdcan.c
    ${APP_ROOT}/bsp/bsp_uart.c
    ${APP_ROOT}/bsp/bsp_usb_cdc.c
    ${APP_ROOT}/device/motor_registry.c
    ${APP_ROOT}/device/motor_go.c
    ${APP_ROOT}/device/motor_m3508.c
    ${APP_ROOT}/service/pid/pid.c
    ${APP_ROOT}/service/protocol/proto_frame.c
    ${APP_ROOT}/service/protocol/proto_dispatch.c
    ${APP_ROOT}/service/gait/gait_if.c
    ${APP_ROOT}/service/gait/gait_params.c
    ${APP_ROOT}/service/gait/gait_stand.c
    ${APP_ROOT}/service/gait/gait_trot.c
    ${APP_ROOT}/service/gait/gait_machine.c
    ${APP_ROOT}/service/script/script_player.c
    ${APP_ROOT}/service/script/script_builtin.c
    ${APP_ROOT}/service/script/gait_script.c
    ${APP_ROOT}/service/kinematics/leg_ik.c
    ${APP_ROOT}/service/kinematics/leg_params.c
    ${APP_ROOT}/service/leg/leg_controller.c
    ${APP_ROOT}/app/task_comm.c
    ${APP_ROOT}/app/task_safety.c
    ${APP_ROOT}/app/task_chassis.c
    ${APP_ROOT}/test/unity/test_util.c
)

enable_testing()

function(add_app_test name src)
    add_executable(${name} ${src} ${APP_SRCS})
    target_link_libraries(${name} m)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_app_test(test_pid             ${APP_ROOT}/test/test_pid.c)
add_app_test(test_proto_frame     ${APP_ROOT}/test/test_proto_frame.c)
add_app_test(test_motor_registry  ${APP_ROOT}/test/test_motor_registry.c)
add_app_test(test_log             ${APP_ROOT}/test/test_log.c)
add_app_test(test_bsp_fdcan       ${APP_ROOT}/test/test_bsp_fdcan.c)
add_app_test(test_gait            ${APP_ROOT}/test/test_gait.c)
add_app_test(test_task_comm       ${APP_ROOT}/test/test_task_comm.c)
add_app_test(test_leg_controller  ${APP_ROOT}/test/test_leg_controller.c)
add_app_test(test_script          ${APP_ROOT}/test/test_script.c)
add_app_test(test_task_chassis    ${APP_ROOT}/test/test_task_chassis.c)

# === Python GUI sim 共享库 ===
# tools/leg_viz/ 下的 Python GUI 通过 ctypes 直接调用本库中的 IK/FK/script 函数
add_library(fengmh_sim SHARED
    ${APP_ROOT}/service/kinematics/leg_ik.c
    ${APP_ROOT}/service/kinematics/leg_params.c
    ${APP_ROOT}/service/script/script_player.c
    ${APP_ROOT}/service/script/script_builtin.c
    ${APP_ROOT}/device/motor_registry.c
    ${APP_ROOT}/common/log.c
)
target_link_libraries(fengmh_sim m)
target_compile_definitions(fengmh_sim PRIVATE APP_TARGET_HOST=1)
set_target_properties(fengmh_sim PROPERTIES
    OUTPUT_NAME fengmh_sim
    POSITION_INDEPENDENT_CODE ON
)
