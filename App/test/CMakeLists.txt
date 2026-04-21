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
    ${APP_ROOT}/common
    ${APP_ROOT}/bsp
    ${APP_ROOT}/device
    ${APP_ROOT}/service/pid
    ${APP_ROOT}/service/protocol
    ${APP_ROOT}/app
    ${APP_ROOT}/test/unity
)

# 共享的 App 源（host 侧能编译的部分）
set(APP_SRCS
    ${APP_ROOT}/common/log.c
    ${APP_ROOT}/bsp/bsp_time.c
    ${APP_ROOT}/bsp/bsp_fdcan.c
    ${APP_ROOT}/bsp/bsp_usb_cdc.c
    ${APP_ROOT}/device/motor_registry.c
    ${APP_ROOT}/service/pid/pid.c
    ${APP_ROOT}/service/protocol/proto_frame.c
    ${APP_ROOT}/service/protocol/proto_dispatch.c
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
