cmake_minimum_required(VERSION 3.20)

project(FengMHHostTests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(fengmh_host_tests
    test_main.c
    stubs.c
    ../App/app/src/task_chassis.c
    ../App/app/src/task_comm.c
    ../App/app/src/task_safety.c
    ../App/bsp/src/bsp_usb_cdc.c
    ../App/common/src/log.c
    ../App/control/src/attitude/attitude_estimator.c
    ../App/control/src/attitude/steer_controller.c
    ../App/control/src/chassis/chassis_control.c
    ../App/control/src/chassis/chassis_planner.c
    ../App/control/src/gait/gait_if.c
    ../App/control/src/gait/gait_machine.c
    ../App/control/src/gait/gait_params.c
    ../App/control/src/gait/gait_stand.c
    ../App/control/src/gait/gait_trot.c
    ../App/control/src/kinematics/leg_ik.c
    ../App/control/src/kinematics/leg_params.c
    ../App/control/src/leg/leg_controller.c
    ../App/control/src/script/gait_script.c
    ../App/control/src/script/script_builtin.c
    ../App/control/src/script/script_player.c
    ../App/device/src/motor_registry.c
    ../App/service/src/pid/pid.c
    ../App/service/src/protocol/proto_dispatch.c
    ../App/service/src/protocol/proto_frame.c
)

target_compile_definitions(fengmh_host_tests PRIVATE
    APP_TARGET_HOST=1
    LOG_BACKEND_STDIO=0
)

target_include_directories(fengmh_host_tests PRIVATE
    ../App
    ../App/app/include
    ../App/bsp/include
    ../App/common/include
    ../App/control/include
    ../App/control/include/attitude
    ../App/control/include/chassis
    ../App/control/include/gait
    ../App/control/include/kinematics
    ../App/control/include/leg
    ../App/control/include/script
    ../App/device/include
    ../App/service/include
    ../App/service/include/pid
    ../App/service/include/protocol
)

if(UNIX)
    target_link_libraries(fengmh_host_tests PRIVATE m)
endif()

enable_testing()
add_test(NAME fengmh_host_tests COMMAND fengmh_host_tests)
