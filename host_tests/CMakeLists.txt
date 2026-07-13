cmake_minimum_required(VERSION 3.20)

project(FengMHHostTests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(fengmh_host_tests
    test_main.c
    stubs.c
    ../App/app/src/usb_debug_text.c
    ../App/app/src/task_chassis.c
    ../App/app/src/task_comm.c
    ../App/app/src/task_arm.c
    ../App/app/src/task_safety.c
    ../App/bsp/src/bsp_gpio.c
    ../App/bsp/src/bsp_usb_cdc.c
    ../App/common/src/log.c
    ../App/control/src/arm/arm_control.c
    ../App/control/src/arm/arm_gravity_comp.c
    ../App/control/src/arm/arm_kinematics.c
    ../App/control/src/arm/arm_legacy_compat.c
    ../App/control/src/arm/arm_motion.c
    ../App/control/src/arm/arm_vision_transform.c
    ../App/control/src/attitude/attitude_estimator.c
    ../App/control/src/attitude/mahony_ahrs.c
    ../App/control/src/attitude/steer_controller.c
    ../App/control/src/chassis/chassis_control.c
    ../App/control/src/chassis/chassis_odometry.c
    ../App/control/src/chassis/chassis_planner.c
    ../App/control/src/gait/gait_if.c
    ../App/control/src/gait/gait_machine.c
    ../App/control/src/gait/gait_params.c
    ../App/control/src/gait/gait_stand.c
    ../App/control/src/gait/gait_trajectory.c
    ../App/control/src/gait/gait_trot.c
    ../App/control/src/gait/gait_walk.c
    ../App/control/src/kinematics/leg_ik.c
    ../App/control/src/kinematics/leg_params.c
    ../App/control/src/leg/leg_controller.c
    ../App/control/src/script/gait_script.c
    ../App/control/src/script/script_builtin.c
    ../App/control/src/script/script_player.c
    ../App/device/src/motor_registry.c
    ../App/device/src/motor_damiao.c
    ../App/device/src/arm_pump.c
    ../App/device/src/dm4310_posvel.c
    ../App/device/src/pump_control.c
    ../App/service/src/pid/pid.c
    ../App/service/src/protocol/arm_serial_protocol.c
    ../App/service/src/protocol/proto_dispatch.c
    ../App/service/src/protocol/proto_frame.c
)

target_compile_definitions(fengmh_host_tests PRIVATE
    APP_TARGET_HOST=1
    APP_USB_CDC_TEXT_DEBUG=0
    LOG_BACKEND_STDIO=0
)

add_executable(usb_text_host_tests
    test_usb_text.c
    ../App/app/src/usb_debug_text.c
    ../App/bsp/src/bsp_usb_cdc.c
    ../App/common/src/log.c
)

add_executable(imu_startup_host_tests
    test_imu_startup.c
    ../App/app/src/imu_startup.c
    ../App/app/src/usb_debug_text.c
    ../App/bsp/src/bsp_usb_cdc.c
    ../App/common/src/log.c
    ../App/control/src/attitude/attitude_estimator.c
    ../App/control/src/attitude/mahony_ahrs.c
)

target_compile_definitions(usb_text_host_tests PRIVATE
    APP_TARGET_HOST=1
    APP_USB_CDC_TEXT_DEBUG=1
    LOG_BACKEND_STDIO=0
)

target_compile_definitions(imu_startup_host_tests PRIVATE
    APP_TARGET_HOST=1
    APP_USB_CDC_TEXT_DEBUG=1
    LOG_BACKEND_STDIO=0
)

target_include_directories(usb_text_host_tests PRIVATE
    ../App/app/include
    ../App/bsp/include
    ../App/common/include
)

target_include_directories(imu_startup_host_tests PRIVATE
    ../App/app/include
    ../App/bsp/include
    ../App/common/include
    ../App/control/include
    ../App/control/include/attitude
    ../App/device/include
)

target_include_directories(fengmh_host_tests PRIVATE
    ../App
    ../App/app/include
    ../App/bsp/include
    ../App/common/include
    ../App/control/include
    ../App/control/include/arm
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

add_executable(chassis_odometry_host_tests
    test_odometry.c
    ../App/common/src/log.c
    ../App/control/src/attitude/attitude_estimator.c
    ../App/control/src/attitude/mahony_ahrs.c
    ../App/control/src/chassis/chassis_odometry.c
)

target_include_directories(chassis_odometry_host_tests PRIVATE
    ../App/control/include
    ../App/control/include/attitude
    ../App/control/include/chassis
    ../App/control/include/gait
    ../App/common/include
)

target_compile_definitions(chassis_odometry_host_tests PRIVATE
    APP_TARGET_HOST=1
    LOG_BACKEND_STDIO=0
)

if(UNIX)
    target_link_libraries(fengmh_host_tests PRIVATE m)
    target_link_libraries(chassis_odometry_host_tests PRIVATE m)
    target_link_libraries(usb_text_host_tests PRIVATE m)
    target_link_libraries(imu_startup_host_tests PRIVATE m)
endif()

enable_testing()
add_test(NAME fengmh_host_tests COMMAND fengmh_host_tests)
add_test(NAME chassis_odometry_host_tests COMMAND chassis_odometry_host_tests)
add_test(NAME usb_text_host_tests COMMAND usb_text_host_tests)
add_test(NAME imu_startup_host_tests COMMAND imu_startup_host_tests)
