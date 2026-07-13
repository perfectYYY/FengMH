/*
 * test_main.c - Host tests for the readable firmware control path.
 *
 * The suite covers:
 *   1. planner math for velocity/yaw commands
 *   2. trot/walk gait output and IK conversion
 *   3. chassis_control/USB end-to-end ticks using stub motors
 */
#include "chassis_control.h"
#include "chassis_odometry.h"
#include "chassis_planner.h"
#include "attitude_estimator.h"
#include "steer_controller.h"
#include "gait_params.h"
#include "gait_trot.h"
#include "gait_walk.h"
#include "imu_bmi088.h"
#include "leg_controller.h"
#include "leg_ik.h"
#include "leg_params.h"
#include "log.h"
#include "motor_damiao.h"
#include "motor_registry.h"
#include "bsp_usb_cdc.h"
#include "proto_defs.h"
#include "proto_frame.h"
#include "arm_control.h"
#include "arm_gravity_comp.h"
#include "arm_kinematics.h"
#include "arm_legacy_compat.h"
#include "arm_motion.h"
#include "arm_vision_transform.h"
#include "arm_pump.h"
#include "arm_serial_protocol.h"
#include "bsp_gpio.h"
#include "bsp_time.h"
#include "dm4310_posvel.h"
#include "pump_control.h"
#include "task_arm.h"
#include "task_chassis.h"
#include "task_comm.h"
#include "task_safety.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_ASSERT(_cond) do { \
    if (!(_cond)) { \
        fprintf(stderr, "ASSERT failed: %s at %s:%d\n", #_cond, __FILE__, __LINE__); \
        exit(1); \
    } \
} while (0)

#define TEST_ASSERT_NEAR(_a, _b, _tol) do { \
    float _va = (_a); \
    float _vb = (_b); \
    float _vt = (_tol); \
    if (fabsf(_va - _vb) > _vt) { \
        fprintf(stderr, "ASSERT near failed: %s=%f %s=%f tol=%f at %s:%d\n", \
                #_a, (double)_va, #_b, (double)_vb, (double)_vt, __FILE__, __LINE__); \
        exit(1); \
    } \
} while (0)

typedef struct {
    uint32_t set_position_count;
    uint32_t set_velocity_count;
    float last_pos;
    float last_vel;
    float last_kp;
    float last_kd;
    float last_tau;
} stub_motor_ctx_t;

static motor_dev_t s_stub_devs[MOTOR_ID_MAX];
static stub_motor_ctx_t s_stub_ctxs[MOTOR_ID_MAX];

void imu_bmi088_test_set(const imu_bmi088_data_t* data, uint8_t ready);

static int stub_set_position(motor_dev_t* dev,
                             float pos,
                             float vel,
                             float kp,
                             float kd,
                             float tau_ff) {
    stub_motor_ctx_t* ctx = (stub_motor_ctx_t*)dev->drv_ctx;
    ctx->set_position_count++;
    ctx->last_pos = pos;
    ctx->last_vel = vel;
    ctx->last_kp = kp;
    ctx->last_kd = kd;
    ctx->last_tau = tau_ff;
    dev->state.angle_rad = pos;
    dev->state.velocity_rads = vel;
    dev->state.online = 1U;
    return APP_OK;
}

static int stub_set_velocity(motor_dev_t* dev, float vel) {
    stub_motor_ctx_t* ctx = (stub_motor_ctx_t*)dev->drv_ctx;
    ctx->set_velocity_count++;
    ctx->last_vel = vel;
    dev->state.velocity_rads = vel;
    dev->state.online = 1U;
    return APP_OK;
}

static int stub_enable(motor_dev_t* dev) {
    dev->state.online = 1U;
    return APP_OK;
}

static int stub_disable(motor_dev_t* dev) {
    dev->state.online = 0U;
    return APP_OK;
}

static const motor_ops_t S_STUB_OPS = {
    .set_current = NULL,
    .set_torque = NULL,
    .set_position = stub_set_position,
    .set_velocity = stub_set_velocity,
    .enable = stub_enable,
    .disable = stub_disable,
    .reset_fault = stub_enable,
    .feed_rx = NULL,
};

static void bind_stub_motors(void) {
    motor_registry_init();
    memset(s_stub_devs, 0, sizeof(s_stub_devs));
    memset(s_stub_ctxs, 0, sizeof(s_stub_ctxs));

    for (uint32_t i = 0; i < MOTOR_ID_MAX; i++) {
        s_stub_devs[i].ops = &S_STUB_OPS;
        s_stub_devs[i].drv_ctx = &s_stub_ctxs[i];
        s_stub_devs[i].state.id = (uint16_t)i;
        s_stub_devs[i].state.online = 1U;
        motor_registry_bind((motor_logical_id_t)i, &s_stub_devs[i]);
    }
}

static void seed_arm_stub_feedback(uint32_t now_ms) {
    static const motor_logical_id_t ids[] = {
        MOTOR_ID_ARM_J1,
        MOTOR_ID_ARM_J2,
        MOTOR_ID_ARM_J3,
        MOTOR_ID_ARM_J4,
    };

    for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); i++) {
        s_stub_devs[ids[i]].state.online = 1U;
        s_stub_devs[ids[i]].state.rx_cnt = 1U;
        s_stub_devs[ids[i]].state.last_rx_tick = now_ms;
        s_stub_devs[ids[i]].state.angle_rad = 0.0f;
        s_stub_devs[ids[i]].state.velocity_rads = 0.0f;
    }
}

static void set_arm_stub_feedback_from_angles(const arm_joint_angles_t* angles,
                                              uint32_t now_ms) {
    TEST_ASSERT(angles != NULL);
    seed_arm_stub_feedback(now_ms);
    s_stub_devs[MOTOR_ID_ARM_J1].state.angle_rad = angles->theta1_motor_rad;
    s_stub_devs[MOTOR_ID_ARM_J2].state.angle_rad =
        ARM_KINEMATICS_MOTOR2_TO_PHYSICAL(angles->theta2_motor_rad);
    s_stub_devs[MOTOR_ID_ARM_J3].state.angle_rad =
        ARM_KINEMATICS_MOTOR3_TO_PHYSICAL(angles->theta3_motor_rad);
    s_stub_devs[MOTOR_ID_ARM_J4].state.angle_rad = angles->theta4_motor_rad;
}

static arm_joint_angles_t arm_test_safe_angles(float theta1_motor_rad) {
    arm_joint_angles_t target;
    memset(&target, 0, sizeof(target));
    target.theta1_motor_rad = theta1_motor_rad;
    target.theta1_geo_rad =
        theta1_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta1_offset_rad;
    target.theta2_motor_rad = APP_ARM_DEG2RAD(APP_ARM_SAFE_MOVE_J2_DEG);
    target.theta3_motor_rad = APP_ARM_DEG2RAD(APP_ARM_SAFE_MOVE_J3_DEG);
    target.theta4_motor_rad = arm_kinematics_compute_t4_from_t3(
        &ARM_KINEMATICS_DEFAULT_OFFSET,
        target.theta3_motor_rad);
    target.theta2_geo_rad =
        target.theta2_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta2_offset_rad;
    target.theta3_geo_rad =
        target.theta3_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta3_offset_rad;
    target.theta4_geo_rad =
        target.theta4_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta4_offset_rad;
    return target;
}

static arm_joint_angles_t arm_test_fixed_angles(float theta1_motor_rad) {
    arm_joint_angles_t target;
    memset(&target, 0, sizeof(target));
    target.theta1_motor_rad = theta1_motor_rad;
    target.theta1_geo_rad =
        theta1_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta1_offset_rad;
    target.theta2_motor_rad = APP_ARM_DEG2RAD(APP_ARM_FIXED_J2_MOTOR_DEG);
    target.theta3_motor_rad = ARM_KINEMATICS_MOTOR3_TO_LOGICAL(
        APP_ARM_DEG2RAD(APP_ARM_FIXED_J3_MOTOR_DEG));
    target.theta4_motor_rad = arm_kinematics_compute_t4_from_t3(
        &ARM_KINEMATICS_DEFAULT_OFFSET,
        target.theta3_motor_rad);
    target.theta2_geo_rad =
        target.theta2_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta2_offset_rad;
    target.theta3_geo_rad =
        target.theta3_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta3_offset_rad;
    target.theta4_geo_rad =
        target.theta4_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta4_offset_rad;
    return target;
}

static arm_joint_angles_t arm_test_park_angles(void) {
    arm_joint_angles_t target;
    memset(&target, 0, sizeof(target));
    target.theta1_motor_rad = APP_ARM_DEG2RAD(APP_ARM_PARK_J1_MOTOR_DEG);
    target.theta1_geo_rad =
        target.theta1_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta1_offset_rad;
    target.theta2_motor_rad = APP_ARM_DEG2RAD(APP_ARM_PARK_J2_MOTOR_DEG);
    target.theta3_motor_rad = ARM_KINEMATICS_MOTOR3_TO_LOGICAL(
        APP_ARM_DEG2RAD(APP_ARM_PARK_J3_MOTOR_DEG));
    target.theta4_motor_rad = arm_kinematics_compute_t4_from_t3(
        &ARM_KINEMATICS_DEFAULT_OFFSET,
        target.theta3_motor_rad);
    target.theta2_geo_rad =
        target.theta2_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta2_offset_rad;
    target.theta3_geo_rad =
        target.theta3_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta3_offset_rad;
    target.theta4_geo_rad =
        target.theta4_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta4_offset_rad;
    return target;
}

static arm_joint_angles_t arm_test_ik_target(float x_m, float y_m, float z_m) {
    arm_pose_t target = {
        .x_m = x_m,
        .y_m = y_m,
        .z_m = z_m,
        .pitch_rad = 0.0f,
    };
    arm_joint_angles_t angles;
    TEST_ASSERT(arm_kinematics_inverse(&ARM_KINEMATICS_DEFAULT_PARAMS,
                                       &ARM_KINEMATICS_DEFAULT_OFFSET,
                                       &target,
                                       &angles) == 0);
    while (angles.theta1_motor_rad > APP_ARM_PI) {
        angles.theta1_motor_rad -= 2.0f * APP_ARM_PI;
    }
    angles.theta4_motor_rad = arm_kinematics_compute_t4_from_t3(
        &ARM_KINEMATICS_DEFAULT_OFFSET,
        angles.theta3_motor_rad);
    angles.theta4_geo_rad =
        angles.theta4_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta4_offset_rad;
    return angles;
}

static arm_joint_angles_t arm_test_home_angles(void) {
    arm_joint_angles_t home;
    memset(&home, 0, sizeof(home));
    home.theta1_geo_rad =
        home.theta1_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta1_offset_rad;
    home.theta2_geo_rad =
        home.theta2_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta2_offset_rad;
    home.theta3_geo_rad =
        home.theta3_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta3_offset_rad;
    home.theta4_motor_rad = arm_kinematics_compute_t4_from_t3(
        &ARM_KINEMATICS_DEFAULT_OFFSET,
        home.theta3_motor_rad);
    home.theta4_geo_rad =
        home.theta4_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta4_offset_rad;
    return home;
}

static void test_arm_grasp_j1_forbidden_ranges_repeat_every_turn(void) {
    TEST_ASSERT(arm_control_grasp_j1_angle_forbidden(APP_ARM_DEG2RAD(-57.0f)) == 1U);
    TEST_ASSERT(arm_control_grasp_j1_angle_forbidden(APP_ARM_DEG2RAD(44.0f)) == 1U);
    TEST_ASSERT(arm_control_grasp_j1_angle_forbidden(APP_ARM_DEG2RAD(-180.0f)) == 0U);
    TEST_ASSERT(arm_control_grasp_j1_angle_forbidden(APP_ARM_DEG2RAD(-227.0f)) == 0U);
    TEST_ASSERT(arm_control_grasp_j1_angle_forbidden(APP_ARM_DEG2RAD(-136.0f)) == 0U);
    TEST_ASSERT(arm_control_grasp_j1_angle_forbidden(APP_ARM_DEG2RAD(-57.0f + 360.0f)) == 1U);
    TEST_ASSERT(arm_control_grasp_j1_angle_forbidden(APP_ARM_DEG2RAD(-180.0f - 720.0f)) == 0U);
    TEST_ASSERT(arm_control_grasp_j1_angle_forbidden(APP_ARM_DEG2RAD(90.0f)) == 0U);
    TEST_ASSERT(arm_control_grasp_j1_angle_forbidden(APP_ARM_DEG2RAD(270.0f + 720.0f)) == 0U);
}

static void reset_wheel_only_travel_cfg(void) {
    g_chassis_turn_cfg.enable_gait_turn = 1U;
    g_chassis_turn_cfg.match_travel_period = 1U;
    g_chassis_turn_cfg.low_vx_thresh_m_s = 0.05f;
    g_chassis_turn_cfg.max_wheel_rads = 4.0f;
    g_chassis_turn_cfg.half_track_m = 0.15f;
    g_chassis_turn_cfg.max_leg_step_m = 0.20f;
    g_chassis_turn_cfg.turn_step_height_m = 0.035f;
    g_chassis_turn_cfg.turn_period_s = 0.80f;
    g_chassis_turn_cfg.turn_duty = 0.75f;
    g_chassis_turn_cfg.turn_leg_scale = 0.25f;
    g_chassis_stride_cfg.enable = 1U;
    g_chassis_stride_cfg.wheel_only_travel = 1U;
    g_chassis_stride_cfg.slow_period_s = 0.25f;
    g_chassis_stride_cfg.fast_period_s = 0.20f;
    g_chassis_stride_cfg.fast_speed_m_s = 0.35f;
    g_chassis_stride_cfg.step_height_m = 0.055f;
    g_chassis_stride_cfg.duty = 0.60f;
}

static void test_planner_forward_and_turn(void) {
    chassis_plan_t plan;
    chassis_cmd_plan_t cmd = {
        .vx_m_s = 0.20f,
        .vy_m_s = 0.0f,
        .wz_rad_s = 0.0f,
    };

    reset_wheel_only_travel_cfg();
    chassis_planner_init();
    TEST_ASSERT(chassis_planner_update(&cmd, &GAIT_PARAMS_TROT_DEFAULT, &plan) == APP_OK);
    TEST_ASSERT(plan.moving == 1U);
    TEST_ASSERT_NEAR(plan.gait_params.step_length_m, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(plan.gait_params.turn_step_m, 0.0f, 1e-6f);
    TEST_ASSERT(plan.gait_params.period_s >= 0.20f);
    TEST_ASSERT(plan.gait_params.period_s <= 0.25f);
    TEST_ASSERT_NEAR(plan.gait_params.step_height_m, 0.055f, 1e-6f);
    TEST_ASSERT_NEAR(plan.gait_params.duty, 0.60f, 1e-6f);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(plan.gait_params.leg_step_length_m[i], 0.0f, 1e-6f);
        TEST_ASSERT(plan.wheel_rads[i] > 0.0f);
    }

    cmd.vx_m_s = 0.0f;
    cmd.wz_rad_s = 1.0f;
    TEST_ASSERT(chassis_planner_update(&cmd, &GAIT_PARAMS_TROT_DEFAULT, &plan) == APP_OK);
    TEST_ASSERT(plan.moving == 1U);
    TEST_ASSERT_NEAR(plan.gait_params.step_length_m, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(plan.gait_params.step_height_m, 0.035f, 1e-6f);
    TEST_ASSERT_NEAR(plan.gait_params.period_s, 0.228571f, 1e-5f);
    TEST_ASSERT_NEAR(plan.gait_params.duty, 0.75f, 1e-6f);
    TEST_ASSERT(plan.gait_params.turn_step_m > 0.0f);
    TEST_ASSERT_NEAR(plan.gait_params.leg_step_length_m[GAIT_LEG_FL], -0.006429f, 1e-5f);
    TEST_ASSERT_NEAR(plan.gait_params.leg_step_length_m[GAIT_LEG_FR], 0.006429f, 1e-5f);
    TEST_ASSERT(plan.wheel_rads[GAIT_LEG_FL] < 0.0f);
    TEST_ASSERT(plan.wheel_rads[GAIT_LEG_FR] > 0.0f);
    TEST_ASSERT_NEAR(plan.wheel_rads[GAIT_LEG_FL], plan.wheel_rads[GAIT_LEG_RL], 1e-6f);
    TEST_ASSERT_NEAR(plan.wheel_rads[GAIT_LEG_FR], plan.wheel_rads[GAIT_LEG_RR], 1e-6f);

    cmd.vx_m_s = 0.20f;
    cmd.wz_rad_s = 0.8f;
    TEST_ASSERT(chassis_planner_update(&cmd, &GAIT_PARAMS_TROT_DEFAULT, &plan) == APP_OK);
    TEST_ASSERT_NEAR(plan.gait_params.step_length_m, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(plan.gait_params.step_height_m, 0.055f, 1e-6f);
    TEST_ASSERT_NEAR(plan.gait_params.duty, 0.60f, 1e-6f);
    TEST_ASSERT(plan.gait_params.turn_step_m > 0.0f);
    TEST_ASSERT(plan.gait_params.leg_step_length_m[GAIT_LEG_FL] < 0.0f);
    TEST_ASSERT(plan.gait_params.leg_step_length_m[GAIT_LEG_FR] > 0.0f);
    TEST_ASSERT(plan.gait_params.leg_step_length_m[GAIT_LEG_FR] >
                plan.gait_params.leg_step_length_m[GAIT_LEG_FL]);
    TEST_ASSERT(plan.wheel_rads[GAIT_LEG_FR] > plan.wheel_rads[GAIT_LEG_FL]);
}

static void test_planner_uses_real_speed_and_common_wheel_scaling(void) {
    chassis_plan_t plan;
    chassis_cmd_plan_t cmd = {.vx_m_s = 0.7f, .vy_m_s = 0.0f, .wz_rad_s = 0.0f};

    reset_wheel_only_travel_cfg();
    g_chassis_turn_cfg.max_wheel_rads = 18.0f;
    for (int i = 0; i < GAIT_LEG_NUM; i++) g_chassis_wheel_scale[i] = 1.0f;
    TEST_ASSERT(chassis_planner_update(&cmd, &GAIT_PARAMS_TROT_DEFAULT, &plan) == APP_OK);
    TEST_ASSERT(plan.wheel_saturated == 0U);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(plan.wheel_rads[i], 0.7f / (LEG_DIM_DEFAULT.wheel_diameter * 0.5f), 1e-4f);
    }

    cmd.vx_m_s = 1.0f;
    cmd.wz_rad_s = 1.0f;
    TEST_ASSERT(chassis_planner_update(&cmd, &GAIT_PARAMS_TROT_DEFAULT, &plan) == APP_OK);
    TEST_ASSERT(plan.wheel_saturated == 1U);
    TEST_ASSERT_NEAR(plan.wheel_rads[GAIT_LEG_FR], 18.0f, 1e-5f);
    TEST_ASSERT(plan.wheel_rads[GAIT_LEG_FL] < plan.wheel_rads[GAIT_LEG_FR]);
    TEST_ASSERT_NEAR(plan.wheel_rads[GAIT_LEG_FL] / plan.wheel_rads[GAIT_LEG_FR],
                     (1.0f - 0.15f) / (1.0f + 0.15f), 1e-5f);
}

static void test_gyro_bias_calibration_and_heading_pid_direction(void) {
    const float accel[3] = {0.0f, 0.0f, 9.81f};
    const float bias_gyro[3] = {0.002f, -0.003f, 0.010f};
    float bias[3];

    TEST_ASSERT(attitude_estimator_init() == APP_OK);
    for (int i = 0; i < 510; i++) {
        TEST_ASSERT(attitude_estimator_update(bias_gyro, accel, 0.002f) == APP_OK);
    }
    TEST_ASSERT(attitude_estimator_is_calibrated() == 1U);
    attitude_estimator_get_gyro_bias(bias);
    TEST_ASSERT_NEAR(bias[2], 0.010f, 1e-5f);

    TEST_ASSERT(steer_controller_init() == APP_OK);
    TEST_ASSERT(steer_controller_set_mode(STEER_MODE_AUTO_HOLD) == APP_OK);
    TEST_ASSERT(steer_controller_set_target_yaw(0.0f) == APP_OK);
    TEST_ASSERT(steer_controller_update(0.2f, 0.0f, 0.002f) < 0.0f);
    TEST_ASSERT(steer_controller_update(-0.2f, 0.0f, 0.002f) > 0.0f);
    TEST_ASSERT(fabsf(steer_controller_update(-2.0f, 0.0f, 0.002f)) <= 0.5f);
}

static void calibrate_attitude(const float gyro[3], const float accel[3]) {
    TEST_ASSERT(attitude_estimator_init() == APP_OK);
    for (int i = 0; i < 510; i++) {
        TEST_ASSERT(attitude_estimator_update(gyro, accel, 0.002f) == APP_OK);
    }
    TEST_ASSERT(attitude_estimator_is_calibrated() == 1U);
}

static void test_mahony_tilt_yaw_and_reset(void) {
    const float roll_expected = 0.20f;
    const float pitch_expected = -0.10f;
    const float accel[3] = {
        -sinf(pitch_expected) * BMI088_GRAVITY,
        sinf(roll_expected) * cosf(pitch_expected) * BMI088_GRAVITY,
        cosf(roll_expected) * cosf(pitch_expected) * BMI088_GRAVITY,
    };
    const float gyro_zero[3] = {0.0f, 0.0f, 0.0f};
    const float gyro_yaw[3] = {0.0f, 0.0f, 1.57079633f};

    calibrate_attitude(gyro_zero, accel);
    const attitude_state_t* state = attitude_estimator_get_state();
    TEST_ASSERT_NEAR(state->roll, roll_expected, 1e-3f);
    TEST_ASSERT_NEAR(state->pitch, pitch_expected, 1e-3f);
    TEST_ASSERT_NEAR(state->yaw, 0.0f, 1e-4f);

    for (int i = 0; i < 500; i++) {
        TEST_ASSERT(attitude_estimator_update(gyro_yaw, accel, 0.002f) == APP_OK);
    }
    state = attitude_estimator_get_state();
    TEST_ASSERT_NEAR(state->yaw, 1.57079633f, 0.03f);
    TEST_ASSERT_NEAR(state->yaw_rate, 1.57079633f, 1e-4f);
    const float roll_before_reset = state->roll;
    const float pitch_before_reset = state->pitch;

    attitude_estimator_reset_yaw();
    state = attitude_estimator_get_state();
    TEST_ASSERT_NEAR(state->yaw, 0.0f, 1e-4f);
    TEST_ASSERT_NEAR(state->roll, roll_before_reset, 1e-6f);
    TEST_ASSERT_NEAR(state->pitch, pitch_before_reset, 1e-6f);
}

static void test_stationary_gyro_drift_is_suppressed(void) {
    const float boot_bias[3] = {0.002f, -0.003f, 0.010f};
    const float drifted_bias[3] = {0.006f, -0.005f, 0.018f};
    const float accel[3] = {0.0f, 0.0f, BMI088_GRAVITY};

    calibrate_attitude(boot_bias, accel);
    for (int i = 0; i < 5000; i++) {
        TEST_ASSERT(attitude_estimator_update(drifted_bias, accel, 0.002f) == APP_OK);
    }
    const attitude_state_t* state = attitude_estimator_get_state();
    TEST_ASSERT_NEAR(state->yaw, 0.0f, 0.02f);
    TEST_ASSERT_NEAR(state->yaw_rate, 0.0f, 1e-6f);
}

static void test_mahony_rejects_dynamic_acceleration(void) {
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    const float gravity[3] = {0.0f, 0.0f, BMI088_GRAVITY};
    const float dynamic_accel[3] = {5.0f, 0.0f, BMI088_GRAVITY};

    calibrate_attitude(gyro, gravity);
    for (int i = 0; i < 500; i++) {
        TEST_ASSERT(attitude_estimator_update(gyro, dynamic_accel, 0.002f) == APP_OK);
    }
    const attitude_state_t* state = attitude_estimator_get_state();
    TEST_ASSERT_NEAR(state->roll, 0.0f, 1e-4f);
    TEST_ASSERT_NEAR(state->pitch, 0.0f, 1e-4f);
}

static void test_attitude_rejects_invalid_sample(void) {
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    const float accel[3] = {0.0f, 0.0f, BMI088_GRAVITY};
    float bad_gyro[3] = {0.0f, 0.0f, NAN};

    calibrate_attitude(gyro, accel);
    const attitude_state_t before = *attitude_estimator_get_state();
    TEST_ASSERT(attitude_estimator_update(bad_gyro, accel, 0.002f) == APP_ERR_INVALID_ARG);
    TEST_ASSERT(attitude_estimator_update(gyro, accel, 0.0f) == APP_ERR_INVALID_ARG);
    const attitude_state_t* after = attitude_estimator_get_state();
    TEST_ASSERT_NEAR(after->yaw, before.yaw, 1e-6f);
    TEST_ASSERT(isfinite(after->roll));
    TEST_ASSERT(isfinite(after->pitch));
}

static void test_low_speed_turn_keeps_full_wheels_and_scales_leg_assist(void) {
    chassis_plan_t wheel_dominant;
    chassis_plan_t legacy_leg_assist;
    chassis_cmd_plan_t cmd = {
        .vx_m_s = 0.0f,
        .vy_m_s = 0.0f,
        .wz_rad_s = 1.0f,
    };

    reset_wheel_only_travel_cfg();
    TEST_ASSERT(chassis_planner_update(&cmd,
                                       &GAIT_PARAMS_WALK_DEFAULT,
                                       &wheel_dominant) == APP_OK);

    g_chassis_turn_cfg.turn_leg_scale = 1.0f;
    TEST_ASSERT(chassis_planner_update(&cmd,
                                       &GAIT_PARAMS_WALK_DEFAULT,
                                       &legacy_leg_assist) == APP_OK);

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(wheel_dominant.wheel_rads[i],
                         legacy_leg_assist.wheel_rads[i],
                         1e-6f);
        TEST_ASSERT_NEAR(wheel_dominant.gait_params.leg_step_length_m[i],
                         legacy_leg_assist.gait_params.leg_step_length_m[i] * 0.25f,
                         1e-6f);
    }
    TEST_ASSERT_NEAR(wheel_dominant.gait_params.period_s,
                     legacy_leg_assist.gait_params.period_s,
                     1e-6f);
    reset_wheel_only_travel_cfg();
}

static void test_planner_keeps_vy_as_motion_only(void) {
    chassis_plan_t plan;
    chassis_cmd_plan_t cmd = {
        .vx_m_s = 0.0f,
        .vy_m_s = 0.15f,
        .wz_rad_s = 0.0f,
    };

    TEST_ASSERT(chassis_planner_update(&cmd, &GAIT_PARAMS_TROT_DEFAULT, &plan) == APP_OK);
    TEST_ASSERT(plan.moving == 1U);
    TEST_ASSERT_NEAR(plan.gait_params.step_length_m, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(plan.gait_params.turn_step_m, 0.0f, 1e-6f);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(plan.gait_params.leg_step_length_m[i], 0.0f, 1e-6f);
        TEST_ASSERT_NEAR(plan.wheel_rads[i], 0.0f, 1e-6f);
    }
}

static void test_planner_deadband_zeroes_wheels(void) {
    chassis_plan_t plan;
    chassis_cmd_plan_t cmd = {
        .vx_m_s = 0.005f,
        .vy_m_s = 0.0f,
        .wz_rad_s = 0.03f,
    };

    TEST_ASSERT(chassis_planner_update(&cmd, &GAIT_PARAMS_TROT_DEFAULT, &plan) == APP_OK);
    TEST_ASSERT(plan.moving == 0U);
    TEST_ASSERT_NEAR(plan.gait_params.step_length_m, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(plan.gait_params.turn_step_m, 0.0f, 1e-6f);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(plan.gait_params.leg_step_length_m[i], 0.0f, 1e-6f);
        TEST_ASSERT_NEAR(plan.wheel_rads[i], 0.0f, 1e-6f);
    }
}

static void test_planner_schedules_high_frequency_wheel_only_travel(void) {
    chassis_plan_t slow;
    chassis_plan_t fast;
    chassis_cmd_plan_t cmd = {
        .vx_m_s = 0.04f,
        .vy_m_s = 0.0f,
        .wz_rad_s = 0.0f,
    };

    reset_wheel_only_travel_cfg();

    TEST_ASSERT(chassis_planner_update(&cmd, &GAIT_PARAMS_WALK_DEFAULT, &slow) == APP_OK);
    TEST_ASSERT(slow.moving == 1U);
    TEST_ASSERT_NEAR(slow.gait_params.period_s, 0.244286f, 1e-5f);
    TEST_ASSERT_NEAR(slow.gait_params.step_length_m, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(slow.gait_params.turn_step_m, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(slow.gait_params.step_height_m, 0.055f, 1e-6f);
    TEST_ASSERT_NEAR(slow.gait_params.duty, 0.60f, 1e-6f);

    cmd.vx_m_s = 0.35f;
    TEST_ASSERT(chassis_planner_update(&cmd, &GAIT_PARAMS_WALK_DEFAULT, &fast) == APP_OK);
    TEST_ASSERT_NEAR(fast.gait_params.period_s, 0.20f, 1e-6f);
    TEST_ASSERT_NEAR(fast.gait_params.step_height_m, 0.055f, 1e-6f);
    TEST_ASSERT_NEAR(fast.gait_params.duty, 0.60f, 1e-6f);
    TEST_ASSERT(fast.gait_params.period_s <= slow.gait_params.period_s);
    TEST_ASSERT_NEAR(fast.gait_params.step_length_m, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(fast.gait_params.turn_step_m, 0.0f, 1e-6f);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(fast.gait_params.leg_step_length_m[i], 0.0f, 1e-6f);
        TEST_ASSERT(fabsf(fast.wheel_rads[i]) > 1e-3f);
    }
}

static void test_planner_wheel_only_travel_can_fallback(void) {
    chassis_plan_t plan;
    chassis_cmd_plan_t cmd = {
        .vx_m_s = 0.20f,
        .vy_m_s = 0.0f,
        .wz_rad_s = 0.0f,
    };

    reset_wheel_only_travel_cfg();
    g_chassis_stride_cfg.wheel_only_travel = 0U;
    TEST_ASSERT(chassis_planner_update(&cmd, &GAIT_PARAMS_TROT_DEFAULT, &plan) == APP_OK);
    TEST_ASSERT(plan.gait_params.step_length_m > 0.0f);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT(plan.gait_params.leg_step_length_m[i] > 0.0f);
        TEST_ASSERT(plan.wheel_rads[i] > 0.0f);
    }
    reset_wheel_only_travel_cfg();
}

static void test_trot_turn_step_drives_left_right_gait(void) {
    gait_if_t* trot = gait_trot_create();
    gait_output_t out;
    gait_params_t params = GAIT_PARAMS_TROT_DEFAULT;

    params.step_length_m = 0.0f;
    params.turn_step_m = 0.04f;

    TEST_ASSERT(trot != NULL);
    TEST_ASSERT(trot->ops->set_param(trot, &params) == APP_OK);
    TEST_ASSERT(trot->ops->init(trot) == APP_OK);
    TEST_ASSERT(trot->ops->update(trot, 0.0f, &out) == APP_OK);

    TEST_ASSERT_NEAR(out.leg[GAIT_LEG_FL].foot_x_m, -0.02f, 1e-6f);
    TEST_ASSERT_NEAR(out.leg[GAIT_LEG_RR].foot_x_m,  0.02f, 1e-6f);
    TEST_ASSERT_NEAR(out.leg[GAIT_LEG_FR].foot_x_m, -0.02f, 1e-6f);
    TEST_ASSERT_NEAR(out.leg[GAIT_LEG_RL].foot_x_m,  0.02f, 1e-6f);
}

static void test_trot_uses_per_leg_step_lengths(void) {
    gait_if_t* trot = gait_trot_create();
    gait_output_t out;
    gait_params_t params = GAIT_PARAMS_TROT_DEFAULT;

    params.step_length_m = 0.0f;
    params.turn_step_m = 0.0f;
    params.leg_step_length_m[GAIT_LEG_FL] = -0.03f;
    params.leg_step_length_m[GAIT_LEG_FR] = 0.05f;
    params.leg_step_length_m[GAIT_LEG_RL] = -0.03f;
    params.leg_step_length_m[GAIT_LEG_RR] = 0.05f;

    TEST_ASSERT(trot != NULL);
    TEST_ASSERT(trot->ops->set_param(trot, &params) == APP_OK);
    TEST_ASSERT(trot->ops->init(trot) == APP_OK);
    TEST_ASSERT(trot->ops->update(trot, 0.0f, &out) == APP_OK);

    TEST_ASSERT_NEAR(out.leg[GAIT_LEG_FL].foot_x_m, -0.015f, 1e-6f);
    TEST_ASSERT_NEAR(out.leg[GAIT_LEG_RR].foot_x_m,  0.025f, 1e-6f);

    params.leg_step_length_m[GAIT_LEG_FL] = 0.0f;
    params.leg_step_length_m[GAIT_LEG_FR] = 0.04f;
    params.leg_step_length_m[GAIT_LEG_RL] = 0.0f;
    params.leg_step_length_m[GAIT_LEG_RR] = 0.04f;
    params.step_length_m = 0.08f;
    params.turn_step_m = 0.02f;

    TEST_ASSERT(trot->ops->set_param(trot, &params) == APP_OK);
    TEST_ASSERT(trot->ops->init(trot) == APP_OK);
    TEST_ASSERT(trot->ops->update(trot, 0.0f, &out) == APP_OK);

    TEST_ASSERT_NEAR(out.leg[GAIT_LEG_FL].foot_x_m, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(out.leg[GAIT_LEG_FR].foot_x_m, -0.02f, 1e-6f);
}

static void test_trot_outputs_explicit_foot_target(void) {
    gait_if_t* trot = gait_trot_create();
    gait_output_t out;

    TEST_ASSERT(trot != NULL);
    TEST_ASSERT(trot->ops->set_param(trot, &GAIT_PARAMS_TROT_DEFAULT) == APP_OK);
    TEST_ASSERT(trot->ops->init(trot) == APP_OK);
    TEST_ASSERT(trot->ops->update(trot, 0.02f, &out) == APP_OK);
    TEST_ASSERT(out.wheel_mode == GAIT_WHEEL_DRIVE);

    uint8_t any_foot_motion = 0U;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (fabsf(out.leg[i].foot_x_m) > 1e-6f || fabsf(out.leg[i].foot_z_m) > 1e-6f) {
            any_foot_motion = 1U;
        }
    }
    TEST_ASSERT(any_foot_motion == 1U);
}

static void test_trot_wheel_only_travel_lifts_in_place(void) {
    chassis_plan_t plan;
    chassis_cmd_plan_t cmd = {
        .vx_m_s = 0.35f,
        .vy_m_s = 0.0f,
        .wz_rad_s = 0.0f,
    };
    gait_if_t* trot = gait_trot_create();
    gait_output_t out;

    reset_wheel_only_travel_cfg();
    TEST_ASSERT(chassis_planner_update(&cmd, &GAIT_PARAMS_TROT_DEFAULT, &plan) == APP_OK);
    TEST_ASSERT_NEAR(plan.gait_params.period_s, 0.20f, 1e-6f);
    TEST_ASSERT_NEAR(plan.gait_params.step_height_m, 0.055f, 1e-6f);
    TEST_ASSERT(trot != NULL);
    TEST_ASSERT(trot->ops->set_param(trot, &plan.gait_params) == APP_OK);
    TEST_ASSERT(trot->ops->init(trot) == APP_OK);
    TEST_ASSERT(trot->ops->update(trot, 0.16f, &out) == APP_OK);

    float max_foot_z_m = 0.0f;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(out.leg[i].foot_x_m, 0.0f, 1e-6f);
        if (out.leg[i].foot_z_m > max_foot_z_m) {
            max_foot_z_m = out.leg[i].foot_z_m;
        }
    }
    TEST_ASSERT_NEAR(max_foot_z_m, 0.055f, 1e-6f);
}

static void assert_walk_support_pattern(const gait_output_t* out) {
    uint32_t stance_count = 0U;
    uint32_t swing_count = 0U;

    TEST_ASSERT(out->wheel_mode == GAIT_WHEEL_DRIVE);

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (out->leg[i].in_stance) {
            stance_count++;
        } else {
            swing_count++;
        }
        TEST_ASSERT(isfinite(out->leg[i].wheel_rads));
        TEST_ASSERT(isfinite(out->leg[i].foot_x_m));
        TEST_ASSERT(isfinite(out->leg[i].foot_z_m));
    }

    TEST_ASSERT(stance_count == 3U);
    TEST_ASSERT(swing_count == 1U);
}

static void test_walk_keeps_three_leg_support(void) {
    gait_if_t* walk = gait_walk_create();
    gait_output_t out;
    gait_params_t params = GAIT_PARAMS_WALK_DEFAULT;
    const int expected_swing_leg[4] = {
        GAIT_LEG_FL,
        GAIT_LEG_RR,
        GAIT_LEG_FR,
        GAIT_LEG_RL,
    };

    TEST_ASSERT(walk != NULL);

    for (uint32_t i = 0; i < 4U; i++) {
        TEST_ASSERT(walk->ops->set_param(walk, &params) == APP_OK);
        TEST_ASSERT(walk->ops->init(walk) == APP_OK);
        float dt_s = ((float)i * 0.25f) * params.period_s;
        TEST_ASSERT(walk->ops->update(walk, dt_s, &out) == APP_OK);
        assert_walk_support_pattern(&out);
        TEST_ASSERT(out.leg[expected_swing_leg[i]].in_stance == 0U);
    }
}

static void test_walk_uses_per_leg_step_lengths(void) {
    gait_if_t* walk = gait_walk_create();
    gait_output_t out;
    gait_params_t params = GAIT_PARAMS_WALK_DEFAULT;

    params.step_length_m = 0.0f;
    params.turn_step_m = 0.0f;
    params.leg_step_length_m[GAIT_LEG_FL] = -0.03f;
    params.leg_step_length_m[GAIT_LEG_FR] = 0.05f;
    params.leg_step_length_m[GAIT_LEG_RL] = -0.02f;
    params.leg_step_length_m[GAIT_LEG_RR] = 0.04f;

    TEST_ASSERT(walk != NULL);
    TEST_ASSERT(walk->ops->set_param(walk, &params) == APP_OK);
    TEST_ASSERT(walk->ops->init(walk) == APP_OK);
    TEST_ASSERT(walk->ops->update(walk, 0.0f, &out) == APP_OK);

    TEST_ASSERT_NEAR(out.leg[GAIT_LEG_FL].foot_x_m,  0.015f, 1e-6f);
    TEST_ASSERT_NEAR(out.leg[GAIT_LEG_FR].foot_x_m,  0.008333f, 1e-5f);
    TEST_ASSERT_NEAR(out.leg[GAIT_LEG_RL].foot_x_m, -0.010f, 1e-6f);
    TEST_ASSERT_NEAR(out.leg[GAIT_LEG_RR].foot_x_m, -0.006667f, 1e-5f);
    assert_walk_support_pattern(&out);
}

static void test_ik_reads_foot_target_fields(void) {
    gait_output_t foot;
    gait_output_t joints;
    memset(&foot, 0, sizeof(foot));

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        foot.leg[i].foot_x_m = 0.01f;
        foot.leg[i].foot_z_m = 0.02f;
        foot.leg[i].in_stance = 1U;
    }

    leg_ik_solve_all(&foot, &LEG_DIM_DEFAULT, -0.18f, &joints);

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT(isfinite(joints.leg[i].hip_rad));
        TEST_ASSERT(isfinite(joints.leg[i].knee_rad));
        TEST_ASSERT(fabsf(joints.leg[i].hip_rad) > 1e-4f);
        TEST_ASSERT(fabsf(joints.leg[i].knee_rad) > 1e-4f);
    }
}

static uint32_t total_position_commands(void) {
    uint32_t n = 0;
    for (uint32_t i = 0; i < MOTOR_ID_MAX; i++) {
        n += s_stub_ctxs[i].set_position_count;
    }
    return n;
}

static uint32_t arm_position_commands(void) {
    return s_stub_ctxs[MOTOR_ID_ARM_J1].set_position_count +
           s_stub_ctxs[MOTOR_ID_ARM_J2].set_position_count +
           s_stub_ctxs[MOTOR_ID_ARM_J3].set_position_count +
           s_stub_ctxs[MOTOR_ID_ARM_J4].set_position_count;
}

static uint32_t wheel_velocity_commands(void) {
    return s_stub_ctxs[MOTOR_ID_FL_WHEEL].set_velocity_count +
           s_stub_ctxs[MOTOR_ID_FR_WHEEL].set_velocity_count +
           s_stub_ctxs[MOTOR_ID_RL_WHEEL].set_velocity_count +
           s_stub_ctxs[MOTOR_ID_RR_WHEEL].set_velocity_count;
}

static void assert_joint_motor_position_path_active(void) {
    static const motor_logical_id_t ids[] = {
        MOTOR_ID_FL_HIP, MOTOR_ID_FL_KNEE,
        MOTOR_ID_FR_HIP, MOTOR_ID_FR_KNEE,
        MOTOR_ID_RL_HIP, MOTOR_ID_RL_KNEE,
        MOTOR_ID_RR_HIP, MOTOR_ID_RR_KNEE,
    };

    for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); i++) {
        const stub_motor_ctx_t* ctx = &s_stub_ctxs[ids[i]];
        TEST_ASSERT(ctx->set_position_count > 0U);
        TEST_ASSERT(isfinite(ctx->last_pos));
        TEST_ASSERT(isfinite(ctx->last_vel));
        TEST_ASSERT(isfinite(ctx->last_kp));
        TEST_ASSERT(isfinite(ctx->last_kd));
    }
}

static void assert_wheel_hold_mit_active(void) {
    static const motor_logical_id_t ids[] = {
        MOTOR_ID_FL_WHEEL,
        MOTOR_ID_FR_WHEEL,
        MOTOR_ID_RL_WHEEL,
        MOTOR_ID_RR_WHEEL,
    };

    for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); i++) {
        const stub_motor_ctx_t* ctx = &s_stub_ctxs[ids[i]];
        TEST_ASSERT(ctx->set_position_count > 0U);
        TEST_ASSERT(ctx->last_kp > 0.0f);
        TEST_ASSERT(ctx->last_kd > 0.0f);
    }
}

static void assert_wheel_drive_mit_active(void) {
    static const motor_logical_id_t ids[] = {
        MOTOR_ID_FL_WHEEL,
        MOTOR_ID_FR_WHEEL,
        MOTOR_ID_RL_WHEEL,
        MOTOR_ID_RR_WHEEL,
    };

    for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); i++) {
        const stub_motor_ctx_t* ctx = &s_stub_ctxs[ids[i]];
        TEST_ASSERT(ctx->set_position_count > 0U);
        TEST_ASSERT(ctx->last_kp > 0.0f);
        TEST_ASSERT(ctx->last_kd > 0.0f);
    }
    TEST_ASSERT(wheel_velocity_commands() == 0U);
}

static void assert_wheel_velocity_zero(void) {
    static const motor_logical_id_t ids[] = {
        MOTOR_ID_FL_WHEEL,
        MOTOR_ID_FR_WHEEL,
        MOTOR_ID_RL_WHEEL,
        MOTOR_ID_RR_WHEEL,
    };

    for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); i++) {
        TEST_ASSERT_NEAR(s_stub_ctxs[ids[i]].last_vel, 0.0f, 1e-6f);
    }
}

static void test_wheel_drive_uses_mit_across_leg_phases(void) {
    leg_controller_t lc;
    gait_output_t target;
    memset(&target, 0, sizeof(target));

    bind_stub_motors();
    leg_controller_init(&lc);
    leg_controller_bind_from_registry(&lc);
    leg_controller_set_stand_height(-GAIT_PARAMS_STAND_DEFAULT.body_height_m);
    target.wheel_mode = GAIT_WHEEL_DRIVE;

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        target.leg[i].in_stance = 1U;
        target.leg[i].wheel_rads = (i == GAIT_LEG_FL || i == GAIT_LEG_RL)
                                 ? -0.3f : 0.3f;
    }

    TEST_ASSERT(leg_controller_apply_dt(&lc, &target, 0.002f) == APP_OK);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_FL_WHEEL].last_vel, -0.3f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_RL_WHEEL].last_vel, -0.3f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_RR_WHEEL].last_vel, 0.3f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_FR_WHEEL].last_vel, 0.3f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_FL_WHEEL].last_tau,
                     -g_leg_wheel_mit.stance_tau_ff_nm,
                     1e-6f);
    TEST_ASSERT(wheel_velocity_commands() == 0U);
    TEST_ASSERT(total_position_commands() == 12U);

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        target.leg[i].in_stance = 0U;
    }
    TEST_ASSERT(leg_controller_apply_dt(&lc, &target, 0.002f) == APP_OK);
    TEST_ASSERT(wheel_velocity_commands() == 0U);
    TEST_ASSERT(total_position_commands() == 24U);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_FL_WHEEL].last_tau, 0.0f, 1e-6f);
    TEST_ASSERT(g_leg_wheel_mit.drive_mask == 0x0FU);
    TEST_ASSERT(g_leg_wheel_mit.hold_mask == 0x00U);
}

static void test_wheel_mit_velocity_integral_is_bounded_and_unwinds(void) {
    leg_controller_t lc;
    gait_output_t target;
    memset(&target, 0, sizeof(target));

    bind_stub_motors();
    leg_controller_init(&lc);
    leg_controller_bind_from_registry(&lc);
    leg_controller_set_stand_height(-GAIT_PARAMS_STAND_DEFAULT.body_height_m);
    target.wheel_mode = GAIT_WHEEL_DRIVE;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        target.leg[i].in_stance = 1U;
        target.leg[i].wheel_rads = 2.1f;
    }

    for (int tick = 0; tick < 500; tick++) {
        for (int i = 0; i < GAIT_LEG_NUM; i++) {
            motor_logical_id_t id = (motor_logical_id_t[]){
                MOTOR_ID_FL_WHEEL, MOTOR_ID_FR_WHEEL,
                MOTOR_ID_RL_WHEEL, MOTOR_ID_RR_WHEEL,
            }[i];
            s_stub_devs[id].state.angle_rad = 0.0f;
            s_stub_devs[id].state.velocity_rads = 0.0f;
        }
        TEST_ASSERT(leg_controller_apply_dt(&lc, &target, 0.002f) == APP_OK);
    }

    float saturated_i = g_leg_wheel_mit.velocity_i_rad[GAIT_LEG_FL];
    TEST_ASSERT(saturated_i > 0.0f);
    TEST_ASSERT(saturated_i < 0.05f);
    TEST_ASSERT(fabsf(saturated_i) <=
                g_leg_wheel_mit.tau_limit_nm / g_leg_wheel_mit.kp + 1e-6f);

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        target.leg[i].wheel_rads = -2.1f;
    }
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        s_stub_devs[(motor_logical_id_t[]){
            MOTOR_ID_FL_WHEEL, MOTOR_ID_FR_WHEEL,
            MOTOR_ID_RL_WHEEL, MOTOR_ID_RR_WHEEL,
        }[i]].state.velocity_rads = 0.0f;
    }
    TEST_ASSERT(leg_controller_apply_dt(&lc, &target, 0.002f) == APP_OK);
    TEST_ASSERT(g_leg_wheel_mit.velocity_i_rad[GAIT_LEG_FL] < saturated_i);
}

static void test_wheel_drive_to_hold_latches_actual_angle_once(void) {
    leg_controller_t lc;
    gait_output_t target;
    memset(&target, 0, sizeof(target));

    bind_stub_motors();
    leg_controller_init(&lc);
    leg_controller_bind_from_registry(&lc);
    leg_controller_set_stand_height(-GAIT_PARAMS_STAND_DEFAULT.body_height_m);

    target.wheel_mode = GAIT_WHEEL_DRIVE;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        target.leg[i].in_stance = 1U;
        target.leg[i].wheel_rads = 0.3f;
    }

    TEST_ASSERT(leg_controller_apply_dt(&lc, &target, 0.002f) == APP_OK);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_FL_WHEEL].last_vel, 0.3f, 1e-6f);
    TEST_ASSERT(g_leg_wheel_mit.ref_valid[GAIT_LEG_FL] == 1U);
    TEST_ASSERT(g_leg_wheel_mit.drive_mask == 0x0FU);
    TEST_ASSERT(g_leg_wheel_mit.hold_mask == 0x00U);

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        target.leg[i].in_stance = 0U;
    }
    s_stub_devs[MOTOR_ID_FL_WHEEL].state.angle_rad = 1.0f;
    TEST_ASSERT(leg_controller_apply_dt(&lc, &target, 0.002f) == APP_OK);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_FL_WHEEL].last_vel, 0.3f, 1e-6f);
    TEST_ASSERT(g_leg_wheel_mit.ref_valid[GAIT_LEG_FL] == 1U);

    target.wheel_mode = GAIT_WHEEL_HOLD;
    s_stub_devs[MOTOR_ID_FL_WHEEL].state.angle_rad = 0.25f;
    TEST_ASSERT(leg_controller_apply_dt(&lc, &target, 0.002f) == APP_OK);
    TEST_ASSERT_NEAR(g_leg_wheel_mit.theta_ref_rad[GAIT_LEG_FL], 0.25f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_FL_WHEEL].last_vel, 0.0f, 1e-6f);
    TEST_ASSERT(g_leg_wheel_mit.drive_mask == 0x00U);
    TEST_ASSERT(g_leg_wheel_mit.hold_mask == 0x0FU);

    s_stub_devs[MOTOR_ID_FL_WHEEL].state.angle_rad = 0.50f;
    TEST_ASSERT(leg_controller_apply_dt(&lc, &target, 0.002f) == APP_OK);
    TEST_ASSERT_NEAR(g_leg_wheel_mit.theta_ref_rad[GAIT_LEG_FL], 0.25f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_FL_WHEEL].last_pos, 0.25f, 1e-6f);
}

static void assert_joint_tau_zero(void) {
    static const motor_logical_id_t ids[] = {
        MOTOR_ID_FL_HIP, MOTOR_ID_FL_KNEE,
        MOTOR_ID_FR_HIP, MOTOR_ID_FR_KNEE,
        MOTOR_ID_RL_HIP, MOTOR_ID_RL_KNEE,
        MOTOR_ID_RR_HIP, MOTOR_ID_RR_KNEE,
    };

    for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); i++) {
        TEST_ASSERT_NEAR(s_stub_ctxs[ids[i]].last_tau, 0.0f, 1e-6f);
    }
}

static void assert_any_joint_tau_nonzero(void) {
    static const motor_logical_id_t ids[] = {
        MOTOR_ID_FL_HIP, MOTOR_ID_FL_KNEE,
        MOTOR_ID_FR_HIP, MOTOR_ID_FR_KNEE,
        MOTOR_ID_RL_HIP, MOTOR_ID_RL_KNEE,
        MOTOR_ID_RR_HIP, MOTOR_ID_RR_KNEE,
    };
    uint8_t any = 0U;

    for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); i++) {
        if (fabsf(s_stub_ctxs[ids[i]].last_tau) > 1e-5f) {
            any = 1U;
        }
    }
    TEST_ASSERT(any == 1U);
}

static void apply_stand_once_with_gravity_cfg(uint8_t enable, float payload_mass_kg) {
    leg_controller_t lc;
    gait_output_t stand;
    memset(&stand, 0, sizeof(stand));

    bind_stub_motors();
    leg_controller_init(&lc);
    leg_controller_bind_from_registry(&lc);
    leg_controller_set_stand_height(-GAIT_PARAMS_STAND_DEFAULT.body_height_m);

    g_leg_gravity_comp.enable = enable;
    g_leg_gravity_comp.compensate_leg_mass = 1U;
    g_leg_gravity_comp.compensate_payload = 1U;
    g_leg_gravity_comp.payload_mass_kg = payload_mass_kg;
    g_leg_gravity_comp.max_tau_nm = 2.0f;

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        stand.leg[i].in_stance = 1U;
    }

    TEST_ASSERT(leg_controller_apply_dt(&lc, &stand, 0.002f) == APP_OK);
}

static void test_gravity_comp_disabled_keeps_zero_tau_ff(void) {
    apply_stand_once_with_gravity_cfg(0U, 2.0f);
    assert_joint_motor_position_path_active();
    assert_wheel_hold_mit_active();
    assert_joint_tau_zero();
    TEST_ASSERT(fabsf(g_leg_gravity_comp.hip_tau_ff_nm[GAIT_LEG_FL]) > 1e-5f);
}

static void test_gravity_comp_enabled_sends_joint_tau_ff(void) {
    apply_stand_once_with_gravity_cfg(1U, 2.0f);
    assert_joint_motor_position_path_active();
    assert_wheel_hold_mit_active();
    assert_any_joint_tau_nonzero();
    TEST_ASSERT(fabsf(s_stub_ctxs[MOTOR_ID_FL_HIP].last_tau) <= g_leg_gravity_comp.max_tau_nm);
    TEST_ASSERT(fabsf(s_stub_ctxs[MOTOR_ID_FL_KNEE].last_tau) <= g_leg_gravity_comp.max_tau_nm);
}

static void test_gravity_comp_payload_com_biases_support_loads(void) {
    leg_controller_t lc;
    gait_output_t stand;
    memset(&stand, 0, sizeof(stand));

    bind_stub_motors();
    leg_controller_init(&lc);
    leg_controller_bind_from_registry(&lc);
    leg_controller_set_stand_height(-GAIT_PARAMS_STAND_DEFAULT.body_height_m);

    g_leg_gravity_comp.enable = 1U;
    g_leg_gravity_comp.compensate_leg_mass = 0U;
    g_leg_gravity_comp.compensate_payload = 1U;
    g_leg_gravity_comp.use_payload_com = 1U;
    g_leg_gravity_comp.scale = 1.0f;
    g_leg_gravity_comp.payload_mass_kg = 4.0f;
    g_leg_gravity_comp.payload_com_x_m = 0.06f;
    g_leg_gravity_comp.payload_com_y_m = 0.04f;
    g_leg_gravity_comp.support_half_length_m = 0.15f;
    g_leg_gravity_comp.support_half_track_m = 0.15f;
    g_leg_gravity_comp.max_leg_payload_kg = 0.0f;
    g_leg_gravity_comp.max_tau_nm = 5.0f;

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        stand.leg[i].in_stance = 1U;
    }

    TEST_ASSERT(leg_controller_apply_dt(&lc, &stand, 0.002f) == APP_OK);
    TEST_ASSERT(g_leg_gravity_comp.payload_active_mask == 0x0FU);
    TEST_ASSERT_NEAR(g_leg_gravity_comp.payload_applied_mass_kg, 4.0f, 1e-5f);
    TEST_ASSERT(g_leg_gravity_comp.payload_leg_mass_kg[GAIT_LEG_FL] >
                g_leg_gravity_comp.payload_leg_mass_kg[GAIT_LEG_FR]);
    TEST_ASSERT(g_leg_gravity_comp.payload_leg_mass_kg[GAIT_LEG_FL] >
                g_leg_gravity_comp.payload_leg_mass_kg[GAIT_LEG_RL]);
    TEST_ASSERT(g_leg_gravity_comp.payload_leg_mass_kg[GAIT_LEG_RR] <
                g_leg_gravity_comp.payload_leg_mass_kg[GAIT_LEG_FR]);
    assert_any_joint_tau_nonzero();
}

static void test_chassis_control_end_to_end(void) {
    chassis_control_input_t input;
    chassis_control_status_t status;
    memset(&input, 0, sizeof(input));

    reset_wheel_only_travel_cfg();
    log_init();
    bind_stub_motors();
    chassis_control_init();
    chassis_control_set_mode(CHASSIS_MODE_ONLINE);

    input.command.vx_m_s = 0.0f;
    input.command.vy_m_s = 0.0f;
    input.command.wz_rad_s = 1.0f;
    input.valid_frame_count = 1U;
    input.last_rx_ms = 0U;

    for (uint32_t tick = 0; tick < 1800U; tick++) {
        chassis_control_tick(&input, 0.002f, tick * 2U);
    }

    chassis_control_get_status(&status);
    TEST_ASSERT(status.online == 1U);
    TEST_ASSERT(status.moving == 1U);
    TEST_ASSERT(status.active_gait == CHASSIS_GAIT_WALK);
    TEST_ASSERT(status.gait_params.turn_step_m > 0.0f);
    TEST_ASSERT(status.wheel_rads[GAIT_LEG_FL] < 0.0f);
    TEST_ASSERT(status.wheel_rads[GAIT_LEG_FR] > 0.0f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_FL_WHEEL].last_vel,
                     status.wheel_rads[GAIT_LEG_FL],
                     1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_FR_WHEEL].last_vel,
                     status.wheel_rads[GAIT_LEG_FR],
                     1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_RL_WHEEL].last_vel,
                     status.wheel_rads[GAIT_LEG_RL],
                     1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_RR_WHEEL].last_vel,
                     status.wheel_rads[GAIT_LEG_RR],
                     1e-6f);
    TEST_ASSERT(total_position_commands() > 0U);
    assert_joint_motor_position_path_active();
    assert_wheel_drive_mit_active();
}

static void test_chassis_travel_uses_trot(void) {
    chassis_control_input_t input;
    chassis_control_status_t status;
    memset(&input, 0, sizeof(input));

    reset_wheel_only_travel_cfg();
    log_init();
    bind_stub_motors();
    chassis_control_init();
    chassis_control_set_mode(CHASSIS_MODE_ONLINE);

    input.command.vx_m_s = 0.25f;
    input.valid_frame_count = 1U;
    input.last_rx_ms = 0U;

    for (uint32_t tick = 0U; tick < 1800U; tick++) {
        chassis_control_tick(&input, 0.002f, tick * 2U);
    }

    chassis_control_get_status(&status);
    TEST_ASSERT(status.online == 1U);
    TEST_ASSERT(status.moving == 1U);
    TEST_ASSERT(status.active_gait == CHASSIS_GAIT_TROT);
    TEST_ASSERT(strcmp(chassis_control_active_gait_name(), "trot") == 0);
    TEST_ASSERT(status.gait_params.period_s >= 0.20f);
    TEST_ASSERT(status.gait_params.period_s <= 0.25f);
    TEST_ASSERT_NEAR(status.gait_params.step_height_m, 0.055f, 1e-6f);
    TEST_ASSERT_NEAR(status.gait_params.duty, 0.60f, 1e-6f);
    TEST_ASSERT_NEAR(status.gait_params.step_length_m, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(status.gait_params.turn_step_m, 0.0f, 1e-6f);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(status.gait_params.leg_step_length_m[i], 0.0f, 1e-6f);
        TEST_ASSERT(status.wheel_rads[i] > 0.0f);
    }
}

static void test_chassis_vx_0p1_drives_all_wheels_continuously(void) {
    static const motor_logical_id_t wheel_ids[GAIT_LEG_NUM] = {
        MOTOR_ID_FL_WHEEL,
        MOTOR_ID_FR_WHEEL,
        MOTOR_ID_RL_WHEEL,
        MOTOR_ID_RR_WHEEL,
    };
    chassis_control_input_t input;
    chassis_control_status_t status;
    memset(&input, 0, sizeof(input));

    reset_wheel_only_travel_cfg();
    log_init();
    bind_stub_motors();
    chassis_control_init();
    chassis_control_set_mode(CHASSIS_MODE_ONLINE);

    input.command.vx_m_s = 0.10f;
    input.valid_frame_count = 1U;
    input.last_rx_ms = 0U;

    for (uint32_t tick = 0U; tick < 2200U; tick++) {
        chassis_control_tick(&input, 0.002f, tick * 2U);
    }

    chassis_control_get_status(&status);
    TEST_ASSERT(status.online == 1U);
    TEST_ASSERT(status.active_gait == CHASSIS_GAIT_TROT);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(status.wheel_rads[i],
                         0.10f / (LEG_DIM_DEFAULT.wheel_diameter * 0.5f),
                         1e-5f);
        TEST_ASSERT_NEAR(s_stub_ctxs[wheel_ids[i]].last_vel,
                         status.wheel_rads[i],
                         1e-5f);
    }
    TEST_ASSERT(g_leg_wheel_mit.drive_mask == 0x0FU);
    TEST_ASSERT(g_leg_wheel_mit.hold_mask == 0x00U);
}

static void test_chassis_control_ramps_stand_height(void) {
    chassis_control_input_t input;
    chassis_control_status_t status;
    memset(&input, 0, sizeof(input));

    log_init();
    bind_stub_motors();
    chassis_control_init();
    chassis_control_set_mode(CHASSIS_MODE_ONLINE);

    input.valid_frame_count = 1U;
    input.last_rx_ms = 0U;

    chassis_control_tick(&input, 0.002f, 0U);
    chassis_control_get_status(&status);
    TEST_ASSERT(status.stand_height_m > 0.12f);
    TEST_ASSERT(status.stand_height_m < GAIT_PARAMS_STAND_DEFAULT.body_height_m);
    TEST_ASSERT(status.moving == 0U);
    TEST_ASSERT(status.active_gait == CHASSIS_GAIT_STAND);

    for (uint32_t tick = 1U; tick < 2200U; tick++) {
        chassis_control_tick(&input, 0.002f, tick * 2U);
    }

    chassis_control_get_status(&status);
    TEST_ASSERT_NEAR(status.stand_height_m,
                     GAIT_PARAMS_STAND_DEFAULT.body_height_m,
                     1e-5f);
}

static void test_chassis_deadband_does_not_roll_wheels(void) {
    chassis_control_input_t input;
    chassis_control_status_t status;
    memset(&input, 0, sizeof(input));

    log_init();
    bind_stub_motors();
    chassis_control_init();
    chassis_control_set_mode(CHASSIS_MODE_ONLINE);

    input.command.vx_m_s = 0.005f;
    input.command.vy_m_s = 0.0f;
    input.command.wz_rad_s = 0.03f;
    input.valid_frame_count = 1U;
    input.last_rx_ms = 0U;

    for (uint32_t tick = 0U; tick < 2200U; tick++) {
        chassis_control_tick(&input, 0.002f, tick * 2U);
    }

    chassis_control_get_status(&status);
    TEST_ASSERT(status.online == 1U);
    TEST_ASSERT(status.moving == 0U);
    TEST_ASSERT(status.active_gait == CHASSIS_GAIT_STAND);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(status.wheel_rads[i], 0.0f, 1e-6f);
    }
    assert_wheel_velocity_zero();
    TEST_ASSERT(g_leg_wheel_mit.drive_mask == 0x00U);
    TEST_ASSERT(g_leg_wheel_mit.hold_mask == 0x0FU);
}

static void test_chassis_wheel_speed_ramps_with_gait_blend(void) {
    chassis_control_input_t input;
    chassis_control_status_t status;
    memset(&input, 0, sizeof(input));

    log_init();
    bind_stub_motors();
    chassis_control_init();
    chassis_control_set_mode(CHASSIS_MODE_ONLINE);

    input.valid_frame_count = 1U;
    input.last_rx_ms = 0U;
    for (uint32_t tick = 0U; tick < 2200U; tick++) {
        chassis_control_tick(&input, 0.002f, tick * 2U);
    }

    input.command.vx_m_s = 0.0f;
    input.command.vy_m_s = 0.0f;
    input.command.wz_rad_s = 1.0f;
    chassis_control_tick(&input, 0.002f, 4400U);

    chassis_control_get_status(&status);
    TEST_ASSERT(status.moving == 0U);
    TEST_ASSERT(status.effective_wz_rad_s > 0.0f);
    TEST_ASSERT(status.effective_wz_rad_s < 0.02f);

    for (uint32_t tick = 1U; tick <= 25U; tick++) {
        chassis_control_tick(&input, 0.002f, 4400U + tick * 2U);
    }

    chassis_control_get_status(&status);
    TEST_ASSERT(status.moving == 1U);
    TEST_ASSERT(status.active_gait == CHASSIS_GAIT_WALK);
    TEST_ASSERT(fabsf(status.wheel_rads[GAIT_LEG_FL]) > 0.1f);

    static const motor_logical_id_t wheel_ids[] = {
        MOTOR_ID_FL_WHEEL,
        MOTOR_ID_FR_WHEEL,
        MOTOR_ID_RL_WHEEL,
        MOTOR_ID_RR_WHEEL,
    };
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        float commanded = fabsf(s_stub_ctxs[wheel_ids[i]].last_vel);
        float planned = fabsf(status.wheel_rads[i]);
        TEST_ASSERT(commanded > 0.0f);
        TEST_ASSERT(commanded < planned * 0.10f);
    }
}

static void test_chassis_motion_gait_switch_keeps_continuous_wheel_drive(void) {
    static const motor_logical_id_t wheel_ids[GAIT_LEG_NUM] = {
        MOTOR_ID_FL_WHEEL,
        MOTOR_ID_FR_WHEEL,
        MOTOR_ID_RL_WHEEL,
        MOTOR_ID_RR_WHEEL,
    };
    chassis_control_input_t input;
    chassis_control_status_t status;
    uint8_t switched_to_walk = 0U;
    memset(&input, 0, sizeof(input));

    reset_wheel_only_travel_cfg();
    log_init();
    bind_stub_motors();
    chassis_control_init();
    chassis_control_set_mode(CHASSIS_MODE_ONLINE);

    input.command.vx_m_s = 0.10f;
    input.valid_frame_count = 1U;
    for (uint32_t tick = 0U; tick < 2200U; tick++) {
        chassis_control_tick(&input, 0.002f, tick * 2U);
    }

    input.command.vx_m_s = 0.0f;
    input.command.wz_rad_s = 1.0f;
    for (uint32_t tick = 1U; tick <= 100U; tick++) {
        chassis_control_tick(&input, 0.002f, 4400U + tick * 2U);
        chassis_control_get_status(&status);
        if (status.active_gait != CHASSIS_GAIT_WALK) continue;

        switched_to_walk = 1U;
        for (int i = 0; i < GAIT_LEG_NUM; i++) {
            TEST_ASSERT(fabsf(status.wheel_rads[i]) > 0.1f);
            TEST_ASSERT_NEAR(s_stub_ctxs[wheel_ids[i]].last_vel,
                             status.wheel_rads[i],
                             1e-5f);
        }
        break;
    }
    TEST_ASSERT(switched_to_walk == 1U);
}

static void test_chassis_translation_rotation_transitions_are_strictly_exclusive(void) {
    chassis_control_input_t input;
    chassis_control_status_t status;
    uint8_t saw_rotation = 0U;
    uint8_t saw_translation_again = 0U;
    memset(&input, 0, sizeof(input));

    reset_wheel_only_travel_cfg();
    log_init();
    bind_stub_motors();
    chassis_control_init();
    chassis_control_set_mode(CHASSIS_MODE_ONLINE);
    input.valid_frame_count = 1U;
    input.command.vx_m_s = 0.25f;

    for (uint32_t tick = 0U; tick < 2200U; tick++) {
        chassis_control_tick(&input, 0.002f, tick * 2U);
    }
    chassis_control_get_status(&status);
    TEST_ASSERT(status.effective_vx_m_s > 0.20f);
    TEST_ASSERT_NEAR(status.effective_wz_rad_s, 0.0f, 1e-6f);

    /* A mixed upstream request is rotate-first, but only after translation is zero. */
    input.command.vx_m_s = 0.25f;
    input.command.wz_rad_s = 1.0f;
    for (uint32_t tick = 1U; tick <= 500U; tick++) {
        chassis_control_tick(&input, 0.002f, 4400U + tick * 2U);
        chassis_control_get_status(&status);
        const uint8_t translating =
            (fabsf(status.effective_vx_m_s) > 1e-6f ||
             fabsf(status.effective_vy_m_s) > 1e-6f) ? 1U : 0U;
        const uint8_t rotating =
            (fabsf(status.effective_wz_rad_s) > 1e-6f) ? 1U : 0U;
        TEST_ASSERT(!(translating && rotating));
        if (rotating) {
            saw_rotation = 1U;
            TEST_ASSERT(!translating);
        }
    }
    TEST_ASSERT(saw_rotation == 1U);

    input.command.wz_rad_s = 0.0f;
    input.command.vx_m_s = -0.20f;
    for (uint32_t tick = 1U; tick <= 900U; tick++) {
        chassis_control_tick(&input, 0.002f, 5400U + tick * 2U);
        chassis_control_get_status(&status);
        const uint8_t translating =
            (fabsf(status.effective_vx_m_s) > 1e-6f ||
             fabsf(status.effective_vy_m_s) > 1e-6f) ? 1U : 0U;
        const uint8_t rotating =
            (fabsf(status.effective_wz_rad_s) > 1e-6f) ? 1U : 0U;
        TEST_ASSERT(!(translating && rotating));
        if (translating) {
            saw_translation_again = 1U;
            TEST_ASSERT(!rotating);
        }
    }
    TEST_ASSERT(saw_translation_again == 1U);
    TEST_ASSERT(status.effective_vx_m_s < -0.15f);
    TEST_ASSERT_NEAR(status.effective_wz_rad_s, 0.0f, 1e-6f);

    chassis_control_set_mode(CHASSIS_MODE_STANDALONE);
    chassis_control_get_status(&status);
    TEST_ASSERT_NEAR(status.effective_vx_m_s, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(status.effective_vy_m_s, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(status.effective_wz_rad_s, 0.0f, 1e-6f);
}

static void test_direct_wheel_test_bypasses_gait_and_times_out_to_hold(void) {
    chassis_control_input_t input;
    float wheel_rads[GAIT_LEG_NUM] = {0.5f, -0.7f, 1.0f, -1.2f};
    memset(&input, 0, sizeof(input));

    log_init();
    bind_stub_motors();
    chassis_control_init();
    chassis_control_set_mode(CHASSIS_MODE_ONLINE);
    input.valid_frame_count = 1U;

    for (uint32_t tick = 0U; tick < 1800U; tick++) {
        chassis_control_tick(&input, 0.002f, tick * 2U);
    }

    TEST_ASSERT(chassis_control_set_wheel_test(1U, 0x05U, wheel_rads, 3600U) == APP_OK);
    chassis_control_tick(&input, 0.002f, 3600U);

    TEST_ASSERT(g_chassis_wheel_test.active == 1U);
    TEST_ASSERT(g_chassis_wheel_test.wheel_mask == 0x05U);
    TEST_ASSERT(g_chassis_wheel_test.command_count == 1U);
    TEST_ASSERT(chassis_control_get_gait_active() == CHASSIS_GAIT_STAND);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_FL_WHEEL].last_vel, 0.5f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_FR_WHEEL].last_vel, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_RL_WHEEL].last_vel, 1.0f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_RR_WHEEL].last_vel, 0.0f, 1e-6f);
    TEST_ASSERT(g_leg_wheel_mit.drive_mask == 0x0FU);
    TEST_ASSERT(wheel_velocity_commands() == 0U);

    chassis_control_tick(&input, 0.002f, 4101U);
    TEST_ASSERT(g_chassis_wheel_test.active == 0U);
    TEST_ASSERT(g_chassis_wheel_test.timeout_count == 1U);
    TEST_ASSERT(g_leg_wheel_mit.drive_mask == 0x00U);
    TEST_ASSERT(g_leg_wheel_mit.hold_mask == 0x0FU);
    assert_wheel_velocity_zero();

    wheel_rads[GAIT_LEG_RR] = 4.1f;
    TEST_ASSERT(chassis_control_set_wheel_test(1U, 0x08U, wheel_rads, 4200U) ==
                APP_ERR_INVALID_ARG);
    TEST_ASSERT(g_chassis_wheel_test.active == 0U);
}

static void test_attitude_comp_applies_balance_torque_ff(void) {
    chassis_control_input_t input;
    imu_bmi088_data_t imu;
    memset(&input, 0, sizeof(input));
    memset(&imu, 0, sizeof(imu));

    log_init();
    bind_stub_motors();
    TEST_ASSERT(imu_bmi088_init() == APP_OK);
    chassis_control_init();
    chassis_control_set_mode(CHASSIS_MODE_ONLINE);

    input.valid_frame_count = 1U;
    input.last_rx_ms = 0U;

    g_chassis_attitude_comp.enable = 1U;
    g_chassis_attitude_comp.stance_only = 0U;
    g_chassis_attitude_comp.scale = 1.0f;
    g_chassis_attitude_comp.half_length_m = 0.25f;
    g_chassis_attitude_comp.half_track_m = 0.15f;
    g_chassis_attitude_comp.max_moment_nm = 3.0f;
    g_chassis_attitude_comp.max_leg_force_n = 20.0f;
    g_chassis_attitude_comp.smooth_tau_s = 0.0f;

    imu.accel[0] = -sinf(0.04f) * BMI088_GRAVITY;
    imu.accel[1] =  sinf(0.06f) * BMI088_GRAVITY;
    imu.accel[2] =  cosf(0.04f) * cosf(0.06f) * BMI088_GRAVITY;
    imu_bmi088_test_set(&imu, 1U);

    for (uint32_t tick = 0U; tick < 2200U; tick++) {
        chassis_control_tick(&input, 0.002f, tick * 2U);
    }

    TEST_ASSERT(fabsf(g_chassis_attitude_comp.roll_rad) > 0.01f);
    TEST_ASSERT(fabsf(g_chassis_attitude_comp.pitch_rad) > 0.01f);
    TEST_ASSERT(fabsf(g_chassis_attitude_comp.filtered_mx_nm) > 1e-4f);
    TEST_ASSERT(fabsf(g_chassis_attitude_comp.filtered_my_nm) > 1e-4f);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(g_chassis_attitude_comp.foot_z_delta_m[i], 0.0f, 1e-7f);
    }
    TEST_ASSERT(g_leg_gravity_comp.compensate_balance == 1U);
    TEST_ASSERT(g_leg_gravity_comp.balance_active_mask == 0x0FU);
    TEST_ASSERT(fabsf(g_leg_gravity_comp.balance_applied_mx_nm) > 1e-4f);
    TEST_ASSERT(fabsf(g_leg_gravity_comp.balance_applied_my_nm) > 1e-4f);
    TEST_ASSERT(fabsf(g_leg_gravity_comp.balance_leg_force_n[GAIT_LEG_FL]) > 1e-4f);
    assert_joint_motor_position_path_active();
    assert_wheel_hold_mit_active();
    assert_any_joint_tau_nonzero();
}

static void test_chassis_arm_load_comp_updates_leg_payload_from_measured_arm(void) {
    chassis_control_input_t input;
    memset(&input, 0, sizeof(input));

    log_init();
    bind_stub_motors();
    TEST_ASSERT(arm_control_init() == APP_OK);
    arm_joint_angles_t arm_angles = arm_test_ik_target(0.45f, 0.08f, 0.20f);
    set_arm_stub_feedback_from_angles(&arm_angles, 10U);
    TEST_ASSERT(arm_control_tick(0.002f, 10U) == APP_OK);

    chassis_control_init();
    chassis_control_set_mode(CHASSIS_MODE_ONLINE);

    g_chassis_arm_load_comp.enable = 1U;
    g_chassis_arm_load_comp.enable_leg_tau_ff = 1U;
    g_chassis_arm_load_comp.prefer_measured = 1U;
    g_chassis_arm_load_comp.include_link_mass = 1U;
    g_chassis_arm_load_comp.scale = 1.0f;
    g_chassis_arm_load_comp.end_to_com_ratio = 0.50f;
    g_chassis_arm_load_comp.smooth_tau_s = 0.0f;
    g_chassis_arm_load_comp.max_leg_payload_kg = 0.0f;
    g_chassis_arm_load_comp.max_tau_nm = 5.0f;

    input.valid_frame_count = 1U;
    input.last_rx_ms = 0U;

    for (uint32_t tick = 0U; tick < 2200U; tick++) {
        chassis_control_tick(&input, 0.002f, tick * 2U);
    }

    TEST_ASSERT(g_chassis_arm_load_comp.source_valid == 1U);
    TEST_ASSERT(g_chassis_arm_load_comp.source_measured == 1U);
    TEST_ASSERT(g_leg_gravity_comp.enable == 1U);
    TEST_ASSERT(g_leg_gravity_comp.compensate_payload == 1U);
    TEST_ASSERT(g_leg_gravity_comp.use_payload_com == 1U);
    TEST_ASSERT(g_leg_gravity_comp.payload_com_x_m > 0.20f);
    TEST_ASSERT(g_leg_gravity_comp.payload_com_y_m > 0.03f);
    TEST_ASSERT(g_leg_gravity_comp.payload_mass_kg > ARM_GRAVITY_MASS_EE_EMPTY);
    TEST_ASSERT_NEAR(g_leg_gravity_comp.payload_mass_kg,
                     g_chassis_arm_load_comp.filtered_total_mass_kg,
                     1e-6f);
    TEST_ASSERT(g_leg_gravity_comp.payload_leg_mass_kg[GAIT_LEG_FL] >
                g_leg_gravity_comp.payload_leg_mass_kg[GAIT_LEG_FR]);
    TEST_ASSERT(g_leg_gravity_comp.payload_leg_mass_kg[GAIT_LEG_FL] >
                g_leg_gravity_comp.payload_leg_mass_kg[GAIT_LEG_RL]);
    assert_any_joint_tau_nonzero();

    g_chassis_arm_load_comp.enable = 0U;
    chassis_control_tick(&input, 0.002f, 4400U);
    TEST_ASSERT(g_chassis_arm_load_comp.source_valid == 0U);
    TEST_ASSERT(g_leg_gravity_comp.compensate_payload == 0U);
    TEST_ASSERT(g_leg_gravity_comp.use_payload_com == 0U);
    TEST_ASSERT_NEAR(g_leg_gravity_comp.payload_mass_kg, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(g_leg_gravity_comp.payload_applied_mass_kg, 0.0f, 1e-6f);
}

static void test_exact_vx_0p1_frame_decodes_without_yaw(void) {
    static const uint8_t frame[] = {
        0x55U, 0xAAU, 0x10U, 0x0CU,
        0xCDU, 0xCCU, 0xCCU, 0x3DU,
        0x00U, 0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U,
        0xBDU,
    };
    task_comm_chassis_cmd_t command;

    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bsp_usb_cdc_test_inject_rx(frame, (uint32_t)sizeof(frame));
    task_comm_get_chassis(&command);

    TEST_ASSERT(task_comm_dispatch_hit() == 1U);
    TEST_ASSERT(command.seq == 1U);
    TEST_ASSERT_NEAR(command.vx, 0.1f, 1e-6f);
    TEST_ASSERT_NEAR(command.vy, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(command.wz, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(command.target_yaw, 0.0f, 1e-6f);
    TEST_ASSERT(command.steer_mode == PROTO_STEER_MODE_OFF);
}

static void test_chassis_command_rejects_ambiguous_payload_length(void) {
    uint8_t frame[64];
    uint8_t payload[PROTO_CHASSIS_CMD_LEGACY_LEN + 1U] = {0U};
    payload_chassis_cmd_t cmd = {
        .vx = 0.2f,
        .vy = 0.0f,
        .wz = 0.0f,
    };
    task_comm_chassis_cmd_t command;
    memcpy(payload, &cmd, sizeof(cmd));

    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    int frame_len = proto_frame_build(PROTO_FUNC_CHASSIS_CMD,
                                      payload,
                                      (uint8_t)sizeof(payload),
                                      frame,
                                      sizeof(frame));
    TEST_ASSERT(frame_len > 0);
    bsp_usb_cdc_test_inject_rx(frame, (uint32_t)frame_len);
    task_comm_get_chassis(&command);

    TEST_ASSERT(task_comm_dispatch_hit() == 1U);
    TEST_ASSERT(command.seq == 0U);
    TEST_ASSERT_NEAR(command.vx, 0.0f, 1e-6f);
    TEST_ASSERT(command.steer_mode == PROTO_STEER_MODE_OFF);
}

static void test_usb_protocol_to_chassis_task_end_to_end(void) {
    uint8_t frame[64];
    payload_chassis_cmd_t cmd = {
        .vx = 0.0f,
        .vy = 0.0f,
        .wz = 1.0f,
    };

    log_init();
    bind_stub_motors();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    task_chassis_init();
    task_chassis_set_mode(CHASSIS_MODE_ONLINE);

    int frame_len = proto_frame_build(PROTO_FUNC_CHASSIS_CMD,
                                      (const uint8_t*)&cmd,
                                      (uint8_t)sizeof(cmd),
                                      frame,
                                      sizeof(frame));
    TEST_ASSERT(frame_len > 0);
    bsp_usb_cdc_test_inject_rx(frame, (uint32_t)frame_len);
    TEST_ASSERT(task_comm_dispatch_hit() > 0U);

    for (uint32_t tick = 0; tick < 1800U; tick++) {
        task_chassis_step_for_test(0.002f, tick * 2U);
    }

    TEST_ASSERT(task_chassis_get_gait_active() == CHASSIS_GAIT_WALK);
    TEST_ASSERT(total_position_commands() > 0U);
    assert_joint_motor_position_path_active();
    assert_wheel_drive_mit_active();
}

static void test_usb_gait_action_can_start_walk(void) {
    uint8_t frame[64];
    gait_params_t walk_params = GAIT_PARAMS_WALK_DEFAULT;
    payload_gait_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));

    cmd.action = PROTO_GAIT_ACTION_WALK;
    cmd.body_height_m = walk_params.body_height_m;
    cmd.step_length_m = walk_params.step_length_m;
    cmd.step_height_m = walk_params.step_height_m;
    cmd.period_s = walk_params.period_s;
    cmd.duty = walk_params.duty;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        cmd.phase_offset[i] = walk_params.phase_offset[i];
    }
    cmd.touchdown_thresh = walk_params.touchdown_thresh;
    cmd.blend_dur_s = 0.0f;

    log_init();
    bind_stub_motors();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    task_chassis_init();

    for (uint32_t tick = 0U; tick < 2200U; tick++) {
        task_chassis_step_for_test(0.002f, tick * 2U);
    }

    int frame_len = proto_frame_build(PROTO_FUNC_GAIT_CMD,
                                      (const uint8_t*)&cmd,
                                      (uint8_t)sizeof(cmd),
                                      frame,
                                      sizeof(frame));
    TEST_ASSERT(frame_len > 0);
    bsp_usb_cdc_test_inject_rx(frame, (uint32_t)frame_len);
    TEST_ASSERT(task_comm_dispatch_hit() > 0U);

    for (uint32_t tick = 2200U; tick < 2220U; tick++) {
        task_chassis_step_for_test(0.002f, tick * 2U);
    }

    TEST_ASSERT(task_chassis_get_gait_active() == CHASSIS_GAIT_WALK);
    TEST_ASSERT(strcmp(task_chassis_active_gait_name(), "walk") == 0);
    assert_joint_motor_position_path_active();
    assert_wheel_drive_mit_active();
}

static void test_protocol_function_ids_are_partitioned(void) {
    const uint8_t competition_uplink_ids[] = {
        PROTO_FUNC_COMMAND_STATUS,
        PROTO_FUNC_SYSTEM_STATUS,
        PROTO_FUNC_ODOMETRY,
        PROTO_FUNC_WHEEL_STATE,
        PROTO_FUNC_CHASSIS_DIAG,
    };

    TEST_ASSERT(PROTO_FUNC_CHASSIS_CMD == 0x10U);
    TEST_ASSERT(PROTO_FUNC_ARM_TARGET == 0x11U);
    TEST_ASSERT(PROTO_FUNC_ARM_CMD == PROTO_FUNC_ARM_TARGET);
    TEST_ASSERT(PROTO_FUNC_GAIT_CMD == 0x12U);
    TEST_ASSERT(PROTO_FUNC_MIT_CMD == 0x13U);
    TEST_ASSERT(PROTO_FUNC_ARM_PUMP == 0x14U);
    TEST_ASSERT(PROTO_FUNC_MODE_CMD == 0x15U);
    TEST_ASSERT(PROTO_FUNC_WHEEL_TEST == 0x16U);
    TEST_ASSERT(PROTO_FUNC_ARM_AUX_GPIO == 0x17U);
    TEST_ASSERT(PROTO_FUNC_ODOM_RESET == 0x18U);
    TEST_ASSERT(PROTO_FUNC_USB_CDC_PING == 0x21U);
    TEST_ASSERT(PROTO_FUNC_ARM_FEEDBACK == 0x86U);
    TEST_ASSERT(PROTO_FUNC_COMMAND_STATUS == 0x88U);
    TEST_ASSERT(PROTO_FUNC_SYSTEM_STATUS == 0x89U);
    TEST_ASSERT(PROTO_FUNC_ODOMETRY == 0x8AU);
    TEST_ASSERT(PROTO_FUNC_WHEEL_STATE == 0x8BU);
    TEST_ASSERT(PROTO_FUNC_CHASSIS_DIAG == 0x8CU);
    TEST_ASSERT(PROTO_ROBOT_MODE_REAR_PLACE == 5U);
    TEST_ASSERT(PROTO_ROBOT_MODE_MAX == PROTO_ROBOT_MODE_REAR_PLACE);

    for (size_t i = 0U;
         i < sizeof(competition_uplink_ids) / sizeof(competition_uplink_ids[0]);
         i++) {
        for (size_t j = i + 1U;
             j < sizeof(competition_uplink_ids) / sizeof(competition_uplink_ids[0]);
             j++) {
            TEST_ASSERT(competition_uplink_ids[i] != competition_uplink_ids[j]);
        }
    }

    TEST_ASSERT(sizeof(payload_arm_target_t) == 13U);
    TEST_ASSERT(sizeof(payload_arm_pump_t) == 1U);
    TEST_ASSERT(sizeof(payload_mode_cmd_t) == 1U);
    TEST_ASSERT(sizeof(payload_wheel_test_t) == 20U);
    TEST_ASSERT(sizeof(payload_arm_aux_gpio_t) == 2U);
    TEST_ASSERT(sizeof(payload_arm_feedback_t) == 17U);
    TEST_ASSERT(sizeof(payload_wheel_state_t) == 21U);
    TEST_ASSERT(sizeof(payload_chassis_diag_t) == 48U);
    TEST_ASSERT(sizeof(payload_odom_reset_t) == 12U);
    TEST_ASSERT(sizeof(payload_odometry_t) == 32U);
}

static void test_usb_tx_priority_classification(void) {
    uint8_t frame[] = {
        PROTO_HEAD1,
        PROTO_HEAD2,
        PROTO_FUNC_COMMAND_STATUS,
        0U,
        0U,
    };

    TEST_ASSERT(bsp_usb_cdc_test_priority_class(frame, sizeof(frame)) == 2U);
    frame[2] = PROTO_FUNC_SYSTEM_STATUS;
    TEST_ASSERT(bsp_usb_cdc_test_priority_class(frame, sizeof(frame)) == 1U);
    frame[2] = PROTO_FUNC_ODOMETRY;
    TEST_ASSERT(bsp_usb_cdc_test_priority_class(frame, sizeof(frame)) == 0U);
    frame[2] = PROTO_FUNC_WHEEL_STATE;
    TEST_ASSERT(bsp_usb_cdc_test_priority_class(frame, sizeof(frame)) == 0U);
    frame[2] = PROTO_FUNC_CHASSIS_DIAG;
    TEST_ASSERT(bsp_usb_cdc_test_priority_class(frame, sizeof(frame)) == 0U);
}

static chassis_odometry_input_t odometry_test_input(void) {
    chassis_odometry_input_t input;
    memset(&input, 0, sizeof(input));
    input.wheel_radius_m = 0.05f;
    input.half_track_m = 0.15f;
    input.wheel_measurement_scale = 1.0f;
    input.imu_ready = 1U;
    input.wheel_online_mask = 0x0FU;
    input.stance_mask = 0x0FU;
    input.allow_velocity_correction = 1U;
    input.allow_yaw_correction = 1U;
    return input;
}

static void test_chassis_odometry_fuses_wheel_speed_and_yaw_rate(void) {
    chassis_odometry_input_t input = odometry_test_input();
    chassis_odometry_init();
    for (int i = 0; i < GAIT_LEG_NUM; i++) input.wheel_velocity_rads[i] = 2.0f;

    for (int i = 0; i < 200; i++) chassis_odometry_update(&input, 0.01f);
    const chassis_odometry_state_t* state = chassis_odometry_get_state();
    TEST_ASSERT(state->quality_flags & CHASSIS_ODOM_FLAG_IMU_READY);
    TEST_ASSERT(state->quality_flags & CHASSIS_ODOM_FLAG_WHEEL_VALID);
    TEST_ASSERT(state->quality_flags & CHASSIS_ODOM_FLAG_WHEEL_CORRECTED);
    TEST_ASSERT_NEAR(state->vx_m_s, 0.10f, 0.01f);
    TEST_ASSERT(state->x_m > 0.15f);

    input.wheel_velocity_rads[GAIT_LEG_FL] = -2.0f;
    input.wheel_velocity_rads[GAIT_LEG_RL] = -2.0f;
    input.wheel_velocity_rads[GAIT_LEG_FR] = 2.0f;
    input.wheel_velocity_rads[GAIT_LEG_RR] = 2.0f;
    for (int i = 0; i < 100; i++) chassis_odometry_update(&input, 0.01f);
    TEST_ASSERT(fabsf(chassis_odometry_get_state()->gyro_z_bias_rad_s) > 0.02f);
}

static void test_chassis_odometry_rejects_transition_wheel_measurement(void) {
    chassis_odometry_input_t input = odometry_test_input();
    chassis_odometry_init();
    input.transition_gated = 1U;
    for (int i = 0; i < GAIT_LEG_NUM; i++) input.wheel_velocity_rads[i] = 4.0f;

    for (int i = 0; i < 100; i++) chassis_odometry_update(&input, 0.01f);
    const chassis_odometry_state_t* state = chassis_odometry_get_state();
    TEST_ASSERT(state->quality_flags & CHASSIS_ODOM_FLAG_TRANSITION_GATED);
    TEST_ASSERT(!(state->quality_flags & CHASSIS_ODOM_FLAG_WHEEL_CORRECTED));
    TEST_ASSERT_NEAR(state->vx_m_s, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(state->x_m, 0.0f, 1e-6f);
}

static void test_arm_kinematics_inverse_forward_roundtrip(void) {
    arm_pose_t target = {
        .x_m = 0.21f,
        .y_m = -0.11f,
        .z_m = 0.34f,
        .pitch_rad = 0.0f,
    };
    arm_joint_angles_t angles;
    arm_pose_t solved;

    TEST_ASSERT(arm_kinematics_inverse(&ARM_KINEMATICS_DEFAULT_PARAMS,
                                       &ARM_KINEMATICS_DEFAULT_OFFSET,
                                       &target,
                                       &angles) == 0);
    TEST_ASSERT(isfinite(angles.theta1_motor_rad));
    TEST_ASSERT(isfinite(angles.theta2_motor_rad));
    TEST_ASSERT(isfinite(angles.theta3_motor_rad));
    TEST_ASSERT(isfinite(angles.theta4_motor_rad));

    arm_kinematics_forward(&ARM_KINEMATICS_DEFAULT_PARAMS, &angles, &solved);
    TEST_ASSERT_NEAR(solved.x_m, target.x_m, 1e-4f);
    TEST_ASSERT_NEAR(solved.y_m, target.y_m, 1e-4f);
    TEST_ASSERT_NEAR(solved.z_m, target.z_m, 1e-4f);
}

static void test_arm_legacy_kinematics_and_gravity_wrappers(void) {
    Arm_Pose_t target = {
        .x = 210.0f,
        .y = -110.0f,
        .z = 340.0f,
        .pitch = 0.0f,
    };
    Arm_Joint_Angles_t angles;
    Arm_Pose_t roundtrip;
    float tau2 = 0.0f;
    float tau3 = 0.0f;
    float tau4 = 0.0f;

    TEST_ASSERT(Arm_Inverse_Kinematics(&arm_params, &arm_offset, &target, &angles) == 0);
    Arm_Forward_Kinematics(&arm_params, &angles, &roundtrip);
    TEST_ASSERT_NEAR(roundtrip.x, target.x, 1e-3f);
    TEST_ASSERT_NEAR(roundtrip.y, target.y, 1e-3f);
    TEST_ASSERT_NEAR(roundtrip.z, target.z, 1e-3f);

    Gravity_Comp_Init();
    Gravity_Comp_SetPayloadState(GRAVITY_PAYLOAD_LOADED);
    Calculate_Gravity_Compensation(angles.theta2_geo,
                                   angles.theta3_geo,
                                   angles.theta4_geo,
                                   &tau2,
                                   &tau3,
                                   &tau4);
    TEST_ASSERT(isfinite(tau2));
    TEST_ASSERT(isfinite(tau3));
    TEST_ASSERT(isfinite(tau4));
    /* 旧公开重补 API 返回原始模型力矩，不在 API 内叠加控制层比例。 */
    float native_tau2 = 0.0f;
    float native_tau3 = 0.0f;
    float native_tau4 = 0.0f;
    arm_gravity_comp_init();
    arm_gravity_comp_set_payload_state(ARM_GRAVITY_PAYLOAD_LOADED);
    arm_gravity_comp_calculate(angles.theta2_geo,
                               angles.theta3_geo,
                               angles.theta4_geo,
                               &native_tau2,
                               &native_tau3,
                               &native_tau4);
    TEST_ASSERT_NEAR(tau2, native_tau2, 1e-6f);
    TEST_ASSERT_NEAR(tau3, native_tau3, 1e-6f);
    TEST_ASSERT_NEAR(tau4, native_tau4, 1e-6f);
}

static void test_arm_motion_quintic_finishes_at_target(void) {
    arm_motion_sample_t start;
    arm_motion_sample_t sample;
    float target[ARM_MOTION_JOINT_COUNT] = {
        0.15f, -0.10f, 0.20f, -0.25f,
    };
    memset(&start, 0, sizeof(start));

    arm_motion_init(NULL);
    TEST_ASSERT(arm_motion_start(&start, target, 100U) == 0);
    TEST_ASSERT(arm_motion_get_state() == ARM_TRAJECTORY_MOVING);
    TEST_ASSERT(arm_motion_get_duration() >= 0.30f);

    TEST_ASSERT(arm_motion_update(100U, &sample) == ARM_TRAJECTORY_MOVING);
    TEST_ASSERT(isfinite(sample.position[0]));
    TEST_ASSERT(isfinite(sample.velocity[0]));

    uint32_t done_ms = 100U + (uint32_t)(arm_motion_get_duration() * 1000.0f) + 20U;
    TEST_ASSERT(arm_motion_update(done_ms, &sample) == ARM_TRAJECTORY_FINISHED);
    for (uint32_t i = 0U; i < ARM_MOTION_JOINT_COUNT; i++) {
        TEST_ASSERT_NEAR(sample.position[i], target[i], 1e-5f);
        TEST_ASSERT_NEAR(sample.velocity[i], 0.0f, 1e-6f);
    }
}

static void reset_arm_gravity_defaults(void) {
    g_arm_gc_mass_l2_kg = ARM_GRAVITY_MASS_L2;
    g_arm_gc_mass_l3_kg = ARM_GRAVITY_MASS_L3;
    g_arm_gc_mass_ee_kg = ARM_GRAVITY_MASS_EE_EMPTY;
    g_arm_gc_mass_cargo_kg = ARM_GRAVITY_MASS_CARGO_BOX;
    g_arm_gc_com_r2_x_m = ARM_GRAVITY_COM_R2_X_M;
    g_arm_gc_com_r2_y_m = ARM_GRAVITY_COM_R2_Y_M;
    g_arm_gc_com_r3_x_m = ARM_GRAVITY_COM_R3_X_M;
    g_arm_gc_com_r3_y_m = ARM_GRAVITY_COM_R3_Y_M;
    g_arm_gc_com_ee_x_m = ARM_GRAVITY_COM_EE_X_M;
}

static void test_arm_legacy_motion_wrappers_share_quintic_engine(void) {
    Arm_Motion_Sample_t start;
    float target[ARM_MOTION_JOINT_COUNT] = { 0.10f, -0.20f, 0.25f, -0.15f };
    memset(&start, 0, sizeof(start));

    Arm_Motion_Init(NULL);
    TEST_ASSERT(Arm_Motion_Start(&start, target, 0U) == 0);
    Arm_Motion_Sample_t sample;
    TEST_ASSERT(Arm_Motion_Update(20000U, &sample) == ARM_TRAJECTORY_FINISHED);
    for (uint32_t i = 0U; i < ARM_MOTION_JOINT_COUNT; i++) {
        TEST_ASSERT_NEAR(sample.position[i], target[i], 1e-6f);
    }
}

static void test_arm_gravity_comp_empty_pose_outputs_expected_signs(void) {
    float tau2 = 0.0f;
    float tau3 = 0.0f;
    float tau4 = 0.0f;

    reset_arm_gravity_defaults();
    arm_gravity_comp_init();
    arm_gravity_comp_calculate(0.0f, 0.0f, 0.0f, &tau2, &tau3, &tau4);

    TEST_ASSERT(tau2 > 5.0f);
    TEST_ASSERT(tau2 < 6.0f);
    TEST_ASSERT(tau3 > 2.0f);
    TEST_ASSERT(tau3 < 2.4f);
    TEST_ASSERT(tau4 < -0.1f);
    TEST_ASSERT(tau4 > -0.4f);
    TEST_ASSERT_NEAR(arm_gravity_comp_get_active_end_mass(),
                     ARM_GRAVITY_MASS_EE_EMPTY,
                     1e-6f);
}

static void test_arm_gravity_comp_payload_mass_transitions_smoothly(void) {
    float tau2 = 0.0f;
    float tau3 = 0.0f;
    float tau4 = 0.0f;

    reset_arm_gravity_defaults();
    arm_gravity_comp_init();
    arm_gravity_comp_set_payload_state(ARM_GRAVITY_PAYLOAD_LOADED);
    TEST_ASSERT_NEAR(arm_gravity_comp_get_target_end_mass(),
                     ARM_GRAVITY_MASS_EE_EMPTY + ARM_GRAVITY_MASS_CARGO_BOX,
                     1e-6f);

    arm_gravity_comp_calculate(0.0f, 0.0f, 0.0f, &tau2, &tau3, &tau4);
    TEST_ASSERT_NEAR(arm_gravity_comp_get_active_end_mass(),
                     ARM_GRAVITY_MASS_EE_EMPTY +
                     ARM_GRAVITY_MASS_TRANSITION_STEP_KG,
                     1e-6f);

    for (uint32_t i = 0U; i < 400U; i++) {
        arm_gravity_comp_calculate(0.0f, 0.0f, 0.0f, &tau2, &tau3, &tau4);
    }
    TEST_ASSERT_NEAR(arm_gravity_comp_get_active_end_mass(),
                     ARM_GRAVITY_MASS_EE_EMPTY + ARM_GRAVITY_MASS_CARGO_BOX,
                     1e-5f);
}

static void test_arm_vision_transform_preserves_legacy_units(void) {
    Arm_Joint_Angles_t angles;
    Arm_Pose_t end_pose;
    angles.theta1_geo = 0.0f;
    end_pose.x = 200.0f;
    end_pose.y = 10.0f;
    end_pose.z = 300.0f;
    vision_camera_data.x = 0.020f;
    vision_camera_data.y = 0.030f;
    vision_camera_data.z = 0.040f;
    vision_camera_data.pitch = 0.0f;

    uint32_t before = debug_vision_rx_count;
    Perform_Vision_Coordinate_Transform(&angles, &end_pose);

    TEST_ASSERT_NEAR(vision_world_data.x, 335.0f, 1e-4f);
    TEST_ASSERT_NEAR(vision_world_data.y, 30.0f, 1e-4f);
    TEST_ASSERT_NEAR(vision_world_data.z, 182.0f, 1e-4f);
    TEST_ASSERT(debug_vision_rx_count == before + 1U);
}

static void test_arm_pump_device_defaults_off_and_sets_main_gpio(void) {
    bsp_gpio_test_reset();
    TEST_ASSERT(arm_pump_init() == APP_OK);
    TEST_ASSERT(arm_pump_is_enabled() == 0U);
    TEST_ASSERT(bsp_gpio_read_latch(BSP_GPIO_ARM_PUMP_MAIN) == 0U);

    TEST_ASSERT(arm_pump_set(1U) == APP_OK);
    TEST_ASSERT(arm_pump_is_enabled() == 1U);
    TEST_ASSERT(bsp_gpio_read_latch(BSP_GPIO_ARM_PUMP_MAIN) == 1U);

    TEST_ASSERT(arm_pump_set(0U) == APP_OK);
    TEST_ASSERT(arm_pump_is_enabled() == 0U);
    TEST_ASSERT(bsp_gpio_read_latch(BSP_GPIO_ARM_PUMP_MAIN) == 0U);
}

static void test_pump_control_legacy_wrappers_update_payload_state(void) {
    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    arm_gravity_comp_init();
    Pump_Control_Init();
    TEST_ASSERT(Pump_Control_IsEnabled() == 0U);
    TEST_ASSERT(debug_pd11_on == 0U);

    Pump_Control_Set(1U);
    TEST_ASSERT(Pump_Control_IsEnabled() == 1U);
    TEST_ASSERT(debug_payload_loaded == 1U);
    TEST_ASSERT(arm_gravity_comp_get_payload_state() == ARM_GRAVITY_PAYLOAD_LOADED);

    Pump_Control_SetPC8(1U);
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC8) == 1U);
    Pump_Control_Set(0U);
    TEST_ASSERT(Pump_Control_IsEnabled() == 0U);
}

static void test_arm_control_pump_updates_device_and_payload_mass(void) {
    arm_control_status_t status;

    bsp_gpio_test_reset();
    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_pump(1U) == APP_OK);
    arm_control_get_status(&status);
    TEST_ASSERT(status.pump_on == 1U);
    TEST_ASSERT(arm_pump_is_enabled() == 1U);
    TEST_ASSERT(bsp_gpio_read_latch(BSP_GPIO_ARM_PUMP_MAIN) == 1U);
    TEST_ASSERT_NEAR(arm_gravity_comp_get_target_end_mass(),
                     ARM_GRAVITY_MASS_EE_EMPTY + ARM_GRAVITY_MASS_CARGO_BOX,
                     1e-6f);

    TEST_ASSERT(arm_control_set_enabled(0U) == APP_OK);
    arm_control_get_status(&status);
    TEST_ASSERT(status.pump_on == 1U);
    TEST_ASSERT(arm_pump_is_enabled() == 1U);
    TEST_ASSERT(bsp_gpio_read_latch(BSP_GPIO_ARM_PUMP_MAIN) == 1U);
    TEST_ASSERT_NEAR(arm_gravity_comp_get_target_end_mass(),
                     ARM_GRAVITY_MASS_EE_EMPTY + ARM_GRAVITY_MASS_CARGO_BOX,
                     1e-6f);

    TEST_ASSERT(arm_control_set_pump(0U) == APP_OK);
    TEST_ASSERT(arm_pump_is_enabled() == 0U);
    TEST_ASSERT(bsp_gpio_read_latch(BSP_GPIO_ARM_PUMP_MAIN) == 0U);
}

static void test_arm_control_plans_target_without_reporting_reached(void) {
    payload_arm_feedback_t feedback;
    arm_control_status_t status;
    const float x_m = 0.21f;
    const float y_m = -0.11f;
    const float z_m = 0.34f;

    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_target(PROTO_ARM_TARGET_GRASP,
                                       x_m,
                                       y_m,
                                       z_m) == APP_OK);

    for (uint32_t now = 0U; now <= 20000U; now += 20U) {
        TEST_ASSERT(arm_control_tick(0.020f, now) == APP_OK);
    }

    arm_control_get_status(&status);
    arm_control_get_feedback(&feedback);
    TEST_ASSERT(status.target_valid == 1U);
    TEST_ASSERT(status.trajectory_state == ARM_TRAJECTORY_FINISHED);
    TEST_ASSERT_NEAR(status.motion_progress, 1.0f, 1e-6f);
    TEST_ASSERT(status.safe_move_stage == 1U);
    TEST_ASSERT(status.reached == 0U);
    TEST_ASSERT(fabsf(status.planned_end_x_m - x_m) > 0.01f);
    TEST_ASSERT(fabsf(status.planned_end_z_m - z_m) > 0.01f);
    TEST_ASSERT(feedback.arm_state == PROTO_ARM_STATE_MOVING);
    TEST_ASSERT(isfinite(status.gravity_tau2_nm));
    TEST_ASSERT(isfinite(status.gravity_tau3_nm));
    TEST_ASSERT(isfinite(status.gravity_tau4_nm));
}

static void test_arm_control_output_gate_defaults_to_no_motor_commands(void) {
    arm_control_status_t status;

    bind_stub_motors();
    seed_arm_stub_feedback(0U);
    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_target(PROTO_ARM_TARGET_GRASP,
                                       0.21f,
                                       -0.11f,
                                       0.34f) == APP_OK);
    TEST_ASSERT(arm_control_tick(0.020f, 0U) == APP_OK);

    arm_control_get_status(&status);
    TEST_ASSERT(status.motor_output_enabled == 0U);
    TEST_ASSERT(status.motor_feedback_fresh == 1U);
    TEST_ASSERT(status.feedback_source_measured == 1U);
    TEST_ASSERT(arm_position_commands() == 0U);
}

static void test_arm_control_reports_measured_idle_feedback_when_available(void) {
    payload_arm_feedback_t feedback;
    arm_control_status_t status;

    bind_stub_motors();
    seed_arm_stub_feedback(0U);
    s_stub_devs[MOTOR_ID_ARM_J1].state.angle_rad = 0.20f;

    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_tick(0.020f, 0U) == APP_OK);

    arm_control_get_status(&status);
    arm_control_get_feedback(&feedback);
    TEST_ASSERT(status.motor_feedback_fresh == 1U);
    TEST_ASSERT(status.feedback_source_measured == 1U);
    TEST_ASSERT(feedback.arm_state == PROTO_ARM_STATE_IDLE);
    TEST_ASSERT_NEAR(feedback.theta1_rad, 0.20f, 1e-6f);
    TEST_ASSERT(feedback.end_z_m > 0.30f);
    TEST_ASSERT(feedback.end_x_m > 0.20f);
    TEST_ASSERT(arm_position_commands() == 0U);
}

static void test_arm_control_idle_gravity_holds_current_pose_when_enabled(void) {
    payload_arm_feedback_t feedback;
    arm_control_status_t status;
    arm_joint_angles_t measured = arm_test_home_angles();

    bind_stub_motors();
    set_arm_stub_feedback_from_angles(&measured, 0U);
    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_motor_output_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_tick(0.020f, 0U) == APP_OK);

    arm_control_get_status(&status);
    arm_control_get_feedback(&feedback);
    TEST_ASSERT(status.enabled == 1U);
    TEST_ASSERT(status.target_valid == 0U);
    TEST_ASSERT(status.motor_output_enabled == 1U);
    TEST_ASSERT(status.motor_feedback_fresh == 1U);
    TEST_ASSERT(feedback.arm_state == PROTO_ARM_STATE_IDLE);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_ARM_J1].set_position_count == 1U);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_ARM_J2].set_position_count == 1U);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_ARM_J3].set_position_count == 1U);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_ARM_J4].set_position_count == 1U);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J1].last_pos,
                     measured.theta1_motor_rad,
                     1e-5f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J2].last_pos,
                     ARM_KINEMATICS_MOTOR2_TO_PHYSICAL(measured.theta2_motor_rad),
                     1e-5f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J3].last_pos,
                     ARM_KINEMATICS_MOTOR3_TO_PHYSICAL(measured.theta3_motor_rad),
                     1e-5f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J4].last_pos,
                     measured.theta4_motor_rad,
                     1e-5f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J1].last_kp, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J1].last_kd, 0.05f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J4].last_kp, 30.0f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J4].last_kd, 0.8f, 1e-6f);
    TEST_ASSERT(isfinite(s_stub_ctxs[MOTOR_ID_ARM_J2].last_tau));
    TEST_ASSERT(isfinite(s_stub_ctxs[MOTOR_ID_ARM_J3].last_tau));
    TEST_ASSERT(isfinite(s_stub_ctxs[MOTOR_ID_ARM_J4].last_tau));
}

static void test_arm_control_idle_hold_waits_for_fresh_feedback(void) {
    arm_control_status_t status;

    bind_stub_motors();
    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_motor_output_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_tick(0.020f, 0U) == APP_OK);

    arm_control_get_status(&status);
    TEST_ASSERT(status.motor_output_enabled == 1U);
    TEST_ASSERT(status.motor_feedback_fresh == 0U);
    TEST_ASSERT(status.target_valid == 0U);
    TEST_ASSERT(arm_position_commands() == 0U);
}

static void test_arm_control_output_gate_sends_damiao_commands_when_feedback_fresh(void) {
    arm_control_status_t status;
    arm_joint_angles_t measured = arm_test_home_angles();
    float expected_tau2 = 0.0f;
    float expected_tau3 = 0.0f;
    float expected_tau4 = 0.0f;

    bind_stub_motors();
    set_arm_stub_feedback_from_angles(&measured, 0U);
    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_motor_output_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_target(PROTO_ARM_TARGET_GRASP,
                                       0.21f,
                                       -0.11f,
                                       0.34f) == APP_OK);
    TEST_ASSERT(arm_control_tick(0.020f, 0U) == APP_OK);

    arm_control_get_status(&status);
    TEST_ASSERT(status.motor_output_enabled == 1U);
    TEST_ASSERT(status.motor_bound_mask == 0x0FU);
    TEST_ASSERT(status.motor_feedback_fresh == 1U);
    TEST_ASSERT(status.motor_tx_fail_mask == 0U);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_ARM_J1].set_position_count > 0U);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_ARM_J2].set_position_count > 0U);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_ARM_J3].set_position_count > 0U);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_ARM_J4].set_position_count > 0U);
    TEST_ASSERT(isfinite(s_stub_ctxs[MOTOR_ID_ARM_J2].last_tau));
    TEST_ASSERT(isfinite(s_stub_ctxs[MOTOR_ID_ARM_J3].last_tau));
    TEST_ASSERT(isfinite(s_stub_ctxs[MOTOR_ID_ARM_J4].last_tau));

    arm_gravity_comp_calculate(measured.theta2_geo_rad,
                               measured.theta3_geo_rad,
                               measured.theta4_geo_rad,
                               &expected_tau2,
                               &expected_tau3,
                               &expected_tau4);
    expected_tau2 *= dbg_gravity_scale_all * dbg_gravity_scale_2;
    expected_tau3 *= dbg_gravity_scale_all * dbg_gravity_scale_3;
    expected_tau4 *= dbg_gravity_scale_all * dbg_gravity_scale_4;
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J2].last_tau,
                     ARM_KINEMATICS_MOTOR2_TO_PHYSICAL(expected_tau2),
                     1e-5f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J3].last_tau,
                     ARM_KINEMATICS_MOTOR3_TO_PHYSICAL(expected_tau3),
                     1e-5f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J4].last_tau,
                     expected_tau4,
                     1e-5f);
}

static void test_arm_control_motion_feedback_expires_at_250ms_and_keeps_pump(void) {
    arm_control_status_t status;

    bind_stub_motors();
    seed_arm_stub_feedback(0U);
    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_motor_output_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_pump(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_target(PROTO_ARM_TARGET_GRASP,
                                       0.21f,
                                       -0.11f,
                                       0.34f) == APP_OK);
    TEST_ASSERT(arm_control_tick(0.020f, 251U) == APP_ERR_OFFLINE);

    arm_control_get_status(&status);
    TEST_ASSERT(status.motor_feedback_fresh == 0U);
    TEST_ASSERT(status.target_valid == 0U);
    TEST_ASSERT(status.pump_on == 1U);
    TEST_ASSERT(arm_pump_is_enabled() == 1U);
}

static void test_arm_control_payload_transition_updates_once_per_motion_tick(void) {
    bind_stub_motors();
    seed_arm_stub_feedback(0U);
    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_pump(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_target(PROTO_ARM_TARGET_GRASP,
                                       0.21f,
                                       -0.11f,
                                       0.34f) == APP_OK);
    TEST_ASSERT(arm_control_tick(0.020f, 0U) == APP_OK);
    TEST_ASSERT_NEAR(arm_gravity_comp_get_active_end_mass(),
                     ARM_GRAVITY_MASS_EE_EMPTY +
                     ARM_GRAVITY_MASS_TRANSITION_STEP_KG,
                     1e-6f);
}

static void test_arm_control_output_gate_blocks_without_fresh_feedback(void) {
    payload_arm_feedback_t feedback;
    arm_control_status_t status;

    bind_stub_motors();
    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_motor_output_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_target(PROTO_ARM_TARGET_GRASP,
                                       0.21f,
                                       -0.11f,
                                       0.34f) == APP_OK);
    TEST_ASSERT(arm_control_tick(0.020f, 1000U) == APP_ERR_OFFLINE);

    arm_control_get_status(&status);
    arm_control_get_feedback(&feedback);
    TEST_ASSERT(status.motor_output_enabled == 1U);
    TEST_ASSERT(status.motor_feedback_fresh == 0U);
    TEST_ASSERT(status.target_valid == 0U);
    TEST_ASSERT(status.last_result == APP_ERR_OFFLINE);
    TEST_ASSERT(feedback.arm_state == PROTO_ARM_STATE_ERROR);
    TEST_ASSERT(arm_position_commands() == 0U);
}

static uint32_t arm_test_complete_stage_at(const arm_joint_angles_t* reached,
                                           uint32_t now_ms) {
    arm_control_status_t status;
    arm_control_get_status(&status);
    uint32_t done_ms = now_ms +
        (uint32_t)(status.motion_duration_s * 1000.0f) + 40U;

    set_arm_stub_feedback_from_angles(reached, done_ms);
    TEST_ASSERT(arm_control_tick(0.020f, done_ms) == APP_OK);
    set_arm_stub_feedback_from_angles(reached, done_ms + 220U);
    TEST_ASSERT(arm_control_tick(0.020f, done_ms + 220U) == APP_OK);
    return done_ms + 220U;
}

static uint32_t arm_test_begin_safe_move_from_home(float x_m,
                                                   float y_m,
                                                   float z_m,
                                                   arm_joint_angles_t* home,
                                                   arm_joint_angles_t* target) {
    arm_control_status_t status;
    uint32_t now = 0U;

    TEST_ASSERT(home != NULL);
    TEST_ASSERT(target != NULL);
    *home = arm_test_home_angles();
    *target = arm_test_ik_target(x_m, y_m, z_m);

    bind_stub_motors();
    set_arm_stub_feedback_from_angles(home, now);
    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_target(PROTO_ARM_TARGET_GRASP,
                                       x_m,
                                       y_m,
                                       z_m) == APP_OK);
    TEST_ASSERT(arm_control_tick(0.020f, now) == APP_OK);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 1U);
    TEST_ASSERT(status.fine_tracking_active == 0U);
    TEST_ASSERT(status.reached == 0U);
    return now;
}

static void test_arm_control_reached_requires_measured_settle(void) {
    payload_arm_feedback_t feedback;
    arm_control_status_t status;
    const float x_m = 0.21f;
    const float y_m = -0.11f;
    const float z_m = 0.34f;
    arm_joint_angles_t target = arm_test_ik_target(x_m, y_m, z_m);

    bind_stub_motors();
    set_arm_stub_feedback_from_angles(&target, 0U);
    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_target(PROTO_ARM_TARGET_GRASP,
                                       x_m,
                                       y_m,
                                       z_m) == APP_OK);
    TEST_ASSERT(arm_control_tick(0.020f, 0U) == APP_OK);

    set_arm_stub_feedback_from_angles(&target, 220U);
    TEST_ASSERT(arm_control_tick(0.020f, 220U) == APP_OK);
    arm_control_get_status(&status);
    arm_control_get_feedback(&feedback);
    TEST_ASSERT(status.safe_move_stage == 0U);
    TEST_ASSERT(status.reached == 1U);
    TEST_ASSERT(status.settle_error_rad <= APP_ARM_DEG2RAD(1.0f));
    TEST_ASSERT(feedback.arm_state == PROTO_ARM_STATE_REACHED);
    TEST_ASSERT_NEAR(feedback.end_x_m, x_m, 1e-4f);
    TEST_ASSERT_NEAR(feedback.end_y_m, y_m, 1e-4f);
    TEST_ASSERT_NEAR(feedback.end_z_m, z_m, 1e-4f);
}

static void test_arm_control_internal_hold_survives_settle_timeout(void) {
    arm_control_status_t status;
    payload_arm_feedback_t feedback;
    arm_joint_angles_t measured = arm_test_home_angles();
    arm_joint_angles_t fixed = arm_test_park_angles();

    bind_stub_motors();
    set_arm_stub_feedback_from_angles(&measured, 0U);
    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_joint_target(ARM_CONTROL_TARGET_INTERNAL_HOLD,
                                             &fixed) == APP_OK);
    TEST_ASSERT(arm_control_tick(0.020f, 0U) == APP_OK);

    /* Keep feedback fresh but deliberately never reach the fixed pose. */
    set_arm_stub_feedback_from_angles(&measured, 5000U);
    TEST_ASSERT(arm_control_tick(0.020f, 5000U) == APP_OK);
    arm_control_get_status(&status);
    arm_control_get_feedback(&feedback);

    TEST_ASSERT(status.target_valid == 1U);
    TEST_ASSERT(status.target_type == ARM_CONTROL_TARGET_INTERNAL_HOLD);
    TEST_ASSERT(status.last_result == APP_OK);
    TEST_ASSERT(status.reached == 0U);
    TEST_ASSERT(feedback.arm_state == PROTO_ARM_STATE_MOVING);
}

static void test_arm_control_safe_move_retract_rotate_extend_sequence(void) {
    payload_arm_feedback_t feedback;
    arm_control_status_t status;
    const float x_m = 0.21f;
    const float y_m = -0.11f;
    const float z_m = 0.34f;
    arm_joint_angles_t home;
    memset(&home, 0, sizeof(home));
    home.theta4_motor_rad = arm_kinematics_compute_t4_from_t3(
        &ARM_KINEMATICS_DEFAULT_OFFSET,
        home.theta3_motor_rad);
    home.theta4_geo_rad =
        home.theta4_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta4_offset_rad;
    arm_joint_angles_t final_target = arm_test_ik_target(x_m, y_m, z_m);
    arm_joint_angles_t retract = arm_test_safe_angles(home.theta1_motor_rad);
    arm_joint_angles_t rotate = arm_test_safe_angles(final_target.theta1_motor_rad);
    uint32_t now = 0U;

    bind_stub_motors();
    set_arm_stub_feedback_from_angles(&home, now);
    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_set_target(PROTO_ARM_TARGET_GRASP,
                                       x_m,
                                       y_m,
                                       z_m) == APP_OK);
    TEST_ASSERT(arm_control_tick(0.020f, now) == APP_OK);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 1U);
    TEST_ASSERT(status.reached == 0U);

    now = arm_test_complete_stage_at(&retract, now);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 2U);
    TEST_ASSERT(status.target_pending == 1U);

    now += 20U;
    set_arm_stub_feedback_from_angles(&retract, now);
    TEST_ASSERT(arm_control_tick(0.020f, now) == APP_OK);
    now = arm_test_complete_stage_at(&rotate, now);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 3U);
    TEST_ASSERT(status.target_pending == 1U);

    now += 20U;
    set_arm_stub_feedback_from_angles(&rotate, now);
    TEST_ASSERT(arm_control_tick(0.020f, now) == APP_OK);
    now = arm_test_complete_stage_at(&final_target, now);
    arm_control_get_status(&status);
    arm_control_get_feedback(&feedback);
    TEST_ASSERT(status.safe_move_stage == 0U);
    TEST_ASSERT(status.reached == 1U);
    TEST_ASSERT(status.safe_move_cycle_count == 1U);
    TEST_ASSERT(feedback.arm_state == PROTO_ARM_STATE_REACHED);
    TEST_ASSERT_NEAR(feedback.end_x_m, x_m, 1e-4f);
    TEST_ASSERT_NEAR(feedback.end_y_m, y_m, 1e-4f);
    TEST_ASSERT_NEAR(feedback.end_z_m, z_m, 1e-4f);
}

static void test_arm_control_retract_stage_uses_latest_target_for_rotate(void) {
    arm_control_status_t status;
    arm_joint_angles_t home;
    arm_joint_angles_t target_a;
    const arm_joint_angles_t retract = arm_test_safe_angles(0.0f);
    const arm_joint_angles_t target_b = arm_test_ik_target(0.21f, 0.11f, 0.34f);
    const arm_joint_angles_t rotate_b = arm_test_safe_angles(target_b.theta1_motor_rad);
    uint32_t now = arm_test_begin_safe_move_from_home(0.21f,
                                                      -0.11f,
                                                      0.34f,
                                                      &home,
                                                      &target_a);
    (void)home;
    (void)target_a;

    TEST_ASSERT(arm_control_set_target(PROTO_ARM_TARGET_GRASP,
                                       0.21f,
                                       0.11f,
                                       0.34f) == APP_OK);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 1U);
    TEST_ASSERT(status.target_pending == 0U);

    now = arm_test_complete_stage_at(&retract, now);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 2U);

    now += 20U;
    set_arm_stub_feedback_from_angles(&retract, now);
    TEST_ASSERT(arm_control_tick(0.020f, now) == APP_OK);
    now = arm_test_complete_stage_at(&rotate_b, now);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 3U);
}

static void test_arm_control_rotate_stage_replans_base_to_latest_target(void) {
    arm_control_status_t status;
    arm_joint_angles_t home;
    arm_joint_angles_t target_a;
    const arm_joint_angles_t retract = arm_test_safe_angles(0.0f);
    const arm_joint_angles_t target_b = arm_test_ik_target(0.21f, 0.11f, 0.34f);
    const arm_joint_angles_t rotate_b = arm_test_safe_angles(target_b.theta1_motor_rad);
    uint32_t now = arm_test_begin_safe_move_from_home(0.21f,
                                                      -0.11f,
                                                      0.34f,
                                                      &home,
                                                      &target_a);
    (void)home;
    (void)target_a;

    now = arm_test_complete_stage_at(&retract, now);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 2U);

    now += 20U;
    set_arm_stub_feedback_from_angles(&retract, now);
    TEST_ASSERT(arm_control_tick(0.020f, now) == APP_OK);
    TEST_ASSERT(arm_control_set_target(PROTO_ARM_TARGET_GRASP,
                                       0.21f,
                                       0.11f,
                                       0.34f) == APP_OK);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 2U);
    TEST_ASSERT(status.target_pending == 1U);

    now += 20U;
    set_arm_stub_feedback_from_angles(&retract, now);
    TEST_ASSERT(arm_control_tick(0.020f, now) == APP_OK);
    now = arm_test_complete_stage_at(&rotate_b, now);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 3U);
}

static void test_arm_control_extend_stage_large_target_change_retracts(void) {
    arm_control_status_t status;
    arm_joint_angles_t home;
    arm_joint_angles_t target_a;
    const arm_joint_angles_t retract = arm_test_safe_angles(0.0f);
    uint32_t now = arm_test_begin_safe_move_from_home(0.21f,
                                                      -0.11f,
                                                      0.34f,
                                                      &home,
                                                      &target_a);
    const arm_joint_angles_t rotate = arm_test_safe_angles(target_a.theta1_motor_rad);
    (void)home;

    now = arm_test_complete_stage_at(&retract, now);
    now += 20U;
    set_arm_stub_feedback_from_angles(&retract, now);
    TEST_ASSERT(arm_control_tick(0.020f, now) == APP_OK);
    now = arm_test_complete_stage_at(&rotate, now);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 3U);

    now += 20U;
    set_arm_stub_feedback_from_angles(&rotate, now);
    TEST_ASSERT(arm_control_tick(0.020f, now) == APP_OK);
    TEST_ASSERT(arm_control_set_target(PROTO_ARM_TARGET_GRASP,
                                       0.097f,
                                       0.040f,
                                       0.335f) == APP_OK);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 1U);
    TEST_ASSERT(status.fine_tracking_active == 0U);
    TEST_ASSERT(status.target_pending == 1U);
}

static void test_arm_control_fine_tracking_replans_small_updates(void) {
    payload_arm_feedback_t feedback;
    arm_control_status_t status;
    arm_joint_angles_t home;
    arm_joint_angles_t target_a;
    const arm_joint_angles_t retract = arm_test_safe_angles(0.0f);
    uint32_t now = arm_test_begin_safe_move_from_home(0.21f,
                                                      -0.11f,
                                                      0.34f,
                                                      &home,
                                                      &target_a);
    const arm_joint_angles_t rotate = arm_test_safe_angles(target_a.theta1_motor_rad);
    const arm_joint_angles_t target_b = arm_test_ik_target(0.20f, -0.10f, 0.34f);
    (void)home;

    now = arm_test_complete_stage_at(&retract, now);
    now += 20U;
    set_arm_stub_feedback_from_angles(&retract, now);
    TEST_ASSERT(arm_control_tick(0.020f, now) == APP_OK);
    now = arm_test_complete_stage_at(&rotate, now);
    now += 20U;
    set_arm_stub_feedback_from_angles(&rotate, now);
    TEST_ASSERT(arm_control_tick(0.020f, now) == APP_OK);
    now = arm_test_complete_stage_at(&target_a, now);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 0U);
    TEST_ASSERT(status.fine_tracking_active == 1U);
    TEST_ASSERT(status.reached == 1U);

    TEST_ASSERT(arm_control_set_target(PROTO_ARM_TARGET_GRASP,
                                       0.21f,
                                       -0.11f,
                                       0.34f) == APP_OK);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 0U);
    TEST_ASSERT(status.target_pending == 0U);
    TEST_ASSERT(status.reached == 1U);

    TEST_ASSERT(arm_control_set_target(PROTO_ARM_TARGET_GRASP,
                                       0.20f,
                                       -0.10f,
                                       0.34f) == APP_OK);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 0U);
    TEST_ASSERT(status.fine_tracking_active == 1U);
    TEST_ASSERT(status.target_pending == 1U);
    TEST_ASSERT(status.reached == 0U);

    now += 20U;
    set_arm_stub_feedback_from_angles(&target_a, now);
    TEST_ASSERT(arm_control_tick(0.020f, now) == APP_OK);
    arm_control_get_status(&status);
    TEST_ASSERT(status.safe_move_stage == 0U);
    TEST_ASSERT(status.target_pending == 0U);

    now = arm_test_complete_stage_at(&target_b, now);
    arm_control_get_status(&status);
    arm_control_get_feedback(&feedback);
    TEST_ASSERT(status.safe_move_stage == 0U);
    TEST_ASSERT(status.fine_tracking_active == 1U);
    TEST_ASSERT(status.reached == 1U);
    TEST_ASSERT(feedback.arm_state == PROTO_ARM_STATE_REACHED);
    TEST_ASSERT_NEAR(feedback.end_x_m, 0.20f, 1e-4f);
    TEST_ASSERT_NEAR(feedback.end_y_m, -0.10f, 1e-4f);
    TEST_ASSERT_NEAR(feedback.end_z_m, 0.34f, 1e-4f);
}

static void test_usb_arm_and_mode_protocol_cache(void) {
    uint8_t frame[64];
    payload_arm_target_t target = {
        .target_type = PROTO_ARM_TARGET_GRASP,
        .x_m = 0.10f,
        .y_m = -0.20f,
        .z_m = 0.30f,
    };
    payload_arm_pump_t pump = {
        .pump_on = 1U,
    };
    payload_mode_cmd_t mode = {
        .mode = PROTO_ROBOT_MODE_ARM,
    };
    task_comm_arm_target_t cached_target;
    task_comm_arm_pump_t cached_pump;
    task_comm_mode_cmd_t cached_mode;

    log_init();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();

    int frame_len = proto_frame_build(PROTO_FUNC_MODE_CMD,
                                      (const uint8_t*)&mode,
                                      (uint8_t)sizeof(mode),
                                      frame,
                                      sizeof(frame));
    TEST_ASSERT(frame_len > 0);
    bsp_usb_cdc_test_inject_rx(frame, (uint32_t)frame_len);

    frame_len = proto_frame_build(PROTO_FUNC_ARM_TARGET,
                                      (const uint8_t*)&target,
                                      (uint8_t)sizeof(target),
                                      frame,
                                      sizeof(frame));
    TEST_ASSERT(frame_len > 0);
    bsp_usb_cdc_test_inject_rx(frame, (uint32_t)frame_len);

    task_comm_get_arm_target(&cached_target);
    TEST_ASSERT(cached_target.seq == 1U);
    TEST_ASSERT(cached_target.target_type == PROTO_ARM_TARGET_GRASP);
    TEST_ASSERT_NEAR(cached_target.x_m, 0.10f, 1e-6f);
    TEST_ASSERT_NEAR(cached_target.y_m, -0.20f, 1e-6f);
    TEST_ASSERT_NEAR(cached_target.z_m, 0.30f, 1e-6f);
    TEST_ASSERT(debug_comm_arm_target_accept_count == 1U);
    TEST_ASSERT(debug_comm_arm_target_reject_count == 0U);
    TEST_ASSERT(debug_comm_arm_target_type == PROTO_ARM_TARGET_GRASP);
    TEST_ASSERT_NEAR(debug_comm_arm_target_x_m, target.x_m, 1e-6f);
    TEST_ASSERT_NEAR(debug_comm_arm_target_y_m, target.y_m, 1e-6f);
    TEST_ASSERT_NEAR(debug_comm_arm_target_z_m, target.z_m, 1e-6f);

    payload_arm_target_t invalid_target = {
        .target_type = PROTO_ARM_TARGET_GRASP,
        .x_m = 3.26503964e18f,
        .y_m = -4.3284671e-11f,
        .z_m = -872.464722f,
    };
    frame_len = proto_frame_build(PROTO_FUNC_ARM_TARGET,
                                  (const uint8_t*)&invalid_target,
                                  (uint8_t)sizeof(invalid_target),
                                  frame,
                                  sizeof(frame));
    TEST_ASSERT(frame_len > 0);
    bsp_usb_cdc_test_inject_rx(frame, (uint32_t)frame_len);
    task_comm_get_arm_target(&cached_target);
    TEST_ASSERT(cached_target.seq == 1U);
    TEST_ASSERT_NEAR(cached_target.x_m, target.x_m, 1e-6f);
    TEST_ASSERT(debug_comm_arm_target_accept_count == 1U);
    TEST_ASSERT(debug_comm_arm_target_reject_count == 1U);
    TEST_ASSERT_NEAR(debug_comm_arm_target_x_m, invalid_target.x_m, 1.0e12f);

    frame_len = proto_frame_build(PROTO_FUNC_ARM_PUMP,
                                  (const uint8_t*)&pump,
                                  (uint8_t)sizeof(pump),
                                  frame,
                                  sizeof(frame));
    TEST_ASSERT(frame_len > 0);
    bsp_usb_cdc_test_inject_rx(frame, (uint32_t)frame_len);

    task_comm_get_arm_pump(&cached_pump);
    TEST_ASSERT(cached_pump.seq == 1U);
    TEST_ASSERT(cached_pump.pump_on == 1U);

    task_comm_get_mode_cmd(&cached_mode);
    TEST_ASSERT(cached_mode.seq == 1U);
    TEST_ASSERT(cached_mode.mode == PROTO_ROBOT_MODE_ARM);

    mode.mode = PROTO_ROBOT_MODE_REAR_PLACE;
    frame_len = proto_frame_build(PROTO_FUNC_MODE_CMD,
                                  (const uint8_t*)&mode,
                                  (uint8_t)sizeof(mode),
                                  frame,
                                  sizeof(frame));
    TEST_ASSERT(frame_len > 0);
    bsp_usb_cdc_test_inject_rx(frame, (uint32_t)frame_len);

    task_comm_get_mode_cmd(&cached_mode);
    TEST_ASSERT(cached_mode.seq == 2U);
    TEST_ASSERT(cached_mode.mode == PROTO_ROBOT_MODE_REAR_PLACE);
    TEST_ASSERT(task_comm_dispatch_hit() >= 3U);
}

static void test_usb_arm_feedback_frame_tx(void) {
    payload_arm_feedback_t feedback = {
        .arm_state = PROTO_ARM_STATE_MOVING,
        .end_x_m = 0.10f,
        .end_y_m = -0.20f,
        .end_z_m = 0.30f,
        .theta1_rad = 1.57f,
    };
    uint8_t tx[64];

    log_init();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bsp_usb_cdc_test_reset();

    int frame_len = task_comm_send_arm_feedback(&feedback);
    TEST_ASSERT(frame_len == 5 + (int)sizeof(feedback));
    TEST_ASSERT(bsp_usb_cdc_test_tx_size() == (uint32_t)frame_len);
    TEST_ASSERT(bsp_usb_cdc_test_read_tx(tx, sizeof(tx)) == (uint32_t)frame_len);
    TEST_ASSERT(tx[0] == PROTO_HEAD1);
    TEST_ASSERT(tx[1] == PROTO_HEAD2);
    TEST_ASSERT(tx[2] == PROTO_FUNC_ARM_FEEDBACK);
    TEST_ASSERT(tx[3] == sizeof(feedback));
    TEST_ASSERT(tx[4] == PROTO_ARM_STATE_MOVING);
    uint8_t checksum = 0U;
    for (int i = 0; i < frame_len - 1; i++) {
        checksum = (uint8_t)(checksum + tx[i]);
    }
    TEST_ASSERT(tx[frame_len - 1] == checksum);
}

static void test_usb_wheel_feedback_frame_tx(void) {
    uint8_t tx[64];
    payload_wheel_state_t payload;

    log_init();
    bind_stub_motors();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bsp_usb_cdc_test_reset();

    s_stub_devs[MOTOR_ID_FL_WHEEL].state.velocity_rads = 1.25f;
    s_stub_devs[MOTOR_ID_FR_WHEEL].state.velocity_rads = 2.50f;
    s_stub_devs[MOTOR_ID_RL_WHEEL].state.velocity_rads = -3.75f;
    s_stub_devs[MOTOR_ID_RR_WHEEL].state.velocity_rads = -5.00f;
    s_stub_devs[MOTOR_ID_FR_WHEEL].state.online = 0U;

    int frame_len = task_comm_send_wheel_feedback();
    TEST_ASSERT(frame_len == 5 + (int)sizeof(payload));
    TEST_ASSERT(bsp_usb_cdc_test_read_tx(tx, sizeof(tx)) == (uint32_t)frame_len);
    TEST_ASSERT(tx[0] == PROTO_HEAD1);
    TEST_ASSERT(tx[1] == PROTO_HEAD2);
    TEST_ASSERT(tx[2] == PROTO_FUNC_WHEEL_STATE);
    TEST_ASSERT(tx[3] == sizeof(payload));

    memcpy(&payload, &tx[4], sizeof(payload));
    TEST_ASSERT_NEAR(payload.fl_velocity_rads, 1.25f, 1e-6f);
    TEST_ASSERT_NEAR(payload.fr_velocity_rads, 2.50f, 1e-6f);
    TEST_ASSERT_NEAR(payload.rl_velocity_rads, -3.75f, 1e-6f);
    TEST_ASSERT_NEAR(payload.rr_velocity_rads, -5.00f, 1e-6f);
    TEST_ASSERT(payload.online_mask == 0x0DU);

    uint8_t checksum = 0U;
    for (int i = 0; i < frame_len - 1; i++) {
        checksum = (uint8_t)(checksum + tx[i]);
    }
    TEST_ASSERT(tx[frame_len - 1] == checksum);
}

static void test_usb_chassis_diag_frame_tx(void) {
    uint8_t tx[64];
    payload_chassis_diag_t payload;

    log_init();
    bind_stub_motors();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bsp_usb_cdc_test_reset();

    int frame_len = task_comm_send_chassis_diag();
    TEST_ASSERT(frame_len == 5 + (int)sizeof(payload));
    TEST_ASSERT(bsp_usb_cdc_test_read_tx(tx, sizeof(tx)) == (uint32_t)frame_len);
    TEST_ASSERT(tx[2] == PROTO_FUNC_CHASSIS_DIAG);
    TEST_ASSERT(tx[3] == sizeof(payload));
    memcpy(&payload, &tx[4], sizeof(payload));
    TEST_ASSERT(payload.speed_scale_permille <= 1000U);

    uint8_t checksum = 0U;
    for (int i = 0; i < frame_len - 1; i++) checksum = (uint8_t)(checksum + tx[i]);
    TEST_ASSERT(tx[frame_len - 1] == checksum);
}

static void test_usb_odometry_frame_tx(void) {
    uint8_t tx[64];
    payload_odometry_t payload;

    log_init();
    chassis_odometry_init();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bsp_usb_cdc_test_reset();

    int frame_len = task_comm_send_odometry();
    TEST_ASSERT(frame_len == 5 + (int)sizeof(payload));
    TEST_ASSERT(bsp_usb_cdc_test_read_tx(tx, sizeof(tx)) == (uint32_t)frame_len);
    TEST_ASSERT(tx[0] == PROTO_HEAD1);
    TEST_ASSERT(tx[1] == PROTO_HEAD2);
    TEST_ASSERT(tx[2] == PROTO_FUNC_ODOMETRY);
    TEST_ASSERT(tx[3] == sizeof(payload));
    memcpy(&payload, &tx[4], sizeof(payload));
    TEST_ASSERT(payload.quality_flags & CHASSIS_ODOM_FLAG_INITIALIZED);
    TEST_ASSERT_NEAR(payload.x_m, 0.0f, 1e-6f);
}

static void test_arm_serial_protocol_legacy_parser_caches_target_and_pump(void) {
    uint8_t target_payload[ARM_PROTOCOL_TARGET_PAYLOAD_LEN];
    uint8_t pump_payload[ARM_PROTOCOL_PUMP_PAYLOAD_LEN] = {1U};
    uint8_t frame[32];

    log_init();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    Arm_Serial_Protocol_Init();

    target_payload[0] = 0U;
    float target_x = 0.12f;
    float target_y = -0.05f;
    float target_z = 0.31f;
    memcpy(&target_payload[1], &target_x, sizeof(target_x));
    memcpy(&target_payload[5], &target_y, sizeof(target_y));
    memcpy(&target_payload[9], &target_z, sizeof(target_z));

    int frame_len = proto_frame_build(ARM_PROTOCOL_FUNC_TARGET,
                                      target_payload,
                                      ARM_PROTOCOL_TARGET_PAYLOAD_LEN,
                                      frame,
                                      sizeof(frame));
    TEST_ASSERT(frame_len > 0);
    Arm_Serial_Protocol_Receive(frame, (uint32_t)frame_len);

    frame_len = proto_frame_build(ARM_PROTOCOL_FUNC_PUMP,
                                  pump_payload,
                                  ARM_PROTOCOL_PUMP_PAYLOAD_LEN,
                                  frame,
                                  sizeof(frame));
    TEST_ASSERT(frame_len > 0);
    Arm_Serial_Protocol_Receive(frame, (uint32_t)frame_len);

    TEST_ASSERT(debug_serial_target_rx_count == 1U);
    TEST_ASSERT(debug_serial_pump_rx_count == 1U);
    TEST_ASSERT(debug_serial_target_type == 0U);
    TEST_ASSERT_NEAR(debug_serial_target_x_m, target_x, 1e-6f);
    TEST_ASSERT_NEAR(debug_serial_target_y_m, target_y, 1e-6f);
    TEST_ASSERT_NEAR(debug_serial_target_z_m, target_z, 1e-6f);
    TEST_ASSERT(debug_serial_pump_on == 1U);
}

static void inject_proto_frame(uint8_t func_id, const void* payload, uint8_t len) {
    uint8_t frame[64];
    int frame_len = proto_frame_build(func_id,
                                      (const uint8_t*)payload,
                                      len,
                                      frame,
                                      sizeof(frame));
    TEST_ASSERT(frame_len > 0);
    bsp_usb_cdc_test_inject_rx(frame, (uint32_t)frame_len);
}

static uint8_t find_last_tx_payload(uint8_t wanted_func,
                                    void* payload_out,
                                    uint8_t expected_len) {
    uint8_t stream[512];
    uint32_t size = bsp_usb_cdc_test_read_tx(stream, sizeof(stream));
    uint8_t found = 0U;
    uint32_t offset = 0U;

    while (offset + 5U <= size) {
        if (stream[offset] != PROTO_HEAD1 || stream[offset + 1U] != PROTO_HEAD2) {
            offset++;
            continue;
        }
        const uint8_t payload_len = stream[offset + 3U];
        const uint32_t frame_len = (uint32_t)payload_len + 5U;
        if (offset + frame_len > size) break;

        uint8_t checksum = 0U;
        for (uint32_t i = 0U; i + 1U < frame_len; i++) {
            checksum = (uint8_t)(checksum + stream[offset + i]);
        }
        if (checksum == stream[offset + frame_len - 1U] &&
            stream[offset + 2U] == wanted_func &&
            payload_len == expected_len) {
            if (payload_out && expected_len > 0U) {
                memcpy(payload_out, &stream[offset + 4U], expected_len);
            }
            found = 1U;
        }
        offset += frame_len;
    }
    return found;
}

static void test_v2_payload_sizes_and_command_status_fields(void) {
    payload_mode_cmd_v2_t mode = {
        .mode = PROTO_ROBOT_MODE_ARM,
        .command_seq = 0x10203040U,
    };
    payload_command_status_t status;
    task_comm_mode_cmd_t cached;

    TEST_ASSERT(sizeof(payload_arm_target_v2_t) == 17U);
    TEST_ASSERT(sizeof(payload_arm_pump_v2_t) == 5U);
    TEST_ASSERT(sizeof(payload_mode_cmd_v2_t) == 5U);
    TEST_ASSERT(sizeof(payload_command_status_t) == 12U);
    TEST_ASSERT(sizeof(payload_system_status_t) == 12U);

    task_safety_estop_set(false);
    bsp_time_init();
    bind_stub_motors();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    task_chassis_init();
    bsp_usb_cdc_test_reset();

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &mode, (uint8_t)sizeof(mode));
    task_comm_get_mode_cmd(&cached);
    TEST_ASSERT(cached.seq == 1U);
    TEST_ASSERT(cached.sequenced == 1U);
    TEST_ASSERT(cached.command_seq == mode.command_seq);
    TEST_ASSERT(cached.mode == PROTO_ROBOT_MODE_ARM);
    TEST_ASSERT(find_last_tx_payload(PROTO_FUNC_COMMAND_STATUS,
                                     &status,
                                     (uint8_t)sizeof(status)) == 1U);
    TEST_ASSERT(status.boot_id == task_comm_boot_id());
    TEST_ASSERT(status.command_seq == mode.command_seq);
    TEST_ASSERT(status.command_func == PROTO_FUNC_MODE_CMD);
    TEST_ASSERT(status.stage == PROTO_COMMAND_STAGE_COMPLETED);
    TEST_ASSERT(status.result == PROTO_COMMAND_RESULT_OK);
    TEST_ASSERT(status.actual_mode == PROTO_ROBOT_MODE_ARM);
}

static void test_v2_target_duplicate_and_stale_sequences_never_reexecute(void) {
    payload_mode_cmd_v2_t mode = {
        .mode = PROTO_ROBOT_MODE_ARM,
        .command_seq = 10U,
    };
    payload_arm_target_v2_t first = {
        .target = {
            .target_type = PROTO_ARM_TARGET_GRASP,
            .x_m = 0.21f,
            .y_m = 0.10f,
            .z_m = 0.34f,
        },
        .command_seq = 100U,
    };
    payload_arm_target_v2_t second = first;
    task_comm_arm_target_t cached;
    payload_command_status_t status;

    second.target.x_m = 0.24f;
    second.command_seq = 101U;

    task_safety_estop_set(false);
    bsp_time_init();
    bind_stub_motors();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    task_chassis_init();
    inject_proto_frame(PROTO_FUNC_MODE_CMD, &mode, (uint8_t)sizeof(mode));
    bsp_usb_cdc_test_reset();

    inject_proto_frame(PROTO_FUNC_ARM_TARGET, &first, (uint8_t)sizeof(first));
    task_comm_get_arm_target(&cached);
    TEST_ASSERT(cached.seq == 1U);
    TEST_ASSERT(cached.command_seq == first.command_seq);

    first.target.y_m = -0.10f;
    inject_proto_frame(PROTO_FUNC_ARM_TARGET, &first, (uint8_t)sizeof(first));
    task_comm_get_arm_target(&cached);
    TEST_ASSERT(cached.seq == 1U);
    TEST_ASSERT_NEAR(cached.y_m, 0.10f, 1e-6f);

    inject_proto_frame(PROTO_FUNC_ARM_TARGET, &second, (uint8_t)sizeof(second));
    task_comm_get_arm_target(&cached);
    TEST_ASSERT(cached.seq == 2U);
    TEST_ASSERT(cached.command_seq == second.command_seq);
    TEST_ASSERT_NEAR(cached.x_m, second.target.x_m, 1e-6f);

    bsp_usb_cdc_test_reset();
    inject_proto_frame(PROTO_FUNC_ARM_TARGET, &first, (uint8_t)sizeof(first));
    task_comm_get_arm_target(&cached);
    TEST_ASSERT(cached.seq == 2U);
    TEST_ASSERT(cached.command_seq == second.command_seq);
    TEST_ASSERT(debug_comm_arm_target_accept_count == 2U);
    TEST_ASSERT(find_last_tx_payload(PROTO_FUNC_COMMAND_STATUS,
                                     &status,
                                     (uint8_t)sizeof(status)) == 1U);
    TEST_ASSERT(status.command_seq == first.command_seq);
    TEST_ASSERT(status.stage == PROTO_COMMAND_STAGE_REJECTED);
    TEST_ASSERT(status.result == PROTO_COMMAND_RESULT_STALE_SEQUENCE);
}

static void test_v2_mode_heartbeat_feeds_watchdog_and_idle_alone_recovers(void) {
    payload_mode_cmd_v2_t nav = {
        .mode = PROTO_ROBOT_MODE_NAV,
        .command_seq = 1000U,
    };
    payload_mode_cmd_v2_t rejected_nav = nav;
    payload_mode_cmd_v2_t idle = nav;
    task_comm_mode_cmd_t cached;

    rejected_nav.command_seq = 1001U;
    idle.mode = PROTO_ROBOT_MODE_IDLE;
    idle.command_seq = 1002U;

    task_safety_estop_set(false);
    bsp_time_init();
    bind_stub_motors();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    task_chassis_init();

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &nav, (uint8_t)sizeof(nav));
    task_comm_get_mode_cmd(&cached);
    TEST_ASSERT(cached.seq == 1U);

    bsp_time_test_advance_ms(400U);
    inject_proto_frame(PROTO_FUNC_MODE_CMD, &nav, (uint8_t)sizeof(nav));
    task_comm_get_mode_cmd(&cached);
    TEST_ASSERT(cached.seq == 1U);
    bsp_time_test_advance_ms(400U);
    task_comm_watchdog_step((uint32_t)bsp_time_now_ms());
    TEST_ASSERT(task_comm_watchdog_active() == 0U);

    bsp_time_test_advance_ms(200U);
    task_comm_watchdog_step((uint32_t)bsp_time_now_ms());
    TEST_ASSERT(task_comm_watchdog_active() == 1U);
    TEST_ASSERT(task_safety_estop_active());
    TEST_ASSERT(task_comm_actual_mode() == PROTO_ROBOT_MODE_ERROR);

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &nav, (uint8_t)sizeof(nav));
    TEST_ASSERT(task_comm_watchdog_active() == 1U);
    inject_proto_frame(PROTO_FUNC_MODE_CMD,
                       &rejected_nav,
                       (uint8_t)sizeof(rejected_nav));
    TEST_ASSERT(task_comm_watchdog_active() == 1U);
    TEST_ASSERT(task_comm_actual_mode() == PROTO_ROBOT_MODE_ERROR);

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &idle, (uint8_t)sizeof(idle));
    TEST_ASSERT(task_comm_watchdog_active() == 0U);
    TEST_ASSERT(!task_safety_estop_active());
    TEST_ASSERT(task_comm_actual_mode() == PROTO_ROBOT_MODE_IDLE);
}

static void test_v2_pump_off_completes_in_watchdog_error_before_idle_reset(void) {
    payload_mode_cmd_v2_t arm = {
        .mode = PROTO_ROBOT_MODE_ARM,
        .command_seq = 500U,
    };
    payload_arm_pump_v2_t pump_off = {
        .pump_on = 0U,
        .command_seq = 700U,
    };
    payload_mode_cmd_v2_t session_idle = {
        .mode = PROTO_ROBOT_MODE_IDLE,
        .command_seq = 1U,
    };
    task_comm_arm_pump_t cached_pump;
    task_comm_mode_cmd_t cached_mode;
    payload_command_status_t status;
    arm_joint_angles_t park = arm_test_park_angles();

    task_safety_estop_set(false);
    bsp_time_init();
    log_init();
    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bind_stub_motors();
    task_chassis_init();
    set_arm_stub_feedback_from_angles(&park, 0U);
    task_arm_init();
    Pump_Control_SetPA8(1U);

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &arm, (uint8_t)sizeof(arm));
    bsp_time_test_advance_ms(APP_COMM_WATCHDOG_TIMEOUT_MS + 1U);
    task_comm_watchdog_step((uint32_t)bsp_time_now_ms());
    TEST_ASSERT(task_comm_watchdog_active() == 1U);
    TEST_ASSERT(task_safety_estop_active());
    TEST_ASSERT(task_comm_actual_mode() == PROTO_ROBOT_MODE_ERROR);

    /* Prove ingress pump-off is effective even while the arm task is gated. */
    Pump_Control_Set(1U);
    bsp_usb_cdc_test_reset();
    inject_proto_frame(PROTO_FUNC_ARM_PUMP,
                       &pump_off,
                       (uint8_t)sizeof(pump_off));
    task_comm_get_arm_pump(&cached_pump);
    TEST_ASSERT(cached_pump.seq == 1U);
    TEST_ASSERT(cached_pump.command_seq == pump_off.command_seq);
    TEST_ASSERT(cached_pump.pump_on == 0U);
    TEST_ASSERT(arm_pump_is_enabled() == 0U);
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA8) == 1U);
    TEST_ASSERT(find_last_tx_payload(PROTO_FUNC_COMMAND_STATUS,
                                     &status,
                                     (uint8_t)sizeof(status)) == 1U);
    TEST_ASSERT(status.command_seq == pump_off.command_seq);
    TEST_ASSERT(status.command_func == PROTO_FUNC_ARM_PUMP);
    TEST_ASSERT(status.stage == PROTO_COMMAND_STAGE_COMPLETED);
    TEST_ASSERT(status.result == PROTO_COMMAND_RESULT_OK);
    TEST_ASSERT(status.actual_mode == PROTO_ROBOT_MODE_ERROR);

    /* ESTOP consumption must not replace the ingress terminal status. */
    task_arm_step_for_test(0.010f, (uint32_t)bsp_time_now_ms());
    TEST_ASSERT(find_last_tx_payload(PROTO_FUNC_COMMAND_STATUS,
                                     &status,
                                     (uint8_t)sizeof(status)) == 1U);
    TEST_ASSERT(status.command_seq == pump_off.command_seq);
    TEST_ASSERT(status.stage == PROTO_COMMAND_STAGE_COMPLETED);
    TEST_ASSERT(status.result == PROTO_COMMAND_RESULT_OK);

    /* Duplicate retries recover COMPLETED without another task-side lifecycle. */
    bsp_usb_cdc_test_reset();
    inject_proto_frame(PROTO_FUNC_ARM_PUMP,
                       &pump_off,
                       (uint8_t)sizeof(pump_off));
    task_comm_get_arm_pump(&cached_pump);
    TEST_ASSERT(cached_pump.seq == 1U);
    TEST_ASSERT(find_last_tx_payload(PROTO_FUNC_COMMAND_STATUS,
                                     &status,
                                     (uint8_t)sizeof(status)) == 1U);
    TEST_ASSERT(status.command_seq == pump_off.command_seq);
    TEST_ASSERT(status.stage == PROTO_COMMAND_STAGE_COMPLETED);

    /* A restarted host can now complete its stale-IDLE session reset. */
    bsp_usb_cdc_test_reset();
    inject_proto_frame(PROTO_FUNC_MODE_CMD,
                       &session_idle,
                       (uint8_t)sizeof(session_idle));
    task_comm_get_mode_cmd(&cached_mode);
    TEST_ASSERT(cached_mode.command_seq == session_idle.command_seq);
    TEST_ASSERT(cached_mode.mode == PROTO_ROBOT_MODE_IDLE);
    TEST_ASSERT(task_comm_watchdog_active() == 0U);
    TEST_ASSERT(!task_safety_estop_active());
    TEST_ASSERT(task_comm_actual_mode() == PROTO_ROBOT_MODE_IDLE);
    TEST_ASSERT(arm_pump_is_enabled() == 0U);
    TEST_ASSERT(find_last_tx_payload(PROTO_FUNC_COMMAND_STATUS,
                                     &status,
                                     (uint8_t)sizeof(status)) == 1U);
    TEST_ASSERT(status.command_seq == session_idle.command_seq);
    TEST_ASSERT(status.command_func == PROTO_FUNC_MODE_CMD);
    TEST_ASSERT(status.stage == PROTO_COMMAND_STAGE_COMPLETED);
    TEST_ASSERT(status.result == PROTO_COMMAND_RESULT_OK);
}

static void test_v2_stale_idle_resets_host_session_but_stale_non_idle_rejects(void) {
    payload_mode_cmd_v2_t arm = {
        .mode = PROTO_ROBOT_MODE_ARM,
        .command_seq = 500U,
    };
    payload_mode_cmd_v2_t stale_nav = {
        .mode = PROTO_ROBOT_MODE_NAV,
        .command_seq = 1U,
    };
    payload_mode_cmd_v2_t session_idle = {
        .mode = PROTO_ROBOT_MODE_IDLE,
        .command_seq = 1U,
    };
    payload_arm_target_v2_t target = {
        .target = {
            .target_type = PROTO_ARM_TARGET_GRASP,
            .x_m = 0.21f,
            .y_m = 0.10f,
            .z_m = 0.34f,
        },
        .command_seq = 700U,
    };
    payload_arm_pump_v2_t pump = {
        .pump_on = 1U,
        .command_seq = 800U,
    };
    task_comm_mode_cmd_t cached_mode;
    task_comm_arm_target_t cached_target;
    task_comm_arm_pump_t cached_pump;
    payload_command_status_t status;

    task_safety_estop_set(false);
    bsp_time_init();
    bsp_gpio_test_reset();
    bind_stub_motors();
    arm_gravity_comp_init();
    Pump_Control_Init();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    task_chassis_init();

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &arm, (uint8_t)sizeof(arm));
    inject_proto_frame(PROTO_FUNC_ARM_TARGET, &target, (uint8_t)sizeof(target));
    inject_proto_frame(PROTO_FUNC_ARM_PUMP, &pump, (uint8_t)sizeof(pump));
    task_comm_get_arm_target(&cached_target);
    task_comm_get_arm_pump(&cached_pump);
    TEST_ASSERT(cached_target.command_seq == 700U);
    TEST_ASSERT(cached_pump.command_seq == 800U);

    /* A restarted host may not use a stale motion-bearing mode as its reset. */
    bsp_usb_cdc_test_reset();
    inject_proto_frame(PROTO_FUNC_MODE_CMD,
                       &stale_nav,
                       (uint8_t)sizeof(stale_nav));
    task_comm_get_mode_cmd(&cached_mode);
    TEST_ASSERT(cached_mode.seq == 1U);
    TEST_ASSERT(cached_mode.mode == PROTO_ROBOT_MODE_ARM);
    TEST_ASSERT(task_comm_actual_mode() == PROTO_ROBOT_MODE_ARM);
    TEST_ASSERT(find_last_tx_payload(PROTO_FUNC_COMMAND_STATUS,
                                     &status,
                                     (uint8_t)sizeof(status)) == 1U);
    TEST_ASSERT(status.command_seq == stale_nav.command_seq);
    TEST_ASSERT(status.stage == PROTO_COMMAND_STAGE_REJECTED);
    TEST_ASSERT(status.result == PROTO_COMMAND_RESULT_STALE_SEQUENCE);

    /* Stale IDLE parks the robot and opens a fresh three-lane V2 session. */
    bsp_usb_cdc_test_reset();
    inject_proto_frame(PROTO_FUNC_MODE_CMD,
                       &session_idle,
                       (uint8_t)sizeof(session_idle));
    task_comm_get_mode_cmd(&cached_mode);
    TEST_ASSERT(cached_mode.seq == 2U);
    TEST_ASSERT(cached_mode.command_seq == session_idle.command_seq);
    TEST_ASSERT(cached_mode.mode == PROTO_ROBOT_MODE_IDLE);
    TEST_ASSERT(task_comm_actual_mode() == PROTO_ROBOT_MODE_IDLE);
    TEST_ASSERT(find_last_tx_payload(PROTO_FUNC_COMMAND_STATUS,
                                     &status,
                                     (uint8_t)sizeof(status)) == 1U);
    TEST_ASSERT(status.command_seq == session_idle.command_seq);
    TEST_ASSERT(status.stage == PROTO_COMMAND_STAGE_COMPLETED);
    TEST_ASSERT(status.result == PROTO_COMMAND_RESULT_OK);

    /* The exact retry is idempotent and recovers the cached terminal status. */
    bsp_usb_cdc_test_reset();
    inject_proto_frame(PROTO_FUNC_MODE_CMD,
                       &session_idle,
                       (uint8_t)sizeof(session_idle));
    task_comm_get_mode_cmd(&cached_mode);
    TEST_ASSERT(cached_mode.seq == 2U);
    TEST_ASSERT(find_last_tx_payload(PROTO_FUNC_COMMAND_STATUS,
                                     &status,
                                     (uint8_t)sizeof(status)) == 1U);
    TEST_ASSERT(status.stage == PROTO_COMMAND_STAGE_COMPLETED);

    /* Late completions from the retired session cannot repopulate its cache. */
    bsp_usb_cdc_test_reset();
    TEST_ASSERT(task_comm_report_command_status(
                    PROTO_FUNC_ARM_TARGET,
                    700U,
                    PROTO_COMMAND_STAGE_COMPLETED,
                    PROTO_COMMAND_RESULT_OK) == APP_ERR_BUSY);
    TEST_ASSERT(bsp_usb_cdc_test_tx_size() == 0U);

    arm.command_seq = 2U;
    target.command_seq = 1U;
    pump.command_seq = 1U;
    inject_proto_frame(PROTO_FUNC_MODE_CMD, &arm, (uint8_t)sizeof(arm));
    inject_proto_frame(PROTO_FUNC_ARM_TARGET, &target, (uint8_t)sizeof(target));
    inject_proto_frame(PROTO_FUNC_ARM_PUMP, &pump, (uint8_t)sizeof(pump));
    task_comm_get_arm_target(&cached_target);
    task_comm_get_arm_pump(&cached_pump);
    TEST_ASSERT(cached_target.seq == 2U);
    TEST_ASSERT(cached_target.command_seq == 1U);
    TEST_ASSERT(cached_pump.seq == 2U);
    TEST_ASSERT(cached_pump.command_seq == 1U);
}

static void test_system_status_reports_mode_pump_and_rear_slots(void) {
    payload_system_status_t status;

    task_safety_estop_set(false);
    bsp_time_init();
    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    arm_gravity_comp_init();
    Pump_Control_Init();
    task_comm_init();
    Pump_Control_Set(1U);
    Pump_Control_SetPA8(1U);
    Pump_Control_SetPC8(1U);
    bsp_usb_cdc_test_reset();

    TEST_ASSERT(task_comm_send_system_status(1234U) > 0);
    TEST_ASSERT(find_last_tx_payload(PROTO_FUNC_SYSTEM_STATUS,
                                     &status,
                                     (uint8_t)sizeof(status)) == 1U);
    TEST_ASSERT(status.boot_id == task_comm_boot_id());
    TEST_ASSERT(status.uptime_ms == 1234U);
    TEST_ASSERT(status.actual_mode == PROTO_ROBOT_MODE_IDLE);
    TEST_ASSERT(status.main_pump_on == 1U);
    TEST_ASSERT((status.safety_flags & PROTO_SAFETY_FLAG_REAR_SLOT_A_HELD) != 0U);
    TEST_ASSERT((status.safety_flags & PROTO_SAFETY_FLAG_REAR_SLOT_B_HELD) != 0U);
}

static void test_usb_wheel_test_protocol_controls_direct_mode(void) {
    payload_wheel_test_t cmd = {
        .enable = PROTO_WHEEL_TEST_ENABLE,
        .wheel_mask = 0x0FU,
        .wheel_rads = {0.4f, 0.5f, 0.6f, 0.7f},
    };
    task_comm_chassis_cmd_t chassis;

    bind_stub_motors();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    task_chassis_init();

    inject_proto_frame(PROTO_FUNC_WHEEL_TEST, &cmd, (uint8_t)sizeof(cmd));
    TEST_ASSERT(g_chassis_wheel_test.active == 1U);
    TEST_ASSERT(g_chassis_wheel_test.wheel_mask == 0x0FU);
    TEST_ASSERT_NEAR(g_chassis_wheel_test.wheel_rads[GAIT_LEG_FL], 0.4f, 1e-6f);
    TEST_ASSERT_NEAR(g_chassis_wheel_test.wheel_rads[GAIT_LEG_FR], 0.5f, 1e-6f);
    TEST_ASSERT_NEAR(g_chassis_wheel_test.wheel_rads[GAIT_LEG_RL], 0.6f, 1e-6f);
    TEST_ASSERT_NEAR(g_chassis_wheel_test.wheel_rads[GAIT_LEG_RR], 0.7f, 1e-6f);

    task_comm_get_chassis(&chassis);
    TEST_ASSERT_NEAR(chassis.vx, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(chassis.vy, 0.0f, 1e-6f);
    TEST_ASSERT_NEAR(chassis.wz, 0.0f, 1e-6f);

    cmd.enable = PROTO_WHEEL_TEST_DISABLE;
    cmd.wheel_mask = 0U;
    inject_proto_frame(PROTO_FUNC_WHEEL_TEST, &cmd, (uint8_t)sizeof(cmd));
    TEST_ASSERT(g_chassis_wheel_test.active == 0U);
}

static void test_usb_arm_aux_gpio_protocol_controls_outputs(void) {
    payload_arm_aux_gpio_t cmd = {
        .channel = PROTO_ARM_AUX_GPIO_PC8,
        .on = 1U,
    };

    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    arm_gravity_comp_init();
    Pump_Control_Init();
    task_comm_init();

    inject_proto_frame(PROTO_FUNC_ARM_AUX_GPIO, &cmd, (uint8_t)sizeof(cmd));
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC8) == 1U);
    TEST_ASSERT(debug_pc8_on == 1U);

    cmd.channel = PROTO_ARM_AUX_GPIO_PC9;
    inject_proto_frame(PROTO_FUNC_ARM_AUX_GPIO, &cmd, (uint8_t)sizeof(cmd));
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC9) == 1U);
    TEST_ASSERT(debug_pc9_on == 1U);

    cmd.channel = PROTO_ARM_AUX_GPIO_PA8;
    inject_proto_frame(PROTO_FUNC_ARM_AUX_GPIO, &cmd, (uint8_t)sizeof(cmd));
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA8) == 1U);
    TEST_ASSERT(debug_pa8_on == 1U);

    cmd.on = 0U;
    inject_proto_frame(PROTO_FUNC_ARM_AUX_GPIO, &cmd, (uint8_t)sizeof(cmd));
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA8) == 0U);
    TEST_ASSERT(debug_pa8_on == 0U);
}

static void test_usb_mit_rejects_arm_motor_bypass(void) {
    payload_mit_cmd_t arm_cmd = {
        .motor_id = MOTOR_ID_ARM_J1,
        .pos_rad = 0.3f,
        .vel_rads = 0.0f,
        .kp = 10.0f,
        .kd = 0.5f,
        .tau_ff_nm = 0.0f,
    };
    payload_mit_cmd_t chassis_cmd = arm_cmd;
    chassis_cmd.motor_id = MOTOR_ID_FL_WHEEL;

    bind_stub_motors();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();

    inject_proto_frame(PROTO_FUNC_MIT_CMD, &arm_cmd, (uint8_t)sizeof(arm_cmd));
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_ARM_J1].set_position_count == 0U);

    inject_proto_frame(PROTO_FUNC_MIT_CMD,
                       &chassis_cmd,
                       (uint8_t)sizeof(chassis_cmd));
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_FL_WHEEL].set_position_count == 1U);
}

static void test_task_arm_startup_requests_fixed_pose_immediately(void) {
    arm_control_status_t status;
    arm_joint_angles_t measured = arm_test_home_angles();
    measured.theta1_motor_rad = 0.15f;
    measured.theta1_geo_rad =
        measured.theta1_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta1_offset_rad;
    arm_joint_angles_t fixed = arm_test_park_angles();
    arm_pose_t fixed_pose;
    arm_kinematics_forward(&ARM_KINEMATICS_DEFAULT_PARAMS, &fixed, &fixed_pose);

    log_init();
    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bind_stub_motors();
    set_arm_stub_feedback_from_angles(&measured, 0U);
    task_arm_init();
    bsp_usb_cdc_test_reset();

    task_arm_step_for_test(0.010f, 20U);
    arm_control_get_status(&status);

    TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_MOVING);
    TEST_ASSERT(debug_arm_fixed_last_result == APP_OK);
    TEST_ASSERT(status.enabled == 1U);
    TEST_ASSERT(status.motor_feedback_fresh == 1U);
    TEST_ASSERT(status.target_valid == 1U);
    TEST_ASSERT(status.target_type == ARM_CONTROL_TARGET_INTERNAL_HOLD);
    TEST_ASSERT(status.safe_move_stage == 0U);
    TEST_ASSERT(status.target_seq == 1U);
    TEST_ASSERT(debug_arm_fixed_profile == 0U);
    TEST_ASSERT_NEAR(status.target_x_m, fixed_pose.x_m, 1e-4f);
    TEST_ASSERT_NEAR(status.target_y_m, fixed_pose.y_m, 1e-4f);
    TEST_ASSERT_NEAR(status.target_z_m, fixed_pose.z_m, 1e-4f);
}

static void test_task_arm_debug_snapshot_collects_common_live_values(void) {
    const payload_arm_target_t host_target = {
        .target_type = PROTO_ARM_TARGET_GRASP,
        .x_m = 0.21f,
        .y_m = -0.11f,
        .z_m = 0.34f,
    };
    arm_control_status_t status;
    arm_joint_angles_t measured = arm_test_home_angles();
    measured.theta1_motor_rad = APP_ARM_DEG2RAD(10.0f);
    measured.theta1_geo_rad = measured.theta1_motor_rad;

    bsp_time_init();
    log_init();
    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bind_stub_motors();
    set_arm_stub_feedback_from_angles(&measured, 100U);
    task_arm_init();
    inject_proto_frame(PROTO_FUNC_ARM_TARGET,
                       &host_target,
                       (uint8_t)sizeof(host_target));

    task_arm_step_for_test(0.010f, 100U);
    arm_control_get_status(&status);

    TEST_ASSERT(g_arm_debug_snapshot.update_sequence == 1U);
    TEST_ASSERT(g_arm_debug_snapshot.update_time_ms == 100U);
    TEST_ASSERT_NEAR(g_arm_debug_snapshot.motor1_position_deg, 10.0f, 1e-4f);
    TEST_ASSERT_NEAR(g_arm_debug_snapshot.motor2_position_deg,
                     APP_ARM_RAD2DEG(s_stub_devs[MOTOR_ID_ARM_J2].state.angle_rad),
                     1e-4f);
    TEST_ASSERT_NEAR(g_arm_debug_snapshot.motor3_position_deg,
                     APP_ARM_RAD2DEG(s_stub_devs[MOTOR_ID_ARM_J3].state.angle_rad),
                     1e-4f);
    TEST_ASSERT_NEAR(g_arm_debug_snapshot.motor4_position_deg,
                     APP_ARM_RAD2DEG(s_stub_devs[MOTOR_ID_ARM_J4].state.angle_rad),
                     1e-4f);
    TEST_ASSERT(g_arm_debug_snapshot.motor1_response_age_ms == 0U);
    TEST_ASSERT(g_arm_debug_snapshot.motor2_response_age_ms == 0U);
    TEST_ASSERT(g_arm_debug_snapshot.motor3_response_age_ms == 0U);
    TEST_ASSERT(g_arm_debug_snapshot.motor4_response_age_ms == 0U);
    TEST_ASSERT_NEAR(g_arm_debug_snapshot.host_target_x_m, host_target.x_m, 1e-6f);
    TEST_ASSERT_NEAR(g_arm_debug_snapshot.host_target_y_m, host_target.y_m, 1e-6f);
    TEST_ASSERT_NEAR(g_arm_debug_snapshot.host_target_z_m, host_target.z_m, 1e-6f);
    TEST_ASSERT_NEAR(g_arm_debug_snapshot.current_end_x_m,
                     status.measured_end_x_m,
                     1e-6f);
    TEST_ASSERT_NEAR(g_arm_debug_snapshot.current_end_y_m,
                     status.measured_end_y_m,
                     1e-6f);
    TEST_ASSERT_NEAR(g_arm_debug_snapshot.current_end_z_m,
                     status.measured_end_z_m,
                     1e-6f);
}

static void test_task_arm_rejects_forbidden_grasp_without_leaving_fixed_pose(void) {
    const payload_mode_cmd_t arm_mode = { .mode = PROTO_ROBOT_MODE_ARM };
    const payload_arm_target_t forbidden_grasp = {
        .target_type = PROTO_ARM_TARGET_GRASP,
        .x_m = 0.21f,
        .y_m = -0.11f,
        .z_m = 0.34f,
    };
    arm_control_status_t status;
    arm_joint_angles_t measured = arm_test_home_angles();

    bsp_time_init();
    log_init();
    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bind_stub_motors();
    set_arm_stub_feedback_from_angles(&measured, 0U);
    task_arm_init();
    task_arm_step_for_test(0.010f, 20U);

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &arm_mode, (uint8_t)sizeof(arm_mode));
    set_arm_stub_feedback_from_angles(&measured, 40U);
    task_arm_step_for_test(0.010f, 40U);
    TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_MOVING);

    /* This integration test begins once the startup waiting pose is held. */
    debug_arm_fixed_state = ARM_FIXED_HOLDING;
    inject_proto_frame(PROTO_FUNC_ARM_TARGET,
                       &forbidden_grasp,
                       (uint8_t)sizeof(forbidden_grasp));
    set_arm_stub_feedback_from_angles(&measured, 60U);
    task_arm_step_for_test(0.010f, 60U);
    arm_control_get_status(&status);

    TEST_ASSERT(debug_arm_grasp_j1_reject_count == 1U);
    TEST_ASSERT(debug_arm_fixed_state != ARM_FIXED_RELEASED_TO_HOST);
    TEST_ASSERT(status.target_valid == 1U);
    TEST_ASSERT(status.target_type == ARM_CONTROL_TARGET_INTERNAL_HOLD);
}

static void test_task_arm_nav_mode_keeps_fixed_pose(void) {
    payload_mode_cmd_t nav_mode = {
        .mode = PROTO_ROBOT_MODE_NAV,
    };
    arm_control_status_t status;
    arm_joint_angles_t measured = arm_test_home_angles();
    measured.theta1_motor_rad = -0.20f;
    measured.theta1_geo_rad =
        measured.theta1_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta1_offset_rad;
    arm_joint_angles_t fixed = arm_test_park_angles();
    arm_pose_t fixed_pose;
    arm_kinematics_forward(&ARM_KINEMATICS_DEFAULT_PARAMS, &fixed, &fixed_pose);

    log_init();
    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bind_stub_motors();
    set_arm_stub_feedback_from_angles(&measured, 0U);
    task_arm_init();
    bsp_usb_cdc_test_reset();

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &nav_mode, (uint8_t)sizeof(nav_mode));
    task_arm_step_for_test(0.010f, 20U);
    arm_control_get_status(&status);

    TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_MOVING);
    TEST_ASSERT(status.motor_feedback_fresh == 1U);
    TEST_ASSERT(status.target_valid == 1U);
    TEST_ASSERT(status.target_type == ARM_CONTROL_TARGET_INTERNAL_HOLD);
    TEST_ASSERT(status.safe_move_stage == 0U);
    TEST_ASSERT(debug_arm_fixed_profile == 0U);
    TEST_ASSERT_NEAR(status.target_x_m, fixed_pose.x_m, 1e-4f);
    TEST_ASSERT_NEAR(status.target_y_m, fixed_pose.y_m, 1e-4f);
    TEST_ASSERT_NEAR(status.target_z_m, fixed_pose.z_m, 1e-4f);
}

static void test_task_arm_arm_mode_alone_keeps_fixed_pose(void) {
    payload_mode_cmd_t arm_mode = {
        .mode = PROTO_ROBOT_MODE_ARM,
    };
    arm_control_status_t status;
    arm_joint_angles_t measured = arm_test_home_angles();

    log_init();
    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bind_stub_motors();
    set_arm_stub_feedback_from_angles(&measured, 0U);
    task_arm_init();
    bsp_usb_cdc_test_reset();

    task_arm_step_for_test(0.010f, 20U);
    arm_control_get_status(&status);
    TEST_ASSERT(status.target_valid == 1U);
    TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_MOVING);

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &arm_mode, (uint8_t)sizeof(arm_mode));
    task_arm_step_for_test(0.010f, 40U);
    arm_control_get_status(&status);

    TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_MOVING);
    TEST_ASSERT(status.target_valid == 1U);
    TEST_ASSERT(status.target_type == ARM_CONTROL_TARGET_INTERNAL_HOLD);
    TEST_ASSERT(Arm_Control_GetMode() == ARM_CONTROL_POSITION_HOLD);
}

static void test_task_arm_live_expression_forces_gravity_only(void) {
    const payload_mode_cmd_t arm_mode = { .mode = PROTO_ROBOT_MODE_ARM };
    const payload_arm_target_t grasp = {
        .target_type = PROTO_ARM_TARGET_GRASP,
        .x_m = 0.0f,
        .y_m = 0.237065f,
        .z_m = 0.34f,
    };
    const payload_arm_pump_t pump = { .pump_on = 1U };
    arm_control_status_t status;
    arm_joint_angles_t measured = arm_test_home_angles();

    task_safety_estop_set(false);
    bsp_time_init();
    log_init();
    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bind_stub_motors();
    set_arm_stub_feedback_from_angles(&measured, 0U);
    task_arm_init();

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &arm_mode, (uint8_t)sizeof(arm_mode));
    task_arm_step_for_test(0.010f, 20U);
    TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_MOVING);

    /* Exercise command handling from the normal held waiting pose. */
    debug_arm_fixed_state = ARM_FIXED_HOLDING;
    inject_proto_frame(PROTO_FUNC_ARM_TARGET, &grasp, (uint8_t)sizeof(grasp));
    inject_proto_frame(PROTO_FUNC_ARM_PUMP, &pump, (uint8_t)sizeof(pump));
    set_arm_stub_feedback_from_angles(&measured, 40U);
    task_arm_step_for_test(0.010f, 40U);
    TEST_ASSERT(arm_pump_is_enabled() == 1U);

    Pump_Control_SetPC8(1U);
    Pump_Control_SetPC9(1U);
    Pump_Control_SetPA8(1U);
    Pump_Control_SetPA9(1U);
    debug_arm_force_gravity_only = 1U;
    set_arm_stub_feedback_from_angles(&measured, 60U);
    task_arm_step_for_test(0.010f, 60U);
    arm_control_get_status(&status);

    TEST_ASSERT(Arm_Control_IsGravityOnlyMode() == 1U);
    TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_GRAVITY_ONLY);
    TEST_ASSERT(status.target_valid == 0U);
    TEST_ASSERT(Arm_Control_GetMode() == ARM_CONTROL_GRAVITY);
    TEST_ASSERT(arm_pump_is_enabled() == 0U);
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC8) == 0U);
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC9) == 0U);
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA8) == 0U);
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA9) == 0U);

    debug_arm_force_gravity_only = 0U;
    set_arm_stub_feedback_from_angles(&measured, 80U);
    task_arm_step_for_test(0.010f, 80U);
    arm_control_get_status(&status);
    TEST_ASSERT(Arm_Control_IsGravityOnlyMode() == 0U);
    TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_MOVING);
    TEST_ASSERT(status.target_valid == 1U);
    TEST_ASSERT(status.target_type == ARM_CONTROL_TARGET_INTERNAL_HOLD);
}

static void test_task_chassis_nav_mode_allows_motion(void) {
    payload_mode_cmd_t mode = {
        .mode = PROTO_ROBOT_MODE_NAV,
    };
    payload_chassis_cmd_t cmd = {
        .vx = 0.0f,
        .vy = 0.0f,
        .wz = 1.0f,
    };

    bsp_time_init();
    log_init();
    bind_stub_motors();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    task_chassis_init();
    task_chassis_set_mode(CHASSIS_MODE_ONLINE);

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &mode, (uint8_t)sizeof(mode));

    for (uint32_t tick = 0U; tick < 1800U; tick++) {
        if ((tick % 100U) == 0U) {
            inject_proto_frame(PROTO_FUNC_CHASSIS_CMD,
                               &cmd,
                               (uint8_t)sizeof(cmd));
        }
        task_chassis_step_for_test(0.002f, (uint32_t)bsp_time_now_ms());
        bsp_time_test_advance_ms(2U);
    }

    TEST_ASSERT(task_chassis_get_gait_active() == CHASSIS_GAIT_WALK);
    TEST_ASSERT(total_position_commands() > 0U);
    assert_joint_motor_position_path_active();
    assert_wheel_drive_mit_active();
}

static void test_task_chassis_arm_mode_gates_chassis_motion(void) {
    payload_mode_cmd_t mode = {
        .mode = PROTO_ROBOT_MODE_ARM,
    };
    payload_chassis_cmd_t cmd = {
        .vx = 0.0f,
        .vy = 0.0f,
        .wz = 1.0f,
    };
    chassis_control_status_t status;

    log_init();
    bind_stub_motors();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    task_chassis_init();
    task_chassis_set_mode(CHASSIS_MODE_ONLINE);

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &mode, (uint8_t)sizeof(mode));
    inject_proto_frame(PROTO_FUNC_CHASSIS_CMD, &cmd, (uint8_t)sizeof(cmd));

    for (uint32_t tick = 0U; tick < 2200U; tick++) {
        task_chassis_step_for_test(0.002f, tick * 2U);
    }

    chassis_control_get_status(&status);
    TEST_ASSERT(status.mode == CHASSIS_MODE_STANDALONE);
    TEST_ASSERT(status.online == 0U);
    TEST_ASSERT(status.moving == 0U);
    TEST_ASSERT(status.active_gait == CHASSIS_GAIT_STAND);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(status.wheel_rads[i], 0.0f, 1e-6f);
    }
    assert_wheel_velocity_zero();
}

static void test_task_arm_consumes_protocol_in_arm_mode(void) {
    payload_mode_cmd_t mode = {
        .mode = PROTO_ROBOT_MODE_ARM,
    };
    payload_arm_target_t target = {
        .target_type = PROTO_ARM_TARGET_GRASP,
        .x_m = 0.0f,
        .y_m = 0.237065f,
        .z_m = 0.34f,
    };
    payload_arm_pump_t pump = {
        .pump_on = 1U,
    };
    arm_control_status_t status;
    arm_joint_angles_t reached = arm_test_ik_target(target.x_m, target.y_m, target.z_m);
    payload_arm_feedback_t feedback;

    bsp_time_init();
    log_init();
    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bind_stub_motors();
    /* Start at the target pose so this test isolates completion signalling
     * without exercising the multi-stage safe-move planner. */
    set_arm_stub_feedback_from_angles(&reached, 0U);
    task_arm_init();
    bsp_usb_cdc_test_reset();

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &mode, (uint8_t)sizeof(mode));
    bsp_time_test_advance_ms(20U);
    task_arm_step_for_test(0.010f, 20U);
    TEST_ASSERT(debug_arm_fixed_state != ARM_FIXED_RELEASED_TO_HOST);
    debug_arm_fixed_state = ARM_FIXED_HOLDING;
    bsp_usb_cdc_test_reset();

    inject_proto_frame(PROTO_FUNC_ARM_TARGET, &target, (uint8_t)sizeof(target));
    inject_proto_frame(PROTO_FUNC_ARM_PUMP, &pump, (uint8_t)sizeof(pump));
    bsp_time_test_advance_ms(20U);
    set_arm_stub_feedback_from_angles(&reached, 40U);
    task_arm_step_for_test(0.010f, 40U);
    arm_control_get_status(&status);

    TEST_ASSERT(status.enabled == 1U);
    TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_RELEASED_TO_HOST);
    TEST_ASSERT(status.target_valid == 1U);
    TEST_ASSERT(status.target_type == PROTO_ARM_TARGET_GRASP);
    TEST_ASSERT_NEAR(status.target_x_m, target.x_m, 1e-6f);
    TEST_ASSERT_NEAR(status.target_y_m, target.y_m, 1e-6f);
    TEST_ASSERT_NEAR(status.target_z_m, target.z_m, 1e-6f);
    TEST_ASSERT(status.pump_on == 1U);
    TEST_ASSERT(arm_pump_is_enabled() == 1U);
    TEST_ASSERT(bsp_gpio_read_latch(BSP_GPIO_ARM_PUMP_MAIN) == 1U);
    TEST_ASSERT(status.target_seq >= 1U);
    TEST_ASSERT(status.pump_seq >= 1U);
    TEST_ASSERT(find_last_tx_payload(PROTO_FUNC_ARM_FEEDBACK,
                                     &feedback,
                                     (uint8_t)sizeof(feedback)) == 1U);
    TEST_ASSERT(feedback.arm_state == PROTO_ARM_STATE_MOVING);

    /* Let the GRASP target finish and settle, then issue PLACE + pump off. */
    bsp_time_test_advance_ms(260U);
    set_arm_stub_feedback_from_angles(&reached, 300U);
    task_arm_step_for_test(0.010f, 300U);
    bsp_time_test_advance_ms(220U);
    set_arm_stub_feedback_from_angles(&reached, 520U);
    task_arm_step_for_test(0.010f, 520U);

    payload_arm_target_t place = target;
    place.target_type = PROTO_ARM_TARGET_PLACE;
    payload_arm_pump_t pump_off = { .pump_on = 0U };
    inject_proto_frame(PROTO_FUNC_ARM_TARGET, &place, (uint8_t)sizeof(place));
    inject_proto_frame(PROTO_FUNC_ARM_PUMP, &pump_off, (uint8_t)sizeof(pump_off));
    bsp_time_test_advance_ms(20U);
    set_arm_stub_feedback_from_angles(&reached, 540U);
    task_arm_step_for_test(0.010f, 540U);

    bsp_time_test_advance_ms(260U);
    set_arm_stub_feedback_from_angles(&reached, 800U);
    task_arm_step_for_test(0.010f, 800U);
    bsp_time_test_advance_ms(260U);
    set_arm_stub_feedback_from_angles(&reached, 1060U);
    task_arm_step_for_test(0.010f, 1060U);

    TEST_ASSERT(Arm_Serial_Protocol_PlaceCycleSequence() == 1U);
    TEST_ASSERT(debug_serial_waiting_next_grasp == 1U);
    TEST_ASSERT(arm_pump_is_enabled() == 0U);
    TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_WAIT_FEEDBACK);
    TEST_ASSERT(Arm_Control_GetMode() == ARM_CONTROL_GRAVITY);

    bsp_time_test_advance_ms(20U);
    task_arm_step_for_test(0.010f, 1080U);
    TEST_ASSERT(Arm_Serial_Protocol_PlaceCycleSequence() == 1U);

    bsp_time_test_advance_ms(3020U);
    set_arm_stub_feedback_from_angles(&reached, 4100U);
    task_arm_step_for_test(0.010f, 4100U);
    TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_WAIT_FEEDBACK);
    bsp_time_test_advance_ms(20U);
    set_arm_stub_feedback_from_angles(&reached, 4120U);
    task_arm_step_for_test(0.010f, 4120U);
    TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_MOVING);
}

static void test_v2_stow_uses_park_without_releasing_rear_slots(void) {
    payload_mode_cmd_v2_t arm_mode = {
        .mode = PROTO_ROBOT_MODE_ARM,
        .command_seq = 500U,
    };
    payload_arm_target_v2_t stow = {
        .target = {
            .target_type = PROTO_ARM_TARGET_STOW,
            .x_m = 0.0f,
            .y_m = 0.0f,
            .z_m = 0.0f,
        },
        .command_seq = 600U,
    };
    arm_joint_angles_t park = arm_test_park_angles();
    payload_command_status_t status;

    task_safety_estop_set(false);
    bsp_time_init();
    log_init();
    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bind_stub_motors();
    task_chassis_init();
    set_arm_stub_feedback_from_angles(&park, 0U);
    task_arm_init();
    Pump_Control_Set(1U);
    Pump_Control_SetPA8(1U);
    Pump_Control_SetPC8(1U);
    Pump_Control_SetPC9(1U);

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &arm_mode, (uint8_t)sizeof(arm_mode));
    inject_proto_frame(PROTO_FUNC_ARM_TARGET, &stow, (uint8_t)sizeof(stow));
    bsp_time_test_advance_ms(20U);
    set_arm_stub_feedback_from_angles(&park, 20U);
    task_arm_step_for_test(0.010f, 20U);

    TEST_ASSERT(debug_arm_fixed_profile == 0U);
    TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_MOVING);
    TEST_ASSERT(arm_pump_is_enabled() == 0U);
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA8) == 1U);
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC8) == 1U);
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC9) == 1U);
    TEST_ASSERT(Arm_Serial_Protocol_PlaceCycleSequence() == 0U);

    bsp_time_test_advance_ms(280U);
    set_arm_stub_feedback_from_angles(&park, 300U);
    task_arm_step_for_test(0.010f, 300U);
    bsp_time_test_advance_ms(260U);
    set_arm_stub_feedback_from_angles(&park, 560U);
    task_arm_step_for_test(0.010f, 560U);
    bsp_time_test_advance_ms(260U);
    set_arm_stub_feedback_from_angles(&park, 820U);
    task_arm_step_for_test(0.010f, 820U);
    TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_HOLDING);

    bsp_usb_cdc_test_reset();
    inject_proto_frame(PROTO_FUNC_ARM_TARGET, &stow, (uint8_t)sizeof(stow));
    TEST_ASSERT(find_last_tx_payload(PROTO_FUNC_COMMAND_STATUS,
                                     &status,
                                     (uint8_t)sizeof(status)) == 1U);
    TEST_ASSERT(status.command_seq == stow.command_seq);
    TEST_ASSERT(status.command_func == PROTO_FUNC_ARM_TARGET);
    TEST_ASSERT(status.stage == PROTO_COMMAND_STAGE_COMPLETED);
    TEST_ASSERT(status.result == PROTO_COMMAND_RESULT_OK);
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA8) == 1U);
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC8) == 1U);
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC9) == 1U);
}

static void test_task_arm_work_mode_switch_stops_old_motion_and_keeps_rear_slots(void) {
    const uint8_t source_modes[2] = {
        PROTO_ROBOT_MODE_ARM,
        PROTO_ROBOT_MODE_REAR_PLACE,
    };
    const uint8_t destination_modes[2] = {
        PROTO_ROBOT_MODE_REAR_PLACE,
        PROTO_ROBOT_MODE_ARM,
    };
    const arm_joint_angles_t measured = arm_test_home_angles();

    for (uint32_t direction = 0U; direction < 2U; direction++) {
        payload_mode_cmd_v2_t source_mode = {
            .mode = source_modes[direction],
            .command_seq = 100U + direction * 10U,
        };
        payload_mode_cmd_v2_t destination_mode = {
            .mode = destination_modes[direction],
            .command_seq = 101U + direction * 10U,
        };
        payload_arm_target_v2_t target = {
            .target = {
                .target_type = PROTO_ARM_TARGET_GRASP,
                .x_m = 0.0f,
                .y_m = 0.237065f,
                .z_m = 0.34f,
            },
            .command_seq = 200U + direction,
        };
        arm_control_status_t status;
        payload_command_status_t terminal;

        task_safety_estop_set(false);
        bsp_time_init();
        log_init();
        bsp_gpio_test_reset();
        TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
        task_comm_init();
        bind_stub_motors();
        task_chassis_init();
        set_arm_stub_feedback_from_angles(&measured, 0U);
        task_arm_init();

        inject_proto_frame(PROTO_FUNC_MODE_CMD,
                           &source_mode,
                           (uint8_t)sizeof(source_mode));
        bsp_time_test_advance_ms(20U);
        set_arm_stub_feedback_from_angles(&measured, 20U);
        task_arm_step_for_test(0.010f, 20U);
        debug_arm_fixed_state = ARM_FIXED_HOLDING;

        inject_proto_frame(PROTO_FUNC_ARM_TARGET,
                           &target,
                           (uint8_t)sizeof(target));
        bsp_time_test_advance_ms(20U);
        set_arm_stub_feedback_from_angles(&measured, 40U);
        task_arm_step_for_test(0.010f, 40U);
        arm_control_get_status(&status);
        TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_RELEASED_TO_HOST);
        TEST_ASSERT(status.target_valid == 1U);
        TEST_ASSERT(status.target_type == PROTO_ARM_TARGET_GRASP);
        TEST_ASSERT(Arm_Control_GetMoveStatus() == ARM_MOVE_MOVING ||
                    Arm_Control_GetMoveStatus() == ARM_MOVE_SETTLING);

        Pump_Control_SetPA8(1U);
        Pump_Control_SetPC8(1U);
        Pump_Control_SetPC9(1U);

        bsp_usb_cdc_test_reset();
        inject_proto_frame(PROTO_FUNC_MODE_CMD,
                           &destination_mode,
                           (uint8_t)sizeof(destination_mode));
        bsp_time_test_advance_ms(20U);
        set_arm_stub_feedback_from_angles(&measured, 60U);
        task_arm_step_for_test(0.010f, 60U);
        arm_control_get_status(&status);

        TEST_ASSERT(debug_arm_fixed_profile == 1U);
        TEST_ASSERT(debug_arm_fixed_state == ARM_FIXED_MOVING);
        TEST_ASSERT(status.target_valid == 1U);
        TEST_ASSERT(status.target_type == ARM_CONTROL_TARGET_INTERNAL_HOLD);
        TEST_ASSERT(debug_arm_rear_place_avoidance_enabled ==
                    (destination_modes[direction] ==
                     PROTO_ROBOT_MODE_REAR_PLACE ? 1U : 0U));
        TEST_ASSERT(arm_pump_is_enabled() == 0U);
        TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA8) == 1U);
        TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC8) == 1U);
        TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC9) == 1U);
        TEST_ASSERT(Arm_Serial_Protocol_PlaceCycleSequence() == 0U);
        TEST_ASSERT(find_last_tx_payload(PROTO_FUNC_COMMAND_STATUS,
                                         &terminal,
                                         (uint8_t)sizeof(terminal)) == 1U);
        TEST_ASSERT(terminal.command_func == PROTO_FUNC_ARM_TARGET);
        TEST_ASSERT(terminal.command_seq == target.command_seq);
        TEST_ASSERT(terminal.stage == PROTO_COMMAND_STAGE_ERROR);
        TEST_ASSERT(terminal.result == PROTO_COMMAND_RESULT_WRONG_MODE);
    }
}

static void test_v2_watchdog_terminates_active_target(void) {
    payload_mode_cmd_v2_t arm_mode = {
        .mode = PROTO_ROBOT_MODE_ARM,
        .command_seq = 900U,
    };
    payload_arm_target_v2_t target = {
        .target = {
            .target_type = PROTO_ARM_TARGET_GRASP,
            .x_m = 0.0f,
            .y_m = 0.237065f,
            .z_m = 0.34f,
        },
        .command_seq = 901U,
    };
    const arm_joint_angles_t measured = arm_test_home_angles();
    payload_command_status_t terminal;

    task_safety_estop_set(false);
    bsp_time_init();
    log_init();
    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bind_stub_motors();
    task_chassis_init();
    set_arm_stub_feedback_from_angles(&measured, 0U);
    task_arm_init();

    inject_proto_frame(PROTO_FUNC_MODE_CMD,
                       &arm_mode,
                       (uint8_t)sizeof(arm_mode));
    bsp_time_test_advance_ms(20U);
    set_arm_stub_feedback_from_angles(&measured, 20U);
    task_arm_step_for_test(0.010f, 20U);
    debug_arm_fixed_state = ARM_FIXED_HOLDING;

    inject_proto_frame(PROTO_FUNC_ARM_TARGET,
                       &target,
                       (uint8_t)sizeof(target));
    bsp_time_test_advance_ms(20U);
    set_arm_stub_feedback_from_angles(&measured, 40U);
    task_arm_step_for_test(0.010f, 40U);

    bsp_usb_cdc_test_reset();
    bsp_time_test_advance_ms(APP_COMM_WATCHDOG_TIMEOUT_MS + 1U);
    task_comm_watchdog_step((uint32_t)bsp_time_now_ms());
    task_arm_step_for_test(0.010f, (uint32_t)bsp_time_now_ms());

    TEST_ASSERT(find_last_tx_payload(PROTO_FUNC_COMMAND_STATUS,
                                     &terminal,
                                     (uint8_t)sizeof(terminal)) == 1U);
    TEST_ASSERT(terminal.command_func == PROTO_FUNC_ARM_TARGET);
    TEST_ASSERT(terminal.command_seq == target.command_seq);
    TEST_ASSERT(terminal.stage == PROTO_COMMAND_STAGE_ERROR);
    TEST_ASSERT(terminal.result == PROTO_COMMAND_RESULT_WATCHDOG);
}

static void settle_direct_arm_at_target(const arm_joint_angles_t* reached) {
    TEST_ASSERT(reached != NULL);
    Arm_Control_Process();
    bsp_time_test_advance_ms(300U);
    set_arm_stub_feedback_from_angles(reached, (uint32_t)bsp_time_now_ms());
    Arm_Control_Process();
    bsp_time_test_advance_ms(260U);
    set_arm_stub_feedback_from_angles(reached, (uint32_t)bsp_time_now_ms());
    Arm_Control_Process();
    bsp_time_test_advance_ms(260U);
    set_arm_stub_feedback_from_angles(reached, (uint32_t)bsp_time_now_ms());
    Arm_Control_Process();
    TEST_ASSERT(Arm_Control_GetMoveStatus() == ARM_MOVE_REACHED);
}

static void test_rear_unload_releases_only_matching_slot_after_main_pump_grasps(void) {
    const float target_x = -0.2170f;
    const float target_z = 0.3635f;

    for (uint8_t slot = 0U; slot < 2U; slot++) {
        const float target_y = (slot == 0U) ? 0.2200f : -0.2200f;
        arm_joint_angles_t reached = arm_test_ik_target(target_x, target_y, target_z);

        bsp_time_init();
        log_init();
        bsp_gpio_test_reset();
        TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
        bind_stub_motors();
        set_arm_stub_feedback_from_angles(&reached, 0U);
        arm_gravity_comp_init();
        Pump_Control_Init();
        Arm_Control_Init(NULL);
        (void)arm_control_set_rear_place_avoidance(1U);
        Arm_Serial_Protocol_Init();
        Arm_Serial_Protocol_SetRearPlaceMode(1U);
        Pump_Control_SetPA8(1U);
        Pump_Control_SetPC8(1U);
        Pump_Control_SetPC9(1U);

        Arm_Serial_Protocol_QueueTarget(PROTO_ARM_TARGET_GRASP,
                                        target_x,
                                        target_y,
                                        target_z);
        Arm_Serial_Protocol_Process();
        TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA8) == 1U);
        TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC8) == 1U);
        TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC9) == 1U);

        settle_direct_arm_at_target(&reached);
        TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA8) == 1U);
        TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC8) == 1U);
        TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC9) == 1U);

        Arm_Serial_Protocol_QueuePump(1U);
        Arm_Serial_Protocol_Process();
        TEST_ASSERT(arm_pump_is_enabled() == 1U);
        if (slot == 0U) {
            TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA8) == 0U);
            TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC8) == 1U);
            TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC9) == 1U);
        } else {
            TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA8) == 1U);
            TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC8) == 0U);
            TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC9) == 0U);
        }
    }
}

static void test_task_arm_holds_state_and_discards_usb_commands_outside_arm_mode(void) {
    payload_mode_cmd_t arm_mode = {
        .mode = PROTO_ROBOT_MODE_ARM,
    };
    payload_mode_cmd_t nav_mode = {
        .mode = PROTO_ROBOT_MODE_NAV,
    };
    payload_arm_pump_t pump = {
        .pump_on = 1U,
    };
    payload_arm_pump_t pump_off = {
        .pump_on = 0U,
    };
    payload_arm_target_t target_a = {
        .target_type = PROTO_ARM_TARGET_GRASP,
        .x_m = 0.0f,
        .y_m = 0.237065f,
        .z_m = 0.34f,
    };
    payload_arm_target_t target_b = {
        .target_type = PROTO_ARM_TARGET_PLACE,
        .x_m = 0.10f,
        .y_m = 0.05f,
        .z_m = 0.31f,
    };
    arm_control_status_t status;
    task_comm_arm_target_t cached_target;
    task_comm_arm_pump_t cached_pump;
    arm_joint_angles_t reached = arm_test_ik_target(target_a.x_m,
                                                     target_a.y_m,
                                                     target_a.z_m);

    bsp_time_init();
    log_init();
    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bind_stub_motors();
    set_arm_stub_feedback_from_angles(&reached, 0U);
    task_arm_init();

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &arm_mode, (uint8_t)sizeof(arm_mode));
    bsp_time_test_advance_ms(20U);
    task_arm_step_for_test(0.010f, 20U);
    debug_arm_fixed_state = ARM_FIXED_HOLDING;
    inject_proto_frame(PROTO_FUNC_ARM_TARGET, &target_a, (uint8_t)sizeof(target_a));
    inject_proto_frame(PROTO_FUNC_ARM_PUMP, &pump, (uint8_t)sizeof(pump));
    bsp_time_test_advance_ms(20U);
    set_arm_stub_feedback_from_angles(&reached, 40U);
    task_arm_step_for_test(0.010f, 40U);
    arm_control_get_status(&status);
    TEST_ASSERT(status.enabled == 1U);
    TEST_ASSERT(status.target_seq >= 1U);
    TEST_ASSERT(status.pump_seq >= 1U);
    TEST_ASSERT(arm_pump_is_enabled() == 1U);
    TEST_ASSERT(bsp_gpio_read_latch(BSP_GPIO_ARM_PUMP_MAIN) == 1U);

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &nav_mode, (uint8_t)sizeof(nav_mode));
    inject_proto_frame(PROTO_FUNC_ARM_TARGET, &target_b, (uint8_t)sizeof(target_b));
    inject_proto_frame(PROTO_FUNC_ARM_PUMP, &pump_off, (uint8_t)sizeof(pump_off));
    bsp_time_test_advance_ms(20U);
    set_arm_stub_feedback_from_angles(&reached, 60U);
    task_arm_step_for_test(0.010f, 60U);
    arm_control_get_status(&status);
    task_comm_get_arm_target(&cached_target);
    task_comm_get_arm_pump(&cached_pump);
    TEST_ASSERT(status.enabled == 1U);
    TEST_ASSERT(status.target_type == ARM_CONTROL_TARGET_INTERNAL_HOLD);
    TEST_ASSERT(debug_arm_fixed_profile == 0U);
    TEST_ASSERT(cached_target.seq == 1U);
    TEST_ASSERT_NEAR(cached_target.x_m, target_a.x_m, 1e-6f);
    TEST_ASSERT_NEAR(cached_target.y_m, target_a.y_m, 1e-6f);
    TEST_ASSERT_NEAR(cached_target.z_m, target_a.z_m, 1e-6f);
    TEST_ASSERT(cached_pump.seq == 2U);
    TEST_ASSERT(cached_pump.pump_on == 0U);
    TEST_ASSERT(arm_pump_is_enabled() == 0U);
    TEST_ASSERT(bsp_gpio_read_latch(BSP_GPIO_ARM_PUMP_MAIN) == 0U);
}

static void test_task_arm_estop_disables_motion_and_main_pump_but_keeps_rear_slot(void) {
    const payload_mode_cmd_t arm_mode = { .mode = PROTO_ROBOT_MODE_ARM };
    const payload_mode_cmd_t estop_mode = { .mode = PROTO_ROBOT_MODE_ESTOP };
    const payload_arm_target_t grasp = {
        .target_type = PROTO_ARM_TARGET_GRASP,
        .x_m = 0.0f,
        .y_m = 0.237065f,
        .z_m = 0.34f,
    };
    const payload_arm_pump_t pump = { .pump_on = 1U };
    arm_control_status_t status;
    arm_joint_angles_t measured = arm_test_home_angles();

    task_safety_estop_set(false);
    bsp_time_init();
    log_init();
    bsp_gpio_test_reset();
    TEST_ASSERT(bsp_usb_cdc_init() == APP_OK);
    task_comm_init();
    bind_stub_motors();
    set_arm_stub_feedback_from_angles(&measured, 0U);
    task_arm_init();
    Pump_Control_SetPA8(1U);

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &arm_mode, (uint8_t)sizeof(arm_mode));
    bsp_time_test_advance_ms(20U);
    task_arm_step_for_test(0.010f, 20U);
    debug_arm_fixed_state = ARM_FIXED_HOLDING;
    inject_proto_frame(PROTO_FUNC_ARM_TARGET, &grasp, (uint8_t)sizeof(grasp));
    inject_proto_frame(PROTO_FUNC_ARM_PUMP, &pump, (uint8_t)sizeof(pump));
    bsp_time_test_advance_ms(20U);
    set_arm_stub_feedback_from_angles(&measured, 40U);
    task_arm_step_for_test(0.010f, 40U);
    TEST_ASSERT(arm_pump_is_enabled() == 1U);

    inject_proto_frame(PROTO_FUNC_MODE_CMD, &estop_mode, (uint8_t)sizeof(estop_mode));
    bsp_time_test_advance_ms(20U);
    task_arm_step_for_test(0.010f, 60U);
    arm_control_get_status(&status);

    TEST_ASSERT(task_safety_estop_active());
    TEST_ASSERT(status.enabled == 0U);
    TEST_ASSERT(status.motor_output_enabled == 0U);
    TEST_ASSERT(arm_pump_is_enabled() == 0U);
    TEST_ASSERT(bsp_gpio_read_latch(BSP_GPIO_ARM_PUMP_MAIN) == 0U);
    TEST_ASSERT(arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA8) == 1U);
    for (uint32_t i = MOTOR_ID_ARM_J1; i <= MOTOR_ID_ARM_J4; i++) {
        TEST_ASSERT(s_stub_devs[i].state.online == 0U);
    }

    task_safety_estop_set(false);
}

static void test_damiao_mit_pack_center_values(void) {
    uint8_t data[8];
    TEST_ASSERT(motor_damiao_pack_mit(DAMIAO_MODEL_DM4340,
                                      0.0f,
                                      0.0f,
                                      0.0f,
                                      0.0f,
                                      0.0f,
                                      data) == APP_OK);
    TEST_ASSERT(data[0] == 0x7FU);
    TEST_ASSERT(data[1] == 0xFFU);
    TEST_ASSERT(data[2] == 0x7FU);
    TEST_ASSERT(data[3] == 0xF0U);
    TEST_ASSERT(data[4] == 0x00U);
    TEST_ASSERT(data[5] == 0x00U);
    TEST_ASSERT(data[6] == 0x07U);
    TEST_ASSERT(data[7] == 0xFFU);
}

static void test_dm4310_legacy_mit_wrapper_uses_registry_motor(void) {
    bind_stub_motors();
    TEST_ASSERT(DM_MIT_Control_4340(NULL,
                                    1U,
                                    90.0f,
                                    0.25f,
                                    12.0f,
                                    0.4f,
                                    1.5f) == HAL_OK);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_ARM_J1].set_position_count == 1U);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J1].last_pos, APP_ARM_PI * 0.5f, 1e-5f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J1].last_vel, 0.25f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J1].last_kp, 12.0f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J1].last_kd, 0.4f, 1e-6f);
    TEST_ASSERT_NEAR(s_stub_ctxs[MOTOR_ID_ARM_J1].last_tau, 1.5f, 1e-6f);
}

static void test_dm4310_legacy_parse_feedback_updates_debug_pose(void) {
    FDCAN_RxHeaderTypeDef header = {
        .Identifier = DM_MOTOR1_FEEDBACK_ID,
    };
    uint8_t rx[8] = {0};
    rx[0] = 0x11U;
    rx[1] = 0x80U;
    rx[2] = 0x00U;
    rx[3] = 0x80U;
    rx[4] = 0x08U;
    rx[5] = 0x00U;
    rx[6] = 36U;
    rx[7] = 42U;

    bsp_time_init();
    DM_Parse_Feedback(&header, rx);
    TEST_ASSERT(dm_feedback_motor1.is_updated == 1U);
    TEST_ASSERT(dm_feedback_motor1.is_enabled == 1U);
    TEST_ASSERT_NEAR(dm_feedback_motor1.position_rad, 0.0f, 1e-3f);
    TEST_ASSERT_NEAR(dm_feedback_motor1.temp_rotor, 42.0f, 1e-6f);
}

static void test_damiao_registry_and_tx_on_fdcan3(void) {
    bsp_fdcan_frame_t frame;
    motor_registry_init();
    TEST_ASSERT(bsp_fdcan_init() == APP_OK);
    TEST_ASSERT(motor_damiao_init_all() == APP_OK);

    const motor_cfg_t* cfg_j1 = motor_get_cfg(MOTOR_ID_ARM_J1);
    const motor_cfg_t* cfg_j5 = motor_get_cfg(MOTOR_ID_ARM_J5);
    TEST_ASSERT(cfg_j1 != NULL);
    TEST_ASSERT(cfg_j1->type == MOTOR_DAMIAO);
    TEST_ASSERT(cfg_j1->can_bus == BSP_FDCAN_3);
    TEST_ASSERT(cfg_j1->can_id == DAMIAO_CAN_ID_J1);
    TEST_ASSERT(cfg_j5 != NULL);
    TEST_ASSERT(cfg_j5->type == MOTOR_UNKNOWN);

    motor_dev_t* j1 = motor_get(MOTOR_ID_ARM_J1);
    TEST_ASSERT(j1 != NULL);
    TEST_ASSERT(j1->state.type == MOTOR_DAMIAO);
    TEST_ASSERT(j1->state.can_bus == BSP_FDCAN_3);
    TEST_ASSERT(j1->state.can_id == DAMIAO_CAN_ID_J1);
    TEST_ASSERT(j1->ops != NULL);
    TEST_ASSERT(j1->ops->set_position != NULL);
    TEST_ASSERT(j1->ops->set_position(j1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f) == APP_OK);

    TEST_ASSERT(bsp_fdcan_test_tx_count(BSP_FDCAN_3) == 1U);
    TEST_ASSERT(bsp_fdcan_test_pop_tx(BSP_FDCAN_3, &frame) == APP_OK);
    TEST_ASSERT(frame.can_id == DAMIAO_CAN_ID_J1);
    TEST_ASSERT(frame.dlc == 8U);
    TEST_ASSERT(frame.data[0] == 0x7FU);
    TEST_ASSERT(frame.data[1] == 0xFFU);
}

static void test_damiao_feedback_updates_motor_state(void) {
    motor_registry_init();
    TEST_ASSERT(bsp_fdcan_init() == APP_OK);
    TEST_ASSERT(motor_damiao_init_all() == APP_OK);

    motor_dev_t* j1 = motor_get(MOTOR_ID_ARM_J1);
    TEST_ASSERT(j1 != NULL);
    memset(&dm_feedback_motor1, 0, sizeof(dm_feedback_motor1));

    bsp_fdcan_frame_t rx;
    memset(&rx, 0, sizeof(rx));
    rx.can_id = DM_MOTOR1_FEEDBACK_ID;
    rx.dlc = 8U;
    rx.data[0] = 0x11U;  /* state=enabled, motor_id=1 */
    rx.data[1] = 0x80U;  /* pos raw 0x8000 -> near zero */
    rx.data[2] = 0x00U;
    rx.data[3] = 0x80U;  /* vel raw 0x800 -> zero */
    rx.data[4] = 0x08U;  /* torque raw 0x800 -> zero */
    rx.data[5] = 0x00U;
    rx.data[6] = 36U;
    rx.data[7] = 42U;

    bsp_fdcan_test_inject_rx(BSP_FDCAN_3, &rx);

    TEST_ASSERT(j1->state.online == 1U);
    TEST_ASSERT_NEAR(j1->state.angle_rad, 0.0f, 1e-3f);
    TEST_ASSERT_NEAR(j1->state.velocity_rads, 0.0f, 1e-3f);
    TEST_ASSERT_NEAR(j1->state.torque_nm, 0.0f, 1e-3f);
    TEST_ASSERT_NEAR(j1->state.temperature_c, 42.0f, 1e-6f);
    TEST_ASSERT(j1->state.rx_cnt == 1U);
    TEST_ASSERT(dm_feedback_motor1.is_updated == 1U);
    TEST_ASSERT(dm_feedback_motor1.is_enabled == 1U);
    TEST_ASSERT_NEAR(dm_feedback_motor1.position_rad, 0.0f, 1e-3f);
    TEST_ASSERT_NEAR(dm_feedback_motor1.temp_rotor, 42.0f, 1e-6f);
}

static void test_damiao_feedback_ignores_shifted_can_id_and_uses_payload_id(void) {
    motor_registry_init();
    TEST_ASSERT(bsp_fdcan_init() == APP_OK);
    TEST_ASSERT(motor_damiao_init_all() == APP_OK);
    memset(&dm_feedback_motor1, 0, sizeof(dm_feedback_motor1));
    memset(&dm_feedback_motor4, 0, sizeof(dm_feedback_motor4));

    motor_dev_t* j1 = motor_get(MOTOR_ID_ARM_J1);
    motor_dev_t* j4 = motor_get(MOTOR_ID_ARM_J4);
    TEST_ASSERT(j1 != NULL);
    TEST_ASSERT(j4 != NULL);

    bsp_fdcan_frame_t rx;
    memset(&rx, 0, sizeof(rx));
    rx.can_id = DM_MOTOR1_FEEDBACK_ID; /* CAN master ID may be shifted. */
    rx.dlc = 8U;
    rx.data[0] = 0x11U;            /* state=enabled, actual motor_id=1 */
    rx.data[1] = 0x90U;
    rx.data[2] = 0x00U;
    rx.data[3] = 0x80U;
    rx.data[4] = 0x08U;
    rx.data[5] = 0x00U;
    rx.data[6] = 36U;
    rx.data[7] = 42U;

    bsp_fdcan_test_inject_rx(BSP_FDCAN_3, &rx);

    TEST_ASSERT(j1->state.rx_cnt == 1U);
    TEST_ASSERT(j4->state.rx_cnt == 0U);
    TEST_ASSERT(dm_feedback_motor1.is_updated == 1U);
    TEST_ASSERT(dm_feedback_motor4.is_updated == 0U);
    TEST_ASSERT(debug_damiao_last_rx_can_id == DM_MOTOR1_FEEDBACK_ID);
    TEST_ASSERT(debug_damiao_last_payload_motor_id == 1U);
    TEST_ASSERT(debug_damiao_feedback_route_count[0] == 1U);
    TEST_ASSERT(debug_damiao_feedback_route_count[1] == 0U);
    TEST_ASSERT(debug_damiao_feedback_invalid_motor_id_count == 0U);

    rx.can_id = DM_MOTOR4_FEEDBACK_ID; /* Physical J4 may report on CAN ID 5. */
    rx.data[0] = 0x14U;                    /* state=enabled, actual motor_id=4 */
    bsp_fdcan_test_inject_rx(BSP_FDCAN_3, &rx);
    TEST_ASSERT(j4->state.rx_cnt == 1U);
    TEST_ASSERT(dm_feedback_motor4.is_updated == 1U);
    TEST_ASSERT(debug_damiao_last_rx_can_id == DM_MOTOR4_FEEDBACK_ID);
    TEST_ASSERT(debug_damiao_last_payload_motor_id == 4U);
    TEST_ASSERT(debug_damiao_feedback_route_count[3] == 1U);
    TEST_ASSERT(debug_damiao_feedback_invalid_motor_id_count == 0U);
}

static void assert_damiao_enable_frame(uint32_t expected_can_id) {
    bsp_fdcan_frame_t frame;
    TEST_ASSERT(bsp_fdcan_test_pop_tx(BSP_FDCAN_3, &frame) == APP_OK);
    TEST_ASSERT(frame.can_id == expected_can_id);
    TEST_ASSERT(frame.dlc == 8U);
    for (uint32_t i = 0U; i < 7U; i++) {
        TEST_ASSERT(frame.data[i] == 0xFFU);
    }
    TEST_ASSERT(frame.data[7] == 0xFCU);
}

static void test_damiao_auto_enable_is_periodic_and_retried(void) {
    motor_registry_init();
    TEST_ASSERT(bsp_fdcan_init() == APP_OK);
    TEST_ASSERT(motor_damiao_init_all() == APP_OK);
    bsp_fdcan_test_reset();

    TEST_ASSERT(motor_damiao_process(0U) == APP_OK);
    TEST_ASSERT(bsp_fdcan_test_tx_count(BSP_FDCAN_3) == 4U);
    assert_damiao_enable_frame(DAMIAO_CAN_ID_J1);
    assert_damiao_enable_frame(DAMIAO_CAN_ID_J2);
    assert_damiao_enable_frame(DAMIAO_CAN_ID_J3);
    assert_damiao_enable_frame(DAMIAO_CAN_ID_J4);
    TEST_ASSERT(motor_damiao_auto_enable_count() == 4U);

    TEST_ASSERT(motor_damiao_process(100U) == APP_OK);
    TEST_ASSERT(bsp_fdcan_test_tx_count(BSP_FDCAN_3) == 0U);
    TEST_ASSERT(motor_damiao_process(220U) == APP_OK);
    TEST_ASSERT(bsp_fdcan_test_tx_count(BSP_FDCAN_3) == 4U);
    TEST_ASSERT(motor_damiao_auto_enable_count() == 8U);

    bsp_fdcan_test_reset();
    TEST_ASSERT(motor_damiao_process(420U) == APP_OK);
    TEST_ASSERT(bsp_fdcan_test_tx_count(BSP_FDCAN_3) == 4U);
    TEST_ASSERT(motor_damiao_auto_enable_count() == 12U);
}

static void test_arm_control_gate_controls_damiao_auto_enable(void) {
    motor_registry_init();
    TEST_ASSERT(bsp_fdcan_init() == APP_OK);
    TEST_ASSERT(motor_damiao_init_all() == APP_OK);
    bsp_fdcan_test_reset();

    TEST_ASSERT(arm_control_init() == APP_OK);
    TEST_ASSERT(arm_control_set_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_tick(0.020f, 0U) == APP_OK);
    TEST_ASSERT(bsp_fdcan_test_tx_count(BSP_FDCAN_3) == 0U);

    TEST_ASSERT(arm_control_set_motor_output_enabled(1U) == APP_OK);
    TEST_ASSERT(arm_control_tick(0.020f, 20U) == APP_OK);
    TEST_ASSERT(bsp_fdcan_test_tx_count(BSP_FDCAN_3) == 4U);
}

int main(void) {
    test_arm_grasp_j1_forbidden_ranges_repeat_every_turn();
    test_planner_forward_and_turn();
    test_planner_uses_real_speed_and_common_wheel_scaling();
    test_gyro_bias_calibration_and_heading_pid_direction();
    test_mahony_tilt_yaw_and_reset();
    test_stationary_gyro_drift_is_suppressed();
    test_mahony_rejects_dynamic_acceleration();
    test_attitude_rejects_invalid_sample();
    test_low_speed_turn_keeps_full_wheels_and_scales_leg_assist();
    test_planner_keeps_vy_as_motion_only();
    test_planner_deadband_zeroes_wheels();
    test_planner_schedules_high_frequency_wheel_only_travel();
    test_planner_wheel_only_travel_can_fallback();
    test_trot_turn_step_drives_left_right_gait();
    test_trot_uses_per_leg_step_lengths();
    test_trot_outputs_explicit_foot_target();
    test_trot_wheel_only_travel_lifts_in_place();
    test_walk_keeps_three_leg_support();
    test_walk_uses_per_leg_step_lengths();
    test_ik_reads_foot_target_fields();
    test_wheel_drive_uses_mit_across_leg_phases();
    test_wheel_mit_velocity_integral_is_bounded_and_unwinds();
    test_wheel_drive_to_hold_latches_actual_angle_once();
    test_gravity_comp_disabled_keeps_zero_tau_ff();
    test_gravity_comp_enabled_sends_joint_tau_ff();
    test_gravity_comp_payload_com_biases_support_loads();
    test_chassis_control_end_to_end();
    test_chassis_travel_uses_trot();
    test_chassis_vx_0p1_drives_all_wheels_continuously();
    test_chassis_control_ramps_stand_height();
    test_chassis_deadband_does_not_roll_wheels();
    test_chassis_wheel_speed_ramps_with_gait_blend();
    test_chassis_motion_gait_switch_keeps_continuous_wheel_drive();
    test_chassis_translation_rotation_transitions_are_strictly_exclusive();
    test_direct_wheel_test_bypasses_gait_and_times_out_to_hold();
    test_attitude_comp_applies_balance_torque_ff();
    test_chassis_arm_load_comp_updates_leg_payload_from_measured_arm();
    test_exact_vx_0p1_frame_decodes_without_yaw();
    test_chassis_command_rejects_ambiguous_payload_length();
    test_usb_protocol_to_chassis_task_end_to_end();
    test_usb_gait_action_can_start_walk();
    test_protocol_function_ids_are_partitioned();
    test_usb_tx_priority_classification();
    test_chassis_odometry_fuses_wheel_speed_and_yaw_rate();
    test_chassis_odometry_rejects_transition_wheel_measurement();
    test_arm_kinematics_inverse_forward_roundtrip();
    test_arm_legacy_kinematics_and_gravity_wrappers();
    test_arm_motion_quintic_finishes_at_target();
    test_arm_legacy_motion_wrappers_share_quintic_engine();
    test_arm_gravity_comp_empty_pose_outputs_expected_signs();
    test_arm_gravity_comp_payload_mass_transitions_smoothly();
    test_arm_vision_transform_preserves_legacy_units();
    test_arm_pump_device_defaults_off_and_sets_main_gpio();
    test_pump_control_legacy_wrappers_update_payload_state();
    test_arm_control_pump_updates_device_and_payload_mass();
    test_arm_control_plans_target_without_reporting_reached();
    test_arm_control_output_gate_defaults_to_no_motor_commands();
    test_arm_control_reports_measured_idle_feedback_when_available();
    test_arm_control_idle_gravity_holds_current_pose_when_enabled();
    test_arm_control_idle_hold_waits_for_fresh_feedback();
    test_arm_control_output_gate_sends_damiao_commands_when_feedback_fresh();
    test_arm_control_output_gate_blocks_without_fresh_feedback();
    test_arm_control_motion_feedback_expires_at_250ms_and_keeps_pump();
    test_arm_control_payload_transition_updates_once_per_motion_tick();
    test_arm_control_reached_requires_measured_settle();
    test_arm_control_internal_hold_survives_settle_timeout();
    test_arm_control_safe_move_retract_rotate_extend_sequence();
    test_arm_control_retract_stage_uses_latest_target_for_rotate();
    test_arm_control_rotate_stage_replans_base_to_latest_target();
    test_arm_control_extend_stage_large_target_change_retracts();
    test_arm_control_fine_tracking_replans_small_updates();
    test_usb_arm_and_mode_protocol_cache();
    test_v2_payload_sizes_and_command_status_fields();
    test_v2_target_duplicate_and_stale_sequences_never_reexecute();
    test_v2_mode_heartbeat_feeds_watchdog_and_idle_alone_recovers();
    test_v2_pump_off_completes_in_watchdog_error_before_idle_reset();
    test_v2_stale_idle_resets_host_session_but_stale_non_idle_rejects();
    test_system_status_reports_mode_pump_and_rear_slots();
    test_usb_arm_feedback_frame_tx();
    test_usb_wheel_feedback_frame_tx();
    test_usb_chassis_diag_frame_tx();
    test_usb_odometry_frame_tx();
    test_usb_wheel_test_protocol_controls_direct_mode();
    test_usb_arm_aux_gpio_protocol_controls_outputs();
    test_usb_mit_rejects_arm_motor_bypass();
    test_arm_serial_protocol_legacy_parser_caches_target_and_pump();
    test_task_arm_startup_requests_fixed_pose_immediately();
    test_task_arm_debug_snapshot_collects_common_live_values();
    test_task_arm_rejects_forbidden_grasp_without_leaving_fixed_pose();
    test_task_arm_nav_mode_keeps_fixed_pose();
    test_task_arm_arm_mode_alone_keeps_fixed_pose();
    test_task_arm_live_expression_forces_gravity_only();
    test_task_chassis_nav_mode_allows_motion();
    test_task_chassis_arm_mode_gates_chassis_motion();
    test_task_arm_consumes_protocol_in_arm_mode();
    test_v2_stow_uses_park_without_releasing_rear_slots();
    test_task_arm_work_mode_switch_stops_old_motion_and_keeps_rear_slots();
    test_v2_watchdog_terminates_active_target();
    test_rear_unload_releases_only_matching_slot_after_main_pump_grasps();
    test_task_arm_holds_state_and_discards_usb_commands_outside_arm_mode();
    test_task_arm_estop_disables_motion_and_main_pump_but_keeps_rear_slot();
    test_damiao_mit_pack_center_values();
    test_dm4310_legacy_mit_wrapper_uses_registry_motor();
    test_dm4310_legacy_parse_feedback_updates_debug_pose();
    test_damiao_registry_and_tx_on_fdcan3();
    test_damiao_feedback_updates_motor_state();
    test_damiao_feedback_ignores_shifted_can_id_and_uses_payload_id();
    test_damiao_auto_enable_is_periodic_and_retried();
    test_arm_control_gate_controls_damiao_auto_enable();
    printf("fengmh_host_tests: PASS\n");
    return 0;
}
