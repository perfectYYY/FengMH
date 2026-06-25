/*
 * test_main.c - Host tests for the readable firmware control path.
 *
 * The suite covers:
 *   1. planner math for velocity/yaw commands
 *   2. trot gait output and IK conversion
 *   3. chassis_control end-to-end tick using stub motors
 */
#include "chassis_control.h"
#include "chassis_planner.h"
#include "gait_params.h"
#include "gait_trot.h"
#include "leg_controller.h"
#include "leg_ik.h"
#include "leg_params.h"
#include "log.h"
#include "motor_registry.h"
#include "bsp_usb_cdc.h"
#include "proto_defs.h"
#include "proto_frame.h"
#include "task_chassis.h"
#include "task_comm.h"

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

static void test_planner_forward_and_turn(void) {
    chassis_plan_t plan;
    chassis_cmd_plan_t cmd = {
        .vx_m_s = 0.20f,
        .vy_m_s = 0.0f,
        .wz_rad_s = 0.0f,
    };

    chassis_planner_init();
    TEST_ASSERT(chassis_planner_update(&cmd, &GAIT_PARAMS_TROT_DEFAULT, &plan) == APP_OK);
    TEST_ASSERT(plan.moving == 1U);
    TEST_ASSERT(plan.gait_params.step_length_m > 0.0f);
    TEST_ASSERT(plan.gait_params.period_s >= 0.35f);
    TEST_ASSERT_NEAR(plan.gait_params.turn_step_m, 0.0f, 1e-6f);

    cmd.vx_m_s = 0.0f;
    cmd.wz_rad_s = 1.0f;
    TEST_ASSERT(chassis_planner_update(&cmd, &GAIT_PARAMS_TROT_DEFAULT, &plan) == APP_OK);
    TEST_ASSERT(plan.moving == 1U);
    TEST_ASSERT_NEAR(plan.gait_params.step_length_m, 0.0f, 1e-6f);
    TEST_ASSERT(plan.gait_params.turn_step_m > 0.0f);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(plan.wheel_rads[i], 0.0f, 1e-6f);
    }

    cmd.vx_m_s = 0.20f;
    cmd.wz_rad_s = 0.8f;
    TEST_ASSERT(chassis_planner_update(&cmd, &GAIT_PARAMS_TROT_DEFAULT, &plan) == APP_OK);
    TEST_ASSERT(plan.gait_params.step_length_m > 0.0f);
    TEST_ASSERT(plan.gait_params.turn_step_m > 0.0f);
    for (int i = 1; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(plan.wheel_rads[i], plan.wheel_rads[0], 1e-6f);
    }
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

static void test_trot_outputs_explicit_foot_target(void) {
    gait_if_t* trot = gait_trot_create();
    gait_output_t out;

    TEST_ASSERT(trot != NULL);
    TEST_ASSERT(trot->ops->set_param(trot, &GAIT_PARAMS_TROT_DEFAULT) == APP_OK);
    TEST_ASSERT(trot->ops->init(trot) == APP_OK);
    TEST_ASSERT(trot->ops->update(trot, 0.02f, &out) == APP_OK);

    uint8_t any_foot_motion = 0U;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (fabsf(out.leg[i].foot_x_m) > 1e-6f || fabsf(out.leg[i].foot_z_m) > 1e-6f) {
            any_foot_motion = 1U;
        }
    }
    TEST_ASSERT(any_foot_motion == 1U);
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

static uint32_t wheel_velocity_commands(void) {
    return s_stub_ctxs[MOTOR_ID_FL_WHEEL].set_velocity_count +
           s_stub_ctxs[MOTOR_ID_FR_WHEEL].set_velocity_count +
           s_stub_ctxs[MOTOR_ID_RL_WHEEL].set_velocity_count +
           s_stub_ctxs[MOTOR_ID_RR_WHEEL].set_velocity_count;
}

static void test_chassis_control_end_to_end(void) {
    chassis_control_input_t input;
    chassis_control_status_t status;
    memset(&input, 0, sizeof(input));

    log_init();
    bind_stub_motors();
    chassis_control_init();
    chassis_control_set_mode(CHASSIS_MODE_ONLINE);

    input.command.vx_m_s = 0.0f;
    input.command.vy_m_s = 0.0f;
    input.command.wz_rad_s = 1.0f;
    input.valid_frame_count = 1U;
    input.last_rx_ms = 0U;

    for (uint32_t tick = 0; tick < 20U; tick++) {
        chassis_control_tick(&input, 0.002f, tick * 2U);
    }

    chassis_control_get_status(&status);
    TEST_ASSERT(status.online == 1U);
    TEST_ASSERT(status.moving == 1U);
    TEST_ASSERT(status.active_gait == CHASSIS_GAIT_TROT);
    TEST_ASSERT(status.gait_params.turn_step_m > 0.0f);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NEAR(status.wheel_rads[i], 0.0f, 1e-6f);
    }
    TEST_ASSERT(total_position_commands() > 0U);
    TEST_ASSERT(wheel_velocity_commands() == 0U);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_FL_HIP].set_position_count > 0U);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_FL_KNEE].set_position_count > 0U);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_FL_WHEEL].set_position_count > 0U);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_FL_WHEEL].last_kp > 0.0f);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_FL_WHEEL].last_kd > 0.0f);
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

    for (uint32_t tick = 0; tick < 20U; tick++) {
        task_chassis_step_for_test(0.002f, tick * 2U);
    }

    TEST_ASSERT(task_chassis_get_gait_active() == CHASSIS_GAIT_TROT);
    TEST_ASSERT(total_position_commands() > 0U);
    TEST_ASSERT(wheel_velocity_commands() == 0U);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_FR_WHEEL].set_position_count > 0U);
    TEST_ASSERT(s_stub_ctxs[MOTOR_ID_FR_WHEEL].last_kp > 0.0f);
}

int main(void) {
    test_planner_forward_and_turn();
    test_trot_turn_step_drives_left_right_gait();
    test_trot_outputs_explicit_foot_target();
    test_ik_reads_foot_target_fields();
    test_chassis_control_end_to_end();
    test_usb_protocol_to_chassis_task_end_to_end();
    printf("fengmh_host_tests: PASS\n");
    return 0;
}
