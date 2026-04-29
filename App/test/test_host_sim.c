/*
 * test_host_sim.c — host terminal actuator smoke tests.
 */
#include "test_util.h"
#include "host_sim.h"
#include "leg_config.h"
#include "motor_registry.h"
#include "gait_if.h"
#include "log.h"

#include <math.h>
#include <string.h>

static void t_init_binds_virtual_motors(void) {
    TEST_ASSERT_EQUAL_INT(0, host_sim_init());

    host_motor_snapshot_t m;
    TEST_ASSERT_EQUAL_INT(0, host_sim_get_motor_snapshot(MOTOR_ID_FL_HIP, &m));
    TEST_ASSERT_EQUAL_UINT(0, m.online);
}

static void t_manual_targets_flow_through_real_leg_controller(void) {
    TEST_ASSERT_EQUAL_INT(0, host_sim_init());
    TEST_ASSERT_EQUAL_INT(0, host_sim_set_stand_height(0.25f));

    float dx[GAIT_LEG_NUM] = { 0.0f, 0.0f, 0.0f, 0.0f };
    float dz[GAIT_LEG_NUM] = { 0.0f, 0.0f, 0.0f, 0.0f };
    float wheel[GAIT_LEG_NUM] = { 1.0f, 2.0f, 3.0f, 4.0f };
    TEST_ASSERT_EQUAL_INT(0, host_sim_apply_foot_targets(dx, dz, wheel));

    host_motor_snapshot_t hip;
    host_motor_snapshot_t wh;
    TEST_ASSERT_EQUAL_INT(0, host_sim_get_motor_snapshot(MOTOR_ID_FL_HIP, &hip));
    TEST_ASSERT_EQUAL_INT(0, host_sim_get_motor_snapshot(MOTOR_ID_RR_WHEEL, &wh));
    TEST_ASSERT_EQUAL_UINT(1, hip.online);
    TEST_ASSERT_TRUE(isfinite(hip.angle_rad));
    TEST_ASSERT_EQUAL_UINT(1, hip.pos_calls);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 4.0f, wh.velocity_rads);

    host_leg_pose_t pose;
    TEST_ASSERT_EQUAL_INT(0, host_sim_get_leg_pose(GAIT_LEG_FL, &pose));
    TEST_ASSERT_TRUE(isfinite(pose.foot_body_x_m));
    TEST_ASSERT_TRUE(isfinite(pose.foot_down_z_m));
    TEST_ASSERT_TRUE(isfinite(pose.knee_body_x_m));
    TEST_ASSERT_TRUE(isfinite(pose.crank_body_x_m));
    TEST_ASSERT_TRUE(isfinite(pose.lower_mount_body_x_m));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, pose.foot_body_y_m, pose.knee_body_y_m);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, pose.foot_body_y_m, pose.crank_body_y_m);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, pose.foot_body_y_m, pose.lower_mount_body_y_m);
    TEST_ASSERT_EQUAL_UINT(1, pose.ik_ok);
}

static void t_body_frame_dx_not_mirrored_between_sides(void) {
    TEST_ASSERT_EQUAL_INT(0, host_sim_init());
    TEST_ASSERT_EQUAL_INT(0, host_sim_set_stand_height(0.18f));

    float dx[GAIT_LEG_NUM] = { 0.03f, 0.03f, 0.03f, 0.03f };
    float dz[GAIT_LEG_NUM] = { 0.0f, 0.0f, 0.0f, 0.0f };
    TEST_ASSERT_EQUAL_INT(0, host_sim_apply_foot_targets(dx, dz, 0));

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        const leg_config_t* cfg = leg_config_get((gait_leg_t)i);
        host_leg_pose_t pose;
        TEST_ASSERT_NOT_NULL(cfg);
        TEST_ASSERT_EQUAL_INT(0, host_sim_get_leg_pose(i, &pose));
        TEST_ASSERT_FLOAT_WITHIN(0.005f, dx[i], pose.foot_body_x_m - cfg->body_x_m);
    }
}

static void t_script_step_uses_task_chassis(void) {
    TEST_ASSERT_EQUAL_INT(0, host_sim_init());
    TEST_ASSERT_EQUAL_INT(0, host_sim_play_script("trot_step"));
    TEST_ASSERT_EQUAL_INT(0, host_sim_step(0.02f));

    host_motor_snapshot_t hip;
    TEST_ASSERT_EQUAL_INT(0, host_sim_get_motor_snapshot(MOTOR_ID_FL_HIP, &hip));
    TEST_ASSERT_EQUAL_UINT(1, hip.online);
    TEST_ASSERT_TRUE(isfinite(hip.angle_rad));
    TEST_ASSERT_EQUAL_UINT(1, hip.pos_calls);
    TEST_ASSERT_NOT_NULL(host_sim_active_gait_name());
}

static void t_custom_trot_params_flow_through_task_chassis(void) {
    TEST_ASSERT_EQUAL_INT(0, host_sim_init());
    gait_params_t p = {
        .body_height_m = 0.18f,
        .step_length_m = 0.04f,
        .step_height_m = 0.02f,
        .period_s = 0.6f,
        .duty = 0.65f,
        .phase_offset = { 0.0f, 0.5f, 0.5f, 0.0f },
        .touchdown_thresh = 0.0f,
    };
    TEST_ASSERT_EQUAL_INT(0, host_sim_play_trot(&p, 0.0f));
    TEST_ASSERT_EQUAL_INT(0, host_sim_step(0.02f));
    TEST_ASSERT_EQUAL_INT(0, strcmp(host_sim_active_gait_name(), "trot"));

    host_leg_pose_t pose;
    TEST_ASSERT_EQUAL_INT(0, host_sim_get_leg_pose(GAIT_LEG_FL, &pose));
    TEST_ASSERT_EQUAL_UINT(1, pose.ik_ok);
    TEST_ASSERT_TRUE(isfinite(pose.foot_down_z_m));
}

int main(void) {
    log_init();
    log_set_global_level(LOG_LVL_ERR);
    TU_RUN(t_init_binds_virtual_motors);
    TU_RUN(t_manual_targets_flow_through_real_leg_controller);
    TU_RUN(t_body_frame_dx_not_mirrored_between_sides);
    TU_RUN(t_script_step_uses_task_chassis);
    TU_RUN(t_custom_trot_params_flow_through_task_chassis);
    TU_MAIN_EPILOGUE();
}
