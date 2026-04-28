/*
 * host_sim.c — PC-side terminal actuator for the real FengMH control stack.
 */
#include "host_sim.h"

#include "gait_if.h"
#include "leg_config.h"
#include "leg_controller.h"
#include "leg_ik.h"
#include "leg_params.h"
#include "log.h"
#include "motor_if.h"
#include "motor_registry.h"
#include "script_builtin.h"
#include "task_chassis.h"

#include <math.h>
#include <string.h>

typedef struct {
    float pos_target_rad;
    float vel_target_rads;
    float kp;
    float kd;
    float tau_ff;
    uint32_t pos_calls;
    uint32_t vel_calls;
} virtual_motor_ctx_t;

static motor_dev_t s_motor[MOTOR_ID_MAX];
static virtual_motor_ctx_t s_motor_ctx[MOTOR_ID_MAX];
static leg_controller_t s_manual_lc;
static uint32_t s_now_ms;
static int s_ready;

static int virtual_set_position(motor_dev_t* d, float pos, float vel, float kp, float kd, float tau_ff) {
    if (!d || !d->drv_ctx) return -1;
    virtual_motor_ctx_t* ctx = (virtual_motor_ctx_t*)d->drv_ctx;
    ctx->pos_target_rad = pos;
    ctx->vel_target_rads = vel;
    ctx->kp = kp;
    ctx->kd = kd;
    ctx->tau_ff = tau_ff;
    ctx->pos_calls++;
    d->state.online = 1;
    d->state.angle_rad = pos;
    d->state.velocity_rads = vel;
    d->state.torque_nm = tau_ff;
    return 0;
}

static int virtual_set_velocity(motor_dev_t* d, float vel) {
    if (!d || !d->drv_ctx) return -1;
    virtual_motor_ctx_t* ctx = (virtual_motor_ctx_t*)d->drv_ctx;
    ctx->vel_target_rads = vel;
    ctx->vel_calls++;
    d->state.online = 1;
    d->state.velocity_rads = vel;
    return 0;
}

static int virtual_enable(motor_dev_t* d) {
    if (!d) return -1;
    d->state.online = 1;
    return 0;
}

static int virtual_disable(motor_dev_t* d) {
    if (!d) return -1;
    d->state.online = 0;
    return 0;
}

static const motor_ops_t s_virtual_ops = {
    .set_position = virtual_set_position,
    .set_velocity = virtual_set_velocity,
    .enable = virtual_enable,
    .disable = virtual_disable,
};

static void reset_virtual_motors(void) {
    memset(s_motor, 0, sizeof(s_motor));
    memset(s_motor_ctx, 0, sizeof(s_motor_ctx));
    for (int i = 0; i < MOTOR_ID_MAX; i++) {
        s_motor[i].ops = &s_virtual_ops;
        s_motor[i].drv_ctx = &s_motor_ctx[i];
        motor_registry_bind((motor_logical_id_t)i, &s_motor[i]);
    }
}

int host_sim_init(void) {
    log_init();
    log_set_global_level(LOG_LVL_WARN);

    motor_registry_init();
    reset_virtual_motors();

    leg_controller_init(&s_manual_lc);
    leg_controller_bind_from_registry(&s_manual_lc);

    task_chassis_init();
    task_chassis_set_mode(CHASSIS_MODE_STANDALONE);
    s_now_ms = 0;
    s_ready = 1;
    return 0;
}

int host_sim_set_stand_height(float height_m) {
    if (!s_ready) host_sim_init();
    if (!isfinite(height_m) || height_m <= 0.0f) return -1;
    leg_controller_set_stand_height(height_m);
    return 0;
}

int host_sim_apply_foot_targets(const float* dx_m, const float* dz_m, const float* wheel_rads) {
    if (!s_ready) host_sim_init();
    if (!dx_m || !dz_m) return -1;

    gait_output_t out;
    memset(&out, 0, sizeof(out));
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        out.leg[i].hip_rad = dx_m[i];
        out.leg[i].knee_rad = dz_m[i];
        out.leg[i].wheel_rads = wheel_rads ? wheel_rads[i] : 0.0f;
        out.leg[i].in_stance = 1;
    }
    return leg_controller_apply(&s_manual_lc, &out);
}

int host_sim_play_script(const char* name) {
    if (!s_ready) host_sim_init();
    const script_t* s = script_builtin_find(name);
    if (!s) return -1;
    task_chassis_set_mode(CHASSIS_MODE_STANDALONE);
    s_now_ms = 0;
    return task_chassis_play_script(s, 0.0f);
}

int host_sim_stop_script(void) {
    if (!s_ready) host_sim_init();
    return task_chassis_stop_script(0.0f);
}

int host_sim_step(float dt_s) {
    if (!s_ready) host_sim_init();
    if (!isfinite(dt_s) || dt_s < 0.0f) return -1;
    s_now_ms += (uint32_t)(dt_s * 1000.0f + 0.5f);
    task_chassis_step_for_test(dt_s, s_now_ms);
    return 0;
}

const char* host_sim_active_gait_name(void) {
    if (!s_ready) host_sim_init();
    return task_chassis_active_gait_name();
}

int host_sim_get_motor_snapshot(int logical_id, host_motor_snapshot_t* out) {
    if (!s_ready) host_sim_init();
    if (!out || logical_id < 0 || logical_id >= MOTOR_ID_MAX) return -1;
    motor_dev_t* d = motor_get((motor_logical_id_t)logical_id);
    if (!d) return -1;
    const virtual_motor_ctx_t* ctx = (const virtual_motor_ctx_t*)d->drv_ctx;
    memset(out, 0, sizeof(*out));
    out->online = d->state.online;
    out->angle_rad = d->state.angle_rad;
    out->velocity_rads = d->state.velocity_rads;
    out->torque_nm = d->state.torque_nm;
    if (ctx) {
        out->pos_target_rad = ctx->pos_target_rad;
        out->vel_target_rads = ctx->vel_target_rads;
        out->kp = ctx->kp;
        out->kd = ctx->kd;
        out->tau_ff = ctx->tau_ff;
        out->pos_calls = ctx->pos_calls;
        out->vel_calls = ctx->vel_calls;
    }
    return 0;
}

static float mount_cmd_to_raw(const motor_cfg_t* cfg, float cmd) {
    if (!cfg || cfg->dir == 0) return cmd;
    return (cmd - cfg->zero_offset) / (float)cfg->dir;
}

int host_sim_get_leg_pose(int leg_id, host_leg_pose_t* out) {
    if (!s_ready) host_sim_init();
    if (!out || leg_id < 0 || leg_id >= GAIT_LEG_NUM) return -1;

    const leg_config_t* leg = leg_config_get((gait_leg_t)leg_id);
    if (!leg) return -1;
    const motor_cfg_t* hip_cfg = motor_get_cfg(leg->motor[LEG_ACT_HIP]);
    const motor_cfg_t* knee_cfg = motor_get_cfg(leg->motor[LEG_ACT_KNEE]);
    motor_dev_t* hip = motor_get(leg->motor[LEG_ACT_HIP]);
    motor_dev_t* knee = motor_get(leg->motor[LEG_ACT_KNEE]);
    if (!hip || !knee) return -1;

    memset(out, 0, sizeof(*out));
    out->hip_cmd_rad = hip->state.angle_rad;
    out->knee_cmd_rad = knee->state.angle_rad;
    out->hip_raw_rad = mount_cmd_to_raw(hip_cfg, out->hip_cmd_rad);
    out->knee_raw_rad = mount_cmd_to_raw(knee_cfg, out->knee_cmd_rad);

    leg_fk_result_t fk;
    leg_fk_solve(out->hip_raw_rad, out->knee_raw_rad, &LEG_DIM_DEFAULT, &fk);
    out->foot_body_x_m = leg->body_x_m + leg->foot_x_dir * fk.x;
    out->foot_body_y_m = leg->body_y_m;
    out->foot_down_z_m = fk.z;
    out->ik_ok = (uint8_t)(hip->state.online && knee->state.online);
    return 0;
}
