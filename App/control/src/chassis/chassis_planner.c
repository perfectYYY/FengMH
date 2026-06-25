/*
 * chassis_planner.c
 */
#include "chassis_planner.h"
#include "leg_params.h"

#include <math.h>
#include <string.h>

#define PLANNER_MOTION_EPSILON_M_S      0.01f
#define PLANNER_YAW_EPSILON_RAD_S       0.05f
#define PLANNER_MIN_PERIOD_S            0.35f
#define PLANNER_MAX_PERIOD_S            3.00f
#define PLANNER_MAX_STEP_M              0.20f
#define PLANNER_DEFAULT_LOW_VX_THRESH_M_S 0.05f
#define PLANNER_DEFAULT_MAX_WHEEL_RADS  4.0f
#define PLANNER_DEFAULT_HALF_TRACK_M    0.15f
#define PLANNER_DEFAULT_MAX_LEG_STEP_M  0.20f
#define PLANNER_DEFAULT_TURN_STEP_HEIGHT_M 0.04f
#define PLANNER_DEFAULT_TURN_PERIOD_S   0.60f
#define PLANNER_DEFAULT_TURN_DUTY       0.50f

volatile chassis_turn_cfg_t g_chassis_turn_cfg = {
    .enable_gait_turn = 1U,
    .reserved = {0U, 0U, 0U},
    .low_vx_thresh_m_s = PLANNER_DEFAULT_LOW_VX_THRESH_M_S,
    .max_wheel_rads = PLANNER_DEFAULT_MAX_WHEEL_RADS,
    .half_track_m = PLANNER_DEFAULT_HALF_TRACK_M,
    .max_leg_step_m = PLANNER_DEFAULT_MAX_LEG_STEP_M,
    .turn_step_height_m = PLANNER_DEFAULT_TURN_STEP_HEIGHT_M,
    .turn_period_s = PLANNER_DEFAULT_TURN_PERIOD_S,
    .turn_duty = PLANNER_DEFAULT_TURN_DUTY,
};

static float clampf_local(float v, float min_v, float max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

void chassis_planner_init(void) {
}

static float planner_limit_wheel(float w) {
    float max_w = g_chassis_turn_cfg.max_wheel_rads;
    if (!isfinite(max_w) || max_w <= 0.0f) return w;
    return clampf_local(w, -max_w, max_w);
}

static int planner_is_low_speed_turn(const chassis_cmd_plan_t* cmd) {
    float low_vx_thresh = g_chassis_turn_cfg.low_vx_thresh_m_s;
    if (!isfinite(low_vx_thresh) || low_vx_thresh < 0.0f) {
        low_vx_thresh = PLANNER_DEFAULT_LOW_VX_THRESH_M_S;
    }
    return (g_chassis_turn_cfg.enable_gait_turn &&
            fabsf(cmd->vx_m_s) <= low_vx_thresh &&
            fabsf(cmd->wz_rad_s) > PLANNER_YAW_EPSILON_RAD_S);
}

static float planner_configured_half_track(void) {
    float half_track = g_chassis_turn_cfg.half_track_m;
    if (!isfinite(half_track) || half_track <= 0.0f) {
        return PLANNER_DEFAULT_HALF_TRACK_M;
    }
    return half_track;
}

static float planner_configured_max_leg_step(void) {
    float limit = g_chassis_turn_cfg.max_leg_step_m;
    if (!isfinite(limit) || limit <= 0.0f) {
        return PLANNER_DEFAULT_MAX_LEG_STEP_M;
    }
    return clampf_local(limit, 0.0f, PLANNER_MAX_STEP_M);
}

static float planner_leg_y_m(int leg_idx) {
    float half_track = planner_configured_half_track();
    return (leg_idx == GAIT_LEG_FL || leg_idx == GAIT_LEG_RL) ? half_track : -half_track;
}

static float planner_leg_local_vx(const chassis_cmd_plan_t* cmd, int leg_idx) {
    float yaw_v = g_chassis_turn_cfg.enable_gait_turn ? (cmd->wz_rad_s * planner_leg_y_m(leg_idx)) : 0.0f;
    return cmd->vx_m_s - yaw_v;
}

static void planner_apply_turn_gait(gait_params_t* p) {
    if (!p) return;

    float turn_height = p->step_height_m;
    if (isfinite(g_chassis_turn_cfg.turn_step_height_m) &&
        g_chassis_turn_cfg.turn_step_height_m > 0.0f) {
        turn_height = g_chassis_turn_cfg.turn_step_height_m;
    }

    float turn_period = p->period_s;
    if (isfinite(g_chassis_turn_cfg.turn_period_s) &&
        g_chassis_turn_cfg.turn_period_s > 0.0f) {
        turn_period = g_chassis_turn_cfg.turn_period_s;
    }

    float turn_duty = p->duty;
    if (isfinite(g_chassis_turn_cfg.turn_duty) &&
        g_chassis_turn_cfg.turn_duty > 0.0f) {
        turn_duty = g_chassis_turn_cfg.turn_duty;
    }

    p->step_length_m = 0.0f;
    p->step_height_m = clampf_local(turn_height, 0.0f, 0.08f);
    p->period_s = clampf_local(turn_period, PLANNER_MIN_PERIOD_S, PLANNER_MAX_PERIOD_S);
    p->duty = clampf_local(turn_duty, 0.30f, 0.80f);
}

static float planner_safe_duty(float duty) {
    if (!isfinite(duty) || duty < 1e-3f) return 1e-3f;
    return duty;
}

static float planner_base_period_for_speed(const gait_params_t* params, float speed_abs) {
    float duty = planner_safe_duty(params->duty);
    float base_step = fabsf(params->step_length_m);
    if (base_step < 1e-4f) base_step = 1e-4f;
    if (base_step > PLANNER_MAX_STEP_M) base_step = PLANNER_MAX_STEP_M;
    if (speed_abs <= PLANNER_MOTION_EPSILON_M_S) return params->period_s;
    return clampf_local(base_step / (speed_abs * duty), PLANNER_MIN_PERIOD_S, PLANNER_MAX_PERIOD_S);
}

static float planner_step_from_vx(float local_vx, float period_s, float duty) {
    if (fabsf(local_vx) <= PLANNER_MOTION_EPSILON_M_S) return 0.0f;
    float step = local_vx * period_s * planner_safe_duty(duty);
    float limit = planner_configured_max_leg_step();
    return clampf_local(step, -limit, limit);
}

static float planner_mean_step(const gait_params_t* p) {
    float sum = 0.0f;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        sum += p->leg_step_length_m[i];
    }
    return sum / (float)GAIT_LEG_NUM;
}

static float planner_right_left_turn_step(const gait_params_t* p) {
    float right = 0.5f * (p->leg_step_length_m[GAIT_LEG_FR] +
                          p->leg_step_length_m[GAIT_LEG_RR]);
    float left = 0.5f * (p->leg_step_length_m[GAIT_LEG_FL] +
                         p->leg_step_length_m[GAIT_LEG_RL]);
    return 0.5f * (right - left);
}

static void planner_fill_leg_steps(const chassis_cmd_plan_t* cmd,
                                   gait_params_t* params,
                                   float period_s,
                                   float* local_vx_out) {
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        float local_vx = planner_leg_local_vx(cmd, i);
        if (local_vx_out) local_vx_out[i] = local_vx;
        params->leg_step_length_m[i] = planner_step_from_vx(local_vx, period_s, params->duty);
    }
    params->step_length_m = planner_mean_step(params);
    params->turn_step_m = planner_right_left_turn_step(params);
}

app_err_t chassis_planner_update(const chassis_cmd_plan_t* cmd,
                                  const gait_params_t* base_trot,
                                  chassis_plan_t* out) {
    if (!cmd || !base_trot || !out) return APP_ERR_INVALID_ARG;

    memset(out, 0, sizeof(*out));
    out->gait_params = *base_trot;

    float forward_speed = fabsf(cmd->vx_m_s);
    float lateral_speed = fabsf(cmd->vy_m_s);
    float turn_edge_speed = fabsf(cmd->wz_rad_s) * planner_configured_half_track();
    float motion_speed = fmaxf(forward_speed, turn_edge_speed);
    out->moving = (forward_speed > PLANNER_MOTION_EPSILON_M_S ||
                   lateral_speed > PLANNER_MOTION_EPSILON_M_S ||
                   fabsf(cmd->wz_rad_s) > PLANNER_YAW_EPSILON_RAD_S) ? 1U : 0U;

    int low_speed_turn = planner_is_low_speed_turn(cmd);

    float local_vx[GAIT_LEG_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};
    if (low_speed_turn) {
        planner_apply_turn_gait(&out->gait_params);
    } else if (motion_speed > PLANNER_MOTION_EPSILON_M_S) {
        out->gait_params.period_s = planner_base_period_for_speed(&out->gait_params, motion_speed);
    } else {
        out->gait_params.step_length_m = 0.0f;
    }

    planner_fill_leg_steps(cmd,
                           &out->gait_params,
                           out->gait_params.period_s,
                           local_vx);

    float wheel_radius = LEG_DIM_DEFAULT.wheel_diameter * 0.5f;
    if (wheel_radius > 1e-6f) {
        for (int i = 0; i < GAIT_LEG_NUM; i++) {
            out->wheel_rads[i] = planner_limit_wheel(local_vx[i] / wheel_radius);
        }
    }

    return APP_OK;
}
