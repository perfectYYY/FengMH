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
#define PLANNER_HALF_TRACK_M            0.15f
#define PLANNER_DEFAULT_FRONT_TURN_GAIN 1.0f
#define PLANNER_DEFAULT_REAR_TURN_GAIN  0.35f
#define PLANNER_DEFAULT_LOW_VX_THRESH_M_S 0.05f
#define PLANNER_DEFAULT_MAX_WHEEL_RADS  4.0f
#define PLANNER_DEFAULT_TURN_STEP_HEIGHT_M 0.04f
#define PLANNER_DEFAULT_TURN_PERIOD_S   0.60f
#define PLANNER_DEFAULT_TURN_DUTY       0.50f

volatile chassis_turn_cfg_t g_chassis_turn_cfg = {
    .enable_turn_weight = 1U,
    .reserved = {0U, 0U, 0U},
    .front_turn_gain = PLANNER_DEFAULT_FRONT_TURN_GAIN,
    .rear_turn_gain = PLANNER_DEFAULT_REAR_TURN_GAIN,
    .low_vx_thresh_m_s = PLANNER_DEFAULT_LOW_VX_THRESH_M_S,
    .max_wheel_rads = PLANNER_DEFAULT_MAX_WHEEL_RADS,
    .turn_step_height_m = PLANNER_DEFAULT_TURN_STEP_HEIGHT_M,
    .turn_period_s = PLANNER_DEFAULT_TURN_PERIOD_S,
    .turn_duty = PLANNER_DEFAULT_TURN_DUTY,
};

static float clampf_local(float v, float min_v, float max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static float signf_local(float v) {
    if (v > 0.0f) return 1.0f;
    if (v < 0.0f) return -1.0f;
    return 0.0f;
}

void chassis_planner_init(void) {
}

static float sanitized_gain(float v, float fallback) {
    if (!isfinite(v)) return fallback;
    return clampf_local(v, 0.0f, 1.5f);
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
    return (g_chassis_turn_cfg.enable_turn_weight &&
            fabsf(cmd->vx_m_s) <= low_vx_thresh &&
            fabsf(cmd->wz_rad_s) > PLANNER_YAW_EPSILON_RAD_S);
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

app_err_t chassis_planner_update(const chassis_cmd_plan_t* cmd,
                                  const gait_params_t* base_trot,
                                  chassis_plan_t* out) {
    if (!cmd || !base_trot || !out) return APP_ERR_INVALID_ARG;

    memset(out, 0, sizeof(*out));
    out->gait_params = *base_trot;

    float forward_speed = fabsf(cmd->vx_m_s);
    float lateral_speed = fabsf(cmd->vy_m_s);
    out->moving = (forward_speed > PLANNER_MOTION_EPSILON_M_S ||
                   lateral_speed > PLANNER_MOTION_EPSILON_M_S ||
                   fabsf(cmd->wz_rad_s) > PLANNER_YAW_EPSILON_RAD_S) ? 1U : 0U;

    int low_speed_turn = planner_is_low_speed_turn(cmd);

    float gait_v = 0.0f;
    if (low_speed_turn) {
        planner_apply_turn_gait(&out->gait_params);
    } else if (forward_speed > PLANNER_MOTION_EPSILON_M_S) {
        float duty = out->gait_params.duty;
        if (duty < 1e-3f) duty = 1e-3f;

        float base_step = fabsf(out->gait_params.step_length_m);
        if (base_step < 1e-4f) base_step = 1e-4f;
        if (base_step > PLANNER_MAX_STEP_M) base_step = PLANNER_MAX_STEP_M;

        float period = base_step / (forward_speed * duty);
        period = clampf_local(period, PLANNER_MIN_PERIOD_S, PLANNER_MAX_PERIOD_S);

        float step = forward_speed * period * duty;
        if (step > PLANNER_MAX_STEP_M) step = PLANNER_MAX_STEP_M;

        float dir = signf_local(cmd->vx_m_s);
        out->gait_params.period_s = period;
        out->gait_params.step_length_m = dir * step;
        gait_v = dir * (step / (period * duty));
    } else {
        out->gait_params.step_length_m = 0.0f;
    }

    float wheel_radius = LEG_DIM_DEFAULT.wheel_diameter * 0.5f;
    if (wheel_radius > 1e-6f) {
        float wheel_forward_v = cmd->vx_m_s - gait_v;
        float yaw_v = cmd->wz_rad_s * PLANNER_HALF_TRACK_M;
        float front_gain = 1.0f;
        float rear_gain = 1.0f;
        if (low_speed_turn) {
            front_gain = sanitized_gain(g_chassis_turn_cfg.front_turn_gain,
                                        PLANNER_DEFAULT_FRONT_TURN_GAIN);
            rear_gain = sanitized_gain(g_chassis_turn_cfg.rear_turn_gain,
                                       PLANNER_DEFAULT_REAR_TURN_GAIN);
        }

        float fl_w = (wheel_forward_v - yaw_v * front_gain) / wheel_radius;
        float fr_w = (wheel_forward_v + yaw_v * front_gain) / wheel_radius;
        float rl_w = (wheel_forward_v - yaw_v * rear_gain) / wheel_radius;
        float rr_w = (wheel_forward_v + yaw_v * rear_gain) / wheel_radius;

        out->wheel_rads[GAIT_LEG_FL] = planner_limit_wheel(fl_w);
        out->wheel_rads[GAIT_LEG_FR] = planner_limit_wheel(fr_w);
        out->wheel_rads[GAIT_LEG_RL] = planner_limit_wheel(rl_w);
        out->wheel_rads[GAIT_LEG_RR] = planner_limit_wheel(rr_w);
    }

    return APP_OK;
}
