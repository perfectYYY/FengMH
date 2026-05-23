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

    float gait_v = 0.0f;
    if (forward_speed > PLANNER_MOTION_EPSILON_M_S) {
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
        float left_v = wheel_forward_v - (cmd->wz_rad_s * PLANNER_HALF_TRACK_M);
        float right_v = wheel_forward_v + (cmd->wz_rad_s * PLANNER_HALF_TRACK_M);
        float left_w = left_v / wheel_radius;
        float right_w = right_v / wheel_radius;

        out->wheel_rads[GAIT_LEG_FL] = left_w;
        out->wheel_rads[GAIT_LEG_RL] = left_w;
        out->wheel_rads[GAIT_LEG_FR] = right_w;
        out->wheel_rads[GAIT_LEG_RR] = right_w;
    }

    return APP_OK;
}
