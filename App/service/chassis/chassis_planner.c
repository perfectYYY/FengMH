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
#define PLANNER_DEFAULT_STRIDE_M        0.15707963f  /* 2*pi*0.025, from Wheel-legged */
#define PLANNER_HALF_TRACK_M            0.15f

static float clampf_local(float v, float min_v, float max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

void chassis_planner_init(void) {
}

app_err_t chassis_planner_update(const chassis_cmd_plan_t* cmd,
                                  const gait_params_t* base_trot,
                                  chassis_plan_t* out) {
    if (!cmd || !base_trot || !out) return APP_ERR_INVALID_ARG;

    memset(out, 0, sizeof(*out));
    out->gait_params = *base_trot;

    float speed = sqrtf((cmd->vx_m_s * cmd->vx_m_s) + (cmd->vy_m_s * cmd->vy_m_s));
    out->moving = (speed > PLANNER_MOTION_EPSILON_M_S ||
                   fabsf(cmd->wz_rad_s) > PLANNER_YAW_EPSILON_RAD_S) ? 1U : 0U;

    if (speed > PLANNER_MOTION_EPSILON_M_S) {
        out->gait_params.period_s = clampf_local(PLANNER_DEFAULT_STRIDE_M / speed,
                                                 PLANNER_MIN_PERIOD_S,
                                                 PLANNER_MAX_PERIOD_S);
    }

    float wheel_radius = LEG_DIM_DEFAULT.wheel_diameter * 0.5f;
    if (wheel_radius > 1e-6f) {
        float left_v = cmd->vx_m_s - (cmd->wz_rad_s * PLANNER_HALF_TRACK_M);
        float right_v = cmd->vx_m_s + (cmd->wz_rad_s * PLANNER_HALF_TRACK_M);
        float left_w = left_v / wheel_radius;
        float right_w = right_v / wheel_radius;

        out->wheel_rads[GAIT_LEG_FL] = left_w;
        out->wheel_rads[GAIT_LEG_RL] = left_w;
        out->wheel_rads[GAIT_LEG_FR] = right_w;
        out->wheel_rads[GAIT_LEG_RR] = right_w;
    }

    return APP_OK;
}
