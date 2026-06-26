/*
 * gait_trajectory.c - 步态通用足端轨迹工具
 */
#include "gait_trajectory.h"

#include <math.h>

float gait_leg_side_sign(int leg_idx) {
    return (leg_idx == GAIT_LEG_FR || leg_idx == GAIT_LEG_RR) ? 1.0f : -1.0f;
}

uint8_t gait_params_has_per_leg_steps(const gait_params_t* p) {
    if (!p) return 0U;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (fabsf(p->leg_step_length_m[i]) > 1e-7f) return 1U;
    }
    return 0U;
}

float gait_resolve_leg_step_length(const gait_params_t* p, int leg_idx) {
    if (!p || leg_idx < 0 || leg_idx >= GAIT_LEG_NUM) return 0.0f;
    if (gait_params_has_per_leg_steps(p)) {
        return p->leg_step_length_m[leg_idx];
    }
    return p->step_length_m + gait_leg_side_sign(leg_idx) * p->turn_step_m;
}

void gait_cycloid_foot_traj(float leg_phase,
                            float duty,
                            float step_len_m,
                            float step_height_m,
                            float* dx_m,
                            float* dz_m,
                            uint8_t* in_stance) {
    float lp = gait_wrap01(leg_phase);
    if (duty <= 0.0f) duty = 0.0f;
    if (duty >= 1.0f) duty = 1.0f;

    if (lp < duty) {
        float t = (duty > 0.0f) ? (lp / duty) : 0.0f;
        if (dx_m) *dx_m = step_len_m * (0.5f - t);
        if (dz_m) *dz_m = 0.0f;
        if (in_stance) *in_stance = 1U;
        return;
    }

    float swing_dur = 1.0f - duty;
    float t = (swing_dur > 0.0f) ? ((lp - duty) / swing_dur) : 0.0f;
    float theta = 2.0f * (float)M_PI * t;
    if (dx_m) {
        *dx_m = step_len_m * (t - (sinf(theta) / (2.0f * (float)M_PI)) - 0.5f);
    }
    if (dz_m) {
        *dz_m = 0.5f * step_height_m * (1.0f - cosf(theta));
    }
    if (in_stance) *in_stance = 0U;
}
