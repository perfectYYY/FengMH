#include "gait_params.h"

const gait_params_t GAIT_PARAMS_TROT_DEFAULT = {
    .body_height_m   = 0.20f,
    .step_length_m   = 0.06f,
    .turn_step_m     = 0.0f,
    .leg_step_length_m = { 0.0f, 0.0f, 0.0f, 0.0f },
    .step_height_m   = 0.04f,
    .period_s        = 0.4f,
    .duty            = 0.5f,
    .phase_offset    = { 0.0f, 0.5f, 0.5f, 0.0f },  /* FL,FR,RL,RR */
    .touchdown_thresh = 0.0f,
};

const gait_params_t GAIT_PARAMS_WALK_DEFAULT = {
    .body_height_m   = 0.20f,
    .step_length_m   = 0.045f,
    .turn_step_m     = 0.0f,
    .leg_step_length_m = { 0.0f, 0.0f, 0.0f, 0.0f },
    .step_height_m   = 0.035f,
    .period_s        = 0.60f,
    .duty            = 0.75f,
    .phase_offset    = { 0.75f, 0.25f, 0.00f, 0.50f },  /* FL,FR,RL,RR */
    .touchdown_thresh = 0.0f,
};

const gait_params_t GAIT_PARAMS_STAND_DEFAULT = {
    .body_height_m   = 0.20f,
    .step_length_m   = 0.0f,
    .turn_step_m     = 0.0f,
    .leg_step_length_m = { 0.0f, 0.0f, 0.0f, 0.0f },
    .step_height_m   = 0.0f,
    .period_s        = 1.0f,
    .duty            = 1.0f,
    .phase_offset    = { 0.0f, 0.0f, 0.0f, 0.0f },
    .touchdown_thresh = 0.0f,
};
