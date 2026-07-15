/*
 * chassis_planner.c
 */
#include "chassis_planner.h"
#include "leg_params.h"

#include <math.h>
#include <string.h>

#define PLANNER_MOTION_EPSILON_M_S      0.01f
#define PLANNER_YAW_EPSILON_RAD_S       0.05f
#define PLANNER_TRAVEL_MIN_PERIOD_S     0.15f
#define PLANNER_MAX_PERIOD_S            3.00f
#define PLANNER_MAX_STEP_M              0.20f
#define PLANNER_DEFAULT_LOW_VX_THRESH_M_S 0.05f
#define PLANNER_DEFAULT_MAX_WHEEL_RADS  18.0f
#define PLANNER_DEFAULT_HALF_TRACK_M    0.15f
#define PLANNER_DEFAULT_MAX_LEG_STEP_M  0.20f
#define PLANNER_DEFAULT_TURN_STEP_HEIGHT_M 0.035f
#define PLANNER_DEFAULT_TURN_PERIOD_S   0.80f
#define PLANNER_DEFAULT_TURN_DUTY       0.75f
#define PLANNER_DEFAULT_TURN_LEG_SCALE  0.0f
#define PLANNER_DEFAULT_SLOW_PERIOD_S   0.2525f
#define PLANNER_DEFAULT_FAST_PERIOD_S   0.2525f
#define PLANNER_DEFAULT_FAST_SPEED_M_S  0.35f
#define PLANNER_DEFAULT_TRAVEL_STEP_HEIGHT_M 0.055f
#define PLANNER_DEFAULT_TRAVEL_DUTY     0.60f

volatile chassis_turn_cfg_t g_chassis_turn_cfg = {
    .enable_gait_turn = 1U,
    .match_travel_period = 1U,
    .reserved = {0U, 0U},
    .low_vx_thresh_m_s = PLANNER_DEFAULT_LOW_VX_THRESH_M_S,
    .max_wheel_rads = PLANNER_DEFAULT_MAX_WHEEL_RADS,
    .half_track_m = PLANNER_DEFAULT_HALF_TRACK_M,
    .max_leg_step_m = PLANNER_DEFAULT_MAX_LEG_STEP_M,
    .turn_step_height_m = PLANNER_DEFAULT_TURN_STEP_HEIGHT_M,
    .turn_period_s = PLANNER_DEFAULT_TURN_PERIOD_S,
    .turn_duty = PLANNER_DEFAULT_TURN_DUTY,
    .turn_leg_scale = PLANNER_DEFAULT_TURN_LEG_SCALE,
};

volatile chassis_stride_cfg_t g_chassis_stride_cfg = {
    .enable = 1U,
    .wheel_only_travel = 1U,
    .reserved = {0U, 0U},
    .slow_period_s = PLANNER_DEFAULT_SLOW_PERIOD_S,
    .fast_period_s = PLANNER_DEFAULT_FAST_PERIOD_S,
    .fast_speed_m_s = PLANNER_DEFAULT_FAST_SPEED_M_S,
    .step_height_m = PLANNER_DEFAULT_TRAVEL_STEP_HEIGHT_M,
    .duty = PLANNER_DEFAULT_TRAVEL_DUTY,
};

volatile float g_chassis_wheel_scale[GAIT_LEG_NUM] = {1.0f, 1.0f, 1.0f, 1.0f};

static float clampf_local(float v, float min_v, float max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

void chassis_planner_init(void) {
}

static void planner_scale_wheels_to_limit(chassis_plan_t* out) {
    float max_w = g_chassis_turn_cfg.max_wheel_rads;
    if (!out || !isfinite(max_w) || max_w <= 0.0f) return;

    float peak = 0.0f;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        float scale = g_chassis_wheel_scale[i];
        if (!isfinite(scale)) scale = 1.0f;
        scale = clampf_local(scale, 0.97f, 1.03f);
        out->wheel_rads[i] *= scale;
        peak = fmaxf(peak, fabsf(out->wheel_rads[i]));
    }
    if (peak <= max_w) return;

    float common_scale = max_w / peak;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        out->wheel_rads[i] *= common_scale;
    }
    out->wheel_saturated = 1U;
}

uint8_t chassis_planner_is_low_speed_turn(const chassis_cmd_plan_t* cmd) {
    if (!cmd) return 0U;
    float low_vx_thresh = g_chassis_turn_cfg.low_vx_thresh_m_s;
    if (!isfinite(low_vx_thresh) || low_vx_thresh < 0.0f) {
        low_vx_thresh = PLANNER_DEFAULT_LOW_VX_THRESH_M_S;
    }
    return (g_chassis_turn_cfg.enable_gait_turn &&
            fabsf(cmd->vx_m_s) <= low_vx_thresh &&
            fabsf(cmd->wz_rad_s) > PLANNER_YAW_EPSILON_RAD_S) ? 1U : 0U;
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

static float planner_leg_gait_vx(const chassis_cmd_plan_t* cmd,
                                 int leg_idx,
                                 uint8_t include_translation,
                                 float turn_scale) {
    float yaw_v = g_chassis_turn_cfg.enable_gait_turn ? (cmd->wz_rad_s * planner_leg_y_m(leg_idx)) : 0.0f;
    float translation_vx = include_translation ? cmd->vx_m_s : 0.0f;
    return translation_vx - turn_scale * yaw_v;
}

static float planner_safe_duty(float duty) {
    if (!isfinite(duty) || duty < 1e-3f) return 1e-3f;
    return duty;
}

static float planner_configured_slow_period(void) {
    float period = g_chassis_stride_cfg.slow_period_s;
    if (!isfinite(period) || period <= 0.0f) period = PLANNER_DEFAULT_SLOW_PERIOD_S;
    return clampf_local(period, PLANNER_TRAVEL_MIN_PERIOD_S, PLANNER_MAX_PERIOD_S);
}

static float planner_configured_fast_period(void) {
    float period = g_chassis_stride_cfg.fast_period_s;
    if (!isfinite(period) || period <= 0.0f) period = PLANNER_DEFAULT_FAST_PERIOD_S;
    return clampf_local(period, PLANNER_TRAVEL_MIN_PERIOD_S, PLANNER_MAX_PERIOD_S);
}

static float planner_configured_fast_speed(void) {
    float speed = g_chassis_stride_cfg.fast_speed_m_s;
    if (!isfinite(speed) || speed <= PLANNER_MOTION_EPSILON_M_S) {
        speed = PLANNER_DEFAULT_FAST_SPEED_M_S;
    }
    return speed;
}

static float planner_period_from_speed(float speed_abs) {
    float slow_period = planner_configured_slow_period();
    float fast_period = planner_configured_fast_period();
    float fast_speed = planner_configured_fast_speed();
    float t = clampf_local(speed_abs / fast_speed, 0.0f, 1.0f);
    return slow_period + (fast_period - slow_period) * t;
}

static float planner_configured_travel_step_height(void) {
    float height = g_chassis_stride_cfg.step_height_m;
    if (!isfinite(height) || height <= 0.0f) {
        height = PLANNER_DEFAULT_TRAVEL_STEP_HEIGHT_M;
    }
    return clampf_local(height, 0.0f, 0.12f);
}

static float planner_configured_travel_duty(void) {
    float duty = g_chassis_stride_cfg.duty;
    if (!isfinite(duty) || duty <= 0.0f) {
        duty = PLANNER_DEFAULT_TRAVEL_DUTY;
    }
    return clampf_local(duty, 0.30f, 0.80f);
}

static void planner_apply_stride_schedule(gait_params_t* params, float motion_speed) {
    if (!params || !g_chassis_stride_cfg.enable) return;
    if (motion_speed <= PLANNER_MOTION_EPSILON_M_S) return;

    /*
     * Wheel-only travel: wheels track ground speed while the legs keep the
     * field-validated in-place cadence at the configured lift height.
     */
    params->period_s = planner_period_from_speed(motion_speed);
    params->step_height_m = planner_configured_travel_step_height();
    params->duty = planner_configured_travel_duty();
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
                                   uint8_t include_translation,
                                   float turn_scale) {
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        float local_vx = planner_leg_gait_vx(cmd,
                                             i,
                                             include_translation,
                                             turn_scale);
        params->leg_step_length_m[i] = planner_step_from_vx(local_vx, period_s, params->duty);
    }
    params->step_length_m = planner_mean_step(params);
    params->turn_step_m = planner_right_left_turn_step(params);
}

app_err_t chassis_planner_update(const chassis_cmd_plan_t* cmd,
                                  const gait_params_t* base_gait,
                                  chassis_plan_t* out) {
    if (!cmd || !base_gait || !out) return APP_ERR_INVALID_ARG;

    memset(out, 0, sizeof(*out));
    out->gait_params = *base_gait;

    float forward_speed = fabsf(cmd->vx_m_s);
    float lateral_speed = fabsf(cmd->vy_m_s);
    float turn_edge_speed = fabsf(cmd->wz_rad_s) * planner_configured_half_track();
    float motion_speed = fmaxf(forward_speed, turn_edge_speed);
    out->moving = (forward_speed > PLANNER_MOTION_EPSILON_M_S ||
                   lateral_speed > PLANNER_MOTION_EPSILON_M_S ||
                   fabsf(cmd->wz_rad_s) > PLANNER_YAW_EPSILON_RAD_S) ? 1U : 0U;

    uint8_t low_speed_turn = chassis_planner_is_low_speed_turn(cmd);
    out->low_speed_turn = low_speed_turn;

    if (low_speed_turn) {
        /* 原地/低速转向保持 stand，腿部不生成任何前后步长。 */
        out->gait_params.step_length_m = 0.0f;
        out->gait_params.turn_step_m = 0.0f;
    } else if (motion_speed > PLANNER_MOTION_EPSILON_M_S) {
        planner_apply_stride_schedule(&out->gait_params, motion_speed);
    } else {
        out->gait_params.step_length_m = 0.0f;
    }

    uint8_t include_translation = (!low_speed_turn && !g_chassis_stride_cfg.wheel_only_travel)
                                ? 1U : 0U;
    float turn_scale = low_speed_turn ? 0.0f : 1.0f;
    planner_fill_leg_steps(cmd,
                           &out->gait_params,
                           out->gait_params.period_s,
                           include_translation,
                           turn_scale);

    float wheel_radius = LEG_DIM_DEFAULT.wheel_diameter * 0.5f;
    if (out->moving && wheel_radius > 1e-6f) {
        for (int i = 0; i < GAIT_LEG_NUM; i++) {
            float wheel_vx = planner_leg_local_vx(cmd, i);
            out->wheel_rads[i] = wheel_vx / wheel_radius;
        }
        planner_scale_wheels_to_limit(out);
    }

    return APP_OK;
}
