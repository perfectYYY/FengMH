/*
 * chassis_control.c - Chassis behavior pipeline.
 *
 * This file is the first pass at making the control path readable. It keeps
 * the existing behavior from the legacy task_chassis implementation, but moves
 * it out of the RTOS task wrapper and into a control module with explicit
 * inputs.
 */
#include "chassis_control.h"

#include "arm_control.h"
#include "arm_gravity_comp.h"
#include "attitude_estimator.h"
#include "bsp_time.h"
#include "chassis_planner.h"
#include "config.h"
#include "err.h"
#include "gait_machine.h"
#include "gait_params.h"
#include "gait_script.h"
#include "gait_stand.h"
#include "gait_trajectory.h"
#include "gait_trot.h"
#include "gait_walk.h"
#include "imu_bmi088.h"
#include "leg_controller.h"
#include "leg_ik.h"
#include "leg_params.h"
#include "log.h"
#include "motor_go.h"
#include "motor_m3508.h"
#include "steer_controller.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char* TAG = "CHASSIS";

#define STAND_RISE_START_HEIGHT_M 0.12f
#define STAND_HEIGHT_RATE_MPS     0.025f
#define ATTITUDE_COMP_DEFAULT_HALF_LENGTH_M 0.25f
#define ATTITUDE_COMP_DEFAULT_HALF_TRACK_M  0.15f
#define ATTITUDE_COMP_DEFAULT_LIMIT_M       0.025f
#define ATTITUDE_COMP_DEFAULT_ROLL_KP_NM_PER_RAD   8.0f
#define ATTITUDE_COMP_DEFAULT_PITCH_KP_NM_PER_RAD  8.0f
#define ATTITUDE_COMP_DEFAULT_ROLL_KD_NM_PER_RADS  0.4f
#define ATTITUDE_COMP_DEFAULT_PITCH_KD_NM_PER_RADS 0.4f
#define ATTITUDE_COMP_DEFAULT_DEADBAND_RAD         0.010f
#define ATTITUDE_COMP_DEFAULT_MAX_MOMENT_NM        3.0f
#define ATTITUDE_COMP_DEFAULT_MAX_LEG_FORCE_N      20.0f
#define ATTITUDE_COMP_DEFAULT_SMOOTH_TAU_S         0.08f
#define ARM_LOAD_COMP_DEFAULT_END_TO_COM_RATIO 0.55f
#define ARM_LOAD_COMP_DEFAULT_SMOOTH_TAU_S     0.08f
#define ARM_LOAD_COMP_DEFAULT_MAX_LEG_KG       2.5f
#define ARM_LOAD_COMP_DEFAULT_MAX_TAU_NM       3.0f
#define CHASSIS_CMD_SLEW_VX_MPS2               0.80f
#define CHASSIS_CMD_SLEW_VY_MPS2               0.60f
#define CHASSIS_CMD_SLEW_WZ_RADPS2             1.50f
#define CHASSIS_WHEEL_TEST_TIMEOUT_MS           500U
#define CHASSIS_WHEEL_TEST_MAX_RADS             18.0f
#define CHASSIS_WHEEL_TEST_DEFAULT_PERIOD_S     0.50f
#define CHASSIS_WHEEL_TEST_DEFAULT_HEIGHT_M     0.015f
#define CHASSIS_WHEEL_TEST_DEFAULT_DUTY         0.70f
#define CHASSIS_TROT_TEST_MIN_PERIOD_S           0.20f
#define CHASSIS_TROT_TEST_MAX_PERIOD_S           1.00f
#define CHASSIS_TROT_TEST_MAX_HEIGHT_M           0.060f
#define CHASSIS_TROT_TEST_MIN_DUTY               0.50f
#define CHASSIS_TROT_TEST_MAX_DUTY               0.85f
#define CHASSIS_TROT_TEST_MAX_TRIM_M             0.010f
#define CHASSIS_WHEEL_TEST_MODE_COAST            2U
#define CHASSIS_WHEEL_TEST_MODE_VERTICAL_DRIVE   3U
#define HEADING_HOLD_MIN_VX_M_S                  0.05f
#define HEADING_HOLD_MANUAL_WZ_RAD_S             0.05f
#define SLIP_FILTER_TAU_S                         0.10f
#define SLIP_WARN_RAD_S                           0.20f
#define SLIP_ACTIVE_RAD_S                         0.35f
#define SLIP_SEVERE_RAD_S                         0.60f
#define SLIP_WARN_HOLD_S                          0.20f
#define SLIP_ACTIVE_HOLD_S                        0.30f
#define SLIP_RECOVER_RAD_S                        0.12f
#define SLIP_RECOVER_HOLD_S                       1.00f
#define SLIP_SCALE_DOWN_RATE_S                    1.00f
#define SLIP_SCALE_UP_RATE_S                      0.20f

typedef enum {
    ACTIVE_STAND = 0,
    ACTIVE_TROT,
    ACTIVE_WALK,
    ACTIVE_SCRIPT,
} active_gait_t;

static gait_machine_t   s_gait_machine;
static leg_controller_t s_leg_controller;
static gait_if_t*       s_stand_gait;
static gait_if_t*       s_trot_gait;
static gait_if_t*       s_walk_gait;
static gait_if_t*       s_script_gait;

static chassis_mode_t s_mode = CHASSIS_MODE_AUTO;
static active_gait_t  s_active = ACTIVE_STAND;
static uint32_t       s_online_timeout_ms = 500U;
static gait_params_t  s_trot_params;
static gait_params_t  s_walk_params;
static uint8_t        s_manual_gait_hold = 0U;
static float          s_last_effective_wz = 0.0f;
static float          s_target_stand_height_m = 0.20f;
static float          s_current_stand_height_m = STAND_RISE_START_HEIGHT_M;
static uint8_t        s_boot_stand_done = 0U;

static uint8_t  s_offline_seq_active = 0U;
static uint8_t  s_offline_seq_done = 0U;
static uint32_t s_offline_seq_start_ms = 0U;
static steer_mode_t s_steer_mode = STEER_MODE_OFF;
static chassis_plan_t s_chassis_plan;
static uint8_t s_last_online = 0U;
volatile chassis_attitude_comp_debug_t g_chassis_attitude_comp;
volatile chassis_arm_load_comp_debug_t g_chassis_arm_load_comp;
volatile chassis_wheel_test_debug_t g_chassis_wheel_test;
volatile chassis_trot_test_debug_t g_chassis_trot_test;
static uint8_t s_wheel_test_was_active = 0U;
static uint8_t s_wheel_test_last_mode = 0U;
static float s_wheel_test_vertical_phase = 0.0f;
static uint8_t s_attitude_balance_filter_valid = 0U;
static uint8_t s_attitude_balance_was_enabled = 0U;
static float s_attitude_filtered_mx_nm = 0.0f;
static float s_attitude_filtered_my_nm = 0.0f;
static uint8_t s_arm_load_filter_valid = 0U;
static uint8_t s_arm_load_was_enabled = 0U;
static float s_arm_load_filtered_mass_kg = 0.0f;
static float s_arm_load_filtered_com_x_m = 0.0f;
static float s_arm_load_filtered_com_y_m = 0.0f;
static chassis_cmd_plan_t s_slewed_plan_cmd;
static uint8_t s_slewed_plan_valid = 0U;
static uint8_t s_heading_latch_valid = 0U;
static uint8_t s_heading_hold_active = 0U;
static float s_heading_latched_yaw = 0.0f;
static float s_slip_residual = 0.0f;
static float s_slip_warn_elapsed = 0.0f;
static float s_slip_active_elapsed = 0.0f;
static float s_slip_severe_elapsed = 0.0f;
static float s_slip_recover_elapsed = 0.0f;
static float s_speed_scale = 1.0f;
static uint16_t s_diagnostic_flags = 0U;

#if APP_OFFLINE_AUTO_MARCH
static const gait_params_t S_OFFLINE_MARCH_PARAMS = {
    .body_height_m    = 0.20f,
    .step_length_m    = 0.0f,
    .turn_step_m      = 0.0f,
    .leg_step_length_m = { 0.0f, 0.0f, 0.0f, 0.0f },
    .step_height_m    = 0.015f,
    .period_s         = 1.0f,
    .duty             = 0.50f,
    .phase_offset     = { 0.0f, 0.5f, 0.5f, 0.0f },
    .touchdown_thresh = 0.0f,
};
#endif

/* Convert user-facing body height to the IK convention: feet below hip are negative z. */
static float controller_height_from_body(float body_height_m) {
    if (!isfinite(body_height_m) || body_height_m == 0.0f) {
        return -0.18f;
    }
    return (body_height_m > 0.0f) ? -body_height_m : body_height_m;
}

static float valid_body_height_or_default(float body_height_m) {
    if (!isfinite(body_height_m) || body_height_m == 0.0f) {
        return 0.18f;
    }
    return fabsf(body_height_m);
}

static float ramp_step(float current, float target, float max_delta) {
    float err = target - current;
    if (fabsf(err) <= max_delta) return target;
    return current + ((err > 0.0f) ? max_delta : -max_delta);
}

static float chassis_clampf(float v, float min_v, float max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static float positive_or_default(float value, float fallback) {
    return (isfinite(value) && value > 0.0f) ? value : fallback;
}

static void set_leg_controller_height(float body_height_m) {
    leg_controller_set_stand_height(controller_height_from_body(body_height_m));
}

/* Record the requested body height; the per-tick ramp applies it smoothly. */
static void apply_controller_height(const gait_params_t* params) {
    if (!params) return;
    s_target_stand_height_m = valid_body_height_or_default(params->body_height_m);
}

static void reset_stand_height_ramp(const gait_params_t* params) {
    float target = params ? valid_body_height_or_default(params->body_height_m) : 0.18f;
    s_target_stand_height_m = target;
    s_current_stand_height_m = fminf(STAND_RISE_START_HEIGHT_M, target);
    set_leg_controller_height(s_current_stand_height_m);
}

static void update_stand_height_ramp(float dt_s) {
    if (!isfinite(dt_s) || dt_s < 0.0f) dt_s = 0.0f;
    if (dt_s > 0.02f) dt_s = 0.02f;

    s_current_stand_height_m = ramp_step(s_current_stand_height_m,
                                         s_target_stand_height_m,
                                         STAND_HEIGHT_RATE_MPS * dt_s);
    set_leg_controller_height(s_current_stand_height_m);
}

/* Validate host-provided gait parameters before they enter the gait machine. */
static int validate_gait_params(const gait_params_t* params) {
    if (!params) return 0;
    if (!isfinite(params->body_height_m) || fabsf(params->body_height_m) < 0.05f ||
        fabsf(params->body_height_m) > 0.40f) {
        return 0;
    }
    if (!isfinite(params->step_length_m) || fabsf(params->step_length_m) > 0.20f) {
        return 0;
    }
    if (!isfinite(params->turn_step_m) || fabsf(params->turn_step_m) > 0.20f) {
        return 0;
    }
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (!isfinite(params->leg_step_length_m[i]) ||
            fabsf(params->leg_step_length_m[i]) > 0.20f) {
            return 0;
        }
    }
    if (!isfinite(params->step_height_m) ||
        params->step_height_m < 0.0f ||
        params->step_height_m > 0.12f) {
        return 0;
    }
    if (!isfinite(params->period_s) || params->period_s < 0.10f || params->period_s > 5.0f) {
        return 0;
    }
    if (!isfinite(params->duty) || params->duty < 0.05f || params->duty > 0.95f) {
        return 0;
    }
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (!isfinite(params->phase_offset[i])) {
            return 0;
        }
    }
    if (!isfinite(params->touchdown_thresh)) {
        return 0;
    }
    return 1;
}

/* Request a gait switch while hiding gait_machine's RUN/BLEND restrictions from callers. */
static int request_gait(gait_if_t* gait, const gait_params_t* params, float blend_s) {
    if (s_gait_machine.current == gait) {
        if (gait && gait->ops && gait->ops->set_param && params) {
            return gait->ops->set_param(gait, params);
        }
        return APP_OK;
    }

    if (s_gait_machine.state != GM_STATE_RUN) {
        return gait_machine_set(&s_gait_machine, gait, params);
    }
    return gait_machine_request(&s_gait_machine, gait, params, blend_s);
}

/* AUTO resolves to online only after at least one valid frame and no heartbeat timeout. */
static int is_offline(const chassis_control_input_t* input, uint32_t now_ms) {
    if (s_mode == CHASSIS_MODE_STANDALONE) return 1;
    if (s_mode == CHASSIS_MODE_ONLINE) return 0;
    if (!input || input->valid_frame_count == 0U) return 1;
    return (now_ms - input->last_rx_ms) > s_online_timeout_ms;
}

/* Online policy: travel -> trot, low-speed/in-place turn -> walk, still -> stand. */
static void online_decide(const chassis_plan_t* plan) {
    if (!plan) return;

    if (plan->moving) {
        gait_if_t* target_gait = plan->low_speed_turn ? s_walk_gait : s_trot_gait;
        active_gait_t target_active = plan->low_speed_turn ? ACTIVE_WALK : ACTIVE_TROT;

        if (s_active != target_active) {
            apply_controller_height(&plan->gait_params);
            if (request_gait(target_gait, &plan->gait_params, 0.3f) == APP_OK) {
                s_active = target_active;
                s_manual_gait_hold = 0U;
            }
        } else if (target_gait && target_gait->ops && target_gait->ops->set_param) {
            apply_controller_height(&plan->gait_params);
            (void)target_gait->ops->set_param(target_gait, &plan->gait_params);
        }
        return;
    }

    if (s_active != ACTIVE_STAND) {
        if (chassis_control_start_stand(0.3f) == APP_OK) {
            s_active = ACTIVE_STAND;
        }
    }
}

static uint8_t is_motion_gait(const gait_if_t* gait) {
    return (gait == s_trot_gait || gait == s_walk_gait) ? 1U : 0U;
}

static float gait_motion_scale(void) {
    if (s_gait_machine.state == GM_STATE_RUN) {
        return is_motion_gait(s_gait_machine.current) ? 1.0f : 0.0f;
    }

    if (s_gait_machine.state == GM_STATE_BLEND) {
        float t = (s_gait_machine.blend_dur_s > 1e-6f)
                ? (s_gait_machine.blend_t_s / s_gait_machine.blend_dur_s)
                : 1.0f;
        t = chassis_clampf(t, 0.0f, 1.0f);

        uint8_t current_is_motion = is_motion_gait(s_gait_machine.current);
        uint8_t target_is_motion = is_motion_gait(s_gait_machine.target);
        if (current_is_motion && target_is_motion) {
            return 1.0f;
        }

        if (target_is_motion) {
            return t;
        }
        if (current_is_motion) {
            return 1.0f - t;
        }
    }

    return 0.0f;
}

/* Motion gaits drive all wheels continuously; leg phase only controls contact feedforward. */
static void apply_plan_wheel_speed(gait_output_t* output, const chassis_plan_t* plan) {
    if (!output || !plan) return;
    float motion_scale = gait_motion_scale();
    float speed_scale = plan->moving ? motion_scale : 0.0f;
    output->wheel_mode = (motion_scale > 0.0f) ? GAIT_WHEEL_DRIVE : GAIT_WHEEL_HOLD;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        output->leg[i].wheel_rads = plan->wheel_rads[i] * speed_scale;
    }
}

static float attitude_comp_valid_or_default(float value, float fallback) {
    return (isfinite(value) && value > 0.0f) ? value : fallback;
}

static float attitude_comp_scale(void) {
    float scale = g_chassis_attitude_comp.scale;
    return isfinite(scale) ? scale : 0.0f;
}

static float attitude_comp_param_or_default(float value, float fallback) {
    return (isfinite(value) && value >= 0.0f) ? value : fallback;
}

static float attitude_comp_apply_deadband(float value, float deadband) {
    if (!isfinite(value)) return 0.0f;
    if (!isfinite(deadband) || deadband <= 0.0f) return value;
    if (fabsf(value) <= deadband) return 0.0f;
    return value - ((value > 0.0f) ? deadband : -deadband);
}

static float attitude_comp_limit_moment(float moment_nm) {
    float limit = attitude_comp_valid_or_default(g_chassis_attitude_comp.max_moment_nm,
                                                ATTITUDE_COMP_DEFAULT_MAX_MOMENT_NM);
    return chassis_clampf(moment_nm, -limit, limit);
}

static float attitude_comp_alpha(float dt_s) {
    float tau_s = g_chassis_attitude_comp.smooth_tau_s;
    if (!isfinite(dt_s) || dt_s <= 0.0f) return 1.0f;
    if (!isfinite(tau_s) || tau_s <= 0.0f) return 1.0f;
    if (dt_s > 0.05f) dt_s = 0.05f;
    return chassis_clampf(dt_s / (tau_s + dt_s), 0.0f, 1.0f);
}

static void attitude_comp_filter_update(float mx_nm, float my_nm, float dt_s) {
    if (!s_attitude_balance_filter_valid) {
        s_attitude_filtered_mx_nm = mx_nm;
        s_attitude_filtered_my_nm = my_nm;
        s_attitude_balance_filter_valid = 1U;
        return;
    }

    float alpha = attitude_comp_alpha(dt_s);
    s_attitude_filtered_mx_nm += alpha * (mx_nm - s_attitude_filtered_mx_nm);
    s_attitude_filtered_my_nm += alpha * (my_nm - s_attitude_filtered_my_nm);
}

static void attitude_comp_release_leg_balance(void) {
    if (!s_attitude_balance_was_enabled) return;

    g_leg_gravity_comp.compensate_balance = 0U;
    g_leg_gravity_comp.balance_active_mask = 0U;
    g_leg_gravity_comp.balance_fz_n = 0.0f;
    g_leg_gravity_comp.balance_mx_nm = 0.0f;
    g_leg_gravity_comp.balance_my_nm = 0.0f;
    g_leg_gravity_comp.balance_applied_fz_n = 0.0f;
    g_leg_gravity_comp.balance_applied_mx_nm = 0.0f;
    g_leg_gravity_comp.balance_applied_my_nm = 0.0f;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        g_leg_gravity_comp.balance_leg_force_n[i] = 0.0f;
    }

    s_attitude_balance_filter_valid = 0U;
    s_attitude_balance_was_enabled = 0U;
}

static void attitude_comp_write_leg_balance(float mx_nm, float my_nm) {
    s_attitude_balance_was_enabled = 1U;
    g_leg_gravity_comp.enable = 1U;
    g_leg_gravity_comp.compensate_balance = 1U;
    g_leg_gravity_comp.balance_fz_n = 0.0f;
    g_leg_gravity_comp.balance_mx_nm = mx_nm;
    g_leg_gravity_comp.balance_my_nm = my_nm;
    g_leg_gravity_comp.support_half_length_m = attitude_comp_valid_or_default(
        g_chassis_attitude_comp.half_length_m,
        ATTITUDE_COMP_DEFAULT_HALF_LENGTH_M);
    g_leg_gravity_comp.support_half_track_m = attitude_comp_valid_or_default(
        g_chassis_attitude_comp.half_track_m,
        ATTITUDE_COMP_DEFAULT_HALF_TRACK_M);
    g_leg_gravity_comp.max_balance_leg_force_n = attitude_comp_valid_or_default(
        g_chassis_attitude_comp.max_leg_force_n,
        ATTITUDE_COMP_DEFAULT_MAX_LEG_FORCE_N);
}

static void attitude_comp_clear_debug(void) {
    g_chassis_attitude_comp.roll_rad = 0.0f;
    g_chassis_attitude_comp.pitch_rad = 0.0f;
    g_chassis_attitude_comp.roll_rate_rad_s = 0.0f;
    g_chassis_attitude_comp.pitch_rate_rad_s = 0.0f;
    g_chassis_attitude_comp.target_mx_nm = 0.0f;
    g_chassis_attitude_comp.target_my_nm = 0.0f;
    g_chassis_attitude_comp.filtered_mx_nm = 0.0f;
    g_chassis_attitude_comp.filtered_my_nm = 0.0f;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        g_chassis_attitude_comp.foot_z_delta_m[i] = 0.0f;
    }
}

static void apply_attitude_compensation(gait_output_t* output, float dt_s) {
    if (!output) return;
    attitude_comp_clear_debug();

#if !APP_CHASSIS_ATTITUDE_COMP_ENABLE
    (void)dt_s;
    attitude_comp_release_leg_balance();
    return;
#endif

    if (!g_chassis_attitude_comp.enable || !imu_bmi088_is_ready()) {
        attitude_comp_release_leg_balance();
        return;
    }

    const attitude_state_t* attitude = attitude_estimator_get_state();
    if (!attitude) return;

    float roll = attitude->roll;
    float pitch = attitude->pitch;
    float roll_rate = attitude->roll_rate;
    float pitch_rate = attitude->pitch_rate;
    if (!isfinite(roll) || !isfinite(pitch)) {
        attitude_comp_release_leg_balance();
        return;
    }
    if (!isfinite(roll_rate)) roll_rate = 0.0f;
    if (!isfinite(pitch_rate)) pitch_rate = 0.0f;

    g_chassis_attitude_comp.roll_rad = roll;
    g_chassis_attitude_comp.pitch_rad = pitch;
    g_chassis_attitude_comp.roll_rate_rad_s = roll_rate;
    g_chassis_attitude_comp.pitch_rate_rad_s = pitch_rate;

    float deadband = attitude_comp_param_or_default(g_chassis_attitude_comp.deadband_rad,
                                                   ATTITUDE_COMP_DEFAULT_DEADBAND_RAD);
    roll = attitude_comp_apply_deadband(roll, deadband);
    pitch = attitude_comp_apply_deadband(pitch, deadband);

    float roll_kp = attitude_comp_param_or_default(g_chassis_attitude_comp.roll_kp_nm_per_rad,
                                                  ATTITUDE_COMP_DEFAULT_ROLL_KP_NM_PER_RAD);
    float pitch_kp = attitude_comp_param_or_default(g_chassis_attitude_comp.pitch_kp_nm_per_rad,
                                                   ATTITUDE_COMP_DEFAULT_PITCH_KP_NM_PER_RAD);
    float roll_kd = attitude_comp_param_or_default(g_chassis_attitude_comp.roll_kd_nm_per_rads,
                                                  ATTITUDE_COMP_DEFAULT_ROLL_KD_NM_PER_RADS);
    float pitch_kd = attitude_comp_param_or_default(g_chassis_attitude_comp.pitch_kd_nm_per_rads,
                                                   ATTITUDE_COMP_DEFAULT_PITCH_KD_NM_PER_RADS);

    float target_mx = -attitude_comp_scale() * ((roll_kp * roll) + (roll_kd * roll_rate));
    float target_my = -attitude_comp_scale() * ((pitch_kp * pitch) + (pitch_kd * pitch_rate));
    target_mx = attitude_comp_limit_moment(target_mx);
    target_my = attitude_comp_limit_moment(target_my);

    attitude_comp_filter_update(target_mx, target_my, dt_s);
    g_chassis_attitude_comp.target_mx_nm = target_mx;
    g_chassis_attitude_comp.target_my_nm = target_my;
    g_chassis_attitude_comp.filtered_mx_nm = s_attitude_filtered_mx_nm;
    g_chassis_attitude_comp.filtered_my_nm = s_attitude_filtered_my_nm;

    attitude_comp_write_leg_balance(s_attitude_filtered_mx_nm,
                                    s_attitude_filtered_my_nm);
}

static float arm_load_mass_or_default(float value, float fallback) {
    return (isfinite(value) && value > 0.0f) ? value : fallback;
}

static uint8_t arm_load_pose_valid(float x_m, float y_m, float z_m) {
    if (!isfinite(x_m) || !isfinite(y_m) || !isfinite(z_m)) return 0U;
    if (fabsf(x_m) > 1.5f || fabsf(y_m) > 1.5f || fabsf(z_m) > 1.5f) return 0U;
    return 1U;
}

static uint8_t arm_load_select_end_pose(const arm_control_status_t* status,
                                        float* x_m,
                                        float* y_m,
                                        float* z_m,
                                        uint8_t* measured_source) {
    if (!status || !x_m || !y_m || !z_m || !measured_source) return 0U;

    uint8_t measured_valid = arm_load_pose_valid(status->measured_end_x_m,
                                                 status->measured_end_y_m,
                                                 status->measured_end_z_m);
    measured_valid = (status->motor_feedback_fresh || status->feedback_source_measured)
                   ? measured_valid : 0U;
    uint8_t planned_valid = arm_load_pose_valid(status->planned_end_x_m,
                                                status->planned_end_y_m,
                                                status->planned_end_z_m);

    if (g_chassis_arm_load_comp.prefer_measured && measured_valid) {
        *x_m = status->measured_end_x_m;
        *y_m = status->measured_end_y_m;
        *z_m = status->measured_end_z_m;
        *measured_source = 1U;
        return 1U;
    }

    if (planned_valid) {
        *x_m = status->planned_end_x_m;
        *y_m = status->planned_end_y_m;
        *z_m = status->planned_end_z_m;
        *measured_source = 0U;
        return 1U;
    }

    if (measured_valid) {
        *x_m = status->measured_end_x_m;
        *y_m = status->measured_end_y_m;
        *z_m = status->measured_end_z_m;
        *measured_source = 1U;
        return 1U;
    }

    return 0U;
}

static float arm_load_alpha(float dt_s) {
    float tau_s = g_chassis_arm_load_comp.smooth_tau_s;
    if (!isfinite(dt_s) || dt_s <= 0.0f) return 1.0f;
    if (!isfinite(tau_s) || tau_s <= 0.0f) return 1.0f;
    if (dt_s > 0.05f) dt_s = 0.05f;
    return chassis_clampf(dt_s / (tau_s + dt_s), 0.0f, 1.0f);
}

static void arm_load_filter_update(float mass_kg, float com_x_m, float com_y_m, float dt_s) {
    if (!s_arm_load_filter_valid) {
        s_arm_load_filtered_mass_kg = mass_kg;
        s_arm_load_filtered_com_x_m = com_x_m;
        s_arm_load_filtered_com_y_m = com_y_m;
        s_arm_load_filter_valid = 1U;
        return;
    }

    float alpha = arm_load_alpha(dt_s);
    s_arm_load_filtered_mass_kg += alpha * (mass_kg - s_arm_load_filtered_mass_kg);
    s_arm_load_filtered_com_x_m += alpha * (com_x_m - s_arm_load_filtered_com_x_m);
    s_arm_load_filtered_com_y_m += alpha * (com_y_m - s_arm_load_filtered_com_y_m);
}

static void arm_load_write_leg_comp(float mass_kg, float com_x_m, float com_y_m) {
    s_arm_load_was_enabled = 1U;
    g_leg_gravity_comp.enable = g_chassis_arm_load_comp.enable_leg_tau_ff ? 1U : 0U;
    g_leg_gravity_comp.compensate_payload = 1U;
    g_leg_gravity_comp.use_payload_com = 1U;
    g_leg_gravity_comp.payload_mass_kg = mass_kg;
    g_leg_gravity_comp.payload_com_x_m = com_x_m;
    g_leg_gravity_comp.payload_com_y_m = com_y_m;
    g_leg_gravity_comp.support_half_length_m = positive_or_default(
        g_chassis_arm_load_comp.support_half_length_m,
        ATTITUDE_COMP_DEFAULT_HALF_LENGTH_M);
    g_leg_gravity_comp.support_half_track_m = positive_or_default(
        g_chassis_arm_load_comp.support_half_track_m,
        ATTITUDE_COMP_DEFAULT_HALF_TRACK_M);
    g_leg_gravity_comp.max_leg_payload_kg =
        (isfinite(g_chassis_arm_load_comp.max_leg_payload_kg) &&
         g_chassis_arm_load_comp.max_leg_payload_kg > 0.0f)
            ? g_chassis_arm_load_comp.max_leg_payload_kg : 0.0f;
    g_leg_gravity_comp.max_tau_nm = positive_or_default(
        g_chassis_arm_load_comp.max_tau_nm,
        ARM_LOAD_COMP_DEFAULT_MAX_TAU_NM);
    if (!isfinite(g_leg_gravity_comp.scale)) {
        g_leg_gravity_comp.scale = 1.0f;
    }
}

static void arm_load_release_leg_comp(void) {
    if (!s_arm_load_was_enabled) return;

    g_leg_gravity_comp.compensate_payload = 0U;
    g_leg_gravity_comp.use_payload_com = 0U;
    g_leg_gravity_comp.payload_active_mask = 0U;
    g_leg_gravity_comp.payload_mass_kg = 0.0f;
    g_leg_gravity_comp.payload_com_x_m = 0.0f;
    g_leg_gravity_comp.payload_com_y_m = 0.0f;
    g_leg_gravity_comp.payload_applied_mass_kg = 0.0f;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        g_leg_gravity_comp.payload_leg_mass_kg[i] = 0.0f;
    }

    s_arm_load_filter_valid = 0U;
    s_arm_load_was_enabled = 0U;
}

static void update_arm_load_compensation(float dt_s) {
#if !APP_CHASSIS_ARM_LOAD_COMP_ENABLE
    (void)dt_s;
    arm_load_release_leg_comp();
    g_chassis_arm_load_comp.source_valid = 0U;
    g_chassis_arm_load_comp.source_measured = 0U;
    s_arm_load_filter_valid = 0U;
    return;
#endif

    if (!g_chassis_arm_load_comp.enable) {
        arm_load_release_leg_comp();
        g_chassis_arm_load_comp.source_valid = 0U;
        g_chassis_arm_load_comp.source_measured = 0U;
        s_arm_load_filter_valid = 0U;
        return;
    }

    arm_control_status_t arm_status;
    arm_control_get_status(&arm_status);

    float end_x = 0.0f;
    float end_y = 0.0f;
    float end_z = 0.0f;
    uint8_t measured_source = 0U;
    uint8_t source_valid = arm_load_select_end_pose(&arm_status,
                                                    &end_x,
                                                    &end_y,
                                                    &end_z,
                                                    &measured_source);
    g_chassis_arm_load_comp.source_valid = source_valid;
    g_chassis_arm_load_comp.source_measured = measured_source;
    if (!source_valid) {
        arm_load_release_leg_comp();
        return;
    }

    float end_mass = arm_gravity_comp_get_active_end_mass();
    end_mass = arm_load_mass_or_default(end_mass, ARM_GRAVITY_MASS_EE_EMPTY);

    float total_mass = end_mass;
    if (g_chassis_arm_load_comp.include_link_mass) {
        total_mass += arm_load_mass_or_default(g_arm_gc_mass_l2_kg,
                                               ARM_GRAVITY_MASS_L2);
        total_mass += arm_load_mass_or_default(g_arm_gc_mass_l3_kg,
                                               ARM_GRAVITY_MASS_L3);
    }

    float scale = g_chassis_arm_load_comp.scale;
    total_mass *= (isfinite(scale) && scale > 0.0f) ? scale : 0.0f;

    float ratio = g_chassis_arm_load_comp.end_to_com_ratio;
    if (!isfinite(ratio)) ratio = ARM_LOAD_COMP_DEFAULT_END_TO_COM_RATIO;
    ratio = chassis_clampf(ratio, 0.0f, 1.0f);

    float target_com_x = g_chassis_arm_load_comp.arm_mount_x_m + end_x * ratio;
    float target_com_y = g_chassis_arm_load_comp.arm_mount_y_m + end_y * ratio;
    if (!isfinite(target_com_x)) target_com_x = 0.0f;
    if (!isfinite(target_com_y)) target_com_y = 0.0f;

    arm_load_filter_update(total_mass, target_com_x, target_com_y, dt_s);

    g_chassis_arm_load_comp.source_end_x_m = end_x;
    g_chassis_arm_load_comp.source_end_y_m = end_y;
    g_chassis_arm_load_comp.source_end_z_m = end_z;
    g_chassis_arm_load_comp.end_mass_kg = end_mass;
    g_chassis_arm_load_comp.target_total_mass_kg = total_mass;
    g_chassis_arm_load_comp.filtered_total_mass_kg = s_arm_load_filtered_mass_kg;
    g_chassis_arm_load_comp.target_com_x_m = target_com_x;
    g_chassis_arm_load_comp.target_com_y_m = target_com_y;
    g_chassis_arm_load_comp.filtered_com_x_m = s_arm_load_filtered_com_x_m;
    g_chassis_arm_load_comp.filtered_com_y_m = s_arm_load_filtered_com_y_m;

    arm_load_write_leg_comp(s_arm_load_filtered_mass_kg,
                            s_arm_load_filtered_com_x_m,
                            s_arm_load_filtered_com_y_m);
}

#if APP_DEBUG_RL_WHEEL_ONLY
static void apply_rl_single_leg_debug(uint8_t online, const chassis_plan_t* plan, float dt_s) {
    if (online && plan) {
        s_manual_gait_hold = 0U;
        online_decide(plan);
    } else if (!s_manual_gait_hold) {
        if (s_active != ACTIVE_STAND) {
            if (request_gait(s_stand_gait, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) {
                s_active = ACTIVE_STAND;
            }
        }
    }

    gait_output_t output;
    gait_machine_update(&s_gait_machine, dt_s, &output);

    float scale = (online && plan && plan->moving) ? gait_motion_scale() : 0.0f;
    output.wheel_mode = (online && gait_motion_scale() > 0.0f)
                      ? GAIT_WHEEL_DRIVE : GAIT_WHEEL_HOLD;
    output.leg[GAIT_LEG_RL].wheel_rads = (scale > 0.0f)
                                       ? (plan->wheel_rads[GAIT_LEG_RL] * scale)
                                       : 0.0f;
    update_arm_load_compensation(dt_s);
    leg_controller_apply_dt(&s_leg_controller, &output, dt_s);
}
#endif

/* Read IMU yaw when requested and turn target_yaw into an effective yaw-rate command. */
static void update_attitude(float dt_s) {
    imu_bmi088_data_t imu_data;
    if (imu_bmi088_read(&imu_data) == APP_OK) {
        (void)attitude_estimator_update(imu_data.gyro, imu_data.accel, dt_s);
    }
}

static void set_steer_mode_if_changed(steer_mode_t requested) {
    if (requested == s_steer_mode) return;
    s_steer_mode = requested;
    (void)steer_controller_set_mode(requested);
}

static void update_steering(float dt_s,
                            uint8_t online,
                            const chassis_control_command_t* command,
                            float* wz_out) {
    if (!command || !wz_out) return;

    *wz_out = command->wz_rad_s;

    uint8_t imu_ready = imu_bmi088_is_ready() && attitude_estimator_is_calibrated();
    const attitude_state_t* attitude = attitude_estimator_get_state();
    steer_mode_t requested = (command->steer_mode == 1U) ? STEER_MODE_ABSOLUTE
                           : (command->steer_mode == 2U) ? STEER_MODE_AUTO_HOLD
                                                        : STEER_MODE_OFF;

    s_heading_hold_active = 0U;
    if (!online || !imu_ready) {
        s_heading_latch_valid = 0U;
        set_steer_mode_if_changed(STEER_MODE_OFF);
        return;
    }

    if (requested == STEER_MODE_AUTO_HOLD) {
        uint8_t straight = fabsf(command->vx_m_s) > HEADING_HOLD_MIN_VX_M_S &&
                           fabsf(command->wz_rad_s) < HEADING_HOLD_MANUAL_WZ_RAD_S;
        if (!straight) {
            s_heading_latch_valid = 0U;
            set_steer_mode_if_changed(STEER_MODE_OFF);
            return;
        }
        if (!s_heading_latch_valid) {
            s_heading_latched_yaw = attitude->yaw;
            s_heading_latch_valid = 1U;
        }
        set_steer_mode_if_changed(STEER_MODE_AUTO_HOLD);
        (void)steer_controller_set_target_yaw(s_heading_latched_yaw);
        s_heading_hold_active = 1U;
    } else if (requested == STEER_MODE_ABSOLUTE) {
        s_heading_latch_valid = 0U;
        if (steer_controller_set_target_yaw(command->target_yaw_rad) != APP_OK) {
            set_steer_mode_if_changed(STEER_MODE_OFF);
            return;
        }
        set_steer_mode_if_changed(STEER_MODE_ABSOLUTE);
        s_heading_hold_active = 1U;
    } else {
        s_heading_latch_valid = 0U;
        set_steer_mode_if_changed(STEER_MODE_OFF);
        return;
    }

    *wz_out = steer_controller_update(attitude->yaw, attitude->yaw_rate, dt_s);
}

static void update_slip_control(float dt_s) {
    static const motor_logical_id_t IDS[GAIT_LEG_NUM] = {
        MOTOR_ID_FL_WHEEL, MOTOR_ID_FR_WHEEL, MOTOR_ID_RL_WHEEL, MOTOR_ID_RR_WHEEL
    };
    m3508_wheel_diag_t wheel[GAIT_LEG_NUM];
    uint8_t valid = 1U;
    uint8_t current_limited = 0U;
    float commanded_wheel_sum = 0.0f;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (motor_m3508_get_wheel_diag(IDS[i], &wheel[i]) != APP_OK || !wheel[i].online) {
            valid = 0U;
        }
        float target = fabsf(wheel[i].target_velocity_rads);
        commanded_wheel_sum += target;
        float err = fabsf(wheel[i].target_velocity_rads - wheel[i].filtered_velocity_rads);
        if (abs(wheel[i].actual_current_raw) > (int)(0.8f * M3508_CURRENT_RAW_MAX) &&
            target > 0.5f && err > 0.2f * target) {
            current_limited = 1U;
        }
    }

    const attitude_state_t* attitude = attitude_estimator_get_state();
    uint8_t moving = commanded_wheel_sum >= (float)GAIT_LEG_NUM;
    if (valid && moving && attitude_estimator_is_calibrated()) {
        float left = 0.5f * (wheel[GAIT_LEG_FL].filtered_velocity_rads +
                             wheel[GAIT_LEG_RL].filtered_velocity_rads);
        float right = 0.5f * (wheel[GAIT_LEG_FR].filtered_velocity_rads +
                              wheel[GAIT_LEG_RR].filtered_velocity_rads);
        float wheel_wz = (LEG_DIM_DEFAULT.wheel_diameter * 0.5f) * (right - left) /
                         (2.0f * g_chassis_turn_cfg.half_track_m);
        float raw_residual = attitude->yaw_rate - wheel_wz;
        float alpha = dt_s / (SLIP_FILTER_TAU_S + dt_s);
        s_slip_residual += alpha * (raw_residual - s_slip_residual);
    } else {
        s_slip_residual = 0.0f;
        s_slip_warn_elapsed = 0.0f;
        s_slip_active_elapsed = 0.0f;
        s_slip_severe_elapsed = 0.0f;
    }

    float residual = fabsf(s_slip_residual);
    s_slip_warn_elapsed = residual > SLIP_WARN_RAD_S ? s_slip_warn_elapsed + dt_s : 0.0f;
    s_slip_active_elapsed = residual > SLIP_ACTIVE_RAD_S ? s_slip_active_elapsed + dt_s : 0.0f;
    s_slip_severe_elapsed = residual > SLIP_SEVERE_RAD_S ? s_slip_severe_elapsed + dt_s : 0.0f;
    s_slip_recover_elapsed = residual < SLIP_RECOVER_RAD_S ? s_slip_recover_elapsed + dt_s : 0.0f;

    float target_scale = s_speed_scale;
    if (s_slip_severe_elapsed >= SLIP_ACTIVE_HOLD_S) target_scale = 0.4f;
    else if (s_slip_active_elapsed >= SLIP_ACTIVE_HOLD_S || current_limited) target_scale = 0.6f;
    else if (s_slip_recover_elapsed >= SLIP_RECOVER_HOLD_S) target_scale = 1.0f;
    float rate = (target_scale < s_speed_scale) ? SLIP_SCALE_DOWN_RATE_S : SLIP_SCALE_UP_RATE_S;
    s_speed_scale = ramp_step(s_speed_scale, target_scale, rate * dt_s);

    s_diagnostic_flags = 0U;
    if (attitude_estimator_is_calibrated()) s_diagnostic_flags |= CHASSIS_DIAG_IMU_READY;
    if (s_heading_hold_active) s_diagnostic_flags |= CHASSIS_DIAG_HEADING_HOLD;
    if (s_chassis_plan.wheel_saturated) s_diagnostic_flags |= CHASSIS_DIAG_WHEEL_SATURATED;
    if (s_slip_warn_elapsed >= SLIP_WARN_HOLD_S) s_diagnostic_flags |= CHASSIS_DIAG_SLIP_WARNING;
    if (s_slip_active_elapsed >= SLIP_ACTIVE_HOLD_S) s_diagnostic_flags |= CHASSIS_DIAG_SLIP_ACTIVE;
    if (current_limited) s_diagnostic_flags |= CHASSIS_DIAG_CURRENT_LIMITED;
}

/* Offline policy keeps manual hold requests; otherwise falls back to a conservative stand. */
static void offline_decide(uint32_t now_ms) {
#if !APP_OFFLINE_AUTO_MARCH
    (void)now_ms;
    if (s_manual_gait_hold &&
        (s_active == ACTIVE_STAND || s_active == ACTIVE_TROT || s_active == ACTIVE_WALK)) {
        return;
    }
    if (!s_offline_seq_active) {
        s_offline_seq_active = 1U;
        s_offline_seq_done = 1U;
        if (request_gait(s_stand_gait, &GAIT_PARAMS_STAND_DEFAULT, 0.0f) == APP_OK) {
            s_active = ACTIVE_STAND;
        }
        LOGI("offline stand hold");
    } else if (s_active != ACTIVE_STAND && !s_manual_gait_hold) {
        if (request_gait(s_stand_gait, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) {
            s_active = ACTIVE_STAND;
        }
    }
    return;
#else
    if (s_manual_gait_hold &&
        (s_active == ACTIVE_STAND || s_active == ACTIVE_TROT || s_active == ACTIVE_WALK)) {
        return;
    }

    if (!s_offline_seq_active) {
        s_offline_seq_active = 1U;
        s_offline_seq_done = 0U;
        s_offline_seq_start_ms = now_ms;
        if (request_gait(s_stand_gait, &GAIT_PARAMS_STAND_DEFAULT, 0.0f) == APP_OK) {
            s_active = ACTIVE_STAND;
        }
        LOGI("offline sequence start: stand 2s -> march 5s -> stand");
    }

    if (!s_offline_seq_done) {
        uint32_t elapsed_ms = now_ms - s_offline_seq_start_ms;

        if (elapsed_ms < 2000U) {
            if (s_active != ACTIVE_STAND) {
                if (request_gait(s_stand_gait, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) {
                    s_active = ACTIVE_STAND;
                }
            }
            return;
        }

        if (elapsed_ms < 7000U) {
            if (s_active != ACTIVE_TROT) {
                if (request_gait(s_trot_gait, &S_OFFLINE_MARCH_PARAMS, 0.3f) == APP_OK) {
                    s_active = ACTIVE_TROT;
                }
            }
            return;
        }

        if (s_active != ACTIVE_STAND) {
            if (request_gait(s_stand_gait, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) {
                s_active = ACTIVE_STAND;
            }
        }
        s_offline_seq_done = 1U;
        LOGI("offline sequence done: stand");
        return;
    }

    if (s_active == ACTIVE_SCRIPT) {
        if (gait_script_state(s_script_gait) == SP_STATE_DONE) {
            if (request_gait(s_stand_gait, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) {
                s_active = ACTIVE_STAND;
            }
        }
        return;
    }
    if (s_active != ACTIVE_STAND) {
        if (request_gait(s_stand_gait, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) {
            s_active = ACTIVE_STAND;
        }
    }
#endif
}

/* MCU-only GO boot window: collect encoder feedback before allowing closed-loop positions. */
static uint8_t go_pre_calibration_ready(void) {
#if APP_TARGET_HOST
    return 1U;
#else
    static uint32_t s_pre_calib_cnt = 0U;
    static uint8_t  s_pre_calib_done = 0U;

    if (s_pre_calib_done) return 1U;

    motor_go_send_all();
    if (s_pre_calib_cnt < 500U) {
        s_pre_calib_cnt++;
        return 0U;
    }

    (void)motor_go_calibrate_all();
    s_pre_calib_done = 1U;
    LOGI("pre-calib done after %u cycles", (unsigned)s_pre_calib_cnt);
    return 0U;
#endif
}

/* Flush staged motor commands to physical buses; host tests keep this as a no-op. */
static void flush_motor_outputs(void) {
#if !APP_TARGET_HOST
    motor_go_send_all();
    motor_m3508_send_all();
#endif
}

static chassis_control_input_t safe_input_or_zero(const chassis_control_input_t* input) {
    chassis_control_input_t safe_input;
    if (input) {
        safe_input = *input;
    } else {
        memset(&safe_input, 0, sizeof(safe_input));
    }
    return safe_input;
}

static chassis_cmd_plan_t make_plan_command(const chassis_control_input_t* input,
                                            float effective_wz) {
    chassis_cmd_plan_t plan_cmd = {
        .vx_m_s = input->command.vx_m_s * s_speed_scale,
        .vy_m_s = input->command.vy_m_s * s_speed_scale,
        .wz_rad_s = effective_wz * s_speed_scale,
    };
    return plan_cmd;
}

static float slew_limit_axis(float current, float target, float rate_abs, float dt_s) {
    if (!isfinite(target)) target = 0.0f;
    if (!isfinite(current)) current = 0.0f;
    if (!isfinite(rate_abs) || rate_abs <= 0.0f ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return target;
    }
    if (dt_s > 0.05f) dt_s = 0.05f;
    return ramp_step(current, target, rate_abs * dt_s);
}

static chassis_cmd_plan_t slew_plan_command(const chassis_cmd_plan_t* target,
                                            float dt_s) {
    if (!target) {
        memset(&s_slewed_plan_cmd, 0, sizeof(s_slewed_plan_cmd));
        s_slewed_plan_valid = 0U;
        return s_slewed_plan_cmd;
    }

    if (!s_slewed_plan_valid) {
        memset(&s_slewed_plan_cmd, 0, sizeof(s_slewed_plan_cmd));
        s_slewed_plan_valid = 1U;
    }

    s_slewed_plan_cmd.vx_m_s = slew_limit_axis(s_slewed_plan_cmd.vx_m_s,
                                               target->vx_m_s,
                                               CHASSIS_CMD_SLEW_VX_MPS2,
                                               dt_s);
    s_slewed_plan_cmd.vy_m_s = slew_limit_axis(s_slewed_plan_cmd.vy_m_s,
                                               target->vy_m_s,
                                               CHASSIS_CMD_SLEW_VY_MPS2,
                                               dt_s);
    s_slewed_plan_cmd.wz_rad_s = slew_limit_axis(s_slewed_plan_cmd.wz_rad_s,
                                                 target->wz_rad_s,
                                                 CHASSIS_CMD_SLEW_WZ_RADPS2,
                                                 dt_s);
    return s_slewed_plan_cmd;
}

static void update_plan_from_input(const chassis_control_input_t* input,
                                   float effective_wz,
                                   float dt_s) {
    chassis_cmd_plan_t target_cmd = make_plan_command(input, effective_wz);
    chassis_cmd_plan_t plan_cmd = slew_plan_command(&target_cmd, dt_s);
    const gait_params_t* base_gait = chassis_planner_is_low_speed_turn(&plan_cmd)
                                   ? &s_walk_params
                                   : &s_trot_params;
    s_last_effective_wz = plan_cmd.wz_rad_s;
    (void)chassis_planner_update(&plan_cmd, base_gait, &s_chassis_plan);
}

static uint8_t update_online_state(const chassis_control_input_t* input,
                                   uint32_t now_ms) {
    uint8_t online = (uint8_t)!is_offline(input, now_ms);
    s_last_online = online;
    return online;
}

static void decide_gait_for_link_state(uint8_t online, uint32_t now_ms) {
    if (!online) {
        offline_decide(now_ms);
        return;
    }

    s_offline_seq_active = 0U;
    s_offline_seq_done = 0U;
    online_decide(&s_chassis_plan);
}

static void apply_gait_output_to_motors(uint8_t online, float dt_s) {
    gait_output_t gait_output;
    gait_machine_update(&s_gait_machine, dt_s, &gait_output);
    if (online) {
        apply_plan_wheel_speed(&gait_output, &s_chassis_plan);
    }
    apply_attitude_compensation(&gait_output, dt_s);
    update_arm_load_compensation(dt_s);
    leg_controller_apply_dt(&s_leg_controller, &gait_output, dt_s);
}

static void wheel_test_force_stand_state(void) {
    (void)request_gait(s_stand_gait, &GAIT_PARAMS_STAND_DEFAULT, 0.0f);
    apply_controller_height(&GAIT_PARAMS_STAND_DEFAULT);
    s_active = ACTIVE_STAND;
    s_manual_gait_hold = 1U;
}

static uint8_t wheel_test_active_at(uint32_t now_ms) {
    if (!g_chassis_wheel_test.active) return 0U;
    if ((now_ms - g_chassis_wheel_test.last_cmd_ms) <= CHASSIS_WHEEL_TEST_TIMEOUT_MS) {
        return 1U;
    }

    g_chassis_wheel_test.active = 0U;
    g_chassis_wheel_test.wheel_mask = 0U;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        g_chassis_wheel_test.wheel_rads[i] = 0.0f;
    }
    g_chassis_wheel_test.timeout_count++;
    motor_m3508_trace_enable(0U);
    return 0U;
}

static void wheel_test_build_vertical_output(gait_output_t* output, float dt_s) {
    static const float phase_offset[GAIT_LEG_NUM] = {0.0f, 0.5f, 0.5f, 0.0f};
    float period_s = g_chassis_trot_test.period_s;
    float step_height_m = g_chassis_trot_test.step_height_m;
    float duty = g_chassis_trot_test.duty;

    if (!isfinite(period_s) || period_s <= 1e-6f) {
        period_s = CHASSIS_WHEEL_TEST_DEFAULT_PERIOD_S;
    }
    if (!isfinite(step_height_m) || step_height_m < 0.0f) {
        step_height_m = CHASSIS_WHEEL_TEST_DEFAULT_HEIGHT_M;
    }
    if (!isfinite(duty) || duty <= 0.0f || duty >= 1.0f) {
        duty = CHASSIS_WHEEL_TEST_DEFAULT_DUTY;
    }

    if (!isfinite(dt_s) || dt_s < 0.0f) dt_s = 0.0f;
    s_wheel_test_vertical_phase = gait_wrap01(s_wheel_test_vertical_phase + dt_s / period_s);
    output->phase = s_wheel_test_vertical_phase;

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        float dz_m = 0.0f;
        uint8_t in_stance = 1U;
        float leg_phase = gait_wrap01(s_wheel_test_vertical_phase + phase_offset[i]);
        if (leg_phase >= duty) {
            float t = (leg_phase - duty) / (1.0f - duty);
            float one_minus_t = 1.0f - t;
            /* 64*t^3*(1-t)^3: both endpoint velocity and acceleration are zero. */
            dz_m = step_height_m * 64.0f * t * t * t *
                   one_minus_t * one_minus_t * one_minus_t;
            in_stance = 0U;
        }
        dz_m += g_chassis_trot_test.foot_z_trim_m[i];
        output->leg[i].in_stance = in_stance;
        output->leg[i].foot_x_m = 0.0f;
        output->leg[i].foot_z_m = dz_m;
        output->leg[i].hip_rad = 0.0f;
        output->leg[i].knee_rad = dz_m;
        g_chassis_trot_test.target_foot_z_m[i] = dz_m;
    }
    g_chassis_trot_test.phase = s_wheel_test_vertical_phase;
    g_chassis_trot_test.stance_mask = 0U;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (output->leg[i].in_stance) {
            g_chassis_trot_test.stance_mask |= (uint8_t)(1U << i);
        }
    }
}

static void wheel_test_update_joint_errors(const gait_output_t* output) {
    static const motor_logical_id_t hip_ids[GAIT_LEG_NUM] = {
        MOTOR_ID_FL_HIP, MOTOR_ID_FR_HIP, MOTOR_ID_RL_HIP, MOTOR_ID_RR_HIP
    };
    static const motor_logical_id_t knee_ids[GAIT_LEG_NUM] = {
        MOTOR_ID_FL_KNEE, MOTOR_ID_FR_KNEE, MOTOR_ID_RL_KNEE, MOTOR_ID_RR_KNEE
    };
    gait_output_t ik_output;
    memset(&ik_output, 0, sizeof(ik_output));
    leg_ik_solve_all(output, &LEG_DIM_DEFAULT, s_current_stand_height_m, &ik_output);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        motor_dev_t* hip = motor_get(hip_ids[i]);
        motor_dev_t* knee = motor_get(knee_ids[i]);
        g_chassis_trot_test.joint_error_rad[i * 2] =
            ik_output.leg[i].hip_rad - (hip ? hip->state.angle_rad : 0.0f);
        g_chassis_trot_test.joint_error_rad[i * 2 + 1] =
            ik_output.leg[i].knee_rad - (knee ? knee->state.angle_rad : 0.0f);
    }
}

static void apply_direct_wheel_test(float dt_s) {
    static const motor_logical_id_t wheel_ids[GAIT_LEG_NUM] = {
        MOTOR_ID_FL_WHEEL, MOTOR_ID_FR_WHEEL, MOTOR_ID_RL_WHEEL, MOTOR_ID_RR_WHEEL
    };
    gait_output_t output;
    uint8_t mode = g_chassis_wheel_test.active;
    uint8_t coast = (mode == CHASSIS_WHEEL_TEST_MODE_COAST) ? 1U : 0U;
    uint8_t vertical_drive = (mode == CHASSIS_WHEEL_TEST_MODE_VERTICAL_DRIVE) ? 1U : 0U;
    memset(&output, 0, sizeof(output));
    output.wheel_mode = coast ? GAIT_WHEEL_HOLD : GAIT_WHEEL_DRIVE;

    if (vertical_drive) {
        wheel_test_build_vertical_output(&output, dt_s);
    }

    uint8_t mask = (uint8_t)(g_chassis_wheel_test.wheel_mask & 0x0FU);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (!vertical_drive) output.leg[i].in_stance = 1U;
        output.leg[i].wheel_rads = (!coast && (mask & (uint8_t)(1U << i)))
                                 ? g_chassis_wheel_test.wheel_rads[i]
                                 : 0.0f;
    }

    leg_controller_apply_dt(&s_leg_controller, &output, dt_s);
    if (vertical_drive) {
        wheel_test_update_joint_errors(&output);
    }

    if (coast) {
        for (int i = 0; i < GAIT_LEG_NUM; i++) {
            if ((mask & (uint8_t)(1U << i)) == 0U) continue;
            motor_dev_t* wheel = motor_get(wheel_ids[i]);
            if (wheel && wheel->ops && wheel->ops->set_current) {
                (void)wheel->ops->set_current(wheel, 0.0f);
            }
        }

        /* Do not retain a pre-push MIT position while the wheels are current-free. */
        g_leg_wheel_mit.reset = 1U;
    }
}

static uint8_t update_boot_stand(float dt_s) {
    if (s_boot_stand_done) return 1U;

    s_active = ACTIVE_STAND;
    s_last_online = 0U;
    s_last_effective_wz = 0.0f;
    memset(&s_chassis_plan, 0, sizeof(s_chassis_plan));
    s_chassis_plan.gait_params = GAIT_PARAMS_STAND_DEFAULT;

    (void)request_gait(s_stand_gait, &GAIT_PARAMS_STAND_DEFAULT, 0.0f);
    apply_controller_height(&GAIT_PARAMS_STAND_DEFAULT);
    update_stand_height_ramp(dt_s);
    apply_gait_output_to_motors(0U, dt_s);
    flush_motor_outputs();

    if (fabsf(s_current_stand_height_m - s_target_stand_height_m) < 1e-4f) {
        s_boot_stand_done = 1U;
        LOGI("boot stand-up done: height=%.3fm", (double)s_current_stand_height_m);
    }
    return 0U;
}

void chassis_control_init(void) {
    s_stand_gait  = gait_stand_create();
    s_trot_gait   = gait_trot_create();
    s_walk_gait   = gait_walk_create();
    s_script_gait = gait_script_create();

    gait_machine_init(&s_gait_machine);
    gait_machine_set(&s_gait_machine, s_stand_gait, &GAIT_PARAMS_STAND_DEFAULT);

    s_trot_params = GAIT_PARAMS_TROT_DEFAULT;
    s_walk_params = GAIT_PARAMS_WALK_DEFAULT;

    leg_controller_init(&s_leg_controller);
    leg_controller_bind_from_registry(&s_leg_controller);
    memset((void*)&g_chassis_attitude_comp, 0, sizeof(g_chassis_attitude_comp));
    memset((void*)&g_chassis_arm_load_comp, 0, sizeof(g_chassis_arm_load_comp));
    memset((void*)&g_chassis_wheel_test, 0, sizeof(g_chassis_wheel_test));
    memset((void*)&g_chassis_trot_test, 0, sizeof(g_chassis_trot_test));
    g_chassis_trot_test.step_height_m = CHASSIS_WHEEL_TEST_DEFAULT_HEIGHT_M;
    g_chassis_trot_test.period_s = CHASSIS_WHEEL_TEST_DEFAULT_PERIOD_S;
    g_chassis_trot_test.duty = CHASSIS_WHEEL_TEST_DEFAULT_DUTY;
    g_chassis_attitude_comp.stance_only = 1U;
    g_chassis_attitude_comp.scale = 1.0f;
    g_chassis_attitude_comp.half_length_m = ATTITUDE_COMP_DEFAULT_HALF_LENGTH_M;
    g_chassis_attitude_comp.half_track_m = ATTITUDE_COMP_DEFAULT_HALF_TRACK_M;
    g_chassis_attitude_comp.max_foot_z_m = ATTITUDE_COMP_DEFAULT_LIMIT_M;
    g_chassis_attitude_comp.roll_kp_nm_per_rad = ATTITUDE_COMP_DEFAULT_ROLL_KP_NM_PER_RAD;
    g_chassis_attitude_comp.pitch_kp_nm_per_rad = ATTITUDE_COMP_DEFAULT_PITCH_KP_NM_PER_RAD;
    g_chassis_attitude_comp.roll_kd_nm_per_rads = ATTITUDE_COMP_DEFAULT_ROLL_KD_NM_PER_RADS;
    g_chassis_attitude_comp.pitch_kd_nm_per_rads = ATTITUDE_COMP_DEFAULT_PITCH_KD_NM_PER_RADS;
    g_chassis_attitude_comp.deadband_rad = ATTITUDE_COMP_DEFAULT_DEADBAND_RAD;
    g_chassis_attitude_comp.max_moment_nm = ATTITUDE_COMP_DEFAULT_MAX_MOMENT_NM;
    g_chassis_attitude_comp.max_leg_force_n = ATTITUDE_COMP_DEFAULT_MAX_LEG_FORCE_N;
    g_chassis_attitude_comp.smooth_tau_s = ATTITUDE_COMP_DEFAULT_SMOOTH_TAU_S;
    g_chassis_arm_load_comp.enable = 0U;
    g_chassis_arm_load_comp.enable_leg_tau_ff = 1U;
    g_chassis_arm_load_comp.prefer_measured = 1U;
    g_chassis_arm_load_comp.include_link_mass = 1U;
    g_chassis_arm_load_comp.scale = 1.0f;
    g_chassis_arm_load_comp.end_to_com_ratio = ARM_LOAD_COMP_DEFAULT_END_TO_COM_RATIO;
    g_chassis_arm_load_comp.arm_mount_x_m = 0.0f;
    g_chassis_arm_load_comp.arm_mount_y_m = 0.0f;
    g_chassis_arm_load_comp.support_half_length_m = ATTITUDE_COMP_DEFAULT_HALF_LENGTH_M;
    g_chassis_arm_load_comp.support_half_track_m = ATTITUDE_COMP_DEFAULT_HALF_TRACK_M;
    g_chassis_arm_load_comp.smooth_tau_s = ARM_LOAD_COMP_DEFAULT_SMOOTH_TAU_S;
    g_chassis_arm_load_comp.max_leg_payload_kg = ARM_LOAD_COMP_DEFAULT_MAX_LEG_KG;
    g_chassis_arm_load_comp.max_tau_nm = ARM_LOAD_COMP_DEFAULT_MAX_TAU_NM;
#if APP_CHASSIS_COMP_FORCE_DISABLE
    g_chassis_attitude_comp.enable = 0U;
    g_chassis_arm_load_comp.enable = 0U;
    g_chassis_arm_load_comp.enable_leg_tau_ff = 0U;
#endif
    s_attitude_balance_filter_valid = 0U;
    s_attitude_balance_was_enabled = 0U;
    s_attitude_filtered_mx_nm = 0.0f;
    s_attitude_filtered_my_nm = 0.0f;
    s_arm_load_filter_valid = 0U;
    s_arm_load_was_enabled = 0U;
    s_arm_load_filtered_mass_kg = 0.0f;
    s_arm_load_filtered_com_x_m = 0.0f;
    s_arm_load_filtered_com_y_m = 0.0f;
    memset(&s_slewed_plan_cmd, 0, sizeof(s_slewed_plan_cmd));
    s_slewed_plan_valid = 0U;
    s_heading_latch_valid = 0U;
    s_heading_hold_active = 0U;
    s_heading_latched_yaw = 0.0f;
    s_slip_residual = 0.0f;
    s_slip_warn_elapsed = s_slip_active_elapsed = 0.0f;
    s_slip_severe_elapsed = s_slip_recover_elapsed = 0.0f;
    s_speed_scale = 1.0f;
    s_diagnostic_flags = 0U;
#if APP_DEBUG_RL_WHEEL_ONLY
    leg_controller_set_output_options((uint8_t)(1U << GAIT_LEG_RL), 1U, 1U, 1.5f, 0.1f);
    LOGW("debug mode: RL single-leg closed loop; stand locks wheel, vx/wz drives online gait + wheel");
#else
    leg_controller_set_output_options(0x0Fu, 1U, 1U, 1.5f, 0.1f);
#endif
    reset_stand_height_ramp(&GAIT_PARAMS_STAND_DEFAULT);

    s_mode = CHASSIS_MODE_AUTO;
    s_active = ACTIVE_STAND;
    s_online_timeout_ms = 500U;
    s_manual_gait_hold = 0U;
    s_last_effective_wz = 0.0f;
    s_boot_stand_done = 0U;

    attitude_estimator_init();
    steer_controller_init();
    chassis_planner_init();
    s_steer_mode = STEER_MODE_OFF;

    s_offline_seq_active = 0U;
    s_offline_seq_done = 0U;
    s_offline_seq_start_ms = 0U;
    s_last_online = 0U;
    s_wheel_test_was_active = 0U;
    s_wheel_test_last_mode = 0U;
    s_wheel_test_vertical_phase = 0.0f;

    LOGI("chassis init: mode=AUTO active=stand timeout=%ums",
         (unsigned)s_online_timeout_ms);
}

/* One complete 500Hz control tick: input -> steering -> planner -> gait -> IK -> motors. */
void chassis_control_tick(const chassis_control_input_t* input,
                          float dt_s,
                          uint32_t now_ms) {
    chassis_control_input_t safe_input;
    float effective_wz = 0.0f;

    if (!go_pre_calibration_ready()) {
        return;
    }

    update_attitude(dt_s);

    if (!update_boot_stand(dt_s)) {
        return;
    }

    uint8_t wheel_test_active = wheel_test_active_at(now_ms);
    if (wheel_test_active) {
        if (!s_wheel_test_was_active || s_wheel_test_last_mode != wheel_test_active) {
            wheel_test_force_stand_state();
            s_wheel_test_vertical_phase = 0.0f;
            s_wheel_test_was_active = 1U;
            s_wheel_test_last_mode = wheel_test_active;
        }
        update_stand_height_ramp(dt_s);
        apply_direct_wheel_test(dt_s);
        flush_motor_outputs();
        return;
    }
    if (s_wheel_test_was_active) {
        if (s_wheel_test_last_mode == CHASSIS_WHEEL_TEST_MODE_COAST) {
            g_leg_wheel_mit.reset = 1U;
        }
        s_wheel_test_was_active = 0U;
        s_wheel_test_last_mode = 0U;
        s_wheel_test_vertical_phase = 0.0f;
        wheel_test_force_stand_state();
        update_stand_height_ramp(dt_s);
        apply_gait_output_to_motors(0U, dt_s);
        flush_motor_outputs();
        return;
    }

    safe_input = safe_input_or_zero(input);
    uint8_t online = update_online_state(&safe_input, now_ms);
    update_slip_control(dt_s);
    update_steering(dt_s, online, &safe_input.command, &effective_wz);
    update_plan_from_input(&safe_input, effective_wz, dt_s);

#if APP_DEBUG_RL_WHEEL_ONLY
    update_stand_height_ramp(dt_s);
    apply_rl_single_leg_debug(online, &s_chassis_plan, dt_s);
    flush_motor_outputs();
    return;
#endif

    decide_gait_for_link_state(online, now_ms);
    update_stand_height_ramp(dt_s);
    apply_gait_output_to_motors(online, dt_s);
    flush_motor_outputs();
}

void chassis_control_set_mode(chassis_mode_t mode) {
    if (mode == s_mode) return;
    s_mode = mode;
    LOGI("mode -> %d", (int)mode);
}

chassis_mode_t chassis_control_get_mode(void) {
    return s_mode;
}

chassis_gait_active_t chassis_control_get_gait_active(void) {
    switch (s_active) {
        case ACTIVE_TROT:   return CHASSIS_GAIT_TROT;
        case ACTIVE_WALK:   return CHASSIS_GAIT_WALK;
        case ACTIVE_SCRIPT: return CHASSIS_GAIT_SCRIPT;
        case ACTIVE_STAND:
        default:            return CHASSIS_GAIT_STAND;
    }
}

void chassis_control_set_online_timeout_ms(uint32_t timeout_ms) {
    s_online_timeout_ms = timeout_ms;
}

uint32_t chassis_control_get_online_timeout_ms(void) {
    return s_online_timeout_ms;
}

int chassis_control_set_wheel_test(uint8_t enable,
                                   uint8_t wheel_mask,
                                   const float wheel_rads[GAIT_LEG_NUM],
                                   uint32_t now_ms) {
    if (enable > CHASSIS_WHEEL_TEST_MODE_VERTICAL_DRIVE) return APP_ERR_INVALID_ARG;

    if (!enable) {
        g_chassis_wheel_test.active = 0U;
        g_chassis_wheel_test.wheel_mask = 0U;
        for (int i = 0; i < GAIT_LEG_NUM; i++) {
            g_chassis_wheel_test.wheel_rads[i] = 0.0f;
        }
        motor_m3508_trace_enable(0U);
        return APP_OK;
    }

    if (!wheel_rads || (wheel_mask & 0x0FU) == 0U || (wheel_mask & 0xF0U) != 0U) {
        return APP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (!isfinite(wheel_rads[i]) || fabsf(wheel_rads[i]) > CHASSIS_WHEEL_TEST_MAX_RADS) {
            return APP_ERR_INVALID_ARG;
        }
    }

    uint8_t was_active = g_chassis_wheel_test.active;
    if (!was_active) {
        g_chassis_wheel_test.active = 0U;
    }
    g_chassis_wheel_test.wheel_mask = (uint8_t)(wheel_mask & 0x0FU);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        g_chassis_wheel_test.wheel_rads[i] = wheel_rads[i];
    }
    g_chassis_wheel_test.last_cmd_ms = now_ms;
    g_chassis_wheel_test.command_count++;
    if (!was_active) {
        motor_m3508_trace_reset(1U);
        motor_m3508_trace_enable(1U);
    }
    g_chassis_wheel_test.active = enable;
    return APP_OK;
}

int chassis_control_set_trot_test_config(float step_height_m,
                                         float period_s,
                                         float duty,
                                         const float foot_z_trim_m[GAIT_LEG_NUM]) {
    if (!isfinite(step_height_m) || step_height_m < 0.0f ||
        step_height_m > CHASSIS_TROT_TEST_MAX_HEIGHT_M ||
        !isfinite(period_s) || period_s < CHASSIS_TROT_TEST_MIN_PERIOD_S ||
        period_s > CHASSIS_TROT_TEST_MAX_PERIOD_S ||
        !isfinite(duty) || duty < CHASSIS_TROT_TEST_MIN_DUTY ||
        duty > CHASSIS_TROT_TEST_MAX_DUTY || !foot_z_trim_m) {
        return APP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (!isfinite(foot_z_trim_m[i]) ||
            fabsf(foot_z_trim_m[i]) > CHASSIS_TROT_TEST_MAX_TRIM_M) {
            return APP_ERR_INVALID_ARG;
        }
    }

    g_chassis_trot_test.step_height_m = step_height_m;
    g_chassis_trot_test.period_s = period_s;
    g_chassis_trot_test.duty = duty;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        g_chassis_trot_test.foot_z_trim_m[i] = foot_z_trim_m[i];
    }
    return APP_OK;
}

void chassis_control_get_trot_test_debug(chassis_trot_test_debug_t* out) {
    if (!out) return;
    memcpy(out, (const void*)&g_chassis_trot_test, sizeof(*out));
}

int chassis_control_play_script(const script_t* script, float blend_dur_s) {
    if (!script) return APP_ERR_INVALID_ARG;
    s_manual_gait_hold = 0U;
    int ret = gait_script_set_script(s_script_gait, script);
    if (ret != APP_OK) return ret;
    ret = request_gait(s_script_gait, &GAIT_PARAMS_STAND_DEFAULT, blend_dur_s);
    if (ret == APP_OK) s_active = ACTIVE_SCRIPT;
    return ret;
}

int chassis_control_stop_script(float blend_dur_s) {
    return chassis_control_start_stand(blend_dur_s);
}

int chassis_control_start_stand(float blend_dur_s) {
    s_manual_gait_hold = 1U;
    int ret = request_gait(s_stand_gait, &GAIT_PARAMS_STAND_DEFAULT, blend_dur_s);
    if (ret == APP_OK) s_active = ACTIVE_STAND;
    return ret;
}

int chassis_control_set_trot_params(const gait_params_t* params) {
    if (!validate_gait_params(params)) return APP_ERR_INVALID_ARG;
    s_trot_params = *params;
    if (s_active == ACTIVE_TROT && s_gait_machine.current == s_trot_gait) {
        apply_controller_height(&s_trot_params);
        return request_gait(s_trot_gait, &s_trot_params, 0.0f);
    }
    return APP_OK;
}

void chassis_control_get_trot_params(gait_params_t* out) {
    if (!out) return;
    *out = s_trot_params;
}

int chassis_control_start_trot(const gait_params_t* params, float blend_dur_s) {
    if (params) {
        int ret = chassis_control_set_trot_params(params);
        if (ret != APP_OK) return ret;
    }
    apply_controller_height(&s_trot_params);
    int ret = request_gait(s_trot_gait, &s_trot_params, blend_dur_s);
    if (ret == APP_OK) {
        s_active = ACTIVE_TROT;
        s_manual_gait_hold = 1U;
    }
    return ret;
}

int chassis_control_set_walk_params(const gait_params_t* params) {
    if (!validate_gait_params(params)) return APP_ERR_INVALID_ARG;
    s_walk_params = *params;
    if (s_active == ACTIVE_WALK && s_gait_machine.current == s_walk_gait) {
        apply_controller_height(&s_walk_params);
        return request_gait(s_walk_gait, &s_walk_params, 0.0f);
    }
    return APP_OK;
}

void chassis_control_get_walk_params(gait_params_t* out) {
    if (!out) return;
    *out = s_walk_params;
}

int chassis_control_start_walk(const gait_params_t* params, float blend_dur_s) {
    if (params) {
        int ret = chassis_control_set_walk_params(params);
        if (ret != APP_OK) return ret;
    }
    apply_controller_height(&s_walk_params);
    int ret = request_gait(s_walk_gait, &s_walk_params, blend_dur_s);
    if (ret == APP_OK) {
        s_active = ACTIVE_WALK;
        s_manual_gait_hold = 1U;
    }
    return ret;
}

const char* chassis_control_active_gait_name(void) {
    if (!s_gait_machine.current ||
        !s_gait_machine.current->ops ||
        !s_gait_machine.current->ops->name) {
        return "?";
    }
    return s_gait_machine.current->ops->name();
}

void chassis_control_reset_yaw(void) {
    attitude_estimator_reset_yaw();
}

float chassis_control_get_yaw(void) {
    return attitude_estimator_get_yaw();
}

float chassis_control_get_effective_wz(void) {
    return s_last_effective_wz;
}

void chassis_control_get_status(chassis_control_status_t* out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->mode = s_mode;
    out->active_gait = chassis_control_get_gait_active();
    out->online = s_last_online;
    out->moving = s_chassis_plan.moving;
    out->heading_hold_active = s_heading_hold_active;
    out->wheel_saturated = s_chassis_plan.wheel_saturated;
    out->diagnostic_flags = s_diagnostic_flags;
    out->effective_wz_rad_s = s_last_effective_wz;
    const attitude_state_t* attitude = attitude_estimator_get_state();
    out->yaw_rad = attitude->yaw;
    out->gyro_z_rad_s = attitude->yaw_rate;
    out->slip_residual_rad_s = s_slip_residual;
    out->speed_scale = s_speed_scale;
    out->stand_height_m = s_current_stand_height_m;
    out->gait_params = s_chassis_plan.gait_params;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        out->wheel_rads[i] = s_chassis_plan.wheel_rads[i];
    }
}
