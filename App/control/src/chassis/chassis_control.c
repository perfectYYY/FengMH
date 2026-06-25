/*
 * chassis_control.c - Chassis behavior pipeline.
 *
 * This file is the first pass at making the control path readable. It keeps
 * the existing behavior from the legacy task_chassis implementation, but moves
 * it out of the RTOS task wrapper and into a control module with explicit
 * inputs.
 */
#include "chassis_control.h"

#include "attitude_estimator.h"
#include "bsp_time.h"
#include "chassis_planner.h"
#include "config.h"
#include "err.h"
#include "gait_machine.h"
#include "gait_params.h"
#include "gait_script.h"
#include "gait_stand.h"
#include "gait_trot.h"
#include "imu_bmi088.h"
#include "leg_controller.h"
#include "log.h"
#include "motor_go.h"
#include "motor_m3508.h"
#include "steer_controller.h"

#include <math.h>
#include <string.h>

static const char* TAG = "CHASSIS";

typedef enum {
    ACTIVE_STAND = 0,
    ACTIVE_TROT,
    ACTIVE_SCRIPT,
} active_gait_t;

static gait_machine_t   s_gait_machine;
static leg_controller_t s_leg_controller;
static gait_if_t*       s_stand_gait;
static gait_if_t*       s_trot_gait;
static gait_if_t*       s_script_gait;

static chassis_mode_t s_mode = CHASSIS_MODE_AUTO;
static active_gait_t  s_active = ACTIVE_STAND;
static uint32_t       s_online_timeout_ms = 500U;
static gait_params_t  s_trot_params;
static uint8_t        s_manual_gait_hold = 0U;
static float          s_last_effective_wz = 0.0f;

static uint8_t  s_offline_seq_active = 0U;
static uint8_t  s_offline_seq_done = 0U;
static uint32_t s_offline_seq_start_ms = 0U;
static steer_mode_t s_steer_mode = STEER_MODE_OFF;
static chassis_plan_t s_chassis_plan;
static uint8_t s_last_online = 0U;

static const gait_params_t S_OFFLINE_MARCH_PARAMS = {
    .body_height_m    = 0.20f,
    .step_length_m    = 0.0f,
    .turn_step_m      = 0.0f,
    .step_height_m    = 0.015f,
    .period_s         = 1.0f,
    .duty             = 0.50f,
    .phase_offset     = { 0.0f, 0.5f, 0.5f, 0.0f },
    .touchdown_thresh = 0.0f,
};

/* Convert user-facing body height to the IK convention: feet below hip are negative z. */
static float controller_height_from_body(float body_height_m) {
    if (!isfinite(body_height_m) || body_height_m == 0.0f) {
        return -0.18f;
    }
    return (body_height_m > 0.0f) ? -body_height_m : body_height_m;
}

/* Keep leg_controller's IK height synchronized with the gait currently being requested. */
static void apply_controller_height(const gait_params_t* params) {
    if (!params) return;
    leg_controller_set_stand_height(controller_height_from_body(params->body_height_m));
}

/* Validate host-provided trot parameters before they enter the gait machine. */
static int validate_trot_params(const gait_params_t* params) {
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

/* Online policy: moving plan -> trot, still plan -> stand. */
static void online_decide(const chassis_plan_t* plan) {
    if (!plan) return;

    if (plan->moving) {
        if (s_active != ACTIVE_TROT) {
            apply_controller_height(&plan->gait_params);
            if (request_gait(s_trot_gait, &plan->gait_params, 0.3f) == APP_OK) {
                s_active = ACTIVE_TROT;
                s_manual_gait_hold = 0U;
            }
        } else if (s_trot_gait && s_trot_gait->ops && s_trot_gait->ops->set_param) {
            apply_controller_height(&plan->gait_params);
            (void)s_trot_gait->ops->set_param(s_trot_gait, &plan->gait_params);
        }
        return;
    }

    if (s_active != ACTIVE_STAND) {
        if (chassis_control_start_stand(0.3f) == APP_OK) {
            s_active = ACTIVE_STAND;
        }
    }
}

/* Wheels are active only while their leg is in stance; swing legs command zero wheel speed. */
static void apply_plan_wheel_speed(gait_output_t* output, const chassis_plan_t* plan) {
    if (!output || !plan) return;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        output->leg[i].wheel_rads = output->leg[i].in_stance ? plan->wheel_rads[i] : 0.0f;
    }
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

    output.leg[GAIT_LEG_RL].wheel_rads = (online && plan) ? plan->wheel_rads[GAIT_LEG_RL] : 0.0f;
    leg_controller_apply_dt(&s_leg_controller, &output, dt_s);
}
#endif

/* Read IMU yaw when requested and turn target_yaw into an effective yaw-rate command. */
static void update_steering(float dt_s,
                            const chassis_control_command_t* command,
                            float* wz_out) {
    if (!command || !wz_out) return;

    *wz_out = command->wz_rad_s;

    imu_bmi088_data_t imu_data;
    if (imu_bmi088_read(&imu_data) == APP_OK) {
        attitude_estimator_update(imu_data.gyro, imu_data.accel, dt_s);
    }

    steer_mode_t requested = (command->steer_mode == 1U) ? STEER_MODE_YAW : STEER_MODE_OFF;
    if (requested != s_steer_mode) {
        s_steer_mode = requested;
        (void)steer_controller_set_mode(requested);
    }

    if (s_steer_mode == STEER_MODE_YAW && imu_bmi088_is_ready()) {
        (void)steer_controller_set_target_yaw(command->target_yaw_rad);
        *wz_out = steer_controller_update(attitude_estimator_get_yaw(), dt_s);
    }
}

/* Offline policy keeps manual hold requests; otherwise falls back to a conservative stand. */
static void offline_decide(uint32_t now_ms) {
#if !APP_OFFLINE_AUTO_MARCH
    (void)now_ms;
    if (s_manual_gait_hold && (s_active == ACTIVE_STAND || s_active == ACTIVE_TROT)) {
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
    if (s_manual_gait_hold && (s_active == ACTIVE_STAND || s_active == ACTIVE_TROT)) {
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
        .vx_m_s = input->command.vx_m_s,
        .vy_m_s = input->command.vy_m_s,
        .wz_rad_s = effective_wz,
    };
    return plan_cmd;
}

static void update_plan_from_input(const chassis_control_input_t* input,
                                   float effective_wz) {
    chassis_cmd_plan_t plan_cmd = make_plan_command(input, effective_wz);
    (void)chassis_planner_update(&plan_cmd, &s_trot_params, &s_chassis_plan);
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
    leg_controller_apply_dt(&s_leg_controller, &gait_output, dt_s);
}

void chassis_control_init(void) {
    s_stand_gait  = gait_stand_create();
    s_trot_gait   = gait_trot_create();
    s_script_gait = gait_script_create();

    gait_machine_init(&s_gait_machine);
    gait_machine_set(&s_gait_machine, s_stand_gait, &GAIT_PARAMS_STAND_DEFAULT);

    s_trot_params = GAIT_PARAMS_TROT_DEFAULT;

    leg_controller_init(&s_leg_controller);
    leg_controller_bind_from_registry(&s_leg_controller);
#if APP_DEBUG_RL_WHEEL_ONLY
    leg_controller_set_output_options((uint8_t)(1U << GAIT_LEG_RL), 1U, 1U, 1.5f, 0.1f);
    apply_controller_height(&GAIT_PARAMS_STAND_DEFAULT);
    LOGW("debug mode: RL single-leg closed loop; stand locks wheel, vx/wz drives RL trot + wheel");
#else
    leg_controller_set_output_options(0x0Fu, 1U, 1U, 1.5f, 0.1f);
    apply_controller_height(&GAIT_PARAMS_STAND_DEFAULT);
#endif

    s_mode = CHASSIS_MODE_AUTO;
    s_active = ACTIVE_STAND;
    s_online_timeout_ms = 500U;
    s_manual_gait_hold = 0U;
    s_last_effective_wz = 0.0f;

    attitude_estimator_init();
    steer_controller_init();
    chassis_planner_init();
    s_steer_mode = STEER_MODE_OFF;

    s_offline_seq_active = 0U;
    s_offline_seq_done = 0U;
    s_offline_seq_start_ms = 0U;
    s_last_online = 0U;

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

    safe_input = safe_input_or_zero(input);

    update_steering(dt_s, &safe_input.command, &effective_wz);
    s_last_effective_wz = effective_wz;

    update_plan_from_input(&safe_input, effective_wz);
    uint8_t online = update_online_state(&safe_input, now_ms);

#if APP_DEBUG_RL_WHEEL_ONLY
    apply_rl_single_leg_debug(online, &s_chassis_plan, dt_s);
    flush_motor_outputs();
    return;
#endif

    decide_gait_for_link_state(online, now_ms);
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
    if (!validate_trot_params(params)) return APP_ERR_INVALID_ARG;
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
    out->effective_wz_rad_s = s_last_effective_wz;
    out->gait_params = s_chassis_plan.gait_params;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        out->wheel_rads[i] = s_chassis_plan.wheel_rads[i];
    }
}
