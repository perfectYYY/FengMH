/*
 * task_chassis.c - RTOS-facing chassis task wrapper.
 *
 * The behavior pipeline lives in control/chassis/chassis_control.c. This file
 * only adapts app-level inputs (task_comm, safety, RTOS timing) to that module
 * and preserves the historical task_chassis public API used by task_comm.
 */
#include "task_chassis.h"

#include "bsp_time.h"
#include "chassis_control.h"
#include "config.h"
#include "log.h"
#include "proto_defs.h"
#include "task_comm.h"
#include "task_safety.h"

#if APP_TARGET_MCU
#include "cmsis_os.h"
#endif

static const char* TAG = "CHASSIS_TASK";

static uint8_t chassis_mode_allows_motion(void) {
    task_comm_mode_cmd_t mode;
    task_comm_get_mode_cmd(&mode);
    if (mode.seq == 0U) {
        return 1U;  /* Legacy host path before the unified mode manager starts. */
    }
    return (mode.mode == PROTO_ROBOT_MODE_NAV) ? 1U : 0U;
}

static void read_chassis_input(chassis_control_input_t* input) {
    if (!input) return;

    task_comm_chassis_cmd_t command;
    task_comm_get_chassis(&command);

    const uint8_t allow_motion = chassis_mode_allows_motion();
    input->command.vx_m_s = allow_motion ? command.vx : 0.0f;
    input->command.vy_m_s = allow_motion ? command.vy : 0.0f;
    input->command.wz_rad_s = allow_motion ? command.wz : 0.0f;
    input->command.target_yaw_rad = allow_motion ? command.target_yaw : 0.0f;
    input->command.steer_mode = allow_motion ? command.steer_mode : 0U;
    input->command.seq = command.seq;
    input->valid_frame_count = task_comm_dispatch_hit();
    input->last_rx_ms = task_comm_last_rx_ms();
}

void task_chassis_init(void) {
    chassis_control_init();
}

void task_chassis_set_mode(chassis_mode_t mode) {
    chassis_control_set_mode(mode);
}

chassis_mode_t task_chassis_get_mode(void) {
    return chassis_control_get_mode();
}

chassis_gait_active_t task_chassis_get_gait_active(void) {
    return chassis_control_get_gait_active();
}

void task_chassis_set_online_timeout_ms(uint32_t timeout_ms) {
    chassis_control_set_online_timeout_ms(timeout_ms);
}

uint32_t task_chassis_get_online_timeout_ms(void) {
    return chassis_control_get_online_timeout_ms();
}

int task_chassis_set_wheel_test(uint8_t enable,
                                uint8_t wheel_mask,
                                const float wheel_rads[GAIT_LEG_NUM],
                                uint32_t now_ms) {
    return chassis_control_set_wheel_test(enable, wheel_mask, wheel_rads, now_ms);
}

int task_chassis_play_script(const script_t* script, float blend_dur_s) {
    return chassis_control_play_script(script, blend_dur_s);
}

int task_chassis_stop_script(float blend_dur_s) {
    return chassis_control_stop_script(blend_dur_s);
}

int task_chassis_start_stand(float blend_dur_s) {
    return chassis_control_start_stand(blend_dur_s);
}

int task_chassis_set_trot_params(const gait_params_t* params) {
    return chassis_control_set_trot_params(params);
}

void task_chassis_get_trot_params(gait_params_t* out) {
    chassis_control_get_trot_params(out);
}

int task_chassis_start_trot(const gait_params_t* params, float blend_dur_s) {
    return chassis_control_start_trot(params, blend_dur_s);
}

int task_chassis_set_walk_params(const gait_params_t* params) {
    return chassis_control_set_walk_params(params);
}

void task_chassis_get_walk_params(gait_params_t* out) {
    chassis_control_get_walk_params(out);
}

int task_chassis_start_walk(const gait_params_t* params, float blend_dur_s) {
    return chassis_control_start_walk(params, blend_dur_s);
}

const char* task_chassis_active_gait_name(void) {
    return chassis_control_active_gait_name();
}

void task_chassis_reset_yaw(void) {
    chassis_control_reset_yaw();
}

float task_chassis_get_yaw(void) {
    return chassis_control_get_yaw();
}

float task_chassis_get_effective_wz(void) {
    return chassis_control_get_effective_wz();
}

void task_chassis_step_for_test(float dt_s, uint32_t now_ms) {
    chassis_control_input_t input;
    read_chassis_input(&input);
    chassis_control_tick(&input, dt_s, now_ms);
}

void task_chassis_entry(void* arg) {
    (void)arg;
    LOGI("task_chassis started");
#if APP_TARGET_MCU
    const float dt = 0.002f; /* 500Hz */
    for (;;) {
        if (task_safety_estop_active()) {
            osDelay(20);
            continue;
        }
        uint32_t now = (uint32_t)bsp_time_now_ms();
        task_chassis_step_for_test(dt, now);
        osDelay(2);
    }
#endif
}
