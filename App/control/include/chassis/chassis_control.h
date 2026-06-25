/*
 * chassis_control.h - Readable chassis control pipeline.
 *
 * The app task owns RTOS timing and communication plumbing. This module owns
 * the robot behavior: mode selection, steering, gait switching, gait update,
 * IK dispatch, and motor command flushing.
 */
#ifndef APP_CONTROL_CHASSIS_CONTROL_H_
#define APP_CONTROL_CHASSIS_CONTROL_H_

#include <stdint.h>

#include "chassis_types.h"
#include "gait_if.h"
#include "script_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float vx_m_s;
    float vy_m_s;
    float wz_rad_s;
    float target_yaw_rad;
    uint8_t steer_mode; /* 0=raw wz, 1=target_yaw closed loop */
    uint32_t seq;
} chassis_control_command_t;

typedef struct {
    chassis_control_command_t command;
    uint32_t valid_frame_count;
    uint32_t last_rx_ms;
} chassis_control_input_t;

typedef struct {
    chassis_mode_t mode;
    chassis_gait_active_t active_gait;
    uint8_t online;
    uint8_t moving;
    float effective_wz_rad_s;
    float wheel_rads[GAIT_LEG_NUM];
    gait_params_t gait_params;
} chassis_control_status_t;

void chassis_control_init(void);
void chassis_control_tick(const chassis_control_input_t* input,
                          float dt_s,
                          uint32_t now_ms);

void chassis_control_set_mode(chassis_mode_t mode);
chassis_mode_t chassis_control_get_mode(void);
chassis_gait_active_t chassis_control_get_gait_active(void);

int chassis_control_play_script(const script_t* script, float blend_dur_s);
int chassis_control_stop_script(float blend_dur_s);
int chassis_control_start_stand(float blend_dur_s);
int chassis_control_set_trot_params(const gait_params_t* params);
void chassis_control_get_trot_params(gait_params_t* out);
int chassis_control_start_trot(const gait_params_t* params, float blend_dur_s);

void chassis_control_set_online_timeout_ms(uint32_t timeout_ms);
uint32_t chassis_control_get_online_timeout_ms(void);

const char* chassis_control_active_gait_name(void);

void chassis_control_reset_yaw(void);
float chassis_control_get_yaw(void);
float chassis_control_get_effective_wz(void);
void chassis_control_get_status(chassis_control_status_t* out);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_CHASSIS_CONTROL_H_ */
