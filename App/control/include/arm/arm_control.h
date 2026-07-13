/*
 * arm_control.h - Arm control facade used by task_arm.
 *
 * Migration facade: it accepts commands, runs the arm kinematics/trajectory
 * planner, reports command feedback, and drives the Damiao output path when
 * APP_ARM_MOTOR_OUTPUT_DEFAULT_ENABLE/runtime output gate allows it.
 */
#ifndef APP_CONTROL_ARM_CONTROL_H_
#define APP_CONTROL_ARM_CONTROL_H_

#include "arm_kinematics.h"
#include "arm_motion.h"
#include "err.h"
#include "proto_defs.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 仅供本地待机姿态使用，不能出现在上位机协议中。 */
#define ARM_CONTROL_TARGET_INTERNAL_HOLD 0xFEu

extern volatile float debug_arm_host_target_j1_motor_deg;
extern volatile float debug_arm_selected_target_j1_motor_deg;
extern volatile uint8_t debug_arm_rear_place_avoidance_enabled;
extern volatile uint32_t debug_arm_grasp_j1_reject_count;

typedef struct {
    uint8_t enabled;
    uint8_t target_valid;
    uint8_t target_type;
    uint8_t pump_on;
    float target_x_m;
    float target_y_m;
    float target_z_m;
    uint32_t target_seq;
    uint32_t pump_seq;
    uint32_t last_tick_ms;
    uint8_t target_pending;
    uint8_t trajectory_state;
    int32_t last_result;
    float planned_end_x_m;
    float planned_end_y_m;
    float planned_end_z_m;
    float planned_theta1_rad;
    float motion_duration_s;
    float motion_progress;
    float gravity_tau2_nm;
    float gravity_tau3_nm;
    float gravity_tau4_nm;
    uint8_t motor_output_enabled;
    uint8_t motor_bound_mask;
    uint8_t motor_online_mask;
    uint8_t motor_feedback_fresh;
    uint8_t feedback_source_measured;
    uint8_t motor_tx_fail_mask;
    uint32_t motor_tx_fail_count;
    float measured_end_x_m;
    float measured_end_y_m;
    float measured_end_z_m;
    float measured_theta1_rad;
    uint8_t safe_move_stage;
    uint8_t reached;
    float settle_error_rad;
    float settle_speed_rads;
    uint32_t safe_move_cycle_count;
    uint8_t fine_tracking_active;
} arm_control_status_t;

app_err_t arm_control_init(void);
app_err_t arm_control_set_enabled(uint8_t enabled);
app_err_t arm_control_set_motor_output_enabled(uint8_t enabled);
app_err_t arm_control_set_rear_place_avoidance(uint8_t enabled);
/* 目标保持期间让 J1 使用重补式零刚度/轻阻尼，可被外力自由转动。 */
app_err_t arm_control_set_j1_free_mode(uint8_t enabled);
app_err_t arm_control_set_gravity_mode(void);
/* 上位机目标预检：GRASP 额外应用周期性的 J1 禁区，PLACE 只做常规 IK 检查。 */
app_err_t arm_control_validate_host_target(uint8_t target_type,
                                           float x_m,
                                           float y_m,
                                           float z_m);
uint8_t arm_control_grasp_j1_angle_forbidden(float theta1_motor_rad);
app_err_t arm_control_set_target(uint8_t target_type, float x_m, float y_m, float z_m);
app_err_t arm_control_set_joint_target(uint8_t target_type,
                                       const arm_joint_angles_t* target);
app_err_t arm_control_set_pump(uint8_t pump_on);
app_err_t arm_control_tick(float dt_s, uint32_t now_ms);
void arm_control_get_feedback(payload_arm_feedback_t* out);
void arm_control_get_status(arm_control_status_t* out);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_ARM_CONTROL_H_ */
