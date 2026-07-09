/*
 * task_arm.h - FreeRTOS wrapper for the arm-control pipeline.
 */
#ifndef APP_TASK_ARM_H_
#define APP_TASK_ARM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void task_arm_init(void);
void task_arm_entry(void* arg);
void task_arm_step_for_test(float dt_s, uint32_t now_ms);

typedef enum {
    ARM_FIXED_WAIT_FEEDBACK = 0,
    ARM_FIXED_MOVING = 1,
    ARM_FIXED_HOLDING = 2,
    ARM_FIXED_ERROR = 3,
    ARM_FIXED_RELEASED_TO_HOST = 4,
    ARM_FIXED_GRAVITY_ONLY = 5,
} task_arm_fixed_state_t;

typedef struct {
    uint32_t update_sequence;
    uint32_t update_time_ms;

    float motor1_position_deg;
    float motor2_position_deg;
    float motor3_position_deg;
    float motor4_position_deg;

    uint32_t motor1_response_age_ms;
    uint32_t motor2_response_age_ms;
    uint32_t motor3_response_age_ms;
    uint32_t motor4_response_age_ms;

    float host_target_x_m;
    float host_target_y_m;
    float host_target_z_m;

    float current_end_x_m;
    float current_end_y_m;
    float current_end_z_m;
} task_arm_debug_snapshot_t;

/* Live Expressions only: 0=normal, non-zero=cancel all arm actions and force gravity only. */
extern volatile uint8_t debug_arm_force_gravity_only;
/* Live Expressions: expand this single 100 ms snapshot instead of polling many symbols. */
extern volatile task_arm_debug_snapshot_t g_arm_debug_snapshot;
extern volatile uint8_t debug_arm_fixed_state;
extern volatile uint8_t debug_arm_fixed_profile;
/* Live Expressions: 1=first box J1 wait angle, 2=second box J1 wait angle. */
extern volatile uint8_t debug_arm_box_position_select;
extern volatile float debug_arm_fixed_j2_motor_deg;
extern volatile float debug_arm_fixed_j3_motor_deg;
extern volatile float debug_arm_fixed_j4_motor_deg;
extern volatile int32_t debug_arm_fixed_last_result;

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_ARM_H_ */
