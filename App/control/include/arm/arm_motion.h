/*
 * arm_motion.h - Synchronized quintic joint trajectory generator.
 *
 * Ported from damiao_new1/Core/Src/arm_motion.c.
 */
#ifndef APP_CONTROL_ARM_MOTION_H_
#define APP_CONTROL_ARM_MOTION_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_MOTION_JOINT_COUNT 4U

typedef enum {
    ARM_TRAJECTORY_IDLE = 0,
    ARM_TRAJECTORY_MOVING,
    ARM_TRAJECTORY_FINISHED,
} arm_trajectory_state_t;

typedef struct {
    float max_velocity[ARM_MOTION_JOINT_COUNT];
    float max_acceleration[ARM_MOTION_JOINT_COUNT];
    float min_duration_s;
    float max_duration_s;
} arm_motion_limits_t;

typedef struct {
    float position[ARM_MOTION_JOINT_COUNT];
    float velocity[ARM_MOTION_JOINT_COUNT];
    float acceleration[ARM_MOTION_JOINT_COUNT];
} arm_motion_sample_t;

void arm_motion_init(const arm_motion_limits_t* limits);
void arm_motion_set_limits(const arm_motion_limits_t* limits);
int arm_motion_start(const arm_motion_sample_t* start,
                     const float target[ARM_MOTION_JOINT_COUNT],
                     uint32_t now_ms);
arm_trajectory_state_t arm_motion_update(uint32_t now_ms,
                                         arm_motion_sample_t* sample);
void arm_motion_stop(const arm_motion_sample_t* hold_sample);
arm_trajectory_state_t arm_motion_get_state(void);
void arm_motion_get_sample(arm_motion_sample_t* sample);
float arm_motion_get_duration(void);
float arm_motion_get_progress(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_ARM_MOTION_H_ */
