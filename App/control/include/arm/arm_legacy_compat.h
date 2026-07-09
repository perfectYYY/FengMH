/*
 * arm_legacy_compat.h - Compatibility API for damiao_new1 arm code.
 *
 * This header keeps the old CamelCase types/functions available for future
 * arm-only development while the integrated firmware uses the new snake_case
 * control path internally.
 */
#ifndef APP_CONTROL_ARM_LEGACY_COMPAT_H_
#define APP_CONTROL_ARM_LEGACY_COMPAT_H_

#include <stdint.h>
#include <math.h>

#include "arm_motion.h"
#include "config.h"

#if APP_TARGET_HOST
typedef struct FDCAN_HandleTypeDef FDCAN_HandleTypeDef;
#else
#include "stm32h7xx_hal.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PI
#define PI 3.1415926535f
#endif
#ifndef RAD2DEG
#define RAD2DEG(rad) ((rad) * 180.0f / PI)
#endif
#ifndef DEG2RAD
#define DEG2RAD(deg) ((deg) * PI / 180.0f)
#endif

#define ARM_MOTOR2_DIR              (1.0f)
#define ARM_MOTOR2_TO_PHYSICAL(x)   (ARM_MOTOR2_DIR * (x))
#define ARM_MOTOR2_TO_LOGICAL(x)    (ARM_MOTOR2_DIR * (x))
#define ARM_MOTOR3_DIR              (-1.0f)
#define ARM_MOTOR3_TO_PHYSICAL(x)   (ARM_MOTOR3_DIR * (x))
#define ARM_MOTOR3_TO_LOGICAL(x)    (ARM_MOTOR3_DIR * (x))

#define ARM_MOTOR2_PHYSICAL_MIN_DEG (-88.0f)
#define ARM_MOTOR2_PHYSICAL_MAX_DEG (90.0f)
#define ARM_MOTOR3_PHYSICAL_MIN_DEG (-40.0f)
#define ARM_MOTOR3_PHYSICAL_MAX_DEG (110.0f)

typedef struct {
    float L2;
    float L3;
} Arm_Params_t;

typedef struct {
    float theta1_geo;
    float theta2_geo;
    float theta3_geo;
    float theta4_geo;
    float theta1_motor;
    float theta2_motor;
    float theta3_motor;
    float theta4_motor;
} Arm_Joint_Angles_t;

typedef struct {
    float x;
    float y;
    float z;
    float pitch;
} Arm_Pose_t;

typedef struct {
    float theta1_offset_deg;
    float theta2_offset_deg;
    float theta3_offset_deg;
    float theta4_offset_deg;
    float theta3_theta4_sum_deg;
} Arm_Offset_Config_t;

typedef enum {
    ARM_CONTROL_GRAVITY = 0,
    ARM_CONTROL_POSITION_HOLD,
} Arm_Control_Mode_t;

typedef enum {
    ARM_MOVE_IDLE = 0,
    ARM_MOVE_MOVING,
    ARM_MOVE_SETTLING,
    ARM_MOVE_REACHED,
    ARM_MOVE_ERROR_INVALID_TARGET,
    ARM_MOVE_ERROR_NO_FEEDBACK,
    ARM_MOVE_ERROR_TIMEOUT,
} Arm_Move_Status_t;

typedef arm_trajectory_state_t Arm_Trajectory_State_t;
typedef arm_motion_limits_t Arm_Motion_Limits_t;
typedef arm_motion_sample_t Arm_Motion_Sample_t;

typedef enum {
    GRAVITY_PAYLOAD_EMPTY = 0,
    GRAVITY_PAYLOAD_LOADED = 1,
} Gravity_Payload_State_t;

extern Arm_Params_t arm_params;
extern Arm_Offset_Config_t arm_offset;

extern volatile uint8_t debug_payload_loaded;
extern volatile float dbg_gravity_scale_all;
extern volatile float dbg_gravity_scale_2;
extern volatile float dbg_gravity_scale_3;
extern volatile float dbg_gravity_scale_4;
extern volatile float debug_gravity_tau2;
extern volatile float debug_gravity_tau3;
extern volatile float debug_gravity_tau4;

extern volatile uint8_t debug_motion_status;
extern volatile int32_t debug_motion_last_result;
extern volatile float debug_motion_duration_s;
extern volatile float debug_motion_progress;
extern volatile float debug_motion_max_error_deg;
extern volatile uint32_t debug_feedback_age1_ms;
extern volatile uint32_t debug_feedback_age2_ms;
extern volatile uint32_t debug_feedback_age3_ms;
extern volatile uint32_t debug_feedback_age4_ms;
extern volatile uint8_t debug_motor_tx_fail_mask;
extern volatile uint32_t debug_motor_tx_fail_count;
extern volatile uint8_t debug_safe_move_stage;
extern volatile uint32_t debug_safe_move_cycle_count;
extern volatile uint8_t debug_fine_tracking;

extern volatile float debug_motion_vmax_1;
extern volatile float debug_motion_vmax_2;
extern volatile float debug_motion_vmax_3;
extern volatile float debug_motion_vmax_4;
extern volatile float debug_motion_amax_1;
extern volatile float debug_motion_amax_2;
extern volatile float debug_motion_amax_3;
extern volatile float debug_motion_amax_4;
extern volatile float debug_move_kp_4340;
extern volatile float debug_move_kd_4340;
extern volatile float debug_move_kp_4310;
extern volatile float debug_move_kd_4310;
extern volatile float debug_hold_kp_4340;
extern volatile float debug_hold_kd_4340;
extern volatile float debug_hold_kp_4310;
extern volatile float debug_hold_kd_4310;
extern volatile float debug_gravity_kp_4;
extern volatile float debug_gravity_kd_4;

extern volatile float dbg_gc_mass_l2;
extern volatile float dbg_gc_mass_l3;
extern volatile float dbg_gc_mass_ee;
extern volatile float dbg_gc_mass_cargo;
extern volatile float dbg_gc_com_r2_x;
extern volatile float dbg_gc_com_r2_y;
extern volatile float dbg_gc_com_r3_x;
extern volatile float dbg_gc_com_r3_y;
extern volatile float dbg_gc_com_ee_x;

void Arm_Forward_Kinematics(const Arm_Params_t* params,
                            const Arm_Joint_Angles_t* angles,
                            Arm_Pose_t* pose);
int Arm_Inverse_Kinematics(const Arm_Params_t* params,
                           const Arm_Offset_Config_t* offset,
                           const Arm_Pose_t* target,
                           Arm_Joint_Angles_t* angles);
float Arm_Compute_T4_From_T3(const Arm_Offset_Config_t* offset,
                             float t3_motor);

void Arm_Motion_Init(const Arm_Motion_Limits_t* limits);
void Arm_Motion_SetLimits(const Arm_Motion_Limits_t* limits);
int Arm_Motion_Start(const Arm_Motion_Sample_t* start,
                     const float target[ARM_MOTION_JOINT_COUNT],
                     uint32_t now_ms);
Arm_Trajectory_State_t Arm_Motion_Update(uint32_t now_ms,
                                         Arm_Motion_Sample_t* sample);
void Arm_Motion_Stop(const Arm_Motion_Sample_t* hold_sample);
Arm_Trajectory_State_t Arm_Motion_GetState(void);
void Arm_Motion_GetSample(Arm_Motion_Sample_t* sample);
float Arm_Motion_GetDuration(void);
float Arm_Motion_GetProgress(uint32_t now_ms);

void Gravity_Comp_Init(void);
void Gravity_Comp_SetPayloadState(Gravity_Payload_State_t state);
Gravity_Payload_State_t Gravity_Comp_GetPayloadState(void);
float Gravity_Comp_GetActiveEndMass(void);
float Gravity_Comp_GetTargetEndMass(void);
void Calculate_Gravity_Compensation(float theta2_rad,
                                    float theta3_rad,
                                    float theta4_rad,
                                    float* tau2,
                                    float* tau3,
                                    float* tau4);

void Arm_Control_Init(FDCAN_HandleTypeDef* hfdcan);
void Arm_Control_Process(void);
void Arm_Control_SetGravityMode(void);
void Arm_Control_SetGravityOnlyMode(uint8_t enabled);
uint8_t Arm_Control_IsGravityOnlyMode(void);
void Arm_Control_SetJointTarget(const Arm_Joint_Angles_t* target);
int Arm_Control_MoveToJointTarget(const Arm_Joint_Angles_t* target);
int Arm_Control_MoveToPose(const Arm_Pose_t* target);
int Arm_Control_MoveToTypedPose(const Arm_Pose_t* target, uint8_t target_type);
Arm_Control_Mode_t Arm_Control_GetMode(void);
Arm_Move_Status_t Arm_Control_GetMoveStatus(void);
void Arm_Control_GetCurrentAngles(Arm_Joint_Angles_t* angles);
uint8_t Arm_Control_IsFeedbackFresh(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_ARM_LEGACY_COMPAT_H_ */
