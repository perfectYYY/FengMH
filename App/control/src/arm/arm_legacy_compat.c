/*
 * arm_legacy_compat.c - Old damiao_new1 API wrappers.
 */
#include "arm_legacy_compat.h"

#include "arm_control.h"
#include "arm_gravity_comp.h"
#include "arm_kinematics.h"
#include "bsp_time.h"
#include "dm4310_posvel.h"
#include "motor_registry.h"
#include "proto_defs.h"

#include <math.h>
#include <string.h>

Arm_Params_t arm_params = {350.0f, 300.0f};
Arm_Offset_Config_t arm_offset = {
    .theta1_offset_deg = 0.0f,
    .theta2_offset_deg = 90.0f,
    .theta3_offset_deg = 0.0f,
    .theta4_offset_deg = 180.0f,
    .theta3_theta4_sum_deg = -85.0f,
};

volatile uint8_t debug_payload_loaded = 0U;
volatile float dbg_gravity_scale_all = 1.0f;
volatile float dbg_gravity_scale_2 = 1.0f;
volatile float dbg_gravity_scale_3 = 2.0f;
volatile float dbg_gravity_scale_4 = 1.0f;
volatile float debug_gravity_tau2 = 0.0f;
volatile float debug_gravity_tau3 = 0.0f;
volatile float debug_gravity_tau4 = 0.0f;

volatile uint8_t debug_motion_status = ARM_MOVE_IDLE;
volatile int32_t debug_motion_last_result = 0;
volatile float debug_motion_duration_s = 0.0f;
volatile float debug_motion_progress = 0.0f;
volatile float debug_motion_max_error_deg = 0.0f;
volatile uint32_t debug_feedback_age1_ms = 0U;
volatile uint32_t debug_feedback_age2_ms = 0U;
volatile uint32_t debug_feedback_age3_ms = 0U;
volatile uint32_t debug_feedback_age4_ms = 0U;
volatile uint8_t debug_motor_tx_fail_mask = 0U;
volatile uint32_t debug_motor_tx_fail_count = 0U;
volatile uint8_t debug_safe_move_stage = 0U;
volatile uint32_t debug_safe_move_cycle_count = 0U;
volatile uint8_t debug_fine_tracking = 0U;

volatile float debug_motion_vmax_1 = 0.90f;
volatile float debug_motion_vmax_2 = 0.80f;
volatile float debug_motion_vmax_3 = 0.80f;
volatile float debug_motion_vmax_4 = 1.00f;
volatile float debug_motion_amax_1 = 1.80f;
volatile float debug_motion_amax_2 = 1.60f;
volatile float debug_motion_amax_3 = 1.60f;
volatile float debug_motion_amax_4 = 2.00f;
volatile float debug_move_kp_4340 = 100.0f;
volatile float debug_move_kd_4340 = 1.5f;
volatile float debug_move_kp_4310 = 100.0f;
volatile float debug_move_kd_4310 = 1.5f;
volatile float debug_hold_kp_4340 = 70.0f;
volatile float debug_hold_kd_4340 = 1.5f;
volatile float debug_hold_kp_4310 = 70.0f;
volatile float debug_hold_kd_4310 = 1.2f;
volatile float debug_gravity_kp_4 = 30.0f;
volatile float debug_gravity_kd_4 = 0.8f;

volatile float dbg_gc_mass_l2 = ARM_GRAVITY_MASS_L2;
volatile float dbg_gc_mass_l3 = ARM_GRAVITY_MASS_L3;
volatile float dbg_gc_mass_ee = ARM_GRAVITY_MASS_EE_EMPTY;
volatile float dbg_gc_mass_cargo = ARM_GRAVITY_MASS_CARGO_BOX;
volatile float dbg_gc_com_r2_x = ARM_GRAVITY_COM_R2_X_M;
volatile float dbg_gc_com_r2_y = ARM_GRAVITY_COM_R2_Y_M;
volatile float dbg_gc_com_r3_x = ARM_GRAVITY_COM_R3_X_M;
volatile float dbg_gc_com_r3_y = ARM_GRAVITY_COM_R3_Y_M;
volatile float dbg_gc_com_ee_x = ARM_GRAVITY_COM_EE_X_M;

static uint8_t s_legacy_initialized;
static uint8_t s_legacy_gravity_only;
static uint8_t s_last_safe_stage;
static uint8_t s_last_fine_tracking;

static void enable_all_motors_legacy(FDCAN_HandleTypeDef* hfdcan) {
    for (uint8_t motor_id = 1U; motor_id <= 4U; motor_id++) {
        (void)DM_Motor_Enable(hfdcan, motor_id);
    }
}

static float legacy_clampf(float v, float min_v, float max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static float wrap_to_2pi(float a) {
    while (a < 0.0f) a += 2.0f * PI;
    while (a >= 2.0f * PI) a -= 2.0f * PI;
    return a;
}

static float normalize_t1_motor(float t1_motor_raw) {
    return wrap_to_2pi(t1_motor_raw);
}

static void sync_legacy_gravity_debug_to_current(void) {
    g_arm_gc_mass_l2_kg = dbg_gc_mass_l2;
    g_arm_gc_mass_l3_kg = dbg_gc_mass_l3;
    g_arm_gc_mass_ee_kg = dbg_gc_mass_ee;
    g_arm_gc_mass_cargo_kg = dbg_gc_mass_cargo;
    g_arm_gc_com_r2_x_m = dbg_gc_com_r2_x;
    g_arm_gc_com_r2_y_m = dbg_gc_com_r2_y;
    g_arm_gc_com_r3_x_m = dbg_gc_com_r3_x;
    g_arm_gc_com_r3_y_m = dbg_gc_com_r3_y;
    g_arm_gc_com_ee_x_m = dbg_gc_com_ee_x;
}

float Arm_Compute_T4_From_T3(const Arm_Offset_Config_t* offset,
                             float t3_motor) {
    if (!offset) return 0.0f;
    return DEG2RAD(offset->theta3_theta4_sum_deg) - t3_motor;
}

void Arm_Forward_Kinematics(const Arm_Params_t* params,
                            const Arm_Joint_Angles_t* angles,
                            Arm_Pose_t* pose) {
    if (!params || !angles || !pose) return;

    float r = params->L2 * cosf(angles->theta2_geo) +
              params->L3 * cosf(angles->theta3_geo);
    pose->x = r * cosf(angles->theta1_geo);
    pose->y = r * sinf(angles->theta1_geo);
    pose->z = params->L2 * sinf(angles->theta2_geo) +
              params->L3 * sinf(angles->theta3_geo);
    pose->pitch = 0.0f;
}

int Arm_Inverse_Kinematics(const Arm_Params_t* params,
                           const Arm_Offset_Config_t* offset,
                           const Arm_Pose_t* target,
                           Arm_Joint_Angles_t* angles) {
    if (!params || !offset || !target || !angles) return -1;

    const float x = target->x;
    const float y = target->y;
    const float z = target->z;
    const float l2 = params->L2;
    const float l3 = params->L3;
    const float eps = 1e-6f;
    if (!isfinite(x) || !isfinite(y) || !isfinite(z) ||
        l2 <= 0.0f || l3 <= 0.0f) {
        return -1;
    }

    const float r = sqrtf(x * x + y * y);
    const float distance_sq = r * r + z * z;
    const float distance = sqrtf(distance_sq);
    if (distance < fabsf(l2 - l3) - 1e-3f ||
        distance > l2 + l3 + 1e-3f ||
        distance < eps) {
        return -1;
    }

    const float theta1_geo = (r < eps) ? 0.0f : atan2f(y, x);
    const float phi = atan2f(z, r);
    float cos_delta = (distance_sq + l2 * l2 - l3 * l3) /
                      (2.0f * distance * l2);
    const float delta = acosf(legacy_clampf(cos_delta, -1.0f, 1.0f));
    const float theta2_candidate[2] = { phi + delta, phi - delta };

    for (uint32_t candidate = 0U; candidate < 2U; candidate++) {
        const float theta2_geo = theta2_candidate[candidate];
        const float remaining_r = r - l2 * cosf(theta2_geo);
        const float remaining_z = z - l2 * sinf(theta2_geo);
        const float theta3_geo = atan2f(remaining_z, remaining_r);
        const float t2_motor = theta2_geo - DEG2RAD(offset->theta2_offset_deg);
        const float t3_motor = theta3_geo - DEG2RAD(offset->theta3_offset_deg);
        const float t2_physical_deg = RAD2DEG(ARM_MOTOR2_TO_PHYSICAL(t2_motor));
        const float t3_physical_deg = RAD2DEG(ARM_MOTOR3_TO_PHYSICAL(t3_motor));

        if (t2_physical_deg < ARM_MOTOR2_PHYSICAL_MIN_DEG - 1e-3f ||
            t2_physical_deg > ARM_MOTOR2_PHYSICAL_MAX_DEG + 1e-3f ||
            t3_physical_deg < ARM_MOTOR3_PHYSICAL_MIN_DEG - 1e-3f ||
            t3_physical_deg > ARM_MOTOR3_PHYSICAL_MAX_DEG + 1e-3f) {
            continue;
        }

        memset(angles, 0, sizeof(*angles));
        angles->theta1_geo = wrap_to_2pi(theta1_geo);
        angles->theta2_geo = theta2_geo;
        angles->theta3_geo = theta3_geo;
        angles->theta1_motor =
            normalize_t1_motor(theta1_geo - DEG2RAD(offset->theta1_offset_deg));
        angles->theta2_motor = t2_motor;
        angles->theta3_motor = t3_motor;
        angles->theta4_motor = Arm_Compute_T4_From_T3(offset, t3_motor);
        angles->theta4_geo =
            angles->theta4_motor + DEG2RAD(offset->theta4_offset_deg);
        return 0;
    }

    return -2;
}

void Arm_Motion_Init(const Arm_Motion_Limits_t* limits) {
    arm_motion_init(limits);
}

void Arm_Motion_SetLimits(const Arm_Motion_Limits_t* limits) {
    arm_motion_set_limits(limits);
}

int Arm_Motion_Start(const Arm_Motion_Sample_t* start,
                     const float target[ARM_MOTION_JOINT_COUNT],
                     uint32_t now_ms) {
    return arm_motion_start(start, target, now_ms);
}

Arm_Trajectory_State_t Arm_Motion_Update(uint32_t now_ms,
                                         Arm_Motion_Sample_t* sample) {
    return arm_motion_update(now_ms, sample);
}

void Arm_Motion_Stop(const Arm_Motion_Sample_t* hold_sample) {
    arm_motion_stop(hold_sample);
}

Arm_Trajectory_State_t Arm_Motion_GetState(void) {
    return arm_motion_get_state();
}

void Arm_Motion_GetSample(Arm_Motion_Sample_t* sample) {
    arm_motion_get_sample(sample);
}

float Arm_Motion_GetDuration(void) {
    return arm_motion_get_duration();
}

float Arm_Motion_GetProgress(uint32_t now_ms) {
    return arm_motion_get_progress(now_ms);
}

void Gravity_Comp_Init(void) {
    sync_legacy_gravity_debug_to_current();
    arm_gravity_comp_init();
}

void Gravity_Comp_SetPayloadState(Gravity_Payload_State_t state) {
    sync_legacy_gravity_debug_to_current();
    arm_gravity_comp_set_payload_state(
        state == GRAVITY_PAYLOAD_LOADED ?
        ARM_GRAVITY_PAYLOAD_LOADED : ARM_GRAVITY_PAYLOAD_EMPTY);
}

Gravity_Payload_State_t Gravity_Comp_GetPayloadState(void) {
    return arm_gravity_comp_get_payload_state() == ARM_GRAVITY_PAYLOAD_LOADED ?
           GRAVITY_PAYLOAD_LOADED : GRAVITY_PAYLOAD_EMPTY;
}

float Gravity_Comp_GetActiveEndMass(void) {
    return arm_gravity_comp_get_active_end_mass();
}

float Gravity_Comp_GetTargetEndMass(void) {
    return arm_gravity_comp_get_target_end_mass();
}

void Calculate_Gravity_Compensation(float theta2_rad,
                                    float theta3_rad,
                                    float theta4_rad,
                                    float* tau2,
                                    float* tau3,
                                    float* tau4) {
    sync_legacy_gravity_debug_to_current();
    arm_gravity_comp_calculate(theta2_rad, theta3_rad, theta4_rad,
                               tau2, tau3, tau4);
}

void Arm_Control_Init(FDCAN_HandleTypeDef* hfdcan) {
    debug_motion_last_result = 0;
    debug_motion_status = ARM_MOVE_IDLE;
    debug_fine_tracking = 0U;
    s_last_safe_stage = 0U;
    s_last_fine_tracking = 0U;
    Gravity_Comp_Init();
    DM_PosVel_Init(hfdcan);
    (void)arm_control_init();
    (void)arm_control_set_enabled(1U);
    enable_all_motors_legacy(hfdcan);
    s_legacy_initialized = 1U;
}

void Arm_Control_SetGravityMode(void) {
    (void)arm_control_set_gravity_mode();
    debug_motion_status = ARM_MOVE_IDLE;
    debug_safe_move_stage = 0U;
    debug_fine_tracking = 0U;
    s_last_safe_stage = 0U;
    s_last_fine_tracking = 0U;
}

void Arm_Control_SetGravityOnlyMode(uint8_t enabled) {
    s_legacy_gravity_only = enabled ? 1U : 0U;
    if (s_legacy_gravity_only) {
        Arm_Control_SetGravityMode();
    }
}

uint8_t Arm_Control_IsGravityOnlyMode(void) {
    return s_legacy_gravity_only;
}

void Arm_Control_SetJointTarget(const Arm_Joint_Angles_t* target) {
    (void)Arm_Control_MoveToJointTarget(target);
}

int Arm_Control_MoveToJointTarget(const Arm_Joint_Angles_t* target) {
    if (s_legacy_gravity_only) return -3;
    if (!target) return -1;
    if (!s_legacy_initialized) {
        Arm_Control_Init(NULL);
    }

    arm_joint_angles_t native;
    memset(&native, 0, sizeof(native));
    native.theta1_motor_rad = target->theta1_motor;
    native.theta2_motor_rad = target->theta2_motor;
    native.theta3_motor_rad = target->theta3_motor;
    native.theta4_motor_rad = target->theta4_motor;
    native.theta1_geo_rad = target->theta1_geo;
    native.theta2_geo_rad = target->theta2_geo;
    native.theta3_geo_rad = target->theta3_geo;
    native.theta4_geo_rad = target->theta4_geo;

    app_err_t err = arm_control_set_joint_target(PROTO_ARM_TARGET_GRASP,
                                                 &native);
    debug_motion_last_result = (err == APP_OK) ? 0 : (int32_t)err;
    return (err == APP_OK) ? 0 : -1;
}

int Arm_Control_MoveToPose(const Arm_Pose_t* target) {
    return Arm_Control_MoveToTypedPose(target, PROTO_ARM_TARGET_GRASP);
}

int Arm_Control_MoveToTypedPose(const Arm_Pose_t* target, uint8_t target_type) {
    if (s_legacy_gravity_only) return -3;
    if (!target) return -1;
    if (target_type != PROTO_ARM_TARGET_GRASP &&
        target_type != PROTO_ARM_TARGET_PLACE) {
        return -1;
    }
    if (!s_legacy_initialized) {
        Arm_Control_Init(NULL);
    }
    app_err_t err = arm_control_set_target(target_type,
                                           target->x * 0.001f,
                                           target->y * 0.001f,
                                           target->z * 0.001f);
    debug_motion_last_result = (err == APP_OK) ? 0 : (int32_t)err;
    return (err == APP_OK) ? 0 : -1;
}

Arm_Control_Mode_t Arm_Control_GetMode(void) {
    arm_control_status_t status;
    arm_control_get_status(&status);
    return status.target_valid ? ARM_CONTROL_POSITION_HOLD : ARM_CONTROL_GRAVITY;
}

Arm_Move_Status_t Arm_Control_GetMoveStatus(void) {
    arm_control_status_t status;
    arm_control_get_status(&status);
    if (status.last_result == APP_ERR_INVALID_ARG) return ARM_MOVE_ERROR_INVALID_TARGET;
    if (status.last_result == APP_ERR_OFFLINE) return ARM_MOVE_ERROR_NO_FEEDBACK;
    if (status.last_result == APP_ERR_TIMEOUT) return ARM_MOVE_ERROR_TIMEOUT;
    if (status.reached) return ARM_MOVE_REACHED;
    if (status.target_pending ||
        status.trajectory_state == (uint8_t)ARM_TRAJECTORY_MOVING) {
        return ARM_MOVE_MOVING;
    }
    if (status.target_valid) return ARM_MOVE_SETTLING;
    return ARM_MOVE_IDLE;
}

static uint32_t feedback_age_for_motor(motor_logical_id_t id, uint32_t now_ms) {
    const motor_dev_t* motor = motor_get(id);
    if (!motor || motor->state.rx_cnt == 0U) return UINT32_MAX;
    return now_ms - motor->state.last_rx_tick;
}

void Arm_Control_GetCurrentAngles(Arm_Joint_Angles_t* angles) {
    if (!angles) return;
    memset(angles, 0, sizeof(*angles));

    const motor_dev_t* j1 = motor_get(MOTOR_ID_ARM_J1);
    const motor_dev_t* j2 = motor_get(MOTOR_ID_ARM_J2);
    const motor_dev_t* j3 = motor_get(MOTOR_ID_ARM_J3);
    const motor_dev_t* j4 = motor_get(MOTOR_ID_ARM_J4);
    if (j1 && j2 && j3 && j4 &&
        j1->state.rx_cnt && j2->state.rx_cnt &&
        j3->state.rx_cnt && j4->state.rx_cnt) {
        angles->theta1_motor = j1->state.angle_rad;
        angles->theta2_motor = ARM_MOTOR2_TO_LOGICAL(j2->state.angle_rad);
        angles->theta3_motor = ARM_MOTOR3_TO_LOGICAL(j3->state.angle_rad);
        angles->theta4_motor = j4->state.angle_rad;
        angles->theta1_geo =
            angles->theta1_motor + DEG2RAD(arm_offset.theta1_offset_deg);
        angles->theta2_geo =
            angles->theta2_motor + DEG2RAD(arm_offset.theta2_offset_deg);
        angles->theta3_geo =
            angles->theta3_motor + DEG2RAD(arm_offset.theta3_offset_deg);
        angles->theta4_geo =
            angles->theta4_motor + DEG2RAD(arm_offset.theta4_offset_deg);
        return;
    }

    arm_control_status_t status;
    arm_control_get_status(&status);
    Arm_Pose_t pose = {
        .x = status.planned_end_x_m * 1000.0f,
        .y = status.planned_end_y_m * 1000.0f,
        .z = status.planned_end_z_m * 1000.0f,
        .pitch = 0.0f,
    };
    (void)Arm_Inverse_Kinematics(&arm_params, &arm_offset, &pose, angles);
}

uint8_t Arm_Control_IsFeedbackFresh(void) {
    arm_control_status_t status;
    arm_control_get_status(&status);
    return status.motor_feedback_fresh ? 1U : 0U;
}

void Arm_Control_Process(void) {
    if (!s_legacy_initialized) {
        Arm_Control_Init(NULL);
    }
    sync_legacy_gravity_debug_to_current();
    arm_gravity_comp_set_payload_state(debug_payload_loaded ?
                                       ARM_GRAVITY_PAYLOAD_LOADED :
                                       ARM_GRAVITY_PAYLOAD_EMPTY);

    if (s_legacy_gravity_only) {
        if (Arm_Control_GetMode() != ARM_CONTROL_GRAVITY) {
            Arm_Control_SetGravityMode();
        }
    }

    uint32_t now = (uint32_t)bsp_time_now_ms();
    (void)arm_control_tick(0.010f, now);

    arm_control_status_t status;
    arm_control_get_status(&status);
    debug_motion_status = (uint8_t)Arm_Control_GetMoveStatus();
    debug_motion_duration_s = status.motion_duration_s;
    debug_motion_progress = status.motion_progress;
    debug_motion_max_error_deg = RAD2DEG(status.settle_error_rad);
    debug_motor_tx_fail_mask = status.motor_tx_fail_mask;
    debug_motor_tx_fail_count = status.motor_tx_fail_count;
    debug_safe_move_stage = status.safe_move_stage;
    debug_fine_tracking = status.fine_tracking_active;
    if (s_last_safe_stage != status.safe_move_stage &&
        status.safe_move_stage == 0U) {
        debug_safe_move_cycle_count++;
    }
    s_last_safe_stage = status.safe_move_stage;
    s_last_fine_tracking = status.fine_tracking_active;
    (void)s_last_fine_tracking;

    debug_feedback_age1_ms = feedback_age_for_motor(MOTOR_ID_ARM_J1, now);
    debug_feedback_age2_ms = feedback_age_for_motor(MOTOR_ID_ARM_J2, now);
    debug_feedback_age3_ms = feedback_age_for_motor(MOTOR_ID_ARM_J3, now);
    debug_feedback_age4_ms = feedback_age_for_motor(MOTOR_ID_ARM_J4, now);
}
