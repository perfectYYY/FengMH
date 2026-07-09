/*
 * dm4310_posvel.c - damiao_new1-compatible Damiao wrappers.
 */
#include "dm4310_posvel.h"

#include "bsp_time.h"
#include "motor_damiao.h"
#include "motor_registry.h"

#include <string.h>

DM_Feedback_Data dm_feedback_motor1 = {0};
DM_Feedback_Data dm_feedback_motor2 = {0};
DM_Feedback_Data dm_feedback_motor3 = {0};
DM_Feedback_Data dm_feedback_motor4 = {0};
Arm_Pose_t current_arm_pose = {0};

static motor_logical_id_t logical_id_for_motor_num(uint8_t motor_num) {
    switch (motor_num) {
        case 1U: return MOTOR_ID_ARM_J1;
        case 2U: return MOTOR_ID_ARM_J2;
        case 3U: return MOTOR_ID_ARM_J3;
        case 4U: return MOTOR_ID_ARM_J4;
        default: return MOTOR_ID_MAX;
    }
}

static DM_Feedback_Data* legacy_feedback_for_motor_num(uint8_t motor_num,
                                                       uint8_t* motor_idx,
                                                       damiao_model_t* model) {
    switch (motor_num) {
        case 1U:
            if (motor_idx) *motor_idx = 1U;
            if (model) *model = DAMIAO_MODEL_DM4340;
            return &dm_feedback_motor1;
        case 2U:
            if (motor_idx) *motor_idx = 2U;
            if (model) *model = DAMIAO_MODEL_DM4340;
            return &dm_feedback_motor2;
        case 3U:
            if (motor_idx) *motor_idx = 3U;
            if (model) *model = DAMIAO_MODEL_DM4340;
            return &dm_feedback_motor3;
        case 4U:
            if (motor_idx) *motor_idx = 4U;
            if (model) *model = DAMIAO_MODEL_DM4310;
            return &dm_feedback_motor4;
        default:
            return NULL;
    }
}

static uint8_t legacy_motor_num_for_feedback_can_id(uint32_t can_id) {
    switch (can_id) {
        case DM_MOTOR1_FEEDBACK_ID: return 1U;
        case DM_MOTOR2_FEEDBACK_ID: return 2U;
        case DM_MOTOR3_FEEDBACK_ID: return 3U;
        case DM_MOTOR4_FEEDBACK_ID: return 4U;
        default: return 0U;
    }
}

static DM_Feedback_Data* legacy_feedback_for_frame(uint32_t can_id,
                                                   const uint8_t rx_data[8],
                                                   uint8_t* motor_idx,
                                                   damiao_model_t* model) {
    /* 主链必须与 motor_damiao 使用同一规则：payload 电机 ID 优先。 */
    if (rx_data) {
        const uint8_t payload_motor_id = (uint8_t)(rx_data[0] & 0x0FU);
        DM_Feedback_Data* feedback =
            legacy_feedback_for_motor_num(payload_motor_id, motor_idx, model);
        if (feedback) return feedback;
    }

    /* 仅供脱离主驱动直接调用旧 parser 时兼容。 */
    return legacy_feedback_for_motor_num(
        legacy_motor_num_for_feedback_can_id(can_id), motor_idx, model);
}

static float input_position_to_rad(float p_des_deg) {
    return (DM_INPUT_IS_RADIAN == 0) ? DEG2RAD(p_des_deg) : p_des_deg;
}

static HAL_StatusTypeDef app_err_to_hal(int err) {
    if (err == APP_OK) return HAL_OK;
    if (err == APP_ERR_BUSY) return HAL_BUSY;
    if (err == APP_ERR_TIMEOUT) return HAL_TIMEOUT;
    return HAL_ERROR;
}

void DM_PosVel_Init(FDCAN_HandleTypeDef* hfdcan) {
    (void)hfdcan;
}

void DM_CAN_Filter_Init(void) {
}

HAL_StatusTypeDef DM_Motor_Enable(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_num) {
    (void)hfdcan;
    motor_logical_id_t id = logical_id_for_motor_num(motor_num);
    motor_dev_t* motor = motor_get(id);
    if (!motor || !motor->ops || !motor->ops->enable) return HAL_ERROR;
    return app_err_to_hal(motor->ops->enable(motor));
}

HAL_StatusTypeDef DM_PosVel_Control(FDCAN_HandleTypeDef* hfdcan,
                                    uint8_t motor_num,
                                    float p_des_deg,
                                    float v_des) {
    (void)hfdcan;
    motor_logical_id_t id = logical_id_for_motor_num(motor_num);
    motor_dev_t* motor = motor_get(id);
    if (!motor || !motor->ops || !motor->ops->set_position) return HAL_ERROR;
    return app_err_to_hal(motor->ops->set_position(motor,
                                                   input_position_to_rad(p_des_deg),
                                                   v_des,
                                                   0.0f,
                                                   0.0f,
                                                   0.0f));
}

static HAL_StatusTypeDef legacy_mit_control(uint8_t motor_num,
                                            float p_des_deg,
                                            float v_des,
                                            float kp,
                                            float kd,
                                            float t_ff) {
    motor_logical_id_t id = logical_id_for_motor_num(motor_num);
    motor_dev_t* motor = motor_get(id);
    if (!motor || !motor->ops || !motor->ops->set_position) return HAL_ERROR;
    return app_err_to_hal(motor->ops->set_position(motor,
                                                   input_position_to_rad(p_des_deg),
                                                   v_des,
                                                   kp,
                                                   kd,
                                                   t_ff));
}

HAL_StatusTypeDef DM_MIT_Control_4310(FDCAN_HandleTypeDef* hfdcan,
                                      uint8_t motor_num,
                                      float p_des_deg,
                                      float v_des,
                                      float kp,
                                      float kd,
                                      float t_ff) {
    (void)hfdcan;
    return legacy_mit_control(motor_num, p_des_deg, v_des, kp, kd, t_ff);
}

HAL_StatusTypeDef DM_MIT_Control_4340(FDCAN_HandleTypeDef* hfdcan,
                                      uint8_t motor_num,
                                      float p_des_deg,
                                      float v_des,
                                      float kp,
                                      float kd,
                                      float t_ff) {
    (void)hfdcan;
    return legacy_mit_control(motor_num, p_des_deg, v_des, kp, kd, t_ff);
}

static void legacy_feedback_from_state(const motor_state_t* state,
                                       const damiao_drv_ctx_t* ctx,
                                       DM_Feedback_Data* out,
                                       uint8_t motor_idx) {
    if (!state || !ctx || !out) return;
    out->motor_id = motor_idx;
    out->error = (DM_Error_Code)ctx->error;
    out->position = (int16_t)ctx->raw_position;
    out->position_rad = state->angle_rad;
    out->position_deg = RAD2DEG(state->angle_rad);
    out->speed = (int16_t)ctx->raw_velocity;
    out->speed_rad_s = state->velocity_rads;
    out->torque = (int16_t)ctx->raw_torque;
    out->torque_nm = state->torque_nm;
    out->temp_mos = ctx->temp_mos_c;
    out->temp_rotor = ctx->temp_rotor_c;
    out->is_updated = 1U;
    out->is_enabled = state->online ? 1U : 0U;
    out->last_rx_ms = state->last_rx_tick;
}

static void update_legacy_current_pose(void) {
    Arm_Joint_Angles_t real_angles = {0};
    real_angles.theta1_motor = dm_feedback_motor1.position_rad;
    real_angles.theta2_motor =
        ARM_MOTOR2_TO_LOGICAL(dm_feedback_motor2.position_rad);
    real_angles.theta3_motor =
        ARM_MOTOR3_TO_LOGICAL(dm_feedback_motor3.position_rad);
    real_angles.theta4_motor = dm_feedback_motor4.position_rad;
    real_angles.theta1_geo =
        real_angles.theta1_motor + DEG2RAD(arm_offset.theta1_offset_deg);
    real_angles.theta2_geo =
        real_angles.theta2_motor + DEG2RAD(arm_offset.theta2_offset_deg);
    real_angles.theta3_geo =
        real_angles.theta3_motor + DEG2RAD(arm_offset.theta3_offset_deg);
    real_angles.theta4_geo =
        real_angles.theta4_motor + DEG2RAD(arm_offset.theta4_offset_deg);
    Arm_Forward_Kinematics(&arm_params, &real_angles, &current_arm_pose);
}

void DM_Parse_Feedback(FDCAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data) {
    if (!rx_header || !rx_data) return;

    uint8_t motor_idx = 0U;
    damiao_model_t model = DAMIAO_MODEL_DM4340;
    DM_Feedback_Data* feedback =
        legacy_feedback_for_frame(rx_header->Identifier,
                                  rx_data,
                                  &motor_idx,
                                  &model);
    if (!feedback) return;

    motor_state_t state;
    damiao_drv_ctx_t ctx;
    memset(&state, 0, sizeof(state));
    memset(&ctx, 0, sizeof(ctx));
    if (motor_damiao_parse_feedback(model, rx_data, &state, &ctx) != APP_OK) {
        return;
    }
    state.last_rx_tick = (uint32_t)bsp_time_now_ms();
    legacy_feedback_from_state(&state, &ctx, feedback, motor_idx);
    update_legacy_current_pose();
}
