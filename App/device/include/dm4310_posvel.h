/*
 * dm4310_posvel.h - damiao_new1-compatible Damiao API.
 */
#ifndef APP_DEVICE_DM4310_POSVEL_H_
#define APP_DEVICE_DM4310_POSVEL_H_

#include <stdint.h>

#include "arm_legacy_compat.h"
#include "config.h"

#if APP_TARGET_HOST
typedef struct {
    uint32_t Identifier;
} FDCAN_RxHeaderTypeDef;

typedef enum {
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U,
} HAL_StatusTypeDef;
#else
#include "stm32h7xx_hal.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define DM_MOTOR1_FEEDBACK_ID 0x02U
#define DM_MOTOR2_FEEDBACK_ID 0x03U
#define DM_MOTOR3_FEEDBACK_ID 0x04U
#define DM_MOTOR4_FEEDBACK_ID 0x05U

#define DM_MOTOR1_ID 0x01U
#define DM_MOTOR2_ID 0x02U
#define DM_MOTOR3_ID 0x03U
#define DM_MOTOR4_ID 0x04U

#define PV_mode 0x100U

#define DM4310_P_MIN (-12.5f)
#define DM4310_P_MAX (12.5f)
#define DM4310_V_MIN (-30.0f)
#define DM4310_V_MAX (30.0f)
#define DM4310_T_MIN (-10.0f)
#define DM4310_T_MAX (10.0f)

#define DM4340_P_MIN (-12.5f)
#define DM4340_P_MAX (12.5f)
#define DM4340_V_MIN (-10.0f)
#define DM4340_V_MAX (10.0f)
#define DM4340_T_MIN (-28.0f)
#define DM4340_T_MAX (28.0f)

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

#define DM4310_MIT_MODE 0x00U
#define DM4340_MIT_MODE 0x00U
#define DM_MOTOR_STATE_DISABLED 0U
#define DM_MOTOR_STATE_ENABLED 1U
#define DM_INPUT_IS_RADIAN 0

typedef enum {
    DM_ERR_NONE = 0,
    DM_ERR_OVERVOLT = 8,
    DM_ERR_UNDERVOLT = 9,
    DM_ERR_OVERCURRENT = 0xA,
    DM_ERR_MOS_OVERTEMP = 0xB,
    DM_ERR_COIL_OVERTEMP = 0xC,
    DM_ERR_COMM_LOST = 0xD,
    DM_ERR_OVERLOAD = 0xE,
} DM_Error_Code;

typedef struct {
    uint8_t motor_id;
    DM_Error_Code error;
    int16_t position;
    float position_rad;
    float position_deg;
    int16_t speed;
    float speed_rad_s;
    int16_t torque;
    float torque_nm;
    uint8_t temp_mos;
    uint8_t temp_rotor;
    uint8_t is_updated;
    uint8_t is_enabled;
    uint32_t last_rx_ms;
} DM_Feedback_Data;

extern DM_Feedback_Data dm_feedback_motor1;
extern DM_Feedback_Data dm_feedback_motor2;
extern DM_Feedback_Data dm_feedback_motor3;
extern DM_Feedback_Data dm_feedback_motor4;
extern Arm_Pose_t current_arm_pose;

void DM_PosVel_Init(FDCAN_HandleTypeDef* hfdcan);
void DM_CAN_Filter_Init(void);
void DM_Parse_Feedback(FDCAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data);
HAL_StatusTypeDef DM_Motor_Enable(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_num);
HAL_StatusTypeDef DM_PosVel_Control(FDCAN_HandleTypeDef* hfdcan,
                                    uint8_t motor_num,
                                    float p_des_deg,
                                    float v_des);
HAL_StatusTypeDef DM_MIT_Control_4310(FDCAN_HandleTypeDef* hfdcan,
                                      uint8_t motor_num,
                                      float p_des_deg,
                                      float v_des,
                                      float kp,
                                      float kd,
                                      float t_ff);
HAL_StatusTypeDef DM_MIT_Control_4340(FDCAN_HandleTypeDef* hfdcan,
                                      uint8_t motor_num,
                                      float p_des_deg,
                                      float v_des,
                                      float kp,
                                      float kd,
                                      float t_ff);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEVICE_DM4310_POSVEL_H_ */
