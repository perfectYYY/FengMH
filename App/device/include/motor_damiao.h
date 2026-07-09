/*
 * motor_damiao.h - Damiao DM43xx motor driver.
 *
 * Stage-3 migration from damiao_new1/Core/Src/dm4310_posvel.c. The driver is
 * device-layer only: it packs MIT frames, parses motor feedback, and talks via
 * bsp_fdcan. Arm kinematics and end-effector pose stay in control/arm.
 */
#ifndef APP_DEVICE_MOTOR_DAMIAO_H_
#define APP_DEVICE_MOTOR_DAMIAO_H_

#include "bsp_fdcan.h"
#include "motor_if.h"
#include "motor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DAMIAO_MOTOR_COUNT 4U

#define DAMIAO_CAN_ID_J1 0x01U
#define DAMIAO_CAN_ID_J2 0x02U
#define DAMIAO_CAN_ID_J3 0x03U
#define DAMIAO_CAN_ID_J4 0x04U

#define DAMIAO_MIT_MODE_ID 0x00U

#define DAMIAO_P_MIN   (-12.5f)
#define DAMIAO_P_MAX   (12.5f)
#define DAMIAO_KP_MIN  (0.0f)
#define DAMIAO_KP_MAX  (500.0f)
#define DAMIAO_KD_MIN  (0.0f)
#define DAMIAO_KD_MAX  (5.0f)

#define DAMIAO_DM4310_V_MIN (-30.0f)
#define DAMIAO_DM4310_V_MAX (30.0f)
#define DAMIAO_DM4310_T_MIN (-10.0f)
#define DAMIAO_DM4310_T_MAX (10.0f)

#define DAMIAO_DM4340_V_MIN (-10.0f)
#define DAMIAO_DM4340_V_MAX (10.0f)
#define DAMIAO_DM4340_T_MIN (-28.0f)
#define DAMIAO_DM4340_T_MAX (28.0f)

#define DAMIAO_STATE_DISABLED 0U
#define DAMIAO_STATE_ENABLED  1U

/* 现场诊断：CAN ID 仅观察；实际路由只使用 data[0] 低四位电机 ID。 */
extern volatile uint32_t debug_damiao_last_rx_can_id;
extern volatile uint8_t debug_damiao_last_payload_motor_id;
extern volatile uint32_t debug_damiao_feedback_route_count[DAMIAO_MOTOR_COUNT];
extern volatile uint32_t debug_damiao_feedback_invalid_motor_id_count;

typedef enum {
    DAMIAO_MODEL_DM4310 = 0,
    DAMIAO_MODEL_DM4340 = 1,
} damiao_model_t;

typedef enum {
    DAMIAO_ERR_NONE          = 0,
    DAMIAO_ERR_OVERVOLT      = 8,
    DAMIAO_ERR_UNDERVOLT     = 9,
    DAMIAO_ERR_OVERCURRENT   = 0xA,
    DAMIAO_ERR_MOS_OVERTEMP  = 0xB,
    DAMIAO_ERR_COIL_OVERTEMP = 0xC,
    DAMIAO_ERR_COMM_LOST     = 0xD,
    DAMIAO_ERR_OVERLOAD      = 0xE,
} damiao_error_t;

typedef struct {
    uint8_t        bus_id;
    uint32_t       can_id;
    damiao_model_t model;
    uint8_t        state_code;
    damiao_error_t error;
    uint8_t        enabled;
    uint8_t        temp_mos_c;
    uint8_t        temp_rotor_c;
    uint16_t       raw_position;
    uint16_t       raw_velocity;
    uint16_t       raw_torque;
    uint32_t       last_enable_attempt_ms;
    uint32_t       enable_attempt_count;
} damiao_drv_ctx_t;

app_err_t motor_damiao_init_all(void);
app_err_t motor_damiao_process(uint32_t now_ms);
uint32_t motor_damiao_auto_enable_count(void);
void motor_damiao_fdcan_rx_cb(bsp_fdcan_bus_t bus,
                              const bsp_fdcan_frame_t* frame,
                              void* user);

app_err_t motor_damiao_pack_mit(damiao_model_t model,
                                float pos_rad,
                                float vel_rads,
                                float kp,
                                float kd,
                                float tau_ff_nm,
                                uint8_t out[8]);

app_err_t motor_damiao_parse_feedback(damiao_model_t model,
                                      const uint8_t data[8],
                                      motor_state_t* out_state,
                                      damiao_drv_ctx_t* out_ctx);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEVICE_MOTOR_DAMIAO_H_ */
