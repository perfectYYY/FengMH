/*
 * motor_registry.c
 *
 * 改动硬件映射只需改本文件，不需要动算法
 */
#include "motor_registry.h"
#include "log.h"

#include <string.h>

static const char* TAG = "REG";

#define GO_GEAR_RATIO       6.33f
#define PI_F                3.14159265f
#define BOOT_MIRROR_THIGH   0.0f
#define BOOT_MIRROR_KNEE   (-0.75f * PI_F)
#define BOOT_ORIG_THIGH    (-1.0f * PI_F)
#define BOOT_ORIG_KNEE     (-0.25f * PI_F)

static const motor_cfg_t s_cfg[MOTOR_ID_MAX] = {
    /* FL/LF: old slots 0/1/2, GO on USART3, wheel on CAN1 DJI1 */
    { MOTOR_ID_FL_HIP,   MOTOR_GO,    2, 0x000, +1, 0.0f, GO_GEAR_RATIO, BOOT_MIRROR_THIGH, -2.0f,       0.0f, "FL_HIP" },
    { MOTOR_ID_FL_KNEE,  MOTOR_GO,    2, 0x001, -1, 0.0f, GO_GEAR_RATIO, BOOT_MIRROR_KNEE,  -PI_F,      -0.5f * PI_F, "FL_KNEE" },
    { MOTOR_ID_FL_WHEEL, MOTOR_M3508, 0, 0x201, +1, 0.0f, 1.0f,          0.0f,               0.0f,        0.0f, "FL_WHEEL" },
    /* RL/LR: old slots 3/4/5, GO on USART2, wheel on CAN1 DJI2 */
    { MOTOR_ID_RL_HIP,   MOTOR_GO,    1, 0x003, +1, 0.0f, GO_GEAR_RATIO, BOOT_ORIG_THIGH,   -PI_F,       2.0f - PI_F, "RL_HIP" },
    { MOTOR_ID_RL_KNEE,  MOTOR_GO,    1, 0x004, -1, 0.0f, GO_GEAR_RATIO, BOOT_ORIG_KNEE,    -0.5f * PI_F, 0.0f, "RL_KNEE" },
    { MOTOR_ID_RL_WHEEL, MOTOR_M3508, 0, 0x202, +1, 0.0f, 1.0f,          0.0f,               0.0f,        0.0f, "RL_WHEEL" },
    /* RR: old slots 6/7/8, GO on USART3, wheel on CAN2 DJI1 */
    { MOTOR_ID_RR_HIP,   MOTOR_GO,    2, 0x006, +1, 0.0f, GO_GEAR_RATIO, BOOT_MIRROR_THIGH, -2.0f,       0.0f, "RR_HIP" },
    { MOTOR_ID_RR_KNEE,  MOTOR_GO,    2, 0x007, -1, 0.0f, GO_GEAR_RATIO, BOOT_MIRROR_KNEE,  -PI_F,      -0.5f * PI_F, "RR_KNEE" },
    { MOTOR_ID_RR_WHEEL, MOTOR_M3508, 1, 0x201, -1, 0.0f, 1.0f,          0.0f,               0.0f,        0.0f, "RR_WHEEL" },
    /* FR/RF: old slots 9/10/11, GO on USART2, wheel on CAN2 DJI2 */
    { MOTOR_ID_FR_HIP,   MOTOR_GO,    1, 0x009, +1, 0.0f, GO_GEAR_RATIO, BOOT_ORIG_THIGH,   -PI_F,       2.0f - PI_F, "FR_HIP" },
    { MOTOR_ID_FR_KNEE,  MOTOR_GO,    1, 0x00A, -1, 0.0f, GO_GEAR_RATIO, BOOT_ORIG_KNEE,    -0.5f * PI_F, 0.0f, "FR_KNEE" },
    { MOTOR_ID_FR_WHEEL, MOTOR_M3508, 1, 0x202, -1, 0.0f, 1.0f,          0.0f,               0.0f,        0.0f, "FR_WHEEL" },
    /* ARM */
    { MOTOR_ID_ARM_J1,   MOTOR_DAMIAO, 3, 0x000, +1, 0.0f, 1.0f, 0.0f, -PI_F, +PI_F, "ARM_J1" },
    { MOTOR_ID_ARM_J2,   MOTOR_DAMIAO, 3, 0x000, +1, 0.0f, 1.0f, 0.0f, -PI_F, +PI_F, "ARM_J2" },
    { MOTOR_ID_ARM_J3,   MOTOR_DAMIAO, 3, 0x000, +1, 0.0f, 1.0f, 0.0f, -PI_F, +PI_F, "ARM_J3" },
    { MOTOR_ID_ARM_J4,   MOTOR_DAMIAO, 3, 0x000, +1, 0.0f, 1.0f, 0.0f, -PI_F, +PI_F, "ARM_J4" },
    { MOTOR_ID_ARM_J5,   MOTOR_M3508,  3, 0x205, +1, 0.0f, 1.0f, 0.0f,  0.0f,  0.0f, "ARM_J5" },
    { MOTOR_ID_ARM_J6,   MOTOR_M3508,  3, 0x206, +1, 0.0f, 1.0f, 0.0f,  0.0f,  0.0f, "ARM_J6" },
};

static motor_dev_t* s_devs[MOTOR_ID_MAX];

app_err_t motor_registry_init(void) {
    memset(s_devs, 0, sizeof(s_devs));
    LOGI("registry: %u slots ready (devs unbound)", (unsigned)MOTOR_ID_MAX);
    return APP_OK;
}

motor_dev_t* motor_get(motor_logical_id_t id) {
    if (id >= MOTOR_ID_MAX) return 0;
    return s_devs[id];
}

const motor_cfg_t* motor_get_cfg(motor_logical_id_t id) {
    if (id >= MOTOR_ID_MAX) return 0;
    return &s_cfg[id];
}

uint32_t motor_registry_count(void) { return (uint32_t)MOTOR_ID_MAX; }

app_err_t motor_registry_bind(motor_logical_id_t id, motor_dev_t* dev) {
    if (id >= MOTOR_ID_MAX) return APP_ERR_INVALID_ARG;
    s_devs[id] = dev;
    if (dev) {
        dev->state.id      = (uint16_t)id;
        dev->state.type    = s_cfg[id].type;
        dev->state.can_bus = s_cfg[id].can_bus;
        dev->state.can_id  = s_cfg[id].can_id;
    }
    return APP_OK;
}
