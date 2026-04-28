/*
 * motor_registry.c
 *
 * 改动硬件映射只需改本文件，不需要动算法
 */
#include "motor_registry.h"
#include "log.h"

#include <string.h>

static const char* TAG = "REG";

static const motor_cfg_t s_cfg[MOTOR_ID_MAX] = {
    /* FL */
    { MOTOR_ID_FL_HIP,   MOTOR_GO,    1, 0x00, +1, 0, -3.14f, +3.14f, "FL_HIP" },
    { MOTOR_ID_FL_KNEE,  MOTOR_GO,    1, 0x00, +1, 0, -3.14f, +3.14f, "FL_KNEE" },
    { MOTOR_ID_FL_WHEEL, MOTOR_M3508, 1, 0x201,+1, 0,  0.0f,   0.0f, "FL_WHEEL" },
    /* FR */
    { MOTOR_ID_FR_HIP,   MOTOR_GO,    1, 0x00, -1, 0, -3.14f, +3.14f, "FR_HIP" },
    { MOTOR_ID_FR_KNEE,  MOTOR_GO,    1, 0x00, -1, 0, -3.14f, +3.14f, "FR_KNEE" },
    { MOTOR_ID_FR_WHEEL, MOTOR_M3508, 1, 0x202,-1, 0,  0.0f,   0.0f, "FR_WHEEL" },
    /* RL */
    { MOTOR_ID_RL_HIP,   MOTOR_GO,    2, 0x00, +1, 0, -3.14f, +3.14f, "RL_HIP" },
    { MOTOR_ID_RL_KNEE,  MOTOR_GO,    2, 0x00, +1, 0, -3.14f, +3.14f, "RL_KNEE" },
    { MOTOR_ID_RL_WHEEL, MOTOR_M3508, 2, 0x203,+1, 0,  0.0f,   0.0f, "RL_WHEEL" },
    /* RR */
    { MOTOR_ID_RR_HIP,   MOTOR_GO,    2, 0x00, -1, 0, -3.14f, +3.14f, "RR_HIP" },
    { MOTOR_ID_RR_KNEE,  MOTOR_GO,    2, 0x00, -1,  0, -3.14f, +3.14f, "RR_KNEE" },
    { MOTOR_ID_RR_WHEEL, MOTOR_M3508, 2, 0x204,-1, 0,  0.0f,   0.0f, "RR_WHEEL" },
    /* ARM */
    { MOTOR_ID_ARM_J1,   MOTOR_DAMIAO, 3, 0x00, +1, 0, -3.14f, +3.14f, "ARM_J1" },
    { MOTOR_ID_ARM_J2,   MOTOR_DAMIAO, 3, 0x00, +1, 0, -3.14f, +3.14f, "ARM_J2" },
    { MOTOR_ID_ARM_J3,   MOTOR_DAMIAO, 3, 0x00, +1, 0, -3.14f, +3.14f, "ARM_J3" },
    { MOTOR_ID_ARM_J4,   MOTOR_DAMIAO, 3, 0x00, +1, 0, -3.14f, +3.14f, "ARM_J4" },
    { MOTOR_ID_ARM_J5,   MOTOR_M3508,  3, 0x205,+1, 0,  0.0f,   0.0f, "ARM_J5" },
    { MOTOR_ID_ARM_J6,   MOTOR_M3508,  3, 0x206,+1, 0,  0.0f,   0.0f, "ARM_J6" },
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
