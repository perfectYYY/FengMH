/*
 * motor_registry.h — 电机注册表
 *
 * 通过"逻辑名 → 物理总线/CAN ID/类型"映射，应用层只拿 motor_dev_t* 操作。
 * M1 阶段只提供接口与空表骨架，具体填表留给后续实测时补齐。
 */
#ifndef APP_DEVICE_MOTOR_REGISTRY_H_
#define APP_DEVICE_MOTOR_REGISTRY_H_

#include "motor_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 逻辑名：约定命名，未真正启用的保留；实际上线前按实测硬件顺序对齐 */
typedef enum {
    /* 四足：左前 FL / 右前 FR / 左后 RL / 右后 RR，每腿髋/膝/轮 */
    MOTOR_ID_FL_HIP = 0,
    MOTOR_ID_FL_KNEE,
    MOTOR_ID_FL_WHEEL,
    MOTOR_ID_FR_HIP,
    MOTOR_ID_FR_KNEE,
    MOTOR_ID_FR_WHEEL,
    MOTOR_ID_RL_HIP,
    MOTOR_ID_RL_KNEE,
    MOTOR_ID_RL_WHEEL,
    MOTOR_ID_RR_HIP,
    MOTOR_ID_RR_KNEE,
    MOTOR_ID_RR_WHEEL,
    /* 机械臂预留 */
    MOTOR_ID_ARM_J1,
    MOTOR_ID_ARM_J2,
    MOTOR_ID_ARM_J3,
    MOTOR_ID_ARM_J4,
    MOTOR_ID_ARM_J5,
    MOTOR_ID_ARM_J6,
    MOTOR_ID_MAX
} motor_logical_id_t;

typedef struct {
    motor_logical_id_t logical;
    motor_type_t       type;
    uint8_t            can_bus;
    uint32_t           can_id;
    int8_t             dir;          /* +1 / -1 */
    float              zero_offset;  /* rad */
    float              limit_min;    /* rad */
    float              limit_max;    /* rad */
    const char*        name;
} motor_cfg_t;

app_err_t    motor_registry_init(void);
motor_dev_t* motor_get(motor_logical_id_t id);
const motor_cfg_t* motor_get_cfg(motor_logical_id_t id);
uint32_t     motor_registry_count(void);

/* 测试/调试：允许运行期注册 device（host 单测用 stub 驱动） */
app_err_t    motor_registry_bind(motor_logical_id_t id, motor_dev_t* dev);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEVICE_MOTOR_REGISTRY_H_ */
