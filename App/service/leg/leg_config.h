/*
 * leg_config.h — 四腿真实装配配置
 *
 * 这张表是固件与 PC 调试工具共享的单一真源：
 *   - 四条腿顺序
 *   - 腿部构型 ORIGINAL / MIRROR
 *   - hip / knee / wheel 逻辑电机对应关系
 *   - 机体坐标中的腿根大致位置
 *   - 足端局部 x 轴和机体 x 轴之间的镜像方向
 */
#ifndef APP_SERVICE_LEG_CONFIG_H_
#define APP_SERVICE_LEG_CONFIG_H_

#include "gait_if.h"
#include "leg_ik.h"
#include "motor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LEG_ACT_HIP = 0,
    LEG_ACT_KNEE,
    LEG_ACT_WHEEL,
    LEG_ACT_NUM
} leg_actuator_role_t;

typedef struct {
    gait_leg_t          leg;
    const char*         name;       /* FL / FR / RL / RR */
    const char*         label;      /* readable corner name */
    leg_type_t          leg_type;
    motor_logical_id_t  motor[LEG_ACT_NUM];
    float               body_x_m;   /* front +, rear - */
    float               body_y_m;   /* left +, right - */
    float               foot_x_dir; /* body x <-> local leg x mirror sign: +1 / -1 */
} leg_config_t;

uint32_t            leg_config_count(void);
const leg_config_t* leg_config_get(gait_leg_t leg);
const char*         leg_config_type_name(leg_type_t type);
const char*         leg_config_motor_role_name(leg_actuator_role_t role);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_LEG_CONFIG_H_ */
