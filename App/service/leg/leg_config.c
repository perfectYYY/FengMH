/*
 * leg_config.c — 四腿真实装配配置
 */
#include "leg_config.h"

/*
 * 腿型约定来自原始机械装配:
 *   ORIGINAL: FR / RL
 *   MIRROR:   FL / RR
 *
 * foot_x_dir 是真实装配的 x 镜像符号:
 *   IK: body-frame dx -> local leg x
 *   FK: local leg x -> body-frame dx
 * 如果这里错，PC 显示和板端步态都会左右/前后反。
 *
 * body_x/body_y 用于 PC 可视化中摆放四条腿。
 * 如果后续实测机体尺寸变化，改这里即可同步固件和 PC 工具。
 */
static const leg_config_t S_LEG_CONFIG[GAIT_LEG_NUM] = {
    {
        .leg = GAIT_LEG_FL,
        .name = "FL",
        .label = "front-left",
        .leg_type = LEG_TYPE_MIRROR,
        .motor = { MOTOR_ID_FL_HIP, MOTOR_ID_FL_KNEE, MOTOR_ID_FL_WHEEL },
        .body_x_m = +0.220f,
        .body_y_m = +0.120f,
        .foot_x_dir = -1.0f,
    },
    {
        .leg = GAIT_LEG_FR,
        .name = "FR",
        .label = "front-right",
        .leg_type = LEG_TYPE_ORIGINAL,
        .motor = { MOTOR_ID_FR_HIP, MOTOR_ID_FR_KNEE, MOTOR_ID_FR_WHEEL },
        .body_x_m = +0.220f,
        .body_y_m = -0.120f,
        .foot_x_dir = +1.0f,
    },
    {
        .leg = GAIT_LEG_RL,
        .name = "RL",
        .label = "rear-left",
        .leg_type = LEG_TYPE_ORIGINAL,
        .motor = { MOTOR_ID_RL_HIP, MOTOR_ID_RL_KNEE, MOTOR_ID_RL_WHEEL },
        .body_x_m = -0.220f,
        .body_y_m = +0.120f,
        .foot_x_dir = -1.0f,
    },
    {
        .leg = GAIT_LEG_RR,
        .name = "RR",
        .label = "rear-right",
        .leg_type = LEG_TYPE_MIRROR,
        .motor = { MOTOR_ID_RR_HIP, MOTOR_ID_RR_KNEE, MOTOR_ID_RR_WHEEL },
        .body_x_m = -0.220f,
        .body_y_m = -0.120f,
        .foot_x_dir = +1.0f,
    },
};

uint32_t leg_config_count(void) {
    return (uint32_t)GAIT_LEG_NUM;
}

const leg_config_t* leg_config_get(gait_leg_t leg) {
    if (leg < 0 || leg >= GAIT_LEG_NUM) return 0;
    return &S_LEG_CONFIG[leg];
}

const char* leg_config_type_name(leg_type_t type) {
    return (type == LEG_TYPE_MIRROR) ? "MIRROR" : "ORIGINAL";
}

const char* leg_config_motor_role_name(leg_actuator_role_t role) {
    switch (role) {
    case LEG_ACT_HIP:   return "hip";
    case LEG_ACT_KNEE:  return "knee";
    case LEG_ACT_WHEEL: return "wheel";
    default:            return "unknown";
    }
}
