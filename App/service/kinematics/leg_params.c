/*
 * leg_params.c — 默认腿尺寸参数
 *
 * 从 Core/Src/gait_plan.c → params_init() 迁移。
 * 数值需根据实际机械设计标定后更新。
 */
#include "leg_params.h"

/*
 * 默认参数来自老工程 Wheel-legged/Core/Src/main.c 中的
 * params_init(&leg_after, ...)，保持 PC host 仿真和板端 IK 使用同一套
 * 等效二连杆尺寸。
 */
const leg_dim_t LEG_DIM_DEFAULT = {
    /* 长度 (m) */
    .thigh_length   = 0.100f,
    .shin_length    = 0.150f,
    .link_length    = 0.040f,
    .wheel_diameter = 0.095f,

    /* 角度范围 (rad) */
    .thigh_angle_range = { .min = -3.1415926f, .max = 0.0f },
    .shin_angle_range  = { .min = -2.3561945f, .max = 0.0f },

    /* 质量 (kg) */
    .thigh_mass_1   = 0.042f,
    .thigh_mass_2   = 0.012f,
    .shin_mass      = 0.090f,
    .link_mass      = 0.008f,
    .wheel_mass     = 0.550f,

    /* 质心距离 (m) */
    .lc_t_m_1       = 0.051f,
    .lc_t_m_2       = 0.050f,
    .lc_s_m         = 0.056f,
    .lc_l_m         = 0.010f,

    /* 重力加速度 (m/s²) */
    .g              = 9.8f,
};
