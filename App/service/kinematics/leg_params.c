/*
 * leg_params.c — 默认腿尺寸参数
 *
 * 从 Core/Src/gait_plan.c → params_init() 迁移。
 * 数值需根据实际机械设计标定后更新。
 */
#include "leg_params.h"

/*
 * 默认参数：根据实际机器狗机械设计填写。
 * 当前为占位值，需要实测后修正。
 */
const leg_dim_t LEG_DIM_DEFAULT = {
    /* 长度 (m) */
    .thigh_length   = 0.210f,
    .shin_length    = 0.210f,
    .link_length    = 0.060f,
    .wheel_diameter = 0.120f,

    /* 角度范围 (rad) */
    .thigh_angle_range = { .min = -1.57f, .max = 1.57f },
    .shin_angle_range  = { .min = -2.09f, .max = 2.09f },

    /* 质量 (kg) */
    .thigh_mass_1   = 0.500f,
    .thigh_mass_2   = 0.300f,
    .shin_mass      = 0.400f,
    .link_mass      = 0.100f,
    .wheel_mass     = 0.300f,

    /* 质心距离 (m) */
    .lc_t_m_1       = 0.105f,
    .lc_t_m_2       = 0.080f,
    .lc_s_m         = 0.105f,
    .lc_l_m         = 0.030f,

    /* 重力加速度 (m/s²) */
    .g              = 9.81f,
};
