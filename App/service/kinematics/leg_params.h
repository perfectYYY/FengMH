/*
 * leg_params.h — 腿尺寸/质量/质心参数
 *
 * 从 Core/Src/gait_plan.c → params_init() / leg_size 迁移。
 * 去除 arm_math 依赖，使用纯 float 结构体。
 */
#ifndef APP_SERVICE_KINEMATICS_LEG_PARAMS_H_
#define APP_SERVICE_KINEMATICS_LEG_PARAMS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 角度范围 */
typedef struct {
    float min;
    float max;
} leg_angle_range_t;

/* 腿尺寸参数 (与真实平行连杆结构对齐) */
typedef struct {
    /* 长度 (m) */
    float thigh_length;     /* 髋轴到上膝点；同时也是 100mm 从动连杆 */
    float shin_length;      /* 上膝点到轮轴/足端 */
    float link_length;      /* 膝电机 40mm 摇臂；同时也是小腿安装点偏置 */
    float wheel_diameter;   /* 轮直径 */

    /* 角度范围 (rad) */
    leg_angle_range_t thigh_angle_range;   /* 髋关节 */
    leg_angle_range_t shin_angle_range;    /* 膝关节 */

    /* 质量 (kg) */
    float thigh_mass_1;     /* 大腿1质量 */
    float thigh_mass_2;     /* 大腿2质量 */
    float shin_mass;        /* 小腿质量 */
    float link_mass;        /* 连杆质量 */
    float wheel_mass;       /* 轮质量 */

    /* 质心距离 (m) */
    float lc_t_m_1;         /* 大腿1质心距离 */
    float lc_t_m_2;         /* 大腿2质心距离 */
    float lc_s_m;           /* 小腿质心距离 */
    float lc_l_m;           /* 连杆质心距离 */

    /* 重力加速度 (m/s²) */
    float g;
} leg_dim_t;

/* 默认腿尺寸参数 (根据实际机械设计填写) */
extern const leg_dim_t LEG_DIM_DEFAULT;

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_KINEMATICS_LEG_PARAMS_H_ */
