/*
 * gait_plan.h
 *
 *  Created on: Dec 29, 2025
 *      Author: FMI
 */

#ifndef INC_GAIT_PLAN_H_
#define INC_GAIT_PLAN_H_


#include "arm_math.h"

#define MAX_GAIT_POINT_NUMBER 100



/* 两种腿型：
 * 1. ORIGINAL: 当前工程里已经写好的原型腿，实际对应左后腿和右前腿
 * 2. MIRROR:   原型腿的镜像版本，实际对应左前腿和右后腿
 */
typedef enum
{
    LEG_TYPE_ORIGINAL = 0,
    LEG_TYPE_MIRROR = 1
} quadruped_leg_type;

// 定义一个表示范围的结构体
typedef struct
{
    float min;
    float max;
} range;

// 轨迹点的向量形式存储
typedef struct
{
    arm_matrix_instance_f32 vector;      // float thigh_length;向量 [ ,  ,  ]^T
    float data[3];            // 数据存储
} vector;

// 机械腿尺寸参数，同时也保留电机角度范围、质量和质心位置
typedef struct
{
    float thigh_length;                 // 大腿全长 (m)
    float shin_length;                  // 小腿全长 (m)
    float link_length;                  // 小腿电机直连连杆长度 (m)
    float wheel_diameter;               // 轮子直径 (m)
    range thigh_motor_angle_range;      // 大腿电机角度范围
    range shin_motor_angle_range;       // 小腿电机角度范围

    float thigh_mass_1;                 // 大腿1质量 (kg)
    float thigh_mass_2;                 // 大腿2质量 (kg)
    float shin_mass;                    // 小腿质量 (kg)
    float link_mass;                    // 小腿电机直连连杆质量 (kg)
    float wheel_mass;                   // 轮子质量 (kg)

    float lc_t_m_1;                     // 大腿1质心距离
    float lc_t_m_2;                     // 大腿2质心距离
    float lc_s_m;                       // 小腿质心距离
    float lc_l_m;                       // 连杆质心距离

    float g;                            // 重力加速度
} leg_size;

// 步态参数结构体
typedef struct 
{
	float radius;		  // 摆线轨迹半径
	float duty_ratio;	  // 占空比 = 支撑相 / 总周期时间
	float cycle_time;	 // 周期时间
	float point_number;	 // 轨迹点数量
	vector position[MAX_GAIT_POINT_NUMBER];  // 位置
	vector velocity[MAX_GAIT_POINT_NUMBER];  // 速度
	vector acceleration[MAX_GAIT_POINT_NUMBER];  // 加速度

} gait_action;

void params_init(leg_size *leg_size, float thigh_length, float shin_length, float link_length,
                 float angle1_max, float angle1_min, float angle2_max, float angle2_min, float wheel_diameter,
                 float thigh_mass_1, float thigh_mass_2, float shin_mass, float link_mass, float wheel_mass,
                 float lc_t_m_1, float lc_t_m_2, float lc_s_m, float lc_l_m, float g);

void gait_action_init(gait_action *gait_action, float radius, float duty_ratio, float cycle_time, float point_number);
void compete_gait(gait_action *gait_action);
void gait_record(gait_action *gait_action, int indx, float x, float y, float z, float vx, float vy, float vz, float ax, float ay, float az);
void inverse_kinematics_position(leg_size *leg_size, vector *position, vector *motor_angle, float hight, quadruped_leg_type leg_type);

#endif /* INC_GAIT_PLAN_H_ */
