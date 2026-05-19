/*
 * leg_ik.h — 2 连杆逆运动学 / 正运动学
 *
 * 从 Core/Src/gait_plan.c → inverse_kinematics_position() / forward_kinematics_position() 迁移。
 * 去除 arm_math 依赖，使用纯 float 运算。
 *
 * 腿型说明：
 *   ORIGINAL: 原型腿 (左后 RL / 右前 FR)，atan2(z,x) - theta
 *   MIRROR:   镜像腿 (左前 FL / 右后 RR)，atan2(z,x) + theta
 */
#ifndef APP_SERVICE_KINEMATICS_LEG_IK_H_
#define APP_SERVICE_KINEMATICS_LEG_IK_H_

#include "leg_params.h"
#include "gait_if.h"    /* gait_output_t, gait_leg_target_t */

#ifdef __cplusplus
extern "C" {
#endif

/* 腿型 (与旧 gait_plan.h quadruped_leg_type 对齐) */
typedef enum {
    LEG_TYPE_ORIGINAL = 0,   /* 原型腿: RL, FR */
    LEG_TYPE_MIRROR   = 1,   /* 镜像腿: FL, RR */
} leg_type_t;

/* IK 输出：关节角度 (rad) */
typedef struct {
    float theta1;   /* 髋关节角度 */
    float theta2;   /* 膝关节角度 */
} leg_ik_result_t;

/* FK 输出：足端位置 (m)，相对于髋关节 */
typedef struct {
    float x;   /* 水平方向 */
    float z;   /* 竖直方向 (向下为正) */
} leg_fk_result_t;

/*
 * 逆运动学：足端位置 → 关节角度
 *
 * 参数：
 *   x, z    - 足端相对于髋关节的位置 (m)
 *   dim     - 腿尺寸参数
 *   hight   - 站立高度偏移 (m)，加在 z 上
 *   leg_type - 腿型 (ORIGINAL/MIRROR)
 *   result  - 输出关节角度
 *
 * 返回：0 成功, -1 无解 (超出工作空间)
 */
int leg_ik_solve(float x, float z, const leg_dim_t* dim,
                  float hight, leg_type_t leg_type,
                  leg_ik_result_t* result);

/*
 * 正运动学：关节角度 → 足端位置
 *
 * 参数：
 *   theta1  - 髋关节角度 (rad)
 *   theta2  - 膝关节角度 (rad)
 *   dim     - 腿尺寸参数
 *   result  - 输出足端位置
 */
void leg_fk_solve(float theta1, float theta2,
                   const leg_dim_t* dim,
                   leg_fk_result_t* result);

/*
 * 批量 IK：对 4 条腿执行 IK
 *
 * 输入 gait_output_t 中 hip_rad/knee_rad 临时携带 (dx, dz) 足端位移，
 * 输出替换为 (theta1, theta2) 关节角度。
 *
 * 参数：
 *   foot_disp - 足端位移 gait_output (hip_rad=dx, knee_rad=dz)
 *   dim       - 腿尺寸参数
 *   hight     - 站立高度 (m)
 *   out       - 输出关节角度 gait_output
 */
void leg_ik_solve_all(const gait_output_t* foot_disp,
                       const leg_dim_t* dim,
                       float hight,
                       gait_output_t* out);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_KINEMATICS_LEG_IK_H_ */
