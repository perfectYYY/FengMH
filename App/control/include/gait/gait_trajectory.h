/*
 * gait_trajectory.h - 步态通用足端轨迹工具
 *
 * 这里放与具体步态相位编排无关的数学：单腿支撑/摆动轨迹、
 * 每腿步长解析等。trot/walk 只负责决定每条腿当前处于哪个相位。
 */
#ifndef APP_SERVICE_GAIT_TRAJECTORY_H_
#define APP_SERVICE_GAIT_TRAJECTORY_H_

#include "gait_if.h"

#ifdef __cplusplus
extern "C" {
#endif

float gait_leg_side_sign(int leg_idx);
uint8_t gait_params_has_per_leg_steps(const gait_params_t* p);
float gait_resolve_leg_step_length(const gait_params_t* p, int leg_idx);

void gait_cycloid_foot_traj(float leg_phase,
                            float duty,
                            float step_len_m,
                            float step_height_m,
                            float* dx_m,
                            float* dz_m,
                            uint8_t* in_stance);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_GAIT_TRAJECTORY_H_ */
