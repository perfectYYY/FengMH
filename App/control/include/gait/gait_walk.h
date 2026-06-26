/*
 * gait_walk.h - 四拍 walk：任意时刻一腿摆动、三腿支撑
 */
#ifndef APP_SERVICE_GAIT_WALK_H_
#define APP_SERVICE_GAIT_WALK_H_

#include "gait_if.h"

#ifdef __cplusplus
extern "C" {
#endif

gait_if_t* gait_walk_create(void);

/* 暴露给单测：walk 与 trot 使用同一套单腿足端轨迹。 */
void gait_walk_foot_traj(float leg_phase,
                         float duty,
                         float step_len_m,
                         float step_height_m,
                         float* dx_m,
                         float* dz_m,
                         uint8_t* in_stance);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_GAIT_WALK_H_ */
