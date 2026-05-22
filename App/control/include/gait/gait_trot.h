/*
 * gait_trot.h — 对角小跑：FL+RR 同相，FR+RL 反相
 */
#ifndef APP_SERVICE_GAIT_TROT_H_
#define APP_SERVICE_GAIT_TROT_H_

#include "gait_if.h"

#ifdef __cplusplus
extern "C" {
#endif

gait_if_t* gait_trot_create(void);

/* 暴露给单测：根据腿相位 [0,1) + 参数计算"足端相对躯干 (x,z) 摆动量"
 * (摆动相走摆线，支撑相走直线后退) */
void gait_trot_foot_traj(float leg_phase,
                         float duty,
                         float step_len_m,
                         float step_height_m,
                         float* dx_m,
                         float* dz_m,
                         uint8_t* in_stance);

#ifdef __cplusplus
}
#endif

#endif
