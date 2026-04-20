/*
 * pid.h
 *
 *  Created on: Apr 18, 2026
 *      Author: FMI
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

typedef struct
{
    float Kp;                 /* 比例增益：误差越大，比例项输出越大。 */
    float Ki;                 /* 积分增益：用于消除稳态误差。 */
    float Kd;                 /* 微分增益：用于抑制变化过快和超调。 */

    float integral;           /* 积分累计值，单位是 error * second。 */
    float prev_error;         /* 上一次误差，用于误差微分和增量式 PID。 */
    float prev_prev_error;    /* 上上次误差，只给增量式 PID 使用。 */
    float prev_measurement;   /* 上一次测量值，用于测量值微分，减少设定值突变冲击。 */

    float output;             /* 当前输出值。 */
    float output_max;         /* 输出上限。 */
    float output_min;         /* 输出下限。 */

    float integral_max;       /* 积分累计上限，用于抗积分饱和。 */
    float integral_min;       /* 积分累计下限，用于抗积分饱和。 */

    uint32_t last_time_ms;    /* 上一次更新时间，单位 ms，供 PID_Update() 使用。 */
    uint8_t initialized;      /* 首次运行标志，避免第一帧微分项突跳。 */
} PID_Controller;

void PID_Init(PID_Controller *pid,
              float Kp,
              float Ki,
              float Kd,
              float output_max,
              float output_min,
              float integral_max,
              float integral_min);

void PID_Reset(PID_Controller *pid);

void PID_SetTunings(PID_Controller *pid, float Kp, float Ki, float Kd);
void PID_SetOutputLimits(PID_Controller *pid, float output_max, float output_min);
void PID_SetIntegralLimits(PID_Controller *pid, float integral_max, float integral_min);

float PID_Update(PID_Controller *pid,
                 float setpoint,
                 float measurement,
                 uint32_t current_time_ms);

float PID_UpdateDt(PID_Controller *pid,
                   float setpoint,
                   float measurement,
                   float dt_s);

float PID_UpdateIncremental(PID_Controller *pid,
                            float setpoint,
                            float measurement,
                            uint32_t current_time_ms);

float PID_UpdateIncrementalDt(PID_Controller *pid,
                              float setpoint,
                              float measurement,
                              float dt_s);

/* 兼容参考文件里的旧函数名。 */
float PID_Update_Incremental(PID_Controller *pid,
                             float setpoint,
                             float measurement,
                             uint32_t current_time_ms);

#endif /* INC_PID_H_ */
