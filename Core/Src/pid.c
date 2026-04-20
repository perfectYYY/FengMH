/*
 * pid.c
 *
 *  Created on: Apr 18, 2026
 *      Author: FMI
 */

#include "pid.h"

#define PID_DEFAULT_DT_S       0.01f
#define PID_MAX_REASONABLE_DT  0.50f
#define PID_MIN_REASONABLE_DT  0.000001f

static float PID_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

static void PID_SortLimits(float *max_value, float *min_value)
{
    if (*max_value < *min_value)
    {
        float temp = *max_value;
        *max_value = *min_value;
        *min_value = temp;
    }
}

static float PID_NormalizeDt(float dt_s)
{
    if (dt_s < PID_MIN_REASONABLE_DT)
    {
        return PID_DEFAULT_DT_S;
    }

    if (dt_s > PID_MAX_REASONABLE_DT)
    {
        return PID_MAX_REASONABLE_DT;
    }

    return dt_s;
}

static float PID_GetDtFromMs(PID_Controller *pid, uint32_t current_time_ms)
{
    uint32_t dt_ms;

    if (pid->last_time_ms == 0U)
    {
        pid->last_time_ms = current_time_ms;
        return PID_DEFAULT_DT_S;
    }

    /* uint32_t 自然溢出可以兼容 HAL_GetTick() 回绕。 */
    dt_ms = current_time_ms - pid->last_time_ms;
    pid->last_time_ms = current_time_ms;

    return PID_NormalizeDt((float)dt_ms / 1000.0f);
}

void PID_Init(PID_Controller *pid,
              float Kp,
              float Ki,
              float Kd,
              float output_max,
              float output_min,
              float integral_max,
              float integral_min)
{
    if (pid == 0)
    {
        return;
    }

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->output_max = output_max;
    pid->output_min = output_min;
    PID_SortLimits(&pid->output_max, &pid->output_min);

    pid->integral_max = integral_max;
    pid->integral_min = integral_min;
    PID_SortLimits(&pid->integral_max, &pid->integral_min);

    PID_Reset(pid);
}

void PID_Reset(PID_Controller *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->output = 0.0f;
    pid->last_time_ms = 0U;
    pid->initialized = 0U;
}

void PID_SetTunings(PID_Controller *pid, float Kp, float Ki, float Kd)
{
    if (pid == 0)
    {
        return;
    }

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_SetOutputLimits(PID_Controller *pid, float output_max, float output_min)
{
    if (pid == 0)
    {
        return;
    }

    pid->output_max = output_max;
    pid->output_min = output_min;
    PID_SortLimits(&pid->output_max, &pid->output_min);

    /* 修改限幅后，立刻把已有输出夹回合法范围。 */
    pid->output = PID_Clamp(pid->output, pid->output_min, pid->output_max);
}

void PID_SetIntegralLimits(PID_Controller *pid, float integral_max, float integral_min)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral_max = integral_max;
    pid->integral_min = integral_min;
    PID_SortLimits(&pid->integral_max, &pid->integral_min);
    pid->integral = PID_Clamp(pid->integral, pid->integral_min, pid->integral_max);
}

float PID_Update(PID_Controller *pid,
                 float setpoint,
                 float measurement,
                 uint32_t current_time_ms)
{
    if (pid == 0)
    {
        return 0.0f;
    }

    return PID_UpdateDt(pid, setpoint, measurement, PID_GetDtFromMs(pid, current_time_ms));
}

float PID_UpdateDt(PID_Controller *pid,
                   float setpoint,
                   float measurement,
                   float dt_s)
{
    float error;
    float proportional;
    float integral_term;
    float derivative;

    if (pid == 0)
    {
        return 0.0f;
    }

    dt_s = PID_NormalizeDt(dt_s);
    error = setpoint - measurement;

    if (pid->initialized == 0U)
    {
        pid->prev_measurement = measurement;
        pid->prev_error = error;
        pid->prev_prev_error = error;
        pid->initialized = 1U;
    }

    proportional = pid->Kp * error;

    /* 积分先按时间累加，再限幅，防止长时间饱和后恢复很慢。 */
    pid->integral += error * dt_s;
    pid->integral = PID_Clamp(pid->integral, pid->integral_min, pid->integral_max);
    integral_term = pid->Ki * pid->integral;

    /*
     * 对测量值做微分，而不是对误差做微分。
     * 这样目标值突然改变时，不会把设定值阶跃直接放大到 D 项里。
     */
    derivative = pid->Kd * (measurement - pid->prev_measurement) / dt_s;

    pid->output = proportional + integral_term - derivative;
    pid->output = PID_Clamp(pid->output, pid->output_min, pid->output_max);

    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    return pid->output;
}

float PID_UpdateIncremental(PID_Controller *pid,
                            float setpoint,
                            float measurement,
                            uint32_t current_time_ms)
{
    if (pid == 0)
    {
        return 0.0f;
    }

    return PID_UpdateIncrementalDt(pid, setpoint, measurement, PID_GetDtFromMs(pid, current_time_ms));
}

float PID_UpdateIncrementalDt(PID_Controller *pid,
                              float setpoint,
                              float measurement,
                              float dt_s)
{
    float error;
    float delta_output;

    if (pid == 0)
    {
        return 0.0f;
    }

    dt_s = PID_NormalizeDt(dt_s);
    error = setpoint - measurement;

    if (pid->initialized == 0U)
    {
        pid->prev_error = error;
        pid->prev_prev_error = error;
        pid->prev_measurement = measurement;
        pid->initialized = 1U;
    }

    /*
     * 增量式 PID 输出的是本周期相对上一周期的变化量。
     * 这里显式保存 prev_prev_error，避免参考文件里 prev_measure 命名混用造成误读。
     */
    delta_output = pid->Kp * (error - pid->prev_error) +
                   pid->Ki * error * dt_s +
                   pid->Kd * (error - 2.0f * pid->prev_error + pid->prev_prev_error) / dt_s;

    pid->output += delta_output;
    pid->output = PID_Clamp(pid->output, pid->output_min, pid->output_max);

    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    return pid->output;
}

float PID_Update_Incremental(PID_Controller *pid,
                             float setpoint,
                             float measurement,
                             uint32_t current_time_ms)
{
    return PID_UpdateIncremental(pid, setpoint, measurement, current_time_ms);
}
