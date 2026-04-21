/*
 * service/pid/pid.h — host-可编译的 PID（位置式 + 增量式）
 *
 * 与 Core/Inc/pid.h 接口保持一致；待 M2 阶段 Core 旧版本完全下线后，
 * 由 task_chassis 等上层改为 include 本头文件。
 */
#ifndef APP_SERVICE_PID_H_
#define APP_SERVICE_PID_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float prev_prev_error;
    float prev_measurement;
    float output;
    float output_max, output_min;
    float integral_max, integral_min;
    uint32_t last_time_ms;
    uint8_t  initialized;
} app_pid_t;

void  app_pid_init(app_pid_t* p,
                   float Kp, float Ki, float Kd,
                   float out_max, float out_min,
                   float i_max,   float i_min);
void  app_pid_reset(app_pid_t* p);
void  app_pid_set_tunings(app_pid_t* p, float Kp, float Ki, float Kd);
void  app_pid_set_output_limits(app_pid_t* p, float out_max, float out_min);
void  app_pid_set_integral_limits(app_pid_t* p, float i_max, float i_min);

float app_pid_update_dt(app_pid_t* p, float setpoint, float measurement, float dt_s);
float app_pid_update_ms(app_pid_t* p, float setpoint, float measurement, uint32_t now_ms);

float app_pid_update_inc_dt(app_pid_t* p, float setpoint, float measurement, float dt_s);
float app_pid_update_inc_ms(app_pid_t* p, float setpoint, float measurement, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_PID_H_ */
