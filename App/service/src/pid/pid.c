/*
 * service/src/pid/pid.c — 与 Core/Src/pid.c 等价；零外设依赖，host 可编译。
 */
#include "pid/pid.h"

#define PID_DEFAULT_DT_S       0.01f
#define PID_MAX_REASONABLE_DT  0.50f
#define PID_MIN_REASONABLE_DT  0.000001f

static float clamp_f(float v, float lo, float hi) {
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

static void sort_limits(float* hi, float* lo) {
    if (*hi < *lo) { float t = *hi; *hi = *lo; *lo = t; }
}

static float normalize_dt(float dt_s) {
    if (dt_s < PID_MIN_REASONABLE_DT) return PID_DEFAULT_DT_S;
    if (dt_s > PID_MAX_REASONABLE_DT) return PID_MAX_REASONABLE_DT;
    return dt_s;
}

static float dt_from_ms(app_pid_t* p, uint32_t now_ms) {
    if (p->last_time_ms == 0U) {
        p->last_time_ms = now_ms;
        return PID_DEFAULT_DT_S;
    }
    uint32_t d = now_ms - p->last_time_ms;
    p->last_time_ms = now_ms;
    return normalize_dt((float)d / 1000.0f);
}

void app_pid_init(app_pid_t* p, float Kp, float Ki, float Kd,
                  float out_max, float out_min, float i_max, float i_min) {
    if (!p) return;
    p->Kp = Kp; p->Ki = Ki; p->Kd = Kd;
    p->output_max = out_max; p->output_min = out_min;
    sort_limits(&p->output_max, &p->output_min);
    p->integral_max = i_max; p->integral_min = i_min;
    sort_limits(&p->integral_max, &p->integral_min);
    app_pid_reset(p);
}

void app_pid_reset(app_pid_t* p) {
    if (!p) return;
    p->integral = p->prev_error = p->prev_prev_error = 0.0f;
    p->prev_measurement = 0.0f;
    p->output = 0.0f;
    p->last_time_ms = 0U;
    p->initialized = 0U;
}

void app_pid_set_tunings(app_pid_t* p, float Kp, float Ki, float Kd) {
    if (!p) return;
    p->Kp = Kp; p->Ki = Ki; p->Kd = Kd;
}

void app_pid_set_output_limits(app_pid_t* p, float out_max, float out_min) {
    if (!p) return;
    p->output_max = out_max; p->output_min = out_min;
    sort_limits(&p->output_max, &p->output_min);
    p->output = clamp_f(p->output, p->output_min, p->output_max);
}

void app_pid_set_integral_limits(app_pid_t* p, float i_max, float i_min) {
    if (!p) return;
    p->integral_max = i_max; p->integral_min = i_min;
    sort_limits(&p->integral_max, &p->integral_min);
    p->integral = clamp_f(p->integral, p->integral_min, p->integral_max);
}

float app_pid_update_dt(app_pid_t* p, float setpoint, float measurement, float dt_s) {
    if (!p) return 0.0f;
    dt_s = normalize_dt(dt_s);
    float err = setpoint - measurement;
    if (!p->initialized) {
        p->prev_measurement = measurement;
        p->prev_error = err;
        p->prev_prev_error = err;
        p->initialized = 1U;
    }
    float prop = p->Kp * err;
    p->integral += err * dt_s;
    p->integral = clamp_f(p->integral, p->integral_min, p->integral_max);
    float inte = p->Ki * p->integral;
    /* 对测量值微分（d-on-measurement），避免设定值阶跃造成 D 冲击 */
    float deri = p->Kd * (measurement - p->prev_measurement) / dt_s;
    p->output = prop + inte - deri;
    p->output = clamp_f(p->output, p->output_min, p->output_max);
    p->prev_prev_error = p->prev_error;
    p->prev_error = err;
    p->prev_measurement = measurement;
    return p->output;
}

float app_pid_update_ms(app_pid_t* p, float setpoint, float measurement, uint32_t now_ms) {
    if (!p) return 0.0f;
    return app_pid_update_dt(p, setpoint, measurement, dt_from_ms(p, now_ms));
}

float app_pid_update_inc_dt(app_pid_t* p, float setpoint, float measurement, float dt_s) {
    if (!p) return 0.0f;
    dt_s = normalize_dt(dt_s);
    float err = setpoint - measurement;
    if (!p->initialized) {
        p->prev_error = err;
        p->prev_prev_error = err;
        p->prev_measurement = measurement;
        p->initialized = 1U;
    }
    float d_out = p->Kp * (err - p->prev_error)
                + p->Ki * err * dt_s
                + p->Kd * (err - 2.0f * p->prev_error + p->prev_prev_error) / dt_s;
    p->output += d_out;
    p->output = clamp_f(p->output, p->output_min, p->output_max);
    p->prev_prev_error = p->prev_error;
    p->prev_error = err;
    p->prev_measurement = measurement;
    return p->output;
}

float app_pid_update_inc_ms(app_pid_t* p, float setpoint, float measurement, uint32_t now_ms) {
    if (!p) return 0.0f;
    return app_pid_update_inc_dt(p, setpoint, measurement, dt_from_ms(p, now_ms));
}
