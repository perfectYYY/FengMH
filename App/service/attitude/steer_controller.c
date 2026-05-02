/*
 * steer_controller.c — 转向 PID 控制器实现
 *
 * 偏航角闭环: 目标角与当前角差值 → PID → 修正角速度 wz
 * 角度包裹: 始终走最短路径 (wrap_pi)
 * 输出限幅: 防止过快的旋转速度
 */
#include "steer_controller.h"
#include "config.h"
#include "log.h"

#include <math.h>
#include <string.h>

static const char* TAG = "STEER";

/* ─── 默认参数 ─── */
#define DEFAULT_KP      2.0f
#define DEFAULT_KI      0.1f
#define DEFAULT_KD      0.05f
#define DEFAULT_I_MAX   0.5f    /* rad */
#define DEFAULT_WZ_MAX  3.0f    /* rad/s, ~172 deg/s */

/* ─── 静态状态 ─── */

static steer_cfg_t s_cfg;
static float s_integral;   /* 积分项累加 */
static float s_prev_err;   /* 上一次误差 (用于微分) */

/* ─── 辅助: 角度包裹到 [-PI, PI] ─── */

static float wrap_pi(float angle) {
    while (angle >  (float)M_PI) angle -= 2.0f * (float)M_PI;
    while (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

/* ─── API ─── */

app_err_t steer_controller_init(void) {
    s_cfg.mode       = STEER_MODE_OFF;
    s_cfg.target_yaw = 0.0f;
    s_cfg.kp         = DEFAULT_KP;
    s_cfg.ki         = DEFAULT_KI;
    s_cfg.kd         = DEFAULT_KD;
    s_cfg.i_max      = DEFAULT_I_MAX;
    s_cfg.wz_max     = DEFAULT_WZ_MAX;
    s_integral       = 0.0f;
    s_prev_err       = 0.0f;

    LOGI("steer_controller init: kp=%.2f ki=%.2f kd=%.2f wz_max=%.2f rad/s",
         (double)s_cfg.kp, (double)s_cfg.ki, (double)s_cfg.kd, (double)s_cfg.wz_max);
    return APP_OK;
}

app_err_t steer_controller_set_mode(steer_mode_t mode) {
    s_cfg.mode = mode;
    /* 模式切换时清零积分 */
    s_integral = 0.0f;
    s_prev_err = 0.0f;
    LOGI("steer mode -> %d", (int)mode);
    return APP_OK;
}

app_err_t steer_controller_set_target_yaw(float yaw_rad) {
    s_cfg.target_yaw = wrap_pi(yaw_rad);
    return APP_OK;
}

float steer_controller_update(float current_yaw, float dt_s) {
    if (s_cfg.mode != STEER_MODE_YAW) {
        return 0.0f;
    }

    if (dt_s <= 0.0f || dt_s > 1.0f) {
        return 0.0f;
    }

    /* 计算偏航误差 (最短路径) */
    float err = wrap_pi(s_cfg.target_yaw - current_yaw);

    /* PID 计算 */
    float p_term = s_cfg.kp * err;

    s_integral += s_cfg.ki * err * dt_s;
    /* 积分限幅 */
    if      (s_integral >  s_cfg.i_max) s_integral =  s_cfg.i_max;
    else if (s_integral < -s_cfg.i_max) s_integral = -s_cfg.i_max;

    float d_term = 0.0f;
    if (dt_s > 1e-6f) {
        d_term = s_cfg.kd * (err - s_prev_err) / dt_s;
    }
    s_prev_err = err;

    float wz_out = p_term + s_integral + d_term;

    /* 输出限幅 */
    if      (wz_out >  s_cfg.wz_max) wz_out =  s_cfg.wz_max;
    else if (wz_out < -s_cfg.wz_max) wz_out = -s_cfg.wz_max;

    return wz_out;
}

const steer_cfg_t* steer_controller_get_cfg(void) {
    return &s_cfg;
}
