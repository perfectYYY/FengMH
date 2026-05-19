/*
 * attitude_estimator.c — 姿态估计器实现
 *
 * 当前算法: 陀螺仪 Z 轴数值积分 + 静止漂移抑制
 *
 * 静止判定: 三轴角速度模长 < gyro_thresh 时, 不积分
 *           这避免了传感器 bias 导致的长期漂移累积。
 *
 * 局限性:
 *   - 仅积分偏航, 不考虑 pitch/roll 耦合
 *   - 无加速度计/MEMS 融合, 长期会漂移
 *   - 适用于短时间转向任务 (< 几分钟)
 *
 * 后续可扩展:
 *   - Mahony / Madgwick AHRS
 *   - 加速度计修正 pitch/roll
 *   - 自动 bias 校准
 */
#include "attitude_estimator.h"
#include "config.h"
#include "log.h"

#include <math.h>
#include <string.h>

static const char* TAG = "ATTEST";

/* ─── 可调参数 ─── */

/* 静止判定阈值 (rad/s)。三轴角速度模长小于此值时认为是静止, 不积分 */
#define DEFAULT_GYRO_THRESH  0.01f   /* ~0.57 deg/s */

/* ─── 静态状态 ─── */

static attitude_state_t s_state;
static float s_gyro_thresh = DEFAULT_GYRO_THRESH;

/* ─── API ─── */

app_err_t attitude_estimator_init(void) {
    memset(&s_state, 0, sizeof(s_state));
    LOGI("attitude_estimator init: gyro_thresh=%.3f rad/s", (double)s_gyro_thresh);
    return APP_OK;
}

app_err_t attitude_estimator_update(const float gyro[3], const float accel[3], float dt_s) {
    (void)accel;  /* 预留, 当前未使用 */

    if (!gyro) return APP_ERR_INVALID_ARG;
    if (dt_s <= 0.0f || dt_s > 1.0f) return APP_ERR_INVALID_ARG;

    /* 静止判定: 三轴角速度模长 */
    float gyro_norm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);

    if (gyro_norm >= s_gyro_thresh) {
        /* 旋转中: 积分偏航角 (仅 Z 轴) */
        s_state.yaw += gyro[2] * dt_s;

        /* 偏航角溢出包装到 [-PI, PI] */
        while (s_state.yaw >  (float)M_PI) s_state.yaw -= 2.0f * (float)M_PI;
        while (s_state.yaw < -(float)M_PI) s_state.yaw += 2.0f * (float)M_PI;
    }
    /* else: 静止 → 冻结积分, 抑制漂移 */

    return APP_OK;
}

float attitude_estimator_get_yaw(void) {
    return s_state.yaw;
}

void attitude_estimator_reset_yaw(void) {
    s_state.yaw = 0.0f;
    LOGI("yaw reset to 0");
}

const attitude_state_t* attitude_estimator_get_state(void) {
    return &s_state;
}
