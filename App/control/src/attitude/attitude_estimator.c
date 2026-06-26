/*
 * attitude_estimator.c — 姿态估计器实现
 *
 * 当前算法: 陀螺仪 Z 轴数值积分 + 加速度计 roll/pitch 倾角估计
 *
 * 静止判定: 三轴角速度模长 < gyro_thresh 时, 不积分
 *           这避免了传感器 bias 导致的长期漂移累积。
 *
 * 局限性:
 *   - yaw 仍仅靠 gyro Z 积分, 长期会漂移
 *   - roll/pitch 使用重力方向修正, 动态加速度大时会有瞬态误差
 *   - 适用于底盘姿态补偿的第一版慢速闭环
 *
 * 后续可扩展:
 *   - Mahony / Madgwick AHRS
 *   - 自动 bias 校准
 */
#include "attitude_estimator.h"
#include "config.h"
#include "log.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

static const char* TAG = "ATTEST";

/* ─── 可调参数 ─── */

/* 静止判定阈值 (rad/s)。三轴角速度模长小于此值时认为是静止, 不积分 */
#define DEFAULT_GYRO_THRESH  0.01f   /* ~0.57 deg/s */
#define DEFAULT_ACCEL_BLEND  0.04f
#define ACCEL_NORM_MIN_MPS2  1.0f
#define ACCEL_NORM_MAX_MPS2  30.0f

/* ─── 静态状态 ─── */

static attitude_state_t s_state;
static float s_gyro_thresh = DEFAULT_GYRO_THRESH;
static float s_accel_blend = DEFAULT_ACCEL_BLEND;
static uint8_t s_accel_initialized = 0U;

static float wrap_pi(float angle_rad) {
    while (angle_rad >  (float)M_PI) angle_rad -= 2.0f * (float)M_PI;
    while (angle_rad < -(float)M_PI) angle_rad += 2.0f * (float)M_PI;
    return angle_rad;
}

static uint8_t accel_to_roll_pitch(const float accel[3], float* roll_rad, float* pitch_rad) {
    if (!accel || !roll_rad || !pitch_rad) return 0U;

    float ax = accel[0];
    float ay = accel[1];
    float az = accel[2];
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (!isfinite(norm) || norm < ACCEL_NORM_MIN_MPS2 || norm > ACCEL_NORM_MAX_MPS2) {
        return 0U;
    }

    ax /= norm;
    ay /= norm;
    az /= norm;

    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    if (!isfinite(roll) || !isfinite(pitch)) return 0U;

    *roll_rad = roll;
    *pitch_rad = pitch;
    return 1U;
}

static void update_roll_pitch_from_accel(const float accel[3]) {
    float roll_acc;
    float pitch_acc;
    if (!accel_to_roll_pitch(accel, &roll_acc, &pitch_acc)) return;

    if (!s_accel_initialized) {
        s_state.roll = roll_acc;
        s_state.pitch = pitch_acc;
        s_accel_initialized = 1U;
        return;
    }

    float blend = s_accel_blend;
    if (!isfinite(blend) || blend < 0.0f) blend = 0.0f;
    if (blend > 1.0f) blend = 1.0f;

    s_state.roll = wrap_pi((1.0f - blend) * s_state.roll + blend * roll_acc);
    s_state.pitch = wrap_pi((1.0f - blend) * s_state.pitch + blend * pitch_acc);
}

/* ─── API ─── */

app_err_t attitude_estimator_init(void) {
    memset(&s_state, 0, sizeof(s_state));
    s_accel_initialized = 0U;
    LOGI("attitude_estimator init: gyro_thresh=%.3f rad/s accel_blend=%.2f",
         (double)s_gyro_thresh,
         (double)s_accel_blend);
    return APP_OK;
}

app_err_t attitude_estimator_update(const float gyro[3], const float accel[3], float dt_s) {
    if (!gyro) return APP_ERR_INVALID_ARG;
    if (dt_s <= 0.0f || dt_s > 1.0f) return APP_ERR_INVALID_ARG;

    s_state.roll = wrap_pi(s_state.roll + gyro[0] * dt_s);
    s_state.pitch = wrap_pi(s_state.pitch + gyro[1] * dt_s);

    /* 静止判定: 三轴角速度模长 */
    float gyro_norm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);

    if (gyro_norm >= s_gyro_thresh) {
        /* 旋转中: 积分偏航角 (仅 Z 轴) */
        s_state.yaw = wrap_pi(s_state.yaw + gyro[2] * dt_s);
    }
    /* else: 静止 → 冻结积分, 抑制漂移 */

    update_roll_pitch_from_accel(accel);
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
