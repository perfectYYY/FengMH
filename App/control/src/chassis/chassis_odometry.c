/* IMU-wheel planar relative odometry. No acceleration or foot-force input is used. */
#include "chassis_odometry.h"

#include <math.h>
#include <string.h>

enum {
    ODOM_STATE_X = 0,
    ODOM_STATE_Y,
    ODOM_STATE_YAW,
    ODOM_STATE_VX,
    ODOM_STATE_GYRO_BIAS,
    ODOM_STATE_NUM,
};

#define ODOM_WHEEL_SPEED_STD_M_S       0.08f
#define ODOM_WHEEL_YAW_STD_RAD_S       0.25f
#define ODOM_ZERO_SPEED_STD_M_S        0.01f
#define ODOM_PROCESS_POS_VAR_M2_S      0.0001f
#define ODOM_PROCESS_YAW_VAR_RAD2_S    0.0004f
#define ODOM_PROCESS_VX_VAR_M2_S3      0.1600f
#define ODOM_PROCESS_BIAS_VAR_RAD2_S3  0.000004f

static chassis_odometry_state_t s_state;
static float s_cov[ODOM_STATE_NUM][ODOM_STATE_NUM];

static float clamp_positive(float value, float fallback) {
    return isfinite(value) && value > 1e-6f ? value : fallback;
}

static float wrap_pi(float angle_rad) {
    while (angle_rad > 3.14159265358979323846f) angle_rad -= 6.28318530717958647692f;
    while (angle_rad < -3.14159265358979323846f) angle_rad += 6.28318530717958647692f;
    return angle_rad;
}

static void covariance_predict(float dt_s) {
    float f[ODOM_STATE_NUM][ODOM_STATE_NUM] = {{0.0f}};
    float fp[ODOM_STATE_NUM][ODOM_STATE_NUM] = {{0.0f}};
    float next[ODOM_STATE_NUM][ODOM_STATE_NUM] = {{0.0f}};
    float cos_yaw = cosf(s_state.yaw_rad);
    float sin_yaw = sinf(s_state.yaw_rad);

    for (int i = 0; i < ODOM_STATE_NUM; i++) f[i][i] = 1.0f;
    f[ODOM_STATE_X][ODOM_STATE_YAW] = -s_state.vx_m_s * sin_yaw * dt_s;
    f[ODOM_STATE_X][ODOM_STATE_VX] = cos_yaw * dt_s;
    f[ODOM_STATE_Y][ODOM_STATE_YAW] = s_state.vx_m_s * cos_yaw * dt_s;
    f[ODOM_STATE_Y][ODOM_STATE_VX] = sin_yaw * dt_s;
    f[ODOM_STATE_YAW][ODOM_STATE_GYRO_BIAS] = -dt_s;

    for (int row = 0; row < ODOM_STATE_NUM; row++) {
        for (int col = 0; col < ODOM_STATE_NUM; col++) {
            for (int k = 0; k < ODOM_STATE_NUM; k++) fp[row][col] += f[row][k] * s_cov[k][col];
        }
    }
    for (int row = 0; row < ODOM_STATE_NUM; row++) {
        for (int col = 0; col < ODOM_STATE_NUM; col++) {
            for (int k = 0; k < ODOM_STATE_NUM; k++) next[row][col] += fp[row][k] * f[col][k];
        }
    }
    next[ODOM_STATE_X][ODOM_STATE_X] += ODOM_PROCESS_POS_VAR_M2_S * dt_s;
    next[ODOM_STATE_Y][ODOM_STATE_Y] += ODOM_PROCESS_POS_VAR_M2_S * dt_s;
    next[ODOM_STATE_YAW][ODOM_STATE_YAW] += ODOM_PROCESS_YAW_VAR_RAD2_S * dt_s;
    next[ODOM_STATE_VX][ODOM_STATE_VX] += ODOM_PROCESS_VX_VAR_M2_S3 * dt_s;
    next[ODOM_STATE_GYRO_BIAS][ODOM_STATE_GYRO_BIAS] += ODOM_PROCESS_BIAS_VAR_RAD2_S3 * dt_s;
    memcpy(s_cov, next, sizeof(s_cov));
}

static void scalar_measurement_update(const float h[ODOM_STATE_NUM],
                                      float innovation,
                                      float variance) {
    float ph[ODOM_STATE_NUM] = {0.0f};
    float gain[ODOM_STATE_NUM] = {0.0f};
    float next[ODOM_STATE_NUM][ODOM_STATE_NUM] = {{0.0f}};
    float s = clamp_positive(variance, 1e-6f);

    if (!isfinite(innovation)) return;
    for (int row = 0; row < ODOM_STATE_NUM; row++) {
        for (int col = 0; col < ODOM_STATE_NUM; col++) ph[row] += s_cov[row][col] * h[col];
        s += h[row] * ph[row];
    }
    if (!isfinite(s) || s <= 1e-9f) return;

    for (int i = 0; i < ODOM_STATE_NUM; i++) gain[i] = ph[i] / s;
    s_state.x_m += gain[ODOM_STATE_X] * innovation;
    s_state.y_m += gain[ODOM_STATE_Y] * innovation;
    s_state.yaw_rad += gain[ODOM_STATE_YAW] * innovation;
    s_state.vx_m_s += gain[ODOM_STATE_VX] * innovation;
    s_state.gyro_z_bias_rad_s += gain[ODOM_STATE_GYRO_BIAS] * innovation;
    s_state.yaw_rad = wrap_pi(s_state.yaw_rad);

    for (int row = 0; row < ODOM_STATE_NUM; row++) {
        for (int col = 0; col < ODOM_STATE_NUM; col++) {
            next[row][col] = s_cov[row][col] - gain[row] * ph[col];
        }
    }
    for (int row = 0; row < ODOM_STATE_NUM; row++) {
        for (int col = row + 1; col < ODOM_STATE_NUM; col++) {
            float symmetric = 0.5f * (next[row][col] + next[col][row]);
            next[row][col] = symmetric;
            next[col][row] = symmetric;
        }
        if (next[row][row] < 1e-9f || !isfinite(next[row][row])) next[row][row] = 1e-9f;
    }
    memcpy(s_cov, next, sizeof(s_cov));
}

static uint8_t wheel_means(const chassis_odometry_input_t* input,
                           float* forward_m_s,
                           float* yaw_rad_s) {
    float left_sum = 0.0f;
    float right_sum = 0.0f;
    uint8_t left_count = 0U;
    uint8_t right_count = 0U;
    const float radius = clamp_positive(input->wheel_radius_m, 0.0475f);
    const float half_track = clamp_positive(input->half_track_m, 0.15f);

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        uint8_t bit = (uint8_t)(1U << i);
        if (!(input->wheel_online_mask & bit) || !(input->stance_mask & bit) ||
            !isfinite(input->wheel_velocity_rads[i])) {
            continue;
        }
        float velocity_m_s = radius * input->wheel_velocity_rads[i];
        if (i == GAIT_LEG_FL || i == GAIT_LEG_RL) {
            left_sum += velocity_m_s;
            left_count++;
        } else {
            right_sum += velocity_m_s;
            right_count++;
        }
    }
    if (!left_count || !right_count) return 0U;
    float left = left_sum / (float)left_count;
    float right = right_sum / (float)right_count;
    *forward_m_s = 0.5f * (left + right);
    *yaw_rad_s = (right - left) / (2.0f * half_track);
    return 1U;
}

void chassis_odometry_init(void) {
    memset(&s_state, 0, sizeof(s_state));
    memset(s_cov, 0, sizeof(s_cov));
    s_cov[ODOM_STATE_X][ODOM_STATE_X] = 0.01f;
    s_cov[ODOM_STATE_Y][ODOM_STATE_Y] = 0.01f;
    s_cov[ODOM_STATE_YAW][ODOM_STATE_YAW] = 0.04f;
    s_cov[ODOM_STATE_VX][ODOM_STATE_VX] = 0.04f;
    s_cov[ODOM_STATE_GYRO_BIAS][ODOM_STATE_GYRO_BIAS] = 0.0025f;
    s_state.quality_flags = CHASSIS_ODOM_FLAG_INITIALIZED;
}

void chassis_odometry_update(const chassis_odometry_input_t* input, float dt_s) {
    float wheel_vx_m_s = 0.0f;
    float wheel_yaw_rad_s = 0.0f;
    const float h_vx[ODOM_STATE_NUM] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    const float h_bias[ODOM_STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, -1.0f};

    if (!input || !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.1f) return;

    s_state.quality_flags = CHASSIS_ODOM_FLAG_INITIALIZED;
    s_state.stance_mask = input->stance_mask;
    s_state.wheel_online_mask = input->wheel_online_mask;
    if (input->imu_ready && isfinite(input->gyro_z_rad_s)) {
        s_state.quality_flags |= CHASSIS_ODOM_FLAG_IMU_READY;
        s_state.yaw_rate_rad_s = input->gyro_z_rad_s - s_state.gyro_z_bias_rad_s;
        s_state.x_m += s_state.vx_m_s * cosf(s_state.yaw_rad) * dt_s;
        s_state.y_m += s_state.vx_m_s * sinf(s_state.yaw_rad) * dt_s;
        s_state.yaw_rad = wrap_pi(s_state.yaw_rad + s_state.yaw_rate_rad_s * dt_s);
        covariance_predict(dt_s);
    } else {
        s_state.yaw_rate_rad_s = 0.0f;
    }

    if (input->transition_gated) s_state.quality_flags |= CHASSIS_ODOM_FLAG_TRANSITION_GATED;
    if (input->slip_gated) s_state.quality_flags |= CHASSIS_ODOM_FLAG_SLIP_GATED;

    uint8_t wheels_valid = wheel_means(input, &wheel_vx_m_s, &wheel_yaw_rad_s);
    if (wheels_valid) s_state.quality_flags |= CHASSIS_ODOM_FLAG_WHEEL_VALID;
    if (wheels_valid && !input->transition_gated && !input->slip_gated && input->imu_ready) {
        float scale = clamp_positive(input->wheel_measurement_scale, 1.0f);
        if (input->allow_velocity_correction) {
            scalar_measurement_update(h_vx,
                                      wheel_vx_m_s - s_state.vx_m_s,
                                      scale * ODOM_WHEEL_SPEED_STD_M_S * ODOM_WHEEL_SPEED_STD_M_S);
            s_state.quality_flags |= CHASSIS_ODOM_FLAG_WHEEL_CORRECTED;
        }
        if (input->allow_yaw_correction) {
            float predicted_yaw_rate = input->gyro_z_rad_s - s_state.gyro_z_bias_rad_s;
            scalar_measurement_update(h_bias,
                                      wheel_yaw_rad_s - predicted_yaw_rate,
                                      scale * ODOM_WHEEL_YAW_STD_RAD_S * ODOM_WHEEL_YAW_STD_RAD_S);
            s_state.yaw_rate_rad_s = input->gyro_z_rad_s - s_state.gyro_z_bias_rad_s;
            s_state.quality_flags |= CHASSIS_ODOM_FLAG_WHEEL_CORRECTED;
        }
    }
    if (input->zero_velocity && input->imu_ready) {
        scalar_measurement_update(h_vx, -s_state.vx_m_s,
                                  ODOM_ZERO_SPEED_STD_M_S * ODOM_ZERO_SPEED_STD_M_S);
        s_state.quality_flags |= CHASSIS_ODOM_FLAG_ZERO_VELOCITY;
    }
}

void chassis_odometry_reset_pose(float x_m, float y_m, float yaw_rad) {
    if (!isfinite(x_m) || !isfinite(y_m) || !isfinite(yaw_rad)) return;
    s_state.x_m = x_m;
    s_state.y_m = y_m;
    s_state.yaw_rad = wrap_pi(yaw_rad);
    s_state.vx_m_s = 0.0f;
    s_state.yaw_rate_rad_s = 0.0f;
    s_state.gyro_z_bias_rad_s = 0.0f;
    memset(s_cov, 0, sizeof(s_cov));
    s_cov[ODOM_STATE_X][ODOM_STATE_X] = 0.01f;
    s_cov[ODOM_STATE_Y][ODOM_STATE_Y] = 0.01f;
    s_cov[ODOM_STATE_YAW][ODOM_STATE_YAW] = 0.04f;
    s_cov[ODOM_STATE_VX][ODOM_STATE_VX] = 0.04f;
    s_cov[ODOM_STATE_GYRO_BIAS][ODOM_STATE_GYRO_BIAS] = 0.0025f;
}

const chassis_odometry_state_t* chassis_odometry_get_state(void) {
    return &s_state;
}
