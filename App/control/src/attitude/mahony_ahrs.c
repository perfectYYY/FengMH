#include "mahony_ahrs.h"

#include <math.h>

static uint8_t normalize_vector(float* x, float* y, float* z) {
    const float norm_sq = (*x * *x) + (*y * *y) + (*z * *z);
    if (!isfinite(norm_sq) || norm_sq <= 1.0e-12f) return 0U;
    const float inv_norm = 1.0f / sqrtf(norm_sq);
    if (!isfinite(inv_norm)) return 0U;
    *x *= inv_norm;
    *y *= inv_norm;
    *z *= inv_norm;
    return 1U;
}

static uint8_t normalize_quaternion(mahony_ahrs_t* ahrs) {
    const float norm_sq = (ahrs->q0 * ahrs->q0) + (ahrs->q1 * ahrs->q1) +
                          (ahrs->q2 * ahrs->q2) + (ahrs->q3 * ahrs->q3);
    if (!isfinite(norm_sq) || norm_sq <= 1.0e-12f) return 0U;
    const float inv_norm = 1.0f / sqrtf(norm_sq);
    if (!isfinite(inv_norm)) return 0U;
    ahrs->q0 *= inv_norm;
    ahrs->q1 *= inv_norm;
    ahrs->q2 *= inv_norm;
    ahrs->q3 *= inv_norm;
    return 1U;
}

static void set_from_euler(mahony_ahrs_t* ahrs, float roll_rad, float pitch_rad, float yaw_rad) {
    const float cr = cosf(0.5f * roll_rad);
    const float sr = sinf(0.5f * roll_rad);
    const float cp = cosf(0.5f * pitch_rad);
    const float sp = sinf(0.5f * pitch_rad);
    const float cy = cosf(0.5f * yaw_rad);
    const float sy = sinf(0.5f * yaw_rad);
    ahrs->q0 = (cr * cp * cy) + (sr * sp * sy);
    ahrs->q1 = (sr * cp * cy) - (cr * sp * sy);
    ahrs->q2 = (cr * sp * cy) + (sr * cp * sy);
    ahrs->q3 = (cr * cp * sy) - (sr * sp * cy);
    (void)normalize_quaternion(ahrs);
}

void mahony_ahrs_init(mahony_ahrs_t* ahrs, float two_kp, float two_ki) {
    if (!ahrs) return;
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;
    ahrs->two_kp = (isfinite(two_kp) && two_kp >= 0.0f) ? two_kp : 0.0f;
    ahrs->two_ki = (isfinite(two_ki) && two_ki >= 0.0f) ? two_ki : 0.0f;
    ahrs->integral_fb[0] = 0.0f;
    ahrs->integral_fb[1] = 0.0f;
    ahrs->integral_fb[2] = 0.0f;
}

uint8_t mahony_ahrs_reset_from_accel(mahony_ahrs_t* ahrs,
                                     const float accel[3],
                                     float yaw_rad) {
    if (!ahrs || !accel || !isfinite(yaw_rad)) return 0U;
    float ax = accel[0];
    float ay = accel[1];
    float az = accel[2];
    if (!isfinite(ax) || !isfinite(ay) || !isfinite(az) || !normalize_vector(&ax, &ay, &az)) {
        return 0U;
    }
    const float roll = atan2f(ay, az);
    const float pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az)));
    if (!isfinite(roll) || !isfinite(pitch)) return 0U;
    set_from_euler(ahrs, roll, pitch, yaw_rad);
    ahrs->integral_fb[0] = 0.0f;
    ahrs->integral_fb[1] = 0.0f;
    ahrs->integral_fb[2] = 0.0f;
    return 1U;
}

uint8_t mahony_ahrs_update_imu(mahony_ahrs_t* ahrs,
                               const float gyro_rad_s[3],
                               const float accel_mps2[3],
                               float dt_s) {
    if (!ahrs || !gyro_rad_s || !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 1.0f) return 0U;
    float gx = gyro_rad_s[0];
    float gy = gyro_rad_s[1];
    float gz = gyro_rad_s[2];
    if (!isfinite(gx) || !isfinite(gy) || !isfinite(gz) || !normalize_quaternion(ahrs)) return 0U;

    if (accel_mps2) {
        float ax = accel_mps2[0];
        float ay = accel_mps2[1];
        float az = accel_mps2[2];
        if (isfinite(ax) && isfinite(ay) && isfinite(az) && normalize_vector(&ax, &ay, &az)) {
            const float halfvx = (ahrs->q1 * ahrs->q3) - (ahrs->q0 * ahrs->q2);
            const float halfvy = (ahrs->q0 * ahrs->q1) + (ahrs->q2 * ahrs->q3);
            const float halfvz = (ahrs->q0 * ahrs->q0) - 0.5f + (ahrs->q3 * ahrs->q3);
            const float halfex = (ay * halfvz) - (az * halfvy);
            const float halfey = (az * halfvx) - (ax * halfvz);
            const float halfez = (ax * halfvy) - (ay * halfvx);
            if (ahrs->two_ki > 0.0f) {
                ahrs->integral_fb[0] += ahrs->two_ki * halfex * dt_s;
                ahrs->integral_fb[1] += ahrs->two_ki * halfey * dt_s;
                ahrs->integral_fb[2] += ahrs->two_ki * halfez * dt_s;
                gx += ahrs->integral_fb[0];
                gy += ahrs->integral_fb[1];
                gz += ahrs->integral_fb[2];
            } else {
                ahrs->integral_fb[0] = 0.0f;
                ahrs->integral_fb[1] = 0.0f;
                ahrs->integral_fb[2] = 0.0f;
            }
            gx += ahrs->two_kp * halfex;
            gy += ahrs->two_kp * halfey;
            gz += ahrs->two_kp * halfez;
        }
    }

    gx *= 0.5f * dt_s;
    gy *= 0.5f * dt_s;
    gz *= 0.5f * dt_s;
    const float q0 = ahrs->q0;
    const float q1 = ahrs->q1;
    const float q2 = ahrs->q2;
    const float q3 = ahrs->q3;
    ahrs->q0 += (-q1 * gx) - (q2 * gy) - (q3 * gz);
    ahrs->q1 += (q0 * gx) + (q2 * gz) - (q3 * gy);
    ahrs->q2 += (q0 * gy) - (q1 * gz) + (q3 * gx);
    ahrs->q3 += (q0 * gz) + (q1 * gy) - (q2 * gx);
    return normalize_quaternion(ahrs);
}

uint8_t mahony_ahrs_get_euler(const mahony_ahrs_t* ahrs,
                              float* roll_rad,
                              float* pitch_rad,
                              float* yaw_rad) {
    if (!ahrs || !roll_rad || !pitch_rad || !yaw_rad) return 0U;
    if (!isfinite(ahrs->q0) || !isfinite(ahrs->q1) ||
        !isfinite(ahrs->q2) || !isfinite(ahrs->q3)) return 0U;
    const float roll = atan2f(2.0f * ((ahrs->q0 * ahrs->q1) + (ahrs->q2 * ahrs->q3)),
                              1.0f - (2.0f * ((ahrs->q1 * ahrs->q1) + (ahrs->q2 * ahrs->q2))));
    float sin_pitch = 2.0f * ((ahrs->q0 * ahrs->q2) - (ahrs->q3 * ahrs->q1));
    if (sin_pitch > 1.0f) sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    const float pitch = asinf(sin_pitch);
    const float yaw = atan2f(2.0f * ((ahrs->q0 * ahrs->q3) + (ahrs->q1 * ahrs->q2)),
                             1.0f - (2.0f * ((ahrs->q2 * ahrs->q2) + (ahrs->q3 * ahrs->q3))));
    if (!isfinite(roll) || !isfinite(pitch) || !isfinite(yaw)) return 0U;
    *roll_rad = roll;
    *pitch_rad = pitch;
    *yaw_rad = yaw;
    return 1U;
}

uint8_t mahony_ahrs_set_yaw(mahony_ahrs_t* ahrs, float yaw_rad) {
    float roll;
    float pitch;
    float ignored_yaw;
    if (!ahrs || !isfinite(yaw_rad) ||
        !mahony_ahrs_get_euler(ahrs, &roll, &pitch, &ignored_yaw)) return 0U;
    set_from_euler(ahrs, roll, pitch, yaw_rad);
    return 1U;
}
