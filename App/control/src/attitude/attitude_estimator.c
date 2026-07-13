/* BMI088 6-axis attitude estimator using the supplied Mahony reference model. */
#include "attitude_estimator.h"
#include "mahony_ahrs.h"
#include "config.h"
#include "log.h"

#include <math.h>
#include <string.h>

#if (APP_IMU_BODY_X_SOURCE_AXIS > 2U) || \
    (APP_IMU_BODY_Y_SOURCE_AXIS > 2U) || \
    (APP_IMU_BODY_Z_SOURCE_AXIS > 2U)
#error "IMU body axis source indices must be in [0, 2]"
#endif

static const char* TAG = "ATTEST";
static attitude_state_t s_state;
static mahony_ahrs_t s_ahrs;
static uint8_t s_initialized;
static uint8_t s_gyro_calibrated;
static float s_gyro_bias[3];
static float s_gyro_cal_sum[3];
static uint32_t s_gyro_cal_samples;
static float s_gyro_cal_elapsed_s;
static float s_static_elapsed_s;

static float vector_norm(const float v[3]) {
    return sqrtf((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
}

static uint8_t vector_is_finite(const float v[3]) {
    return v && isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static void map_to_body_axes(const float source[3], float body[3]) {
    body[0] = APP_IMU_BODY_X_SIGN * source[APP_IMU_BODY_X_SOURCE_AXIS];
    body[1] = APP_IMU_BODY_Y_SIGN * source[APP_IMU_BODY_Y_SOURCE_AXIS];
    body[2] = APP_IMU_BODY_Z_SIGN * source[APP_IMU_BODY_Z_SOURCE_AXIS];
}

static uint8_t accel_is_near_gravity(const float accel[3], float tolerance_mps2) {
    if (!vector_is_finite(accel) || tolerance_mps2 < 0.0f) return 0U;
    const float norm = vector_norm(accel);
    return isfinite(norm) && fabsf(norm - APP_IMU_GRAVITY_MPS2) <= tolerance_mps2;
}

static uint8_t publish_state(const float gyro_rad_s[3]) {
    float roll;
    float pitch;
    float yaw;
    if (!gyro_rad_s || !mahony_ahrs_get_euler(&s_ahrs, &roll, &pitch, &yaw)) return 0U;
    s_state.roll = roll;
    s_state.pitch = pitch;
    s_state.yaw = yaw;
    s_state.roll_rate = gyro_rad_s[0];
    s_state.pitch_rate = gyro_rad_s[1];
    s_state.yaw_rate = gyro_rad_s[2];
    return 1U;
}

static void reset_boot_calibration(void) {
    memset(s_gyro_cal_sum, 0, sizeof(s_gyro_cal_sum));
    s_gyro_cal_samples = 0U;
    s_gyro_cal_elapsed_s = 0.0f;
}

app_err_t attitude_estimator_init(void) {
    memset(&s_state, 0, sizeof(s_state));
    mahony_ahrs_init(&s_ahrs, APP_IMU_MAHONY_TWO_KP, APP_IMU_MAHONY_TWO_KI);
    s_gyro_calibrated = 0U;
    memset(s_gyro_bias, 0, sizeof(s_gyro_bias));
    reset_boot_calibration();
    s_static_elapsed_s = 0.0f;
    s_initialized = 1U;
    LOGI("attitude init: Mahony twoKp=%.2f boot_cal=%.1fs",
         (double)APP_IMU_MAHONY_TWO_KP,
         (double)APP_IMU_BOOT_CAL_DURATION_S);
    return APP_OK;
}

app_err_t attitude_estimator_update(const float gyro[3], const float accel[3], float dt_s) {
    float body_gyro[3];
    float body_accel[3];
    if (!gyro || !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 1.0f || !vector_is_finite(gyro)) {
        return APP_ERR_INVALID_ARG;
    }
    map_to_body_axes(gyro, body_gyro);
    const uint8_t accel_valid = vector_is_finite(accel);
    if (accel_valid) map_to_body_axes(accel, body_accel);

    const float raw_gyro_norm = vector_norm(body_gyro);
    if (!s_gyro_calibrated) {
        const uint8_t cal_sample_valid = accel_valid && isfinite(raw_gyro_norm) &&
            raw_gyro_norm <= APP_IMU_BOOT_CAL_GYRO_MAX_RAD_S &&
            accel_is_near_gravity(body_accel, APP_IMU_BOOT_CAL_ACCEL_TOL_MPS2);
        if (!cal_sample_valid) {
            reset_boot_calibration();
            return APP_OK;
        }
        for (int i = 0; i < 3; i++) s_gyro_cal_sum[i] += body_gyro[i];
        s_gyro_cal_samples++;
        s_gyro_cal_elapsed_s += dt_s;
        (void)mahony_ahrs_reset_from_accel(&s_ahrs, body_accel, 0.0f);
        (void)publish_state((float[3]){0.0f, 0.0f, 0.0f});
        if (s_gyro_cal_elapsed_s < APP_IMU_BOOT_CAL_DURATION_S) return APP_OK;

        for (int i = 0; i < 3; i++) {
            s_gyro_bias[i] = s_gyro_cal_sum[i] / (float)s_gyro_cal_samples;
        }
        s_gyro_calibrated = 1U;
        s_static_elapsed_s = 0.0f;
        LOGI("gyro bias calibrated: %.5f %.5f %.5f rad/s",
             (double)s_gyro_bias[0], (double)s_gyro_bias[1], (double)s_gyro_bias[2]);
        return APP_OK;
    }

    float corrected[3] = {
        body_gyro[0] - s_gyro_bias[0],
        body_gyro[1] - s_gyro_bias[1],
        body_gyro[2] - s_gyro_bias[2],
    };
    const float corrected_norm = vector_norm(corrected);
    const uint8_t stationary_sample = accel_valid && isfinite(corrected_norm) &&
        corrected_norm <= APP_IMU_STATIC_GYRO_MAX_RAD_S &&
        accel_is_near_gravity(body_accel, APP_IMU_STATIC_ACCEL_TOL_MPS2);
    if (stationary_sample) {
        s_static_elapsed_s += dt_s;
        if (s_static_elapsed_s >= APP_IMU_STATIC_HOLD_S) {
            const float alpha = dt_s / (APP_IMU_BIAS_TRACK_TAU_S + dt_s);
            for (int i = 0; i < 3; i++) {
                s_gyro_bias[i] += alpha * (body_gyro[i] - s_gyro_bias[i]);
                corrected[i] = 0.0f;
            }
        }
    } else {
        s_static_elapsed_s = 0.0f;
    }

    const float* gravity_accel = accel_valid &&
        accel_is_near_gravity(body_accel, APP_IMU_AHRS_ACCEL_TOL_MPS2) ? body_accel : NULL;
    if (!mahony_ahrs_update_imu(&s_ahrs, corrected, gravity_accel, dt_s)) {
        if (gravity_accel && mahony_ahrs_reset_from_accel(&s_ahrs, gravity_accel, s_state.yaw)) {
            (void)publish_state((float[3]){0.0f, 0.0f, 0.0f});
        }
        return APP_ERR_GENERIC;
    }
    return publish_state(corrected) ? APP_OK : APP_ERR_GENERIC;
}

uint8_t attitude_estimator_is_initialized(void) { return s_initialized; }
float attitude_estimator_get_yaw(void) { return s_state.yaw; }
uint8_t attitude_estimator_is_calibrated(void) { return s_gyro_calibrated; }

void attitude_estimator_get_gyro_bias(float out_bias[3]) {
    if (!out_bias) return;
    memcpy(out_bias, s_gyro_bias, sizeof(s_gyro_bias));
}

void attitude_estimator_reset_yaw(void) {
    if (mahony_ahrs_set_yaw(&s_ahrs, 0.0f)) {
        (void)publish_state((float[3]){s_state.roll_rate, s_state.pitch_rate, s_state.yaw_rate});
    }
    LOGI("yaw reset to 0");
}

const attitude_state_t* attitude_estimator_get_state(void) { return &s_state; }
