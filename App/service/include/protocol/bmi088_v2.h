/*
 * bmi088_v2.h - A5 5A v2 processed BMI088 frame decoder and latest-sample cache.
 */
#ifndef APP_SERVICE_BMI088_V2_H_
#define APP_SERVICE_BMI088_V2_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMI088_V2_MAGIC0             0xA5U
#define BMI088_V2_MAGIC1             0x5AU
#define BMI088_V2_VERSION            0x02U
#define BMI088_V2_TYPE_PROCESSED     0x01U
#define BMI088_V2_PAYLOAD_LEN        188U
#define BMI088_V2_MAX_PAYLOAD_LEN    192U
#define BMI088_V2_FRAME_LEN          198U
#define BMI088_V2_TIMEOUT_MS         50U

typedef struct {
    uint32_t device_id;
    uint32_t sequence;
    uint64_t sample_time_us;
    float calibrated_accel[3];
    float calibrated_gyro[3];
    float temperature;
    uint8_t imu_status;
    uint8_t ins_state;
    uint16_t flags;
    uint32_t sample_misses;
    uint32_t watchdog_samples;
    uint32_t usb_dropped;
    uint32_t rx_invalid;
    float quaternion[4];             /* w, x, y, z; body -> NWU */
    float roll;
    float pitch;
    float yaw;
    float navigation_accel[3];
    float velocity[3];
    float position[3];
    float accel_bias[3];
    float gyro_bias[3];
    float stationary_score;
    uint32_t zupt_updates;
    uint32_t zaru_updates;
    uint32_t gravity_updates;
    uint32_t segment_corrections;
    uint32_t calibration_resets;
    float last_position_correction[3];
    uint32_t received_ms;            /* FengMH local receive time */
} bmi088_v2_sample_t;

typedef struct {
    uint32_t accepted;
    uint32_t crc_errors;
    uint32_t header_errors;
    uint32_t state_errors;
    uint32_t nonfinite_errors;
    uint32_t sequence_errors;
    uint32_t skipped_samples;
} bmi088_v2_stats_t;

void bmi088_v2_init(void);

/* frame must contain one complete 198-byte A5 5A frame. Returns 1 if accepted. */
int bmi088_v2_accept_frame(const uint8_t* frame, uint32_t len, uint32_t received_ms);

/* Returns 1 only when a valid cached sample is no older than timeout_ms. */
int bmi088_v2_get_latest(bmi088_v2_sample_t* out,
                         uint32_t now_ms,
                         uint32_t timeout_ms);
int bmi088_v2_is_fresh(uint32_t now_ms, uint32_t timeout_ms);
void bmi088_v2_get_stats(bmi088_v2_stats_t* out);

uint32_t bmi088_v2_crc32(const uint8_t* data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_BMI088_V2_H_ */
