#include "bmi088_v2.h"

#include <math.h>
#include <string.h>

static bmi088_v2_sample_t s_samples[2];
static volatile uint8_t s_active_index;
static volatile uint8_t s_has_sample;
static bmi088_v2_stats_t s_stats;
static uint32_t s_last_sequence;
static uint8_t s_have_sequence;

static uint16_t read_le16(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_le32(const uint8_t* p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint64_t read_le64(const uint8_t* p) {
    return (uint64_t)read_le32(p) | ((uint64_t)read_le32(p + 4) << 32);
}

static float read_le_float(const uint8_t* p) {
    uint32_t bits = read_le32(p);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void read_float_array(const uint8_t* p, float* out, uint32_t count) {
    for (uint32_t i = 0U; i < count; i++) out[i] = read_le_float(p + i * 4U);
}

uint32_t bmi088_v2_crc32(const uint8_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFU;
    if (!data) return 0U;
    for (uint32_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint32_t bit = 0U; bit < 8U; bit++) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
    }
    return ~crc;
}

void bmi088_v2_init(void) {
    memset(s_samples, 0, sizeof(s_samples));
    memset(&s_stats, 0, sizeof(s_stats));
    s_active_index = 0U;
    s_has_sample = 0U;
    s_last_sequence = 0U;
    s_have_sequence = 0U;
}

static int all_finite(const float* values, uint32_t count) {
    for (uint32_t i = 0U; i < count; i++) {
        if (!isfinite(values[i])) return 0;
    }
    return 1;
}

static void decode_payload(const uint8_t* p, bmi088_v2_sample_t* s) {
    s->device_id = read_le32(p + 0U);
    s->sequence = read_le32(p + 4U);
    s->sample_time_us = read_le64(p + 8U);
    read_float_array(p + 16U, s->calibrated_accel, 3U);
    read_float_array(p + 28U, s->calibrated_gyro, 3U);
    s->temperature = read_le_float(p + 40U);
    s->imu_status = p[44U];
    s->ins_state = p[45U];
    s->flags = read_le16(p + 46U);
    s->sample_misses = read_le32(p + 48U);
    s->watchdog_samples = read_le32(p + 52U);
    s->usb_dropped = read_le32(p + 56U);
    s->rx_invalid = read_le32(p + 60U);
    read_float_array(p + 64U, s->quaternion, 4U);
    s->roll = read_le_float(p + 80U);
    s->pitch = read_le_float(p + 84U);
    s->yaw = read_le_float(p + 88U);
    read_float_array(p + 92U, s->navigation_accel, 3U);
    read_float_array(p + 104U, s->velocity, 3U);
    read_float_array(p + 116U, s->position, 3U);
    read_float_array(p + 128U, s->accel_bias, 3U);
    read_float_array(p + 140U, s->gyro_bias, 3U);
    s->stationary_score = read_le_float(p + 152U);
    s->zupt_updates = read_le32(p + 156U);
    s->zaru_updates = read_le32(p + 160U);
    s->gravity_updates = read_le32(p + 164U);
    s->segment_corrections = read_le32(p + 168U);
    s->calibration_resets = read_le32(p + 172U);
    read_float_array(p + 176U, s->last_position_correction, 3U);
}

static int control_fields_are_finite(const bmi088_v2_sample_t* s) {
    return all_finite(s->calibrated_accel, 3U) &&
           all_finite(s->calibrated_gyro, 3U) &&
           isfinite(s->temperature) &&
           all_finite(s->quaternion, 4U) &&
           all_finite(&s->roll, 3U) &&
           all_finite(s->navigation_accel, 3U) &&
           all_finite(s->velocity, 3U) &&
           all_finite(s->position, 3U) &&
           all_finite(s->accel_bias, 3U) &&
           all_finite(s->gyro_bias, 3U) &&
           isfinite(s->stationary_score) &&
           all_finite(s->last_position_correction, 3U);
}

int bmi088_v2_accept_frame(const uint8_t* frame, uint32_t len, uint32_t received_ms) {
    if (!frame || len != BMI088_V2_FRAME_LEN ||
        frame[0] != BMI088_V2_MAGIC0 || frame[1] != BMI088_V2_MAGIC1 ||
        frame[2] != BMI088_V2_VERSION || frame[3] != BMI088_V2_TYPE_PROCESSED ||
        read_le16(frame + 4U) != BMI088_V2_PAYLOAD_LEN) {
        s_stats.header_errors++;
        return 0;
    }

    uint32_t expected_crc = read_le32(frame + 194U);
    uint32_t actual_crc = bmi088_v2_crc32(frame + 2U, 192U);
    if (actual_crc != expected_crc) {
        s_stats.crc_errors++;
        return 0;
    }

    bmi088_v2_sample_t sample;
    memset(&sample, 0, sizeof(sample));
    decode_payload(frame + 6U, &sample);
    if (sample.imu_status != 0U || sample.ins_state != 1U) {
        s_stats.state_errors++;
        return 0;
    }
    if (!control_fields_are_finite(&sample)) {
        s_stats.nonfinite_errors++;
        return 0;
    }

    if (s_have_sequence) {
        uint32_t advance = sample.sequence - s_last_sequence;
        if (advance == 0U || advance >= 0x80000000U) {
            s_stats.sequence_errors++;
            return 0;
        }
        s_stats.skipped_samples += advance - 1U;
    }

    sample.received_ms = received_ms;
    uint8_t next = (uint8_t)(s_active_index ^ 1U);
    s_samples[next] = sample;
    s_active_index = next;
    s_has_sample = 1U;
    s_last_sequence = sample.sequence;
    s_have_sequence = 1U;
    s_stats.accepted++;
    return 1;
}

int bmi088_v2_get_latest(bmi088_v2_sample_t* out,
                         uint32_t now_ms,
                         uint32_t timeout_ms) {
    if (!out || !s_has_sample) return 0;
    uint8_t index = s_active_index;
    *out = s_samples[index];
    if (index != s_active_index) {
        index = s_active_index;
        *out = s_samples[index];
    }
    return (uint32_t)(now_ms - out->received_ms) <= timeout_ms;
}

int bmi088_v2_is_fresh(uint32_t now_ms, uint32_t timeout_ms) {
    bmi088_v2_sample_t sample;
    return bmi088_v2_get_latest(&sample, now_ms, timeout_ms);
}

void bmi088_v2_get_stats(bmi088_v2_stats_t* out) {
    if (out) *out = s_stats;
}
