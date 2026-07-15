#include "bmi088_v2.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int s_failures;

#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        s_failures++; \
    } \
} while (0)

static void write_le32(uint8_t* p, uint32_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static void write_float(uint8_t* p, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    write_le32(p, bits);
}

static void finish_crc(uint8_t frame[BMI088_V2_FRAME_LEN]) {
    write_le32(frame + 194U, bmi088_v2_crc32(frame + 2U, 192U));
}

static void make_frame(uint8_t frame[BMI088_V2_FRAME_LEN], uint32_t sequence) {
    memset(frame, 0, BMI088_V2_FRAME_LEN);
    frame[0] = BMI088_V2_MAGIC0;
    frame[1] = BMI088_V2_MAGIC1;
    frame[2] = BMI088_V2_VERSION;
    frame[3] = BMI088_V2_TYPE_PROCESSED;
    frame[4] = (uint8_t)BMI088_V2_PAYLOAD_LEN;
    frame[5] = (uint8_t)(BMI088_V2_PAYLOAD_LEN >> 8);
    write_le32(frame + 6U, 0x12345678U);
    write_le32(frame + 10U, sequence);
    frame[6U + 44U] = 0U;
    frame[6U + 45U] = 1U;
    write_float(frame + 6U + 64U, 1.0f); /* quaternion w */
    write_float(frame + 6U + 88U, 1.25f);
    write_float(frame + 6U + 28U + 8U, 0.30f);
    write_float(frame + 6U + 140U + 8U, 0.05f);
    write_float(frame + 6U + 104U + 4U, 0.40f);
    write_float(frame + 6U + 116U + 4U, 0.20f);
    finish_crc(frame);
}

int main(void) {
    static const uint8_t check_text[] = "123456789";
    uint8_t frame[BMI088_V2_FRAME_LEN];
    bmi088_v2_sample_t sample;
    bmi088_v2_stats_t stats;

    CHECK(bmi088_v2_crc32(check_text, 9U) == 0xCBF43926U);

    bmi088_v2_init();
    make_frame(frame, 10U);
    CHECK(bmi088_v2_accept_frame(frame, sizeof(frame), 100U) == 1);
    CHECK(bmi088_v2_get_latest(&sample, 150U, 50U) == 1);
    CHECK(sample.device_id == 0x12345678U);
    CHECK(sample.sequence == 10U);
    CHECK(fabsf(sample.yaw - 1.25f) < 1e-6f);
    CHECK(fabsf((sample.calibrated_gyro[2] - sample.gyro_bias[2]) - 0.25f) < 1e-6f);
    CHECK(bmi088_v2_is_fresh(151U, 50U) == 0);

    make_frame(frame, 13U);
    CHECK(bmi088_v2_accept_frame(frame, sizeof(frame), 160U) == 1);
    CHECK(bmi088_v2_accept_frame(frame, sizeof(frame), 161U) == 0);
    bmi088_v2_get_stats(&stats);
    CHECK(stats.accepted == 2U);
    CHECK(stats.skipped_samples == 2U);
    CHECK(stats.sequence_errors == 1U);

    make_frame(frame, 14U);
    frame[100] ^= 0x01U;
    CHECK(bmi088_v2_accept_frame(frame, sizeof(frame), 170U) == 0);
    bmi088_v2_get_stats(&stats);
    CHECK(stats.crc_errors == 1U);

    make_frame(frame, 14U);
    write_float(frame + 6U + 88U, NAN);
    finish_crc(frame);
    CHECK(bmi088_v2_accept_frame(frame, sizeof(frame), 180U) == 0);
    bmi088_v2_get_stats(&stats);
    CHECK(stats.nonfinite_errors == 1U);

    return s_failures ? 1 : 0;
}
