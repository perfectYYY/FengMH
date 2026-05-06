/*
 * task_imu_test.c - BMI088 board smoke test task
 *
 * This task is intended for APP_BRINGUP_STAGE_IMU_TEST. It only reads BMI088
 * and sends telemetry, so motor outputs stay disabled during the IMU check.
 */
#include "task_imu_test.h"

#include "bsp_time.h"
#include "bsp_usb_cdc.h"
#include "config.h"
#include "imu_bmi088.h"
#include "log.h"
#include "proto_defs.h"
#include "proto_frame.h"

#include <string.h>

#if APP_TARGET_MCU
#include "cmsis_os.h"
#endif

static const char* TAG = "IMUTEST";

#define IMU_TEST_PERIOD_MS          10U
#define IMU_TEST_LOG_PERIOD_MS      1000U
#define IMU_TEST_USB_PERIOD_MS      20U

#pragma pack(push, 1)
typedef struct {
    uint32_t seq;
    uint32_t uptime_ms;
    int16_t  err;
    uint8_t  ready;
    uint8_t  status;
    float    accel[3];
    float    gyro[3];
    float    temp_c;
} payload_imu_state_t;
#pragma pack(pop)

static uint32_t s_seq;
static uint32_t s_ok_cnt;
static uint32_t s_err_cnt;
static app_err_t s_last_err = APP_ERR_UNINIT;

static void imu_test_send_frame(const imu_bmi088_data_t* data,
                                app_err_t err,
                                uint32_t now_ms) {
    payload_imu_state_t payload;
    uint8_t frame[80];
    int n;

    memset(&payload, 0, sizeof(payload));
    payload.seq = s_seq;
    payload.uptime_ms = now_ms;
    payload.err = (int16_t)err;
    payload.ready = imu_bmi088_is_ready();

    if (data) {
        payload.status = data->status;
        memcpy(payload.accel, data->accel, sizeof(payload.accel));
        memcpy(payload.gyro, data->gyro, sizeof(payload.gyro));
        payload.temp_c = data->temp;
    }

    n = proto_frame_build(PROTO_FUNC_IMU_STATE,
                          (const uint8_t*)&payload,
                          (uint8_t)sizeof(payload),
                          frame,
                          sizeof(frame));
    if (n > 0) {
        (void)bsp_usb_cdc_send(frame, (uint32_t)n);
    }
}

void task_imu_test_init(void) {
    s_seq = 0U;
    s_ok_cnt = 0U;
    s_err_cnt = 0U;
    s_last_err = imu_bmi088_is_ready() ? APP_OK : APP_ERR_UNINIT;
    LOGI("imu board test init: ready=%u", (unsigned)imu_bmi088_is_ready());
}

void task_imu_test_entry(void* arg) {
    (void)arg;
    LOGI("imu board test started: period=%ums usb=%ums",
         (unsigned)IMU_TEST_PERIOD_MS,
         (unsigned)IMU_TEST_USB_PERIOD_MS);

#if APP_TARGET_MCU
    uint32_t last_log_ms = 0U;
    uint32_t last_usb_ms = 0U;

    for (;;) {
        uint32_t now_ms = (uint32_t)bsp_time_now_ms();
        imu_bmi088_data_t data;
        app_err_t err = imu_bmi088_read(&data);

        s_seq++;
        s_last_err = err;
        if (err == APP_OK) {
            s_ok_cnt++;
        } else {
            s_err_cnt++;
            memset(&data, 0, sizeof(data));
            data.status = 0xFFU;
        }

        if ((now_ms - last_usb_ms) >= IMU_TEST_USB_PERIOD_MS) {
            imu_test_send_frame((err == APP_OK) ? &data : NULL, err, now_ms);
            last_usb_ms = now_ms;
        }

        if ((now_ms - last_log_ms) >= IMU_TEST_LOG_PERIOD_MS) {
            LOGI("seq=%lu ok=%lu err=%lu last=%d ready=%u acc=[%.3f %.3f %.3f] gyro=[%.4f %.4f %.4f] temp=%.2f",
                 (unsigned long)s_seq,
                 (unsigned long)s_ok_cnt,
                 (unsigned long)s_err_cnt,
                 (int)s_last_err,
                 (unsigned)imu_bmi088_is_ready(),
                 (double)data.accel[0],
                 (double)data.accel[1],
                 (double)data.accel[2],
                 (double)data.gyro[0],
                 (double)data.gyro[1],
                 (double)data.gyro[2],
                 (double)data.temp);
            last_log_ms = now_ms;
        }

        osDelay(IMU_TEST_PERIOD_MS);
    }
#endif
}
