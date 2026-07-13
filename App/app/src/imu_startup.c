/* Keep all actuator-facing initialization behind a verified IMU startup gate. */
#include "imu_startup.h"

#include "attitude_estimator.h"
#include "bsp_time.h"
#include "bsp_usb_cdc.h"
#include "config.h"
#include "imu_bmi088.h"
#include "usb_debug_text.h"

#include <math.h>
#include <stdint.h>

static int32_t to_milli(float value) {
    if (!isfinite(value)) return 0;
    if (value >= 2147483.0f) return INT32_MAX;
    if (value <= -2147483.0f) return INT32_MIN;
    return (int32_t)lroundf(value * 1000.0f);
}

static void report_bmi(uint32_t attempt) {
    imu_bmi088_diag_t diag;
    imu_bmi088_get_diag(&diag);
    (void)app_usb_text_send("BMI,n=%lu,s=%u,e=%d,i=%d,r=%02X,v=%02X,ai=%02X,gi=%02X\r\n",
                            (unsigned long)attempt, (unsigned)diag.stage,
                            (int)diag.err, (int)diag.io_err, (unsigned)diag.reg,
                            (unsigned)diag.val, (unsigned)diag.acc_initial_id,
                            (unsigned)diag.gyro_initial_id);
}

void imu_startup_wait_until_ready(void) {
    uint32_t attempt = 0U;

    for (;;) {
        app_err_t err;
        imu_bmi088_data_t sample;
        uint32_t last_ms;
        uint32_t cal_started_ms;
        uint32_t last_report_ms;

        attempt++;
        err = imu_bmi088_init();
        if (err != APP_OK) {
            report_bmi(attempt);
            bsp_usb_cdc_process();
            bsp_time_delay_ms(APP_IMU_STARTUP_RETRY_MS);
            continue;
        }

        (void)attitude_estimator_init();
        last_ms = (uint32_t)bsp_time_now_ms();
        cal_started_ms = last_ms;
        last_report_ms = last_ms;
        (void)app_usb_text_send("CAL,n=%lu,t=0,c=0\r\n", (unsigned long)attempt);

        for (;;) {
            uint32_t now_ms;
            float dt_s;

            bsp_time_delay_ms(APP_IMU_STARTUP_SAMPLE_PERIOD_MS);
            now_ms = (uint32_t)bsp_time_now_ms();
            dt_s = (float)(now_ms - last_ms) * 0.001f;
            last_ms = now_ms;

            err = imu_bmi088_read(&sample);
            if (err != APP_OK) {
                report_bmi(attempt);
                bsp_usb_cdc_process();
                bsp_time_delay_ms(APP_IMU_STARTUP_RETRY_MS);
                break;
            }
            (void)attitude_estimator_update(sample.gyro, sample.accel, dt_s);
            if (attitude_estimator_is_calibrated()) {
                float bias[3];
                attitude_estimator_get_gyro_bias(bias);
                (void)app_usb_text_send("BMI,n=%lu,ready=1,bz=%ld\r\n",
                                        (unsigned long)attempt,
                                        (long)to_milli(bias[2]));
                return;
            }
            if ((now_ms - last_report_ms) >= APP_IMU_STARTUP_RETRY_MS) {
                (void)app_usb_text_send("CAL,n=%lu,t=%lu,c=0\r\n",
                                        (unsigned long)attempt,
                                        (unsigned long)(now_ms - cal_started_ms));
                last_report_ms = now_ms;
            }
            bsp_usb_cdc_process();
        }
    }
}
