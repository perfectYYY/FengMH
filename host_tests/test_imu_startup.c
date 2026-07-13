#include "attitude_estimator.h"
#include "bsp_time.h"
#include "bsp_usb_cdc.h"
#include "err.h"
#include "imu_bmi088.h"
#include "imu_startup.h"

#include <assert.h>
#include <string.h>

static app_tick_t s_now_ms;
static uint32_t s_init_calls;
static imu_bmi088_diag_t s_diag;

void bsp_time_init(void) { s_now_ms = 0U; }
app_tick_t bsp_time_now_ms(void) { return s_now_ms; }
uint64_t bsp_time_now_us(void) { return (uint64_t)s_now_ms * 1000U; }
void bsp_time_delay_ms(uint32_t ms) { s_now_ms += ms; }
void bsp_time_delay_us(uint32_t us) { s_now_ms += (us + 999U) / 1000U; }

app_err_t imu_bmi088_init(void) {
    s_init_calls++;
    memset(&s_diag, 0, sizeof(s_diag));
    if (s_init_calls < 3U) {
        s_diag.stage = IMU_BMI088_DIAG_ACC_INITIAL_ID;
        s_diag.err = APP_ERR_IO;
        s_diag.reg = BMI088_ACC_CHIP_ID;
        s_diag.val = 0xFFU;
        return APP_ERR_IO;
    }
    s_diag.stage = IMU_BMI088_DIAG_READY;
    return APP_OK;
}

app_err_t imu_bmi088_read(imu_bmi088_data_t* out) {
    if (!out) return APP_ERR_INVALID_ARG;
    memset(out, 0, sizeof(*out));
    out->accel[2] = BMI088_GRAVITY;
    return APP_OK;
}

uint8_t imu_bmi088_is_ready(void) { return s_init_calls >= 3U; }
void imu_bmi088_get_diag(imu_bmi088_diag_t* out) { if (out) *out = s_diag; }

int main(void) {
    uint8_t text[512];

    bsp_time_init();
    assert(bsp_usb_cdc_init() == APP_OK);
    imu_startup_wait_until_ready();

    assert(s_init_calls == 3U);
    assert(bsp_time_now_ms() >= 2000U);
    assert(attitude_estimator_is_calibrated() == 1U);
    assert(bsp_usb_cdc_test_read_tx(text, sizeof(text)) > 0U);
    return 0;
}
