/*
 * stubs.c - Host-only hardware and motor stubs for control-path tests.
 *
 * These functions replace the MCU-facing BSP and concrete motor drivers so the
 * control stack can run as a normal process. The tests bind stub motors through
 * motor_registry and then inspect the commands issued by leg_controller.
 */
#include "bsp_fdcan.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "err.h"
#include "imu_bmi088.h"
#include "motor_go.h"
#include "motor_if.h"
#include "motor_m3508.h"

#include <string.h>
#include <time.h>

static app_tick_t s_now_ms;
static imu_bmi088_data_t s_imu_data;
static uint8_t s_imu_ready = 1U;

void bsp_time_init(void) {
    s_now_ms = 0;
}

app_tick_t bsp_time_now_ms(void) {
    return s_now_ms;
}

uint64_t bsp_time_now_us(void) {
    return (uint64_t)s_now_ms * 1000ULL;
}

void bsp_time_delay_ms(uint32_t ms) {
    s_now_ms += ms;
}

void bsp_time_delay_us(uint32_t us) {
    s_now_ms += (app_tick_t)((us + 999U) / 1000U);
}

void bsp_time_test_advance_ms(uint32_t ms) {
    s_now_ms += ms;
}

app_err_t bsp_uart_init(bsp_uart_bus_t bus) {
    (void)bus;
    return APP_OK;
}

app_err_t bsp_uart_attach_rx(bsp_uart_bus_t bus, bsp_uart_rx_cb_t cb, void* user) {
    (void)bus;
    (void)cb;
    (void)user;
    return APP_OK;
}

app_err_t bsp_uart_send(bsp_uart_bus_t bus, const uint8_t* data, uint32_t len) {
    (void)bus;
    (void)data;
    (void)len;
    return APP_OK;
}

app_err_t bsp_uart_wait_tx_done(bsp_uart_bus_t bus, uint32_t timeout_us) {
    (void)bus;
    (void)timeout_us;
    return APP_OK;
}

void bsp_uart_on_rx(bsp_uart_bus_t bus, const uint8_t* data, uint32_t len) {
    (void)bus;
    (void)data;
    (void)len;
}

void bsp_uart_on_tx_done(bsp_uart_bus_t bus) {
    (void)bus;
}

uint32_t bsp_uart_test_tx_count(bsp_uart_bus_t bus) {
    (void)bus;
    return 0;
}

app_err_t bsp_uart_test_pop_tx(bsp_uart_bus_t bus, uint8_t* out, uint32_t* out_len, uint32_t max_len) {
    (void)bus;
    (void)out;
    (void)out_len;
    (void)max_len;
    return APP_ERR_NOT_FOUND;
}

void bsp_uart_test_inject_rx(bsp_uart_bus_t bus, const uint8_t* data, uint32_t len) {
    (void)bus;
    (void)data;
    (void)len;
}

void bsp_uart_test_reset(void) {}

app_err_t bsp_fdcan_init(void) {
    return APP_OK;
}

app_err_t bsp_fdcan_attach_rx(bsp_fdcan_bus_t bus, bsp_fdcan_rx_cb_t cb, void* user) {
    (void)bus;
    (void)cb;
    (void)user;
    return APP_OK;
}

app_err_t bsp_fdcan_send(bsp_fdcan_bus_t bus, const bsp_fdcan_frame_t* frame) {
    (void)bus;
    (void)frame;
    return APP_OK;
}

uint32_t bsp_fdcan_test_tx_count(bsp_fdcan_bus_t bus) {
    (void)bus;
    return 0;
}

app_err_t bsp_fdcan_test_pop_tx(bsp_fdcan_bus_t bus, bsp_fdcan_frame_t* out) {
    (void)bus;
    (void)out;
    return APP_ERR_NOT_FOUND;
}

void bsp_fdcan_test_inject_rx(bsp_fdcan_bus_t bus, const bsp_fdcan_frame_t* frame) {
    (void)bus;
    (void)frame;
}

void bsp_fdcan_test_reset(void) {}

app_err_t imu_bmi088_init(void) {
    memset(&s_imu_data, 0, sizeof(s_imu_data));
    s_imu_data.accel[2] = BMI088_GRAVITY;
    s_imu_ready = 1U;
    return APP_OK;
}

app_err_t imu_bmi088_read(imu_bmi088_data_t* data) {
    if (!data) return APP_ERR_INVALID_ARG;
    *data = s_imu_data;
    return APP_OK;
}

uint8_t imu_bmi088_is_ready(void) {
    return s_imu_ready;
}

void imu_bmi088_test_set(const imu_bmi088_data_t* data, uint8_t ready) {
    if (data) {
        s_imu_data = *data;
    } else {
        memset(&s_imu_data, 0, sizeof(s_imu_data));
        s_imu_data.accel[2] = BMI088_GRAVITY;
    }
    s_imu_ready = ready;
}

app_err_t motor_go_init_all(void) {
    return APP_OK;
}

app_err_t motor_go_send_all(void) {
    return APP_OK;
}

app_err_t motor_go_calibrate_all(void) {
    return APP_OK;
}

app_err_t motor_go_debug_get_state(uint16_t logical_id, go_debug_state_t* out) {
    (void)logical_id;
    (void)out;
    return APP_ERR_UNSUPPORTED;
}

app_err_t motor_m3508_init_all(void) {
    return APP_OK;
}

app_err_t motor_m3508_send_all(void) {
    return APP_OK;
}

void motor_m3508_trace_reset(uint32_t decim) {
    (void)decim;
}

void motor_m3508_trace_enable(uint8_t enable) {
    (void)enable;
}

app_err_t motor_m3508_set_mit_limits(motor_dev_t* dev,
                                     float tau_limit_nm,
                                     float pos_err_limit_rad) {
    (void)dev;
    (void)tau_limit_nm;
    (void)pos_err_limit_rad;
    return APP_OK;
}

void motor_m3508_fdcan_rx_cb(bsp_fdcan_bus_t bus,
                             const bsp_fdcan_frame_t* frame,
                             void* user) {
    (void)bus;
    (void)frame;
    (void)user;
}
