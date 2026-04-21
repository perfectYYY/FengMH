/*
 * bsp_time.c — 跨 host/MCU 时间实现
 */
#include "bsp_time.h"
#include "config.h"

#if APP_TARGET_HOST
#include <time.h>

static uint64_t s_host_base_ns = 0;
static uint32_t s_host_fake_offset_ms = 0;

static uint64_t now_ns_raw(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

void bsp_time_init(void) {
    s_host_base_ns = now_ns_raw();
    s_host_fake_offset_ms = 0;
}

app_tick_t bsp_time_now_ms(void) {
    uint64_t d = now_ns_raw() - s_host_base_ns;
    return (app_tick_t)(d / 1000000ULL) + s_host_fake_offset_ms;
}

uint64_t bsp_time_now_us(void) {
    uint64_t d = now_ns_raw() - s_host_base_ns;
    return (d / 1000ULL) + (uint64_t)s_host_fake_offset_ms * 1000ULL;
}

void bsp_time_delay_ms(uint32_t ms) {
    struct timespec ts = {
        .tv_sec  = ms / 1000,
        .tv_nsec = (long)(ms % 1000) * 1000000L
    };
    nanosleep(&ts, 0);
}

void bsp_time_test_advance_ms(uint32_t ms) {
    s_host_fake_offset_ms += ms;
}

#else  /* APP_TARGET_MCU */

#include "stm32h7xx_hal.h"

void bsp_time_init(void) { /* HAL_Init 已负责 SysTick */ }

app_tick_t bsp_time_now_ms(void) { return (app_tick_t)HAL_GetTick(); }

uint64_t bsp_time_now_us(void) {
    /* 简化实现：ms 分辨率放大到 us；后续接 DWT 提升精度 */
    return (uint64_t)HAL_GetTick() * 1000ULL;
}

void bsp_time_delay_ms(uint32_t ms) { HAL_Delay(ms); }

void bsp_time_test_advance_ms(uint32_t ms) { (void)ms; }

#endif
