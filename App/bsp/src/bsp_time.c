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

void bsp_time_delay_us(uint32_t us) {
    struct timespec ts = {
        .tv_sec = us / 1000000U,
        .tv_nsec = (long)(us % 1000000U) * 1000L
    };
    nanosleep(&ts, 0);
}

void bsp_time_test_advance_ms(uint32_t ms) {
    s_host_fake_offset_ms += ms;
}

#else  /* APP_TARGET_MCU */

#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

void bsp_time_init(void) { /* HAL_Init 已负责 SysTick */ }

app_tick_t bsp_time_now_ms(void) { return (app_tick_t)HAL_GetTick(); }

uint64_t bsp_time_now_us(void) {
    /* 简化实现：ms 分辨率放大到 us；后续接 DWT 提升精度 */
    return (uint64_t)HAL_GetTick() * 1000ULL;
}

static void dwt_delay_us(uint32_t us) {
    if (us == 0U) return;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) {
        cycles_per_us = 1U;
    }

    while (us > 0U) {
        uint32_t chunk_us = (us > 1000U) ? 1000U : us;
        uint32_t wait_cycles = chunk_us * cycles_per_us;
        uint32_t start = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - start) < wait_cycles) {
            /* busy wait */
        }
        us -= chunk_us;
    }
}

void bsp_time_delay_ms(uint32_t ms) {
    osKernelState_t state = osKernelGetState();
    if (state == osKernelRunning) {
        (void)osDelay(ms);
        return;
    }

    if (state == osKernelReady ||
        state == osKernelLocked ||
        state == osKernelSuspended) {
        while (ms > 0U) {
            dwt_delay_us(1000U);
            ms--;
        }
        return;
    }

    HAL_Delay(ms);
}

void bsp_time_delay_us(uint32_t us) {
    dwt_delay_us(us);
}

void bsp_time_test_advance_ms(uint32_t ms) { (void)ms; }

#endif
