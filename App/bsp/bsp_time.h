/*
 * bsp_time.h — 系统时间抽象（ms / us）
 *   - 板上：HAL_GetTick / DWT
 *   - host：posix CLOCK_MONOTONIC
 */
#ifndef APP_BSP_TIME_H_
#define APP_BSP_TIME_H_

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void       bsp_time_init(void);
app_tick_t bsp_time_now_ms(void);
uint64_t   bsp_time_now_us(void);
void       bsp_time_delay_ms(uint32_t ms);

/* 供 host 单测手动推进时间。MCU 构建下这个函数是空实现。 */
void       bsp_time_test_advance_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_BSP_TIME_H_ */
