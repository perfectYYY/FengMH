/*
 * bsp_gpio.c - Small GPIO output abstraction for App-level devices.
 */
#include "bsp_gpio.h"
#include "config.h"

#include <string.h>

static uint8_t s_latch[BSP_GPIO_OUTPUT_MAX];
static uint8_t s_initialized[BSP_GPIO_OUTPUT_MAX];

#if !APP_TARGET_HOST
#include "stm32h7xx_hal.h"

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} gpio_map_t;

static const gpio_map_t S_GPIO_MAP[BSP_GPIO_OUTPUT_MAX] = {
    [BSP_GPIO_ARM_PUMP_MAIN] = { GPIOD, GPIO_PIN_11 },
    [BSP_GPIO_ARM_AUX_PC8]   = { GPIOC, GPIO_PIN_8 },
    [BSP_GPIO_ARM_AUX_PC9]   = { GPIOC, GPIO_PIN_9 },
    [BSP_GPIO_ARM_AUX_PA8]   = { GPIOA, GPIO_PIN_8 },
    [BSP_GPIO_ARM_AUX_PA9]   = { GPIOA, GPIO_PIN_9 },
};

static void enable_port_clock(GPIO_TypeDef* port) {
    if (port == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (port == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    } else if (port == GPIOD) {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
}
#endif

app_err_t bsp_gpio_output_init(bsp_gpio_output_t out) {
    if (out >= BSP_GPIO_OUTPUT_MAX) return APP_ERR_INVALID_ARG;

#if !APP_TARGET_HOST
    const gpio_map_t* map = &S_GPIO_MAP[out];
    enable_port_clock(map->port);
    HAL_GPIO_WritePin(map->port, map->pin, GPIO_PIN_RESET);

    GPIO_InitTypeDef init;
    memset(&init, 0, sizeof(init));
    init.Pin = map->pin;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_PULLDOWN;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(map->port, &init);
#endif

    s_latch[out] = 0U;
    s_initialized[out] = 1U;
    return APP_OK;
}

app_err_t bsp_gpio_write(bsp_gpio_output_t out, uint8_t level) {
    if (out >= BSP_GPIO_OUTPUT_MAX) return APP_ERR_INVALID_ARG;
    if (!s_initialized[out]) {
        app_err_t err = bsp_gpio_output_init(out);
        if (err != APP_OK) return err;
    }

    s_latch[out] = level ? 1U : 0U;
#if !APP_TARGET_HOST
    const gpio_map_t* map = &S_GPIO_MAP[out];
    HAL_GPIO_WritePin(map->port,
                      map->pin,
                      s_latch[out] ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
    return APP_OK;
}

uint8_t bsp_gpio_read_latch(bsp_gpio_output_t out) {
    if (out >= BSP_GPIO_OUTPUT_MAX) return 0U;
    return s_latch[out];
}

void bsp_gpio_test_reset(void) {
    memset(s_latch, 0, sizeof(s_latch));
    memset(s_initialized, 0, sizeof(s_initialized));
}
