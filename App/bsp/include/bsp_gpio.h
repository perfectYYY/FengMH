/*
 * bsp_gpio.h - Small GPIO output abstraction for App-level devices.
 */
#ifndef APP_BSP_GPIO_H_
#define APP_BSP_GPIO_H_

#include "err.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_GPIO_ARM_PUMP_MAIN = 0,  /* PD11, confirmed main vacuum pump */
    BSP_GPIO_ARM_AUX_PC8,
    BSP_GPIO_ARM_AUX_PC9,
    BSP_GPIO_ARM_AUX_PA8,
    BSP_GPIO_ARM_AUX_PA9,
    BSP_GPIO_OUTPUT_MAX,
} bsp_gpio_output_t;

app_err_t bsp_gpio_output_init(bsp_gpio_output_t out);
app_err_t bsp_gpio_write(bsp_gpio_output_t out, uint8_t level);
uint8_t bsp_gpio_read_latch(bsp_gpio_output_t out);

void bsp_gpio_test_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BSP_GPIO_H_ */
