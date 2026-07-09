/*
 * arm_pump.c - Vacuum pump device wrapper.
 */
#include "arm_pump.h"

#include "bsp_gpio.h"

static uint8_t s_main_enabled;
static uint8_t s_aux_enabled[ARM_PUMP_AUX_MAX];

static bsp_gpio_output_t aux_to_gpio(arm_pump_aux_t aux) {
    switch (aux) {
        case ARM_PUMP_AUX_PC8: return BSP_GPIO_ARM_AUX_PC8;
        case ARM_PUMP_AUX_PC9: return BSP_GPIO_ARM_AUX_PC9;
        case ARM_PUMP_AUX_PA8: return BSP_GPIO_ARM_AUX_PA8;
        case ARM_PUMP_AUX_PA9: return BSP_GPIO_ARM_AUX_PA9;
        default: return BSP_GPIO_OUTPUT_MAX;
    }
}

app_err_t arm_pump_init(void) {
    s_main_enabled = 0U;
    for (uint32_t i = 0U; i < ARM_PUMP_AUX_MAX; i++) {
        s_aux_enabled[i] = 0U;
    }

    app_err_t err = bsp_gpio_output_init(BSP_GPIO_ARM_PUMP_MAIN);
    if (err != APP_OK) return err;
    return bsp_gpio_write(BSP_GPIO_ARM_PUMP_MAIN, 0U);
}

app_err_t arm_pump_set(uint8_t enabled) {
    s_main_enabled = enabled ? 1U : 0U;
    return bsp_gpio_write(BSP_GPIO_ARM_PUMP_MAIN, s_main_enabled);
}

uint8_t arm_pump_is_enabled(void) {
    return s_main_enabled;
}

app_err_t arm_pump_set_aux(arm_pump_aux_t aux, uint8_t enabled) {
    if (aux >= ARM_PUMP_AUX_MAX) return APP_ERR_INVALID_ARG;
    bsp_gpio_output_t out = aux_to_gpio(aux);
    if (out >= BSP_GPIO_OUTPUT_MAX) return APP_ERR_INVALID_ARG;

    s_aux_enabled[aux] = enabled ? 1U : 0U;
    return bsp_gpio_write(out, s_aux_enabled[aux]);
}

uint8_t arm_pump_aux_is_enabled(arm_pump_aux_t aux) {
    if (aux >= ARM_PUMP_AUX_MAX) return 0U;
    return s_aux_enabled[aux];
}
