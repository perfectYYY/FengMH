/*
 * pump_control.c - damiao_new1-compatible pump wrappers.
 */
#include "pump_control.h"

#include "arm_control.h"
#include "arm_legacy_compat.h"
#include "arm_pump.h"

volatile uint8_t debug_pd11_on = 0U;
volatile uint8_t debug_pc8_on = 0U;
volatile uint8_t debug_pc9_on = 0U;
volatile uint8_t debug_pa8_on = 0U;
volatile uint8_t debug_pa9_on = 0U;

void Pump_Control_Init(void) {
    (void)arm_pump_init();
    Pump_Control_Set(0U);
    Pump_Control_SetPC8(0U);
    Pump_Control_SetPC9(0U);
    Pump_Control_SetPA8(0U);
    Pump_Control_SetPA9(0U);
}

void Pump_Control_Set(uint8_t enabled) {
    debug_pd11_on = enabled ? 1U : 0U;
    debug_payload_loaded = debug_pd11_on;
    (void)arm_control_set_pump(debug_pd11_on);
}

uint8_t Pump_Control_IsEnabled(void) {
    return arm_pump_is_enabled();
}

void Pump_Control_SetPC8(uint8_t enabled) {
    debug_pc8_on = enabled ? 1U : 0U;
    (void)arm_pump_set_aux(ARM_PUMP_AUX_PC8, debug_pc8_on);
}

void Pump_Control_SetPC9(uint8_t enabled) {
    debug_pc9_on = enabled ? 1U : 0U;
    (void)arm_pump_set_aux(ARM_PUMP_AUX_PC9, debug_pc9_on);
}

void Pump_Control_SetPA8(uint8_t enabled) {
    debug_pa8_on = enabled ? 1U : 0U;
    (void)arm_pump_set_aux(ARM_PUMP_AUX_PA8, debug_pa8_on);
}

void Pump_Control_SetPA9(uint8_t enabled) {
    debug_pa9_on = enabled ? 1U : 0U;
    (void)arm_pump_set_aux(ARM_PUMP_AUX_PA9, debug_pa9_on);
}

void Pump_Control_Process(void) {
    if (debug_pd11_on != arm_pump_is_enabled()) Pump_Control_Set(debug_pd11_on);
    if (debug_pc8_on != arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC8)) {
        Pump_Control_SetPC8(debug_pc8_on);
    }
    if (debug_pc9_on != arm_pump_aux_is_enabled(ARM_PUMP_AUX_PC9)) {
        Pump_Control_SetPC9(debug_pc9_on);
    }
    if (debug_pa8_on != arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA8)) {
        Pump_Control_SetPA8(debug_pa8_on);
    }
    if (debug_pa9_on != arm_pump_aux_is_enabled(ARM_PUMP_AUX_PA9)) {
        Pump_Control_SetPA9(debug_pa9_on);
    }
}
