/*
 * arm_pump.h - Vacuum pump device wrapper.
 */
#ifndef APP_DEVICE_ARM_PUMP_H_
#define APP_DEVICE_ARM_PUMP_H_

#include "err.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ARM_PUMP_AUX_PC8 = 0,
    ARM_PUMP_AUX_PC9,
    ARM_PUMP_AUX_PA8,
    ARM_PUMP_AUX_PA9,
    ARM_PUMP_AUX_MAX,
} arm_pump_aux_t;

app_err_t arm_pump_init(void);
app_err_t arm_pump_set(uint8_t enabled);
uint8_t arm_pump_is_enabled(void);

app_err_t arm_pump_set_aux(arm_pump_aux_t aux, uint8_t enabled);
uint8_t arm_pump_aux_is_enabled(arm_pump_aux_t aux);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEVICE_ARM_PUMP_H_ */
