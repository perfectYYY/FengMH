/*
 * pump_control.h - damiao_new1-compatible pump API.
 */
#ifndef APP_DEVICE_PUMP_CONTROL_H_
#define APP_DEVICE_PUMP_CONTROL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint8_t debug_pd11_on;
extern volatile uint8_t debug_pc8_on;
extern volatile uint8_t debug_pc9_on;
extern volatile uint8_t debug_pa8_on;
extern volatile uint8_t debug_pa9_on;

void Pump_Control_Init(void);
void Pump_Control_Process(void);
void Pump_Control_Set(uint8_t enabled);
uint8_t Pump_Control_IsEnabled(void);
void Pump_Control_SetPC8(uint8_t enabled);
void Pump_Control_SetPC9(uint8_t enabled);
void Pump_Control_SetPA8(uint8_t enabled);
void Pump_Control_SetPA9(uint8_t enabled);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEVICE_PUMP_CONTROL_H_ */
