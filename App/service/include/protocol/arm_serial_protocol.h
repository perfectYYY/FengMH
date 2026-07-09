/*
 * arm_serial_protocol.h - damiao_new1-compatible standalone arm protocol.
 *
 * This module is not attached to USB by default; task_comm owns the integrated
 * robot protocol. Keep it available for arm-only bring-up and legacy tools.
 */
#ifndef APP_SERVICE_ARM_SERIAL_PROTOCOL_H_
#define APP_SERVICE_ARM_SERIAL_PROTOCOL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_PROTOCOL_HEAD1                0x55U
#define ARM_PROTOCOL_HEAD2                0xAAU
#define ARM_PROTOCOL_FUNC_TARGET          0x12U
#define ARM_PROTOCOL_FUNC_PUMP            0x13U
#define ARM_PROTOCOL_FUNC_FEEDBACK        0x21U
#define ARM_PROTOCOL_TARGET_PAYLOAD_LEN   13U
#define ARM_PROTOCOL_PUMP_PAYLOAD_LEN     1U
#define ARM_PROTOCOL_FEEDBACK_PAYLOAD_LEN 17U

extern volatile uint8_t debug_serial_target_type;
extern volatile float debug_serial_target_x_m;
extern volatile float debug_serial_target_y_m;
extern volatile float debug_serial_target_z_m;
extern volatile uint32_t debug_serial_target_rx_count;
extern volatile uint8_t debug_serial_pump_on;
extern volatile uint32_t debug_serial_pump_rx_count;
extern volatile uint8_t debug_serial_pump_off_deferred;
extern volatile uint8_t debug_serial_waiting_next_grasp;
extern volatile uint8_t debug_serial_usb_connected;
extern volatile uint8_t debug_serial_feedback_state;
extern volatile float debug_serial_feedback_x_m;
extern volatile float debug_serial_feedback_y_m;
extern volatile float debug_serial_feedback_z_m;
extern volatile float debug_serial_feedback_theta1_rad;
extern volatile uint32_t debug_serial_feedback_tx_count;
extern volatile uint32_t debug_serial_feedback_tx_fail_count;
extern volatile uint8_t debug_serial_last_tx_result;
extern volatile uint32_t debug_serial_checksum_error_count;
extern volatile uint32_t debug_serial_rejected_command_count;

void Arm_Serial_Protocol_Init(void);
void Arm_Serial_Protocol_Receive(const uint8_t* data, uint32_t length);
void Arm_Serial_Protocol_QueueTarget(uint8_t target_type,
                                      float x_m,
                                      float y_m,
                                      float z_m);
void Arm_Serial_Protocol_QueuePump(uint8_t pump_on);
void Arm_Serial_Protocol_Process(void);
/* Increments once after a PLACE target is reached and the pump is released. */
uint32_t Arm_Serial_Protocol_PlaceCycleSequence(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_ARM_SERIAL_PROTOCOL_H_ */
