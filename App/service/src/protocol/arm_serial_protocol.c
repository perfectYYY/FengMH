/*
 * arm_serial_protocol.c - standalone legacy arm protocol parser.
 */
#include "arm_serial_protocol.h"

#include "arm_control.h"
#include "arm_legacy_compat.h"
#include "bsp_time.h"
#include "bsp_usb_cdc.h"
#include "config.h"
#include "pump_control.h"

#include <math.h>
#include <string.h>

#define RX_STREAM_CAPACITY       64U
#define MAX_PROTOCOL_PAYLOAD     32U
#define FEEDBACK_PERIOD_MS       20U
#define FEEDBACK_RETRY_PERIOD_MS 5U
#define METERS_TO_MILLIMETERS    1000.0f
#define MILLIMETERS_TO_METERS    0.001f

typedef struct {
    uint8_t type;
    float x_m;
    float y_m;
    float z_m;
} Arm_Target_Command_t;

volatile uint8_t debug_serial_target_type;
volatile float debug_serial_target_x_m;
volatile float debug_serial_target_y_m;
volatile float debug_serial_target_z_m;
volatile uint32_t debug_serial_target_rx_count;
volatile uint8_t debug_serial_pump_on;
volatile uint32_t debug_serial_pump_rx_count;
volatile uint8_t debug_serial_pump_off_deferred;
volatile uint8_t debug_serial_waiting_next_grasp;
volatile uint8_t debug_serial_usb_connected;
volatile uint8_t debug_serial_feedback_state;
volatile float debug_serial_feedback_x_m;
volatile float debug_serial_feedback_y_m;
volatile float debug_serial_feedback_z_m;
volatile float debug_serial_feedback_theta1_rad;
volatile uint32_t debug_serial_feedback_tx_count;
volatile uint32_t debug_serial_feedback_tx_fail_count;
volatile uint8_t debug_serial_last_tx_result;
volatile uint32_t debug_serial_checksum_error_count;
volatile uint32_t debug_serial_rejected_command_count;

static uint8_t s_rx_stream[RX_STREAM_CAPACITY];
static uint32_t s_rx_length;
static Arm_Target_Command_t s_pending_target;
static volatile uint32_t s_pending_target_sequence;
static uint32_t s_handled_target_sequence;
static uint8_t s_pending_pump_on;
static volatile uint32_t s_pending_pump_sequence;
static uint32_t s_handled_pump_sequence;
static uint8_t s_deferred_pump_off;
static uint8_t s_waiting_for_next_grasp;
static volatile uint32_t s_place_cycle_sequence;
static uint32_t s_last_feedback_tick;
static uint32_t s_last_feedback_attempt_tick;

static uint8_t calculate_checksum(const uint8_t* data, uint32_t length) {
    uint8_t checksum = 0U;
    for (uint32_t i = 0U; i < length; i++) {
        checksum = (uint8_t)(checksum + data[i]);
    }
    return checksum;
}

static float read_f32_le(const uint8_t* data) {
    uint32_t bits = ((uint32_t)data[0]) |
                    ((uint32_t)data[1] << 8U) |
                    ((uint32_t)data[2] << 16U) |
                    ((uint32_t)data[3] << 24U);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void write_f32_le(uint8_t* data, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    data[0] = (uint8_t)bits;
    data[1] = (uint8_t)(bits >> 8U);
    data[2] = (uint8_t)(bits >> 16U);
    data[3] = (uint8_t)(bits >> 24U);
}

static void discard_stream_prefix(uint32_t count) {
    if (count >= s_rx_length) {
        s_rx_length = 0U;
        return;
    }
    memmove(s_rx_stream, &s_rx_stream[count], s_rx_length - count);
    s_rx_length -= count;
}

static void accept_target_payload(const uint8_t* payload) {
    Arm_Target_Command_t target = {
        .type = payload[0],
        .x_m = read_f32_le(&payload[1]),
        .y_m = read_f32_le(&payload[5]),
        .z_m = read_f32_le(&payload[9]),
    };

    if (target.type > 1U || !isfinite(target.x_m) ||
        !isfinite(target.y_m) || !isfinite(target.z_m)) {
        debug_serial_rejected_command_count++;
        return;
    }

    s_pending_target = target;
    s_pending_target_sequence++;
    debug_serial_target_type = target.type;
    debug_serial_target_x_m = target.x_m;
    debug_serial_target_y_m = target.y_m;
    debug_serial_target_z_m = target.z_m;
    debug_serial_target_rx_count++;
}

void Arm_Serial_Protocol_QueueTarget(uint8_t target_type,
                                      float x_m,
                                      float y_m,
                                      float z_m) {
    Arm_Target_Command_t target = {
        .type = target_type,
        .x_m = x_m,
        .y_m = y_m,
        .z_m = z_m,
    };

    if (target.type > 1U || !isfinite(target.x_m) ||
        !isfinite(target.y_m) || !isfinite(target.z_m)) {
        debug_serial_rejected_command_count++;
        return;
    }

    s_pending_target = target;
    s_pending_target_sequence++;
    debug_serial_target_type = target.type;
    debug_serial_target_x_m = target.x_m;
    debug_serial_target_y_m = target.y_m;
    debug_serial_target_z_m = target.z_m;
    debug_serial_target_rx_count++;
}

void Arm_Serial_Protocol_QueuePump(uint8_t pump_on) {
    if (pump_on > 1U) {
        debug_serial_rejected_command_count++;
        return;
    }

    s_pending_pump_on = pump_on;
    s_pending_pump_sequence++;
    debug_serial_pump_on = pump_on;
    debug_serial_pump_rx_count++;
}

static void parse_stream(void) {
    for (;;) {
        uint32_t header = 0U;
        while (header + 1U < s_rx_length &&
               (s_rx_stream[header] != ARM_PROTOCOL_HEAD1 ||
                s_rx_stream[header + 1U] != ARM_PROTOCOL_HEAD2)) {
            header++;
        }
        if (header > 0U) discard_stream_prefix(header);
        if (s_rx_length < 4U) return;

        uint32_t payload_length = s_rx_stream[3];
        if (payload_length > MAX_PROTOCOL_PAYLOAD) {
            discard_stream_prefix(1U);
            continue;
        }

        uint32_t frame_length = payload_length + 5U;
        if (s_rx_length < frame_length) return;

        if (calculate_checksum(s_rx_stream, frame_length - 1U) !=
            s_rx_stream[frame_length - 1U]) {
            debug_serial_checksum_error_count++;
            discard_stream_prefix(1U);
            continue;
        }

        if (s_rx_stream[2] == ARM_PROTOCOL_FUNC_TARGET &&
            payload_length == ARM_PROTOCOL_TARGET_PAYLOAD_LEN) {
            accept_target_payload(&s_rx_stream[4]);
        } else if (s_rx_stream[2] == ARM_PROTOCOL_FUNC_PUMP &&
                   payload_length == ARM_PROTOCOL_PUMP_PAYLOAD_LEN) {
            Arm_Serial_Protocol_QueuePump(s_rx_stream[4]);
        }
        discard_stream_prefix(frame_length);
    }
}

static uint8_t arm_feedback_state(void) {
    Arm_Move_Status_t status = Arm_Control_GetMoveStatus();
    if (status == ARM_MOVE_MOVING || status == ARM_MOVE_SETTLING) return 1U;
    if (status == ARM_MOVE_REACHED) return 2U;
    return 0U;
}

static void process_pending_target(void) {
    if (s_pending_target_sequence == s_handled_target_sequence) return;
    s_handled_target_sequence = s_pending_target_sequence;

    if (s_waiting_for_next_grasp) {
        if (s_pending_target.type == 1U) return;
        s_waiting_for_next_grasp = 0U;
        debug_serial_waiting_next_grasp = 0U;
    }

    if (Arm_Control_IsGravityOnlyMode()) return;

    if (arm_control_validate_host_target(s_pending_target.type,
                                         s_pending_target.x_m,
                                         s_pending_target.y_m,
                                         s_pending_target.z_m) != APP_OK) {
        debug_serial_rejected_command_count++;
        return;
    }

    Arm_Pose_t pose = {
        .x = s_pending_target.x_m * METERS_TO_MILLIMETERS,
        .y = s_pending_target.y_m * METERS_TO_MILLIMETERS,
        .z = s_pending_target.z_m * METERS_TO_MILLIMETERS,
        .pitch = 0.0f,
    };
    (void)Arm_Control_MoveToTypedPose(&pose, s_pending_target.type);
}

static void finish_place_cycle(void) {
    s_deferred_pump_off = 0U;
    debug_serial_pump_off_deferred = 0U;
    Pump_Control_Set(0U);
    Arm_Control_SetGravityMode();
    s_waiting_for_next_grasp = 1U;
    debug_serial_waiting_next_grasp = 1U;
    s_place_cycle_sequence++;
}

static void process_pending_pump(void) {
    if (Arm_Control_IsGravityOnlyMode()) {
        s_deferred_pump_off = 0U;
        debug_serial_pump_off_deferred = 0U;
        Pump_Control_Set(0U);
        return;
    }

    if (s_waiting_for_next_grasp) {
        s_handled_pump_sequence = s_pending_pump_sequence;
        s_deferred_pump_off = 0U;
        debug_serial_pump_off_deferred = 0U;
        Pump_Control_Set(0U);
        return;
    }

    if (s_pending_pump_sequence != s_handled_pump_sequence) {
        s_handled_pump_sequence = s_pending_pump_sequence;
        if (s_pending_pump_on) {
            s_deferred_pump_off = 0U;
            debug_serial_pump_off_deferred = 0U;
            Pump_Control_Set(1U);
        } else if (s_pending_target.type == 1U &&
                   Arm_Control_GetMoveStatus() != ARM_MOVE_REACHED) {
            s_deferred_pump_off = 1U;
            debug_serial_pump_off_deferred = 1U;
        } else {
            Pump_Control_Set(0U);
            if (s_pending_target.type == 1U &&
                Arm_Control_GetMoveStatus() == ARM_MOVE_REACHED) {
                finish_place_cycle();
            }
        }
    }

    if (s_deferred_pump_off && s_pending_target.type == 1U &&
        Arm_Control_GetMoveStatus() == ARM_MOVE_REACHED) {
        finish_place_cycle();
    }
}

static void send_feedback(uint32_t now) {
    uint8_t frame[ARM_PROTOCOL_FEEDBACK_PAYLOAD_LEN + 5U];
    Arm_Joint_Angles_t angles = {0};
    Arm_Pose_t pose = {0};

    Arm_Control_GetCurrentAngles(&angles);
    Arm_Forward_Kinematics(&arm_params, &angles, &pose);

    frame[0] = ARM_PROTOCOL_HEAD1;
    frame[1] = ARM_PROTOCOL_HEAD2;
    frame[2] = ARM_PROTOCOL_FUNC_FEEDBACK;
    frame[3] = ARM_PROTOCOL_FEEDBACK_PAYLOAD_LEN;

    uint8_t state = arm_feedback_state();
    float x_m = pose.x * MILLIMETERS_TO_METERS;
    float y_m = pose.y * MILLIMETERS_TO_METERS;
    float z_m = pose.z * MILLIMETERS_TO_METERS;

    frame[4] = state;
    write_f32_le(&frame[5], x_m);
    write_f32_le(&frame[9], y_m);
    write_f32_le(&frame[13], z_m);
    write_f32_le(&frame[17], angles.theta1_geo);
    frame[21] = calculate_checksum(frame, 21U);

    debug_serial_feedback_state = state;
    debug_serial_feedback_x_m = x_m;
    debug_serial_feedback_y_m = y_m;
    debug_serial_feedback_z_m = z_m;
    debug_serial_feedback_theta1_rad = angles.theta1_geo;

    s_last_feedback_attempt_tick = now;
#if APP_ARM_LEGACY_SERIAL_FEEDBACK_ENABLE
    app_err_t err = bsp_usb_cdc_send(frame, sizeof(frame));
    debug_serial_last_tx_result = (uint8_t)(err == APP_OK ? 0U : 1U);
    if (err == APP_OK) {
        s_last_feedback_tick = now;
        debug_serial_feedback_tx_count++;
    } else {
        debug_serial_feedback_tx_fail_count++;
    }
#else
    (void)frame;
    s_last_feedback_tick = now;
    debug_serial_last_tx_result = 0U;
#endif
}

void Arm_Serial_Protocol_Init(void) {
    s_rx_length = 0U;
    s_pending_target = (Arm_Target_Command_t){0};
    s_pending_target_sequence = 0U;
    s_handled_target_sequence = 0U;
    s_pending_pump_on = 0U;
    s_pending_pump_sequence = 0U;
    s_handled_pump_sequence = 0U;
    s_deferred_pump_off = 0U;
    s_waiting_for_next_grasp = 0U;
    s_place_cycle_sequence = 0U;
    debug_serial_target_type = 0U;
    debug_serial_target_x_m = 0.0f;
    debug_serial_target_y_m = 0.0f;
    debug_serial_target_z_m = 0.0f;
    debug_serial_target_rx_count = 0U;
    debug_serial_pump_on = 0U;
    debug_serial_pump_rx_count = 0U;
    debug_serial_pump_off_deferred = 0U;
    debug_serial_waiting_next_grasp = 0U;
    debug_serial_usb_connected = 0U;
    debug_serial_feedback_state = 0U;
    debug_serial_feedback_x_m = 0.0f;
    debug_serial_feedback_y_m = 0.0f;
    debug_serial_feedback_z_m = 0.0f;
    debug_serial_feedback_theta1_rad = 0.0f;
    debug_serial_feedback_tx_count = 0U;
    debug_serial_feedback_tx_fail_count = 0U;
    debug_serial_last_tx_result = 0U;
    debug_serial_checksum_error_count = 0U;
    debug_serial_rejected_command_count = 0U;
    s_last_feedback_tick = (uint32_t)bsp_time_now_ms();
    s_last_feedback_attempt_tick = s_last_feedback_tick;
}

uint32_t Arm_Serial_Protocol_PlaceCycleSequence(void) {
    return s_place_cycle_sequence;
}

void Arm_Serial_Protocol_Receive(const uint8_t* data, uint32_t length) {
    if (!data) return;

    for (uint32_t i = 0U; i < length; i++) {
        if (s_rx_length == RX_STREAM_CAPACITY) {
            discard_stream_prefix(1U);
        }
        s_rx_stream[s_rx_length++] = data[i];
        parse_stream();
    }
}

void Arm_Serial_Protocol_Process(void) {
    uint32_t now = (uint32_t)bsp_time_now_ms();
    debug_serial_usb_connected = 1U;
    process_pending_target();
    process_pending_pump();
    if ((now - s_last_feedback_tick) >= FEEDBACK_PERIOD_MS &&
        (now - s_last_feedback_attempt_tick) >= FEEDBACK_RETRY_PERIOD_MS) {
        send_feedback(now);
    }
}
