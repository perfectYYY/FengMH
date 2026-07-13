/*
 * task_arm.c - Arm task wrapper.
 *
 * The arm control loop is a resident subsystem. Robot mode only gates which USB
 * commands are consumed; it does not stop the arm from holding its current pose.
 */
#include "task_arm.h"

#include "arm_control.h"
#include "arm_legacy_compat.h"
#include "arm_serial_protocol.h"
#include "bsp_time.h"
#include "config.h"
#include "log.h"
#include "motor_registry.h"
#include "proto_defs.h"
#include "pump_control.h"
#include "task_comm.h"
#include "task_safety.h"

#include <math.h>
#include <string.h>

#if APP_TARGET_MCU
#include "cmsis_os.h"
#endif

static const char* TAG = "ARM";

static uint32_t s_last_target_seq;
static uint32_t s_last_pump_seq;
static uint32_t s_last_feedback_tx_ms;
static uint32_t s_last_feedback_attempt_ms;
static uint32_t s_last_place_cycle_sequence;
static uint32_t s_place_hold_start_ms;

static const uint32_t ARM_FEEDBACK_TX_INTERVAL_MS = 20U;  /* 50 Hz */
static const uint32_t ARM_FEEDBACK_RETRY_INTERVAL_MS = 5U;
static const uint32_t ARM_FIXED_ERROR_RETRY_MS = 500U;
static const uint32_t ARM_DEBUG_SNAPSHOT_INTERVAL_MS = 100U;

static uint8_t s_estop_was_active;
static uint8_t s_gravity_only_override_active;
static uint8_t s_last_robot_mode;
static uint8_t s_place_hold_active;
static uint8_t s_last_box_position_select;

static uint32_t s_fixed_error_ms;
static uint32_t s_last_debug_snapshot_ms;
static uint32_t s_power_on_fixed_grasp_start_ms;
static uint32_t s_active_target_command_seq;
static uint32_t s_active_pump_command_seq;
static uint8_t s_active_target_status_terminal;
static uint8_t s_active_pump_status_terminal;
static uint8_t s_active_pump_desired;
static uint8_t s_stow_active;

static void force_main_pump_off(void);

typedef enum {
    ARM_FIXED_PROFILE_PARK = 0,
    ARM_FIXED_PROFILE_FIRST_LIFT = 1,
    ARM_FIXED_PROFILE_FIRST_J1 = 2,
    ARM_FIXED_PROFILE_SECOND_SAFE_LIFT = 3,
    ARM_FIXED_PROFILE_SECOND_J1 = 4,
    ARM_FIXED_PROFILE_SECOND_FIXED_LIFT = 5,
} arm_fixed_profile_t;

static arm_fixed_profile_t s_fixed_profile;

volatile uint8_t debug_arm_force_gravity_only;
volatile task_arm_debug_snapshot_t g_arm_debug_snapshot;
volatile uint8_t debug_arm_fixed_state;
volatile uint8_t debug_arm_fixed_profile;
volatile uint8_t debug_arm_box_position_select;
volatile float debug_arm_fixed_j2_motor_deg;
volatile float debug_arm_fixed_j3_motor_deg;
volatile float debug_arm_fixed_j4_motor_deg;
volatile int32_t debug_arm_fixed_last_result;

static float nearest_equivalent_angle_rad(float target_deg, float reference_rad) {
    float target_rad = APP_ARM_DEG2RAD(target_deg);
    while ((target_rad - reference_rad) > APP_ARM_PI) {
        target_rad -= 2.0f * APP_ARM_PI;
    }
    while ((target_rad - reference_rad) < -APP_ARM_PI) {
        target_rad += 2.0f * APP_ARM_PI;
    }
    return target_rad;
}

static uint8_t selected_box_position(void) {
    return (debug_arm_box_position_select == 2U) ? 2U : 1U;
}

static float selected_box_j1_motor_deg(void) {
    return (selected_box_position() == 2U) ?
           APP_ARM_SECOND_WAIT_J1_MOTOR_DEG :
           APP_ARM_FIRST_WAIT_J1_MOTOR_DEG;
}

static void reset_fixed_hold(arm_fixed_profile_t profile) {
    s_fixed_profile = profile;
    debug_arm_fixed_profile = (uint8_t)profile;
    debug_arm_fixed_state = ARM_FIXED_WAIT_FEEDBACK;
    if (profile == ARM_FIXED_PROFILE_PARK) {
        debug_arm_fixed_j2_motor_deg = APP_ARM_PARK_J2_MOTOR_DEG;
        debug_arm_fixed_j3_motor_deg = APP_ARM_PARK_J3_MOTOR_DEG;
    } else {
        debug_arm_fixed_j2_motor_deg = APP_ARM_FIXED_J2_MOTOR_DEG;
        debug_arm_fixed_j3_motor_deg = APP_ARM_FIXED_J3_MOTOR_DEG;
    }
    debug_arm_fixed_j4_motor_deg =
        -85.0f - ARM_KINEMATICS_MOTOR3_TO_LOGICAL(debug_arm_fixed_j3_motor_deg);
    debug_arm_fixed_last_result = APP_OK;
    (void)arm_control_set_j1_free_mode(0U);
    (void)arm_control_set_gravity_mode();
}

static app_err_t command_fixed_pose(void) {
    Arm_Joint_Angles_t current;
    memset(&current, 0, sizeof(current));
    Arm_Control_GetCurrentAngles(&current);

    arm_joint_angles_t target;
    memset(&target, 0, sizeof(target));
    target.theta1_motor_rad = current.theta1_motor;
    target.theta2_motor_rad = current.theta2_motor;
    target.theta3_motor_rad = current.theta3_motor;

    if (s_fixed_profile == ARM_FIXED_PROFILE_PARK) {
        target.theta1_motor_rad =
            nearest_equivalent_angle_rad(APP_ARM_PARK_J1_MOTOR_DEG,
                                         current.theta1_motor);
        target.theta2_motor_rad = APP_ARM_DEG2RAD(APP_ARM_PARK_J2_MOTOR_DEG);
        target.theta3_motor_rad = ARM_KINEMATICS_MOTOR3_TO_LOGICAL(
            APP_ARM_DEG2RAD(APP_ARM_PARK_J3_MOTOR_DEG));
    } else if (s_fixed_profile == ARM_FIXED_PROFILE_FIRST_LIFT) {
        target.theta2_motor_rad = APP_ARM_DEG2RAD(APP_ARM_FIXED_J2_MOTOR_DEG);
        target.theta3_motor_rad = ARM_KINEMATICS_MOTOR3_TO_LOGICAL(
            APP_ARM_DEG2RAD(APP_ARM_FIXED_J3_MOTOR_DEG));
    } else if (s_fixed_profile == ARM_FIXED_PROFILE_FIRST_J1) {
        target.theta1_motor_rad =
            nearest_equivalent_angle_rad(selected_box_j1_motor_deg(),
                                         current.theta1_motor);
        target.theta2_motor_rad = APP_ARM_DEG2RAD(APP_ARM_FIXED_J2_MOTOR_DEG);
        target.theta3_motor_rad = ARM_KINEMATICS_MOTOR3_TO_LOGICAL(
            APP_ARM_DEG2RAD(APP_ARM_FIXED_J3_MOTOR_DEG));
    } else if (s_fixed_profile == ARM_FIXED_PROFILE_SECOND_SAFE_LIFT) {
        target.theta2_motor_rad = APP_ARM_DEG2RAD(APP_ARM_SAFE_MOVE_J2_DEG);
        target.theta3_motor_rad = APP_ARM_DEG2RAD(APP_ARM_SAFE_MOVE_J3_DEG);
    } else if (s_fixed_profile == ARM_FIXED_PROFILE_SECOND_J1) {
        target.theta1_motor_rad =
            nearest_equivalent_angle_rad(APP_ARM_SECOND_WAIT_J1_MOTOR_DEG,
                                         current.theta1_motor);
        target.theta2_motor_rad = APP_ARM_DEG2RAD(APP_ARM_SAFE_MOVE_J2_DEG);
        target.theta3_motor_rad = APP_ARM_DEG2RAD(APP_ARM_SAFE_MOVE_J3_DEG);
    } else {
        target.theta1_motor_rad =
            nearest_equivalent_angle_rad(APP_ARM_SECOND_WAIT_J1_MOTOR_DEG,
                                         current.theta1_motor);
        target.theta2_motor_rad = APP_ARM_DEG2RAD(APP_ARM_FIXED_J2_MOTOR_DEG);
        target.theta3_motor_rad = ARM_KINEMATICS_MOTOR3_TO_LOGICAL(
            APP_ARM_DEG2RAD(APP_ARM_FIXED_J3_MOTOR_DEG));
    }

    /* J4继续由原耦合公式自动生成；等待位 J1 使用位置控制，防止被外力带偏。 */
    (void)arm_control_set_j1_free_mode(0U);
    app_err_t result = arm_control_set_joint_target(ARM_CONTROL_TARGET_INTERNAL_HOLD,
                                                    &target);
    debug_arm_fixed_last_result = result;
    debug_arm_fixed_state =
        (result == APP_OK) ? ARM_FIXED_MOVING : ARM_FIXED_ERROR;
    return result;
}

static uint8_t angle_close_rad(float a_rad, float b_rad, float tolerance_rad) {
    return (fabsf(a_rad - b_rad) <= tolerance_rad) ? 1U : 0U;
}

static uint8_t staged_profile_ready(void) {
    const float tolerance = APP_ARM_DEG2RAD(3.0f);
    Arm_Joint_Angles_t current;
    memset(&current, 0, sizeof(current));
    Arm_Control_GetCurrentAngles(&current);

    if (s_fixed_profile == ARM_FIXED_PROFILE_FIRST_LIFT) {
        return (uint8_t)(
            angle_close_rad(current.theta2_motor,
                            APP_ARM_DEG2RAD(APP_ARM_FIXED_J2_MOTOR_DEG),
                            tolerance) &&
            angle_close_rad(current.theta3_motor,
                            ARM_KINEMATICS_MOTOR3_TO_LOGICAL(
                                APP_ARM_DEG2RAD(APP_ARM_FIXED_J3_MOTOR_DEG)),
                            tolerance));
    }

    if (s_fixed_profile == ARM_FIXED_PROFILE_SECOND_SAFE_LIFT) {
        return (uint8_t)(
            angle_close_rad(current.theta2_motor,
                            APP_ARM_DEG2RAD(APP_ARM_SAFE_MOVE_J2_DEG),
                            tolerance) &&
            angle_close_rad(current.theta3_motor,
                            APP_ARM_DEG2RAD(APP_ARM_SAFE_MOVE_J3_DEG),
                            tolerance));
    }

    if (s_fixed_profile == ARM_FIXED_PROFILE_SECOND_J1) {
        const float target_j1 =
            nearest_equivalent_angle_rad(APP_ARM_SECOND_WAIT_J1_MOTOR_DEG,
                                         current.theta1_motor);
        return angle_close_rad(current.theta1_motor, target_j1, tolerance);
    }

    return 0U;
}

static void process_power_on_fixed_grasp_test(void) {
#if APP_ARM_POWER_ON_FIXED_GRASP_TEST
    uint8_t selected = selected_box_position();
    if (s_last_box_position_select != selected) {
        s_last_box_position_select = selected;
        if (s_fixed_profile == ARM_FIXED_PROFILE_FIRST_J1 ||
            s_fixed_profile == ARM_FIXED_PROFILE_SECOND_J1 ||
            s_fixed_profile == ARM_FIXED_PROFILE_SECOND_FIXED_LIFT ||
            debug_arm_fixed_state == ARM_FIXED_HOLDING) {
            reset_fixed_hold(ARM_FIXED_PROFILE_FIRST_J1);
        }
    }
#endif
}

static uint8_t power_on_fixed_grasp_delay_done(uint32_t now_ms) {
#if APP_ARM_POWER_ON_FIXED_GRASP_TEST
    return ((now_ms - s_power_on_fixed_grasp_start_ms) >=
            APP_ARM_POWER_ON_FIXED_GRASP_DELAY_MS) ? 1U : 0U;
#else
    (void)now_ms;
    return 1U;
#endif
}

static void advance_staged_profile(void) {
    if (s_fixed_profile == ARM_FIXED_PROFILE_FIRST_LIFT) {
        s_fixed_profile = ARM_FIXED_PROFILE_FIRST_J1;
    } else if (s_fixed_profile == ARM_FIXED_PROFILE_SECOND_SAFE_LIFT) {
        s_fixed_profile = ARM_FIXED_PROFILE_SECOND_J1;
    } else if (s_fixed_profile == ARM_FIXED_PROFILE_SECOND_J1) {
        s_fixed_profile = ARM_FIXED_PROFILE_SECOND_FIXED_LIFT;
    } else {
        return;
    }
    debug_arm_fixed_profile = (uint8_t)s_fixed_profile;
    debug_arm_fixed_state = ARM_FIXED_WAIT_FEEDBACK;
}

static void process_fixed_hold(uint32_t now_ms) {
    arm_control_status_t status;
    arm_control_get_status(&status);

    if (debug_arm_fixed_state == ARM_FIXED_WAIT_FEEDBACK) {
        if (!status.motor_feedback_fresh) return;
        if (command_fixed_pose() != APP_OK) {
            s_fixed_error_ms = now_ms;
        }
        return;
    }

    if (debug_arm_fixed_state == ARM_FIXED_MOVING) {
        Arm_Move_Status_t move_status = Arm_Control_GetMoveStatus();
        if (staged_profile_ready()) {
            advance_staged_profile();
        } else if (move_status == ARM_MOVE_REACHED) {
            if (s_fixed_profile == ARM_FIXED_PROFILE_FIRST_LIFT) {
                advance_staged_profile();
            } else if (s_fixed_profile == ARM_FIXED_PROFILE_SECOND_SAFE_LIFT) {
                advance_staged_profile();
            } else if (s_fixed_profile == ARM_FIXED_PROFILE_SECOND_J1) {
                advance_staged_profile();
            } else {
                debug_arm_fixed_state = ARM_FIXED_HOLDING;
            }
        } else if (move_status >= ARM_MOVE_ERROR_INVALID_TARGET) {
            debug_arm_fixed_last_result = (int32_t)move_status;
            debug_arm_fixed_state = ARM_FIXED_ERROR;
            s_fixed_error_ms = now_ms;
        }
        return;
    }

    if (debug_arm_fixed_state == ARM_FIXED_HOLDING) {
        if (status.motor_feedback_fresh && status.target_valid) return;

        int32_t lost_result = (int32_t)Arm_Control_GetMoveStatus();
        reset_fixed_hold(s_fixed_profile);
        debug_arm_fixed_last_result = lost_result;
        return;
    }

    if (debug_arm_fixed_state == ARM_FIXED_ERROR) {
        if (!status.motor_feedback_fresh ||
            (now_ms - s_fixed_error_ms) < ARM_FIXED_ERROR_RETRY_MS) {
            return;
        }

        reset_fixed_hold(s_fixed_profile);
        if (command_fixed_pose() != APP_OK) {
            s_fixed_error_ms = now_ms;
        }
    }
}

static uint8_t host_has_new_valid_grasp_target(void) {
    task_comm_arm_target_t target;
    task_comm_get_arm_target(&target);
    if (target.seq == s_last_target_seq ||
        target.target_type != PROTO_ARM_TARGET_GRASP) {
        return 0U;
    }

    if (arm_control_validate_host_target(target.target_type,
                                         target.x_m,
                                         target.y_m,
                                         target.z_m) != APP_OK) {
        /* 错误抓取坐标只消费本条 target；保持固定姿态并等待下一条 GRASP。 */
        s_last_target_seq = target.seq;
        debug_arm_fixed_last_result = APP_ERR_INVALID_ARG;
        return 0U;
    }
    return 1U;
}

static void release_fixed_pose_to_host(void) {
    /*
     * 恢复 J1 正常位置控制，但不先切重补清空轨迹。待处理的上位机目标会在
     * 本控制拍内覆盖内部待机目标，从当前位置连续重规划。
    */
    (void)arm_control_set_j1_free_mode(0U);
    debug_arm_fixed_state = ARM_FIXED_RELEASED_TO_HOST;
    debug_arm_fixed_last_result = APP_OK;
}

static uint8_t current_robot_mode(void) {
    task_comm_mode_cmd_t mode;
    task_comm_get_mode_cmd(&mode);
    return (mode.seq > 0U) ? mode.mode : PROTO_ROBOT_MODE_IDLE;
}

static uint8_t arm_work_mode(uint8_t robot_mode) {
    return (robot_mode == PROTO_ROBOT_MODE_ARM ||
            robot_mode == PROTO_ROBOT_MODE_REAR_PLACE) ? 1U : 0U;
}

static uint8_t arm_usb_commands_allowed(uint8_t robot_mode) {
    return arm_work_mode(robot_mode);
}

static uint8_t consume_stow_command_if_pending(uint8_t robot_mode) {
    task_comm_arm_target_t target;
    task_comm_get_arm_target(&target);
    if (target.seq == s_last_target_seq ||
        target.target_type != PROTO_ARM_TARGET_STOW) {
        return 0U;
    }

    s_last_target_seq = target.seq;
    s_active_target_command_seq = target.command_seq;
    s_active_target_status_terminal = 0U;
    if (robot_mode != PROTO_ROBOT_MODE_ARM) {
        (void)task_comm_report_command_status(
            PROTO_FUNC_ARM_TARGET,
            target.command_seq,
            PROTO_COMMAND_STAGE_REJECTED,
            PROTO_COMMAND_RESULT_WRONG_MODE);
        s_active_target_status_terminal = 1U;
        return 1U;
    }

    Arm_Serial_Protocol_Init();
    s_last_place_cycle_sequence = Arm_Serial_Protocol_PlaceCycleSequence();
    force_main_pump_off();
    s_place_hold_active = 0U;
    s_stow_active = 1U;
    reset_fixed_hold(ARM_FIXED_PROFILE_PARK);
    (void)task_comm_report_command_status(
        PROTO_FUNC_ARM_TARGET,
        target.command_seq,
        PROTO_COMMAND_STAGE_EXECUTING,
        PROTO_COMMAND_RESULT_OK);
    return 1U;
}

static void report_target_progress(void) {
    if (s_active_target_status_terminal || s_stow_active) return;
    Arm_Move_Status_t move_status = Arm_Control_GetMoveStatus();
    if (move_status == ARM_MOVE_REACHED) {
        (void)task_comm_report_command_status(
            PROTO_FUNC_ARM_TARGET,
            s_active_target_command_seq,
            PROTO_COMMAND_STAGE_COMPLETED,
            PROTO_COMMAND_RESULT_OK);
        s_active_target_status_terminal = 1U;
    } else if (move_status >= ARM_MOVE_ERROR_INVALID_TARGET) {
        (void)task_comm_report_command_status(
            PROTO_FUNC_ARM_TARGET,
            s_active_target_command_seq,
            PROTO_COMMAND_STAGE_ERROR,
            PROTO_COMMAND_RESULT_INTERNAL);
        s_active_target_status_terminal = 1U;
    }
}

static void report_stow_progress(void) {
    if (!s_stow_active || s_active_target_status_terminal) return;
    if (debug_arm_fixed_state == ARM_FIXED_HOLDING &&
        s_fixed_profile == ARM_FIXED_PROFILE_PARK) {
        (void)task_comm_report_command_status(
            PROTO_FUNC_ARM_TARGET,
            s_active_target_command_seq,
            PROTO_COMMAND_STAGE_COMPLETED,
            PROTO_COMMAND_RESULT_OK);
        s_active_target_status_terminal = 1U;
        /* Keep PARK latched after reporting completion. The host needs time to
         * receive 0x88 and switch to NAV; a new arm target explicitly releases
         * this latch through consume_new_commands(). */
    } else if (debug_arm_fixed_state == ARM_FIXED_ERROR) {
        (void)task_comm_report_command_status(
            PROTO_FUNC_ARM_TARGET,
            s_active_target_command_seq,
            PROTO_COMMAND_STAGE_ERROR,
            PROTO_COMMAND_RESULT_INTERNAL);
        s_active_target_status_terminal = 1U;
        s_stow_active = 0U;
    }
}

static void report_pump_progress(void) {
    if (s_active_pump_status_terminal) return;
    if (task_comm_watchdog_active() || task_safety_estop_active()) {
        (void)task_comm_report_command_status(
            PROTO_FUNC_ARM_PUMP,
            s_active_pump_command_seq,
            PROTO_COMMAND_STAGE_ERROR,
            task_comm_watchdog_active() ? PROTO_COMMAND_RESULT_WATCHDOG :
                                          PROTO_COMMAND_RESULT_ESTOP);
        s_active_pump_status_terminal = 1U;
        return;
    }
    if ((Pump_Control_IsEnabled() ? 1U : 0U) == s_active_pump_desired) {
        (void)task_comm_report_command_status(
            PROTO_FUNC_ARM_PUMP,
            s_active_pump_command_seq,
            PROTO_COMMAND_STAGE_COMPLETED,
            PROTO_COMMAND_RESULT_OK);
        s_active_pump_status_terminal = 1U;
    }
}

static void consume_new_commands(uint8_t command_allowed) {
    task_comm_arm_target_t target;
    task_comm_arm_pump_t pump;

    task_comm_get_arm_target(&target);
    task_comm_get_arm_pump(&pump);

    if (!command_allowed) {
        s_last_target_seq = target.seq;
        s_last_pump_seq = pump.seq;
        return;
    }

    if (target.seq != s_last_target_seq) {
        if (target.target_type == PROTO_ARM_TARGET_STOW) {
            s_last_target_seq = target.seq;
        } else if (arm_control_validate_host_target(target.target_type,
                                                    target.x_m,
                                                    target.y_m,
                                                    target.z_m) != APP_OK) {
            (void)task_comm_report_command_status(
                PROTO_FUNC_ARM_TARGET,
                target.command_seq,
                PROTO_COMMAND_STAGE_REJECTED,
                PROTO_COMMAND_RESULT_INVALID_TARGET);
        } else {
            Arm_Serial_Protocol_QueueTarget(target.target_type,
                                             target.x_m,
                                             target.y_m,
                                             target.z_m);
            s_active_target_command_seq = target.command_seq;
            s_active_target_status_terminal = 0U;
            s_stow_active = 0U;
            (void)task_comm_report_command_status(
                PROTO_FUNC_ARM_TARGET,
                target.command_seq,
                PROTO_COMMAND_STAGE_EXECUTING,
                PROTO_COMMAND_RESULT_OK);
        }
        s_last_target_seq = target.seq;
    }

    if (pump.seq != s_last_pump_seq) {
        Arm_Serial_Protocol_QueuePump(pump.pump_on);
        s_active_pump_command_seq = pump.command_seq;
        s_active_pump_desired = pump.pump_on ? 1U : 0U;
        if (pump.pump_on) {
            s_active_pump_status_terminal = 0U;
            (void)task_comm_report_command_status(
                PROTO_FUNC_ARM_PUMP,
                pump.command_seq,
                PROTO_COMMAND_STAGE_EXECUTING,
                PROTO_COMMAND_RESULT_OK);
        } else {
            /* Communication ingress already executed pump-off and published
             * COMPLETED. Consume it only for PLACE-cycle bookkeeping. */
            s_active_pump_status_terminal = 1U;
        }
        s_last_pump_seq = pump.seq;
    }
}

static uint8_t legacy_feedback_state(void) {
    Arm_Move_Status_t status = Arm_Control_GetMoveStatus();
    if (status == ARM_MOVE_MOVING || status == ARM_MOVE_SETTLING) {
        return PROTO_ARM_STATE_MOVING;
    }
    if (status == ARM_MOVE_REACHED) {
        return PROTO_ARM_STATE_REACHED;
    }
    if (status >= ARM_MOVE_ERROR_INVALID_TARGET) {
        return PROTO_ARM_STATE_ERROR;
    }
    return PROTO_ARM_STATE_IDLE;
}

static void build_integrated_feedback(payload_arm_feedback_t* feedback) {
    if (!feedback) return;

    Arm_Joint_Angles_t angles = {0};
    Arm_Pose_t pose = {0};
    Arm_Control_GetCurrentAngles(&angles);
    Arm_Forward_Kinematics(&arm_params, &angles, &pose);

    feedback->arm_state = legacy_feedback_state();
    feedback->end_x_m = pose.x * 0.001f;
    feedback->end_y_m = pose.y * 0.001f;
    feedback->end_z_m = pose.z * 0.001f;
    feedback->theta1_rad = angles.theta1_geo;
}

static void build_motor_angles_feedback(payload_arm_motor_angles_t* angles) {
    if (!angles) return;

    const motor_logical_id_t motor_ids[4] = {
        MOTOR_ID_ARM_J1, MOTOR_ID_ARM_J2, MOTOR_ID_ARM_J3, MOTOR_ID_ARM_J4,
    };
    memset(angles, 0, sizeof(*angles));
    for (uint32_t i = 0U; i < 4U; i++) {
        const motor_dev_t* motor = motor_get(motor_ids[i]);
        if (motor && motor->state.online) {
            angles->online_mask |= (uint8_t)(1U << i);
            switch (i) {
                case 0U:
                    angles->j1_angle_rad = motor->state.angle_rad;
                    break;
                case 1U:
                    angles->j2_angle_rad = motor->state.angle_rad;
                    break;
                case 2U:
                    angles->j3_angle_rad = motor->state.angle_rad;
                    break;
                case 3U:
                    angles->j4_angle_rad = motor->state.angle_rad;
                    break;
                default:
                    break;
            }
        }
    }
}

static void send_feedback_if_due(uint32_t now_ms) {
    if ((now_ms - s_last_feedback_tx_ms) < ARM_FEEDBACK_TX_INTERVAL_MS) {
        return;
    }
    if ((now_ms - s_last_feedback_attempt_ms) < ARM_FEEDBACK_RETRY_INTERVAL_MS) {
        return;
    }

    payload_arm_feedback_t feedback;
    payload_arm_motor_angles_t motor_angles;
    build_integrated_feedback(&feedback);
    build_motor_angles_feedback(&motor_angles);
    s_last_feedback_attempt_ms = now_ms;
    if (task_comm_send_arm_feedback(&feedback) > 0) {
        s_last_feedback_tx_ms = now_ms;
    }
    (void)task_comm_send_arm_motor_angles(&motor_angles);
}

static uint32_t snapshot_motor_age_ms(const motor_dev_t* motor,
                                      uint32_t now_ms) {
    if (!motor || motor->state.rx_cnt == 0U) return UINT32_MAX;
    return now_ms - motor->state.last_rx_tick;
}

static void update_debug_snapshot_if_due(uint32_t now_ms) {
    if ((now_ms - s_last_debug_snapshot_ms) < ARM_DEBUG_SNAPSHOT_INTERVAL_MS) {
        return;
    }
    s_last_debug_snapshot_ms = now_ms;

    const motor_dev_t* motor1 = motor_get(MOTOR_ID_ARM_J1);
    const motor_dev_t* motor2 = motor_get(MOTOR_ID_ARM_J2);
    const motor_dev_t* motor3 = motor_get(MOTOR_ID_ARM_J3);
    const motor_dev_t* motor4 = motor_get(MOTOR_ID_ARM_J4);
    const uint32_t next_sequence = g_arm_debug_snapshot.update_sequence + 1U;
    g_arm_debug_snapshot.update_time_ms = now_ms;
    g_arm_debug_snapshot.motor1_position_deg = motor1 ?
        APP_ARM_RAD2DEG(motor1->state.angle_rad) : 0.0f;
    g_arm_debug_snapshot.motor2_position_deg = motor2 ?
        APP_ARM_RAD2DEG(motor2->state.angle_rad) : 0.0f;
    g_arm_debug_snapshot.motor3_position_deg = motor3 ?
        APP_ARM_RAD2DEG(motor3->state.angle_rad) : 0.0f;
    g_arm_debug_snapshot.motor4_position_deg = motor4 ?
        APP_ARM_RAD2DEG(motor4->state.angle_rad) : 0.0f;
    g_arm_debug_snapshot.motor1_response_age_ms =
        snapshot_motor_age_ms(motor1, now_ms);
    g_arm_debug_snapshot.motor2_response_age_ms =
        snapshot_motor_age_ms(motor2, now_ms);
    g_arm_debug_snapshot.motor3_response_age_ms =
        snapshot_motor_age_ms(motor3, now_ms);
    g_arm_debug_snapshot.motor4_response_age_ms =
        snapshot_motor_age_ms(motor4, now_ms);
    g_arm_debug_snapshot.host_target_x_m = debug_comm_arm_target_x_m;
    g_arm_debug_snapshot.host_target_y_m = debug_comm_arm_target_y_m;
    g_arm_debug_snapshot.host_target_z_m = debug_comm_arm_target_z_m;
    g_arm_debug_snapshot.current_end_x_m = debug_serial_feedback_x_m;
    g_arm_debug_snapshot.current_end_y_m = debug_serial_feedback_y_m;
    g_arm_debug_snapshot.current_end_z_m = debug_serial_feedback_z_m;
    /* 最后更新序号；现场看到序号变化，说明前面的字段已经写完。 */
    g_arm_debug_snapshot.update_sequence = next_sequence;
}

static void force_all_pump_outputs_off(void) {
    Pump_Control_Set(0U);
    Pump_Control_SetPC8(0U);
    Pump_Control_SetPC9(0U);
    Pump_Control_SetPA8(0U);
    Pump_Control_SetPA9(0U);
}

static void force_main_pump_off(void) {
    Pump_Control_Set(0U);
}

static void terminate_arm_commands_for_mode_edge(uint8_t result) {
    task_comm_arm_target_t target;
    task_comm_arm_pump_t pump;
    task_comm_get_arm_target(&target);
    task_comm_get_arm_pump(&pump);

    if (!s_active_target_status_terminal) {
        (void)task_comm_report_command_status(
            PROTO_FUNC_ARM_TARGET,
            s_active_target_command_seq,
            PROTO_COMMAND_STAGE_ERROR,
            result);
    }
    if (target.seq != s_last_target_seq) {
        (void)task_comm_report_command_status(
            PROTO_FUNC_ARM_TARGET,
            target.command_seq,
            PROTO_COMMAND_STAGE_ERROR,
            result);
        s_last_target_seq = target.seq;
    }

    if (!s_active_pump_status_terminal) {
        (void)task_comm_report_command_status(
            PROTO_FUNC_ARM_PUMP,
            s_active_pump_command_seq,
            PROTO_COMMAND_STAGE_ERROR,
            result);
    }
    if (pump.seq != s_last_pump_seq) {
        /* pump-off already completed synchronously in task_comm ingress. */
        if (pump.pump_on) {
            (void)task_comm_report_command_status(
                PROTO_FUNC_ARM_PUMP,
                pump.command_seq,
                PROTO_COMMAND_STAGE_ERROR,
                result);
        }
        s_last_pump_seq = pump.seq;
    }
}

static void reset_host_arm_protocol_state(void) {
    Arm_Serial_Protocol_Init();
    s_last_place_cycle_sequence = Arm_Serial_Protocol_PlaceCycleSequence();
    /* A/B rear-slot retention survives ARM/NAV/REAR_PLACE mode boundaries. */
    force_main_pump_off();
    s_place_hold_active = 0U;
    s_active_target_command_seq = 0U;
    s_active_pump_command_seq = 0U;
    s_active_target_status_terminal = 1U;
    s_active_pump_status_terminal = 1U;
    s_active_pump_desired = 0U;
    s_stow_active = 0U;
}

static void enter_gravity_only_override(void) {
    /* Clear every queued/active grasp-place state before enabling gravity only. */
    Arm_Serial_Protocol_Init();
    s_last_place_cycle_sequence = Arm_Serial_Protocol_PlaceCycleSequence();
    Arm_Control_SetGravityOnlyMode(1U);
    (void)arm_control_set_j1_free_mode(0U);
    force_all_pump_outputs_off();
    debug_arm_fixed_state = ARM_FIXED_GRAVITY_ONLY;
    debug_arm_fixed_last_result = APP_OK;
    s_place_hold_active = 0U;
    s_gravity_only_override_active = 1U;
}

static void process_gravity_only_override(void) {
    consume_new_commands(0U);
    if (!s_gravity_only_override_active) {
        enter_gravity_only_override();
    }
    if (!Arm_Control_IsGravityOnlyMode()) {
        Arm_Control_SetGravityOnlyMode(1U);
    }
    (void)arm_control_set_j1_free_mode(0U);
    force_all_pump_outputs_off();
    debug_arm_fixed_state = ARM_FIXED_GRAVITY_ONLY;
    Arm_Control_Process();
    Arm_Serial_Protocol_Process();
    force_all_pump_outputs_off();
    Pump_Control_Process();
}

static void leave_gravity_only_override(void) {
    s_gravity_only_override_active = 0U;
    Arm_Control_SetGravityOnlyMode(0U);
#if APP_ARM_POWER_ON_FIXED_GRASP_TEST
    reset_fixed_hold(ARM_FIXED_PROFILE_FIRST_LIFT);
#elif APP_ARM_POWER_ON_HOST_READY_ENABLE
    reset_fixed_hold(ARM_FIXED_PROFILE_FIRST_LIFT);
#else
    reset_fixed_hold(ARM_FIXED_PROFILE_PARK);
#endif
}

void task_arm_init(void) {
    s_last_target_seq = 0U;
    s_last_pump_seq = 0U;
    s_last_feedback_tx_ms = 0U;
    s_last_feedback_attempt_ms = 0U;
    s_last_place_cycle_sequence = 0U;
    s_place_hold_start_ms = 0U;
    s_estop_was_active = 0U;
    s_gravity_only_override_active = 0U;
    s_last_robot_mode = PROTO_ROBOT_MODE_IDLE;
    s_place_hold_active = 0U;
    s_last_box_position_select = 1U;
    s_fixed_error_ms = 0U;
    s_last_debug_snapshot_ms = 0U;
    s_power_on_fixed_grasp_start_ms = 0U;
    s_active_target_command_seq = 0U;
    s_active_pump_command_seq = 0U;
    s_active_target_status_terminal = 1U;
    s_active_pump_status_terminal = 1U;
    s_active_pump_desired = 0U;
    s_stow_active = 0U;
    s_fixed_profile =
#if APP_ARM_POWER_ON_FIXED_GRASP_TEST
        ARM_FIXED_PROFILE_FIRST_LIFT;
#elif APP_ARM_POWER_ON_HOST_READY_ENABLE
        ARM_FIXED_PROFILE_FIRST_LIFT;
#else
        ARM_FIXED_PROFILE_PARK;
#endif
    debug_arm_fixed_profile = (uint8_t)s_fixed_profile;
    debug_arm_force_gravity_only = 0U;
    debug_arm_box_position_select = 1U;
    memset((void*)&g_arm_debug_snapshot, 0, sizeof(g_arm_debug_snapshot));
    debug_arm_fixed_state = ARM_FIXED_WAIT_FEEDBACK;
    debug_arm_fixed_j2_motor_deg = APP_ARM_FIXED_J2_MOTOR_DEG;
    debug_arm_fixed_j3_motor_deg = APP_ARM_FIXED_J3_MOTOR_DEG;
    debug_arm_fixed_j4_motor_deg =
        -85.0f - ARM_KINEMATICS_MOTOR3_TO_LOGICAL(APP_ARM_FIXED_J3_MOTOR_DEG);
    debug_arm_fixed_last_result = APP_OK;
    Pump_Control_Init();
    Arm_Control_Init(NULL);
    Arm_Control_SetGravityOnlyMode(APP_ARM_FORCE_GRAVITY_ONLY ? 1U : 0U);
    Arm_Serial_Protocol_Init();
    if (!APP_ARM_FORCE_GRAVITY_ONLY) {
#if APP_ARM_POWER_ON_FIXED_GRASP_TEST
        reset_fixed_hold(ARM_FIXED_PROFILE_FIRST_LIFT);
#elif APP_ARM_POWER_ON_HOST_READY_ENABLE
        reset_fixed_hold(ARM_FIXED_PROFILE_FIRST_LIFT);
#else
        reset_fixed_hold(ARM_FIXED_PROFILE_PARK);
#endif
    }
    LOGI("arm task init: force_gravity_only=%u",
         (unsigned)APP_ARM_FORCE_GRAVITY_ONLY);
}

void task_arm_step_for_test(float dt_s, uint32_t now_ms) {
    (void)dt_s;
#if APP_TARGET_HOST
    uint32_t current_ms = (uint32_t)bsp_time_now_ms();
    if (now_ms > current_ms) {
        bsp_time_test_advance_ms(now_ms - current_ms);
    }
#endif
    uint8_t robot_mode = current_robot_mode();
    uint8_t entered_arm_mode =
        (arm_work_mode(robot_mode) && !arm_work_mode(s_last_robot_mode)) ? 1U : 0U;
    uint8_t switched_arm_work_mode =
        (arm_work_mode(robot_mode) && arm_work_mode(s_last_robot_mode) &&
         robot_mode != s_last_robot_mode) ? 1U : 0U;
    uint8_t exited_arm_work_mode =
        (!arm_work_mode(robot_mode) && arm_work_mode(s_last_robot_mode)) ? 1U : 0U;
    s_last_robot_mode = robot_mode;
    uint8_t rear_place_mode =
        (robot_mode == PROTO_ROBOT_MODE_REAR_PLACE) ? 1U : 0U;
    (void)arm_control_set_rear_place_avoidance(
        rear_place_mode);
    if (switched_arm_work_mode || exited_arm_work_mode) {
        uint8_t terminal_result = task_comm_watchdog_active() ?
                                  PROTO_COMMAND_RESULT_WATCHDOG :
                                  task_safety_estop_active() ?
                                  PROTO_COMMAND_RESULT_ESTOP :
                                  PROTO_COMMAND_RESULT_WRONG_MODE;
        terminate_arm_commands_for_mode_edge(terminal_result);
    }
    if (entered_arm_mode || switched_arm_work_mode || exited_arm_work_mode) {
        reset_host_arm_protocol_state();
    }
    Arm_Serial_Protocol_SetRearPlaceMode(rear_place_mode);

    if (task_safety_estop_active()) {
        consume_new_commands(0U);
        if (!s_estop_was_active) {
            (void)arm_control_set_motor_output_enabled(0U);
            (void)arm_control_set_enabled(0U);
            s_estop_was_active = 1U;
        }
        /* 急停时不进入 Arm_Control_Process，避免自动重新使能达妙。 */
        Pump_Control_Process();
        report_pump_progress();
        if (!s_active_target_status_terminal) {
            (void)task_comm_report_command_status(
                PROTO_FUNC_ARM_TARGET,
                s_active_target_command_seq,
                PROTO_COMMAND_STAGE_ERROR,
                task_comm_watchdog_active() ? PROTO_COMMAND_RESULT_WATCHDOG :
                                              PROTO_COMMAND_RESULT_ESTOP);
            s_active_target_status_terminal = 1U;
            s_stow_active = 0U;
        }
        update_debug_snapshot_if_due(now_ms);
        send_feedback_if_due(now_ms);
        return;
    }

    if (s_estop_was_active) {
        (void)arm_control_set_enabled(1U);
        (void)arm_control_set_motor_output_enabled(
            APP_ARM_MOTOR_OUTPUT_DEFAULT_ENABLE ? 1U : 0U);
        (void)arm_control_set_gravity_mode();
        s_estop_was_active = 0U;
#if APP_ARM_POWER_ON_FIXED_GRASP_TEST
        reset_fixed_hold(ARM_FIXED_PROFILE_FIRST_LIFT);
#elif APP_ARM_POWER_ON_HOST_READY_ENABLE
        reset_fixed_hold(ARM_FIXED_PROFILE_FIRST_LIFT);
#else
        reset_fixed_hold(arm_work_mode(robot_mode) ?
                         ARM_FIXED_PROFILE_FIRST_LIFT :
                         ARM_FIXED_PROFILE_PARK);
#endif
    }

    /*
     * Compile-time field mode and the volatile Live Expressions switch share
     * one exclusive path. No protocol handler can write the manual switch.
     */
    if (APP_ARM_FORCE_GRAVITY_ONLY || debug_arm_force_gravity_only) {
        process_gravity_only_override();
        update_debug_snapshot_if_due(now_ms);
        send_feedback_if_due(now_ms);
        return;
    }
    if (s_gravity_only_override_active) {
        leave_gravity_only_override();
    }

    if (arm_work_mode(robot_mode)) {
        (void)consume_stow_command_if_pending(robot_mode);
    }

#if APP_ARM_POWER_ON_FIXED_GRASP_TEST
    (void)entered_arm_mode;
    (void)robot_mode;
    process_power_on_fixed_grasp_test();
#else
    if (!s_stow_active && arm_work_mode(robot_mode) &&
        (entered_arm_mode || switched_arm_work_mode ||
         s_fixed_profile == ARM_FIXED_PROFILE_PARK)) {
        s_place_hold_active = 0U;
        reset_fixed_hold(ARM_FIXED_PROFILE_FIRST_LIFT);
    } else if (!APP_ARM_POWER_ON_HOST_READY_ENABLE &&
               !arm_work_mode(robot_mode) &&
               s_fixed_profile != ARM_FIXED_PROFILE_PARK) {
        s_place_hold_active = 0U;
        reset_fixed_hold(ARM_FIXED_PROFILE_PARK);
    }
#endif

#if APP_ARM_POWER_ON_FIXED_GRASP_TEST
    if (!power_on_fixed_grasp_delay_done(now_ms)) {
        consume_new_commands(0U);
        Arm_Control_Process();
        Pump_Control_Process();
        update_debug_snapshot_if_due(now_ms);
        send_feedback_if_due(now_ms);
        return;
    }
#endif

    /* 固定姿态用于等待箱子；新的 GRASP 目标可在任意启动阶段立即抢占。 */
    uint8_t consume_commands = 1U;
#if !APP_ARM_POWER_ON_FIXED_GRASP_TEST
    if (debug_arm_fixed_state == ARM_FIXED_HOLDING &&
        arm_work_mode(robot_mode) &&
        host_has_new_valid_grasp_target()) {
        /* 等待姿态到位后，新的 GRASP 才接管控制。 */
        release_fixed_pose_to_host();
    }
#endif
    uint8_t command_allowed =
        (debug_arm_fixed_state == ARM_FIXED_RELEASED_TO_HOST) ?
        arm_usb_commands_allowed(robot_mode) : 0U;
    if (debug_arm_fixed_state != ARM_FIXED_RELEASED_TO_HOST &&
        arm_work_mode(robot_mode)) {
        /* ARM 等待目标期间保留可能先到达的泵命令。 */
        consume_commands = 0U;
    }
    if (command_allowed || consume_commands) {
        consume_new_commands(command_allowed);
    }

    /* 先应用本拍新目标，避免释放内部待机姿态后多输出一拍重力补偿。 */
    Arm_Serial_Protocol_Process();
    Arm_Control_Process();
    Pump_Control_Process();
    /* 保留运动完成后 PLACE/关泵的同拍处理语义。 */
    Arm_Serial_Protocol_Process();
    report_target_progress();
    report_pump_progress();
    /*
     * A completed PLACE cycle has already switched the payload model to
     * no-box gravity compensation and turned the pump off. Start the same
     * fixed waiting sequence used at power-up: move J2/J3 immediately after
     * feedback is ready while J1 remains freely movable.
     */
    uint32_t place_cycle_sequence =
        Arm_Serial_Protocol_PlaceCycleSequence();
    if (place_cycle_sequence != s_last_place_cycle_sequence) {
        s_last_place_cycle_sequence = place_cycle_sequence;
        s_place_hold_active = 1U;
        s_place_hold_start_ms = now_ms;
        debug_arm_fixed_state = ARM_FIXED_WAIT_FEEDBACK;
        (void)arm_control_set_j1_free_mode(0U);
    }
    if (s_place_hold_active) {
        if ((now_ms - s_place_hold_start_ms) >=
            APP_ARM_PLACE_HOLD_AFTER_PUMP_OFF_MS) {
            s_place_hold_active = 0U;
#if APP_ARM_POWER_ON_FIXED_GRASP_TEST
            reset_fixed_hold(ARM_FIXED_PROFILE_FIRST_LIFT);
#else
            reset_fixed_hold(ARM_FIXED_PROFILE_SECOND_SAFE_LIFT);
#endif
        }
#if APP_ARM_POWER_ON_FIXED_GRASP_TEST
    } else if (debug_arm_fixed_state == ARM_FIXED_MOVING ||
               debug_arm_fixed_state == ARM_FIXED_HOLDING ||
               debug_arm_fixed_state == ARM_FIXED_WAIT_FEEDBACK ||
               debug_arm_fixed_state == ARM_FIXED_ERROR) {
        process_fixed_hold(now_ms);
#else
    } else if (APP_ARM_POWER_ON_HOST_READY_ENABLE ||
               arm_work_mode(robot_mode) ||
               s_fixed_profile == ARM_FIXED_PROFILE_PARK ||
               debug_arm_fixed_state == ARM_FIXED_MOVING ||
               debug_arm_fixed_state == ARM_FIXED_HOLDING ||
               debug_arm_fixed_state == ARM_FIXED_WAIT_FEEDBACK ||
               debug_arm_fixed_state == ARM_FIXED_ERROR) {
        process_fixed_hold(now_ms);
#endif
    }
    report_stow_progress();
    update_debug_snapshot_if_due(now_ms);
    send_feedback_if_due(now_ms);
}

void task_arm_entry(void* arg) {
    (void)arg;
    LOGI("task_arm started");
#if APP_TARGET_MCU
    uint32_t last_ms = (uint32_t)bsp_time_now_ms();
    for (;;) {
        uint32_t now_ms = (uint32_t)bsp_time_now_ms();
        float dt_s = (float)(now_ms - last_ms) * 0.001f;
        last_ms = now_ms;
        task_arm_step_for_test(dt_s, now_ms);
        osDelay(1);
    }
#endif
}
