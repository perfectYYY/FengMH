/*
 * task_comm.c — 解析 USB CDC 字节 → 帧 → FuncID 分发 + 上行帧发送
 *
 * 下行：USB CDC RX → proto_frame 解析 → proto_dispatch 分发
 * 上行：定时构造 0x80(整机状态) / 0x81(电机状态) / 0x89(系统状态) /
 *       0x8A(里程计) / 0x8B(四轮实际转速) / 0x8C(底盘诊断) 帧。
 */
#include "task_comm.h"
#include "log.h"
#include "config.h"
#include "bsp_usb_cdc.h"
#include "bsp_time.h"
#include "proto_frame.h"
#include "proto_dispatch.h"
#include "proto_defs.h"
#include "motor_registry.h"
#include "motor_m3508.h"
#include "pump_control.h"
#include "task_chassis.h"
#include "task_safety.h"
#include "err.h"

#if APP_TARGET_MCU
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#endif

#include <string.h>
#include <math.h>

static const char* TAG = "COMM";

static proto_frame_parser_t  s_parser;
static proto_dispatcher_t    s_disp;
static task_comm_chassis_cmd_t s_chassis;
static task_comm_arm_target_t s_arm_target;
static task_comm_arm_pump_t   s_arm_pump;
static task_comm_mode_cmd_t   s_mode_cmd;
static volatile uint32_t     s_last_rx_ms = 0;
static uint32_t s_boot_id;
static volatile uint8_t s_actual_mode = PROTO_ROBOT_MODE_IDLE;
static volatile uint8_t s_comm_watchdog_active;
static uint8_t s_legacy_target_valid;

typedef struct {
    uint8_t valid;
    uint8_t command_func;
    uint8_t stage;
    uint8_t result;
    uint32_t command_seq;
} command_status_cache_t;

typedef struct {
    uint8_t valid;
    uint32_t command_seq;
} command_sequence_tracker_t;

static command_status_cache_t s_target_status;
static command_status_cache_t s_pump_status;
static command_status_cache_t s_mode_status;
static command_sequence_tracker_t s_target_sequence;
static command_sequence_tracker_t s_pump_sequence;
static command_sequence_tracker_t s_mode_sequence;

volatile uint8_t  debug_comm_arm_target_raw[sizeof(payload_arm_target_t)];
volatile uint8_t  debug_comm_arm_target_type;
volatile float    debug_comm_arm_target_x_m;
volatile float    debug_comm_arm_target_y_m;
volatile float    debug_comm_arm_target_z_m;
volatile uint32_t debug_comm_arm_target_accept_count;
volatile uint32_t debug_comm_arm_target_reject_count;

/* 上行帧发送周期控制 */
static uint32_t s_last_state_tx_ms  = 0;  /* 0x80 上次发送时间 */
static uint32_t s_last_motor_tx_ms  = 0;  /* 0x81 上次发送时间 */
static uint32_t s_last_system_tx_ms = 0;  /* 0x89 上次发送时间 */
static uint32_t s_last_odom_tx_ms   = 0;  /* 0x8A 上次发送时间 */
static uint32_t s_last_wheel_tx_ms  = 0;  /* 0x8B 上次发送时间 */
static uint32_t s_last_diag_tx_ms   = 0;  /* 0x8C 上次发送时间 */
#if APP_TARGET_MCU
static const uint32_t STATE_TX_INTERVAL_MS  = 100;  /* 10Hz */
static const uint32_t MOTOR_TX_INTERVAL_MS  = 200;  /* 5Hz, motor states are round-robin */
static const uint32_t WHEEL_TX_INTERVAL_MS  = 40;   /* 25Hz, FL/FR/RL/RR synchronized */
static const uint32_t DIAG_TX_INTERVAL_MS   = 40;   /* 25Hz */
static const uint32_t ODOM_TX_INTERVAL_MS   = 20;   /* 50Hz */
#endif

static uint32_t comm_critical_enter(void) {
#if APP_TARGET_MCU
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
#else
    return 0U;
#endif
}

static void comm_critical_exit(uint32_t state) {
#if APP_TARGET_MCU
    if (state == 0U) __enable_irq();
#else
    (void)state;
#endif
}

static uint32_t generate_boot_id(void) {
#if APP_TARGET_MCU
    uint32_t uid_mix = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
    uint32_t timing_mix = DWT->CYCCNT ^ (uint32_t)bsp_time_now_ms();
    uint32_t id = uid_mix ^ timing_mix ^ 0x4753494EU;
#else
    static uint32_t host_boot_generation;
    uint32_t id = 0x484F5354U ^ (++host_boot_generation * 0x9E3779B9U);
#endif
    return id ? id : 1U;
}

static command_status_cache_t* status_cache_for(uint8_t command_func) {
    if (command_func == PROTO_FUNC_ARM_TARGET) return &s_target_status;
    if (command_func == PROTO_FUNC_ARM_PUMP) return &s_pump_status;
    if (command_func == PROTO_FUNC_MODE_CMD) return &s_mode_status;
    return NULL;
}

static command_sequence_tracker_t* sequence_tracker_for(uint8_t command_func) {
    if (command_func == PROTO_FUNC_ARM_TARGET) return &s_target_sequence;
    if (command_func == PROTO_FUNC_ARM_PUMP) return &s_pump_sequence;
    if (command_func == PROTO_FUNC_MODE_CMD) return &s_mode_sequence;
    return NULL;
}

static uint8_t sequence_is_newer(uint32_t candidate, uint32_t current) {
    return ((int32_t)(candidate - current) > 0) ? 1U : 0U;
}

static void mark_valid_rx(void) {
    s_last_rx_ms = (uint32_t)bsp_time_now_ms();
}

#if APP_CHASSIS_ENABLE
static void cache_chassis_base_command(const uint8_t* p) {
    payload_chassis_cmd_t cmd;
    memcpy(&cmd, p, sizeof(cmd));
    s_chassis.vx = cmd.vx;
    s_chassis.vy = cmd.vy;
    s_chassis.wz = cmd.wz;
}

static void cache_chassis_steering_extension(const uint8_t* p, uint8_t len) {
    if (len >= PROTO_CHASSIS_CMD_EXT_LEN) {
        const uint8_t* ext = p + sizeof(payload_chassis_cmd_t);
        memcpy(&s_chassis.target_yaw, ext, sizeof(float));
        uint8_t mode = ext[sizeof(float)];
        s_chassis.steer_mode = (mode <= PROTO_STEER_MODE_AUTO_HOLD)
                             ? mode : PROTO_STEER_MODE_OFF;
    } else {
        s_chassis.target_yaw = 0.0f;
        s_chassis.steer_mode = PROTO_STEER_MODE_OFF;
    }
}

static void mark_chassis_command_rx(void) {
    s_chassis.seq++;
    mark_valid_rx();
}
#endif

static int send_proto_payload(uint8_t func_id, const void* payload, uint8_t len);

static int send_command_status(uint8_t command_func,
                               uint32_t command_seq,
                               uint8_t stage,
                               uint8_t result) {
    payload_command_status_t status = {
        .boot_id = s_boot_id,
        .command_seq = command_seq,
        .command_func = command_func,
        .stage = stage,
        .result = result,
        .actual_mode = s_actual_mode,
    };
    return send_proto_payload(PROTO_FUNC_COMMAND_STATUS,
                              &status,
                              (uint8_t)sizeof(status));
}

uint32_t task_comm_boot_id(void) { return s_boot_id; }
uint8_t task_comm_actual_mode(void) { return s_actual_mode; }
uint8_t task_comm_watchdog_active(void) { return s_comm_watchdog_active; }

int task_comm_report_command_status(uint8_t command_func,
                                    uint32_t command_seq,
                                    uint8_t stage,
                                    uint8_t result) {
    command_status_cache_t* cache = status_cache_for(command_func);
    command_sequence_tracker_t* tracker = sequence_tracker_for(command_func);
    if (!cache || stage > PROTO_COMMAND_STAGE_ERROR ||
        result > PROTO_COMMAND_RESULT_STALE_SEQUENCE) {
        return APP_ERR_INVALID_ARG;
    }

    uint32_t critical = comm_critical_enter();
    if (command_seq != 0U &&
        (!tracker || !tracker->valid || tracker->command_seq != command_seq)) {
        comm_critical_exit(critical);
        return APP_ERR_BUSY;
    }
    cache->valid = 1U;
    cache->command_func = command_func;
    cache->command_seq = command_seq;
    cache->stage = stage;
    cache->result = result;
    comm_critical_exit(critical);
    return send_command_status(command_func, command_seq, stage, result);
}

static int resend_cached_status(uint8_t command_func, uint32_t command_seq) {
    command_status_cache_t* cache = status_cache_for(command_func);
    if (!cache) return 0;

    command_status_cache_t snapshot;
    uint32_t critical = comm_critical_enter();
    snapshot = *cache;
    comm_critical_exit(critical);
    if (!snapshot.valid || snapshot.command_seq != command_seq) return 0;

    (void)send_command_status(command_func,
                              command_seq,
                              snapshot.stage,
                              snapshot.result);
    return 1;
}

/*
 * Returns 0 for a new sequence, 1 for a duplicate, and -1 for a stale or
 * out-of-order sequence. A sequence is reserved before payload validation so
 * retrying the same id with different bytes can never execute a second time.
 */
static void reset_v2_session_locked(uint32_t idle_command_seq) {
    memset(&s_target_status, 0, sizeof(s_target_status));
    memset(&s_pump_status, 0, sizeof(s_pump_status));
    memset(&s_mode_status, 0, sizeof(s_mode_status));
    memset(&s_target_sequence, 0, sizeof(s_target_sequence));
    memset(&s_pump_sequence, 0, sizeof(s_pump_sequence));
    memset(&s_mode_sequence, 0, sizeof(s_mode_sequence));
    s_mode_sequence.valid = 1U;
    s_mode_sequence.command_seq = idle_command_seq;
    s_legacy_target_valid = 0U;
}

static int prepare_v2_sequence(uint8_t command_func,
                               uint32_t command_seq,
                               uint8_t allow_stale_idle_reset) {
    command_sequence_tracker_t* tracker = sequence_tracker_for(command_func);
    if (!tracker || command_seq == 0U) return -1;

    uint32_t critical = comm_critical_enter();
    if (!tracker->valid) {
        tracker->valid = 1U;
        tracker->command_seq = command_seq;
        comm_critical_exit(critical);
        return 0;
    }

    const uint32_t current = tracker->command_seq;
    if (command_seq == current) {
        comm_critical_exit(critical);
        mark_valid_rx();
        if (!resend_cached_status(command_func, command_seq)) {
            (void)send_command_status(command_func,
                                      command_seq,
                                      PROTO_COMMAND_STAGE_REJECTED,
                                      PROTO_COMMAND_RESULT_BUSY);
        }
        return 1;
    }

    if (!sequence_is_newer(command_seq, current)) {
        if (allow_stale_idle_reset) {
            /* A restarted host begins at sequence 1 while this MCU keeps its
             * old watermarks. An explicit IDLE is the only safe command that
             * may open a new V2 session without rebooting the controller. */
            reset_v2_session_locked(command_seq);
            comm_critical_exit(critical);
            return 0;
        }
        comm_critical_exit(critical);
        (void)send_command_status(command_func,
                                  command_seq,
                                  PROTO_COMMAND_STAGE_REJECTED,
                                  PROTO_COMMAND_RESULT_STALE_SEQUENCE);
        return -1;
    }

    tracker->command_seq = command_seq;
    comm_critical_exit(critical);
    return 0;
}

static void report_rejected(uint8_t command_func,
                            uint32_t command_seq,
                            uint8_t result) {
    (void)task_comm_report_command_status(command_func,
                                          command_seq,
                                          PROTO_COMMAND_STAGE_REJECTED,
                                          result);
}

#if APP_CHASSIS_ENABLE
static void request_mode_safe_stand(void) {
    (void)task_chassis_set_wheel_test(0U, 0U, NULL, (uint32_t)bsp_time_now_ms());
    uint32_t critical = comm_critical_enter();
    s_chassis.vx = 0.0f;
    s_chassis.vy = 0.0f;
    s_chassis.wz = 0.0f;
    s_chassis.target_yaw = 0.0f;
    s_chassis.steer_mode = 0U;
    s_chassis.seq++;
    comm_critical_exit(critical);

    task_chassis_set_mode(CHASSIS_MODE_STANDALONE);
    (void)task_chassis_start_stand(0.3f);
}
#endif

static int handle_wheel_test(const uint8_t* p, uint8_t len) {
#if !APP_CHASSIS_ENABLE
    (void)p;
    (void)len;
    return APP_ERR_UNSUPPORTED;
#else
    if (len != sizeof(payload_wheel_test_t)) return APP_ERR_INVALID_ARG;

    payload_wheel_test_t cmd;
    float wheel_rads[GAIT_LEG_NUM];
    memcpy(&cmd, p, sizeof(cmd));
    memcpy(wheel_rads, cmd.wheel_rads, sizeof(wheel_rads));
    int ret = task_chassis_set_wheel_test(cmd.enable,
                                          cmd.wheel_mask,
                                          wheel_rads,
                                          (uint32_t)bsp_time_now_ms());
    if (ret != APP_OK) return ret;

    s_chassis.vx = 0.0f;
    s_chassis.vy = 0.0f;
    s_chassis.wz = 0.0f;
    s_chassis.target_yaw = 0.0f;
    s_chassis.steer_mode = 0U;
    s_chassis.seq++;
    return APP_OK;
#endif
}

static int handle_chassis(const uint8_t* p, uint8_t len) {
#if !APP_CHASSIS_ENABLE
    (void)p;
    (void)len;
    return APP_ERR_UNSUPPORTED;
#else
    /*
     * 兼容两种帧长:
     *   12 bytes: 旧协议 {vx, vy, wz}
     *   17 bytes: 新协议 {vx, vy, wz, target_yaw, steer_mode}
     */
    if (len != PROTO_CHASSIS_CMD_LEGACY_LEN &&
        len != PROTO_CHASSIS_CMD_EXT_LEN) {
        return APP_ERR_INVALID_ARG;
    }

    cache_chassis_base_command(p);
    cache_chassis_steering_extension(p, len);
    mark_chassis_command_rx();
    return 0;
#endif
}

#if APP_CHASSIS_ENABLE
static void gait_params_from_payload(const payload_gait_cmd_t* in, gait_params_t* out) {
    memset(out, 0, sizeof(*out));
    out->body_height_m = in->body_height_m;
    out->step_length_m = in->step_length_m;
    out->step_height_m = in->step_height_m;
    out->period_s = in->period_s;
    out->duty = in->duty;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        out->phase_offset[i] = in->phase_offset[i];
    }
    out->touchdown_thresh = in->touchdown_thresh;
}
#endif

static int handle_gait(const uint8_t* p, uint8_t len) {
#if !APP_CHASSIS_ENABLE
    (void)p;
    (void)len;
    return APP_ERR_UNSUPPORTED;
#else
    if (len != sizeof(payload_gait_cmd_t)) return -1;

    payload_gait_cmd_t cmd;
    gait_params_t params;
    int ret = APP_ERR_INVALID_ARG;

    memcpy(&cmd, p, sizeof(cmd));
    gait_params_from_payload(&cmd, &params);

    switch (cmd.action) {
        case PROTO_GAIT_ACTION_STAND:
            task_chassis_set_mode(CHASSIS_MODE_STANDALONE);
            ret = task_chassis_start_stand(cmd.blend_dur_s);
            break;
        case PROTO_GAIT_ACTION_TROT:
            task_chassis_set_mode(CHASSIS_MODE_STANDALONE);
            ret = task_chassis_start_trot(&params, cmd.blend_dur_s);
            break;
        case PROTO_GAIT_ACTION_SET_TROT_PARAMS:
            ret = task_chassis_set_trot_params(&params);
            break;
        case PROTO_GAIT_ACTION_WALK:
            task_chassis_set_mode(CHASSIS_MODE_STANDALONE);
            ret = task_chassis_start_walk(&params, cmd.blend_dur_s);
            break;
        case PROTO_GAIT_ACTION_SET_WALK_PARAMS:
            ret = task_chassis_set_walk_params(&params);
            break;
        default:
            ret = APP_ERR_INVALID_ARG;
            break;
    }

    if (ret == APP_OK) {
        mark_valid_rx();
        return 0;
    }
    return ret;
#endif
}

static int handle_mit_cmd(const uint8_t* p, uint8_t len) {
    if (len != sizeof(payload_mit_cmd_t)) return -1;

    payload_mit_cmd_t cmd;
    memcpy(&cmd, p, sizeof(cmd));

    /* 机械臂只有task_arm可以写电机；调试统一使用纯重补测试态。 */
    if (cmd.motor_id >= MOTOR_ID_ARM_J1 &&
        cmd.motor_id <= MOTOR_ID_ARM_J6) {
        return APP_ERR_UNSUPPORTED;
    }

    motor_dev_t* dev = motor_get((motor_logical_id_t)cmd.motor_id);
    if (!dev || !dev->ops || !dev->ops->set_position) return -1;

    int ret = dev->ops->set_position(dev,
                                     cmd.pos_rad, cmd.vel_rads,
                                     cmd.kp, cmd.kd, cmd.tau_ff_nm);
    if (ret == APP_OK) {
        mark_valid_rx();
        return 0;
    }
    return ret;
}

static int handle_arm_target(const uint8_t* p, uint8_t len) {
    payload_arm_target_t cmd;
    uint32_t command_seq = 0U;
    uint8_t sequenced = 0U;
    if (len == sizeof(payload_arm_target_t)) {
        memcpy(&cmd, p, sizeof(cmd));
    } else if (len == sizeof(payload_arm_target_v2_t)) {
        payload_arm_target_v2_t v2;
        memcpy(&v2, p, sizeof(v2));
        cmd = v2.target;
        command_seq = v2.command_seq;
        sequenced = 1U;
        if (command_seq == 0U) {
            report_rejected(PROTO_FUNC_ARM_TARGET,
                            command_seq,
                            PROTO_COMMAND_RESULT_BAD_VALUE);
            return APP_ERR_INVALID_ARG;
        }
        int sequence_state = prepare_v2_sequence(PROTO_FUNC_ARM_TARGET,
                                                 command_seq,
                                                 0U);
        if (sequence_state != 0) {
            return (sequence_state > 0) ? APP_OK : APP_ERR_BUSY;
        }
    } else {
        report_rejected(PROTO_FUNC_ARM_TARGET, 0U, PROTO_COMMAND_RESULT_BAD_LENGTH);
        return APP_ERR_INVALID_ARG;
    }

    for (uint32_t i = 0U; i < sizeof(payload_arm_target_t); i++) {
        debug_comm_arm_target_raw[i] = p[i];
    }
    debug_comm_arm_target_type = cmd.target_type;
    debug_comm_arm_target_x_m = cmd.x_m;
    debug_comm_arm_target_y_m = cmd.y_m;
    debug_comm_arm_target_z_m = cmd.z_m;

    if (cmd.target_type != PROTO_ARM_TARGET_GRASP &&
        cmd.target_type != PROTO_ARM_TARGET_PLACE &&
        cmd.target_type != PROTO_ARM_TARGET_STOW) {
        debug_comm_arm_target_reject_count++;
        report_rejected(PROTO_FUNC_ARM_TARGET,
                        command_seq,
                        PROTO_COMMAND_RESULT_BAD_VALUE);
        return APP_ERR_INVALID_ARG;
    }

    if (s_actual_mode != PROTO_ROBOT_MODE_ARM &&
        s_actual_mode != PROTO_ROBOT_MODE_REAR_PLACE) {
        debug_comm_arm_target_reject_count++;
        report_rejected(PROTO_FUNC_ARM_TARGET,
                        command_seq,
                        PROTO_COMMAND_RESULT_WRONG_MODE);
        return APP_ERR_INVALID_ARG;
    }
    if (cmd.target_type == PROTO_ARM_TARGET_STOW &&
        s_actual_mode != PROTO_ROBOT_MODE_ARM) {
        debug_comm_arm_target_reject_count++;
        report_rejected(PROTO_FUNC_ARM_TARGET,
                        command_seq,
                        PROTO_COMMAND_RESULT_WRONG_MODE);
        return APP_ERR_INVALID_ARG;
    }

    /* 机械臂两连杆总长 0.65 m；在通信入口拦截错字节序/错单位的有限异常值。 */
    const float distance_sq = cmd.x_m * cmd.x_m +
                              cmd.y_m * cmd.y_m +
                              cmd.z_m * cmd.z_m;
    const float min_reach_m = 0.045f;
    const float max_reach_m = 0.655f;
    if (cmd.target_type != PROTO_ARM_TARGET_STOW &&
        (!isfinite(cmd.x_m) || !isfinite(cmd.y_m) || !isfinite(cmd.z_m) ||
        !isfinite(distance_sq) ||
        distance_sq < min_reach_m * min_reach_m ||
        distance_sq > max_reach_m * max_reach_m)) {
        debug_comm_arm_target_reject_count++;
        report_rejected(PROTO_FUNC_ARM_TARGET,
                        command_seq,
                        PROTO_COMMAND_RESULT_INVALID_TARGET);
        return APP_ERR_INVALID_ARG;
    }

    if (!sequenced && s_legacy_target_valid &&
        s_arm_target.target_type == cmd.target_type &&
        fabsf(s_arm_target.x_m - cmd.x_m) <= 1e-6f &&
        fabsf(s_arm_target.y_m - cmd.y_m) <= 1e-6f &&
        fabsf(s_arm_target.z_m - cmd.z_m) <= 1e-6f) {
        (void)resend_cached_status(PROTO_FUNC_ARM_TARGET, 0U);
        mark_valid_rx();
        return APP_OK;
    }

    uint32_t critical = comm_critical_enter();
    s_arm_target.target_type = cmd.target_type;
    s_arm_target.x_m = cmd.x_m;
    s_arm_target.y_m = cmd.y_m;
    s_arm_target.z_m = cmd.z_m;
    s_arm_target.command_seq = command_seq;
    s_arm_target.sequenced = sequenced;
    s_arm_target.seq++;
    s_legacy_target_valid = sequenced ? 0U : 1U;
    comm_critical_exit(critical);
    debug_comm_arm_target_accept_count++;
    mark_valid_rx();
    (void)task_comm_report_command_status(PROTO_FUNC_ARM_TARGET,
                                          command_seq,
                                          PROTO_COMMAND_STAGE_RECEIVED,
                                          PROTO_COMMAND_RESULT_OK);
    return 0;
}

static int handle_arm_pump(const uint8_t* p, uint8_t len) {
    payload_arm_pump_t cmd;
    uint32_t command_seq = 0U;
    uint8_t sequenced = 0U;
    if (len == sizeof(payload_arm_pump_t)) {
        memcpy(&cmd, p, sizeof(cmd));
    } else if (len == sizeof(payload_arm_pump_v2_t)) {
        payload_arm_pump_v2_t v2;
        memcpy(&v2, p, sizeof(v2));
        cmd.pump_on = v2.pump_on;
        command_seq = v2.command_seq;
        sequenced = 1U;
        if (command_seq == 0U) {
            report_rejected(PROTO_FUNC_ARM_PUMP,
                            command_seq,
                            PROTO_COMMAND_RESULT_BAD_VALUE);
            return APP_ERR_INVALID_ARG;
        }
        int sequence_state = prepare_v2_sequence(PROTO_FUNC_ARM_PUMP,
                                                 command_seq,
                                                 0U);
        if (sequence_state != 0) {
            return (sequence_state > 0) ? APP_OK : APP_ERR_BUSY;
        }
    } else {
        report_rejected(PROTO_FUNC_ARM_PUMP, 0U, PROTO_COMMAND_RESULT_BAD_LENGTH);
        return APP_ERR_INVALID_ARG;
    }
    if (cmd.pump_on > 1U) {
        report_rejected(PROTO_FUNC_ARM_PUMP,
                        command_seq,
                        PROTO_COMMAND_RESULT_BAD_VALUE);
        return APP_ERR_INVALID_ARG;
    }
    if (cmd.pump_on && s_actual_mode != PROTO_ROBOT_MODE_ARM &&
        s_actual_mode != PROTO_ROBOT_MODE_REAR_PLACE) {
        report_rejected(PROTO_FUNC_ARM_PUMP,
                        command_seq,
                        PROTO_COMMAND_RESULT_WRONG_MODE);
        return APP_ERR_INVALID_ARG;
    }

    uint32_t critical = comm_critical_enter();
    s_arm_pump.pump_on = cmd.pump_on;
    s_arm_pump.command_seq = command_seq;
    s_arm_pump.sequenced = sequenced;
    s_arm_pump.seq++;
    comm_critical_exit(critical);
    mark_valid_rx();
    if (!cmd.pump_on) {
        /* Pump-off is the recovery primitive used from ERROR/ESTOP. Execute it
         * at ingress so completion never depends on the arm task running. The
         * cached command is still consumed there to finish normal PLACE state. */
        Pump_Control_Set(0U);
        (void)task_comm_report_command_status(PROTO_FUNC_ARM_PUMP,
                                              command_seq,
                                              PROTO_COMMAND_STAGE_COMPLETED,
                                              PROTO_COMMAND_RESULT_OK);
    } else {
        (void)task_comm_report_command_status(PROTO_FUNC_ARM_PUMP,
                                              command_seq,
                                              PROTO_COMMAND_STAGE_RECEIVED,
                                              PROTO_COMMAND_RESULT_OK);
    }
    return 0;
}

static int handle_arm_aux_gpio(const uint8_t* p, uint8_t len) {
    if (len != sizeof(payload_arm_aux_gpio_t)) return -1;

    payload_arm_aux_gpio_t cmd;
    memcpy(&cmd, p, sizeof(cmd));
    if (cmd.on > 1U || cmd.channel > PROTO_ARM_AUX_GPIO_PA9) {
        return APP_ERR_INVALID_ARG;
    }

    switch (cmd.channel) {
        case PROTO_ARM_AUX_GPIO_PC8:
            Pump_Control_SetPC8(cmd.on);
            break;
        case PROTO_ARM_AUX_GPIO_PC9:
            Pump_Control_SetPC9(cmd.on);
            break;
        case PROTO_ARM_AUX_GPIO_PA8:
            Pump_Control_SetPA8(cmd.on);
            break;
        case PROTO_ARM_AUX_GPIO_PA9:
            Pump_Control_SetPA9(cmd.on);
            break;
        default:
            return APP_ERR_INVALID_ARG;
    }

    mark_valid_rx();
    return 0;
}

static int handle_odom_reset(const uint8_t* p, uint8_t len) {
#if !APP_CHASSIS_ENABLE
    (void)p;
    (void)len;
    return APP_ERR_UNSUPPORTED;
#else
    payload_odom_reset_t cmd;
    if (len != sizeof(cmd)) return APP_ERR_INVALID_ARG;
    memcpy(&cmd, p, sizeof(cmd));
    int ret = task_chassis_reset_odometry(cmd.x_m, cmd.y_m, cmd.yaw_rad);
    if (ret == APP_OK) mark_valid_rx();
    return ret;
#endif
}

static int handle_mode_cmd(const uint8_t* p, uint8_t len) {
    payload_mode_cmd_t cmd;
    uint32_t command_seq = 0U;
    uint8_t sequenced = 0U;
    if (len == sizeof(payload_mode_cmd_t)) {
        memcpy(&cmd, p, sizeof(cmd));
    } else if (len == sizeof(payload_mode_cmd_v2_t)) {
        payload_mode_cmd_v2_t v2;
        memcpy(&v2, p, sizeof(v2));
        cmd.mode = v2.mode;
        command_seq = v2.command_seq;
        sequenced = 1U;
        if (command_seq == 0U) {
            report_rejected(PROTO_FUNC_MODE_CMD,
                            command_seq,
                            PROTO_COMMAND_RESULT_BAD_VALUE);
            return APP_ERR_INVALID_ARG;
        }
        int sequence_state = prepare_v2_sequence(
            PROTO_FUNC_MODE_CMD,
            command_seq,
            (cmd.mode == PROTO_ROBOT_MODE_IDLE) ? 1U : 0U);
        if (sequence_state != 0) {
            return (sequence_state > 0) ? APP_OK : APP_ERR_BUSY;
        }
    } else {
        report_rejected(PROTO_FUNC_MODE_CMD, 0U, PROTO_COMMAND_RESULT_BAD_LENGTH);
        return APP_ERR_INVALID_ARG;
    }
    if (cmd.mode > PROTO_ROBOT_MODE_MAX) {
        report_rejected(PROTO_FUNC_MODE_CMD,
                        command_seq,
                        PROTO_COMMAND_RESULT_BAD_VALUE);
        return APP_ERR_INVALID_ARG;
    }

    if (s_comm_watchdog_active && cmd.mode != PROTO_ROBOT_MODE_IDLE) {
        report_rejected(PROTO_FUNC_MODE_CMD,
                        command_seq,
                        PROTO_COMMAND_RESULT_WATCHDOG);
        return APP_ERR_BUSY;
    }

    uint32_t critical = comm_critical_enter();
    uint8_t previous_mode = s_actual_mode;
    s_mode_cmd.mode = cmd.mode;
    s_mode_cmd.command_seq = command_seq;
    s_mode_cmd.sequenced = sequenced;
    s_mode_cmd.seq++;
    s_actual_mode = cmd.mode;
    if (cmd.mode == PROTO_ROBOT_MODE_IDLE) s_comm_watchdog_active = 0U;
    if (previous_mode != cmd.mode) s_legacy_target_valid = 0U;
    comm_critical_exit(critical);

    const uint8_t lock_motion =
        (cmd.mode == PROTO_ROBOT_MODE_ESTOP ||
         cmd.mode == PROTO_ROBOT_MODE_ERROR) ? 1U : 0U;
    if (previous_mode != cmd.mode) Pump_Control_Set(0U);
    task_safety_estop_set(lock_motion != 0U);
#if APP_CHASSIS_ENABLE
    if (cmd.mode != PROTO_ROBOT_MODE_NAV) {
        request_mode_safe_stand();
    } else {
        task_chassis_set_mode(CHASSIS_MODE_AUTO);
    }
#endif
    mark_valid_rx();
    (void)task_comm_report_command_status(PROTO_FUNC_MODE_CMD,
                                          command_seq,
                                          PROTO_COMMAND_STAGE_COMPLETED,
                                          PROTO_COMMAND_RESULT_OK);
    return 0;
}

static const proto_entry_t s_tbl[] = {
    { PROTO_FUNC_CHASSIS_CMD, 0, handle_chassis, "chassis" },  /* expect_len=0: 由 handler 内自行校验 */
    { PROTO_FUNC_ARM_TARGET, 0, handle_arm_target, "arm_target" },
    { PROTO_FUNC_GAIT_CMD, sizeof(payload_gait_cmd_t), handle_gait, "gait" },
    { PROTO_FUNC_MIT_CMD, sizeof(payload_mit_cmd_t), handle_mit_cmd, "mit" },
    { PROTO_FUNC_ARM_PUMP, 0, handle_arm_pump, "arm_pump" },
    { PROTO_FUNC_MODE_CMD, 0, handle_mode_cmd, "mode" },
    { PROTO_FUNC_WHEEL_TEST, sizeof(payload_wheel_test_t), handle_wheel_test, "wheel_test" },
    { PROTO_FUNC_ARM_AUX_GPIO, sizeof(payload_arm_aux_gpio_t), handle_arm_aux_gpio, "arm_aux_gpio" },
    { PROTO_FUNC_ODOM_RESET, sizeof(payload_odom_reset_t), handle_odom_reset, "odom_reset" },
};

static void on_usb_rx(const uint8_t* d, uint32_t n, void* user) {
    (void)user;
    proto_frame_feed(&s_parser, d, n);
}

void task_comm_init(void) {
    memset(&s_chassis, 0, sizeof(s_chassis));
    memset(&s_arm_target, 0, sizeof(s_arm_target));
    memset(&s_arm_pump, 0, sizeof(s_arm_pump));
    memset(&s_mode_cmd, 0, sizeof(s_mode_cmd));
    memset(&s_target_status, 0, sizeof(s_target_status));
    memset(&s_pump_status, 0, sizeof(s_pump_status));
    memset(&s_mode_status, 0, sizeof(s_mode_status));
    memset(&s_target_sequence, 0, sizeof(s_target_sequence));
    memset(&s_pump_sequence, 0, sizeof(s_pump_sequence));
    memset(&s_mode_sequence, 0, sizeof(s_mode_sequence));
    memset((void*)debug_comm_arm_target_raw, 0, sizeof(debug_comm_arm_target_raw));
    debug_comm_arm_target_type = 0U;
    debug_comm_arm_target_x_m = 0.0f;
    debug_comm_arm_target_y_m = 0.0f;
    debug_comm_arm_target_z_m = 0.0f;
    debug_comm_arm_target_accept_count = 0U;
    debug_comm_arm_target_reject_count = 0U;
    s_last_rx_ms = 0;
    s_boot_id = generate_boot_id();
    s_actual_mode = PROTO_ROBOT_MODE_IDLE;
    s_comm_watchdog_active = 0U;
    s_legacy_target_valid = 0U;
    s_last_state_tx_ms = 0;
    s_last_motor_tx_ms = 0;
    s_last_system_tx_ms = 0;
    s_last_odom_tx_ms = 0;
    s_last_wheel_tx_ms = 0;
    s_last_diag_tx_ms = 0;
    proto_dispatcher_init(&s_disp, s_tbl, sizeof(s_tbl)/sizeof(s_tbl[0]));
    proto_frame_init(&s_parser, proto_dispatch_on_frame, &s_disp);
    bsp_usb_cdc_attach_rx(on_usb_rx, NULL);
    LOGI("comm init: %u handlers boot_id=0x%08lX",
         (unsigned)(sizeof(s_tbl)/sizeof(s_tbl[0])),
         (unsigned long)s_boot_id);
}

uint32_t task_comm_good_cnt(void)        { return s_parser.good_cnt; }
uint32_t task_comm_bad_cnt(void)         { return s_parser.bad_cnt; }
uint32_t task_comm_dispatch_hit(void)    { return s_disp.hit_cnt; }
uint32_t task_comm_dispatch_miss(void)   { return s_disp.miss_cnt; }
uint32_t task_comm_last_rx_ms(void)      { return s_last_rx_ms; }

void task_comm_get_chassis(task_comm_chassis_cmd_t* out) {
    if (!out) return;
    uint32_t critical = comm_critical_enter();
    *out = s_chassis;
    comm_critical_exit(critical);
}

void task_comm_get_arm_target(task_comm_arm_target_t* out) {
    if (!out) return;
    uint32_t critical = comm_critical_enter();
    *out = s_arm_target;
    comm_critical_exit(critical);
}

void task_comm_get_arm_pump(task_comm_arm_pump_t* out) {
    if (!out) return;
    uint32_t critical = comm_critical_enter();
    *out = s_arm_pump;
    comm_critical_exit(critical);
}

void task_comm_get_mode_cmd(task_comm_mode_cmd_t* out) {
    if (!out) return;
    uint32_t critical = comm_critical_enter();
    *out = s_mode_cmd;
    comm_critical_exit(critical);
}

void task_comm_watchdog_step(uint32_t now_ms) {
    if (s_comm_watchdog_active) return;
    if ((now_ms - s_last_rx_ms) <= APP_COMM_WATCHDOG_TIMEOUT_MS) return;

    uint32_t critical = comm_critical_enter();
    s_comm_watchdog_active = 1U;
    s_actual_mode = PROTO_ROBOT_MODE_ERROR;
    s_mode_cmd.mode = PROTO_ROBOT_MODE_ERROR;
    s_mode_cmd.seq++;
    uint32_t command_seq = s_mode_cmd.command_seq;
    comm_critical_exit(critical);

    /* Cargo-slot holding outputs are intentionally not touched here. */
    Pump_Control_Set(0U);
    task_safety_estop_set(true);
#if APP_CHASSIS_ENABLE
    request_mode_safe_stand();
#endif
    (void)task_comm_report_command_status(PROTO_FUNC_MODE_CMD,
                                          command_seq,
                                          PROTO_COMMAND_STAGE_ERROR,
                                          PROTO_COMMAND_RESULT_WATCHDOG);
    LOGE("host communication watchdog latched after %ums",
         (unsigned)APP_COMM_WATCHDOG_TIMEOUT_MS);
}

/* ─── 上行帧构造 ─── */

#pragma pack(push, 1)

/* 0x80 整机状态 payload */
typedef struct {
    uint8_t  mode;          /* chassis_mode_t */
    uint8_t  gait_active;   /* 0=stand 1=trot 2=walk 3=script */
    uint8_t  estop;         /* 0=normal 1=estop */
    uint8_t  reserved;
    float    vx_cmd;        /* 当前速度指令 */
    float    vy_cmd;
    float    wz_cmd;
    uint32_t uptime_ms;     /* 运行时间 */
} payload_state_t;

/* 0x81 单个电机状态 (8B) */
typedef struct {
    uint16_t id;            /* logical id */
    uint8_t  online;        /* 0/1 */
    uint8_t  type;          /* motor_type_t */
    float    angle_rad;
    float    velocity_rads;
} payload_motor_one_t;

#pragma pack(pop)

static payload_state_t build_state_payload(void) {
    payload_state_t p;
    memset(&p, 0, sizeof(p));
#if APP_CHASSIS_ENABLE
    p.mode        = (uint8_t)task_chassis_get_mode();
    p.gait_active = (uint8_t)task_chassis_get_gait_active();
#endif
    p.estop       = task_safety_estop_active() ? 1 : 0;
    p.vx_cmd      = s_chassis.vx;
    p.vy_cmd      = s_chassis.vy;
    p.wz_cmd      = s_chassis.wz;
    p.uptime_ms   = (uint32_t)bsp_time_now_ms();
    return p;
}

static payload_system_status_t build_system_status_payload(uint32_t now_ms) {
    uint8_t flags = 0U;
    command_status_cache_t target_status;
    uint32_t critical = comm_critical_enter();
    target_status = s_target_status;
    comm_critical_exit(critical);
    if (task_safety_estop_active()) flags |= PROTO_SAFETY_FLAG_ESTOP_ACTIVE;
    if (s_comm_watchdog_active) flags |= PROTO_SAFETY_FLAG_COMM_WATCHDOG;
    if (target_status.valid && target_status.stage == PROTO_COMMAND_STAGE_ERROR) {
        flags |= PROTO_SAFETY_FLAG_ARM_ERROR;
    }
    if ((now_ms - s_last_rx_ms) > APP_COMM_WATCHDOG_TIMEOUT_MS) {
        flags |= PROTO_SAFETY_FLAG_CHASSIS_OFFLINE;
    }
    if (debug_pa8_on) flags |= PROTO_SAFETY_FLAG_REAR_SLOT_A_HELD;
    if (debug_pc8_on || debug_pc9_on) flags |= PROTO_SAFETY_FLAG_REAR_SLOT_B_HELD;

    payload_system_status_t status = {
        .boot_id = s_boot_id,
        .uptime_ms = now_ms,
        .actual_mode = s_actual_mode,
        .safety_flags = flags,
        .main_pump_on = Pump_Control_IsEnabled() ? 1U : 0U,
        .reserved = 0U,
    };
    return status;
}

static int send_proto_payload(uint8_t func_id, const void* payload, uint8_t len) {
    uint8_t buf[80];
    int n = proto_frame_build(func_id, (const uint8_t*)payload, len, buf, sizeof(buf));
    if (n > 0) {
        app_err_t err = bsp_usb_cdc_send(buf, (uint32_t)n);
        if (err != APP_OK) return (int)err;
    }
    return n;
}

int task_comm_send_arm_feedback(const payload_arm_feedback_t* feedback) {
    if (!feedback) return APP_ERR_INVALID_ARG;
    return send_proto_payload(PROTO_FUNC_ARM_FEEDBACK,
                              feedback,
                              (uint8_t)sizeof(*feedback));
}

int task_comm_send_arm_motor_angles(const payload_arm_motor_angles_t* angles) {
    if (!angles) return APP_ERR_INVALID_ARG;
    return send_proto_payload(PROTO_FUNC_ARM_MOTOR_ANGLES,
                              angles,
                              (uint8_t)sizeof(*angles));
}

static float read_wheel_velocity(motor_logical_id_t id,
                                 uint8_t online_bit,
                                 uint8_t* online_mask) {
    motor_dev_t* d = motor_get(id);
    if (!d) return 0.0f;

    if (d->state.online) {
        *online_mask |= online_bit;
    }
    return d->state.velocity_rads;
}

int task_comm_send_wheel_feedback(void) {
    payload_wheel_state_t p;
    memset(&p, 0, sizeof(p));
    p.timestamp_ms = (uint32_t)bsp_time_now_ms();

    uint8_t online_mask = 0U;
    p.fl_velocity_rads = read_wheel_velocity(MOTOR_ID_FL_WHEEL, 0x01U,
                                             &online_mask);
    p.fr_velocity_rads = read_wheel_velocity(MOTOR_ID_FR_WHEEL, 0x02U,
                                             &online_mask);
    p.rl_velocity_rads = read_wheel_velocity(MOTOR_ID_RL_WHEEL, 0x04U,
                                             &online_mask);
    p.rr_velocity_rads = read_wheel_velocity(MOTOR_ID_RR_WHEEL, 0x08U,
                                             &online_mask);
    p.online_mask = online_mask;

    return send_proto_payload(PROTO_FUNC_WHEEL_STATE,
                              &p,
                              (uint8_t)sizeof(p));
}

static int16_t diag_float_to_i16(float value, float scale) {
    float scaled = value * scale;
    if (!isfinite(scaled)) return 0;
    if (scaled > 32767.0f) return 32767;
    if (scaled < -32768.0f) return -32768;
    return (int16_t)scaled;
}

int task_comm_send_chassis_diag(void) {
    static const motor_logical_id_t IDS[GAIT_LEG_NUM] = {
        MOTOR_ID_FL_WHEEL, MOTOR_ID_FR_WHEEL, MOTOR_ID_RL_WHEEL, MOTOR_ID_RR_WHEEL
    };
    payload_chassis_diag_t p;
    chassis_control_status_t status;
    memset(&p, 0, sizeof(p));
    memset(&status, 0, sizeof(status));
    task_chassis_get_control_status(&status);
    p.timestamp_ms = (uint32_t)bsp_time_now_ms();

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        m3508_wheel_diag_t d;
        memset(&d, 0, sizeof(d));
        (void)motor_m3508_get_wheel_diag(IDS[i], &d);
        p.target_mrad_s[i] = diag_float_to_i16(d.target_velocity_rads, 1000.0f);
        p.filtered_mrad_s[i] = diag_float_to_i16(d.filtered_velocity_rads, 1000.0f);
        p.cmd_current_raw[i] = d.cmd_current_raw;
        p.actual_current_raw[i] = d.actual_current_raw;
    }
    p.yaw_mrad = diag_float_to_i16(status.yaw_rad, 1000.0f);
    p.gyro_z_mrad_s = diag_float_to_i16(status.gyro_z_rad_s, 1000.0f);
    p.effective_wz_mrad_s = diag_float_to_i16(status.effective_wz_rad_s, 1000.0f);
    p.slip_residual_mrad_s = diag_float_to_i16(status.slip_residual_rad_s, 1000.0f);
    p.speed_scale_permille = (uint16_t)diag_float_to_i16(status.speed_scale, 1000.0f);
    p.flags = status.diagnostic_flags;
    return send_proto_payload(PROTO_FUNC_CHASSIS_DIAG, &p, (uint8_t)sizeof(p));
}

int task_comm_send_odometry(void) {
    chassis_odometry_state_t odom;
    payload_odometry_t p;
    memset(&odom, 0, sizeof(odom));
    memset(&p, 0, sizeof(p));
    task_chassis_get_odometry(&odom);
    p.timestamp_ms = (uint32_t)bsp_time_now_ms();
    p.x_m = odom.x_m;
    p.y_m = odom.y_m;
    p.yaw_rad = odom.yaw_rad;
    p.vx_m_s = odom.vx_m_s;
    p.yaw_rate_rad_s = odom.yaw_rate_rad_s;
    p.gyro_z_bias_rad_s = odom.gyro_z_bias_rad_s;
    p.quality_flags = odom.quality_flags;
    p.stance_mask = odom.stance_mask;
    p.wheel_online_mask = odom.wheel_online_mask;
    return send_proto_payload(PROTO_FUNC_ODOMETRY, &p, (uint8_t)sizeof(p));
}

/* 发送上行帧 */
static void send_state_frame(void) {
    payload_state_t p = build_state_payload();
    (void)send_proto_payload(PROTO_FUNC_STATE, &p, (uint8_t)sizeof(p));
}

int task_comm_send_system_status(uint32_t now_ms) {
    payload_system_status_t status = build_system_status_payload(now_ms);
    return send_proto_payload(PROTO_FUNC_SYSTEM_STATUS,
                              &status,
                              (uint8_t)sizeof(status));
}

static int build_motor_payload(uint8_t motor_idx, payload_motor_one_t* out) {
    if (!out) return 0;

    motor_dev_t* d = motor_get((motor_logical_id_t)motor_idx);
    if (!d) return 0;

    out->id            = (uint16_t)motor_idx;
    out->online        = d->state.online;
    out->type          = (uint8_t)d->state.type;
    out->angle_rad     = d->state.angle_rad;
    out->velocity_rads = d->state.velocity_rads;
    return 1;
}

static uint8_t next_motor_tx_index(uint8_t motor_idx) {
    return (uint8_t)((motor_idx + 1U) % (uint8_t)motor_registry_count());
}

static void send_motor_payload_if_present(uint8_t motor_idx) {
    payload_motor_one_t p;
    if (build_motor_payload(motor_idx, &p)) {
        (void)send_proto_payload(PROTO_FUNC_MOTOR_STATE, &p, (uint8_t)sizeof(p));
    }
}

static void send_motor_frame(void) {
    /* 每次发送一个电机的状态，循环发送 */
    static uint8_t s_motor_tx_idx = 0;

    send_motor_payload_if_present(s_motor_tx_idx);
    s_motor_tx_idx = next_motor_tx_index(s_motor_tx_idx);
}

void task_comm_entry(void* arg) {
    (void)arg;
    LOGI("task_comm started");
#if APP_TARGET_MCU
    /* 解析在 bsp_usb_cdc 的 rx 回调里同步完成；本任务做上行帧定时发送 */
    for (;;) {
        uint32_t now = (uint32_t)bsp_time_now_ms();
        task_comm_watchdog_step(now);
        bsp_usb_cdc_process();

        /* 低频发送整机状态，避免串口调试助手被上行帧刷满 */
        if ((now - s_last_state_tx_ms) >= STATE_TX_INTERVAL_MS) {
            send_state_frame();
            s_last_state_tx_ms = now;
        }

        if ((now - s_last_system_tx_ms) >= STATE_TX_INTERVAL_MS) {
            if (task_comm_send_system_status(now) > 0) {
                s_last_system_tx_ms = now;
            }
        }

        /* 低频发送电机状态 (轮流) */
        if ((now - s_last_motor_tx_ms) >= MOTOR_TX_INTERVAL_MS) {
            send_motor_frame();
            s_last_motor_tx_ms = now;
        }

        /* 四轮同步反馈用于排查直行左右轮速差；USB 忙时下一轮重试。 */
        if ((now - s_last_wheel_tx_ms) >= WHEEL_TX_INTERVAL_MS) {
            if (task_comm_send_wheel_feedback() > 0) {
                s_last_wheel_tx_ms = now;
            }
        }

        if ((now - s_last_diag_tx_ms) >= DIAG_TX_INTERVAL_MS) {
            if (task_comm_send_chassis_diag() > 0) {
                s_last_diag_tx_ms = now;
            }
        }

        if ((now - s_last_odom_tx_ms) >= ODOM_TX_INTERVAL_MS) {
            if (task_comm_send_odometry() > 0) {
                s_last_odom_tx_ms = now;
            }
        }

        bsp_usb_cdc_process();

        osDelay(5);
    }
#endif
}
