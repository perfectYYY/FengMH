/*
 * task_comm.c — 解析 USB CDC 字节 → 帧 → FuncID 分发 + 上行帧发送
 *
 * 下行：USB CDC RX → proto_frame 解析 → proto_dispatch 分发
 * 上行：定时构造 0x80(整机状态) / 0x81(电机状态) /
 *       0x88(四轮实际转速) / 0x89(底盘诊断) / 0x8A(TROT测试诊断) 帧 → USB CDC TX
 */
#include "task_comm.h"
#include "log.h"
#include "config.h"
#include "bsp_usb_cdc.h"
#include "bsp_time.h"
#include "proto_frame.h"
#include "proto_dispatch.h"
#include "proto_defs.h"
#include "bmi088_v2.h"
#include "motor_registry.h"
#include "motor_m3508.h"
#include "pump_control.h"
#include "task_chassis.h"
#include "task_safety.h"
#include "err.h"

#if APP_TARGET_MCU
#include "cmsis_os.h"
#endif

#include <string.h>
#include <math.h>

static const char* TAG = "COMM";

#define NAV_VALIDATED_MAX_VX_M_S 0.50f
#define COMM_RX_RING_SIZE         2048U
#define COMM_RX_RING_MASK         (COMM_RX_RING_SIZE - 1U)

static uint8_t s_rx_ring[COMM_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static task_comm_rx_stats_t s_rx_stats;

static proto_frame_parser_t  s_parser;
static proto_dispatcher_t    s_disp;
static task_comm_chassis_cmd_t s_chassis;
static task_comm_arm_target_t s_arm_target;
static task_comm_arm_pump_t   s_arm_pump;
static task_comm_mode_cmd_t   s_mode_cmd;
static uint8_t                 s_estop_latched;
static volatile uint32_t     s_last_rx_ms = 0;

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
static uint32_t s_last_wheel_tx_ms  = 0;  /* 0x88 上次发送时间 */
static uint32_t s_last_imu_tx_ms    = 0;  /* 0x83 上次发送时间 */
static uint32_t s_last_diag_tx_ms   = 0;  /* 0x89 上次发送时间 */
static uint32_t s_last_trot_test_tx_ms = 0; /* 0x8A 上次发送时间 */
#if APP_TARGET_MCU
static const uint32_t STATE_TX_INTERVAL_MS  = 100;  /* 10Hz */
static const uint32_t MOTOR_TX_INTERVAL_MS  = 200;  /* 5Hz, motor states are round-robin */
static const uint32_t WHEEL_TX_INTERVAL_MS  = 40;   /* 25Hz, FL/FR/RL/RR synchronized */
static const uint32_t IMU_TX_INTERVAL_MS    = 100;  /* 10Hz, BMI receive diagnostics */
static const uint32_t DIAG_TX_INTERVAL_MS   = 40;   /* 25Hz */
static const uint32_t TROT_TEST_TX_INTERVAL_MS = 40; /* 25Hz, only mode 3 */
static const uint8_t TX_STATE_ENABLE        = 1U;   /* 0x80: complete host state feedback */
static const uint8_t TX_MOTOR_ENABLE        = 1U;   /* 0x81: round-robin motor feedback */
static const uint8_t TX_WHEEL_ENABLE        = 1U;   /* 0x88: keep four-wheel velocity telemetry */
static const uint8_t TX_DIAG_ENABLE         = 1U;   /* 0x89: keep steer diagnostics when present */
static const uint8_t TX_TROT_TEST_ENABLE    = 1U;
#endif

static void mark_valid_rx(void) {
    s_last_rx_ms = (uint32_t)bsp_time_now_ms();
}

#if APP_CHASSIS_ENABLE
static void cache_chassis_base_command(const uint8_t* p) {
    payload_chassis_cmd_t cmd;
    memcpy(&cmd, p, sizeof(cmd));
    if (!isfinite(cmd.vx)) {
        s_chassis.vx = 0.0f;
    } else if (cmd.vx > NAV_VALIDATED_MAX_VX_M_S) {
        s_chassis.vx = NAV_VALIDATED_MAX_VX_M_S;
    } else if (cmd.vx < -NAV_VALIDATED_MAX_VX_M_S) {
        s_chassis.vx = -NAV_VALIDATED_MAX_VX_M_S;
    } else {
        s_chassis.vx = cmd.vx;
    }
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
        s_chassis.steer_mode = PROTO_STEER_MODE_AUTO_HOLD;
    }
}

static void mark_chassis_command_rx(void) {
    s_chassis.seq++;
    mark_valid_rx();
}
#endif

static int send_proto_payload(uint8_t func_id, const void* payload, uint8_t len);

#if APP_CHASSIS_ENABLE
static void request_mode_safe_stand(void) {
    (void)task_chassis_set_wheel_test(0U, 0U, NULL, (uint32_t)bsp_time_now_ms());
    s_chassis.vx = 0.0f;
    s_chassis.vy = 0.0f;
    s_chassis.wz = 0.0f;
    s_chassis.target_yaw = 0.0f;
    s_chassis.steer_mode = 0U;
    s_chassis.seq++;

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

static int handle_trot_test_config(const uint8_t* p, uint8_t len) {
#if !APP_CHASSIS_ENABLE
    (void)p;
    (void)len;
    return APP_ERR_UNSUPPORTED;
#else
    if (len != sizeof(payload_trot_test_config_t)) return APP_ERR_INVALID_ARG;

    payload_trot_test_config_t cmd;
    float trims[GAIT_LEG_NUM];
    memcpy(&cmd, p, sizeof(cmd));
    memcpy(trims, cmd.foot_z_trim_m, sizeof(trims));
    int ret = task_chassis_set_trot_test_config(cmd.step_height_m,
                                                cmd.period_s,
                                                cmd.duty,
                                                trims);
    if (ret != APP_OK) return ret;
    mark_valid_rx();
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
    if (len < sizeof(payload_chassis_cmd_t)) return -1;

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
    if (len != sizeof(payload_arm_target_t)) return -1;

    payload_arm_target_t cmd;
    for (uint32_t i = 0U; i < sizeof(payload_arm_target_t); i++) {
        debug_comm_arm_target_raw[i] = p[i];
    }
    memcpy(&cmd, p, sizeof(cmd));
    debug_comm_arm_target_type = cmd.target_type;
    debug_comm_arm_target_x_m = cmd.x_m;
    debug_comm_arm_target_y_m = cmd.y_m;
    debug_comm_arm_target_z_m = cmd.z_m;

    if (cmd.target_type != PROTO_ARM_TARGET_GRASP &&
        cmd.target_type != PROTO_ARM_TARGET_PLACE) {
        debug_comm_arm_target_reject_count++;
        return APP_ERR_INVALID_ARG;
    }

    /* 机械臂两连杆总长 0.65 m；在通信入口拦截错字节序/错单位的有限异常值。 */
    const float distance_sq = cmd.x_m * cmd.x_m +
                              cmd.y_m * cmd.y_m +
                              cmd.z_m * cmd.z_m;
    const float min_reach_m = 0.045f;
    const float max_reach_m = 0.655f;
    if (!isfinite(cmd.x_m) || !isfinite(cmd.y_m) || !isfinite(cmd.z_m) ||
        !isfinite(distance_sq) ||
        distance_sq < min_reach_m * min_reach_m ||
        distance_sq > max_reach_m * max_reach_m) {
        debug_comm_arm_target_reject_count++;
        return APP_ERR_INVALID_ARG;
    }

    s_arm_target.target_type = cmd.target_type;
    s_arm_target.x_m = cmd.x_m;
    s_arm_target.y_m = cmd.y_m;
    s_arm_target.z_m = cmd.z_m;
    s_arm_target.seq++;
    debug_comm_arm_target_accept_count++;
    mark_valid_rx();
    return 0;
}

static int handle_arm_pump(const uint8_t* p, uint8_t len) {
    if (len != sizeof(payload_arm_pump_t)) return -1;

    payload_arm_pump_t cmd;
    memcpy(&cmd, p, sizeof(cmd));
    if (cmd.pump_on > 1U) {
        return APP_ERR_INVALID_ARG;
    }

    s_arm_pump.pump_on = cmd.pump_on;
    s_arm_pump.seq++;
    mark_valid_rx();
    return 0;
}

static int handle_arm_aux_gpio(const uint8_t* p, uint8_t len) {
    if (len != sizeof(payload_arm_aux_gpio_t)) return -1;

    payload_arm_aux_gpio_t cmd;
    memcpy(&cmd, p, sizeof(cmd));
    if (cmd.on > 1U || cmd.channel > PROTO_ARM_AUX_GPIO_PA9) {
        return APP_ERR_INVALID_ARG;
    }
    if (s_mode_cmd.seq == 0U ||
        (s_mode_cmd.mode != PROTO_ROBOT_MODE_ARM &&
         s_mode_cmd.mode != PROTO_ROBOT_MODE_REAR_PLACE)) {
        return APP_ERR_UNSUPPORTED;
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

static int handle_mode_cmd(const uint8_t* p, uint8_t len) {
    if (len != sizeof(payload_mode_cmd_t)) return -1;

    payload_mode_cmd_t cmd;
    memcpy(&cmd, p, sizeof(cmd));
    if (cmd.mode > PROTO_ROBOT_MODE_MAX) {
        return APP_ERR_INVALID_ARG;
    }

    if (s_estop_latched && cmd.mode != PROTO_ROBOT_MODE_ESTOP) {
        return APP_ERR_UNSUPPORTED;
    }
    if (task_safety_estop_active() &&
        cmd.mode != PROTO_ROBOT_MODE_ESTOP &&
        cmd.mode != PROTO_ROBOT_MODE_ERROR &&
        cmd.mode != PROTO_ROBOT_MODE_IDLE) {
        return APP_ERR_UNSUPPORTED;
    }
    if (s_mode_cmd.seq > 0U && s_mode_cmd.mode == cmd.mode) {
        mark_valid_rx();
        return 0;
    }

    s_mode_cmd.mode = cmd.mode;
    s_mode_cmd.seq++;
    if (cmd.mode == PROTO_ROBOT_MODE_ESTOP) {
        s_estop_latched = 1U;
        task_safety_estop_set(true);
    } else if (cmd.mode == PROTO_ROBOT_MODE_ERROR) {
        task_safety_estop_set(true);
    } else if (cmd.mode == PROTO_ROBOT_MODE_IDLE) {
        /* ERROR recovery is explicit: only a clean IDLE request may release it. */
        task_safety_estop_set(false);
    }
#if APP_CHASSIS_ENABLE
    if (cmd.mode == PROTO_ROBOT_MODE_ESTOP ||
        cmd.mode == PROTO_ROBOT_MODE_ERROR) {
        request_mode_safe_stand();
    }
#endif
    mark_valid_rx();
    return 0;
}

static const proto_entry_t s_tbl[] = {
    { PROTO_FUNC_CHASSIS_CMD, 0, handle_chassis, "chassis" },  /* expect_len=0: 由 handler 内自行校验 */
    { PROTO_FUNC_ARM_TARGET, sizeof(payload_arm_target_t), handle_arm_target, "arm_target" },
    { PROTO_FUNC_GAIT_CMD, sizeof(payload_gait_cmd_t), handle_gait, "gait" },
    { PROTO_FUNC_MIT_CMD, sizeof(payload_mit_cmd_t), handle_mit_cmd, "mit" },
    { PROTO_FUNC_ARM_PUMP, sizeof(payload_arm_pump_t), handle_arm_pump, "arm_pump" },
    { PROTO_FUNC_MODE_CMD, sizeof(payload_mode_cmd_t), handle_mode_cmd, "mode" },
    { PROTO_FUNC_WHEEL_TEST, sizeof(payload_wheel_test_t), handle_wheel_test, "wheel_test" },
    { PROTO_FUNC_ARM_AUX_GPIO, sizeof(payload_arm_aux_gpio_t), handle_arm_aux_gpio, "arm_aux_gpio" },
    { PROTO_FUNC_TROT_TEST_CONFIG, sizeof(payload_trot_test_config_t), handle_trot_test_config, "trot_test_config" },
};

static void on_usb_rx(const uint8_t* d, uint32_t n, void* user) {
    (void)user;
    if (!d) return;
    for (uint32_t i = 0U; i < n; i++) {
        uint16_t head = s_rx_head;
        uint16_t next = (uint16_t)((head + 1U) & COMM_RX_RING_MASK);
        if (next == s_rx_tail) {
            s_rx_stats.ring_overflow_bytes++;
            continue;
        }
        s_rx_ring[head] = d[i];
        s_rx_head = next;
    }
#if APP_TARGET_HOST
    /* Host tests have no communication RTOS task. */
    task_comm_process_rx();
#endif
}

static uint16_t rx_available(void) {
    return (uint16_t)((s_rx_head - s_rx_tail) & COMM_RX_RING_MASK);
}

static uint8_t rx_peek(uint16_t offset) {
    return s_rx_ring[(s_rx_tail + offset) & COMM_RX_RING_MASK];
}

static void rx_discard(uint16_t count) {
    s_rx_tail = (uint16_t)((s_rx_tail + count) & COMM_RX_RING_MASK);
}

static void rx_copy(uint8_t* out, uint16_t count) {
    for (uint16_t i = 0U; i < count; i++) out[i] = rx_peek(i);
}

void task_comm_process_rx(void) {
    uint8_t frame[6U + BMI088_V2_MAX_PAYLOAD_LEN + 4U];

    for (;;) {
        uint16_t available = rx_available();
        if (available < 2U) return;

        uint8_t b0 = rx_peek(0U);
        uint8_t b1 = rx_peek(1U);
        if (b0 == PROTO_HEAD1 && b1 == PROTO_HEAD2) {
            if (available < 4U) return;
            uint16_t total = (uint16_t)(5U + rx_peek(3U));
            if (rx_peek(3U) > PROTO_MAX_PAYLOAD) {
                s_rx_stats.demux_discarded_bytes++;
                rx_discard(1U);
                continue;
            }
            if (available < total) return;
            rx_copy(frame, total);
            rx_discard(total);
            proto_frame_feed(&s_parser, frame, total);
            s_rx_stats.legacy_frames++;
            continue;
        }

        if (b0 == BMI088_V2_MAGIC0 && b1 == BMI088_V2_MAGIC1) {
            if (available < 6U) return;
            uint16_t payload_len = (uint16_t)rx_peek(4U) |
                                   ((uint16_t)rx_peek(5U) << 8);
            if (rx_peek(2U) != BMI088_V2_VERSION ||
                payload_len > BMI088_V2_MAX_PAYLOAD_LEN) {
                s_rx_stats.bmi_header_errors++;
                rx_discard(1U);
                continue;
            }
            uint16_t total = (uint16_t)(6U + payload_len + 4U);
            if (available < total) return;
            rx_copy(frame, total);
            rx_discard(total);
            if (bmi088_v2_accept_frame(frame, total,
                                       (uint32_t)bsp_time_now_ms())) {
                s_rx_stats.bmi_frames++;
            }
            continue;
        }

        s_rx_stats.demux_discarded_bytes++;
        rx_discard(1U);
    }
}

void task_comm_init(void) {
    memset(&s_chassis, 0, sizeof(s_chassis));
    memset(&s_arm_target, 0, sizeof(s_arm_target));
    memset(&s_arm_pump, 0, sizeof(s_arm_pump));
    memset(&s_mode_cmd, 0, sizeof(s_mode_cmd));
    s_estop_latched = 0U;
    memset((void*)debug_comm_arm_target_raw, 0, sizeof(debug_comm_arm_target_raw));
    debug_comm_arm_target_type = 0U;
    debug_comm_arm_target_x_m = 0.0f;
    debug_comm_arm_target_y_m = 0.0f;
    debug_comm_arm_target_z_m = 0.0f;
    debug_comm_arm_target_accept_count = 0U;
    debug_comm_arm_target_reject_count = 0U;
    s_last_rx_ms = 0;
    s_last_state_tx_ms = 0;
    s_last_motor_tx_ms = 0;
    s_last_wheel_tx_ms = 0;
    s_last_imu_tx_ms = 0;
    s_last_diag_tx_ms = 0;
    s_last_trot_test_tx_ms = 0;
    s_rx_head = 0U;
    s_rx_tail = 0U;
    memset(&s_rx_stats, 0, sizeof(s_rx_stats));
    bmi088_v2_init();
    proto_dispatcher_init(&s_disp, s_tbl, sizeof(s_tbl)/sizeof(s_tbl[0]));
    proto_frame_init(&s_parser, proto_dispatch_on_frame, &s_disp);
    bsp_usb_cdc_attach_rx(on_usb_rx, NULL);
    LOGI("comm init: %u handlers", (unsigned)(sizeof(s_tbl)/sizeof(s_tbl[0])));
}

uint32_t task_comm_good_cnt(void)        { return s_parser.good_cnt; }
uint32_t task_comm_bad_cnt(void)         { return s_parser.bad_cnt; }
uint32_t task_comm_dispatch_hit(void)    { return s_disp.hit_cnt; }
uint32_t task_comm_dispatch_miss(void)   { return s_disp.miss_cnt; }
uint32_t task_comm_last_rx_ms(void)      { return s_last_rx_ms; }

void task_comm_get_rx_stats(task_comm_rx_stats_t* out) {
    if (out) *out = s_rx_stats;
}

void task_comm_get_chassis(task_comm_chassis_cmd_t* out) {
    if (!out) return;
    *out = s_chassis;
}

void task_comm_get_arm_target(task_comm_arm_target_t* out) {
    if (!out) return;
    *out = s_arm_target;
}

void task_comm_get_arm_pump(task_comm_arm_pump_t* out) {
    if (!out) return;
    *out = s_arm_pump;
}

void task_comm_get_mode_cmd(task_comm_mode_cmd_t* out) {
    if (!out) return;
    *out = s_mode_cmd;
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

static int send_proto_payload(uint8_t func_id, const void* payload, uint8_t len) {
    uint8_t buf[64];
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

int task_comm_send_mode_feedback(const payload_mode_feedback_t* feedback) {
    if (!feedback) return APP_ERR_INVALID_ARG;
    return send_proto_payload(PROTO_FUNC_MODE_FEEDBACK,
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

int task_comm_send_imu_feedback(void) {
    payload_imu_state_t p;
    bmi088_v2_sample_t sample;
    bmi088_v2_stats_t stats;
    uint32_t now = (uint32_t)bsp_time_now_ms();
    memset(&p, 0, sizeof(p));
    bmi088_v2_get_stats(&stats);
    p.timestamp_ms = now;
    p.accepted = stats.accepted;
    p.crc_errors = stats.crc_errors;
    p.skipped_samples = stats.skipped_samples;
    if (bmi088_v2_get_latest(&sample, now, BMI088_V2_TIMEOUT_MS)) {
        p.sequence = sample.sequence;
        p.roll_rad = sample.roll;
        p.pitch_rad = sample.pitch;
        p.yaw_rad = sample.yaw;
        p.gyro_z_rad_s = sample.calibrated_gyro[2] - sample.gyro_bias[2];
        p.velocity_n_m_s = sample.velocity[0];
        p.velocity_w_m_s = sample.velocity[1];
        p.velocity_u_m_s = sample.velocity[2];
        uint32_t age = now - sample.received_ms;
        p.age_ms = (uint16_t)(age > 0xFFFFU ? 0xFFFFU : age);
        p.valid = 1U;
    }
    return send_proto_payload(PROTO_FUNC_IMU_STATE, &p, (uint8_t)sizeof(p));
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
    p.flags = (uint16_t)(status.diagnostic_flags |
              (((uint16_t)g_chassis_wheel_test.active << PROTO_DIAG_WHEEL_TEST_MODE_SHIFT) &
               PROTO_DIAG_WHEEL_TEST_MODE_MASK));
    return send_proto_payload(PROTO_FUNC_CHASSIS_DIAG, &p, (uint8_t)sizeof(p));
}

int task_comm_send_trot_test_diag(void) {
    payload_trot_test_diag_t p;
    chassis_trot_test_debug_t debug;
    memset(&p, 0, sizeof(p));
    memset(&debug, 0, sizeof(debug));
    task_chassis_get_trot_test_debug(&debug);

    p.timestamp_ms = (uint32_t)bsp_time_now_ms();
    float phase_permille = debug.phase * 1000.0f;
    if (!isfinite(phase_permille) || phase_permille < 0.0f) phase_permille = 0.0f;
    if (phase_permille > 999.0f) phase_permille = 999.0f;
    p.phase_permille = (uint16_t)phase_permille;
    p.stance_mask = (uint8_t)(debug.stance_mask & 0x0FU);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        p.target_z_mm[i] = diag_float_to_i16(debug.target_foot_z_m[i], 1000.0f);
        p.joint_error_mrad[i * 2] = diag_float_to_i16(debug.joint_error_rad[i * 2], 1000.0f);
        p.joint_error_mrad[i * 2 + 1] =
            diag_float_to_i16(debug.joint_error_rad[i * 2 + 1], 1000.0f);
    }
    return send_proto_payload(PROTO_FUNC_TROT_TEST_DIAG, &p, (uint8_t)sizeof(p));
}

/* 发送上行帧 */
static void send_state_frame(void) {
    payload_state_t p = build_state_payload();
    (void)send_proto_payload(PROTO_FUNC_STATE, &p, (uint8_t)sizeof(p));
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
    /* CDC 回调只入环形缓冲区；本任务统一解复用 55 AA / A5 5A。 */
    for (;;) {
        uint32_t now = (uint32_t)bsp_time_now_ms();

        task_comm_process_rx();

        /* 低频发送整机状态，避免串口调试助手被上行帧刷满 */
        if (TX_STATE_ENABLE && (now - s_last_state_tx_ms) >= STATE_TX_INTERVAL_MS) {
            send_state_frame();
            s_last_state_tx_ms = now;
        }

        /* 低频发送电机状态 (轮流) */
        if (TX_MOTOR_ENABLE && (now - s_last_motor_tx_ms) >= MOTOR_TX_INTERVAL_MS) {
            send_motor_frame();
            s_last_motor_tx_ms = now;
        }

        /* 四轮同步反馈用于排查直行左右轮速差；USB 忙时下一轮重试。 */
        if (TX_WHEEL_ENABLE && (now - s_last_wheel_tx_ms) >= WHEEL_TX_INTERVAL_MS) {
            if (task_comm_send_wheel_feedback() > 0) {
                s_last_wheel_tx_ms = now;
            }
        }

        if ((now - s_last_imu_tx_ms) >= IMU_TX_INTERVAL_MS) {
            if (task_comm_send_imu_feedback() > 0) {
                s_last_imu_tx_ms = now;
            }
        }

        if (TX_DIAG_ENABLE && (now - s_last_diag_tx_ms) >= DIAG_TX_INTERVAL_MS) {
            if (task_comm_send_chassis_diag() > 0) {
                s_last_diag_tx_ms = now;
            }
        }


        if (TX_TROT_TEST_ENABLE &&
            g_chassis_wheel_test.active == PROTO_WHEEL_TEST_VERTICAL_DRIVE &&
            (now - s_last_trot_test_tx_ms) >= TROT_TEST_TX_INTERVAL_MS) {
            if (task_comm_send_trot_test_diag() > 0) {
                s_last_trot_test_tx_ms = now;
            }
        }

        bsp_usb_cdc_process();

        osDelay(5);
    }
#endif
}
