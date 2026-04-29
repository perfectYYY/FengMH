/*
 * task_comm.c — 解析 USB CDC 字节 → 帧 → FuncID 分发 + 上行帧发送
 *
 * 下行：USB CDC RX → proto_frame 解析 → proto_dispatch 分发
 * 上行：定时构造 0x80(整机状态) / 0x81(电机状态) 帧 → USB CDC TX
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
#include "task_chassis.h"
#include "task_safety.h"

#if APP_TARGET_MCU
#include "cmsis_os.h"
#endif

#include <string.h>

static const char* TAG = "COMM";

static proto_frame_parser_t  s_parser;
static proto_dispatcher_t    s_disp;
static task_comm_chassis_cmd_t s_chassis;
static volatile uint32_t     s_last_rx_ms = 0;

/* 上行帧发送周期控制 */
static uint32_t s_last_state_tx_ms  = 0;  /* 0x80 上次发送时间 */
static uint32_t s_last_motor_tx_ms  = 0;  /* 0x81 上次发送时间 */
#if APP_TARGET_MCU
static const uint32_t STATE_TX_INTERVAL_MS  = 10;  /* 100Hz */
static const uint32_t MOTOR_TX_INTERVAL_MS  = 50;  /* 20Hz */
#endif

static int handle_chassis(const uint8_t* p, uint8_t len) {
    if (len != sizeof(payload_chassis_cmd_t)) return -1;
    payload_chassis_cmd_t cmd;
    memcpy(&cmd, p, sizeof(cmd));
    s_chassis.vx = cmd.vx;
    s_chassis.vy = cmd.vy;
    s_chassis.wz = cmd.wz;
    s_chassis.seq++;
    s_last_rx_ms = (uint32_t)bsp_time_now_ms();
    return 0;
}

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

static int handle_gait(const uint8_t* p, uint8_t len) {
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
    default:
        ret = APP_ERR_INVALID_ARG;
        break;
    }

    if (ret == APP_OK) {
        s_last_rx_ms = (uint32_t)bsp_time_now_ms();
        return 0;
    }
    return ret;
}

static const proto_entry_t s_tbl[] = {
    { PROTO_FUNC_CHASSIS_CMD, sizeof(payload_chassis_cmd_t), handle_chassis, "chassis" },
    { PROTO_FUNC_GAIT_CMD, sizeof(payload_gait_cmd_t), handle_gait, "gait" },
};

static void on_usb_rx(const uint8_t* d, uint32_t n, void* user) {
    (void)user;
    proto_frame_feed(&s_parser, d, n);
}

void task_comm_init(void) {
    memset(&s_chassis, 0, sizeof(s_chassis));
    s_last_rx_ms = 0;
    s_last_state_tx_ms = 0;
    s_last_motor_tx_ms = 0;
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

void task_comm_get_chassis(task_comm_chassis_cmd_t* out) {
    if (!out) return;
    *out = s_chassis;
}

/* ─── 上行帧构造 ─── */

#pragma pack(push, 1)

/* 0x80 整机状态 payload */
typedef struct {
    uint8_t  mode;          /* chassis_mode_t */
    uint8_t  gait_active;   /* 0=stand 1=trot 2=script */
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

/* 发送上行帧 */
static void send_state_frame(void) {
    payload_state_t p;
    memset(&p, 0, sizeof(p));
    p.mode        = (uint8_t)task_chassis_get_mode();
    p.gait_active = 0;  /* TODO: 从 task_chassis 获取 */
    p.estop       = task_safety_estop_active() ? 1 : 0;
    p.vx_cmd      = s_chassis.vx;
    p.vy_cmd      = s_chassis.vy;
    p.wz_cmd      = s_chassis.wz;
    p.uptime_ms   = (uint32_t)bsp_time_now_ms();

    uint8_t buf[64];
    int n = proto_frame_build(PROTO_FUNC_STATE,
                               (const uint8_t*)&p, (uint8_t)sizeof(p),
                               buf, sizeof(buf));
    if (n > 0) {
        bsp_usb_cdc_send(buf, (uint32_t)n);
    }
}

static void send_motor_frame(void) {
    /* 每次发送一个电机的状态，循环发送 */
    static uint8_t s_motor_tx_idx = 0;

    motor_dev_t* d = motor_get((motor_logical_id_t)s_motor_tx_idx);
    if (d) {
        payload_motor_one_t p;
        p.id            = (uint16_t)s_motor_tx_idx;
        p.online        = d->state.online;
        p.type          = (uint8_t)d->state.type;
        p.angle_rad     = d->state.angle_rad;
        p.velocity_rads = d->state.velocity_rads;

        uint8_t buf[64];
        int n = proto_frame_build(PROTO_FUNC_MOTOR_STATE,
                                   (const uint8_t*)&p, (uint8_t)sizeof(p),
                                   buf, sizeof(buf));
        if (n > 0) {
            bsp_usb_cdc_send(buf, (uint32_t)n);
        }
    }

    s_motor_tx_idx = (s_motor_tx_idx + 1) % (uint8_t)motor_registry_count();
}

void task_comm_entry(void* arg) {
    (void)arg;
    LOGI("task_comm started");
#if APP_TARGET_MCU
    /* 解析在 bsp_usb_cdc 的 rx 回调里同步完成；本任务做上行帧定时发送 */
    for (;;) {
        uint32_t now = (uint32_t)bsp_time_now_ms();

        /* 100Hz 发送整机状态 */
        if ((now - s_last_state_tx_ms) >= STATE_TX_INTERVAL_MS) {
            send_state_frame();
            s_last_state_tx_ms = now;
        }

        /* 20Hz 发送电机状态 (轮流) */
        if ((now - s_last_motor_tx_ms) >= MOTOR_TX_INTERVAL_MS) {
            send_motor_frame();
            s_last_motor_tx_ms = now;
        }

        osDelay(5);
    }
#endif
}
