/*
 * task_comm.c — 解析 USB CDC 字节 → 帧 → FuncID 分发
 */
#include "task_comm.h"
#include "log.h"
#include "config.h"
#include "bsp_usb_cdc.h"
#include "proto_frame.h"
#include "proto_dispatch.h"
#include "proto_defs.h"

#if APP_TARGET_MCU
#include "cmsis_os.h"
#endif

#include <string.h>

static const char* TAG = "COMM";

static proto_frame_parser_t  s_parser;
static proto_dispatcher_t    s_disp;
static task_comm_chassis_cmd_t s_chassis;

static int handle_chassis(const uint8_t* p, uint8_t len) {
    if (len != sizeof(payload_chassis_cmd_t)) return -1;
    payload_chassis_cmd_t cmd;
    memcpy(&cmd, p, sizeof(cmd));
    s_chassis.vx = cmd.vx;
    s_chassis.vy = cmd.vy;
    s_chassis.wz = cmd.wz;
    s_chassis.seq++;
    return 0;
}

static const proto_entry_t s_tbl[] = {
    { PROTO_FUNC_CHASSIS_CMD, sizeof(payload_chassis_cmd_t), handle_chassis, "chassis" },
};

static void on_usb_rx(const uint8_t* d, uint32_t n, void* user) {
    (void)user;
    proto_frame_feed(&s_parser, d, n);
}

void task_comm_init(void) {
    memset(&s_chassis, 0, sizeof(s_chassis));
    proto_dispatcher_init(&s_disp, s_tbl, sizeof(s_tbl)/sizeof(s_tbl[0]));
    proto_frame_init(&s_parser, proto_dispatch_on_frame, &s_disp);
    bsp_usb_cdc_attach_rx(on_usb_rx, NULL);
    LOGI("comm init: %u handlers", (unsigned)(sizeof(s_tbl)/sizeof(s_tbl[0])));
}

uint32_t task_comm_good_cnt(void)        { return s_parser.good_cnt; }
uint32_t task_comm_bad_cnt(void)         { return s_parser.bad_cnt; }
uint32_t task_comm_dispatch_hit(void)    { return s_disp.hit_cnt; }
uint32_t task_comm_dispatch_miss(void)   { return s_disp.miss_cnt; }

void task_comm_get_chassis(task_comm_chassis_cmd_t* out) {
    if (!out) return;
    *out = s_chassis;
}

void task_comm_entry(void* arg) {
    (void)arg;
    LOGI("task_comm started");
#if APP_TARGET_MCU
    /* 解析在 bsp_usb_cdc 的 rx 回调里同步完成；本任务只做心跳 + 后续上行 */
    for (;;) { osDelay(50); }
#endif
}
