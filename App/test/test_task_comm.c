/*
 * test_task_comm.c — 验证 CDC→解析→分发→chassis_cmd 流水线
 */
#include "test_util.h"
#include "task_comm.h"
#include "bsp_usb_cdc.h"
#include "proto_frame.h"
#include "proto_defs.h"
#include "log.h"

static void t_chassis_cmd_flow(void) {
    bsp_usb_cdc_init();
    task_comm_init();

    payload_chassis_cmd_t cmd = { .vx = 0.3f, .vy = -0.1f, .wz = 0.5f };
    uint8_t frame[64];
    int n = proto_frame_build(PROTO_FUNC_CHASSIS_CMD, (uint8_t*)&cmd, 12, frame, sizeof(frame));
    TEST_ASSERT(n > 0);

    bsp_usb_cdc_test_inject_rx(frame, (uint32_t)n);

    task_comm_chassis_cmd_t got;
    task_comm_get_chassis(&got);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f,  got.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.1f, got.vy);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f,  got.wz);
    TEST_ASSERT_EQUAL_UINT(1, got.seq);
    TEST_ASSERT_EQUAL_UINT(1, task_comm_good_cnt());
    TEST_ASSERT_EQUAL_UINT(1, task_comm_dispatch_hit());
}

static void t_bad_frame_counts(void) {
    bsp_usb_cdc_init();
    task_comm_init();
    /* 故意构造错误 FuncID */
    payload_chassis_cmd_t cmd = {0};
    uint8_t frame[64];
    int n = proto_frame_build(0x77, (uint8_t*)&cmd, 12, frame, sizeof(frame));
    bsp_usb_cdc_test_inject_rx(frame, (uint32_t)n);
    TEST_ASSERT_EQUAL_UINT(1, task_comm_dispatch_miss());
}

int main(void) {
    log_init();
    log_set_global_level(LOG_LVL_ERR);
    TU_RUN(t_chassis_cmd_flow);
    TU_RUN(t_bad_frame_counts);
    TU_MAIN_EPILOGUE();
}
