/*
 * test_proto_frame.c — 帧解析状态机测试
 */
#include "test_util.h"
#include "proto_frame.h"
#include "proto_dispatch.h"
#include "log.h"

#include <string.h>

static int           s_cb_calls;
static proto_frame_t s_last_frame;

static void on_frame(const proto_frame_t* f, void* user) {
    (void)user;
    s_cb_calls++;
    s_last_frame = *f;
}

static void t_build_and_parse_chassis_cmd(void) {
    proto_frame_parser_t p;
    proto_frame_init(&p, on_frame, NULL);

    payload_chassis_cmd_t cmd = { .vx = 0.5f, .vy = -0.25f, .wz = 1.0f };
    uint8_t buf[64];
    int n = proto_frame_build(PROTO_FUNC_CHASSIS_CMD,
                              (const uint8_t*)&cmd, sizeof(cmd),
                              buf, sizeof(buf));
    TEST_ASSERT(n == 5 + (int)sizeof(cmd));
    TEST_ASSERT_EQUAL_HEX8(0x55, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xAA, buf[1]);
    TEST_ASSERT_EQUAL_HEX8(PROTO_FUNC_CHASSIS_CMD, buf[2]);
    TEST_ASSERT_EQUAL_INT(12, buf[3]);

    s_cb_calls = 0;
    proto_frame_feed(&p, buf, (uint32_t)n);
    TEST_ASSERT_EQUAL_INT(1, s_cb_calls);
    TEST_ASSERT_EQUAL_UINT(1, p.good_cnt);
    TEST_ASSERT_EQUAL_UINT(0, p.bad_cnt);
    TEST_ASSERT_EQUAL_INT(PROTO_FUNC_CHASSIS_CMD, s_last_frame.func_id);
    TEST_ASSERT_EQUAL_INT(12, s_last_frame.len);

    payload_chassis_cmd_t got;
    memcpy(&got, s_last_frame.payload, sizeof(got));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f,   got.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.25f, got.vy);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f,   got.wz);
}

static void t_byte_by_byte_feed(void) {
    proto_frame_parser_t p;
    proto_frame_init(&p, on_frame, NULL);
    payload_chassis_cmd_t cmd = { 1.0f, 2.0f, 3.0f };
    uint8_t buf[64];
    int n = proto_frame_build(PROTO_FUNC_CHASSIS_CMD, (uint8_t*)&cmd, 12, buf, sizeof(buf));
    s_cb_calls = 0;
    for (int i = 0; i < n; i++) proto_frame_feed(&p, &buf[i], 1);
    TEST_ASSERT_EQUAL_INT(1, s_cb_calls);
}

static void t_bad_checksum(void) {
    proto_frame_parser_t p;
    proto_frame_init(&p, on_frame, NULL);
    payload_chassis_cmd_t cmd = { 0,0,0 };
    uint8_t buf[64];
    int n = proto_frame_build(PROTO_FUNC_CHASSIS_CMD, (uint8_t*)&cmd, 12, buf, sizeof(buf));
    buf[n - 1] ^= 0xFF;  /* 破坏校验 */

    s_cb_calls = 0;
    proto_frame_feed(&p, buf, (uint32_t)n);
    TEST_ASSERT_EQUAL_INT(0, s_cb_calls);
    TEST_ASSERT_EQUAL_UINT(1, p.bad_cnt);
}

static void t_garbage_then_good(void) {
    proto_frame_parser_t p;
    proto_frame_init(&p, on_frame, NULL);
    /* 喂一段不会触发头序的纯垃圾，再发完整帧应能正常解析 */
    uint8_t junk[] = { 0x00, 0xFF, 0x12, 0x34, 0x00, 0x00 };
    proto_frame_feed(&p, junk, sizeof(junk));

    payload_chassis_cmd_t cmd = { 0,0,0 };
    uint8_t buf[64];
    int n = proto_frame_build(PROTO_FUNC_CHASSIS_CMD, (uint8_t*)&cmd, 12, buf, sizeof(buf));
    s_cb_calls = 0;
    proto_frame_feed(&p, buf, (uint32_t)n);
    TEST_ASSERT_EQUAL_INT(1, s_cb_calls);
}

static void t_oversize_len(void) {
    proto_frame_parser_t p;
    proto_frame_init(&p, on_frame, NULL);
    uint8_t bad[] = { 0x55, 0xAA, 0x10, 0xFF /* 超 PROTO_MAX_PAYLOAD */ };
    s_cb_calls = 0;
    proto_frame_feed(&p, bad, sizeof(bad));
    TEST_ASSERT_EQUAL_INT(0, s_cb_calls);
    TEST_ASSERT_EQUAL_UINT(1, p.overflow_cnt);
}

/* dispatch */
static int s_handler_calls;
static int handler_chassis(const uint8_t* payload, uint8_t len) {
    (void)payload;
    s_handler_calls++;
    return (int)len;
}

static void t_dispatch_known_and_unknown(void) {
    static const proto_entry_t tbl[] = {
        { PROTO_FUNC_CHASSIS_CMD, sizeof(payload_chassis_cmd_t), handler_chassis, "chassis" },
    };
    proto_dispatcher_t d;
    proto_dispatcher_init(&d, tbl, 1);

    /* 已知 */
    proto_frame_t f;
    f.func_id = PROTO_FUNC_CHASSIS_CMD;
    f.len = 12;
    memset(f.payload, 0, sizeof(f.payload));
    s_handler_calls = 0;
    proto_dispatch_on_frame(&f, &d);
    TEST_ASSERT_EQUAL_INT(1, s_handler_calls);
    TEST_ASSERT_EQUAL_UINT(1, d.hit_cnt);

    /* 未知 */
    f.func_id = 0x77;
    proto_dispatch_on_frame(&f, &d);
    TEST_ASSERT_EQUAL_UINT(1, d.miss_cnt);

    /* 长度不匹配 */
    f.func_id = PROTO_FUNC_CHASSIS_CMD;
    f.len = 11;
    proto_dispatch_on_frame(&f, &d);
    TEST_ASSERT_EQUAL_UINT(1, d.bad_len_cnt);
}

int main(void) {
    log_init();
    log_set_global_level(LOG_LVL_ERR);  /* 测试期间静音 */
    TU_RUN(t_build_and_parse_chassis_cmd);
    TU_RUN(t_byte_by_byte_feed);
    TU_RUN(t_bad_checksum);
    TU_RUN(t_garbage_then_good);
    TU_RUN(t_oversize_len);
    TU_RUN(t_dispatch_known_and_unknown);
    TU_MAIN_EPILOGUE();
}
