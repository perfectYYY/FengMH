/*
 * proto_frame.c — 帧解析状态机实现
 *
 * 校验和约定：Byte0..Byte(4+Len-1) 累加低 8 位，即
 *   sum = (0x55 + 0xAA + func + len + payload[0..len-1]) & 0xFF
 */
#include "proto_frame.h"
#include "log.h"

#include <string.h>

static const char* TAG = "PROTO";

void proto_frame_init(proto_frame_parser_t* p, proto_frame_cb_t cb, void* user) {
    if (!p) return;
    memset(p, 0, sizeof(*p));
    p->cb = cb;
    p->user = user;
    p->state = PF_ST_IDLE;
}

void proto_frame_reset(proto_frame_parser_t* p) {
    if (!p) return;
    p->state = PF_ST_IDLE;
    p->payload_idx = 0;
    p->sum = 0;
    memset(&p->f, 0, sizeof(p->f));
}

static void fail_and_reset(proto_frame_parser_t* p) {
    p->bad_cnt++;
    proto_frame_reset(p);
}

void proto_frame_feed(proto_frame_parser_t* p, const uint8_t* data, uint32_t len) {
    if (!p || !data) return;
    for (uint32_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        switch (p->state) {
        case PF_ST_IDLE:
            if (b == PROTO_HEAD1) {
                p->sum = b;
                p->state = PF_ST_HEAD2;
            }
            break;
        case PF_ST_HEAD2:
            if (b == PROTO_HEAD2) {
                p->sum += b;
                p->state = PF_ST_FUNC;
            } else {
                /* 某些错位场景：新来的字节本身就是新 HEAD1 */
                if (b == PROTO_HEAD1) { p->sum = b; p->state = PF_ST_HEAD2; }
                else                  { fail_and_reset(p); }
            }
            break;
        case PF_ST_FUNC:
            p->f.func_id = b;
            p->sum += b;
            p->state = PF_ST_LEN;
            break;
        case PF_ST_LEN:
            if (b > PROTO_MAX_PAYLOAD) {
                LOGW("len too big: %u", (unsigned)b);
                p->overflow_cnt++;
                fail_and_reset(p);
                break;
            }
            p->f.len = b;
            p->sum += b;
            p->payload_idx = 0;
            p->state = (b == 0) ? PF_ST_CKSUM : PF_ST_PAYLOAD;
            break;
        case PF_ST_PAYLOAD:
            p->f.payload[p->payload_idx++] = b;
            p->sum += b;
            if (p->payload_idx >= p->f.len) {
                p->state = PF_ST_CKSUM;
            }
            break;
        case PF_ST_CKSUM:
            if (b == p->sum) {
                p->good_cnt++;
                if (p->cb) p->cb(&p->f, p->user);
            } else {
                LOGW("bad cksum func=0x%02X len=%u got=0x%02X exp=0x%02X",
                     p->f.func_id, p->f.len, b, p->sum);
                p->bad_cnt++;
            }
            proto_frame_reset(p);
            break;
        default:
            fail_and_reset(p);
            break;
        }
    }
}

int proto_frame_build(uint8_t func_id, const uint8_t* payload, uint8_t len,
                      uint8_t* out, uint32_t out_cap) {
    if (!out) return -1;
    if (len > 0 && !payload) return -1;
    if (len > PROTO_MAX_PAYLOAD) return -1;
    uint32_t total = 5u + (uint32_t)len;
    if (out_cap < total) return -1;
    out[0] = PROTO_HEAD1;
    out[1] = PROTO_HEAD2;
    out[2] = func_id;
    out[3] = len;
    uint8_t sum = (uint8_t)(PROTO_HEAD1 + PROTO_HEAD2 + func_id + len);
    for (uint8_t i = 0; i < len; i++) {
        out[4 + i] = payload[i];
        sum = (uint8_t)(sum + payload[i]);
    }
    out[4 + len] = sum;
    return (int)total;
}
