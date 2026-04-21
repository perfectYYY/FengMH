/*
 * proto_frame.h — 字节流 → 完整帧解析状态机
 *
 * 不处理 FuncID 语义，只负责完整帧切分 + 校验。
 */
#ifndef APP_SERVICE_PROTO_FRAME_H_
#define APP_SERVICE_PROTO_FRAME_H_

#include "types.h"
#include "err.h"
#include "proto_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PF_ST_IDLE = 0,
    PF_ST_HEAD2,
    PF_ST_FUNC,
    PF_ST_LEN,
    PF_ST_PAYLOAD,
    PF_ST_CKSUM
} proto_frame_state_t;

typedef struct {
    uint8_t  func_id;
    uint8_t  len;
    uint8_t  payload[PROTO_MAX_PAYLOAD];
} proto_frame_t;

/* 完整帧回调（解析器保证 checksum 已通过） */
typedef void (*proto_frame_cb_t)(const proto_frame_t* frame, void* user);

typedef struct {
    proto_frame_state_t state;
    proto_frame_t       f;
    uint8_t             payload_idx;
    uint8_t             sum;  /* 累加校验的在途值 */

    uint32_t            good_cnt;
    uint32_t            bad_cnt;
    uint32_t            overflow_cnt;

    proto_frame_cb_t    cb;
    void*               user;
} proto_frame_parser_t;

void       proto_frame_init(proto_frame_parser_t* p, proto_frame_cb_t cb, void* user);
void       proto_frame_reset(proto_frame_parser_t* p);
void       proto_frame_feed(proto_frame_parser_t* p, const uint8_t* data, uint32_t len);

/* 构造帧到 out（out 必须 >= 5+len），返回总长度；失败返回 <0 */
int        proto_frame_build(uint8_t func_id, const uint8_t* payload, uint8_t len,
                             uint8_t* out, uint32_t out_cap);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_PROTO_FRAME_H_ */
