/*
 * proto_dispatch.h — FuncID 分发表
 */
#ifndef APP_SERVICE_PROTO_DISPATCH_H_
#define APP_SERVICE_PROTO_DISPATCH_H_

#include "types.h"
#include "err.h"
#include "proto_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*proto_handler_fn)(const uint8_t* payload, uint8_t len);

typedef struct {
    uint8_t          func_id;
    uint8_t          expect_len;  /* 0 表示不校验长度 */
    proto_handler_fn handler;
    const char*      name;
} proto_entry_t;

typedef struct {
    const proto_entry_t* table;
    uint32_t             count;
    uint32_t             hit_cnt;
    uint32_t             miss_cnt;     /* 未知 FuncID */
    uint32_t             bad_len_cnt;
} proto_dispatcher_t;

void      proto_dispatcher_init(proto_dispatcher_t* d,
                                const proto_entry_t* table, uint32_t count);
void      proto_dispatch_on_frame(const proto_frame_t* f, void* user /* = proto_dispatcher_t* */);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_PROTO_DISPATCH_H_ */
