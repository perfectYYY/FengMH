/*
 * proto_dispatch.c
 */
#include "proto_dispatch.h"
#include "log.h"
#include <string.h>

static const char* TAG = "PROTO";

void proto_dispatcher_init(proto_dispatcher_t* d, const proto_entry_t* tbl, uint32_t cnt) {
    if (!d) return;
    memset(d, 0, sizeof(*d));
    d->table = tbl;
    d->count = cnt;
}

void proto_dispatch_on_frame(const proto_frame_t* f, void* user) {
    proto_dispatcher_t* d = (proto_dispatcher_t*)user;
    if (!d || !f || !d->table) return;
    for (uint32_t i = 0; i < d->count; i++) {
        const proto_entry_t* e = &d->table[i];
        if (e->func_id != f->func_id) continue;
        if (e->expect_len != 0 && e->expect_len != f->len) {
            d->bad_len_cnt++;
            LOGW("func 0x%02X %s bad len: got=%u exp=%u",
                 f->func_id, e->name ? e->name : "-", f->len, e->expect_len);
            return;
        }
        if (e->handler) e->handler(f->payload, f->len);
        d->hit_cnt++;
        return;
    }
    d->miss_cnt++;
    LOGW("unknown func 0x%02X len=%u", f->func_id, f->len);
}
