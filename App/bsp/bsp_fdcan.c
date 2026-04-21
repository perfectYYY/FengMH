/*
 * bsp_fdcan.c — host mock + 板上 HAL 的最小壳层（M1 阶段）
 *
 * 本版只保证 host mock 可用；板上 HAL 绑定留到 M1 末期接入，避开
 * 与旧 Core/Src/M3508.c 的 FDCAN 初始化冲突（USE_LEGACY=1 时旧路径仍工作）。
 */
#include "bsp_fdcan.h"
#include "config.h"
#include "log.h"

#include <string.h>

static const char* TAG = "FDCAN";

#define BSP_FDCAN_TXQ_LEN 64

typedef struct {
    bsp_fdcan_rx_cb_t cb;
    void*             user;
} rx_slot_t;

static rx_slot_t s_rx[BSP_FDCAN_BUS_MAX];

#if APP_TARGET_HOST
static bsp_fdcan_frame_t s_txq[BSP_FDCAN_BUS_MAX][BSP_FDCAN_TXQ_LEN];
static uint32_t          s_tx_head[BSP_FDCAN_BUS_MAX];
static uint32_t          s_tx_tail[BSP_FDCAN_BUS_MAX];
#endif

app_err_t bsp_fdcan_init(void) {
    memset(s_rx, 0, sizeof(s_rx));
#if APP_TARGET_HOST
    memset(s_txq, 0, sizeof(s_txq));
    memset(s_tx_head, 0, sizeof(s_tx_head));
    memset(s_tx_tail, 0, sizeof(s_tx_tail));
#endif
    LOGI("init ok (host=%d)", (int)APP_TARGET_HOST);
    return APP_OK;
}

app_err_t bsp_fdcan_attach_rx(bsp_fdcan_bus_t bus, bsp_fdcan_rx_cb_t cb, void* user) {
    if (bus >= BSP_FDCAN_BUS_MAX) return APP_ERR_INVALID_ARG;
    s_rx[bus].cb   = cb;
    s_rx[bus].user = user;
    return APP_OK;
}

app_err_t bsp_fdcan_send(bsp_fdcan_bus_t bus, const bsp_fdcan_frame_t* f) {
    if (bus >= BSP_FDCAN_BUS_MAX || !f || f->dlc > 8) return APP_ERR_INVALID_ARG;

#if APP_TARGET_HOST
    uint32_t head = s_tx_head[bus];
    uint32_t next = (head + 1U) % BSP_FDCAN_TXQ_LEN;
    if (next == s_tx_tail[bus]) {
        LOGW("bus%u tx queue overflow", (unsigned)bus);
        return APP_ERR_OVERFLOW;
    }
    s_txq[bus][head] = *f;
    s_tx_head[bus] = next;
    return APP_OK;
#else
    /* 板上 HAL 绑定留待后续接入（保持 USE_LEGACY=1 时旧路径不受影响） */
    (void)f;
    return APP_ERR_UNSUPPORTED;
#endif
}

#if APP_TARGET_HOST
uint32_t bsp_fdcan_test_tx_count(bsp_fdcan_bus_t bus) {
    if (bus >= BSP_FDCAN_BUS_MAX) return 0;
    uint32_t h = s_tx_head[bus], t = s_tx_tail[bus];
    return (h + BSP_FDCAN_TXQ_LEN - t) % BSP_FDCAN_TXQ_LEN;
}

app_err_t bsp_fdcan_test_pop_tx(bsp_fdcan_bus_t bus, bsp_fdcan_frame_t* out) {
    if (bus >= BSP_FDCAN_BUS_MAX || !out) return APP_ERR_INVALID_ARG;
    if (s_tx_head[bus] == s_tx_tail[bus]) return APP_ERR_NOT_FOUND;
    *out = s_txq[bus][s_tx_tail[bus]];
    s_tx_tail[bus] = (s_tx_tail[bus] + 1U) % BSP_FDCAN_TXQ_LEN;
    return APP_OK;
}

void bsp_fdcan_test_inject_rx(bsp_fdcan_bus_t bus, const bsp_fdcan_frame_t* f) {
    if (bus >= BSP_FDCAN_BUS_MAX || !f) return;
    if (s_rx[bus].cb) s_rx[bus].cb(bus, f, s_rx[bus].user);
}

void bsp_fdcan_test_reset(void) {
    memset(s_txq, 0, sizeof(s_txq));
    memset(s_tx_head, 0, sizeof(s_tx_head));
    memset(s_tx_tail, 0, sizeof(s_tx_tail));
}
#else
uint32_t  bsp_fdcan_test_tx_count(bsp_fdcan_bus_t bus) { (void)bus; return 0; }
app_err_t bsp_fdcan_test_pop_tx(bsp_fdcan_bus_t bus, bsp_fdcan_frame_t* out) {
    (void)bus; (void)out; return APP_ERR_UNSUPPORTED;
}
void bsp_fdcan_test_inject_rx(bsp_fdcan_bus_t bus, const bsp_fdcan_frame_t* f) {
    (void)bus; (void)f;
}
void bsp_fdcan_test_reset(void) {}
#endif
