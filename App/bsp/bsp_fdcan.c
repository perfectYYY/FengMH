/*
 * bsp_fdcan.c — FDCAN BSP 层
 *
 * Host 端: mock TX 队列 + RX 注入
 * MCU  端: HAL FDCAN 收发绑定
 *   - init: 配置滤波器 + 启动 FDCAN + 注册 RX 中断
 *   - send: 构造 FDCAN_TxHeaderTypeDef → HAL_FDCAN_AddMessageToTxFifoQ
 *   - RX:   HAL_FDCAN_RxFifo0Callback → bsp_fdcan_on_rx → 用户回调
 */
#include "bsp_fdcan.h"
#include "bsp_time.h"
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

/* ─── MCU 端 HAL 句柄 ─── */
#if !APP_TARGET_HOST
#include "stm32h7xx_hal.h"
#include "fdcan.h"  /* hfdcan1 / hfdcan2 */

static FDCAN_HandleTypeDef* s_hfdcan[BSP_FDCAN_BUS_MAX] = {
    &hfdcan1, &hfdcan2
};
#endif

/* ─── 公共接口 ─── */

app_err_t bsp_fdcan_init(void) {
    memset(s_rx, 0, sizeof(s_rx));

#if APP_TARGET_HOST
    memset(s_txq, 0, sizeof(s_txq));
    memset(s_tx_head, 0, sizeof(s_tx_head));
    memset(s_tx_tail, 0, sizeof(s_tx_tail));
#else
    /* MCU: 配置滤波器 + 启动 FDCAN + 注册 RX 中断通知 */
    for (int i = 0; i < BSP_FDCAN_BUS_MAX; i++) {
        if (!s_hfdcan[i]) continue;

        FDCAN_FilterTypeDef filter;
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterIndex = 0;
        filter.FilterType = FDCAN_FILTER_MASK;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        /* Accept the DJI C620/M3508 control and feedback range: 0x200~0x20F. */
        filter.FilterID1 = 0x200;
        filter.FilterID2 = 0x7F0;

        HAL_FDCAN_ConfigGlobalFilter(s_hfdcan[i],
                                      FDCAN_REJECT, FDCAN_REJECT,
                                      FDCAN_REJECT_REMOTE,
                                      FDCAN_REJECT_REMOTE);
        HAL_FDCAN_ConfigFilter(s_hfdcan[i], &filter);
        HAL_FDCAN_Start(s_hfdcan[i]);
        HAL_FDCAN_ActivateNotification(s_hfdcan[i],
                                        FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                        0);
    }
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
    /* MCU: HAL FDCAN 发送 */
    if (!s_hfdcan[bus]) return APP_ERR_UNINIT;

    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier          = f->can_id;
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    /* 将 dlc 字节数映射到 FDCAN DLC 枚举 */
    tx_header.DataLength          = FDCAN_DLC_BYTES_8;  /* C620 固定 8 字节 */
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0;

    HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan[bus],
                                                           &tx_header,
                                                           f->data);
    if (ret != HAL_OK) {
        LOGE("fdcan%u HAL TX error %d", (unsigned)bus, (int)ret);
        return APP_ERR_IO;
    }
    return APP_OK;
#endif
}

/* MCU 端 HAL RX 回调桥接 */
#if !APP_TARGET_HOST
/*
 * 在 Core/Src/stm32h7xx_it.c 或 freertos.c 中:
 *   extern void bsp_fdcan_hal_rxfifo0_cb(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
 *   void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
 *       bsp_fdcan_hal_rxfifo0_cb(hfdcan, RxFifo0ITs);
 *   }
 */
void bsp_fdcan_hal_rxfifo0_cb(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;

    bsp_fdcan_bus_t bus = BSP_FDCAN_BUS_MAX;
    if      (hfdcan == &hfdcan1) bus = BSP_FDCAN_1;
    else if (hfdcan == &hfdcan2) bus = BSP_FDCAN_2;
    else return;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                    &rx_header, rx_data) != HAL_OK) {
            return;
        }
        if (rx_header.IdType != FDCAN_STANDARD_ID ||
            rx_header.RxFrameType != FDCAN_DATA_FRAME) {
            continue;
        }

        bsp_fdcan_frame_t frame;
        frame.can_id  = rx_header.Identifier;
        frame.dlc     = 8;
        memcpy(frame.data, rx_data, 8);
        frame.rx_tick = bsp_time_now_ms();

        if (s_rx[bus].cb) {
            s_rx[bus].cb(bus, &frame, s_rx[bus].user);
        }
    }
}
#endif /* !APP_TARGET_HOST */

/* ─── Host 单测辅助 ─── */

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
