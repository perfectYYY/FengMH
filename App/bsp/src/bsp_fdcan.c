/*
 * bsp_fdcan.c — FDCAN BSP 层
 *
 * Host 端: mock TX 队列 + RX 注入
 * MCU  端: HAL FDCAN 收发绑定
 *   - init: 配置滤波器 + 启动 FDCAN + 注册 RX 中断
 *   - send: 等 TX FIFO 有空位 → 入队 → 短等待该 slot 释放，避免周期控制帧堆积
 *   - RX:   HAL_FDCAN_RxFifo0Callback → bsp_fdcan_on_rx → 用户回调
 */
#include "bsp_fdcan.h"
#include "bsp_time.h"
#include "config.h"
#include "log.h"

#include <string.h>

static const char* TAG = "FDCAN";

#define BSP_FDCAN_TXQ_LEN 64
#define BSP_FDCAN_TX_ROOM_TIMEOUT_US   300U
#define BSP_FDCAN_TX_DRAIN_TIMEOUT_US  300U
#define BSP_FDCAN_TX_ABORT_TIMEOUT_US  100U
#define BSP_FDCAN_WARN_INTERVAL_MS     100U

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

/* 每条总线的累计错误次数（TX 入队失败 + Bus Off + Error Passive / Warning）
 * 非零说明 CAN 总线有问题，可通过 bsp_fdcan_get_bus_err_cnt() 读取 */
static volatile uint32_t s_bus_err_cnt[BSP_FDCAN_BUS_MAX];
static uint32_t s_last_tx_warn_ms[BSP_FDCAN_BUS_MAX];

#define BSP_FDCAN_TX_FIFO_ABORT_MASK \
    (FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2 | FDCAN_TX_BUFFER3)

static void fdcan_dwt_enable(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t fdcan_cycles_per_us(void) {
    uint32_t hz = SystemCoreClock ? SystemCoreClock : HAL_RCC_GetHCLKFreq();
    uint32_t cycles = hz / 1000000U;
    return cycles ? cycles : 1U;
}

static int fdcan_wait_free_level(FDCAN_HandleTypeDef* hfdcan,
                                  uint32_t min_free,
                                  uint32_t timeout_us) {
    fdcan_dwt_enable();
    uint32_t start = DWT->CYCCNT;
    uint32_t timeout_cycles = timeout_us * fdcan_cycles_per_us();

    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) < min_free) {
        if ((uint32_t)(DWT->CYCCNT - start) >= timeout_cycles) {
            return 0;
        }
    }
    return 1;
}

static void fdcan_warn_throttled(bsp_fdcan_bus_t bus, const char* msg,
                                 uint32_t a, uint32_t b) {
    uint32_t now = (uint32_t)bsp_time_now_ms();
    if (s_last_tx_warn_ms[bus] == 0U ||
        (now - s_last_tx_warn_ms[bus]) >= BSP_FDCAN_WARN_INTERVAL_MS) {
        s_last_tx_warn_ms[bus] = now;
        LOGW("fdcan%u %s a=%lu b=%lu err=%lu",
             (unsigned)bus,
             msg ? msg : "tx warn",
             (unsigned long)a,
             (unsigned long)b,
             (unsigned long)s_bus_err_cnt[bus]);
    }
}
#endif

/* ─── 公共接口 ─── */

app_err_t bsp_fdcan_init(void) {
    memset(s_rx, 0, sizeof(s_rx));

#if APP_TARGET_HOST
    memset(s_txq, 0, sizeof(s_txq));
    memset(s_tx_head, 0, sizeof(s_tx_head));
    memset(s_tx_tail, 0, sizeof(s_tx_tail));
#else
    memset((void*)s_bus_err_cnt, 0, sizeof(s_bus_err_cnt));
    memset(s_last_tx_warn_ms, 0, sizeof(s_last_tx_warn_ms));

    /* MCU: 配置滤波器 + 启动 FDCAN + 注册 RX/错误中断通知 */
    for (int i = 0; i < BSP_FDCAN_BUS_MAX; i++) {
        if (!s_hfdcan[i]) continue;

        FDCAN_FilterTypeDef filter;
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterIndex = 0;
        filter.FilterType = FDCAN_FILTER_MASK;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        /* 接收 0x200~0x20F：(ID & 0x7F0) == 0x200，覆盖 M3508 反馈 0x201~0x208 */
        filter.FilterID1 = 0x200;
        filter.FilterID2 = 0x7F0;

        HAL_FDCAN_ConfigGlobalFilter(s_hfdcan[i],
                                      FDCAN_REJECT, FDCAN_REJECT,
                                      FDCAN_REJECT_REMOTE,
                                      FDCAN_REJECT_REMOTE);
        HAL_FDCAN_ConfigFilter(s_hfdcan[i], &filter);

        /* App 层强制覆盖：启用自动重传（CubeMX 默认 DISABLE）。
         * C620 上电需要约 200ms 才能应答 ACK；DISABLE 时每次丢帧让 TEC +8，
         * 约 32 帧（64ms @500Hz）就会进入 Bus Off，整条总线锁死且静默无声。
         * 在 HAL_FDCAN_Start 之前重新 Init 即可生效（控制器仍处于 Init 模式）。 */
        s_hfdcan[i]->Init.AutoRetransmission = ENABLE;
        HAL_FDCAN_Init(s_hfdcan[i]);

        HAL_FDCAN_Start(s_hfdcan[i]);
        HAL_FDCAN_ActivateNotification(s_hfdcan[i],
                                        FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                        FDCAN_IT_ERROR_WARNING          |
                                        FDCAN_IT_ERROR_PASSIVE          |
                                        FDCAN_IT_BUS_OFF,
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

    /*
     * 周期控制帧宁可短暂等待，也不要无脑塞 TX FIFO。
     * 如果上一帧还没被仲裁/ACK 释放，这里会等一个 CAN 数据帧量级的窗口；
     * 超时会返回 BUSY 并计数，便于上板区分"没发出去"和"电机未回包"。
     */
    if (!fdcan_wait_free_level(s_hfdcan[bus], 1U, BSP_FDCAN_TX_ROOM_TIMEOUT_US)) {
        s_bus_err_cnt[bus]++;
        (void)HAL_FDCAN_AbortTxRequest(s_hfdcan[bus], BSP_FDCAN_TX_FIFO_ABORT_MASK);
        (void)fdcan_wait_free_level(s_hfdcan[bus], 1U, BSP_FDCAN_TX_ABORT_TIMEOUT_US);
        fdcan_warn_throttled(bus, "tx fifo full before enqueue",
                             HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan[bus]),
                             f->can_id);
        return APP_ERR_BUSY;
    }

    uint32_t free_before = HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan[bus]);

    HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan[bus],
                                                           &tx_header,
                                                           f->data);
    uint32_t req_buf = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(s_hfdcan[bus]);
    if (ret != HAL_OK) {
        s_bus_err_cnt[bus]++;
        fdcan_warn_throttled(bus, "tx enqueue failed",
                             (uint32_t)ret,
                             HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan[bus]));
        return APP_ERR_IO;
    }

    if (!fdcan_wait_free_level(s_hfdcan[bus], free_before, BSP_FDCAN_TX_DRAIN_TIMEOUT_US)) {
        s_bus_err_cnt[bus]++;
        if (req_buf != 0U) {
            (void)HAL_FDCAN_AbortTxRequest(s_hfdcan[bus], req_buf);
            (void)fdcan_wait_free_level(s_hfdcan[bus], free_before, BSP_FDCAN_TX_ABORT_TIMEOUT_US);
        }
        fdcan_warn_throttled(bus, "tx fifo not drained",
                             HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan[bus]),
                             f->can_id);
        return APP_ERR_BUSY;
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

/*
 * HAL_FDCAN_ErrorStatusCallback 桥接：监控总线错误状态 + 自动 Bus Off 恢复
 *
 * 使用方法：在 Core/Src/main.c USER CODE BEGIN 4 中:
 *   extern void bsp_fdcan_hal_error_cb(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);
 *   void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
 *       bsp_fdcan_hal_error_cb(hfdcan, ErrorStatusITs);
 *   }
 *
 * Bus Off 恢复原理：Bus Off 时硬件自动置 CCCR.INIT=1 锁死总线。
 * 调用 HAL_FDCAN_Start 清 INIT 位，触发 128×11 个隐性位的恢复序列，
 * 之后控制器自动回到 Error Active 状态，无需任何额外操作。
 */
void bsp_fdcan_hal_error_cb(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
    bsp_fdcan_bus_t bus = BSP_FDCAN_BUS_MAX;
    if      (hfdcan == &hfdcan1) bus = BSP_FDCAN_1;
    else if (hfdcan == &hfdcan2) bus = BSP_FDCAN_2;
    else return;

    s_bus_err_cnt[bus]++;

    if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
        /* TEC 溢出超过 255，整条总线锁死，必须立即恢复 */
        LOGE("FDCAN%u BUS_OFF (cnt=%lu), recovering...",
             (unsigned)bus, (unsigned long)s_bus_err_cnt[bus]);
        HAL_FDCAN_Start(hfdcan);  /* 清 CCCR.INIT，触发 bus-off recovery 序列 */
        /* 恢复后重新注册所有通知（Start 内部会清通知状态） */
        HAL_FDCAN_ActivateNotification(hfdcan,
                                        FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                        FDCAN_IT_ERROR_WARNING          |
                                        FDCAN_IT_ERROR_PASSIVE          |
                                        FDCAN_IT_BUS_OFF,
                                        0);
    } else if (ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) {
        LOGW("FDCAN%u error-passive (TEC/REC>=128, cnt=%lu)",
             (unsigned)bus, (unsigned long)s_bus_err_cnt[bus]);
    } else if (ErrorStatusITs & FDCAN_IT_ERROR_WARNING) {
        LOGW("FDCAN%u error-warning (TEC/REC>=96, cnt=%lu)",
             (unsigned)bus, (unsigned long)s_bus_err_cnt[bus]);
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
