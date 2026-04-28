/*
 * bsp_uart.c — UART/RS485 BSP 层
 *
 * MCU 端：
 *   - init: 启动 HAL_UARTEx_ReceiveToIdle_DMA 持续接收
 *   - send: RS485 DE 拉高 → HAL_UART_Transmit_DMA → TX完成回调 DE 拉低
 *           注意：USART2/3 已配置为 RS485 模式 (HAL_RS485Ex_Init)，
 *           DE 引脚由硬件自动控制，无需手动 GPIO 操作
 *   - RX:  HAL_UARTEx_RxEventCallback → bsp_uart_on_rx → 用户回调
 *
 * Host 端：
 *   - TX 队列 + RX 注入 mock，与 bsp_fdcan 保持一致风格
 */
#include "bsp_uart.h"
#include "config.h"
#include "log.h"

#include <string.h>

static const char* TAG = "UART";

/* ─── 内部数据结构 ─── */

typedef struct {
    bsp_uart_rx_cb_t cb;
    void*            user;
} uart_rx_slot_t;

static uart_rx_slot_t s_rx[BSP_UART_BUS_MAX];

#if APP_TARGET_HOST
/* host mock: TX 环形队列 */
#define BSP_UART_TXQ_LEN  256
#define BSP_UART_TXQ_BUF  64

typedef struct {
    uint8_t  data[BSP_UART_TXQ_BUF];
    uint32_t len;
} uart_tx_item_t;

static uart_tx_item_t s_txq[BSP_UART_BUS_MAX][BSP_UART_TXQ_LEN];
static uint32_t       s_tx_head[BSP_UART_BUS_MAX];
static uint32_t       s_tx_tail[BSP_UART_BUS_MAX];
#endif

/* ─── MCU 端 HAL 句柄 ─── */
#if !APP_TARGET_HOST
#include "stm32h7xx_hal.h"
#include "usart.h"   /* huart1/2/3 */

static UART_HandleTypeDef* s_huart[BSP_UART_BUS_MAX] = {
    &huart1, &huart2, &huart3
};

/* DMA 接收缓冲区：每条总线一个 16 字节缓冲 (GO-8010 反馈帧 16B) */
#define BSP_UART_RX_BUF_SIZE  32
static uint8_t s_rx_buf[BSP_UART_BUS_MAX][BSP_UART_RX_BUF_SIZE];
static volatile uint8_t s_tx_busy[BSP_UART_BUS_MAX];  /* DMA 发送中标志 */
#endif

/* ─── 公共接口 ─── */

app_err_t bsp_uart_init(bsp_uart_bus_t bus) {
    if (bus >= BSP_UART_BUS_MAX) return APP_ERR_INVALID_ARG;

    s_rx[bus].cb   = NULL;
    s_rx[bus].user = NULL;

#if APP_TARGET_HOST
    memset(s_txq[bus], 0, sizeof(s_txq[bus]));
    s_tx_head[bus] = 0;
    s_tx_tail[bus] = 0;
#else
    s_tx_busy[bus] = 0;
    /* 启动 DMA 接收 (IDLE 线检测 + DMA 半满/全满) */
    if (s_huart[bus]) {
        HAL_UARTEx_ReceiveToIdle_DMA(s_huart[bus],
                                      s_rx_buf[bus],
                                      BSP_UART_RX_BUF_SIZE);
        /* 关闭 DMA 半传输中断，只留全传输和 IDLE 中断 */
        __HAL_DMA_DISABLE_IT(s_huart[bus]->hdmarx, DMA_IT_HT);
    }
#endif

    LOGI("uart%u init ok (host=%d)", (unsigned)bus, (int)APP_TARGET_HOST);
    return APP_OK;
}

app_err_t bsp_uart_attach_rx(bsp_uart_bus_t bus, bsp_uart_rx_cb_t cb, void* user) {
    if (bus >= BSP_UART_BUS_MAX) return APP_ERR_INVALID_ARG;
    s_rx[bus].cb   = cb;
    s_rx[bus].user = user;
    return APP_OK;
}

app_err_t bsp_uart_send(bsp_uart_bus_t bus, const uint8_t* data, uint32_t len) {
    if (bus >= BSP_UART_BUS_MAX || !data || len == 0) return APP_ERR_INVALID_ARG;

#if APP_TARGET_HOST
    /* host mock: 入 TX 队列 */
    uint32_t head = s_tx_head[bus];
    uint32_t next = (head + 1U) % BSP_UART_TXQ_LEN;
    if (next == s_tx_tail[bus]) {
        LOGW("uart%u tx queue overflow", (unsigned)bus);
        return APP_ERR_OVERFLOW;
    }
    uint32_t copy_len = (len > BSP_UART_TXQ_BUF) ? BSP_UART_TXQ_BUF : len;
    memcpy(s_txq[bus][head].data, data, copy_len);
    s_txq[bus][head].len = copy_len;
    s_tx_head[bus] = next;
    return APP_OK;
#else
    /* MCU: RS485 DE 由 HAL RS485 模式硬件自动控制 */
    if (!s_huart[bus]) return APP_ERR_UNINIT;
    if (s_tx_busy[bus]) {
        LOGW("uart%u tx busy, drop", (unsigned)bus);
        return APP_ERR_BUSY;
    }
    s_tx_busy[bus] = 1;
    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(s_huart[bus], (uint8_t*)data, len);
    if (ret != HAL_OK) {
        s_tx_busy[bus] = 0;
        LOGE("uart%u HAL TX error %d", (unsigned)bus, (int)ret);
        return APP_ERR_IO;
    }
    return APP_OK;
#endif
}

void bsp_uart_on_rx(bsp_uart_bus_t bus, const uint8_t* data, uint32_t len) {
    if (bus >= BSP_UART_BUS_MAX) return;
    if (s_rx[bus].cb) {
        s_rx[bus].cb(bus, data, len, s_rx[bus].user);
    }
}

void bsp_uart_on_tx_done(bsp_uart_bus_t bus) {
    if (bus >= BSP_UART_BUS_MAX) return;
#if !APP_TARGET_HOST
    s_tx_busy[bus] = 0;
    /* 重新启动 DMA 接收 */
    if (s_huart[bus]) {
        HAL_UARTEx_ReceiveToIdle_DMA(s_huart[bus],
                                      s_rx_buf[bus],
                                      BSP_UART_RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(s_huart[bus]->hdmarx, DMA_IT_HT);
    }
#endif
}

/* ─── Host 单测辅助 ─── */

#if APP_TARGET_HOST
uint32_t bsp_uart_test_tx_count(bsp_uart_bus_t bus) {
    if (bus >= BSP_UART_BUS_MAX) return 0;
    uint32_t h = s_tx_head[bus], t = s_tx_tail[bus];
    return (h + BSP_UART_TXQ_LEN - t) % BSP_UART_TXQ_LEN;
}

app_err_t bsp_uart_test_pop_tx(bsp_uart_bus_t bus, uint8_t* out, uint32_t* out_len, uint32_t max_len) {
    if (bus >= BSP_UART_BUS_MAX || !out || !out_len) return APP_ERR_INVALID_ARG;
    if (s_tx_head[bus] == s_tx_tail[bus]) return APP_ERR_NOT_FOUND;
    uart_tx_item_t* item = &s_txq[bus][s_tx_tail[bus]];
    uint32_t copy_len = (item->len > max_len) ? max_len : item->len;
    memcpy(out, item->data, copy_len);
    *out_len = copy_len;
    s_tx_tail[bus] = (s_tx_tail[bus] + 1U) % BSP_UART_TXQ_LEN;
    return APP_OK;
}

void bsp_uart_test_inject_rx(bsp_uart_bus_t bus, const uint8_t* data, uint32_t len) {
    if (bus >= BSP_UART_BUS_MAX || !data) return;
    if (s_rx[bus].cb) s_rx[bus].cb(bus, data, len, s_rx[bus].user);
}

void bsp_uart_test_reset(void) {
    memset(s_txq, 0, sizeof(s_txq));
    memset(s_tx_head, 0, sizeof(s_tx_head));
    memset(s_tx_tail, 0, sizeof(s_tx_tail));
}
#else
uint32_t  bsp_uart_test_tx_count(bsp_uart_bus_t bus) { (void)bus; return 0; }
app_err_t bsp_uart_test_pop_tx(bsp_uart_bus_t bus, uint8_t* out, uint32_t* out_len, uint32_t max_len) {
    (void)bus; (void)out; (void)out_len; (void)max_len; return APP_ERR_UNSUPPORTED;
}
void bsp_uart_test_inject_rx(bsp_uart_bus_t bus, const uint8_t* data, uint32_t len) {
    (void)bus; (void)data; (void)len;
}
void bsp_uart_test_reset(void) {}
#endif

/* ─── MCU 端 HAL 回调桥接 ─── */
#if !APP_TARGET_HOST
/*
 * HAL_UARTEx_RxEventCallback — UART 接收事件回调 (IDLE / DMA 全满)
 * 在 stm32h7xx_it.c 或 freertos.c 中会被调用，需要在 app 层桥接。
 *
 * 使用方法：在 Core/Src/stm32h7xx_it.c 或 freertos.c 中:
 *   extern void bsp_uart_hal_rx_event(UART_HandleTypeDef *huart, uint16_t size);
 *   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
 *       bsp_uart_hal_rx_event(huart, size);
 *   }
 */
void bsp_uart_hal_rx_event(UART_HandleTypeDef *huart, uint16_t size) {
    bsp_uart_bus_t bus = BSP_UART_BUS_MAX;
    if      (huart == &huart1) bus = BSP_UART_1;
    else if (huart == &huart2) bus = BSP_UART_2;
    else if (huart == &huart3) bus = BSP_UART_3;
    else return;

    bsp_uart_on_rx(bus, s_rx_buf[bus], (uint32_t)size);

    /* 重新启动 DMA 接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(huart, s_rx_buf[bus], BSP_UART_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

/*
 * HAL_UART_TxCpltCallback — UART 发送完成回调
 * 使用方法：同上桥接
 */
void bsp_uart_hal_tx_done(UART_HandleTypeDef *huart) {
    bsp_uart_bus_t bus = BSP_UART_BUS_MAX;
    if      (huart == &huart1) bus = BSP_UART_1;
    else if (huart == &huart2) bus = BSP_UART_2;
    else if (huart == &huart3) bus = BSP_UART_3;
    else return;

    bsp_uart_on_tx_done(bus);
}
#endif
