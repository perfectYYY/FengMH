/*
 * bsp_usb_cdc.c — host mock；板上桥接到 USBD_CDC_HS（由 usbd_cdc_if.c 调用 hook）
 */
#include "bsp_usb_cdc.h"
#include "config.h"
#include "log.h"

#if APP_TARGET_MCU
#include "stm32h7xx_hal.h"
#endif

#include <string.h>

static const char* TAG = "USBCDC";

#define BSP_USB_TX_MAX 512

static bsp_usb_rx_cb_t s_rx_cb;
static void*           s_rx_user;

#if APP_TARGET_MCU
/* 由 usbd_cdc_if.c 提供，避免本 BSP 反向依赖 USB 中间层头文件 */
extern uint8_t CDC_Transmit_HS(uint8_t* buf, uint16_t len);

#define BSP_USB_TXQ_LEN       32U
#define BSP_USB_TX_FRAME_MAX  72U
typedef struct {
    uint8_t data[BSP_USB_TX_FRAME_MAX];
    uint16_t len;
} usb_tx_item_t;
static usb_tx_item_t s_txq[BSP_USB_TXQ_LEN];
static volatile uint8_t s_tx_head;
static volatile uint8_t s_tx_tail;
static volatile uint8_t s_tx_busy;

static uint32_t usb_enter_critical(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void usb_exit_critical(uint32_t primask) {
    if (!primask) __enable_irq();
}
#endif

#if APP_TARGET_HOST
static uint8_t  s_tx_buf[BSP_USB_TX_MAX];
static uint32_t s_tx_len;
#endif

app_err_t bsp_usb_cdc_init(void) {
    s_rx_cb   = 0;
    s_rx_user = 0;
#if APP_TARGET_HOST
    s_tx_len = 0;
#else
    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_tx_busy = 0U;
#endif
    LOGI("init ok (host=%d)", (int)APP_TARGET_HOST);
    return APP_OK;
}

app_err_t bsp_usb_cdc_attach_rx(bsp_usb_rx_cb_t cb, void* user) {
    s_rx_cb = cb; s_rx_user = user;
    return APP_OK;
}

app_err_t bsp_usb_cdc_send(const uint8_t* data, uint32_t len) {
    if (!data || len == 0) return APP_ERR_INVALID_ARG;
#if APP_TARGET_HOST
    if (s_tx_len + len > BSP_USB_TX_MAX) return APP_ERR_OVERFLOW;
    memcpy(s_tx_buf + s_tx_len, data, len);
    s_tx_len += len;
    return APP_OK;
#else
    if (len > BSP_USB_TX_FRAME_MAX) return APP_ERR_OVERFLOW;
    uint32_t primask = usb_enter_critical();
    uint8_t next = (uint8_t)((s_tx_head + 1U) % BSP_USB_TXQ_LEN);
    if (next == s_tx_tail) {
        usb_exit_critical(primask);
        return APP_ERR_OVERFLOW;
    }
    memcpy(s_txq[s_tx_head].data, data, len);
    s_txq[s_tx_head].len = (uint16_t)len;
    s_tx_head = next;
    usb_exit_critical(primask);
    bsp_usb_cdc_process();
    return APP_OK;
#endif
}

void bsp_usb_cdc_process(void) {
#if APP_TARGET_MCU
    uint32_t primask = usb_enter_critical();
    if (s_tx_busy || s_tx_tail == s_tx_head) {
        usb_exit_critical(primask);
        return;
    }
    uint8_t tail = s_tx_tail;
    s_tx_busy = 1U;
    usb_exit_critical(primask);

    if (CDC_Transmit_HS(s_txq[tail].data, s_txq[tail].len) != 0U) {
        primask = usb_enter_critical();
        s_tx_busy = 0U;
        usb_exit_critical(primask);
    }
#endif
}

void bsp_usb_cdc_on_tx_complete(void) {
#if APP_TARGET_MCU
    uint32_t primask = usb_enter_critical();
    if (s_tx_busy && s_tx_tail != s_tx_head) {
        s_tx_tail = (uint8_t)((s_tx_tail + 1U) % BSP_USB_TXQ_LEN);
    }
    s_tx_busy = 0U;
    usb_exit_critical(primask);
    bsp_usb_cdc_process();
#endif
}

/* usbd_cdc_if.c 在 CDC_Receive_HS 里调本函数，把字节灌进解析器 */
void bsp_usb_cdc_on_rx(const uint8_t* data, uint32_t len) {
    if (s_rx_cb && data && len) s_rx_cb(data, len, s_rx_user);
}

#if APP_TARGET_HOST
void bsp_usb_cdc_test_inject_rx(const uint8_t* data, uint32_t len) {
    bsp_usb_cdc_on_rx(data, len);
}
uint32_t bsp_usb_cdc_test_tx_size(void) { return s_tx_len; }
uint32_t bsp_usb_cdc_test_read_tx(uint8_t* out, uint32_t max_len) {
    if (!out) return 0;
    uint32_t n = (s_tx_len < max_len) ? s_tx_len : max_len;
    memcpy(out, s_tx_buf, n);
    return n;
}
void bsp_usb_cdc_test_reset(void) { s_tx_len = 0; }
#else
void bsp_usb_cdc_test_inject_rx(const uint8_t* d, uint32_t l) { (void)d; (void)l; }
uint32_t bsp_usb_cdc_test_tx_size(void) { return 0; }
uint32_t bsp_usb_cdc_test_read_tx(uint8_t* o, uint32_t m) { (void)o; (void)m; return 0; }
void bsp_usb_cdc_test_reset(void) {}
#endif
