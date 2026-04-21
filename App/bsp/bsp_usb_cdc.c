/*
 * bsp_usb_cdc.c — 板上接 USB Device CDC（M2 接入），M1 仅提供接口 + host mock
 */
#include "bsp_usb_cdc.h"
#include "config.h"
#include "log.h"

#include <string.h>

static const char* TAG = "USBCDC";

#define BSP_USB_TX_MAX 512

static bsp_usb_rx_cb_t s_rx_cb;
static void*           s_rx_user;

#if APP_TARGET_HOST
static uint8_t  s_tx_buf[BSP_USB_TX_MAX];
static uint32_t s_tx_len;
#endif

app_err_t bsp_usb_cdc_init(void) {
    s_rx_cb   = 0;
    s_rx_user = 0;
#if APP_TARGET_HOST
    s_tx_len = 0;
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
    return APP_ERR_UNSUPPORTED;  /* M2 起接 USBD_CDC */
#endif
}

#if APP_TARGET_HOST
void bsp_usb_cdc_test_inject_rx(const uint8_t* data, uint32_t len) {
    if (s_rx_cb && data && len) s_rx_cb(data, len, s_rx_user);
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
