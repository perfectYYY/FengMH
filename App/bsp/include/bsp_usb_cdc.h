/*
 * bsp_usb_cdc.h — 上位机 USB CDC 抽象（M1 阶段接口 + host mock）
 */
#ifndef APP_BSP_USB_CDC_H_
#define APP_BSP_USB_CDC_H_

#include "types.h"
#include "err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 收到字节时回调（host 下可由测试直接触发） */
typedef void (*bsp_usb_rx_cb_t)(const uint8_t* data, uint32_t len, void* user);

app_err_t bsp_usb_cdc_init(void);
app_err_t bsp_usb_cdc_attach_rx(bsp_usb_rx_cb_t cb, void* user);
app_err_t bsp_usb_cdc_send(const uint8_t* data, uint32_t len);
void      bsp_usb_cdc_process(void);
void      bsp_usb_cdc_on_tx_complete(void);

/* 板上 usbd_cdc_if.c 在收到字节时调用此 hook（host 也可调用以模拟） */
void      bsp_usb_cdc_on_rx(const uint8_t* data, uint32_t len);

/* host 测试辅助 */
void      bsp_usb_cdc_test_inject_rx(const uint8_t* data, uint32_t len);
uint32_t  bsp_usb_cdc_test_tx_size(void);
uint32_t  bsp_usb_cdc_test_read_tx(uint8_t* out, uint32_t max_len);
void      bsp_usb_cdc_test_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BSP_USB_CDC_H_ */
