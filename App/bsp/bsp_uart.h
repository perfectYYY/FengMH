/*
 * bsp_uart.h — UART/RS485 抽象接口
 *
 * 设计目的：
 *   - 所有上层（GO-8010 电机驱动）只调用 bsp_uart_send()/bsp_uart_attach_rx()
 *   - 板上：包 HAL_UART + RS485 DE 控制 + DMA；host 单测：内存回环
 *
 * USART1: 调试串口 115200 (暂不使用)
 * USART2: RS485-1 (GO 电机腿1/2), 4Mbps, DE=PD4
 * USART3: RS485-2 (GO 电机腿3/4), 4Mbps, DE=PB14
 */
#ifndef APP_BSP_UART_H_
#define APP_BSP_UART_H_

#include "types.h"
#include "err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_UART_1 = 0,   /* 调试串口 115200 */
    BSP_UART_2 = 1,   /* RS485-1 (GO 电机) 4Mbps, DE=PD4 */
    BSP_UART_3 = 2,   /* RS485-2 (GO 电机) 4Mbps, DE=PB14 */
    BSP_UART_BUS_MAX
} bsp_uart_bus_t;

/* RX 回调：ISR/task 上下文都可能调，回调实现必须短 */
typedef void (*bsp_uart_rx_cb_t)(bsp_uart_bus_t bus,
                                  const uint8_t* data,
                                  uint32_t len,
                                  void* user);

/* 初始化指定 UART 总线 (MCU: HAL 句柄+DMA, Host: mock) */
app_err_t bsp_uart_init(bsp_uart_bus_t bus);

/* 注册 RX 回调 */
app_err_t bsp_uart_attach_rx(bsp_uart_bus_t bus, bsp_uart_rx_cb_t cb, void* user);

/* 发送数据 (MCU: RS485 DE 拉高 → HAL_UART_Transmit_DMA → TX完成回调 DE 拉低; Host: mock) */
app_err_t bsp_uart_send(bsp_uart_bus_t bus, const uint8_t* data, uint32_t len);

/* MCU 端 HAL 回调桥接：HAL_UART_RxCpltCallback / HAL_UART_TxCpltCallback 中调用 */
void bsp_uart_on_rx(bsp_uart_bus_t bus, const uint8_t* data, uint32_t len);
void bsp_uart_on_tx_done(bsp_uart_bus_t bus);

/* MCU 端：HAL 回调直接桥接（UART_HandleTypeDef → bus 自动映射）*/
#if !APP_TARGET_HOST
#include "stm32h7xx_hal.h"
void bsp_uart_hal_rx_event(UART_HandleTypeDef *huart, uint16_t size);
void bsp_uart_hal_tx_done(UART_HandleTypeDef *huart);
#endif

/* Host 单测辅助 */
uint32_t  bsp_uart_test_tx_count(bsp_uart_bus_t bus);
app_err_t bsp_uart_test_pop_tx(bsp_uart_bus_t bus, uint8_t* out, uint32_t* out_len, uint32_t max_len);
void      bsp_uart_test_inject_rx(bsp_uart_bus_t bus, const uint8_t* data, uint32_t len);
void      bsp_uart_test_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BSP_UART_H_ */
