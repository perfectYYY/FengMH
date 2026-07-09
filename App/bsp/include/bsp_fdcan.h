/*
 * bsp_fdcan.h — FDCAN 抽象接口
 *
 * 设计目的：
 *   - 所有上层（电机驱动）只调用 bsp_fdcan_send()/bsp_fdcan_attach_rx()
 *   - 板上：包 HAL_FDCAN_*；host 单测：内存回环，捕获发送帧
 */
#ifndef APP_BSP_FDCAN_H_
#define APP_BSP_FDCAN_H_

#include "types.h"
#include "err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_FDCAN_1 = 0,
    BSP_FDCAN_2 = 1,
    BSP_FDCAN_3 = 2,
    BSP_FDCAN_BUS_MAX
} bsp_fdcan_bus_t;

typedef struct {
    uint32_t can_id;     /* 标准帧 ID */
    uint8_t  dlc;        /* 字节数，0..8 */
    uint8_t  data[8];
    app_tick_t rx_tick;  /* 接收时戳 */
} bsp_fdcan_frame_t;

/* RX 回调：ISR/task 上下文都可能调，回调实现必须短 */
typedef void (*bsp_fdcan_rx_cb_t)(bsp_fdcan_bus_t bus,
                                  const bsp_fdcan_frame_t* f,
                                  void* user);

app_err_t bsp_fdcan_init(void);
app_err_t bsp_fdcan_attach_rx(bsp_fdcan_bus_t bus, bsp_fdcan_rx_cb_t cb, void* user);
app_err_t bsp_fdcan_send(bsp_fdcan_bus_t bus, const bsp_fdcan_frame_t* f);
uint32_t  bsp_fdcan_get_bus_err_cnt(bsp_fdcan_bus_t bus);

/* host 单测辅助 */
uint32_t  bsp_fdcan_test_tx_count(bsp_fdcan_bus_t bus);
app_err_t bsp_fdcan_test_pop_tx(bsp_fdcan_bus_t bus, bsp_fdcan_frame_t* out);
void      bsp_fdcan_test_inject_rx(bsp_fdcan_bus_t bus, const bsp_fdcan_frame_t* f);
void      bsp_fdcan_test_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BSP_FDCAN_H_ */
