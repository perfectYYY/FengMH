/*
 * bsp_spi.h — SPI 抽象接口
 *
 * 设计目的：
 *   - 设备层只调用 bsp_spi_transmit_receive()/bsp_spi_set_cs()
 *   - 板上：包 HAL_SPI + GPIO CS；host：可注入回调做单测/仿真
 */
#ifndef APP_BSP_SPI_H_
#define APP_BSP_SPI_H_

#include "types.h"
#include "err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_SPI_2 = 0,
    BSP_SPI_BUS_MAX
} bsp_spi_bus_t;

typedef enum {
    BSP_SPI_CS_BMI088_ACCEL = 0,
    BSP_SPI_CS_BMI088_GYRO,
    BSP_SPI_CS_MAX
} bsp_spi_cs_t;

typedef uint8_t (*bsp_spi_test_xfer_cb_t)(bsp_spi_bus_t bus, uint8_t tx, void* user);

app_err_t bsp_spi_init(bsp_spi_bus_t bus);
app_err_t bsp_spi_set_cs(bsp_spi_cs_t cs, uint8_t selected);
app_err_t bsp_spi_transmit_receive(bsp_spi_bus_t bus,
                                   const uint8_t* tx,
                                   uint8_t* rx,
                                   uint32_t len,
                                   uint32_t timeout_ms);
app_err_t bsp_spi_transfer_byte(bsp_spi_bus_t bus, uint8_t tx, uint8_t* rx);

/* Host 单测辅助 */
void bsp_spi_test_attach_xfer(bsp_spi_test_xfer_cb_t cb, void* user);
void bsp_spi_test_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BSP_SPI_H_ */
