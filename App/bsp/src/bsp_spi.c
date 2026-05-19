/*
 * bsp_spi.c — SPI BSP 层
 *
 * MCU 端使用阻塞式 HAL_SPI_TransmitReceive。BMI088 每次访问只有几个字节，
 * 不占用现有 UART/FDCAN DMA 资源，调试也更直接。
 */
#include "bsp_spi.h"
#include "config.h"
#include "log.h"

#include <string.h>

static const char* TAG = "SPI";

#if APP_TARGET_HOST
static bsp_spi_test_xfer_cb_t s_test_xfer;
static void* s_test_user;
static uint8_t s_cs_selected[BSP_SPI_CS_MAX];
#else
#include "stm32h7xx_hal.h"
#include "spi.h"

static SPI_HandleTypeDef* s_hspi[BSP_SPI_BUS_MAX] = {
    &hspi2
};
#endif

app_err_t bsp_spi_init(bsp_spi_bus_t bus) {
    if (bus >= BSP_SPI_BUS_MAX) return APP_ERR_INVALID_ARG;
#if APP_TARGET_HOST
    memset(s_cs_selected, 0, sizeof(s_cs_selected));
#else
    if (!s_hspi[bus]) return APP_ERR_UNINIT;
#endif
    LOGI("spi%u init ok (host=%d)", (unsigned)bus, (int)APP_TARGET_HOST);
    return APP_OK;
}

app_err_t bsp_spi_set_cs(bsp_spi_cs_t cs, uint8_t selected) {
    if (cs >= BSP_SPI_CS_MAX) return APP_ERR_INVALID_ARG;

#if APP_TARGET_HOST
    s_cs_selected[cs] = selected ? 1U : 0U;
    return APP_OK;
#else
    GPIO_PinState pin_state = selected ? GPIO_PIN_RESET : GPIO_PIN_SET;
    switch (cs) {
        case BSP_SPI_CS_BMI088_ACCEL:
            HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, pin_state);
            return APP_OK;
        case BSP_SPI_CS_BMI088_GYRO:
            HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, pin_state);
            return APP_OK;
        default:
            return APP_ERR_INVALID_ARG;
    }
#endif
}

app_err_t bsp_spi_transmit_receive(bsp_spi_bus_t bus,
                                   const uint8_t* tx,
                                   uint8_t* rx,
                                   uint32_t len,
                                   uint32_t timeout_ms) {
    if (bus >= BSP_SPI_BUS_MAX || !tx || !rx || len == 0U) return APP_ERR_INVALID_ARG;

#if APP_TARGET_HOST
    for (uint32_t i = 0; i < len; i++) {
        rx[i] = s_test_xfer ? s_test_xfer(bus, tx[i], s_test_user) : 0xFFU;
    }
    return APP_OK;
#else
    if (!s_hspi[bus]) return APP_ERR_UNINIT;
    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(s_hspi[bus],
                                                     (uint8_t*)tx,
                                                     rx,
                                                     (uint16_t)len,
                                                     timeout_ms);
    return (ret == HAL_OK) ? APP_OK : APP_ERR_IO;
#endif
}

app_err_t bsp_spi_transfer_byte(bsp_spi_bus_t bus, uint8_t tx, uint8_t* rx) {
    if (!rx) return APP_ERR_INVALID_ARG;
    uint8_t r = 0U;
    app_err_t err = bsp_spi_transmit_receive(bus, &tx, &r, 1U, 100U);
    *rx = r;
    return err;
}

void bsp_spi_test_attach_xfer(bsp_spi_test_xfer_cb_t cb, void* user) {
#if APP_TARGET_HOST
    s_test_xfer = cb;
    s_test_user = user;
#else
    (void)cb;
    (void)user;
#endif
}

void bsp_spi_test_reset(void) {
#if APP_TARGET_HOST
    s_test_xfer = NULL;
    s_test_user = NULL;
    memset(s_cs_selected, 0, sizeof(s_cs_selected));
#endif
}
