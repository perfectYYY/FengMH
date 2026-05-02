/*
 * app_init.c — 汇总式初始化
 *
 * 初始化顺序：
 *   1. log + bsp (time, fdcan, uart, usb_cdc)
 *   2. motor_registry (空表)
 *   3. motor_go_init_all (创建 GO 实例，注册 UART RX 回调，绑定到 registry)
 *   4. motor_m3508_init_all (创建 M3508 实例，注册 FDCAN RX 回调，绑定到 registry)
 */
#include "app_init.h"
#include "log.h"
#include "bsp_time.h"
#include "bsp_fdcan.h"
#include "bsp_uart.h"
#include "bsp_usb_cdc.h"
#include "bsp_spi.h"
#include "motor_registry.h"
#include "motor_go.h"
#include "motor_m3508.h"
#include "imu_bmi088.h"

static const char* TAG = "INIT";

app_err_t app_init(void) {
    log_init();
    log_set_global_level(LOG_LVL_INFO);
    LOGI("FengMH app_init start");

    /* BSP 层初始化 */
    bsp_time_init();
    bsp_fdcan_init();
    bsp_uart_init(BSP_UART_2);
    bsp_uart_init(BSP_UART_3);
    bsp_usb_cdc_init();

    /* SPI 初始化 (BMI088) */
    bsp_spi_init(BSP_SPI_2);

    /* 设备层初始化 */
    motor_registry_init();
    motor_go_init_all();       /* GO-8010: 创建实例 + 注册 UART RX + 绑定 registry */
    motor_m3508_init_all();    /* M3508:   创建实例 + 注册 FDCAN RX + 绑定 registry */

    /* IMU 初始化 (不阻塞, 失败时降级运行) */
    {
        app_err_t err = imu_bmi088_init();
        if (err != APP_OK) {
            LOGW("BMI088 init failed: %d (steering disabled, robot runs in degraded mode)", (int)err);
        }
    }

    LOGI("FengMH app_init done");
    return APP_OK;
}

app_err_t app_init_bring_up(void) {

    return app_init();
}
