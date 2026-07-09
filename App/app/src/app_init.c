/*
 * app_init.c - 汇总式初始化
 *
 * 初始化顺序：
 *   1. log + bsp (time, fdcan, uart, usb_cdc, spi)
 *   2. motor_registry
 *   3. GO/M3508 device instances
 *   4. BMI088, with degraded-mode fallback
 */
#include "app_init.h"
#include "config.h"
#include "log.h"
#include "bsp_time.h"
#include "bsp_fdcan.h"
#include "bsp_uart.h"
#include "bsp_usb_cdc.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "arm_pump.h"
#include "motor_registry.h"
#include "motor_go.h"
#include "motor_m3508.h"
#include "motor_damiao.h"
#include "imu_bmi088.h"

static const char* TAG = "INIT";

app_err_t app_init(void) {
    log_init();
    log_set_global_level(LOG_LVL_INFO);
    LOGI("FengMH app_init start");

    bsp_time_init();

    /* BSP 层初始化 */
    bsp_fdcan_init();
#if APP_CHASSIS_ENABLE
    bsp_uart_init(BSP_UART_2);
    bsp_uart_init(BSP_UART_3);
    bsp_uart_init(BSP_UART_4);
    bsp_uart_init(BSP_UART_7);
#endif
    bsp_usb_cdc_init();

#if APP_CHASSIS_ENABLE
    /* SPI 初始化 (BMI088) */
    bsp_spi_init(BSP_SPI_2);
#endif
    bsp_gpio_output_init(BSP_GPIO_ARM_PUMP_MAIN);

    /* 设备层初始化 */
    arm_pump_init();           /* Vacuum pump: default off */
    motor_registry_init();
#if APP_CHASSIS_ENABLE
    motor_go_init_all();       /* GO-8010: 创建实例 + 注册 UART RX + 绑定 registry */
    motor_m3508_init_all();    /* M3508:   创建实例 + 注册 FDCAN RX + 绑定 registry */
#endif
    motor_damiao_init_all();   /* Damiao:  机械臂 J1-J4, FDCAN3 */

#if APP_CHASSIS_ENABLE
    app_err_t err = imu_bmi088_init();
    if (err != APP_OK) {
        LOGW("BMI088 init failed: %d (steering disabled, robot runs in degraded mode)", (int)err);
    }
#else
    LOGI("arm-only debug: chassis devices disabled");
#endif

    LOGI("FengMH app_init done");
    return APP_OK;
}
