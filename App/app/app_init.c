/*
 * app_init.c — 汇总式初始化
 */
#include "app_init.h"
#include "log.h"
#include "bsp_time.h"
#include "bsp_fdcan.h"
#include "bsp_usb_cdc.h"
#include "motor_registry.h"

static const char* TAG = "INIT";

app_err_t app_init(void) {
    log_init();
    log_set_global_level(LOG_LVL_INFO);
    LOGI("FengMH app_init start");

    bsp_time_init();
    bsp_fdcan_init();
    bsp_usb_cdc_init();
    motor_registry_init();

    LOGI("FengMH app_init done");
    return APP_OK;
}

app_err_t app_init_bring_up(void) {
    /* M1 阶段的 bring-up：只初始化不依赖 HAL 句柄的 App 层，
       旧的 PID_M3508_CAN_Init()/motor_instance 初始化仍由 main.c 调用。 */
    return app_init();
}
