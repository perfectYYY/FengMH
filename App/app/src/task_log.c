/*
 * task_log.c — 暂时只是 1Hz 心跳；后续接 lock-free ring 消费
 */
#include "task_log.h"
#include "log.h"
#include "config.h"

#if APP_TARGET_MCU
#include "cmsis_os.h"
#endif

static const char* TAG = "LOG";

void task_log_entry(void* arg) {
    (void)arg;
    LOGI("task_log started");
#if APP_TARGET_MCU
    for (;;) {
        LOGT("hb");
        osDelay(1000);
    }
#endif
}
