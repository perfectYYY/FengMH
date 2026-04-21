/*
 * task_safety.c — M2 阶段：维护一个全局 estop 标志 + 简单事件计数
 */
#include "task_safety.h"
#include "log.h"
#include "config.h"
#include "motor_registry.h"

#if APP_TARGET_MCU
#include "cmsis_os.h"
#endif

static const char* TAG = "SAFETY";

static volatile bool s_estop = false;
static volatile uint32_t s_evt = 0;

bool task_safety_estop_active(void) { return s_estop; }
uint32_t task_safety_event_count(void) { return s_evt; }

void task_safety_estop_set(bool on) {
    if (on != s_estop) {
        s_estop = on;
        s_evt++;
        LOGW("estop=%d", (int)on);
        if (on) {
            for (uint32_t i = 0; i < motor_registry_count(); i++) {
                motor_dev_t* d = motor_get((motor_logical_id_t)i);
                if (d && d->ops && d->ops->disable) d->ops->disable(d);
            }
        }
    }
}

void task_safety_entry(void* arg) {
    (void)arg;
    LOGI("task_safety started");
#if APP_TARGET_MCU
    for (;;) {
        /* 占位：未来扫描 motor_state.last_rx_tick / temperature_c */
        osDelay(5);
    }
#endif
}
