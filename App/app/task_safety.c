/*
 * task_safety.c — 安全监控任务
 *
 * 功能：
 *   1. 全局 estop 标志 + 事件计数
 *   2. 电机超时检测：last_rx_tick > 100ms → 标记离线
 *   3. 电机温度保护：>80°C 限功率，>85°C 停机
 *   4. 200Hz 扫描周期
 */
#include "task_safety.h"
#include "log.h"
#include "config.h"
#include "motor_registry.h"
#include "bsp_time.h"

#if APP_TARGET_MCU
#include "cmsis_os.h"
#endif

static const char* TAG = "SAFETY";

static volatile bool s_estop = false;
static volatile uint32_t s_evt = 0;

/* 超时阈值 */
static uint32_t s_motor_timeout_ms = 100;   /* 100ms 无反馈视为离线 */
static float    s_temp_warn_c      = 80.0f; /* 温度警告阈值 */
static float    s_temp_stop_c      = 85.0f; /* 温度停机阈值 */

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
    LOGI("task_safety started (timeout=%lums, temp_stop=%.0fC)",
         (unsigned long)s_motor_timeout_ms, s_temp_stop_c);
#if APP_TARGET_MCU
    for (;;) {
        uint32_t now = (uint32_t)bsp_time_now_ms();
        for (uint32_t i = 0; i < motor_registry_count(); i++) {
            motor_dev_t* d = motor_get((motor_logical_id_t)i);
            if (!d) continue;

            /* 超时检测 */
            if (d->state.online && d->state.rx_cnt > 0) {
                if ((now - d->state.last_rx_tick) > s_motor_timeout_ms) {
                    d->state.online = 0;
                    LOGW("motor %u offline (no rx for %lums)",
                         (unsigned)i, (unsigned long)(now - d->state.last_rx_tick));
                }
            }

            /* 温度保护 */
            if (d->state.temperature_c > s_temp_stop_c) {
                if (d->ops && d->ops->disable) d->ops->disable(d);
                s_evt++;
                LOGE("motor %u overtemp %.1fC, disabled",
                     (unsigned)i, d->state.temperature_c);
            } else if (d->state.temperature_c > s_temp_warn_c) {
                /* 限功率由 motor_m3508 内部 temp_limit_phase 处理 */
                LOGW("motor %u temp %.1fC (warn threshold %.0fC)",
                     (unsigned)i, d->state.temperature_c, s_temp_warn_c);
            }
        }
        osDelay(5);  /* 200Hz */
    }
#endif
}
