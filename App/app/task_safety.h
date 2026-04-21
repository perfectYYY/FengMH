/*
 * task_safety.h — 监控电机超时 / 过温，发布软急停
 */
#ifndef APP_TASK_SAFETY_H_
#define APP_TASK_SAFETY_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>

void task_safety_entry(void* arg);

bool task_safety_estop_active(void);
void task_safety_estop_set(bool on);
uint32_t task_safety_event_count(void);
#ifdef __cplusplus
}
#endif
#endif
