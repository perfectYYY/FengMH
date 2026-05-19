/*
 * app_tasks.c — 创建所有 app 任务（仅板上有效）
 */
#include "app_tasks.h"
#include "config.h"
#include "log.h"
#include "task_log.h"
#include "task_safety.h"
#include "task_comm.h"
#include "task_chassis.h"

#if APP_TARGET_MCU
#include "cmsis_os.h"

static const osThreadAttr_t s_attr_log     = { .name="t_log",     .stack_size=512,  .priority=osPriorityLow };
static const osThreadAttr_t s_attr_safety  = { .name="t_safety",  .stack_size=512,  .priority=osPriorityRealtime };
static const osThreadAttr_t s_attr_comm    = { .name="t_comm",    .stack_size=1024, .priority=osPriorityAboveNormal };
static const osThreadAttr_t s_attr_chassis = { .name="t_chassis", .stack_size=2048, .priority=osPriorityHigh };
#endif

static const char* TAG = "INIT";

void app_tasks_create(void) {
    LOGI("app_tasks_create start");

    task_comm_init();

    /* 业务初始化：注册 USB 解析器 + chassis 步态 */
    task_chassis_init();

#if APP_TARGET_MCU
    osThreadNew(task_log_entry,     NULL, &s_attr_log);
    osThreadNew(task_safety_entry,  NULL, &s_attr_safety);
    osThreadNew(task_comm_entry,    NULL, &s_attr_comm);
    osThreadNew(task_chassis_entry, NULL, &s_attr_chassis);
    LOGI("app_tasks_create: 4 tasks spawned");
#else
    LOGI("app_tasks_create: host build, no RTOS threads");
#endif
}
