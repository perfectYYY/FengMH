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
#include "task_arm.h"

#if APP_TARGET_MCU
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

/* CMSIS-RTOS2 的 stack_size 单位是字节。log_emit() 自身有 256B
 * 格式化缓冲，newlib 的 vsnprintf() 还会继续使用调用者栈。 */
static const osThreadAttr_t s_attr_log     = { .name="t_log",     .stack_size=2048, .priority=osPriorityLow };
static const osThreadAttr_t s_attr_safety  = { .name="t_safety",  .stack_size=2048, .priority=osPriorityRealtime };
static const osThreadAttr_t s_attr_comm    = { .name="t_comm",    .stack_size=2048, .priority=osPriorityAboveNormal };
#if APP_CHASSIS_ENABLE
static const osThreadAttr_t s_attr_chassis = { .name="t_chassis", .stack_size=4096, .priority=osPriorityHigh };
#endif
static const osThreadAttr_t s_attr_arm     = { .name="t_arm",     .stack_size=2048, .priority=osPriorityHigh };

/* 现场表达式：非零时分别表示栈溢出或 configASSERT。 */
volatile uint32_t debug_rtos_stack_overflow_count;
volatile const char* debug_rtos_stack_overflow_task;
volatile char debug_rtos_stack_overflow_name[16];
volatile uint32_t debug_rtos_assert_count;
volatile uint32_t debug_rtos_assert_line;
volatile const char* debug_rtos_assert_file;

/* 覆盖 CMSIS-RTOS2 提供的 weak hook。 */
void vApplicationStackOverflowHook(TaskHandle_t task, char* task_name) {
    (void)task;
    debug_rtos_stack_overflow_task = task_name;
    for (uint32_t i = 0U; i < (sizeof(debug_rtos_stack_overflow_name) - 1U); i++) {
        char c = task_name ? task_name[i] : '\0';
        debug_rtos_stack_overflow_name[i] = c;
        if (c == '\0') break;
    }
    debug_rtos_stack_overflow_name[sizeof(debug_rtos_stack_overflow_name) - 1U] = '\0';
    debug_rtos_stack_overflow_count++;
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void app_freertos_assert_failed(const char* file, uint32_t line) {
    debug_rtos_assert_file = file;
    debug_rtos_assert_line = line;
    debug_rtos_assert_count++;
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}
#endif

static const char* TAG = "INIT";
static uint8_t s_tasks_created;

void app_tasks_create(void) {
    if (s_tasks_created) {
        LOGW("app_tasks_create ignored: already created");
        return;
    }
    s_tasks_created = 1U;

    LOGI("app_tasks_create start");

    task_comm_init();

    /* 业务初始化：注册 USB 解析器 + 已使能的控制子系统 */
#if APP_CHASSIS_ENABLE
    task_chassis_init();
#endif
    task_arm_init();

#if APP_TARGET_MCU
    osThreadNew(task_log_entry,     NULL, &s_attr_log);
    osThreadNew(task_safety_entry,  NULL, &s_attr_safety);
    osThreadNew(task_comm_entry,    NULL, &s_attr_comm);
#if APP_CHASSIS_ENABLE
    osThreadNew(task_chassis_entry, NULL, &s_attr_chassis);
#endif
    osThreadNew(task_arm_entry,     NULL, &s_attr_arm);
    LOGI("app_tasks_create: %u tasks spawned",
         (unsigned)(APP_CHASSIS_ENABLE ? 5U : 4U));
#else
    LOGI("app_tasks_create: host build, no RTOS threads");
#endif
}
