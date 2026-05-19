/*
 * log.h — 统一日志宏 / tag 分级 / 可切换后端
 *
 * 使用方法：
 *   static const char* TAG = "M3508";
 *   LOGI("online, rx_cnt=%u", rx_cnt);
 */
#ifndef APP_COMMON_LOG_H_
#define APP_COMMON_LOG_H_

#include <stdarg.h>
#include "types.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LOG_LVL_ERR   = 1,
    LOG_LVL_WARN  = 2,
    LOG_LVL_INFO  = 3,
    LOG_LVL_DBG   = 4,
    LOG_LVL_TRACE = 5
} log_lvl_t;

/* 日志后端接口：可注册多个 */
typedef void (*log_backend_fn)(log_lvl_t lvl, const char* tag, const char* msg);

void log_init(void);
void log_register_backend(log_backend_fn fn);
void log_set_global_level(log_lvl_t lvl);
log_lvl_t log_get_global_level(void);

/* 给 tag 设定级别（未设定则用全局级别） */
void log_set_level(const char* tag, log_lvl_t lvl);

/* 内部分发入口，宏展开使用 */
void log_emit(log_lvl_t lvl, const char* tag, const char* fmt, ...);

/* 测试/host 便利：抓取最后一条日志（不做环形队列，只是最近一条副本） */
const char* log_last_line(void);

#define LOGE(fmt, ...) log_emit(LOG_LVL_ERR,   TAG, (fmt), ##__VA_ARGS__)
#define LOGW(fmt, ...) log_emit(LOG_LVL_WARN,  TAG, (fmt), ##__VA_ARGS__)
#define LOGI(fmt, ...) log_emit(LOG_LVL_INFO,  TAG, (fmt), ##__VA_ARGS__)
#define LOGD(fmt, ...) log_emit(LOG_LVL_DBG,   TAG, (fmt), ##__VA_ARGS__)
#define LOGT(fmt, ...) log_emit(LOG_LVL_TRACE, TAG, (fmt), ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* APP_COMMON_LOG_H_ */
