/*
 * log.c — M1 阶段的简化实现
 *  - 后端列表（最多 4 个）
 *  - 运行期级别控制（全局 + tag 表，tag 表上限 16）
 *  - 格式化到固定栈缓冲（256B），避免堆分配
 *  - 末条日志副本用于 host 测试断言
 *
 * 注意：此实现不是线程安全的；进 FreeRTOS 后由 log.c 的 task_log 后续接管。
 */
#include "log.h"

#include <stdio.h>
#include <string.h>

#if LOG_BACKEND_STDIO
#include <stdio.h>
#endif

#define LOG_MAX_BACKENDS   4
#define LOG_MAX_TAG_RULES  16
#define LOG_LINE_MAX       256
#define LOG_TAG_MAX        16

typedef struct {
    char      tag[LOG_TAG_MAX];
    log_lvl_t lvl;
} tag_rule_t;

static log_backend_fn s_backends[LOG_MAX_BACKENDS];
static uint8_t        s_backend_cnt = 0;

static tag_rule_t     s_tag_rules[LOG_MAX_TAG_RULES];
static uint8_t        s_tag_rule_cnt = 0;

static log_lvl_t      s_global_lvl = (log_lvl_t)LOG_DEFAULT_LEVEL;

static char           s_last_line[LOG_LINE_MAX];

#if LOG_BACKEND_STDIO
static void stdio_backend(log_lvl_t lvl, const char* tag, const char* msg) {
    static const char* LVL_TXT[] = {"?", "E", "W", "I", "D", "T"};
    int i = (int)lvl;
    if (i < 0 || i > 5) i = 0;
    fprintf(stdout, "[%s][%s] %s\n", LVL_TXT[i], tag ? tag : "-", msg);
}
#endif

static log_lvl_t lookup_tag_level(const char* tag) {
    if (!tag) return s_global_lvl;
    for (uint8_t i = 0; i < s_tag_rule_cnt; i++) {
        if (strncmp(s_tag_rules[i].tag, tag, LOG_TAG_MAX) == 0) {
            return s_tag_rules[i].lvl;
        }
    }
    return s_global_lvl;
}

void log_init(void) {
    s_backend_cnt = 0;
    s_tag_rule_cnt = 0;
    s_global_lvl = (log_lvl_t)LOG_DEFAULT_LEVEL;
    s_last_line[0] = '\0';
#if LOG_BACKEND_STDIO
    log_register_backend(stdio_backend);
#endif
}

void log_register_backend(log_backend_fn fn) {
    if (!fn || s_backend_cnt >= LOG_MAX_BACKENDS) return;
    s_backends[s_backend_cnt++] = fn;
}

void log_set_global_level(log_lvl_t lvl) { s_global_lvl = lvl; }
log_lvl_t log_get_global_level(void) { return s_global_lvl; }

void log_set_level(const char* tag, log_lvl_t lvl) {
    if (!tag) return;
    for (uint8_t i = 0; i < s_tag_rule_cnt; i++) {
        if (strncmp(s_tag_rules[i].tag, tag, LOG_TAG_MAX) == 0) {
            s_tag_rules[i].lvl = lvl;
            return;
        }
    }
    if (s_tag_rule_cnt < LOG_MAX_TAG_RULES) {
        strncpy(s_tag_rules[s_tag_rule_cnt].tag, tag, LOG_TAG_MAX - 1);
        s_tag_rules[s_tag_rule_cnt].tag[LOG_TAG_MAX - 1] = '\0';
        s_tag_rules[s_tag_rule_cnt].lvl = lvl;
        s_tag_rule_cnt++;
    }
}

void log_emit(log_lvl_t lvl, const char* tag, const char* fmt, ...) {
    if (lvl > lookup_tag_level(tag)) return;

    char buf[LOG_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt ? fmt : "", ap);
    va_end(ap);
    if (n < 0) return;

    /* 保存最新一行，供 host 单测断言 */
    strncpy(s_last_line, buf, LOG_LINE_MAX - 1);
    s_last_line[LOG_LINE_MAX - 1] = '\0';

    for (uint8_t i = 0; i < s_backend_cnt; i++) {
        s_backends[i](lvl, tag, buf);
    }
}

const char* log_last_line(void) { return s_last_line; }
