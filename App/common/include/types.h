/*
 * types.h — 全工程通用基础类型
 */
#ifndef APP_COMMON_TYPES_H_
#define APP_COMMON_TYPES_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* 通用工具宏 */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

#ifndef UNUSED_ARG
#define UNUSED_ARG(x) ((void)(x))
#endif

/* 纳入工程的"时间戳"统一为 ms */
typedef uint32_t app_tick_t;

#endif /* APP_COMMON_TYPES_H_ */
