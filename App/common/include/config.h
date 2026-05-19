/*
 * config.h - App-wide compile-time switches.
 */
#ifndef APP_COMMON_CONFIG_H_
#define APP_COMMON_CONFIG_H_

/* 运行目标区分：板上固件 vs PC 单测 host 构建 */
#ifndef APP_TARGET_HOST
#define APP_TARGET_HOST 0
#endif

#ifndef APP_TARGET_MCU
#define APP_TARGET_MCU (!APP_TARGET_HOST)
#endif

/* 日志后端开关：SEGGER RTT（板上）/ 标准输出（host） */
#ifndef LOG_BACKEND_RTT
#define LOG_BACKEND_RTT 0   /* 暂不强制依赖 SEGGER 源码；当 RTT 源码接入后切 1 */
#endif

#ifndef LOG_BACKEND_STDIO
#define LOG_BACKEND_STDIO (APP_TARGET_HOST)
#endif

/* 日志默认运行期级别 */
#ifndef LOG_DEFAULT_LEVEL
#define LOG_DEFAULT_LEVEL 3  /* INFO */
#endif

/* 协议解析栈缓冲最大 payload 长度 */
#ifndef PROTO_MAX_PAYLOAD
#define PROTO_MAX_PAYLOAD 64
#endif

/*
 * 当前上板调试阶段：只保持 RL 腿站立，并只允许 RL_WHEEL 响应 vx/wz
 * 规划出的轮速；vx=wz=0 时用于测试 3508 零速锁定。
 * 正常四轮底盘联调时把它改为 0，或在编译参数里覆盖。
 */
#ifndef APP_DEBUG_RL_WHEEL_ONLY
#define APP_DEBUG_RL_WHEEL_ONLY 1
#endif

#endif /* APP_COMMON_CONFIG_H_ */
