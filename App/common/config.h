/*
 *

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

/*
 * Board bring-up stages.
 *
 * Normal firmware builds use APP_BRINGUP_STAGE_NORMAL. The scripts under
 * tools/bringup pass lower stage values to hard-limit actuator output while
 * the board is being tested for the first time.
 */
#define APP_BRINGUP_STAGE_BOARD_ONLY      0
#define APP_BRINGUP_STAGE_BUS_ZERO_TX     1
#define APP_BRINGUP_STAGE_M3508_ZERO      2
#define APP_BRINGUP_STAGE_GO_ZERO         3
#define APP_BRINGUP_STAGE_M3508_JOG       4
#define APP_BRINGUP_STAGE_GO_LEG_HOLD     5
#define APP_BRINGUP_STAGE_STAND_LOW_GAIN  6
#define APP_BRINGUP_STAGE_TROT_LOW_GAIN   7
#define APP_BRINGUP_STAGE_USB_CDC_TEST    8
#define APP_BRINGUP_STAGE_IMU_TEST        9
#define APP_BRINGUP_STAGE_NORMAL          100

#ifndef APP_BRINGUP_STAGE
#define APP_BRINGUP_STAGE APP_BRINGUP_STAGE_NORMAL
#endif

#ifndef APP_BRINGUP_LEG_MASK
#define APP_BRINGUP_LEG_MASK 0x01u  /* default: FL only */
#endif

#ifndef APP_BRINGUP_WHEEL_MASK
#define APP_BRINGUP_WHEEL_MASK 0x01u  /* default: FL wheel only */
#endif

#ifndef APP_BRINGUP_WHEEL_JOG_RAD_S
#define APP_BRINGUP_WHEEL_JOG_RAD_S 0.5f
#endif

/* 过渡期：旧 Core/Src 代码与 App/ 并行编译；M1 完成阶段保持 1，M2 起逐步关闭 */
#ifndef USE_LEGACY
#define USE_LEGACY 1
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

#endif /* APP_COMMON_CONFIG_H_ */
