/*
 * app_init.h — 全局初始化汇总
 *
 * 目标：M1 完成后，main.c 的 USER CODE 区域最终只剩
 *      app_init();
 *      osKernelStart();
 * 当前阶段先提供 app_init_bring_up()，由 main() 在旧流程前调用，
 * 不替换旧初始化；确保并行切换。
 */
#ifndef APP_APP_INIT_H_
#define APP_APP_INIT_H_

#include "err.h"

#ifdef __cplusplus
extern "C" {
#endif

app_err_t app_init(void);
app_err_t app_init_bring_up(void);  /* M1 过渡期入口 */

#ifdef __cplusplus
}
#endif

#endif /* APP_APP_INIT_H_ */
