/*
 * gait_script.h — 把 script_player 包装成 gait_if_t
 *
 * 用法：
 *   gait_if_t* g = gait_script_create();
 *   gait_script_set_script(g, &MY_SCRIPT);
 *   gait_machine_request(&gm, g, &GAIT_PARAMS_STAND_DEFAULT, 0.3f);
 *
 * 设计选择：
 *   - 单实例（与 stand/trot 一致）；M3 内一次只播一段脚本，需要叠播留 M5
 *   - set_param 不会重置脚本时间；想从头播请调 gait_script_rewind()
 */
#ifndef APP_SERVICE_GAIT_SCRIPT_H_
#define APP_SERVICE_GAIT_SCRIPT_H_

#include "gait_if.h"
#include "script_if.h"
#include "script_player.h"

#ifdef __cplusplus
extern "C" {
#endif

gait_if_t* gait_script_create(void);
app_err_t  gait_script_set_script(gait_if_t* self, const script_t* s);
app_err_t  gait_script_rewind(gait_if_t* self);
script_player_state_t gait_script_state(const gait_if_t* self);

#ifdef __cplusplus
}
#endif

#endif
