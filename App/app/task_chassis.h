/*
 * task_chassis.h — 步态/底盘控制任务（M3：三模式 + 心跳超时回退）
 *
 * 模式：
 *   STANDALONE — 完全脱离 USB CDC：跑一个固定脚本（默认 stand_hold）。
 *                想自定义死程序步态：构造 script_t 调 task_chassis_play_script()。
 *   ONLINE     — 严格按 task_comm 的 chassis_cmd 走 stand/trot 状态机。
 *   AUTO       — 上电默认；心跳活着按 ONLINE，超时(>online_timeout_ms)自动回退 STANDALONE。
 *
 * 接口：
 *   task_chassis_init / entry
 *   task_chassis_set_mode / get_mode
 *   task_chassis_play_script         切到 SCRIPT 子状态并装入指定脚本
 *   task_chassis_stop_script         脚本播完或手动停 → 回 stand
 *   task_chassis_set_online_timeout_ms
 */
#ifndef APP_TASK_CHASSIS_H_
#define APP_TASK_CHASSIS_H_

#include <stdint.h>
#include "gait_if.h"
#include "../script/script_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CHASSIS_MODE_AUTO = 0,
    CHASSIS_MODE_ONLINE,
    CHASSIS_MODE_STANDALONE,
} chassis_mode_t;

void  task_chassis_entry(void* arg);
void  task_chassis_init(void);

void           task_chassis_set_mode(chassis_mode_t m);
chassis_mode_t task_chassis_get_mode(void);

/* 切到 SCRIPT 子状态并播放指定脚本（仅在 STANDALONE / AUTO 离线分支生效）。 */
int   task_chassis_play_script(const script_t* s, float blend_dur_s);
int   task_chassis_stop_script(float blend_dur_s);
int   task_chassis_start_stand(float blend_dur_s);
int   task_chassis_set_trot_params(const gait_params_t* p);
void  task_chassis_get_trot_params(gait_params_t* out);
int   task_chassis_start_trot(const gait_params_t* p, float blend_dur_s);

void     task_chassis_set_online_timeout_ms(uint32_t ms);
uint32_t task_chassis_get_online_timeout_ms(void);

/* host / 板上诊断接口：手动喂时间，让逻辑可单测 */
void  task_chassis_step_for_test(float dt_s, uint32_t now_ms);
const char* task_chassis_active_gait_name(void);

/* BMI088 转向接口 */
void  task_chassis_reset_yaw(void);
float task_chassis_get_yaw(void);
float task_chassis_get_effective_wz(void);

#ifdef __cplusplus
}
#endif

#endif
