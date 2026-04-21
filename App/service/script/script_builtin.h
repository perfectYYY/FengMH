/*
 * script_builtin.h — 出厂内置脚本集合（类似旧 main.c 里的死程序步态）
 *
 * 当前提供：
 *   - SCRIPT_BUILTIN_STAND_HOLD : 全腿支撑、零目标，1s 单帧
 *   - SCRIPT_BUILTIN_WAVE_UP_DOWN : 单腿抬放（9 关键帧，周期 0.9s，loop）
 *                                    对应旧 up_down[9] + ang_mir/ang_ori 的抬腿序列
 *   - SCRIPT_BUILTIN_TROT_STEP    : 对角 trot 4 关键帧（loop），用于无上位机连线时快速跑起来
 *
 * 自定义：用户直接在 script_builtin.c 里增补 keyframe 表即可；
 *         或在外部翻译单元定义自己的 script_t 变量传给 gait_script_set_script()。
 */
#ifndef APP_SERVICE_SCRIPT_BUILTIN_H_
#define APP_SERVICE_SCRIPT_BUILTIN_H_

#include "script_if.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const script_t SCRIPT_BUILTIN_STAND_HOLD;
extern const script_t SCRIPT_BUILTIN_WAVE_UP_DOWN;
extern const script_t SCRIPT_BUILTIN_TROT_STEP;

/* 按名字查找内置脚本；未找到返回 NULL */
const script_t* script_builtin_find(const char* name);

#ifdef __cplusplus
}
#endif

#endif
