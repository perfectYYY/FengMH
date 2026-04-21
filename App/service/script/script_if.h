/*
 * script_if.h — 脚本化"死程序"步态接口
 *
 * 设计目标：
 *   - 像旧 main.c 那样手写关键帧（时间戳 + 四腿目标），无需上位机
 *   - 关键帧间线性插值输出到 gait_output_t，复用 gait_machine/leg_controller
 *   - 完全 host 可测试，不依赖外设
 *
 * 关键帧结构：
 *   script_keyframe_t { t_s, leg[4] = {hip_rad, knee_rad, wheel_rads, in_stance} }
 *
 * 使用示例：
 *   static const script_keyframe_t k_frames[] = {
 *       { 0.00f, {{0,0,0,1},{0,0,0,1},{0,0,0,1},{0,0,0,1}} },
 *       { 0.50f, {{0.1f,-0.2f,0,1}, ...} },
 *       ...
 *   };
 *   const script_t MY_SCRIPT = { "my", k_frames, 5, 0 };  // loop=0
 */
#ifndef APP_SERVICE_SCRIPT_IF_H_
#define APP_SERVICE_SCRIPT_IF_H_

#include "gait_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float              t_s;                     /* 从脚本起始计的时间戳 (s) */
    gait_leg_target_t  leg[GAIT_LEG_NUM];
} script_keyframe_t;

typedef struct {
    const char*                name;
    const script_keyframe_t*   frames;
    uint16_t                   n_frames;
    uint8_t                    loop;            /* 1=到末尾自动回 0 重播 */
} script_t;

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_SCRIPT_IF_H_ */
