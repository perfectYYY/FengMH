/*
 * gait_machine.h — 步态切换状态机 + 平滑过渡
 *
 * 切换策略：
 *   1) request(target) 后进入 BLEND 状态
 *   2) BLEND 阶段同时跑当前/目标，输出 = lerp(cur, tgt, t/blend_dur)
 *   3) t >= blend_dur 时切到 RUN，旧步态 exit
 */
#ifndef APP_SERVICE_GAIT_MACHINE_H_
#define APP_SERVICE_GAIT_MACHINE_H_

#include "gait_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GM_STATE_IDLE = 0,
    GM_STATE_RUN,
    GM_STATE_BLEND
} gait_machine_state_t;

typedef struct {
    gait_if_t* current;
    gait_if_t* target;       /* 仅 BLEND 时有值 */
    float      blend_dur_s;
    float      blend_t_s;
    gait_machine_state_t state;

    gait_output_t buf_a;
    gait_output_t buf_b;
} gait_machine_t;

void      gait_machine_init(gait_machine_t* m);
app_err_t gait_machine_set(gait_machine_t* m, gait_if_t* g, const gait_params_t* p);
app_err_t gait_machine_request(gait_machine_t* m, gait_if_t* g,
                               const gait_params_t* p, float blend_dur_s);
app_err_t gait_machine_update(gait_machine_t* m, float dt_s, gait_output_t* out);
gait_machine_state_t gait_machine_state(const gait_machine_t* m);

#ifdef __cplusplus
}
#endif

#endif
