/*
 * script_player.h — 脚本播放器（关键帧线性插值）
 */
#ifndef APP_SERVICE_SCRIPT_PLAYER_H_
#define APP_SERVICE_SCRIPT_PLAYER_H_

#include "script_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SP_STATE_IDLE = 0,
    SP_STATE_RUNNING,
    SP_STATE_DONE
} script_player_state_t;

typedef struct {
    const script_t*       s;
    float                 t_s;         /* 当前累计时间 */
    uint32_t              tick_count;
    script_player_state_t state;
} script_player_t;

void      script_player_init(script_player_t* p);
app_err_t script_player_load(script_player_t* p, const script_t* s);
app_err_t script_player_reset(script_player_t* p);
app_err_t script_player_update(script_player_t* p, float dt_s, gait_output_t* out);
script_player_state_t script_player_state(const script_player_t* p);

/* 纯函数：从脚本 + 时间戳插值出一帧（host 测试友好） */
app_err_t script_sample(const script_t* s, float t_s, gait_output_t* out);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_SCRIPT_PLAYER_H_ */
