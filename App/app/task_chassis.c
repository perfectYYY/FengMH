/*
 * task_chassis.c
 *
 * 状态机（task 内）：
 *   ACT_STAND  默认空闲（gait_stand）
 *   ACT_TROT   ONLINE 收到 |v|>eps 切入
 *   ACT_SCRIPT 由 task_chassis_play_script() 切入；脚本播完自动回 ACT_STAND
 *
 * 模式：
 *   ONLINE      → 仅根据 task_comm 决策 STAND/TROT，不接受脚本
 *   STANDALONE  → 完全忽略 USB；默认放 SCRIPT(SCRIPT_BUILTIN_STAND_HOLD)
 *   AUTO        → 心跳活 → ONLINE 行为；心跳超时 → 切 STANDALONE 行为
 *
 * 心跳超时 = now_ms - task_comm_last_rx_ms() > online_timeout_ms (默认 500)
 *
 */
#include "task_chassis.h"
#include "log.h"
#include "config.h"
#include "task_comm.h"
#include "task_safety.h"
#include "bsp_time.h"
#include "gait_if.h"
#include "gait_stand.h"
#include "gait_trot.h"
#include "gait_machine.h"
#include "gait_params.h"
#include "gait_script.h"
#include "script_builtin.h"
#include "leg_controller.h"
#include "motor_go.h"
#include "motor_m3508.h"

#if APP_TARGET_MCU
#include "cmsis_os.h"
#endif

#include <math.h>

static const char* TAG = "CHASSIS";

typedef enum {
    ACT_STAND = 0,
    ACT_TROT,
    ACT_SCRIPT,
} active_t;

static gait_machine_t   s_gm;
static leg_controller_t s_lc;
static gait_if_t*       s_stand;
static gait_if_t*       s_trot;
static gait_if_t*       s_script;

static chassis_mode_t   s_mode = CHASSIS_MODE_AUTO;
static active_t         s_active = ACT_STAND;
static uint32_t         s_online_timeout_ms = 500;

static int request_gait(gait_if_t* g, const gait_params_t* p, float blend) {
    if (s_gm.current == g) return APP_OK;
    /* gait_machine_request 只在 RUN 态接受切换；BLEND 中先用 set 强切 */
    if (s_gm.state != GM_STATE_RUN) {
        return gait_machine_set(&s_gm, g, p);
    }
    return gait_machine_request(&s_gm, g, p, blend);
}

void task_chassis_init(void) {
    s_stand  = gait_stand_create();
    s_trot   = gait_trot_create();
    s_script = gait_script_create();
    gait_machine_init(&s_gm);
    gait_machine_set(&s_gm, s_stand, &GAIT_PARAMS_STAND_DEFAULT);
    leg_controller_init(&s_lc);
    leg_controller_bind_from_registry(&s_lc);
    s_active = ACT_STAND;
    s_mode   = CHASSIS_MODE_AUTO;          /* host 单测可重复 init 时需复位 */
    s_online_timeout_ms = 500;
    LOGI("chassis init: mode=AUTO active=stand timeout=%ums",
         (unsigned)s_online_timeout_ms);
}

void task_chassis_set_mode(chassis_mode_t m) {
    if (m == s_mode) return;
    s_mode = m;
    LOGI("mode -> %d", (int)m);
}
chassis_mode_t task_chassis_get_mode(void) { return s_mode; }

void     task_chassis_set_online_timeout_ms(uint32_t ms) { s_online_timeout_ms = ms; }
uint32_t task_chassis_get_online_timeout_ms(void)        { return s_online_timeout_ms; }

int task_chassis_play_script(const script_t* s, float blend_dur_s) {
    if (!s) return APP_ERR_INVALID_ARG;
    int r = gait_script_set_script(s_script, s);
    if (r != APP_OK) return r;
    r = request_gait(s_script, &GAIT_PARAMS_STAND_DEFAULT, blend_dur_s);
    if (r == APP_OK) s_active = ACT_SCRIPT;
    return r;
}

int task_chassis_stop_script(float blend_dur_s) {
    int r = request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, blend_dur_s);
    if (r == APP_OK) s_active = ACT_STAND;
    return r;
}

const char* task_chassis_active_gait_name(void) {
    if (!s_gm.current || !s_gm.current->ops || !s_gm.current->ops->name) return "?";
    return s_gm.current->ops->name();
}

/* 是否处在"事实上的离线"状态 */
static int is_offline(uint32_t now_ms) {
    if (s_mode == CHASSIS_MODE_STANDALONE) return 1;
    if (s_mode == CHASSIS_MODE_ONLINE)     return 0;
    /* AUTO：从未收过任何帧（按 dispatch_hit 判定，不依赖时戳基准）→ 离线 */
    if (task_comm_dispatch_hit() == 0) return 1;
    uint32_t last = task_comm_last_rx_ms();
    return (now_ms - last) > s_online_timeout_ms;
}

/* 在线分支：根据 chassis_cmd 二选一 stand/trot */
static void online_decide(void) {
    task_comm_chassis_cmd_t cmd;
    task_comm_get_chassis(&cmd);
    float n = fabsf(cmd.vx) + fabsf(cmd.vy) + fabsf(cmd.wz);
    if (n > 0.05f) {
        if (s_active != ACT_TROT) {
            if (request_gait(s_trot, &GAIT_PARAMS_TROT_DEFAULT, 0.3f) == APP_OK) s_active = ACT_TROT;
        }
    } else {
        if (s_active != ACT_STAND) {
            if (request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) s_active = ACT_STAND;
        }
    }
}

/* 离线分支：保留当前 SCRIPT；否则确保是 stand */
static void offline_decide(void) {
    if (s_active == ACT_SCRIPT) {
        /* 脚本播完自动回 stand */
        if (gait_script_state(s_script) == SP_STATE_DONE) {
            if (request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) s_active = ACT_STAND;
        }
        return;
    }
    if (s_active != ACT_STAND) {
        if (request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) s_active = ACT_STAND;
    }
}

void task_chassis_step_for_test(float dt_s, uint32_t now_ms) {
    if (is_offline(now_ms)) offline_decide();
    else                    online_decide();
    gait_output_t out;
    gait_machine_update(&s_gm, dt_s, &out);
    leg_controller_apply(&s_lc, &out);

#if !APP_TARGET_HOST
    /* MCU 端：将电机指令推送到物理总线 */
    motor_go_send_all();
    motor_m3508_send_all();
#endif
}

void task_chassis_entry(void* arg) {
    (void)arg;
    LOGI("task_chassis started");
#if APP_TARGET_MCU
    const float dt = 0.002f; /* 500Hz */
    for (;;) {
        if (task_safety_estop_active()) {
            osDelay(20);
            continue;
        }
        uint32_t now = (uint32_t)bsp_time_now_ms();
        task_chassis_step_for_test(dt, now);
        osDelay(2);
    }
#endif
}
