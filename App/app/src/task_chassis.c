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
 *   STANDALONE  → 完全忽略 USB；跑离线 stand/march/stand 序列
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
#include "chassis_planner.h"
#include "gait_script.h"
#include "leg_controller.h"
#include "motor_go.h"
#include "motor_m3508.h"
#include "imu_bmi088.h"
#include "attitude_estimator.h"
#include "steer_controller.h"

#if APP_TARGET_MCU
#include "cmsis_os.h"
#endif

#include <math.h>
#include <string.h>

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
static gait_params_t    s_trot_params;
static uint8_t          s_manual_gait_hold = 0U;
static float            s_last_effective_wz = 0.0f;

static const gait_params_t S_OFFLINE_MARCH_PARAMS = {
    .body_height_m    = 0.20f,
    .step_length_m    = 0.0f,
    .step_height_m    = 0.015f,
    .period_s         = 1.0f,
    .duty             = 0.50f,
    .phase_offset     = { 0.0f, 0.5f, 0.5f, 0.0f },
    .touchdown_thresh = 0.0f,
};

static uint8_t  s_offline_seq_active = 0U;
static uint8_t  s_offline_seq_done = 0U;
static uint32_t s_offline_seq_start_ms = 0U;
static steer_mode_t s_steer_mode = STEER_MODE_OFF;
static chassis_plan_t s_chassis_plan;

static float controller_height_from_body(float body_height_m) {
    if (!isfinite(body_height_m) || body_height_m == 0.0f) {
        return -0.18f;
    }
    return (body_height_m > 0.0f) ? -body_height_m : body_height_m;
}

static void apply_controller_height(const gait_params_t* p) {
    if (!p) return;
    leg_controller_set_stand_height(controller_height_from_body(p->body_height_m));
}

static int validate_trot_params(const gait_params_t* p) {
    if (!p) return 0;
    if (!isfinite(p->body_height_m) || fabsf(p->body_height_m) < 0.05f ||
        fabsf(p->body_height_m) > 0.40f) {
        return 0;
    }
    if (!isfinite(p->step_length_m) || fabsf(p->step_length_m) > 0.20f) {
        return 0;
    }
    if (!isfinite(p->step_height_m) || p->step_height_m < 0.0f || p->step_height_m > 0.12f) {
        return 0;
    }
    if (!isfinite(p->period_s) || p->period_s < 0.10f || p->period_s > 5.0f) {
        return 0;
    }
    if (!isfinite(p->duty) || p->duty < 0.05f || p->duty > 0.95f) {
        return 0;
    }
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (!isfinite(p->phase_offset[i])) {
            return 0;
        }
    }
    if (!isfinite(p->touchdown_thresh)) {
        return 0;
    }
    return 1;
}

static int request_gait(gait_if_t* g, const gait_params_t* p, float blend) {
    if (s_gm.current == g) {
        if (g && g->ops && g->ops->set_param && p) {
            return g->ops->set_param(g, p);
        }
        return APP_OK;
    }
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
    s_trot_params = GAIT_PARAMS_TROT_DEFAULT;
    leg_controller_init(&s_lc);
    leg_controller_bind_from_registry(&s_lc);
#if APP_DEBUG_RL_WHEEL_ONLY
    leg_controller_set_output_options((uint8_t)(1U << GAIT_LEG_RL), 1U, 1U, 1.5f, 0.1f);
    apply_controller_height(&GAIT_PARAMS_STAND_DEFAULT);
    LOGW("debug mode: RL single-leg closed loop; stand locks wheel, vx/wz drives RL trot + wheel");
#else
    leg_controller_set_output_options(0x0Fu, 1U, 1U, 1.5f, 0.1f);
    apply_controller_height(&GAIT_PARAMS_STAND_DEFAULT);
#endif
    s_active = ACT_STAND;
    s_mode   = CHASSIS_MODE_AUTO;          /* host 单测可重复 init 时需复位 */
    s_online_timeout_ms = 500;
    s_manual_gait_hold = 0U;
    s_last_effective_wz = 0.0f;

    /* BMI088 转向初始化 */
    attitude_estimator_init();
    steer_controller_init();
    chassis_planner_init();
    s_steer_mode = STEER_MODE_OFF;

    s_offline_seq_active = 0U;
    s_offline_seq_done = 0U;
    s_offline_seq_start_ms = 0U;
    LOGI("chassis init: mode=AUTO active=stand timeout=%ums",
         (unsigned)s_online_timeout_ms);
}

void task_chassis_set_mode(chassis_mode_t m) {
    if (m == s_mode) return;
    s_mode = m;
    LOGI("mode -> %d", (int)m);
}
chassis_mode_t task_chassis_get_mode(void) { return s_mode; }

chassis_gait_active_t task_chassis_get_gait_active(void) {
    switch (s_active) {
        case ACT_TROT:   return CHASSIS_GAIT_TROT;
        case ACT_SCRIPT: return CHASSIS_GAIT_SCRIPT;
        case ACT_STAND:
        default:         return CHASSIS_GAIT_STAND;
    }
}

void     task_chassis_set_online_timeout_ms(uint32_t ms) { s_online_timeout_ms = ms; }
uint32_t task_chassis_get_online_timeout_ms(void)        { return s_online_timeout_ms; }

int task_chassis_play_script(const script_t* s, float blend_dur_s) {
    if (!s) return APP_ERR_INVALID_ARG;
    s_manual_gait_hold = 0U;
    int r = gait_script_set_script(s_script, s);
    if (r != APP_OK) return r;
    r = request_gait(s_script, &GAIT_PARAMS_STAND_DEFAULT, blend_dur_s);
    if (r == APP_OK) s_active = ACT_SCRIPT;
    return r;
}

int task_chassis_stop_script(float blend_dur_s) {
    return task_chassis_start_stand(blend_dur_s);
}

int task_chassis_start_stand(float blend_dur_s) {
    s_manual_gait_hold = 1U;
    int r = request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, blend_dur_s);
    if (r == APP_OK) s_active = ACT_STAND;
    return r;
}

int task_chassis_set_trot_params(const gait_params_t* p) {
    if (!validate_trot_params(p)) return APP_ERR_INVALID_ARG;
    s_trot_params = *p;
    if (s_active == ACT_TROT && s_gm.current == s_trot) {
        apply_controller_height(&s_trot_params);
        return request_gait(s_trot, &s_trot_params, 0.0f);
    }
    return APP_OK;
}

void task_chassis_get_trot_params(gait_params_t* out) {
    if (!out) return;
    *out = s_trot_params;
}

int task_chassis_start_trot(const gait_params_t* p, float blend_dur_s) {
    if (p) {
        int r = task_chassis_set_trot_params(p);
        if (r != APP_OK) return r;
    }
    apply_controller_height(&s_trot_params);
    int r = request_gait(s_trot, &s_trot_params, blend_dur_s);
    if (r == APP_OK) {
        s_active = ACT_TROT;
        s_manual_gait_hold = 1U;
    }
    return r;
}

const char* task_chassis_active_gait_name(void) {
    if (!s_gm.current || !s_gm.current->ops || !s_gm.current->ops->name) return "?";
    return s_gm.current->ops->name();
}

/* ─── BMI088 转向接口 ─── */

void task_chassis_reset_yaw(void) {
    attitude_estimator_reset_yaw();
}

float task_chassis_get_yaw(void) {
    return attitude_estimator_get_yaw();
}

float task_chassis_get_effective_wz(void) {
    return s_last_effective_wz;
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

/* 在线分支：根据规划结果二选一 stand/trot */
static void online_decide(const chassis_plan_t* plan) {
    if (!plan) return;
    if (plan->moving) {
        if (s_active != ACT_TROT) {
            apply_controller_height(&plan->gait_params);
            if (request_gait(s_trot, &plan->gait_params, 0.3f) == APP_OK) {
                s_active = ACT_TROT;
                s_manual_gait_hold = 0U;
            }
        } else if (s_trot && s_trot->ops && s_trot->ops->set_param) {
            apply_controller_height(&plan->gait_params);
            (void)s_trot->ops->set_param(s_trot, &plan->gait_params);
        }
    } else {
        if (s_active != ACT_STAND) {
            if (task_chassis_start_stand(0.3f) == APP_OK) s_active = ACT_STAND;
        }
    }
}

static void apply_plan_wheel_speed(gait_output_t* out, const chassis_plan_t* plan) {
    if (!out || !plan) return;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        out->leg[i].wheel_rads = out->leg[i].in_stance ? plan->wheel_rads[i] : 0.0f;
    }
}

#if APP_DEBUG_RL_WHEEL_ONLY
static void apply_rl_single_leg_debug(uint8_t online, const chassis_plan_t* plan, float dt_s) {
    if (online && plan) {
        s_manual_gait_hold = 0U;
        online_decide(plan);
    } else if (!s_manual_gait_hold) {
        if (s_active != ACT_STAND) {
            if (request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) {
                s_active = ACT_STAND;
            }
        }
    }

    gait_output_t out;
    gait_machine_update(&s_gm, dt_s, &out);

    /* 只输出 RL：轮速来自 planner；无在线速度命令时给 0，用 M3508 速度环锁住。 */
    out.leg[GAIT_LEG_RL].wheel_rads = (online && plan) ? plan->wheel_rads[GAIT_LEG_RL] : 0.0f;
    leg_controller_apply_dt(&s_lc, &out, dt_s);
}
#endif

static void update_steering(float dt_s, const task_comm_chassis_cmd_t* cmd, float* wz_out) {
    if (!cmd || !wz_out) return;

    *wz_out = cmd->wz;

    imu_bmi088_data_t imu_data;
    if (imu_bmi088_read(&imu_data) == APP_OK) {
        attitude_estimator_update(imu_data.gyro, imu_data.accel, dt_s);
    }

    steer_mode_t requested = (cmd->steer_mode == 1U) ? STEER_MODE_YAW : STEER_MODE_OFF;
    if (requested != s_steer_mode) {
        s_steer_mode = requested;
        (void)steer_controller_set_mode(requested);
    }

    if (s_steer_mode == STEER_MODE_YAW && imu_bmi088_is_ready()) {
        (void)steer_controller_set_target_yaw(cmd->target_yaw);
        *wz_out = steer_controller_update(attitude_estimator_get_yaw(), dt_s);
    }
}

/* 离线分支：保留当前 SCRIPT；否则确保是 stand */
static void offline_decide(uint32_t now_ms) {
#if !APP_OFFLINE_AUTO_MARCH
    (void)now_ms;
    if (s_manual_gait_hold && (s_active == ACT_STAND || s_active == ACT_TROT)) {
        return;
    }
    if (!s_offline_seq_active) {
        s_offline_seq_active = 1U;
        s_offline_seq_done = 1U;
        if (request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, 0.0f) == APP_OK) {
            s_active = ACT_STAND;
        }
        LOGI("offline stand hold");
    } else if (s_active != ACT_STAND && !s_manual_gait_hold) {
        if (request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) {
            s_active = ACT_STAND;
        }
    }
    return;
#else
    if (s_manual_gait_hold && (s_active == ACT_STAND || s_active == ACT_TROT)) {
        return;
    }

    if (!s_offline_seq_active) {
        s_offline_seq_active = 1U;
        s_offline_seq_done = 0U;
        s_offline_seq_start_ms = now_ms;
        if (request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, 0.0f) == APP_OK) {
            s_active = ACT_STAND;
        }
        LOGI("offline sequence start: stand 2s -> march 5s -> stand");
    }

    if (!s_offline_seq_done) {
        uint32_t elapsed_ms = now_ms - s_offline_seq_start_ms;

        if (elapsed_ms < 2000U) {
            if (s_active != ACT_STAND) {
                if (request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) {
                    s_active = ACT_STAND;
                }
            }
            return;
        }

        if (elapsed_ms < 7000U) {
            if (s_active != ACT_TROT) {
                if (request_gait(s_trot, &S_OFFLINE_MARCH_PARAMS, 0.3f) == APP_OK) {
                    s_active = ACT_TROT;
                }
            }
            return;
        }

        if (s_active != ACT_STAND) {
            if (request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) {
                s_active = ACT_STAND;
            }
        }
        s_offline_seq_done = 1U;
        LOGI("offline sequence done: stand");
        return;
    }

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
#endif
}

void task_chassis_step_for_test(float dt_s, uint32_t now_ms) {
    task_comm_chassis_cmd_t cmd;
    float effective_wz = 0.0f;
    uint8_t online = 0U;

#if !APP_TARGET_HOST
    {
        static uint32_t s_pre_calib_cnt = 0;
        static uint8_t  s_pre_calib_done = 0;

        if (!s_pre_calib_done) {
            motor_go_send_all();          /* 零力矩收发：mode=1, kp/kd/tau=0，收集编码器反馈 */
            if (s_pre_calib_cnt < 500U) {  /* 1000ms @ 500Hz，给 GO 电机足够时间首包响应 */
                s_pre_calib_cnt++;
                return;
            }
            (void)motor_go_calibrate_all(); /* 用等待窗口末尾的最新 raw 统一锁存 zero_offset */
            s_pre_calib_done = 1;
            LOGI("pre-calib done after %u cycles", (unsigned)s_pre_calib_cnt);
            /* 超时后不再要求全部回包：只要有电机成功标定即可进入闭环 */
            return;
        }
    }
#endif

    task_comm_get_chassis(&cmd);
    update_steering(dt_s, &cmd, &effective_wz);
    s_last_effective_wz = effective_wz;

    chassis_cmd_plan_t plan_cmd = {
        .vx_m_s = cmd.vx,
        .vy_m_s = cmd.vy,
        .wz_rad_s = effective_wz,
    };
    (void)chassis_planner_update(&plan_cmd, &s_trot_params, &s_chassis_plan);

    online = (uint8_t)!is_offline(now_ms);

#if APP_DEBUG_RL_WHEEL_ONLY
    apply_rl_single_leg_debug(online, &s_chassis_plan, dt_s);
#if !APP_TARGET_HOST
    motor_go_send_all();
    motor_m3508_send_all();
#endif
    return;
#endif

    if (!online) {
        offline_decide(now_ms);
    } else {
        s_offline_seq_active = 0U;
        s_offline_seq_done = 0U;
        online_decide(&s_chassis_plan);
    }

    gait_output_t out;
    gait_machine_update(&s_gm, dt_s, &out);
    if (online) {
        apply_plan_wheel_speed(&out, &s_chassis_plan);
    }
    leg_controller_apply_dt(&s_lc, &out, dt_s);

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
