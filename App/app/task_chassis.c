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
#include "chassis_planner.h"
#include "../script/gait_script.h"
#include "../script/script_builtin.h"
#include "leg_controller.h"
#include "motor_registry.h"
#include "motor_go.h"
#include "motor_m3508.h"
#include "imu_bmi088.h"
#include "service/attitude/attitude_estimator.h"
#include "service/attitude/steer_controller.h"

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
static gait_params_t    s_trot_params;
static uint8_t          s_manual_gait_hold = 0U;
static float            s_last_effective_wz = 0.0f;

static const motor_logical_id_t WHEEL_ID[GAIT_LEG_NUM] = {
    MOTOR_ID_FL_WHEEL, MOTOR_ID_FR_WHEEL, MOTOR_ID_RL_WHEEL, MOTOR_ID_RR_WHEEL
};

static const motor_logical_id_t HIP_ID[GAIT_LEG_NUM] = {
    MOTOR_ID_FL_HIP, MOTOR_ID_FR_HIP, MOTOR_ID_RL_HIP, MOTOR_ID_RR_HIP
};

static const motor_logical_id_t KNEE_ID[GAIT_LEG_NUM] = {
    MOTOR_ID_FL_KNEE, MOTOR_ID_FR_KNEE, MOTOR_ID_RL_KNEE, MOTOR_ID_RR_KNEE
};

static const gait_params_t S_BRINGUP_TROT_PARAMS = {
    .body_height_m    = 0.20f,
    .step_length_m    = 0.015f,
    .step_height_m    = 0.010f,
    .period_s         = 1.0f,
    .duty             = 0.75f,
    .phase_offset     = { 0.0f, 0.5f, 0.5f, 0.0f },
    .touchdown_thresh = 0.0f,
};

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

static int bringup_is_normal(void) {
    return APP_BRINGUP_STAGE >= APP_BRINGUP_STAGE_NORMAL;
}

static int bringup_allow_trot(void) {
    return bringup_is_normal() || (APP_BRINGUP_STAGE >= APP_BRINGUP_STAGE_TROT_LOW_GAIN);
}

static int bringup_allow_go_tx(void) {
    return bringup_is_normal() ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_BUS_ZERO_TX) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_GO_ZERO) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_GO_LEG_HOLD) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_STAND_LOW_GAIN) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_TROT_LOW_GAIN);
}

static int bringup_allow_m3508_tx(void) {
    return bringup_is_normal() ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_BUS_ZERO_TX) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_M3508_ZERO) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_M3508_JOG) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_STAND_LOW_GAIN) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_TROT_LOW_GAIN);
}

static void bringup_configure_leg_controller(void) {
    if (bringup_is_normal()) {
        leg_controller_set_output_options(0x0Fu, 1U, 1U, 1.5f, 0.1f);
        return;
    }

    switch (APP_BRINGUP_STAGE) {
        case APP_BRINGUP_STAGE_GO_LEG_HOLD:
            leg_controller_set_output_options((uint8_t)APP_BRINGUP_LEG_MASK, 1U, 0U, 0.08f, 0.02f);
            break;

        case APP_BRINGUP_STAGE_STAND_LOW_GAIN:
            /* 匹配 Wheel-legged stand: KP=0.35 KD=0.08 */
            leg_controller_set_output_options(0x0Fu, 1U, 1U, 0.35f, 0.08f);
            break;

        case APP_BRINGUP_STAGE_TROT_LOW_GAIN:
            leg_controller_set_output_options(0x0Fu, 1U, 1U, 0.20f, 0.04f);
            break;

        default:
            leg_controller_set_output_options(0x00u, 0U, 0U, 0.0f, 0.0f);
            break;
    }
}

static void bringup_apply_wheel_jog(void) {
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        motor_dev_t* wheel = motor_get(WHEEL_ID[i]);
        if (!wheel || !wheel->ops || !wheel->ops->set_velocity) {
            continue;
        }

        float target = (((uint8_t)(1U << i) & (uint8_t)APP_BRINGUP_WHEEL_MASK) != 0U) ?
                       (float)APP_BRINGUP_WHEEL_JOG_RAD_S : 0.0f;
                       // bringup_apply_wheel_jog 中，set_velocity 之前：
        if (target != 0.0f && wheel->ops->enable) {
            wheel->ops->enable(wheel);
        }
        (void)wheel->ops->set_velocity(wheel, target);
    }
}

#define MOTOR_TEST_WARMUP_MS      3000U
#define MOTOR_TEST_JOINT_MOVE_MS  2500U
#define MOTOR_TEST_JOINT_GAP_MS    700U
#define MOTOR_TEST_WHEEL_MOVE_MS  2500U
#define MOTOR_TEST_WHEEL_STOP_MS   700U
#define MOTOR_TEST_JOINT_KP          0.08f
#define MOTOR_TEST_JOINT_KD          0.02f
#define MOTOR_TEST_HIP_AMP_RAD       0.08f
#define MOTOR_TEST_KNEE_AMP_RAD      0.06f
#define MOTOR_TEST_TWO_PI            6.28318530f

typedef enum {
    MOTOR_TEST_PHASE_WARMUP = 0,
    MOTOR_TEST_PHASE_HIP,
    MOTOR_TEST_PHASE_JOINT_GAP,
    MOTOR_TEST_PHASE_KNEE,
    MOTOR_TEST_PHASE_WHEEL_FWD,
    MOTOR_TEST_PHASE_WHEEL_STOP,
    MOTOR_TEST_PHASE_WHEEL_REV,
    MOTOR_TEST_PHASE_DONE,
} motor_test_phase_t;

static uint32_t s_motor_test_start_ms = 0U;
static uint8_t  s_motor_test_started = 0U;
static volatile uint8_t  s_motor_test_phase = MOTOR_TEST_PHASE_WARMUP;
static volatile uint8_t  s_motor_test_index = 0U;
static volatile uint32_t s_motor_test_elapsed_ms = 0U;
static volatile float    s_motor_test_joint_wave = 0.0f;
static volatile float    s_motor_test_wheel_target = 0.0f;

static int bringup_mask_has(uint32_t mask, int index) {
    return ((mask & (1UL << (uint32_t)index)) != 0UL);
}

static float bringup_clamp_joint_target(const motor_cfg_t* cfg, float target) {
    if (!cfg) return target;
    if (cfg->limit_min < cfg->limit_max) {
        if (target < cfg->limit_min) return cfg->limit_min;
        if (target > cfg->limit_max) return cfg->limit_max;
    }
    return target;
}

static float bringup_joint_test_target(const motor_cfg_t* cfg, float amp, float wave) {
    float center = cfg ? cfg->boot_angle : 0.0f;
    if (cfg && cfg->limit_min < cfg->limit_max) {
        float lo = cfg->limit_min + amp;
        float hi = cfg->limit_max - amp;
        if (lo <= hi) {
            if (center < lo) center = lo;
            if (center > hi) center = hi;
        }
    }
    return bringup_clamp_joint_target(cfg, center + amp * wave);
}

static void bringup_disable_motor(motor_logical_id_t id) {
    motor_dev_t* dev = motor_get(id);
    if (dev && dev->ops && dev->ops->disable) {
        (void)dev->ops->disable(dev);
    }
}

static void bringup_set_joint_if_ready(motor_logical_id_t id, float target) {
    motor_dev_t* dev = motor_get(id);
    if (!dev || !dev->ops || !dev->ops->set_position || dev->state.rx_cnt == 0U) {
        bringup_disable_motor(id);
        return;
    }
    (void)dev->ops->set_position(dev, target, 0.0f,
                                 MOTOR_TEST_JOINT_KP,
                                 MOTOR_TEST_JOINT_KD,
                                 0.0f);
}

static void bringup_motor_test_zero_go(void) {
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        bringup_disable_motor(HIP_ID[i]);
        bringup_disable_motor(KNEE_ID[i]);
    }
}

static void bringup_motor_test_zero_wheels(int closed_loop_stop) {
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        motor_dev_t* wheel = motor_get(WHEEL_ID[i]);
        if (!wheel || !wheel->ops) continue;
        if (closed_loop_stop && wheel->ops->set_velocity) {
            (void)wheel->ops->set_velocity(wheel, 0.0f);
        } else if (wheel->ops->disable) {
            (void)wheel->ops->disable(wheel);
        }
    }
}

static void bringup_motor_test_apply_joint(int leg_index, int move_hip,
                                           int move_knee, float wave) {
    bringup_motor_test_zero_wheels(0);

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        const int active = (i == leg_index) &&
                           bringup_mask_has((uint32_t)APP_BRINGUP_LEG_MASK, i);
        if (!active) {
            bringup_disable_motor(HIP_ID[i]);
            bringup_disable_motor(KNEE_ID[i]);
            continue;
        }

        const motor_cfg_t* hip_cfg = motor_get_cfg(HIP_ID[i]);
        const motor_cfg_t* knee_cfg = motor_get_cfg(KNEE_ID[i]);
        float hip_wave = move_hip ? wave : 0.0f;
        float knee_wave = move_knee ? wave : 0.0f;

        bringup_set_joint_if_ready(HIP_ID[i],
            bringup_joint_test_target(hip_cfg, MOTOR_TEST_HIP_AMP_RAD, hip_wave));
        bringup_set_joint_if_ready(KNEE_ID[i],
            bringup_joint_test_target(knee_cfg, MOTOR_TEST_KNEE_AMP_RAD, knee_wave));
    }
}

static void bringup_motor_test_apply_wheel(int wheel_index, float target_rads) {
    bringup_motor_test_zero_go();

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        motor_dev_t* wheel = motor_get(WHEEL_ID[i]);
        if (!wheel || !wheel->ops || !wheel->ops->set_velocity) {
            continue;
        }

        float target = (i == wheel_index &&
                        bringup_mask_has((uint32_t)APP_BRINGUP_WHEEL_MASK, i)) ?
                       target_rads : 0.0f;
        (void)wheel->ops->set_velocity(wheel, target);
    }
}

static void bringup_motor_fixed_test_step(uint32_t now_ms) {
    const uint32_t joint_slot_ms = MOTOR_TEST_JOINT_MOVE_MS +
                                   MOTOR_TEST_JOINT_GAP_MS +
                                   MOTOR_TEST_JOINT_MOVE_MS +
                                   MOTOR_TEST_JOINT_GAP_MS;
    const uint32_t wheel_slot_ms = MOTOR_TEST_WHEEL_MOVE_MS +
                                   MOTOR_TEST_WHEEL_STOP_MS +
                                   MOTOR_TEST_WHEEL_MOVE_MS +
                                   MOTOR_TEST_WHEEL_STOP_MS;
    const uint32_t joint_total_ms = joint_slot_ms * (uint32_t)GAIT_LEG_NUM;
    const uint32_t wheel_total_ms = wheel_slot_ms * (uint32_t)GAIT_LEG_NUM;

    if (!s_motor_test_started) {
        s_motor_test_start_ms = now_ms;
        s_motor_test_started = 1U;
        LOGI("motor fixed test start: leg_mask=0x%02x wheel_mask=0x%02x wheel_speed=%.3f",
             (unsigned)APP_BRINGUP_LEG_MASK,
             (unsigned)APP_BRINGUP_WHEEL_MASK,
             (double)APP_BRINGUP_WHEEL_JOG_RAD_S);
    }

    uint32_t elapsed = now_ms - s_motor_test_start_ms;
    s_motor_test_elapsed_ms = elapsed;
    s_motor_test_joint_wave = 0.0f;
    s_motor_test_wheel_target = 0.0f;

    if (elapsed < MOTOR_TEST_WARMUP_MS) {
        s_motor_test_phase = MOTOR_TEST_PHASE_WARMUP;
        s_motor_test_index = 0U;
        bringup_motor_test_zero_go();
        bringup_motor_test_zero_wheels(0);
        (void)motor_go_calibrate_all();
        motor_go_send_all();
        motor_m3508_send_all();
        return;
    }

    elapsed -= MOTOR_TEST_WARMUP_MS;
    (void)motor_go_calibrate_all();

    if (elapsed < joint_total_ms) {
        uint32_t leg = elapsed / joint_slot_ms;
        uint32_t t = elapsed % joint_slot_ms;
        s_motor_test_index = (uint8_t)leg;

        if (t < MOTOR_TEST_JOINT_MOVE_MS) {
            float wave = sinf(MOTOR_TEST_TWO_PI * (float)t / (float)MOTOR_TEST_JOINT_MOVE_MS);
            s_motor_test_phase = MOTOR_TEST_PHASE_HIP;
            s_motor_test_joint_wave = wave;
            bringup_motor_test_apply_joint((int)leg, 1, 0, wave);
        } else if (t < (MOTOR_TEST_JOINT_MOVE_MS + MOTOR_TEST_JOINT_GAP_MS)) {
            s_motor_test_phase = MOTOR_TEST_PHASE_JOINT_GAP;
            bringup_motor_test_apply_joint((int)leg, 0, 0, 0.0f);
        } else if (t < (MOTOR_TEST_JOINT_MOVE_MS + MOTOR_TEST_JOINT_GAP_MS + MOTOR_TEST_JOINT_MOVE_MS)) {
            uint32_t kt = t - MOTOR_TEST_JOINT_MOVE_MS - MOTOR_TEST_JOINT_GAP_MS;
            float wave = sinf(MOTOR_TEST_TWO_PI * (float)kt / (float)MOTOR_TEST_JOINT_MOVE_MS);
            s_motor_test_phase = MOTOR_TEST_PHASE_KNEE;
            s_motor_test_joint_wave = wave;
            bringup_motor_test_apply_joint((int)leg, 0, 1, wave);
        } else {
            s_motor_test_phase = MOTOR_TEST_PHASE_JOINT_GAP;
            bringup_motor_test_apply_joint((int)leg, 0, 0, 0.0f);
        }

        motor_go_send_all();
        motor_m3508_send_all();
        return;
    }

    elapsed -= joint_total_ms;
    if (elapsed < wheel_total_ms) {
        uint32_t wheel = elapsed / wheel_slot_ms;
        uint32_t t = elapsed % wheel_slot_ms;
        s_motor_test_index = (uint8_t)wheel;

        if (t < MOTOR_TEST_WHEEL_MOVE_MS) {
            s_motor_test_phase = MOTOR_TEST_PHASE_WHEEL_FWD;
            s_motor_test_wheel_target = (float)APP_BRINGUP_WHEEL_JOG_RAD_S;
            bringup_motor_test_apply_wheel((int)wheel, s_motor_test_wheel_target);
        } else if (t < (MOTOR_TEST_WHEEL_MOVE_MS + MOTOR_TEST_WHEEL_STOP_MS)) {
            s_motor_test_phase = MOTOR_TEST_PHASE_WHEEL_STOP;
            bringup_motor_test_apply_wheel((int)wheel, 0.0f);
        } else if (t < (MOTOR_TEST_WHEEL_MOVE_MS + MOTOR_TEST_WHEEL_STOP_MS + MOTOR_TEST_WHEEL_MOVE_MS)) {
            s_motor_test_phase = MOTOR_TEST_PHASE_WHEEL_REV;
            s_motor_test_wheel_target = -(float)APP_BRINGUP_WHEEL_JOG_RAD_S;
            bringup_motor_test_apply_wheel((int)wheel, s_motor_test_wheel_target);
        } else {
            s_motor_test_phase = MOTOR_TEST_PHASE_WHEEL_STOP;
            bringup_motor_test_apply_wheel((int)wheel, 0.0f);
        }

        motor_go_send_all();
        motor_m3508_send_all();
        return;
    }

    s_motor_test_phase = MOTOR_TEST_PHASE_DONE;
    s_motor_test_index = 0U;
    bringup_motor_test_zero_go();
    bringup_motor_test_zero_wheels(0);
    motor_go_send_all();
    motor_m3508_send_all();
}

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
    if (!isfinite(p->step_length_m) || p->step_length_m < 0.0f || p->step_length_m > 0.20f) {
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
    bringup_configure_leg_controller();
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
    LOGI("chassis init: mode=AUTO active=stand timeout=%ums bringup_stage=%d",
         (unsigned)s_online_timeout_ms, (int)APP_BRINGUP_STAGE);
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
    if (!bringup_allow_trot()) return APP_ERR_BUSY;
    if (p) {
        int r = task_chassis_set_trot_params(p);
        if (r != APP_OK) return r;
    }
    const gait_params_t* trot_params = bringup_is_normal() ?
                                       &s_trot_params :
                                       &S_BRINGUP_TROT_PARAMS;
    apply_controller_height(trot_params);
    int r = request_gait(s_trot, trot_params, blend_dur_s);
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
            const gait_params_t* trot_params = bringup_is_normal() ?
                                               &plan->gait_params :
                                               &S_BRINGUP_TROT_PARAMS;
            apply_controller_height(trot_params);
            if (request_gait(s_trot, trot_params, 0.3f) == APP_OK) {
                s_active = ACT_TROT;
                s_manual_gait_hold = 0U;
            }
        } else if (s_trot && s_trot->ops && s_trot->ops->set_param) {
            const gait_params_t* trot_params = bringup_is_normal() ?
                                               &plan->gait_params :
                                               &S_BRINGUP_TROT_PARAMS;
            apply_controller_height(trot_params);
            (void)s_trot->ops->set_param(s_trot, trot_params);
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
        out->leg[i].wheel_rads = plan->wheel_rads[i];
    }
}

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
}

void task_chassis_step_for_test(float dt_s, uint32_t now_ms) {
    task_comm_chassis_cmd_t cmd;
    float effective_wz = 0.0f;
    uint8_t online = 0U;

#if !APP_TARGET_HOST
    if (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_BOARD_ONLY) {
        return;
    }

    if (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_BUS_ZERO_TX) {
        motor_go_send_all();
        motor_m3508_send_all();
        return;
    }

    if (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_M3508_ZERO) {
        motor_m3508_send_all();
        return;
    }

    if (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_GO_ZERO) {
        motor_go_send_all();
        /* 在零力矩模式下计算 zero_offset，避免 GO_LEG_HOLD 切入闭环时飞踢 */
        (void)motor_go_calibrate_all();
        return;
    }

    if (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_M3508_JOG) {
        bringup_apply_wheel_jog();
        motor_m3508_send_all();
        return;
    }

    if (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_MOTOR_FIXED_TEST) {
        bringup_motor_fixed_test_step(now_ms);
        return;
    }

    /*
     * 预标定安全网：当 stage >= GO_LEG_HOLD 但未跑过 GO_ZERO 时，
     * 先在零力矩模式下收几个周期编码器反馈、算 zero_offset，避免首次闭环飞踢。
     */
    {
        static uint32_t s_pre_calib_cnt = 0;
        static uint8_t  s_pre_calib_done = 0;

        if (!s_pre_calib_done && APP_BRINGUP_STAGE >= APP_BRINGUP_STAGE_GO_LEG_HOLD) {
            motor_go_send_all();          /* 零力矩收发：mode=1, kp/kd/tau=0，收集编码器反馈 */
            if (s_pre_calib_cnt < 50U) {  /* 100ms @ 500Hz */
                s_pre_calib_cnt++;
                return;
            }
            (void)motor_go_calibrate_all();
            s_pre_calib_done = 1;
            LOGI("pre-calib: motors calibrated, entering stand");
            /* 同周期不继续：下一个周期走正常控制流程 */
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

    if (!bringup_allow_trot()) {
        s_offline_seq_active = 0U;
        s_offline_seq_done = 0U;
        if (s_active != ACT_STAND) {
            if (request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) {
                s_active = ACT_STAND;
            }
        }
    } else if (!online) {
        offline_decide(now_ms);
    } else {
        s_offline_seq_active = 0U;
        s_offline_seq_done = 0U;
        online_decide(&s_chassis_plan);
    }

    gait_output_t out;
    gait_machine_update(&s_gm, dt_s, &out);
    if (online && bringup_allow_trot()) {
        apply_plan_wheel_speed(&out, &s_chassis_plan);
    }
    leg_controller_apply(&s_lc, &out);

#if !APP_TARGET_HOST
    /* MCU 端：将电机指令推送到物理总线 */
    if (bringup_allow_go_tx()) {
        motor_go_send_all();
    }
    if (bringup_allow_m3508_tx()) {
        motor_m3508_send_all();
    }
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
