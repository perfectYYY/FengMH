/*
 * leg_controller.c
 */
#include "leg_controller.h"
#include "leg_ik.h"
#include "leg_params.h"
#include "motor_m3508.h"
#include "log.h"
#include <string.h>
#include <math.h>

static const char* TAG = "LEG";

static const motor_logical_id_t HIP[GAIT_LEG_NUM]   = { MOTOR_ID_FL_HIP, MOTOR_ID_FR_HIP, MOTOR_ID_RL_HIP, MOTOR_ID_RR_HIP };
static const motor_logical_id_t KNEE[GAIT_LEG_NUM]  = { MOTOR_ID_FL_KNEE,MOTOR_ID_FR_KNEE,MOTOR_ID_RL_KNEE,MOTOR_ID_RR_KNEE };
static const motor_logical_id_t WHEEL[GAIT_LEG_NUM] = { MOTOR_ID_FL_WHEEL,MOTOR_ID_FR_WHEEL,MOTOR_ID_RL_WHEEL,MOTOR_ID_RR_WHEEL };

/* IK z 偏置沿用老工程语义：足端在髋关节下方时 z 为负。 */
static float s_stand_height = -0.18f;
static uint8_t s_leg_mask = 0x0Fu;
static uint8_t s_enable_joints = 1U;
static uint8_t s_enable_wheels = 1U;
static float s_joint_kp = 1.5f;
static float s_joint_kd = 0.1f;
volatile leg_wheel_mit_debug_t g_leg_wheel_mit;

void leg_controller_init(leg_controller_t* lc) {
    if (!lc) return;
    memset(lc, 0, sizeof(*lc));
    memset((void*)&g_leg_wheel_mit, 0, sizeof(g_leg_wheel_mit));
    g_leg_wheel_mit.enable = 1U;
    g_leg_wheel_mit.hold_swing = 1U;
    g_leg_wheel_mit.kp = 20.0f;
    g_leg_wheel_mit.kd = 0.6f;
    g_leg_wheel_mit.tau_limit_nm = M3508_MIT_TAU_MAX_NM;
    g_leg_wheel_mit.pos_err_limit_rad = M3508_MIT_POS_ERR_MAX_RAD;
}

app_err_t leg_controller_bind_from_registry(leg_controller_t* lc) {
    if (!lc) return APP_ERR_INVALID_ARG;
    int bound = 0;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        lc->leg[i].hip   = motor_get(HIP[i]);
        lc->leg[i].knee  = motor_get(KNEE[i]);
        lc->leg[i].wheel = motor_get(WHEEL[i]);
        bound += (lc->leg[i].hip ? 1 : 0)
              +  (lc->leg[i].knee ? 1 : 0)
              +  (lc->leg[i].wheel ? 1 : 0);
    }
    LOGI("bind from registry: %d/12 actuators present", bound);
    return APP_OK;
}

void leg_controller_set_stand_height(float h) {
    s_stand_height = h;
}

void leg_controller_set_output_options(uint8_t leg_mask,
                                        uint8_t enable_joints,
                                        uint8_t enable_wheels,
                                        float joint_kp,
                                        float joint_kd) {
    s_leg_mask = (uint8_t)(leg_mask & 0x0Fu);
    s_enable_joints = enable_joints ? 1U : 0U;
    s_enable_wheels = enable_wheels ? 1U : 0U;
    s_joint_kp = (joint_kp > 0.0f) ? joint_kp : 0.0f;
    s_joint_kd = (joint_kd > 0.0f) ? joint_kd : 0.0f;
    LOGI("output options: leg_mask=0x%02x joints=%u wheels=%u kp=%.3f kd=%.3f",
         (unsigned)s_leg_mask,
         (unsigned)s_enable_joints,
         (unsigned)s_enable_wheels,
         (double)s_joint_kp,
         (double)s_joint_kd);
}

static int try_set_pos(motor_dev_t* d, float rad) {
    if (!d || !d->ops || !d->ops->set_position) return APP_ERR_UNSUPPORTED;
    return d->ops->set_position(d, rad, 0.0f, s_joint_kp, s_joint_kd, 0.0f);
}

static int try_set_wheel_mit(motor_dev_t* d,
                             float pos,
                             float vel,
                             float kp,
                             float kd,
                             float tau_limit,
                             float pos_err_limit) {
    if (!d || !d->ops || !d->ops->set_position) return APP_ERR_UNSUPPORTED;
    int ret = d->ops->set_position(d, pos, vel, kp, kd, 0.0f);
    if (ret == APP_OK) {
        (void)motor_m3508_set_mit_limits(d, tau_limit, pos_err_limit);
    }
    return ret;
}

static void wheel_mit_reset_refs(void) {
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        g_leg_wheel_mit.theta_ref_rad[i] = 0.0f;
        g_leg_wheel_mit.ref_valid[i] = 0U;
    }
    g_leg_wheel_mit.active_mask = 0U;
}

typedef struct {
    float kp;
    float kd;
    float tau_limit_nm;
    float pos_err_limit_rad;
} wheel_mit_cfg_t;

static wheel_mit_cfg_t wheel_mit_read_cfg(const volatile leg_wheel_mit_debug_t* dbg) {
    wheel_mit_cfg_t cfg;
    cfg.kp = isfinite(dbg->kp) && dbg->kp > 0.0f ? dbg->kp : 20.0f;
    cfg.kd = isfinite(dbg->kd) && dbg->kd >= 0.0f ? dbg->kd : 0.6f;
    cfg.tau_limit_nm = isfinite(dbg->tau_limit_nm) && dbg->tau_limit_nm > 0.0f
                     ? dbg->tau_limit_nm : M3508_MIT_TAU_MAX_NM;
    cfg.pos_err_limit_rad = isfinite(dbg->pos_err_limit_rad) &&
                            dbg->pos_err_limit_rad > 0.0f
                          ? dbg->pos_err_limit_rad : M3508_MIT_POS_ERR_MAX_RAD;
    return cfg;
}

static float wheel_mit_limit_dt(float dt_s) {
    if (!isfinite(dt_s) || dt_s < 0.0f) return 0.0f;
    if (dt_s > 0.02f) return 0.02f;
    return dt_s;
}

static void wheel_mit_latch_ref_if_needed(volatile leg_wheel_mit_debug_t* dbg,
                                          int leg_idx,
                                          const motor_dev_t* wheel) {
    if (!dbg->ref_valid[leg_idx]) {
        dbg->theta_ref_rad[leg_idx] = wheel->state.angle_rad;
        dbg->ref_valid[leg_idx] = 1U;
    }
}

static int wheel_mit_command(motor_dev_t* wheel,
                             int leg_idx,
                             float wheel_rads,
                             const wheel_mit_cfg_t* cfg) {
    volatile leg_wheel_mit_debug_t* dbg = &g_leg_wheel_mit;
    return try_set_wheel_mit(wheel,
                             dbg->theta_ref_rad[leg_idx],
                             wheel_rads,
                             cfg->kp,
                             cfg->kd,
                             cfg->tau_limit_nm,
                             cfg->pos_err_limit_rad);
}

static int wheel_mit_apply_stance(motor_dev_t* wheel,
                                  const gait_leg_target_t* t,
                                  int leg_idx,
                                  float dt_s,
                                  const wheel_mit_cfg_t* cfg) {
    volatile leg_wheel_mit_debug_t* dbg = &g_leg_wheel_mit;
    dbg->theta_ref_rad[leg_idx] += t->wheel_rads * dt_s;
    dbg->active_mask |= (uint8_t)(1U << leg_idx);
    return wheel_mit_command(wheel, leg_idx, t->wheel_rads, cfg);
}

static int wheel_mit_apply_swing(motor_dev_t* wheel,
                                 int leg_idx,
                                 const wheel_mit_cfg_t* cfg) {
    volatile leg_wheel_mit_debug_t* dbg = &g_leg_wheel_mit;
    dbg->theta_ref_rad[leg_idx] = wheel->state.angle_rad;
    dbg->active_mask &= (uint8_t)~(1U << leg_idx);
    dbg->hold_swing = 1U;
    return wheel_mit_command(wheel, leg_idx, 0.0f, cfg);
}

static int try_set_wheel(motor_dev_t* wheel,
                         const gait_leg_target_t* t,
                         int leg_idx,
                         float dt_s) {
    if (!wheel || !t) return APP_ERR_UNSUPPORTED;
    volatile leg_wheel_mit_debug_t* dbg = &g_leg_wheel_mit;

    if (dbg->reset) {
        wheel_mit_reset_refs();
        dbg->reset = 0U;
    }

    dbg->enable = 1U;

    wheel_mit_cfg_t cfg = wheel_mit_read_cfg(dbg);
    dt_s = wheel_mit_limit_dt(dt_s);
    wheel_mit_latch_ref_if_needed(dbg, leg_idx, wheel);

    if (t->in_stance) {
        return wheel_mit_apply_stance(wheel, t, leg_idx, dt_s, &cfg);
    }
    return wheel_mit_apply_swing(wheel, leg_idx, &cfg);
}

static uint8_t leg_output_enabled(int leg_idx) {
    return (((uint8_t)(1U << leg_idx) & s_leg_mask) != 0U) ? 1U : 0U;
}

static uint8_t leg_has_required_actuators(const leg_actuators_t* leg) {
    if (!leg) return 0U;
    if (s_enable_joints && (!leg->hip || !leg->knee)) return 0U;
    if (s_enable_wheels && !leg->wheel) return 0U;
    return 1U;
}

static void send_joint_targets(const leg_actuators_t* leg,
                               const gait_leg_target_t* target) {
    if (!s_enable_joints) return;
    try_set_pos(leg->hip, target->hip_rad);
    try_set_pos(leg->knee, target->knee_rad);
}

static void send_wheel_target(const leg_actuators_t* leg,
                              const gait_leg_target_t* target,
                              int leg_idx,
                              float dt_s) {
    if (!s_enable_wheels) return;
    try_set_wheel(leg->wheel, target, leg_idx, dt_s);
}

static void send_leg_targets(const leg_actuators_t* leg,
                             const gait_leg_target_t* target,
                             int leg_idx,
                             float dt_s) {
    send_joint_targets(leg, target);
    send_wheel_target(leg, target, leg_idx, dt_s);
}

app_err_t leg_controller_apply_dt(leg_controller_t* lc, const gait_output_t* o, float dt_s) {
    if (!lc || !o) return APP_ERR_INVALID_ARG;

    /* IK 解算：将步态输出的足端位移 (dx, dz) 转换为关节角度 (theta1, theta2) */
    gait_output_t ik_out;
    leg_ik_solve_all(o, &LEG_DIM_DEFAULT, s_stand_height, &ik_out);

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        const gait_leg_target_t* t = &ik_out.leg[i];
        leg_actuators_t* leg = &lc->leg[i];
        if (!leg_output_enabled(i)) {
            continue;
        }
        if (!leg_has_required_actuators(leg)) {
            lc->miss_cnt++;
            continue;
        }
        send_leg_targets(leg, t, i, dt_s);
        lc->send_cnt++;
    }
    return APP_OK;
}

app_err_t leg_controller_apply(leg_controller_t* lc, const gait_output_t* o) {
    return leg_controller_apply_dt(lc, o, 0.002f);
}
