/*
 * leg_controller.c
 */
#include "leg_controller.h"
#include "config.h"
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

#define LEG_PAYLOAD_DEFAULT_HALF_LENGTH_M 0.25f
#define LEG_PAYLOAD_DEFAULT_HALF_TRACK_M  0.15f
#define LEG_PAYLOAD_EPS                   1e-6f
#define LEG_WHEEL_MIT_DEFAULT_KP          20.0f
#define LEG_WHEEL_MIT_DEFAULT_KD          0.6f
#define LEG_WHEEL_MIT_DEFAULT_TAU_LIMIT_NM M3508_MIT_TAU_MAX_NM
#define LEG_WHEEL_MIT_DEFAULT_STANCE_FF_N 1.0f
#define LEG_WHEEL_MIT_FF_DEADBAND_RADS    0.05f
#define LEG_WHEEL_PURE_TURN_TAU_FF_NM     3.2f

/* IK z 偏置沿用老工程语义：足端在髋关节下方时 z 为负。 */
static float s_stand_height = -0.18f;
static uint8_t s_leg_mask = 0x0Fu;
static uint8_t s_enable_joints = 1U;
static uint8_t s_enable_wheels = 1U;
static float s_joint_kp = 1.5f;
static float s_joint_kd = 0.1f;
volatile leg_wheel_mit_debug_t g_leg_wheel_mit;
volatile leg_gravity_comp_debug_t g_leg_gravity_comp;
volatile uint8_t g_fl_fixed_tau_ff_enable;
volatile float g_fl_hip_fixed_tau_ff_nm;
volatile float g_fl_knee_fixed_tau_ff_nm;

static float wheel_mit_default_stance_tau_ff_nm(void) {
    return LEG_WHEEL_MIT_DEFAULT_STANCE_FF_N * LEG_DIM_DEFAULT.wheel_diameter * 0.5f;
}

void leg_controller_init(leg_controller_t* lc) {
    if (!lc) return;
    memset(lc, 0, sizeof(*lc));
    memset((void*)&g_leg_wheel_mit, 0, sizeof(g_leg_wheel_mit));
    memset((void*)&g_leg_gravity_comp, 0, sizeof(g_leg_gravity_comp));
    g_leg_wheel_mit.enable = 1U;
    g_leg_wheel_mit.kp = LEG_WHEEL_MIT_DEFAULT_KP;
    g_leg_wheel_mit.kd = LEG_WHEEL_MIT_DEFAULT_KD;
    g_leg_wheel_mit.tau_limit_nm = LEG_WHEEL_MIT_DEFAULT_TAU_LIMIT_NM;
    g_leg_wheel_mit.pos_err_limit_rad = M3508_MIT_POS_ERR_MAX_RAD;
    g_leg_wheel_mit.stance_tau_ff_nm = wheel_mit_default_stance_tau_ff_nm();
    g_leg_wheel_mit.pure_turn_active = 0U;
    g_leg_wheel_mit.pure_turn_tau_ff_nm = LEG_WHEEL_PURE_TURN_TAU_FF_NM;
    g_leg_gravity_comp.enable = 0U;
    g_leg_gravity_comp.compensate_leg_mass = 0U;
    g_leg_gravity_comp.compensate_payload = 0U;
    g_leg_gravity_comp.use_payload_com = 0U;
    g_leg_gravity_comp.compensate_balance = 0U;
    g_leg_gravity_comp.payload_active_mask = 0U;
    g_leg_gravity_comp.balance_active_mask = 0U;
    g_leg_gravity_comp.scale = 1.0f;
    g_leg_gravity_comp.payload_mass_kg = 0.0f;
    g_leg_gravity_comp.payload_com_x_m = 0.0f;
    g_leg_gravity_comp.payload_com_y_m = 0.0f;
    g_leg_gravity_comp.support_half_length_m = LEG_PAYLOAD_DEFAULT_HALF_LENGTH_M;
    g_leg_gravity_comp.support_half_track_m = LEG_PAYLOAD_DEFAULT_HALF_TRACK_M;
    g_leg_gravity_comp.max_leg_payload_kg = 0.0f;
    g_leg_gravity_comp.payload_applied_mass_kg = 0.0f;
    g_leg_gravity_comp.balance_fz_n = 0.0f;
    g_leg_gravity_comp.balance_mx_nm = 0.0f;
    g_leg_gravity_comp.balance_my_nm = 0.0f;
    g_leg_gravity_comp.max_balance_leg_force_n = 0.0f;
    g_leg_gravity_comp.balance_applied_fz_n = 0.0f;
    g_leg_gravity_comp.balance_applied_mx_nm = 0.0f;
    g_leg_gravity_comp.balance_applied_my_nm = 0.0f;
    g_leg_gravity_comp.max_tau_nm = 3.0f;
    g_fl_fixed_tau_ff_enable = 1U;
    g_fl_hip_fixed_tau_ff_nm = 0.25f;
    g_fl_knee_fixed_tau_ff_nm = 0.50f;
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

void leg_controller_set_pure_wheel_turn(uint8_t active) {
    g_leg_wheel_mit.pure_turn_active = active ? 1U : 0U;
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

static int try_set_pos(motor_dev_t* d, float rad, float tau_ff_nm) {
    if (!d || !d->ops || !d->ops->set_position) return APP_ERR_UNSUPPORTED;
    return d->ops->set_position(d, rad, 0.0f, s_joint_kp, s_joint_kd, tau_ff_nm);
}

static int try_set_wheel_mit(motor_dev_t* d,
                             float pos,
                             float vel,
                             float kp,
                             float kd,
                             float tau_ff,
                             float tau_limit,
                             float pos_err_limit) {
    if (!d || !d->ops || !d->ops->set_position) return APP_ERR_UNSUPPORTED;
    int ret = d->ops->set_position(d, pos, vel, kp, kd, tau_ff);
    if (ret == APP_OK) {
        (void)motor_m3508_set_mit_limits(d, tau_limit, pos_err_limit);
    }
    return ret;
}

static void wheel_mit_reset_refs(void) {
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        g_leg_wheel_mit.theta_ref_rad[i] = 0.0f;
        g_leg_wheel_mit.velocity_i_rad[i] = 0.0f;
        g_leg_wheel_mit.ref_valid[i] = 0U;
    }
    g_leg_wheel_mit.drive_mask = 0U;
    g_leg_wheel_mit.hold_mask = 0U;
}

typedef struct {
    float kp;
    float kd;
    float tau_limit_nm;
    float pos_err_limit_rad;
    float stance_tau_ff_nm;
} wheel_mit_cfg_t;

static wheel_mit_cfg_t wheel_mit_read_cfg(const volatile leg_wheel_mit_debug_t* dbg) {
    wheel_mit_cfg_t cfg;
    cfg.kp = isfinite(dbg->kp) && dbg->kp > 0.0f
           ? dbg->kp : LEG_WHEEL_MIT_DEFAULT_KP;
    cfg.kd = isfinite(dbg->kd) && dbg->kd >= 0.0f
           ? dbg->kd : LEG_WHEEL_MIT_DEFAULT_KD;
    cfg.tau_limit_nm = isfinite(dbg->tau_limit_nm) && dbg->tau_limit_nm > 0.0f
                     ? dbg->tau_limit_nm : LEG_WHEEL_MIT_DEFAULT_TAU_LIMIT_NM;
    cfg.pos_err_limit_rad = isfinite(dbg->pos_err_limit_rad) &&
                            dbg->pos_err_limit_rad > 0.0f
                          ? dbg->pos_err_limit_rad : M3508_MIT_POS_ERR_MAX_RAD;
    cfg.stance_tau_ff_nm = isfinite(dbg->stance_tau_ff_nm)
                         ? fabsf(dbg->stance_tau_ff_nm)
                         : wheel_mit_default_stance_tau_ff_nm();
    if (cfg.stance_tau_ff_nm > cfg.tau_limit_nm) {
        cfg.stance_tau_ff_nm = cfg.tau_limit_nm;
    }
    return cfg;
}

static float wheel_mit_limit_dt(float dt_s) {
    if (!isfinite(dt_s) || dt_s < 0.0f) return 0.0f;
    if (dt_s > 0.02f) return 0.02f;
    return dt_s;
}

static int wheel_mit_command(motor_dev_t* wheel,
                             int leg_idx,
                             float wheel_rads,
                             float tau_ff_nm,
                             const wheel_mit_cfg_t* cfg) {
    volatile leg_wheel_mit_debug_t* dbg = &g_leg_wheel_mit;
    return try_set_wheel_mit(wheel,
                             dbg->theta_ref_rad[leg_idx],
                             wheel_rads,
                             cfg->kp,
                             cfg->kd,
                             tau_ff_nm,
                             cfg->tau_limit_nm,
                             cfg->pos_err_limit_rad);
}

static float wheel_mit_velocity_i_limit(const wheel_mit_cfg_t* cfg) {
    if (!cfg || cfg->kp <= 1e-6f) return 0.0f;
    float limit_from_tau = cfg->tau_limit_nm / cfg->kp;
    return fminf(cfg->pos_err_limit_rad, limit_from_tau);
}

static uint8_t wheel_mit_should_integrate(float velocity_error,
                                          float tau_unsat,
                                          float tau_limit) {
    if (fabsf(tau_unsat) < tau_limit) return 1U;
    if (tau_unsat >= tau_limit && velocity_error < 0.0f) return 1U;
    if (tau_unsat <= -tau_limit && velocity_error > 0.0f) return 1U;
    return 0U;
}

static int wheel_mit_apply_drive(motor_dev_t* wheel,
                                 const gait_leg_target_t* t,
                                 int leg_idx,
                                 float dt_s,
                                 const wheel_mit_cfg_t* cfg) {
    volatile leg_wheel_mit_debug_t* dbg = &g_leg_wheel_mit;
    uint8_t bit = (uint8_t)(1U << leg_idx);
    if ((dbg->drive_mask & bit) == 0U || !dbg->ref_valid[leg_idx]) {
        dbg->velocity_i_rad[leg_idx] = 0.0f;
        dbg->theta_ref_rad[leg_idx] = wheel->state.angle_rad;
        dbg->ref_valid[leg_idx] = 1U;
    }

    float tau_ff = 0.0f;
    if (dbg->pure_turn_active &&
        fabsf(t->wheel_rads) > LEG_WHEEL_MIT_FF_DEADBAND_RADS) {
        float turn_tau = isfinite(dbg->pure_turn_tau_ff_nm)
                       ? fabsf(dbg->pure_turn_tau_ff_nm) : 0.0f;
        turn_tau = fminf(turn_tau, M3508_MIT_TAU_HARD_MAX_NM);
        tau_ff = copysignf(turn_tau, t->wheel_rads);
    } else if (t->in_stance &&
               fabsf(t->wheel_rads) > LEG_WHEEL_MIT_FF_DEADBAND_RADS) {
        tau_ff = copysignf(cfg->stance_tau_ff_nm, t->wheel_rads);
    }

    float velocity_error = t->wheel_rads - wheel->state.velocity_rads;
    float tau_unsat = cfg->kp * dbg->velocity_i_rad[leg_idx]
                    + cfg->kd * velocity_error
                    + tau_ff;
    if (wheel_mit_should_integrate(velocity_error, tau_unsat, cfg->tau_limit_nm)) {
        dbg->velocity_i_rad[leg_idx] += velocity_error * dt_s;
    }

    float i_limit = wheel_mit_velocity_i_limit(cfg);
    dbg->velocity_i_rad[leg_idx] = fminf(fmaxf(dbg->velocity_i_rad[leg_idx],
                                              -i_limit),
                                              i_limit);
    dbg->theta_ref_rad[leg_idx] = wheel->state.angle_rad
                                + dbg->velocity_i_rad[leg_idx];
    dbg->drive_mask |= bit;
    dbg->hold_mask &= (uint8_t)~bit;
    return wheel_mit_command(wheel, leg_idx, t->wheel_rads, tau_ff, cfg);
}

static int wheel_mit_apply_hold(motor_dev_t* wheel,
                                int leg_idx,
                                const wheel_mit_cfg_t* cfg) {
    volatile leg_wheel_mit_debug_t* dbg = &g_leg_wheel_mit;
    uint8_t bit = (uint8_t)(1U << leg_idx);
    if ((dbg->hold_mask & bit) == 0U || !dbg->ref_valid[leg_idx]) {
        dbg->velocity_i_rad[leg_idx] = 0.0f;
        dbg->theta_ref_rad[leg_idx] = wheel->state.angle_rad;
        dbg->ref_valid[leg_idx] = 1U;
    }
    dbg->drive_mask &= (uint8_t)~bit;
    dbg->hold_mask |= bit;
    return wheel_mit_command(wheel, leg_idx, 0.0f, 0.0f, cfg);
}

static int try_set_wheel(motor_dev_t* wheel,
                         const gait_leg_target_t* t,
                         int leg_idx,
                         float dt_s,
                         gait_wheel_mode_t wheel_mode) {
    if (!wheel || !t) return APP_ERR_UNSUPPORTED;
    volatile leg_wheel_mit_debug_t* dbg = &g_leg_wheel_mit;

    if (dbg->reset) {
        wheel_mit_reset_refs();
        dbg->reset = 0U;
    }

    dbg->enable = 1U;

    wheel_mit_cfg_t cfg = wheel_mit_read_cfg(dbg);
    if (dbg->pure_turn_active && isfinite(dbg->pure_turn_tau_ff_nm)) {
        float turn_tau = fminf(fabsf(dbg->pure_turn_tau_ff_nm),
                               M3508_MIT_TAU_HARD_MAX_NM);
        if (cfg.tau_limit_nm < turn_tau) cfg.tau_limit_nm = turn_tau;
    }
    dt_s = wheel_mit_limit_dt(dt_s);

    if (wheel_mode == GAIT_WHEEL_DRIVE) {
        return wheel_mit_apply_drive(wheel, t, leg_idx, dt_s, &cfg);
    }
    return wheel_mit_apply_hold(wheel, leg_idx, &cfg);
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

static float leg_clampf(float v, float min_v, float max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static uint32_t count_stance_legs(const gait_output_t* target) {
    uint32_t n = 0U;
    if (!target) return 0U;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (target->leg[i].in_stance) n++;
    }
    return n;
}

static float gravity_comp_limit(float tau_nm) {
    float limit = g_leg_gravity_comp.max_tau_nm;
    if (!isfinite(limit) || limit <= 0.0f) limit = 3.0f;
    return leg_clampf(tau_nm, -limit, limit);
}

static float gravity_comp_scale(void) {
    float scale = g_leg_gravity_comp.scale;
    return isfinite(scale) ? scale : 0.0f;
}

static float gravity_comp_valid_positive_or_default(float value, float fallback) {
    return (isfinite(value) && value > 0.0f) ? value : fallback;
}

static float gravity_comp_leg_x_m(int leg_idx) {
    float half_length = gravity_comp_valid_positive_or_default(
        g_leg_gravity_comp.support_half_length_m,
        LEG_PAYLOAD_DEFAULT_HALF_LENGTH_M);
    return (leg_idx == GAIT_LEG_FL || leg_idx == GAIT_LEG_FR) ? half_length : -half_length;
}

static float gravity_comp_leg_y_m(int leg_idx) {
    float half_track = gravity_comp_valid_positive_or_default(
        g_leg_gravity_comp.support_half_track_m,
        LEG_PAYLOAD_DEFAULT_HALF_TRACK_M);
    return (leg_idx == GAIT_LEG_FL || leg_idx == GAIT_LEG_RL) ? half_track : -half_track;
}

static void gravity_comp_clear_payload_debug(void) {
    g_leg_gravity_comp.payload_active_mask = 0U;
    g_leg_gravity_comp.payload_applied_mass_kg = 0.0f;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        g_leg_gravity_comp.payload_leg_mass_kg[i] = 0.0f;
    }
}

static void gravity_comp_clear_balance_debug(void) {
    g_leg_gravity_comp.balance_active_mask = 0U;
    g_leg_gravity_comp.balance_applied_fz_n = 0.0f;
    g_leg_gravity_comp.balance_applied_mx_nm = 0.0f;
    g_leg_gravity_comp.balance_applied_my_nm = 0.0f;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        g_leg_gravity_comp.balance_leg_force_n[i] = 0.0f;
    }
}

static void gravity_comp_store_payload_shares(const float share_kg[GAIT_LEG_NUM],
                                              uint8_t active_mask) {
    float applied_mass = 0.0f;
    float max_leg_payload = g_leg_gravity_comp.max_leg_payload_kg;
    uint8_t use_leg_limit =
        (isfinite(max_leg_payload) && max_leg_payload > 0.0f) ? 1U : 0U;

    g_leg_gravity_comp.payload_active_mask = active_mask;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        float share = share_kg ? share_kg[i] : 0.0f;
        if (!isfinite(share) || share < 0.0f) share = 0.0f;
        if (use_leg_limit) {
            share = leg_clampf(share, 0.0f, max_leg_payload);
        }
        g_leg_gravity_comp.payload_leg_mass_kg[i] = share;
        applied_mass += share;
    }
    g_leg_gravity_comp.payload_applied_mass_kg = applied_mass;
}

static void gravity_comp_prepare_equal_payload(const gait_output_t* target,
                                               uint32_t stance_count,
                                               float payload_mass_kg) {
    float share_kg[GAIT_LEG_NUM] = {0};
    uint8_t active_mask = 0U;
    float per_leg = payload_mass_kg / (float)stance_count;

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (!target->leg[i].in_stance) continue;
        share_kg[i] = per_leg;
        active_mask |= (uint8_t)(1U << i);
    }
    gravity_comp_store_payload_shares(share_kg, active_mask);
}

static void gravity_comp_prepare_com_payload(const gait_output_t* target,
                                             uint32_t stance_count,
                                             float payload_mass_kg) {
    float raw_kg[GAIT_LEG_NUM] = {0};
    float share_kg[GAIT_LEG_NUM] = {0};
    uint8_t active_mask = 0U;
    float sum_x2 = 0.0f;
    float sum_y2 = 0.0f;
    float raw_sum = 0.0f;
    float com_x = g_leg_gravity_comp.payload_com_x_m;
    float com_y = g_leg_gravity_comp.payload_com_y_m;

    if (!isfinite(com_x)) com_x = 0.0f;
    if (!isfinite(com_y)) com_y = 0.0f;

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (!target->leg[i].in_stance) continue;
        float x = gravity_comp_leg_x_m(i);
        float y = gravity_comp_leg_y_m(i);
        sum_x2 += x * x;
        sum_y2 += y * y;
        active_mask |= (uint8_t)(1U << i);
    }

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (!target->leg[i].in_stance) continue;
        float x = gravity_comp_leg_x_m(i);
        float y = gravity_comp_leg_y_m(i);
        float share = payload_mass_kg / (float)stance_count;

        if (sum_x2 > LEG_PAYLOAD_EPS) {
            share += payload_mass_kg * com_x * x / sum_x2;
        }
        if (sum_y2 > LEG_PAYLOAD_EPS) {
            share += payload_mass_kg * com_y * y / sum_y2;
        }

        raw_kg[i] = (share > 0.0f && isfinite(share)) ? share : 0.0f;
        raw_sum += raw_kg[i];
    }

    if (raw_sum <= LEG_PAYLOAD_EPS) {
        gravity_comp_prepare_equal_payload(target, stance_count, payload_mass_kg);
        return;
    }

    float normalize = payload_mass_kg / raw_sum;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        share_kg[i] = raw_kg[i] * normalize;
    }
    gravity_comp_store_payload_shares(share_kg, active_mask);
}

static void gravity_comp_prepare_payload_shares(const gait_output_t* target,
                                                uint32_t stance_count) {
    gravity_comp_clear_payload_debug();

    if (!g_leg_gravity_comp.compensate_payload || !target || stance_count == 0U) {
        return;
    }

    float payload_mass = g_leg_gravity_comp.payload_mass_kg;
    if (!isfinite(payload_mass) || payload_mass <= 0.0f) {
        return;
    }

    if (g_leg_gravity_comp.use_payload_com) {
        gravity_comp_prepare_com_payload(target, stance_count, payload_mass);
    } else {
        gravity_comp_prepare_equal_payload(target, stance_count, payload_mass);
    }
}

static float gravity_comp_limited_balance_force(float fz_n) {
    float limit = g_leg_gravity_comp.max_balance_leg_force_n;
    if (!isfinite(fz_n)) return 0.0f;
    if (!isfinite(limit) || limit <= 0.0f) return fz_n;
    return leg_clampf(fz_n, -limit, limit);
}

static void gravity_comp_store_balance_forces(const gait_output_t* target,
                                              const float force_n[GAIT_LEG_NUM],
                                              uint8_t active_mask) {
    float applied_fz = 0.0f;
    float applied_mx = 0.0f;
    float applied_my = 0.0f;

    g_leg_gravity_comp.balance_active_mask = active_mask;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        float force = force_n ? force_n[i] : 0.0f;
        force = gravity_comp_limited_balance_force(force);
        g_leg_gravity_comp.balance_leg_force_n[i] = force;
        if (!target || !target->leg[i].in_stance) continue;

        float x = gravity_comp_leg_x_m(i);
        float y = gravity_comp_leg_y_m(i);
        applied_fz += force;
        applied_mx += y * force;
        applied_my += -x * force;
    }

    g_leg_gravity_comp.balance_applied_fz_n = applied_fz;
    g_leg_gravity_comp.balance_applied_mx_nm = applied_mx;
    g_leg_gravity_comp.balance_applied_my_nm = applied_my;
}

static void gravity_comp_prepare_balance_forces(const gait_output_t* target,
                                                uint32_t stance_count) {
    float force_n[GAIT_LEG_NUM] = {0};
    uint8_t active_mask = 0U;
    float sum_x2 = 0.0f;
    float sum_y2 = 0.0f;
    float fz = g_leg_gravity_comp.balance_fz_n;
    float mx = g_leg_gravity_comp.balance_mx_nm;
    float my = g_leg_gravity_comp.balance_my_nm;

    gravity_comp_clear_balance_debug();

    if (!g_leg_gravity_comp.compensate_balance || !target || stance_count == 0U) {
        return;
    }

    if (!isfinite(fz)) fz = 0.0f;
    if (!isfinite(mx)) mx = 0.0f;
    if (!isfinite(my)) my = 0.0f;

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (!target->leg[i].in_stance) continue;
        float x = gravity_comp_leg_x_m(i);
        float y = gravity_comp_leg_y_m(i);
        sum_x2 += x * x;
        sum_y2 += y * y;
        active_mask |= (uint8_t)(1U << i);
    }

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        if (!target->leg[i].in_stance) continue;
        float x = gravity_comp_leg_x_m(i);
        float y = gravity_comp_leg_y_m(i);
        float force = fz / (float)stance_count;

        if (sum_y2 > LEG_PAYLOAD_EPS) {
            force += mx * y / sum_y2;
        }
        if (sum_x2 > LEG_PAYLOAD_EPS) {
            force += -my * x / sum_x2;
        }
        force_n[i] = force;
    }

    gravity_comp_store_balance_forces(target, force_n, active_mask);
}

static float gravity_comp_payload_share(int leg_idx, uint8_t in_stance) {
    if (!g_leg_gravity_comp.compensate_payload || !in_stance ||
        leg_idx < 0 || leg_idx >= GAIT_LEG_NUM) {
        return 0.0f;
    }

    float share = g_leg_gravity_comp.payload_leg_mass_kg[leg_idx];
    return (isfinite(share) && share > 0.0f) ? share : 0.0f;
}

static float gravity_comp_balance_force(int leg_idx, uint8_t in_stance) {
    if (!g_leg_gravity_comp.compensate_balance || !in_stance ||
        leg_idx < 0 || leg_idx >= GAIT_LEG_NUM) {
        return 0.0f;
    }

    float force = g_leg_gravity_comp.balance_leg_force_n[leg_idx];
    return isfinite(force) ? force : 0.0f;
}

static void gravity_comp_compute(int leg_idx,
                                 const gait_leg_target_t* target,
                                 float* hip_tau_nm,
                                 float* knee_tau_nm) {
    if (!target || !hip_tau_nm || !knee_tau_nm) return;

#if !APP_LEG_TAU_FF_COMP_ENABLE
    g_leg_gravity_comp.hip_tau_ff_nm[leg_idx] = 0.0f;
    g_leg_gravity_comp.knee_tau_ff_nm[leg_idx] = 0.0f;
    *hip_tau_nm = 0.0f;
    *knee_tau_nm = 0.0f;
    return;
#endif

    const leg_dim_t* dim = &LEG_DIM_DEFAULT;
    float hip_tau = 0.0f;
    float knee_tau = 0.0f;
    float payload_share_kg;
    float vertical_force_n = 0.0f;

    if (g_leg_gravity_comp.compensate_leg_mass) {
        hip_tau += dim->g *
                   ((dim->thigh_mass_1 * dim->lc_t_m_1) +
                    (dim->thigh_mass_2 * dim->lc_t_m_2)) *
                   cosf(target->hip_rad);

        knee_tau += dim->g *
                    ((dim->shin_mass * dim->lc_s_m) +
                     (dim->link_mass * dim->lc_l_m) +
                     (dim->wheel_mass * dim->shin_length)) *
                    cosf(target->knee_rad);
    }

    payload_share_kg = gravity_comp_payload_share(leg_idx, target->in_stance);
    if (payload_share_kg > 0.0f) {
        vertical_force_n += payload_share_kg * dim->g;
    }
    vertical_force_n += gravity_comp_balance_force(leg_idx, target->in_stance);

    if (fabsf(vertical_force_n) > 1e-6f) {
        hip_tau += vertical_force_n * dim->thigh_length * cosf(target->hip_rad);
        knee_tau += vertical_force_n * dim->shin_length * cosf(target->knee_rad);
    }

    hip_tau = gravity_comp_limit(hip_tau * gravity_comp_scale());
    knee_tau = gravity_comp_limit(knee_tau * gravity_comp_scale());

    g_leg_gravity_comp.hip_tau_ff_nm[leg_idx] = hip_tau;
    g_leg_gravity_comp.knee_tau_ff_nm[leg_idx] = knee_tau;

    if (!g_leg_gravity_comp.enable) {
        hip_tau = 0.0f;
        knee_tau = 0.0f;
    }

    *hip_tau_nm = hip_tau;
    *knee_tau_nm = knee_tau;
}

static void send_joint_targets(const leg_actuators_t* leg,
                               const gait_leg_target_t* target,
                               int leg_idx) {
    if (!s_enable_joints) return;

    float hip_tau = 0.0f;
    float knee_tau = 0.0f;
    gravity_comp_compute(leg_idx, target, &hip_tau, &knee_tau);

    if (leg_idx == GAIT_LEG_FL && g_fl_fixed_tau_ff_enable) {
        float hip_fixed = isfinite(g_fl_hip_fixed_tau_ff_nm)
                        ? g_fl_hip_fixed_tau_ff_nm : 0.0f;
        float knee_fixed = isfinite(g_fl_knee_fixed_tau_ff_nm)
                         ? g_fl_knee_fixed_tau_ff_nm : 0.0f;
        hip_tau = gravity_comp_limit(hip_tau + hip_fixed);
        knee_tau = gravity_comp_limit(knee_tau + knee_fixed);
    }

    try_set_pos(leg->hip, target->hip_rad, hip_tau);
    try_set_pos(leg->knee, target->knee_rad, knee_tau);
}

static void send_wheel_target(const leg_actuators_t* leg,
                              const gait_leg_target_t* target,
                              int leg_idx,
                              float dt_s,
                              gait_wheel_mode_t wheel_mode) {
    if (!s_enable_wheels) return;
    try_set_wheel(leg->wheel, target, leg_idx, dt_s, wheel_mode);
}

static void send_leg_targets(const leg_actuators_t* leg,
                             const gait_leg_target_t* target,
                             int leg_idx,
                             float dt_s,
                             gait_wheel_mode_t wheel_mode) {
    send_joint_targets(leg, target, leg_idx);
    send_wheel_target(leg, target, leg_idx, dt_s, wheel_mode);
}

app_err_t leg_controller_apply_dt(leg_controller_t* lc, const gait_output_t* o, float dt_s) {
    if (!lc || !o) return APP_ERR_INVALID_ARG;

    /* IK 解算：将步态输出的足端位移 (dx, dz) 转换为关节角度 (theta1, theta2) */
    gait_output_t ik_out;
    leg_ik_solve_all(o, &LEG_DIM_DEFAULT, s_stand_height, &ik_out);
    uint32_t stance_count = count_stance_legs(&ik_out);
    gravity_comp_prepare_payload_shares(&ik_out, stance_count);
    gravity_comp_prepare_balance_forces(&ik_out, stance_count);

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
        send_leg_targets(leg, t, i, dt_s, ik_out.wheel_mode);
        lc->send_cnt++;
    }
    return APP_OK;
}

app_err_t leg_controller_apply(leg_controller_t* lc, const gait_output_t* o) {
    return leg_controller_apply_dt(lc, o, 0.002f);
}
