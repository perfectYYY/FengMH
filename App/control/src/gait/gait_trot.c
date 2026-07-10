/*
 * gait_trot.c — 对角小跑相位发生器 + 足端轨迹
 *
 * 时间推进：每次 update(dt) 累加主相位 phase += dt/period，并 wrap 到 [0,1)。
 * 单腿相位 leg_phase = wrap(phase + offset_i)。
 * 在每条腿的 leg_phase 内：
 *   leg_phase < duty   → 支撑相：足端 x 从 +step/2 线性扫到 -step/2，z 维持 0
 *   leg_phase >= duty  → 摆动相：足端按摆线从 -step/2 抬到 +step/2
 * yaw 转向优先使用 planner 写入的每腿步长；未写入时回退到 step_length_m ± turn_step_m。
 *
 * 该层不做 IK。它输出 body-frame 足端位移 foot_x_m / foot_z_m；
 * leg_controller 后续接 IK 转换为关节角。
 */
#include "gait_trot.h"
#include "gait_trajectory.h"
#include "log.h"
#include <string.h>

static const char* TAG = "GAIT";

typedef struct {
    gait_if_t base;
    float     phase;
} trot_ctx_t;

static trot_ctx_t s_inst;

static int trot_init(gait_if_t* self) {
    if (!self) return APP_ERR_INVALID_ARG;
    trot_ctx_t* c = (trot_ctx_t*)self;
    c->phase = 0.0f;
    return APP_OK;
}

static int trot_set_param(gait_if_t* self, const gait_params_t* p) {
    if (!self || !p) return APP_ERR_INVALID_ARG;
    if (p->period_s <= 1e-6f) return APP_ERR_INVALID_ARG;
    if (p->duty < 0.0f || p->duty > 1.0f) return APP_ERR_INVALID_ARG;
    self->params = *p;
    return APP_OK;
}

void gait_trot_foot_traj(float leg_phase, float duty,
                         float step_len_m, float step_height_m,
                         float* dx_m, float* dz_m, uint8_t* in_stance) {
    gait_cycloid_foot_traj(leg_phase,
                           duty,
                           step_len_m,
                           step_height_m,
                           dx_m,
                           dz_m,
                           in_stance);
}

static int trot_update(gait_if_t* self, float dt_s, gait_output_t* out) {
    if (!self || !out) return APP_ERR_INVALID_ARG;
    trot_ctx_t* c = (trot_ctx_t*)self;
    const gait_params_t* p = &self->params;

    if (p->period_s <= 1e-6f) return APP_ERR_UNINIT;
    if (dt_s < 0.0f) dt_s = 0.0f;

    c->phase = gait_wrap01(c->phase + dt_s / p->period_s);

    memset(out, 0, sizeof(*out));
    out->phase = c->phase;
    out->tick_count = 1; /* 由调用方累加；这里仅占位 */
    out->wheel_mode = GAIT_WHEEL_DRIVE;

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        float lp = gait_wrap01(c->phase + p->phase_offset[i]);
        float leg_step = gait_resolve_leg_step_length(p, i);
        float dx, dz; uint8_t st;
        gait_trot_foot_traj(lp, p->duty, leg_step, p->step_height_m, &dx, &dz, &st);
        out->leg[i].in_stance  = st;
        out->leg[i].foot_x_m   = dx;
        out->leg[i].foot_z_m   = dz;
        /* 兼容旧调试工具；IK 当前读取 foot_x_m/foot_z_m。 */
        out->leg[i].hip_rad    = dx;
        out->leg[i].knee_rad   = dz;
        /* 手动 gait 的粗略轮速也跨相位连续；在线模式会由 planner 覆盖。 */
        out->leg[i].wheel_rads = -leg_step / (p->period_s * p->duty + 1e-9f);
    }
    return APP_OK;
}

static int trot_exit(gait_if_t* self) { (void)self; return APP_OK; }
static const char* trot_name(void) { return "trot"; }

static const gait_ops_t s_ops = {
    .init = trot_init, .set_param = trot_set_param,
    .update = trot_update, .exit = trot_exit, .name = trot_name,
};

gait_if_t* gait_trot_create(void) {
    memset(&s_inst, 0, sizeof(s_inst));
    s_inst.base.ops = &s_ops;
    LOGI("trot created");
    return &s_inst.base;
}
