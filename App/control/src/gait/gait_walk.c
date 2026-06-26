/*
 * gait_walk.c - 四拍三支撑 walk 相位发生器
 *
 * 相位编排：
 *   FL -> RR -> FR -> RL 依次进入摆动相。
 * 默认 duty=0.75 时，任意主相位只有 1 条腿摆动、3 条腿支撑。
 *
 * 该层只输出机体系足端位移与轮速，不做 IK、不触碰外设。
 */
#include "gait_walk.h"
#include "gait_trajectory.h"
#include "log.h"

#include <string.h>

static const char* TAG = "GAIT";

typedef struct {
    gait_if_t base;
    float     phase;
} walk_ctx_t;

static walk_ctx_t s_inst;

static int walk_init(gait_if_t* self) {
    if (!self) return APP_ERR_INVALID_ARG;
    walk_ctx_t* c = (walk_ctx_t*)self;
    c->phase = 0.0f;
    return APP_OK;
}

static int walk_set_param(gait_if_t* self, const gait_params_t* p) {
    if (!self || !p) return APP_ERR_INVALID_ARG;
    if (p->period_s <= 1e-6f) return APP_ERR_INVALID_ARG;
    if (p->duty < 0.0f || p->duty > 1.0f) return APP_ERR_INVALID_ARG;
    self->params = *p;
    return APP_OK;
}

void gait_walk_foot_traj(float leg_phase,
                         float duty,
                         float step_len_m,
                         float step_height_m,
                         float* dx_m,
                         float* dz_m,
                         uint8_t* in_stance) {
    gait_cycloid_foot_traj(leg_phase,
                           duty,
                           step_len_m,
                           step_height_m,
                           dx_m,
                           dz_m,
                           in_stance);
}

static int walk_update(gait_if_t* self, float dt_s, gait_output_t* out) {
    if (!self || !out) return APP_ERR_INVALID_ARG;
    walk_ctx_t* c = (walk_ctx_t*)self;
    const gait_params_t* p = &self->params;

    if (p->period_s <= 1e-6f) return APP_ERR_UNINIT;
    if (dt_s < 0.0f) dt_s = 0.0f;

    c->phase = gait_wrap01(c->phase + dt_s / p->period_s);

    memset(out, 0, sizeof(*out));
    out->phase = c->phase;
    out->tick_count = 1;

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        float lp = gait_wrap01(c->phase + p->phase_offset[i]);
        float leg_step = gait_resolve_leg_step_length(p, i);
        float dx, dz;
        uint8_t st;
        gait_walk_foot_traj(lp, p->duty, leg_step, p->step_height_m, &dx, &dz, &st);

        out->leg[i].in_stance = st;
        out->leg[i].foot_x_m = dx;
        out->leg[i].foot_z_m = dz;
        out->leg[i].hip_rad = dx;
        out->leg[i].knee_rad = dz;
        out->leg[i].wheel_rads = (st ? (-leg_step / (p->period_s * p->duty + 1e-9f)) : 0.0f);
    }
    return APP_OK;
}

static int walk_exit(gait_if_t* self) { (void)self; return APP_OK; }
static const char* walk_name(void) { return "walk"; }

static const gait_ops_t s_ops = {
    .init = walk_init,
    .set_param = walk_set_param,
    .update = walk_update,
    .exit = walk_exit,
    .name = walk_name,
};

gait_if_t* gait_walk_create(void) {
    memset(&s_inst, 0, sizeof(s_inst));
    s_inst.base.ops = &s_ops;
    LOGI("walk created");
    return &s_inst.base;
}
