/*
 * gait_trot.c — 对角小跑相位发生器 + 足端轨迹
 *
 * 时间推进：每次 update(dt) 累加主相位 phase += dt/period，并 wrap 到 [0,1)。
 * 单腿相位 leg_phase = wrap(phase + offset_i)。
 * 在每条腿的 leg_phase 内：
 *   leg_phase < duty   → 支撑相：足端 x 从 +step/2 线性扫到 -step/2，z 维持 0
 *   leg_phase >= duty  → 摆动相：足端按摆线从 -step/2 抬到 +step/2
 *
 * 该层不做 IK。把 (dx, dz) 写到 leg_target 的 hip/knee 字段供上层调试观察；
 * leg_controller 后续接 IK 转换为关节角。
 */
#include "gait_trot.h"
#include "log.h"
#include <string.h>
#include <math.h>

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
    float lp = gait_wrap01(leg_phase);
    if (duty <= 0.0f) duty = 0.0f;
    if (duty >= 1.0f) duty = 1.0f;

    if (lp < duty) {
        /* 支撑相：足端在地面，x 从 +half 线性退到 -half */
        float t = (duty > 0.0f) ? (lp / duty) : 0.0f;
        if (dx_m) *dx_m = step_len_m * (0.5f - t);
        if (dz_m) *dz_m = 0.0f;
        if (in_stance) *in_stance = 1u;
    } else {
        /* 摆动相摆线：起落足处 x/z 速度都为 0。 */
        float swing_dur = 1.0f - duty;
        float t = (swing_dur > 0.0f) ? ((lp - duty) / swing_dur) : 0.0f;
        float theta = 2.0f * (float)M_PI * t;
        if (dx_m) {
            *dx_m = step_len_m * (t - (sinf(theta) / (2.0f * (float)M_PI)) - 0.5f);
        }
        if (dz_m) {
            *dz_m = 0.5f * step_height_m * (1.0f - cosf(theta));
        }
        if (in_stance) *in_stance = 0u;
    }
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

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        float lp = gait_wrap01(c->phase + p->phase_offset[i]);
        float dx, dz; uint8_t st;
        gait_trot_foot_traj(lp, p->duty, p->step_length_m, p->step_height_m, &dx, &dz, &st);
        out->leg[i].in_stance  = st;
        /* 临时把 dx/dz 寄到 hip/knee 通道，供调试与 host 测试断言；
           等 leg_controller 接入后会替换为真正的关节角度 */
        out->leg[i].hip_rad    = dx;
        out->leg[i].knee_rad   = dz;
        /* 轮速：支撑相用 dx 方向（不在本层做完整里程，只给一个粗略前向轮速） */
        out->leg[i].wheel_rads = (st ? (-p->step_length_m / (p->period_s * p->duty + 1e-9f)) : 0.0f);
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
