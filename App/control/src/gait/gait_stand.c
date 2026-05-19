/*
 * gait_stand.c — 站立步态实现
 *
 * 不调 IK，只把每条腿目标角度 / 轮速归零（等 leg_controller 后续接 IK 把 height 转角度）。
 * 重要的是 in_stance=1 与 wheel_rads=0；上层即可应用重力补偿与制动。
 */
#include "gait_stand.h"
#include "log.h"
#include <string.h>

static const char* TAG = "GAIT";

typedef struct {
    gait_if_t base;
} stand_ctx_t;

static stand_ctx_t s_inst;

static int stand_init(gait_if_t* self) {
    (void)self;
    return APP_OK;
}

static int stand_set_param(gait_if_t* self, const gait_params_t* p) {
    if (!self || !p) return APP_ERR_INVALID_ARG;
    self->params = *p;
    return APP_OK;
}

static int stand_update(gait_if_t* self, float dt_s, gait_output_t* out) {
    if (!self || !out) return APP_ERR_INVALID_ARG;
    (void)dt_s;
    memset(out, 0, sizeof(*out));
    out->phase = 0.0f;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        out->leg[i].in_stance  = 1u;
        out->leg[i].wheel_rads = 0.0f;
        /* hip/knee 留 0：M3 阶段 leg_controller 接 IK 后用 body_height_m 反解。 */
    }
    return APP_OK;
}

static int stand_exit(gait_if_t* self) { (void)self; return APP_OK; }
static const char* stand_name(void) { return "stand"; }

static const gait_ops_t s_ops = {
    .init = stand_init, .set_param = stand_set_param,
    .update = stand_update, .exit = stand_exit, .name = stand_name,
};

gait_if_t* gait_stand_create(void) {
    memset(&s_inst, 0, sizeof(s_inst));
    s_inst.base.ops = &s_ops;
    LOGI("stand created");
    return &s_inst.base;
}
