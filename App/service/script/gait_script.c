/*
 * gait_script.c
 */
#include "gait_script.h"
#include "log.h"
#include <string.h>

static const char* TAG = "GAIT";

typedef struct {
    gait_if_t       base;
    script_player_t player;
} script_ctx_t;

static script_ctx_t s_inst;

static int gs_init(gait_if_t* self) {
    if (!self) return APP_ERR_INVALID_ARG;
    script_ctx_t* c = (script_ctx_t*)self;
    /* 若已 load 过脚本则从头开始；否则维持 IDLE 等待 set_script */
    if (c->player.s) {
        script_player_reset(&c->player);
    }
    return APP_OK;
}

static int gs_set_param(gait_if_t* self, const gait_params_t* p) {
    if (!self || !p) return APP_ERR_INVALID_ARG;
    self->params = *p;
    return APP_OK;
}

static int gs_update(gait_if_t* self, float dt_s, gait_output_t* out) {
    if (!self || !out) return APP_ERR_INVALID_ARG;
    script_ctx_t* c = (script_ctx_t*)self;
    if (!c->player.s) {
        /* 脚本没装：输出全 0 + in_stance=1，保守的"站立保持" */
        memset(out, 0, sizeof(*out));
        for (int i = 0; i < GAIT_LEG_NUM; i++) out->leg[i].in_stance = 1u;
        return APP_ERR_UNINIT;
    }
    return script_player_update(&c->player, dt_s, out);
}

static int gs_exit(gait_if_t* self) { (void)self; return APP_OK; }
static const char* gs_name(void) { return "script"; }

static const gait_ops_t s_ops = {
    .init = gs_init, .set_param = gs_set_param,
    .update = gs_update, .exit = gs_exit, .name = gs_name,
};

gait_if_t* gait_script_create(void) {
    memset(&s_inst, 0, sizeof(s_inst));
    s_inst.base.ops = &s_ops;
    script_player_init(&s_inst.player);
    LOGI("script gait created");
    return &s_inst.base;
}

app_err_t gait_script_set_script(gait_if_t* self, const script_t* s) {
    if (!self) return APP_ERR_INVALID_ARG;
    script_ctx_t* c = (script_ctx_t*)self;
    return script_player_load(&c->player, s);
}

app_err_t gait_script_rewind(gait_if_t* self) {
    if (!self) return APP_ERR_INVALID_ARG;
    script_ctx_t* c = (script_ctx_t*)self;
    return script_player_reset(&c->player);
}

script_player_state_t gait_script_state(const gait_if_t* self) {
    if (!self) return SP_STATE_IDLE;
    const script_ctx_t* c = (const script_ctx_t*)self;
    return script_player_state(&c->player);
}
