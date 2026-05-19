/*
 * gait_machine.c
 */
#include "gait_machine.h"
#include "log.h"
#include <string.h>

static const char* TAG = "GAIT";

static float lerp(float a, float b, float t) { return a + (b - a) * t; }

static void blend_outputs(const gait_output_t* a, const gait_output_t* b,
                          float t, gait_output_t* o) {
    o->phase = b->phase;  /* 主相位以目标步态为准 */
    o->tick_count = b->tick_count;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        o->leg[i].hip_rad    = lerp(a->leg[i].hip_rad,    b->leg[i].hip_rad,    t);
        o->leg[i].knee_rad   = lerp(a->leg[i].knee_rad,   b->leg[i].knee_rad,   t);
        o->leg[i].wheel_rads = lerp(a->leg[i].wheel_rads, b->leg[i].wheel_rads, t);
        /* in_stance 取目标值；过渡期保守，避免抖动 */
        o->leg[i].in_stance  = b->leg[i].in_stance;
    }
}

void gait_machine_init(gait_machine_t* m) {
    if (!m) return;
    memset(m, 0, sizeof(*m));
    m->state = GM_STATE_IDLE;
}

app_err_t gait_machine_set(gait_machine_t* m, gait_if_t* g, const gait_params_t* p) {
    if (!m || !g || !p || !g->ops) return APP_ERR_INVALID_ARG;
    if (g->ops->set_param) g->ops->set_param(g, p);
    if (g->ops->init) g->ops->init(g);
    m->current = g;
    m->target = NULL;
    m->blend_dur_s = m->blend_t_s = 0.0f;
    m->state = GM_STATE_RUN;
    LOGI("set %s", g->ops->name ? g->ops->name() : "?");
    return APP_OK;
}

app_err_t gait_machine_request(gait_machine_t* m, gait_if_t* g,
                               const gait_params_t* p, float blend_dur_s) {
    if (!m || !g || !p || !g->ops) return APP_ERR_INVALID_ARG;
    if (m->state != GM_STATE_RUN) return APP_ERR_BUSY;
    if (g == m->current) return APP_OK;
    if (blend_dur_s < 0.0f) blend_dur_s = 0.0f;
    if (g->ops->set_param) g->ops->set_param(g, p);
    if (g->ops->init) g->ops->init(g);
    m->target = g;
    m->blend_dur_s = blend_dur_s;
    m->blend_t_s = 0.0f;
    m->state = (blend_dur_s > 0.0f) ? GM_STATE_BLEND : GM_STATE_RUN;
    if (m->state == GM_STATE_RUN) {
        if (m->current && m->current->ops && m->current->ops->exit) m->current->ops->exit(m->current);
        m->current = g;
        m->target = NULL;
    }
    LOGI("request %s blend=%.3fs", g->ops->name ? g->ops->name() : "?", (double)blend_dur_s);
    return APP_OK;
}

app_err_t gait_machine_update(gait_machine_t* m, float dt_s, gait_output_t* out) {
    if (!m || !out) return APP_ERR_INVALID_ARG;
    if (!m->current) { memset(out, 0, sizeof(*out)); return APP_ERR_UNINIT; }

    if (m->state == GM_STATE_RUN || !m->target) {
        return m->current->ops->update(m->current, dt_s, out);
    }

    /* BLEND */
    m->current->ops->update(m->current, dt_s, &m->buf_a);
    m->target ->ops->update(m->target,  dt_s, &m->buf_b);
    m->blend_t_s += dt_s;
    float t = (m->blend_dur_s > 0.0f) ? (m->blend_t_s / m->blend_dur_s) : 1.0f;
    if (t >= 1.0f) {
        if (m->current->ops->exit) m->current->ops->exit(m->current);
        m->current = m->target;
        m->target = NULL;
        m->state = GM_STATE_RUN;
        *out = m->buf_b;
        return APP_OK;
    }
    blend_outputs(&m->buf_a, &m->buf_b, t, out);
    return APP_OK;
}

gait_machine_state_t gait_machine_state(const gait_machine_t* m) {
    return m ? m->state : GM_STATE_IDLE;
}
