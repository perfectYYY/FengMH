/*
 * script_player.c — 关键帧线性插值
 *
 * 算法：
 *   - 找到 t 落在哪两个 key (k_i, k_{i+1}) 之间
 *   - alpha = (t - k_i.t) / (k_{i+1}.t - k_i.t)
 *   - 浮点字段做 lerp，in_stance 取较新的 k_{i+1}
 *   - t < frames[0].t_s → 输出第 0 帧
 *   - t >= frames[n-1].t_s → 输出末帧；非 loop 设 DONE，loop 则 t 取模回放
 *   - phase 输出 = t / total_dur（[0,1]，便于上层观察）
 */
#include "script_player.h"
#include "log.h"
#include <string.h>

static const char* TAG = "SCRIPT";

static float lerp(float a, float b, float t) { return a + (b - a) * t; }

static void normalize_legacy_foot_fields(gait_leg_target_t* leg) {
    if (!leg) return;
    if (leg->foot_x_m == 0.0f && leg->foot_z_m == 0.0f &&
        (leg->hip_rad != 0.0f || leg->knee_rad != 0.0f)) {
        leg->foot_x_m = leg->hip_rad;
        leg->foot_z_m = leg->knee_rad;
    }
}

static void blend_kf(const script_keyframe_t* a, const script_keyframe_t* b,
                     float alpha, gait_output_t* out) {
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        gait_leg_target_t leg_a = a->leg[i];
        gait_leg_target_t leg_b = b->leg[i];
        normalize_legacy_foot_fields(&leg_a);
        normalize_legacy_foot_fields(&leg_b);

        out->leg[i].foot_x_m   = lerp(leg_a.foot_x_m,   leg_b.foot_x_m,   alpha);
        out->leg[i].foot_z_m   = lerp(leg_a.foot_z_m,   leg_b.foot_z_m,   alpha);
        out->leg[i].hip_rad    = lerp(a->leg[i].hip_rad,    b->leg[i].hip_rad,    alpha);
        out->leg[i].knee_rad   = lerp(a->leg[i].knee_rad,   b->leg[i].knee_rad,   alpha);
        out->leg[i].wheel_rads = lerp(a->leg[i].wheel_rads, b->leg[i].wheel_rads, alpha);
        out->leg[i].in_stance  = b->leg[i].in_stance;
    }
}

app_err_t script_sample(const script_t* s, float t_s, gait_output_t* out) {
    if (!s || !out || !s->frames || s->n_frames == 0) return APP_ERR_INVALID_ARG;
    memset(out, 0, sizeof(*out));
    out->wheel_mode = GAIT_WHEEL_DRIVE;

    float total = s->frames[s->n_frames - 1].t_s;
    if (s->loop && total > 1e-6f) {
        float u = t_s / total;
        u -= (float)((int)u);
        if (u < 0.0f) u += 1.0f;
        t_s = u * total;
    }

    if (t_s <= s->frames[0].t_s) {
        for (int i = 0; i < GAIT_LEG_NUM; i++) out->leg[i] = s->frames[0].leg[i];
        for (int i = 0; i < GAIT_LEG_NUM; i++) normalize_legacy_foot_fields(&out->leg[i]);
        out->phase = 0.0f;
        return APP_OK;
    }
    if (t_s >= total) {
        for (int i = 0; i < GAIT_LEG_NUM; i++) out->leg[i] = s->frames[s->n_frames - 1].leg[i];
        for (int i = 0; i < GAIT_LEG_NUM; i++) normalize_legacy_foot_fields(&out->leg[i]);
        out->phase = 1.0f;
        return APP_OK;
    }

    /* 线性查找：脚本帧数通常很小（< 100），无需二分 */
    uint16_t idx = 0;
    for (uint16_t i = 1; i < s->n_frames; i++) {
        if (s->frames[i].t_s >= t_s) { idx = i; break; }
    }
    const script_keyframe_t* a = &s->frames[idx - 1];
    const script_keyframe_t* b = &s->frames[idx];
    float dur = b->t_s - a->t_s;
    float alpha = (dur > 1e-6f) ? ((t_s - a->t_s) / dur) : 0.0f;
    blend_kf(a, b, alpha, out);
    out->phase = (total > 1e-6f) ? (t_s / total) : 0.0f;
    return APP_OK;
}

void script_player_init(script_player_t* p) {
    if (!p) return;
    memset(p, 0, sizeof(*p));
}

app_err_t script_player_load(script_player_t* p, const script_t* s) {
    if (!p || !s || !s->frames || s->n_frames == 0) return APP_ERR_INVALID_ARG;
    p->s = s;
    p->t_s = 0.0f;
    p->tick_count = 0;
    p->state = SP_STATE_RUNNING;
    LOGI("load script '%s' frames=%u loop=%u",
         s->name ? s->name : "?", (unsigned)s->n_frames, (unsigned)s->loop);
    return APP_OK;
}

app_err_t script_player_reset(script_player_t* p) {
    if (!p || !p->s) return APP_ERR_INVALID_ARG;
    p->t_s = 0.0f;
    p->tick_count = 0;
    p->state = SP_STATE_RUNNING;
    return APP_OK;
}

app_err_t script_player_update(script_player_t* p, float dt_s, gait_output_t* out) {
    if (!p || !out) return APP_ERR_INVALID_ARG;
    if (!p->s) { memset(out, 0, sizeof(*out)); return APP_ERR_UNINIT; }
    if (dt_s < 0.0f) dt_s = 0.0f;

    if (p->state == SP_STATE_RUNNING) {
        p->t_s += dt_s;
        p->tick_count++;
        float total = p->s->frames[p->s->n_frames - 1].t_s;
        if (!p->s->loop && p->t_s >= total) {
            p->state = SP_STATE_DONE;
            LOGI("script '%s' done", p->s->name ? p->s->name : "?");
        }
    }
    app_err_t r = script_sample(p->s, p->t_s, out);
    out->tick_count = p->tick_count;
    return r;
}

script_player_state_t script_player_state(const script_player_t* p) {
    return p ? p->state : SP_STATE_IDLE;
}
