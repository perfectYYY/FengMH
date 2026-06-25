/*
 * script_builtin.c — 内置脚本实现
 *
 * 这里写的 keyframe 对应"不需要任何上位机参与"的手工步态。
 * 用户后续可直接编辑/追加新 script_t 变量；接口对上层保持一致。
 */
#include "script_builtin.h"
#include <string.h>

#define FOOT_TARGET(_x, _z, _wheel, _stance) \
    { .foot_x_m = (_x), .foot_z_m = (_z), .wheel_rads = (_wheel), .in_stance = (_stance) }

/* ---- 1. STAND_HOLD : 单帧，所有腿 in_stance=1，目标 0 ---- */
static const script_keyframe_t K_STAND_HOLD[] = {
    { 0.0f, {
        FOOT_TARGET(0, 0, 0, 1),
        FOOT_TARGET(0, 0, 0, 1),
        FOOT_TARGET(0, 0, 0, 1),
        FOOT_TARGET(0, 0, 0, 1),
    }},
    { 1.0f, {
        FOOT_TARGET(0, 0, 0, 1),
        FOOT_TARGET(0, 0, 0, 1),
        FOOT_TARGET(0, 0, 0, 1),
        FOOT_TARGET(0, 0, 0, 1),
    }},
};
const script_t SCRIPT_BUILTIN_STAND_HOLD = {
    .name = "stand_hold",
    .frames = K_STAND_HOLD,
    .n_frames = sizeof(K_STAND_HOLD)/sizeof(K_STAND_HOLD[0]),
    .loop = 1,
};

/* ---- 2. WAVE_UP_DOWN : 四腿同时按正弦抬放（等价旧 up_down[9]） ----
 * z 量写到 foot_z_m 表示足端抬腿高度，
 * 周期 0.9s（对应 9 帧 × 100ms），loop 连续；wheel=0, in_stance 摆动期 0。
 */
#define WAVE_KNEE_AMP  0.05f
static const script_keyframe_t K_WAVE[] = {
    { 0.0f, {FOOT_TARGET(0, 0.00f, 0, 1), FOOT_TARGET(0, 0.00f, 0, 1), FOOT_TARGET(0, 0.00f, 0, 1), FOOT_TARGET(0, 0.00f, 0, 1)} },
    { 0.1f, {FOOT_TARGET(0, 0.01f, 0, 0), FOOT_TARGET(0, 0.01f, 0, 0), FOOT_TARGET(0, 0.01f, 0, 0), FOOT_TARGET(0, 0.01f, 0, 0)} },
    { 0.2f, {FOOT_TARGET(0, 0.03f, 0, 0), FOOT_TARGET(0, 0.03f, 0, 0), FOOT_TARGET(0, 0.03f, 0, 0), FOOT_TARGET(0, 0.03f, 0, 0)} },
    { 0.3f, {FOOT_TARGET(0, 0.04f, 0, 0), FOOT_TARGET(0, 0.04f, 0, 0), FOOT_TARGET(0, 0.04f, 0, 0), FOOT_TARGET(0, 0.04f, 0, 0)} },
    { 0.4f, {FOOT_TARGET(0, 0.05f, 0, 0), FOOT_TARGET(0, 0.05f, 0, 0), FOOT_TARGET(0, 0.05f, 0, 0), FOOT_TARGET(0, 0.05f, 0, 0)} },
    { 0.5f, {FOOT_TARGET(0, 0.04f, 0, 0), FOOT_TARGET(0, 0.04f, 0, 0), FOOT_TARGET(0, 0.04f, 0, 0), FOOT_TARGET(0, 0.04f, 0, 0)} },
    { 0.6f, {FOOT_TARGET(0, 0.03f, 0, 0), FOOT_TARGET(0, 0.03f, 0, 0), FOOT_TARGET(0, 0.03f, 0, 0), FOOT_TARGET(0, 0.03f, 0, 0)} },
    { 0.7f, {FOOT_TARGET(0, 0.01f, 0, 0), FOOT_TARGET(0, 0.01f, 0, 0), FOOT_TARGET(0, 0.01f, 0, 0), FOOT_TARGET(0, 0.01f, 0, 0)} },
    { 0.8f, {FOOT_TARGET(0, 0.00f, 0, 1), FOOT_TARGET(0, 0.00f, 0, 1), FOOT_TARGET(0, 0.00f, 0, 1), FOOT_TARGET(0, 0.00f, 0, 1)} },
    { 0.9f, {FOOT_TARGET(0, 0.00f, 0, 1), FOOT_TARGET(0, 0.00f, 0, 1), FOOT_TARGET(0, 0.00f, 0, 1), FOOT_TARGET(0, 0.00f, 0, 1)} },
};
const script_t SCRIPT_BUILTIN_WAVE_UP_DOWN = {
    .name = "wave_up_down",
    .frames = K_WAVE,
    .n_frames = sizeof(K_WAVE)/sizeof(K_WAVE[0]),
    .loop = 1,
};

/* ---- 3. TROT_STEP : 对角 trot 4 关键帧 loop ---- */
/* FL/RR 同相支撑，FR/RL 摆动；然后交换。foot_x_m/foot_z_m 承载足端位移。 */
static const script_keyframe_t K_TROT[] = {
    { 0.00f, {
        FOOT_TARGET(+0.03f, 0.00f, 0.0f, 1),  /* FL stance, 前 */
        FOOT_TARGET(-0.03f, 0.02f, 0.0f, 0),  /* FR swing, 后上 */
        FOOT_TARGET(-0.03f, 0.02f, 0.0f, 0),  /* RL swing */
        FOOT_TARGET(+0.03f, 0.00f, 0.0f, 1),  /* RR stance */
    }},
    { 0.20f, {
        FOOT_TARGET(-0.03f, 0.00f, 0.0f, 1),
        FOOT_TARGET(+0.03f, 0.00f, 0.0f, 1),  /* FR touchdown */
        FOOT_TARGET(+0.03f, 0.00f, 0.0f, 1),
        FOOT_TARGET(-0.03f, 0.00f, 0.0f, 1),
    }},
    { 0.40f, {
        FOOT_TARGET(-0.03f, 0.02f, 0.0f, 0),
        FOOT_TARGET(+0.03f, 0.00f, 0.0f, 1),
        FOOT_TARGET(+0.03f, 0.00f, 0.0f, 1),
        FOOT_TARGET(-0.03f, 0.02f, 0.0f, 0),
    }},
    { 0.60f, {
        FOOT_TARGET(+0.03f, 0.00f, 0.0f, 1),
        FOOT_TARGET(-0.03f, 0.00f, 0.0f, 1),
        FOOT_TARGET(-0.03f, 0.00f, 0.0f, 1),
        FOOT_TARGET(+0.03f, 0.00f, 0.0f, 1),
    }},
};
const script_t SCRIPT_BUILTIN_TROT_STEP = {
    .name = "trot_step",
    .frames = K_TROT,
    .n_frames = sizeof(K_TROT)/sizeof(K_TROT[0]),
    .loop = 1,
};

static const script_t* const TABLE[] = {
    &SCRIPT_BUILTIN_STAND_HOLD,
    &SCRIPT_BUILTIN_WAVE_UP_DOWN,
    &SCRIPT_BUILTIN_TROT_STEP,
};

const script_t* script_builtin_find(const char* name) {
    if (!name) return 0;
    for (unsigned i = 0; i < sizeof(TABLE)/sizeof(TABLE[0]); i++) {
        if (TABLE[i]->name && strcmp(TABLE[i]->name, name) == 0) return TABLE[i];
    }
    return 0;
}
