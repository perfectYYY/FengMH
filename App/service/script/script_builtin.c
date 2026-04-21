/*
 * script_builtin.c — 内置脚本实现
 *
 * 这里写的 keyframe 对应"不需要任何上位机参与"的手工步态。
 * 用户后续可直接编辑/追加新 script_t 变量；接口对上层保持一致。
 */
#include "script_builtin.h"
#include <string.h>

/* ---- 1. STAND_HOLD : 单帧，所有腿 in_stance=1，目标 0 ---- */
static const script_keyframe_t K_STAND_HOLD[] = {
    { 0.0f, {
        {0,0,0,1},{0,0,0,1},{0,0,0,1},{0,0,0,1}
    }},
    { 1.0f, {
        {0,0,0,1},{0,0,0,1},{0,0,0,1},{0,0,0,1}
    }},
};
const script_t SCRIPT_BUILTIN_STAND_HOLD = {
    .name = "stand_hold",
    .frames = K_STAND_HOLD,
    .n_frames = sizeof(K_STAND_HOLD)/sizeof(K_STAND_HOLD[0]),
    .loop = 1,
};

/* ---- 2. WAVE_UP_DOWN : 四腿同时按正弦抬放（等价旧 up_down[9]） ----
 * z 量写到 knee_rad 通道充当足端抬腿高度（M3 接 IK 后由 leg_ik 替换），
 * 周期 0.9s（对应 9 帧 × 100ms），loop 连续；wheel=0, in_stance 摆动期 0。
 */
#define WAVE_KNEE_AMP  0.05f
static const script_keyframe_t K_WAVE[] = {
    { 0.0f, {{0, 0.00f,0,1},{0, 0.00f,0,1},{0, 0.00f,0,1},{0, 0.00f,0,1}} },
    { 0.1f, {{0, 0.01f,0,0},{0, 0.01f,0,0},{0, 0.01f,0,0},{0, 0.01f,0,0}} },
    { 0.2f, {{0, 0.03f,0,0},{0, 0.03f,0,0},{0, 0.03f,0,0},{0, 0.03f,0,0}} },
    { 0.3f, {{0, 0.04f,0,0},{0, 0.04f,0,0},{0, 0.04f,0,0},{0, 0.04f,0,0}} },
    { 0.4f, {{0, 0.05f,0,0},{0, 0.05f,0,0},{0, 0.05f,0,0},{0, 0.05f,0,0}} },
    { 0.5f, {{0, 0.04f,0,0},{0, 0.04f,0,0},{0, 0.04f,0,0},{0, 0.04f,0,0}} },
    { 0.6f, {{0, 0.03f,0,0},{0, 0.03f,0,0},{0, 0.03f,0,0},{0, 0.03f,0,0}} },
    { 0.7f, {{0, 0.01f,0,0},{0, 0.01f,0,0},{0, 0.01f,0,0},{0, 0.01f,0,0}} },
    { 0.8f, {{0, 0.00f,0,1},{0, 0.00f,0,1},{0, 0.00f,0,1},{0, 0.00f,0,1}} },
    { 0.9f, {{0, 0.00f,0,1},{0, 0.00f,0,1},{0, 0.00f,0,1},{0, 0.00f,0,1}} },
};
const script_t SCRIPT_BUILTIN_WAVE_UP_DOWN = {
    .name = "wave_up_down",
    .frames = K_WAVE,
    .n_frames = sizeof(K_WAVE)/sizeof(K_WAVE[0]),
    .loop = 1,
};

/* ---- 3. TROT_STEP : 对角 trot 4 关键帧 loop ---- */
/* FL/RR 同相支撑，FR/RL 摆动；然后交换。hip_rad 字段承载 x 方向位移(m)，knee_rad 承载 z */
static const script_keyframe_t K_TROT[] = {
    { 0.00f, {
        { +0.03f, 0.00f, 0.0f, 1 },  /* FL stance, 前 */
        { -0.03f, 0.02f, 0.0f, 0 },  /* FR swing, 后上 */
        { -0.03f, 0.02f, 0.0f, 0 },  /* RL swing */
        { +0.03f, 0.00f, 0.0f, 1 },  /* RR stance */
    }},
    { 0.20f, {
        { -0.03f, 0.00f, 0.0f, 1 },
        { +0.03f, 0.00f, 0.0f, 1 },  /* FR touchdown */
        { +0.03f, 0.00f, 0.0f, 1 },
        { -0.03f, 0.00f, 0.0f, 1 },
    }},
    { 0.40f, {
        { -0.03f, 0.02f, 0.0f, 0 },
        { +0.03f, 0.00f, 0.0f, 1 },
        { +0.03f, 0.00f, 0.0f, 1 },
        { -0.03f, 0.02f, 0.0f, 0 },
    }},
    { 0.60f, {
        { +0.03f, 0.00f, 0.0f, 1 },
        { -0.03f, 0.00f, 0.0f, 1 },
        { -0.03f, 0.00f, 0.0f, 1 },
        { +0.03f, 0.00f, 0.0f, 1 },
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
