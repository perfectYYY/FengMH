/*
 * test_script.c — script_player + gait_script 单测
 */
#include "test_util.h"
#include "script_if.h"
#include "script_player.h"
#include "script_builtin.h"
#include "gait_script.h"
#include "gait_if.h"
#include "log.h"

#include <string.h>

/* 简单 2 帧脚本，便于手算插值 */
static const script_keyframe_t K_SIMPLE[] = {
    { 0.0f, { {0,0,0,1},{0,0,0,1},{0,0,0,1},{0,0,0,1} } },
    { 1.0f, { {1.0f,2.0f,3.0f,0},{0,0,0,0},{0,0,0,0},{0,0,0,0} } },
};
static const script_t S_SIMPLE = {
    .name = "simple", .frames = K_SIMPLE,
    .n_frames = 2, .loop = 0,
};

static void t_sample_endpoints(void) {
    gait_output_t o;
    TEST_ASSERT_EQUAL_INT(APP_OK, script_sample(&S_SIMPLE, -1.0f, &o));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, o.leg[0].hip_rad);

    TEST_ASSERT_EQUAL_INT(APP_OK, script_sample(&S_SIMPLE, 2.0f, &o));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, o.leg[0].hip_rad);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, o.leg[0].knee_rad);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.0f, o.leg[0].wheel_rads);
}

static void t_sample_interp(void) {
    gait_output_t o;
    script_sample(&S_SIMPLE, 0.5f, &o);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, o.leg[0].hip_rad);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, o.leg[0].knee_rad);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.5f, o.leg[0].wheel_rads);
    /* in_stance 取 later 帧值 = 0 */
    TEST_ASSERT_EQUAL_UINT(0, o.leg[0].in_stance);
}

static void t_player_non_loop_done(void) {
    script_player_t p;
    script_player_init(&p);
    TEST_ASSERT_EQUAL_INT(APP_OK, script_player_load(&p, &S_SIMPLE));
    gait_output_t o;
    for (int i = 0; i < 600; i++) {
        script_player_update(&p, 0.002f, &o);  /* 1.2s */
    }
    TEST_ASSERT_EQUAL_INT(SP_STATE_DONE, script_player_state(&p));
}

static void t_player_loop_wraps(void) {
    script_player_t p;
    script_player_init(&p);
    script_t s_loop = S_SIMPLE; s_loop.loop = 1;
    script_player_load(&p, &s_loop);
    gait_output_t o;
    for (int i = 0; i < 600; i++) {
        script_player_update(&p, 0.002f, &o);
    }
    /* loop 永远 RUNNING */
    TEST_ASSERT_EQUAL_INT(SP_STATE_RUNNING, script_player_state(&p));
}

static void t_builtin_find(void) {
    TEST_ASSERT(script_builtin_find("stand_hold") == &SCRIPT_BUILTIN_STAND_HOLD);
    TEST_ASSERT(script_builtin_find("wave_up_down") == &SCRIPT_BUILTIN_WAVE_UP_DOWN);
    TEST_ASSERT_NULL(script_builtin_find("nonexistent"));
}

static void t_builtin_wave_midpoint(void) {
    gait_output_t o;
    /* 0.4s 对应 WAVE 峰值 0.05 */
    TEST_ASSERT_EQUAL_INT(APP_OK, script_sample(&SCRIPT_BUILTIN_WAVE_UP_DOWN, 0.4f, &o));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.05f, o.leg[0].knee_rad);
}

static void t_gait_script_update(void) {
    gait_if_t* g = gait_script_create();
    TEST_ASSERT_NOT_NULL(g);
    TEST_ASSERT_EQUAL_INT(APP_OK, gait_script_set_script(g, &S_SIMPLE));
    g->ops->init(g);
    gait_output_t o;
    for (int i = 0; i < 250; i++) {
        g->ops->update(g, 0.002f, &o);  /* 0.5s */
    }
    TEST_ASSERT_FLOAT_WITHIN(5e-2f, 0.5f, o.leg[0].hip_rad);
}

int main(void) {
    log_init();
    log_set_global_level(LOG_LVL_ERR);
    TU_RUN(t_sample_endpoints);
    TU_RUN(t_sample_interp);
    TU_RUN(t_player_non_loop_done);
    TU_RUN(t_player_loop_wraps);
    TU_RUN(t_builtin_find);
    TU_RUN(t_builtin_wave_midpoint);
    TU_RUN(t_gait_script_update);
    TU_MAIN_EPILOGUE();
}
