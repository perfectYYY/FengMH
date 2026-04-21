/*
 * test_gait.c — trot 相位发生器、足端轨迹、状态机测试
 */
#include "test_util.h"
#include "gait_if.h"
#include "gait_stand.h"
#include "gait_trot.h"
#include "gait_machine.h"
#include "gait_params.h"
#include "log.h"

#include <math.h>

static void t_wrap01(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.25f, gait_wrap01(0.25f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.25f, gait_wrap01(1.25f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.75f, gait_wrap01(-0.25f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,  gait_wrap01(3.0f));
}

static void t_stand_outputs_stance(void) {
    gait_if_t* g = gait_stand_create();
    g->ops->set_param(g, &GAIT_PARAMS_STAND_DEFAULT);
    g->ops->init(g);
    gait_output_t o;
    g->ops->update(g, 0.002f, &o);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_EQUAL_INT(1, o.leg[i].in_stance);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, o.leg[i].wheel_rads);
    }
}

static void t_trot_foot_traj_basic(void) {
    /* duty=0.5, step=0.1, height=0.04 */
    float dx, dz; uint8_t st;

    /* lp=0: 支撑相起点，x=+0.05 */
    gait_trot_foot_traj(0.0f, 0.5f, 0.1f, 0.04f, &dx, &dz, &st);
    TEST_ASSERT_EQUAL_INT(1, st);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f, dx);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,  dz);

    /* lp=0.25 中段支撑，x=0 */
    gait_trot_foot_traj(0.25f, 0.5f, 0.1f, 0.04f, &dx, &dz, &st);
    TEST_ASSERT_EQUAL_INT(1, st);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, dx);

    /* lp=0.5 摆动起点 */
    gait_trot_foot_traj(0.5f, 0.5f, 0.1f, 0.04f, &dx, &dz, &st);
    TEST_ASSERT_EQUAL_INT(0, st);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.05f, dx);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, dz);

    /* lp=0.75 摆动中点，z 峰值 = step_height */
    gait_trot_foot_traj(0.75f, 0.5f, 0.1f, 0.04f, &dx, &dz, &st);
    TEST_ASSERT_EQUAL_INT(0, st);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, dx);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.04f, dz);
}

static void t_trot_phase_offsets(void) {
    gait_if_t* g = gait_trot_create();
    gait_params_t p = GAIT_PARAMS_TROT_DEFAULT;
    g->ops->set_param(g, &p);
    g->ops->init(g);

    /* 推 0.05s (=phase 0.125)；FL 偏移 0 → lp=0.125 支撑；FR 偏移 0.5 → lp=0.625 摆动 */
    gait_output_t o;
    g->ops->update(g, 0.05f, &o);
    TEST_ASSERT_EQUAL_INT(1, o.leg[GAIT_LEG_FL].in_stance);
    TEST_ASSERT_EQUAL_INT(1, o.leg[GAIT_LEG_RR].in_stance);  /* 偏移 0，同 FL */
    TEST_ASSERT_EQUAL_INT(0, o.leg[GAIT_LEG_FR].in_stance);
    TEST_ASSERT_EQUAL_INT(0, o.leg[GAIT_LEG_RL].in_stance);
}

static void t_trot_periodic(void) {
    gait_if_t* g = gait_trot_create();
    g->ops->set_param(g, &GAIT_PARAMS_TROT_DEFAULT);
    g->ops->init(g);
    gait_output_t o;
    /* 推完整周期，phase 应回到 ~0 或 ~1（累加误差使 wrap 刚好不回零） */
    for (int i = 0; i < 200; i++) {
        g->ops->update(g, GAIT_PARAMS_TROT_DEFAULT.period_s / 200.0f, &o);
    }
    float ph = o.phase;
    /* 距 0 或 1 都算周期性正确 */
    float d0 = ph, d1 = 1.0f - ph;
    float d = (d0 < d1) ? d0 : d1;
    TEST_ASSERT_FLOAT_WITHIN(5e-3f, 0.0f, d);
}

static void t_machine_set_and_switch(void) {
    gait_machine_t m;
    gait_machine_init(&m);
    gait_if_t* stand = gait_stand_create();
    gait_if_t* trot  = gait_trot_create();
    TEST_ASSERT_EQUAL_INT(APP_OK, gait_machine_set(&m, stand, &GAIT_PARAMS_STAND_DEFAULT));
    TEST_ASSERT_EQUAL_INT(GM_STATE_RUN, gait_machine_state(&m));

    gait_output_t o;
    gait_machine_update(&m, 0.002f, &o);
    TEST_ASSERT_EQUAL_INT(1, o.leg[0].in_stance);

    /* 请求切到 trot，blend 0.1s */
    TEST_ASSERT_EQUAL_INT(APP_OK, gait_machine_request(&m, trot, &GAIT_PARAMS_TROT_DEFAULT, 0.1f));
    TEST_ASSERT_EQUAL_INT(GM_STATE_BLEND, gait_machine_state(&m));

    /* 推足够长时间，blend 结束 */
    for (int i = 0; i < 60; i++) gait_machine_update(&m, 0.002f, &o);
    TEST_ASSERT_EQUAL_INT(GM_STATE_RUN, gait_machine_state(&m));
}

int main(void) {
    log_init();
    log_set_global_level(LOG_LVL_ERR);
    TU_RUN(t_wrap01);
    TU_RUN(t_stand_outputs_stance);
    TU_RUN(t_trot_foot_traj_basic);
    TU_RUN(t_trot_phase_offsets);
    TU_RUN(t_trot_periodic);
    TU_RUN(t_machine_set_and_switch);
    TU_MAIN_EPILOGUE();
}
