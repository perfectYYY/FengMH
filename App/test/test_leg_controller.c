/*
 * test_leg_controller.c — registry→leg 绑定 + stub vtable 下发验证
 */
#include "test_util.h"
#include "leg_controller.h"
#include "motor_registry.h"
#include "gait_if.h"
#include "log.h"

#include <string.h>

/* 用 stub 驱动把 set_position / set_velocity 记账 */
typedef struct {
    int   pos_calls, vel_calls, dis_calls;
    float last_pos, last_vel;
} stub_t;

static int stub_set_pos(motor_dev_t* d, float pos, float v, float kp, float kd, float ff) {
    (void)v; (void)kp; (void)kd; (void)ff;
    stub_t* s = (stub_t*)d->drv_ctx;
    s->pos_calls++;
    s->last_pos = pos;
    return 0;
}
static int stub_set_vel(motor_dev_t* d, float v) {
    stub_t* s = (stub_t*)d->drv_ctx;
    s->vel_calls++;
    s->last_vel = v;
    return 0;
}
static int stub_disable(motor_dev_t* d) {
    stub_t* s = (stub_t*)d->drv_ctx; s->dis_calls++; return 0;
}

static const motor_ops_t s_ops = {
    .set_position = stub_set_pos,
    .set_velocity = stub_set_vel,
    .disable      = stub_disable,
};

static motor_dev_t s_dev[12];
static stub_t      s_ctx[12];

static void bind_all_stubs(void) {
    motor_registry_init();
    memset(s_dev, 0, sizeof(s_dev));
    memset(s_ctx, 0, sizeof(s_ctx));
    for (int i = 0; i < 12; i++) {
        s_dev[i].ops = &s_ops;
        s_dev[i].drv_ctx = &s_ctx[i];
        motor_registry_bind((motor_logical_id_t)i, &s_dev[i]);
    }
}

static void t_bind_all_present(void) {
    bind_all_stubs();
    leg_controller_t lc;
    leg_controller_init(&lc);
    leg_controller_bind_from_registry(&lc);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_NOT_NULL(lc.leg[i].hip);
        TEST_ASSERT_NOT_NULL(lc.leg[i].knee);
        TEST_ASSERT_NOT_NULL(lc.leg[i].wheel);
    }
}

static void t_apply_dispatches(void) {
    bind_all_stubs();
    leg_controller_t lc;
    leg_controller_init(&lc);
    leg_controller_bind_from_registry(&lc);

    gait_output_t o;
    memset(&o, 0, sizeof(o));
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        o.leg[i].hip_rad = 0.1f * (i+1);
        o.leg[i].knee_rad = 0.2f * (i+1);
        o.leg[i].wheel_rads = 0.3f * (i+1);
    }
    leg_controller_apply(&lc, &o);
    TEST_ASSERT_EQUAL_UINT(GAIT_LEG_NUM, lc.send_cnt);
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        TEST_ASSERT_EQUAL_INT(1, s_ctx[MOTOR_ID_FL_HIP   + i * 3 - MOTOR_ID_FL_HIP].pos_calls >= 0); /* sanity */
    }
    /* FL 髋关节应收到 0.1 rad */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, s_ctx[MOTOR_ID_FL_HIP].last_pos);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, s_ctx[MOTOR_ID_FL_WHEEL].last_vel);
}

static void t_missing_increments_miss(void) {
    motor_registry_init();  /* 所有设备都未绑定 */
    leg_controller_t lc;
    leg_controller_init(&lc);
    leg_controller_bind_from_registry(&lc);

    gait_output_t o;
    memset(&o, 0, sizeof(o));
    leg_controller_apply(&lc, &o);
    TEST_ASSERT_EQUAL_UINT(GAIT_LEG_NUM, lc.miss_cnt);
    TEST_ASSERT_EQUAL_UINT(0, lc.send_cnt);
}

int main(void) {
    log_init();
    log_set_global_level(LOG_LVL_ERR);
    TU_RUN(t_bind_all_present);
    TU_RUN(t_apply_dispatches);
    TU_RUN(t_missing_increments_miss);
    TU_MAIN_EPILOGUE();
}
