/*
 * test_pid.c — service/pid host 单测
 */
#include "test_util.h"
#include "pid.h"

static void t_init_and_reset(void) {
    app_pid_t p;
    app_pid_init(&p, 1.0f, 0.5f, 0.1f, 100.0f, -100.0f, 50.0f, -50.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, p.Kp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, p.Ki);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, p.Kd);
    TEST_ASSERT_EQUAL_INT(0, p.initialized);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, p.output);
}

static void t_output_clamp(void) {
    app_pid_t p;
    app_pid_init(&p, 1000.0f, 0.0f, 0.0f, 5.0f, -5.0f, 0.0f, 0.0f);
    float out = app_pid_update_dt(&p, 10.0f, 0.0f, 0.01f);
    /* Kp*err = 10000, 应被夹到 5 */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5.0f, out);

    out = app_pid_update_dt(&p, -10.0f, 0.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -5.0f, out);
}

static void t_integral_converges_to_setpoint(void) {
    /* 模拟一阶被控对象：y' = -a*y + k*u, 用 PID 达到设定值 */
    app_pid_t p;
    app_pid_init(&p, 2.0f, 4.0f, 0.05f, 20.0f, -20.0f, 10.0f, -10.0f);
    float y = 0.0f;
    const float sp = 1.0f;
    const float dt = 0.01f;
    for (int i = 0; i < 2000; i++) {
        float u = app_pid_update_dt(&p, sp, y, dt);
        /* 一阶模型: a=2, k=1 */
        y += dt * (-2.0f * y + 1.0f * u);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, sp, y);
}

static void t_incremental_accumulates(void) {
    app_pid_t p;
    app_pid_init(&p, 1.0f, 0.0f, 0.0f, 100.0f, -100.0f, 10.0f, -10.0f);
    float o1 = app_pid_update_inc_dt(&p, 1.0f, 0.0f, 0.01f);
    float o2 = app_pid_update_inc_dt(&p, 1.0f, 0.0f, 0.01f);
    /* 增量式：每次 delta = Kp*(err-prev_err); 第一步 prev_err 被初始化为当前 err，
       所以首次 delta=0，o1=0；第二次 err 仍=1, delta=0 → o2 也=0 */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, o1);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, o2);

    /* 错位一次 setpoint，会看到一次 delta */
    float o3 = app_pid_update_inc_dt(&p, 2.0f, 0.0f, 0.01f);
    /* 增量: Kp*(err-prev_err) = 1*(2-1) = 1 */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, o3);
}

static void t_ms_dt_auto(void) {
    app_pid_t p;
    app_pid_init(&p, 1.0f, 1.0f, 0.0f, 100.0f, -100.0f, 50.0f, -50.0f);
    /* 第一次调用：dt 用默认值 0.01s，error=1, 积分累加 0.01 */
    (void)app_pid_update_ms(&p, 1.0f, 0.0f, 1000u);
    /* 第二次调用：dt=100ms=0.1s，error=1, 积分再加 0.1，总 0.11 */
    (void)app_pid_update_ms(&p, 1.0f, 0.0f, 1100u);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.11f, p.integral);
}

int main(void) {
    TU_RUN(t_init_and_reset);
    TU_RUN(t_output_clamp);
    TU_RUN(t_integral_converges_to_setpoint);
    TU_RUN(t_incremental_accumulates);
    TU_RUN(t_ms_dt_auto);
    TU_MAIN_EPILOGUE();
}
