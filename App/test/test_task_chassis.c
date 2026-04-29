/*
 * test_task_chassis.c — 三模式 + 心跳超时 + 脚本播放
 *
 * 用 stub motor 驱动注满 registry，验证 task_chassis_step_for_test 在不同
 * 模式 / 心跳条件下选择正确的步态。
 */
#include "test_util.h"
#include "task_chassis.h"
#include "task_comm.h"
#include "motor_registry.h"
#include "motor_if.h"
#include "bsp_usb_cdc.h"
#include "bsp_time.h"
#include "proto_frame.h"
#include "proto_defs.h"
#include "gait_params.h"
#include "script_builtin.h"
#include "log.h"

#include <string.h>

static int s_pos_calls = 0;
static int s_vel_calls = 0;
static int stub_set_pos(motor_dev_t* d, float p, float v, float kp, float kd, float ff) {
    (void)d;(void)p;(void)v;(void)kp;(void)kd;(void)ff; s_pos_calls++; return 0;
}
static int stub_set_vel(motor_dev_t* d, float v) { (void)d;(void)v; s_vel_calls++; return 0; }
static int stub_disable(motor_dev_t* d) { (void)d; return 0; }
static const motor_ops_t s_ops = {
    .set_position = stub_set_pos,
    .set_velocity = stub_set_vel,
    .disable      = stub_disable,
};
static motor_dev_t s_dev[12];

static void setup_all(void) {
    s_pos_calls = 0; s_vel_calls = 0;
    bsp_time_init();
    bsp_usb_cdc_init();
    motor_registry_init();
    memset(s_dev, 0, sizeof(s_dev));
    for (int i = 0; i < 12; i++) {
        s_dev[i].ops = &s_ops;
        motor_registry_bind((motor_logical_id_t)i, &s_dev[i]);
    }
    task_comm_init();
    task_chassis_init();
}

static void inject_chassis_cmd(float vx) {
    payload_chassis_cmd_t cmd = { .vx = vx, .vy = 0, .wz = 0 };
    uint8_t frame[64];
    int n = proto_frame_build(PROTO_FUNC_CHASSIS_CMD, (uint8_t*)&cmd, 12, frame, sizeof(frame));
    bsp_usb_cdc_test_inject_rx(frame, (uint32_t)n);
}

static void t_default_is_stand(void) {
    setup_all();
    task_chassis_step_for_test(0.002f, 100);
    TEST_ASSERT_EQUAL_INT(0, strcmp(task_chassis_active_gait_name(), "stand"));
}

static void t_online_trot_on_cmd(void) {
    setup_all();
    task_chassis_set_mode(CHASSIS_MODE_ONLINE);
    inject_chassis_cmd(0.3f);
    /* 推进若干 tick 让 BLEND 完成 */
    for (int i = 0; i < 200; i++) task_chassis_step_for_test(0.002f, 200 + i);
    TEST_ASSERT_EQUAL_INT(0, strcmp(task_chassis_active_gait_name(), "trot"));
}

static void t_auto_falls_back_when_silent(void) {
    setup_all();
    /* AUTO 模式默认；从未收过帧（last_rx_ms=0）就当离线 → stand */
    for (int i = 0; i < 5; i++) task_chassis_step_for_test(0.002f, 50 + i);
    TEST_ASSERT_EQUAL_INT(0, strcmp(task_chassis_active_gait_name(), "stand"));
}

static void t_auto_uses_online_after_cmd(void) {
    setup_all();
    inject_chassis_cmd(0.5f);
    /* 注入帧时 bsp_time 还是 0；用 host 推进时间但维持 last_rx 较新 */
    uint32_t now = bsp_time_now_ms();
    for (int i = 0; i < 200; i++) task_chassis_step_for_test(0.002f, now + 50);
    TEST_ASSERT_EQUAL_INT(0, strcmp(task_chassis_active_gait_name(), "trot"));
}

static void t_auto_falls_back_on_timeout(void) {
    setup_all();
    task_chassis_set_online_timeout_ms(500);
    inject_chassis_cmd(0.5f);
    uint32_t now = bsp_time_now_ms();
    /* 进入 trot */
    for (int i = 0; i < 200; i++) task_chassis_step_for_test(0.002f, now + 100);
    TEST_ASSERT_EQUAL_INT(0, strcmp(task_chassis_active_gait_name(), "trot"));
    /* 模拟 1500ms 后没有任何新帧 → 应回 stand */
    for (int i = 0; i < 500; i++) task_chassis_step_for_test(0.002f, now + 1500);
    TEST_ASSERT_EQUAL_INT(0, strcmp(task_chassis_active_gait_name(), "stand"));
}

static void t_play_script_then_stop(void) {
    setup_all();
    task_chassis_set_mode(CHASSIS_MODE_STANDALONE);
    int r = task_chassis_play_script(&SCRIPT_BUILTIN_WAVE_UP_DOWN, 0.0f);
    TEST_ASSERT_EQUAL_INT(APP_OK, r);
    /* 推一些 tick；script gait 名应叫 "script" */
    for (int i = 0; i < 50; i++) task_chassis_step_for_test(0.002f, 100 + i);
    TEST_ASSERT_EQUAL_INT(0, strcmp(task_chassis_active_gait_name(), "script"));

    task_chassis_stop_script(0.0f);
    for (int i = 0; i < 5; i++) task_chassis_step_for_test(0.002f, 1000 + i);
    TEST_ASSERT_EQUAL_INT(0, strcmp(task_chassis_active_gait_name(), "stand"));
}

static void t_standalone_ignores_usb(void) {
    setup_all();
    task_chassis_set_mode(CHASSIS_MODE_STANDALONE);
    inject_chassis_cmd(0.5f);
    uint32_t now = bsp_time_now_ms();
    for (int i = 0; i < 200; i++) task_chassis_step_for_test(0.002f, now + 50);
    /* STANDALONE 不切 trot */
    TEST_ASSERT_EQUAL_INT(0, strcmp(task_chassis_active_gait_name(), "stand"));
}

static void t_manual_trot_holds_without_usb_velocity(void) {
    setup_all();
    gait_params_t p = GAIT_PARAMS_TROT_DEFAULT;
    p.body_height_m = 0.18f;
    p.step_length_m = 0.04f;
    p.step_height_m = 0.02f;
    p.period_s = 0.6f;
    p.duty = 0.65f;
    task_chassis_set_mode(CHASSIS_MODE_STANDALONE);
    TEST_ASSERT_EQUAL_INT(APP_OK, task_chassis_start_trot(&p, 0.0f));
    for (int i = 0; i < 50; i++) task_chassis_step_for_test(0.002f, 100 + i);
    TEST_ASSERT_EQUAL_INT(0, strcmp(task_chassis_active_gait_name(), "trot"));

    TEST_ASSERT_EQUAL_INT(APP_OK, task_chassis_start_stand(0.0f));
    task_chassis_step_for_test(0.002f, 300);
    TEST_ASSERT_EQUAL_INT(0, strcmp(task_chassis_active_gait_name(), "stand"));
}

static void t_motor_calls_happen(void) {
    setup_all();
    task_chassis_set_mode(CHASSIS_MODE_STANDALONE);
    int before = s_pos_calls;
    for (int i = 0; i < 10; i++) task_chassis_step_for_test(0.002f, 100 + i);
    TEST_ASSERT(s_pos_calls > before);
}

int main(void) {
    log_init();
    log_set_global_level(LOG_LVL_ERR);
    TU_RUN(t_default_is_stand);
    TU_RUN(t_online_trot_on_cmd);
    TU_RUN(t_auto_falls_back_when_silent);
    TU_RUN(t_auto_uses_online_after_cmd);
    TU_RUN(t_auto_falls_back_on_timeout);
    TU_RUN(t_play_script_then_stop);
    TU_RUN(t_standalone_ignores_usb);
    TU_RUN(t_manual_trot_holds_without_usb_velocity);
    TU_RUN(t_motor_calls_happen);
    TU_MAIN_EPILOGUE();
}
