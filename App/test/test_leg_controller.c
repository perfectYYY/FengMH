/*
 * test_leg_controller.c — registry→leg 绑定 + stub vtable 下发验证
 *
 * 注意: leg_controller_apply 现在内部调用 leg_ik_solve_all，
 *       gait_output_t 中的 hip_rad/knee_rad 被解释为足端位移 (dx, dz)，
 *       IK 解算后替换为关节角度 (theta1, theta2)。
 */
#include "test_util.h"
#include "leg_config.h"
#include "leg_controller.h"
#include "leg_ik.h"
#include "leg_params.h"
#include "motor_registry.h"
#include "gait_if.h"
#include "log.h"

#include <string.h>
#include <math.h>

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

static void t_real_leg_config_mapping(void) {
    TEST_ASSERT_EQUAL_UINT(GAIT_LEG_NUM, leg_config_count());

    const leg_config_t* fl = leg_config_get(GAIT_LEG_FL);
    const leg_config_t* fr = leg_config_get(GAIT_LEG_FR);
    const leg_config_t* rl = leg_config_get(GAIT_LEG_RL);
    const leg_config_t* rr = leg_config_get(GAIT_LEG_RR);
    TEST_ASSERT_NOT_NULL(fl);
    TEST_ASSERT_NOT_NULL(fr);
    TEST_ASSERT_NOT_NULL(rl);
    TEST_ASSERT_NOT_NULL(rr);

    TEST_ASSERT_EQUAL_INT(LEG_TYPE_MIRROR, fl->leg_type);
    TEST_ASSERT_EQUAL_INT(LEG_TYPE_ORIGINAL, fr->leg_type);
    TEST_ASSERT_EQUAL_INT(LEG_TYPE_ORIGINAL, rl->leg_type);
    TEST_ASSERT_EQUAL_INT(LEG_TYPE_MIRROR, rr->leg_type);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, fl->foot_x_dir);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, +1.0f, fr->foot_x_dir);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, rl->foot_x_dir);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, +1.0f, rr->foot_x_dir);

    TEST_ASSERT_EQUAL_INT(MOTOR_ID_FL_HIP, fl->motor[LEG_ACT_HIP]);
    TEST_ASSERT_EQUAL_INT(MOTOR_ID_FL_KNEE, fl->motor[LEG_ACT_KNEE]);
    TEST_ASSERT_EQUAL_INT(MOTOR_ID_FL_WHEEL, fl->motor[LEG_ACT_WHEEL]);
    TEST_ASSERT_EQUAL_INT(MOTOR_ID_RR_HIP, rr->motor[LEG_ACT_HIP]);
    TEST_ASSERT_EQUAL_INT(MOTOR_ID_RR_KNEE, rr->motor[LEG_ACT_KNEE]);
    TEST_ASSERT_EQUAL_INT(MOTOR_ID_RR_WHEEL, rr->motor[LEG_ACT_WHEEL]);
}

static void t_apply_dispatches(void) {
    bind_all_stubs();
    leg_controller_t lc;
    leg_controller_init(&lc);
    leg_controller_bind_from_registry(&lc);

    /* 零位移 = 站立位置，IK 应解算出关节角度 */
    gait_output_t o;
    memset(&o, 0, sizeof(o));
    /* 设置轮速以验证直通 */
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        o.leg[i].wheel_rads = 1.0f * (i + 1);
    }

    leg_controller_apply(&lc, &o);

    /* 所有 4 条腿都应成功下发 */
    TEST_ASSERT_EQUAL_UINT(GAIT_LEG_NUM, lc.send_cnt);

    /* wheel_rads 直通，不走 IK */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, s_ctx[MOTOR_ID_FL_WHEEL].last_vel);

    /* 髋/膝关节角度应为有限值 (IK 解算结果) */
    TEST_ASSERT_TRUE(isfinite(s_ctx[MOTOR_ID_FL_HIP].last_pos));
    TEST_ASSERT_TRUE(isfinite(s_ctx[MOTOR_ID_FL_KNEE].last_pos));
}

static void t_ik_roundtrip(void) {
    /* IK → FK 一致性验证：给定足端位置，IK 解算后 FK 回到原位 */
    const leg_dim_t* dim = &LEG_DIM_DEFAULT;
    float hight = -0.18f;
    /* firmware-dev 语义：足端在髋关节下方时 z_total 为负。 */

    /* 零位移 = 站立状态应可解 */
    {
        leg_ik_result_t ik;
        int ret = leg_ik_solve(0.0f, 0.0f, dim, hight,
                               LEG_TYPE_ORIGINAL, &ik);
        TEST_ASSERT_EQUAL_INT(0, ret);

        /* FK 验证 */
        leg_fk_result_t fk;
        leg_fk_solve(ik.theta1, ik.theta2, dim, &fk);
        /* 站立时 z_total = hight, x ≈ 0 */
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, fk.x);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, hight, fk.z);
    }

    /* 测试 ORIGINAL 腿型：微小偏移 (不触发角度限幅) */
    {
        float target_x = 0.02f;
        float target_z = 0.02f;

        leg_ik_result_t ik;
        int ret = leg_ik_solve(target_x, target_z, dim, hight,
                               LEG_TYPE_ORIGINAL, &ik);
        TEST_ASSERT_EQUAL_INT(0, ret);

        /* FK 验证 */
        leg_fk_result_t fk;
        leg_fk_solve(ik.theta1, ik.theta2, dim, &fk);
        float z_total = target_z + hight;
        TEST_ASSERT_FLOAT_WITHIN(0.005f, target_x, fk.x);
        TEST_ASSERT_FLOAT_WITHIN(0.005f, z_total, fk.z);
    }

    /* 测试 MIRROR 腿型：微小偏移 */
    {
        float target_x = 0.02f;
        float target_z = 0.02f;

        leg_ik_result_t ik;
        int ret = leg_ik_solve(target_x, target_z, dim, hight,
                               LEG_TYPE_MIRROR, &ik);
        TEST_ASSERT_EQUAL_INT(0, ret);

        /* FK 验证 */
        leg_fk_result_t fk;
        leg_fk_solve(ik.theta1, ik.theta2, dim, &fk);
        float z_total = target_z + hight;
        TEST_ASSERT_FLOAT_WITHIN(0.005f, target_x, fk.x);
        TEST_ASSERT_FLOAT_WITHIN(0.005f, z_total, fk.z);
    }

    /* 超出工作空间应返回 -1 */
    {
        leg_ik_result_t ik;
        int ret = leg_ik_solve(0.0f, 0.5f, dim, hight,
                               LEG_TYPE_ORIGINAL, &ik);
        TEST_ASSERT_EQUAL_INT(-1, ret);
    }
}

static float dist2d(float ax, float az, float bx, float bz) {
    float dx = ax - bx;
    float dz = az - bz;
    return sqrtf(dx * dx + dz * dz);
}

static void t_real_linkage_geometry(void) {
    const leg_dim_t* dim = &LEG_DIM_DEFAULT;
    leg_linkage_pose_t linkage;
    leg_fk_result_t fk;

    leg_linkage_solve(-2.1f, -1.1f, dim, &linkage);
    leg_fk_solve(-2.1f, -1.1f, dim, &fk);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, dim->thigh_length,
                             dist2d(linkage.hip_x, linkage.hip_z,
                                    linkage.knee_x, linkage.knee_z));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, dim->link_length,
                             dist2d(linkage.hip_x, linkage.hip_z,
                                    linkage.crank_x, linkage.crank_z));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, dim->thigh_length,
                             dist2d(linkage.crank_x, linkage.crank_z,
                                    linkage.lower_mount_x, linkage.lower_mount_z));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, dim->link_length,
                             dist2d(linkage.knee_x, linkage.knee_z,
                                    linkage.lower_mount_x, linkage.lower_mount_z));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, dim->shin_length,
                             dist2d(linkage.knee_x, linkage.knee_z,
                                    linkage.foot_x, linkage.foot_z));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, linkage.foot_x, fk.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, linkage.foot_z, fk.z);
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
    TU_RUN(t_real_leg_config_mapping);
    TU_RUN(t_bind_all_present);
    TU_RUN(t_apply_dispatches);
    TU_RUN(t_ik_roundtrip);
    TU_RUN(t_real_linkage_geometry);
    TU_RUN(t_missing_increments_miss);
    TU_MAIN_EPILOGUE();
}
