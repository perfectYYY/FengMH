/*
 * leg_ik.c — 2 连杆逆运动学 / 正运动学
 *
 * 从 Core/Src/gait_plan.c → inverse_kinematics_position() / forward_kinematics_position() 迁移。
 * 去除 arm_math 依赖，使用纯 float 运算 (sqrtf, acosf, atan2f, sinf, cosf)。
 *
 * 坐标系约定 (与旧代码一致)：
 *   - x: 水平方向，向前为正
 *   - z: 竖直方向，向下为正
 *   - 原点在髋关节
 *
 * 腿型/电机映射由 leg_config.c 统一维护，固件和 PC 调试工具共用。
 */
#include "leg_ik.h"
#include "leg_config.h"

#include <math.h>
#include <string.h>

/* ─── 单腿 IK ─── */

int leg_ik_solve(float x, float z, const leg_dim_t* dim,
                  float hight, leg_type_t leg_type,
                  leg_ik_result_t* result) {
    if (!dim || !result) return -1;

    float z_total = z + hight;
    float L1 = dim->thigh_length;
    float L2 = dim->shin_length;

    float D = sqrtf(x * x + z_total * z_total);

    /* 工作空间检查 */
    if (D > (L1 + L2) || D < fabsf(L1 - L2)) {
        /* 无解：超出工作空间，返回最近可达姿态 */
        result->theta1 = atan2f(z_total, x);
        result->theta2 = result->theta1;  /* 腿完全伸直 */
        return -1;
    }

    float cos_theta = (L1 * L1 + D * D - L2 * L2) / (2.0f * L1 * D);
    if (cos_theta > 1.0f)  cos_theta = 1.0f;
    if (cos_theta < -1.0f) cos_theta = -1.0f;
    float theta = acosf(cos_theta);

    float a;
    if (leg_type == LEG_TYPE_MIRROR) {
        a = atan2f(z_total, x) + theta;
    } else {  /* LEG_TYPE_ORIGINAL */
        a = atan2f(z_total, x) - theta;
    }

    result->theta1 = a;
    result->theta2 = atan2f((z_total - L1 * sinf(a)), (x - L1 * cosf(a)));

    return 0;
}

/* ─── 正运动学 ─── */

void leg_fk_solve(float theta1, float theta2,
                   const leg_dim_t* dim,
                   leg_fk_result_t* result) {
    if (!dim || !result) return;

    float L1 = dim->thigh_length;
    float L2 = dim->shin_length;

    result->x = L1 * cosf(theta1) + L2 * cosf(theta2);
    result->z = L1 * sinf(theta1) + L2 * sinf(theta2);
}

/* ─── 批量 IK ─── */

static float motor_mount_to_cmd(const motor_cfg_t* cfg, float rad) {
    if (!cfg) return rad;
    return rad * (float)cfg->dir + cfg->zero_offset;
}

void leg_ik_solve_all(const gait_output_t* foot_disp,
                       const leg_dim_t* dim,
                       float hight,
                       gait_output_t* out) {
    if (!foot_disp || !dim || !out) return;

    /* 复制 wheel_rads 和 in_stance (IK 不改变这些字段) */
    memcpy(out, foot_disp, sizeof(*out));

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        const leg_config_t* cfg = leg_config_get((gait_leg_t)i);
        if (!cfg) continue;

        const gait_leg_target_t* ft = &foot_disp->leg[i];
        gait_leg_target_t* ot = &out->leg[i];

        /* foot_disp 中 hip_rad 临时携带 body-frame dx, knee_rad 临时携带 dz */
        float dx = ft->hip_rad * cfg->foot_x_dir;
        float dz = ft->knee_rad;

        leg_ik_result_t ik;
        int ret = leg_ik_solve(dx, dz, dim, hight, cfg->leg_type, &ik);

        if (ret != 0) {
            /* IK 无解时保持上一帧角度或零位 (安全降级) */
            ot->hip_rad  = 0.0f;
            ot->knee_rad = 0.0f;
        } else {
            ot->hip_rad  = motor_mount_to_cmd(motor_get_cfg(cfg->motor[LEG_ACT_HIP]), ik.theta1);
            ot->knee_rad = motor_mount_to_cmd(motor_get_cfg(cfg->motor[LEG_ACT_KNEE]), ik.theta2);
        }
    }
}
