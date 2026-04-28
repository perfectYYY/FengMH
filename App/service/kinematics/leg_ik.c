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
 * 腿型映射 (与 motor_registry 的 dir 字段对齐)：
 *   FL (dir=-1) → MIRROR
 *   FR (dir=-1) → MIRROR
 *   RL (dir=+1) → ORIGINAL
 *   RR (dir=+1) → ORIGINAL
 *
 * 注意：实际映射需根据机械装配确认，这里暂按 dir 判断。
 */
#include "leg_ik.h"
#include "motor_registry.h"

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

/*
 * 腿型映射表 (与 motor_registry 中的 dir 对应)：
 *   FL (index 0): dir=-1 → MIRROR
 *   FR (index 1): dir=-1 → MIRROR
 *   RL (index 2): dir=+1 → ORIGINAL
 *   RR (index 3): dir=+1 → ORIGINAL
 */
static const leg_type_t s_leg_type[GAIT_LEG_NUM] = {
    LEG_TYPE_MIRROR,    /* FL */
    LEG_TYPE_MIRROR,    /* FR */
    LEG_TYPE_ORIGINAL,  /* RL */
    LEG_TYPE_ORIGINAL,  /* RR */
};

/* 对应的 motor_logical_id */
static const motor_logical_id_t HIP_ID[GAIT_LEG_NUM] = {
    MOTOR_ID_FL_HIP, MOTOR_ID_FR_HIP, MOTOR_ID_RL_HIP, MOTOR_ID_RR_HIP
};

void leg_ik_solve_all(const gait_output_t* foot_disp,
                       const leg_dim_t* dim,
                       float hight,
                       gait_output_t* out) {
    if (!foot_disp || !dim || !out) return;

    /* 复制 wheel_rads 和 in_stance (IK 不改变这些字段) */
    memcpy(out, foot_disp, sizeof(*out));

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        const gait_leg_target_t* ft = &foot_disp->leg[i];
        gait_leg_target_t* ot = &out->leg[i];

        /* foot_disp 中 hip_rad 临时携带 dx, knee_rad 临时携带 dz */
        float dx = ft->hip_rad;
        float dz = ft->knee_rad;

        leg_ik_result_t ik;
        int ret = leg_ik_solve(dx, dz, dim, hight, s_leg_type[i], &ik);

        if (ret != 0) {
            /* IK 无解时保持上一帧角度或零位 (安全降级) */
            ot->hip_rad  = 0.0f;
            ot->knee_rad = 0.0f;
        } else {
            /* 应用 motor_registry 中的方向系数 */
            const motor_cfg_t* hip_cfg = motor_get_cfg(HIP_ID[i]);
            float dir = hip_cfg ? (float)hip_cfg->dir : 1.0f;
            ot->hip_rad  = ik.theta1 * dir;
            ot->knee_rad = ik.theta2 * dir;
        }
    }
}
