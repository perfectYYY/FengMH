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
 * 腿型映射沿用老工程：
 *   LF/FL、RR 为镜像腿，LR/RL、RF/FR 为原型腿。
 */
#include "leg_ik.h"

#include <math.h>
#include <string.h>

static float leg_ik_clampf(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void leg_ik_get_joint_limit(leg_type_t leg_type,
                                    leg_angle_range_t* thigh_range,
                                    leg_angle_range_t* shin_range) {
    const float pi = 3.14159265f;

    if (!thigh_range || !shin_range) return;

    if (leg_type == LEG_TYPE_MIRROR) {
        thigh_range->min = -2.0f;
        thigh_range->max = 0.0f;
        shin_range->min = -pi;
        shin_range->max = -0.5f * pi;
    } else {
        thigh_range->min = -pi;
        thigh_range->max = 2.0f - pi;
        shin_range->min = -0.5f * pi;
        shin_range->max = 0.0f;
    }
}

static void leg_ik_update_best(float target_thigh,
                                float target_shin,
                                float candidate_thigh,
                                float candidate_shin,
                                float* best_cost,
                                float* best_thigh,
                                float* best_shin) {
    float cost = fabsf(candidate_thigh - target_thigh) +
                 fabsf(candidate_shin - target_shin);

    if (cost < *best_cost) {
        *best_cost = cost;
        *best_thigh = candidate_thigh;
        *best_shin = candidate_shin;
    }
}

static void leg_ik_constrain_joint_target(leg_ik_result_t* result,
                                           leg_type_t leg_type) {
    const float pi = 3.14159265f;
    const float d_min = 0.25f * pi;
    const float d_max = 0.87f * pi;
    leg_angle_range_t thigh_range;
    leg_angle_range_t shin_range;
    float thigh;
    float shin;
    float best_cost = INFINITY;
    float best_thigh = 0.0f;
    float best_shin = 0.0f;

    if (!result) return;

    thigh = result->theta1;
    shin = result->theta2;
    if (!isfinite(thigh) || !isfinite(shin)) return;

    leg_ik_get_joint_limit(leg_type, &thigh_range, &shin_range);
    thigh = leg_ik_clampf(thigh, thigh_range.min, thigh_range.max);
    shin = leg_ik_clampf(shin, shin_range.min, shin_range.max);

    if (leg_type == LEG_TYPE_MIRROR) {
        float candidate_low;
        float candidate_high;
        float candidate_shin;
        float candidate_thigh;

        candidate_low = fmaxf(shin_range.min, thigh - d_max);
        candidate_high = fminf(shin_range.max, thigh - d_min);
        if (candidate_low <= candidate_high) {
            candidate_shin = leg_ik_clampf(shin, candidate_low, candidate_high);
            leg_ik_update_best(thigh, shin, thigh, candidate_shin,
                               &best_cost, &best_thigh, &best_shin);
        }

        candidate_low = fmaxf(thigh_range.min, shin + d_min);
        candidate_high = fminf(thigh_range.max, shin + d_max);
        if (candidate_low <= candidate_high) {
            candidate_thigh = leg_ik_clampf(thigh, candidate_low, candidate_high);
            leg_ik_update_best(thigh, shin, candidate_thigh, shin,
                               &best_cost, &best_thigh, &best_shin);
        }

        candidate_thigh = thigh_range.min;
        candidate_low = fmaxf(shin_range.min, candidate_thigh - d_max);
        candidate_high = fminf(shin_range.max, candidate_thigh - d_min);
        if (candidate_low <= candidate_high) {
            candidate_shin = leg_ik_clampf(shin, candidate_low, candidate_high);
            leg_ik_update_best(thigh, shin, candidate_thigh, candidate_shin,
                               &best_cost, &best_thigh, &best_shin);
        }

        candidate_thigh = thigh_range.max;
        candidate_low = fmaxf(shin_range.min, candidate_thigh - d_max);
        candidate_high = fminf(shin_range.max, candidate_thigh - d_min);
        if (candidate_low <= candidate_high) {
            candidate_shin = leg_ik_clampf(shin, candidate_low, candidate_high);
            leg_ik_update_best(thigh, shin, candidate_thigh, candidate_shin,
                               &best_cost, &best_thigh, &best_shin);
        }

        candidate_shin = shin_range.min;
        candidate_low = fmaxf(thigh_range.min, candidate_shin + d_min);
        candidate_high = fminf(thigh_range.max, candidate_shin + d_max);
        if (candidate_low <= candidate_high) {
            candidate_thigh = leg_ik_clampf(thigh, candidate_low, candidate_high);
            leg_ik_update_best(thigh, shin, candidate_thigh, candidate_shin,
                               &best_cost, &best_thigh, &best_shin);
        }

        candidate_shin = shin_range.max;
        candidate_low = fmaxf(thigh_range.min, candidate_shin + d_min);
        candidate_high = fminf(thigh_range.max, candidate_shin + d_max);
        if (candidate_low <= candidate_high) {
            candidate_thigh = leg_ik_clampf(thigh, candidate_low, candidate_high);
            leg_ik_update_best(thigh, shin, candidate_thigh, candidate_shin,
                               &best_cost, &best_thigh, &best_shin);
        }
    } else {
        float candidate_low;
        float candidate_high;
        float candidate_shin;
        float candidate_thigh;

        candidate_low = fmaxf(shin_range.min, thigh + d_min);
        candidate_high = fminf(shin_range.max, thigh + d_max);
        if (candidate_low <= candidate_high) {
            candidate_shin = leg_ik_clampf(shin, candidate_low, candidate_high);
            leg_ik_update_best(thigh, shin, thigh, candidate_shin,
                               &best_cost, &best_thigh, &best_shin);
        }

        candidate_low = fmaxf(thigh_range.min, shin - d_max);
        candidate_high = fminf(thigh_range.max, shin - d_min);
        if (candidate_low <= candidate_high) {
            candidate_thigh = leg_ik_clampf(thigh, candidate_low, candidate_high);
            leg_ik_update_best(thigh, shin, candidate_thigh, shin,
                               &best_cost, &best_thigh, &best_shin);
        }

        candidate_thigh = thigh_range.min;
        candidate_low = fmaxf(shin_range.min, candidate_thigh + d_min);
        candidate_high = fminf(shin_range.max, candidate_thigh + d_max);
        if (candidate_low <= candidate_high) {
            candidate_shin = leg_ik_clampf(shin, candidate_low, candidate_high);
            leg_ik_update_best(thigh, shin, candidate_thigh, candidate_shin,
                               &best_cost, &best_thigh, &best_shin);
        }

        candidate_thigh = thigh_range.max;
        candidate_low = fmaxf(shin_range.min, candidate_thigh + d_min);
        candidate_high = fminf(shin_range.max, candidate_thigh + d_max);
        if (candidate_low <= candidate_high) {
            candidate_shin = leg_ik_clampf(shin, candidate_low, candidate_high);
            leg_ik_update_best(thigh, shin, candidate_thigh, candidate_shin,
                               &best_cost, &best_thigh, &best_shin);
        }

        candidate_shin = shin_range.min;
        candidate_low = fmaxf(thigh_range.min, candidate_shin - d_max);
        candidate_high = fminf(thigh_range.max, candidate_shin - d_min);
        if (candidate_low <= candidate_high) {
            candidate_thigh = leg_ik_clampf(thigh, candidate_low, candidate_high);
            leg_ik_update_best(thigh, shin, candidate_thigh, candidate_shin,
                               &best_cost, &best_thigh, &best_shin);
        }

        candidate_shin = shin_range.max;
        candidate_low = fmaxf(thigh_range.min, candidate_shin - d_max);
        candidate_high = fminf(thigh_range.max, candidate_shin - d_min);
        if (candidate_low <= candidate_high) {
            candidate_thigh = leg_ik_clampf(thigh, candidate_low, candidate_high);
            leg_ik_update_best(thigh, shin, candidate_thigh, candidate_shin,
                               &best_cost, &best_thigh, &best_shin);
        }
    }

    if (best_cost != INFINITY) {
        result->theta1 = best_thigh;
        result->theta2 = best_shin;
    } else {
        result->theta1 = thigh;
        result->theta2 = shin;
    }
}

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

    leg_ik_constrain_joint_target(result, leg_type);
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
 * 腿型映射表：
 *   FL/LF: MIRROR, FR/RF: ORIGINAL, RL/LR: ORIGINAL, RR: MIRROR
 *
 * x_dir 把步态层 body-frame dx 转成本腿局部 IK x，并在 PC FK/可视化
 * 中反向使用。若这里错，真实步态左右/前后会反。
 */
static const leg_type_t s_leg_type[GAIT_LEG_NUM] = {
    LEG_TYPE_MIRROR,    /* FL */
    LEG_TYPE_ORIGINAL,  /* FR */
    LEG_TYPE_ORIGINAL,  /* RL */
    LEG_TYPE_MIRROR,    /* RR */
};

static const float s_leg_x_dir[GAIT_LEG_NUM] = {
    -1.0f,  /* FL */
    +1.0f,  /* FR */
    -1.0f,  /* RL */
    +1.0f,  /* RR */
};

void leg_ik_solve_all(const gait_output_t* foot_disp,
                       const leg_dim_t* dim,
                       float hight,
                       gait_output_t* out) {
    if (!foot_disp || !dim || !out) return;

    /* 复制 wheel_rads / stance / foot target metadata. */
    memcpy(out, foot_disp, sizeof(*out));

    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        const gait_leg_target_t* ft = &foot_disp->leg[i];
        gait_leg_target_t* ot = &out->leg[i];

        float foot_x = ft->foot_x_m;
        float foot_z = ft->foot_z_m;

        /*
         * Compatibility path for legacy scripts that still store foot dx/dz in
         * hip_rad/knee_rad. Trot and new code should always set foot_*.
         */
        if (fabsf(foot_x) < 1e-7f && fabsf(foot_z) < 1e-7f &&
            (fabsf(ft->hip_rad) > 1e-7f || fabsf(ft->knee_rad) > 1e-7f)) {
            foot_x = ft->hip_rad;
            foot_z = ft->knee_rad;
        }

        float dx = foot_x * s_leg_x_dir[i];
        float dz = foot_z;

        leg_ik_result_t ik;
        int ret = leg_ik_solve(dx, dz, dim, hight, s_leg_type[i], &ik);

        if (ret != 0) {
            /* IK 无解时保持上一帧角度或零位 (安全降级) */
            ot->hip_rad  = 0.0f;
            ot->knee_rad = 0.0f;
        } else {
            ot->hip_rad  = ik.theta1;
            ot->knee_rad = ik.theta2;
        }
    }
}
