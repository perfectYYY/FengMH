/*
 * arm_kinematics.c - Four-joint arm kinematics.
 */
#include "arm_kinematics.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

const arm_kinematics_params_t ARM_KINEMATICS_DEFAULT_PARAMS = {
    .l2_m = 0.35f,
    .l3_m = 0.30f,
};

const arm_offset_config_t ARM_KINEMATICS_DEFAULT_OFFSET = {
    .theta1_offset_rad = APP_ARM_DEG2RAD(0.0f),
    .theta2_offset_rad = APP_ARM_DEG2RAD(90.0f),
    .theta3_offset_rad = APP_ARM_DEG2RAD(0.0f),
    .theta4_offset_rad = APP_ARM_DEG2RAD(180.0f),
    .theta3_theta4_sum_rad = APP_ARM_DEG2RAD(-85.0f),
};

static float arm_clampf(float v, float min_v, float max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static float wrap_to_2pi(float a) {
    while (a < 0.0f) a += 2.0f * APP_ARM_PI;
    while (a >= 2.0f * APP_ARM_PI) a -= 2.0f * APP_ARM_PI;
    return a;
}

static float normalize_theta1_motor(float theta1_motor_raw) {
    return wrap_to_2pi(theta1_motor_raw);
}

float arm_kinematics_compute_t4_from_t3(const arm_offset_config_t* offset,
                                        float theta3_motor_rad) {
    if (!offset) return 0.0f;
    return offset->theta3_theta4_sum_rad - theta3_motor_rad;
}

void arm_kinematics_forward(const arm_kinematics_params_t* params,
                            const arm_joint_angles_t* angles,
                            arm_pose_t* pose) {
    if (!params || !angles || !pose) return;

    const float r = (params->l2_m * cosf(angles->theta2_geo_rad) +
                     params->l3_m * cosf(angles->theta3_geo_rad));
    const float z = (params->l2_m * sinf(angles->theta2_geo_rad) +
                     params->l3_m * sinf(angles->theta3_geo_rad));

    pose->x_m = r * cosf(angles->theta1_geo_rad);
    pose->y_m = r * sinf(angles->theta1_geo_rad);
    pose->z_m = z;
    pose->pitch_rad = 0.0f;
}

int arm_kinematics_inverse(const arm_kinematics_params_t* params,
                           const arm_offset_config_t* offset,
                           const arm_pose_t* target,
                           arm_joint_angles_t* angles) {
    if (!params || !offset || !target || !angles) return -1;

    const float x = target->x_m;
    const float y = target->y_m;
    const float z = target->z_m;
    const float l2 = params->l2_m;
    const float l3 = params->l3_m;
    const float eps = 1e-6f;

    if (!isfinite(x) || !isfinite(y) || !isfinite(z) ||
        l2 <= 0.0f || l3 <= 0.0f) {
        return -1;
    }

    const float r = sqrtf(x * x + y * y);
    const float distance_sq = r * r + z * z;
    const float distance = sqrtf(distance_sq);

    if (distance < fabsf(l2 - l3) - 1e-6f ||
        distance > l2 + l3 + 1e-6f ||
        distance < eps) {
        return -1;
    }

    const float theta1_geo = (r < eps) ? 0.0f : atan2f(y, x);
    const float phi = atan2f(z, r);
    float cos_delta = (distance_sq + l2 * l2 - l3 * l3) /
                      (2.0f * distance * l2);
    const float delta = acosf(arm_clampf(cos_delta, -1.0f, 1.0f));
    const float theta2_candidate[2] = { phi + delta, phi - delta };

    for (uint32_t candidate = 0U; candidate < 2U; candidate++) {
        const float theta2_geo = theta2_candidate[candidate];
        const float remaining_r = r - l2 * cosf(theta2_geo);
        const float remaining_z = z - l2 * sinf(theta2_geo);
        const float theta3_geo = atan2f(remaining_z, remaining_r);
        const float theta2_motor = theta2_geo - offset->theta2_offset_rad;
        const float theta3_motor = theta3_geo - offset->theta3_offset_rad;
        const float theta2_physical =
            ARM_KINEMATICS_MOTOR2_TO_PHYSICAL(theta2_motor);
        const float theta3_physical =
            ARM_KINEMATICS_MOTOR3_TO_PHYSICAL(theta3_motor);

        const uint8_t theta2_valid =
            (theta2_physical >= ARM_KINEMATICS_MOTOR2_PHYSICAL_MIN_RAD - 1e-6f &&
             theta2_physical <= ARM_KINEMATICS_MOTOR2_PHYSICAL_MAX_RAD + 1e-6f);
        const uint8_t theta3_valid =
            (theta3_physical >= ARM_KINEMATICS_MOTOR3_PHYSICAL_MIN_RAD - 1e-6f &&
             theta3_physical <= ARM_KINEMATICS_MOTOR3_PHYSICAL_MAX_RAD + 1e-6f);
        if (!theta2_valid || !theta3_valid) continue;

        memset(angles, 0, sizeof(*angles));
        angles->theta1_geo_rad = wrap_to_2pi(theta1_geo);
        angles->theta2_geo_rad = theta2_geo;
        angles->theta3_geo_rad = theta3_geo;
        angles->theta1_motor_rad =
            normalize_theta1_motor(theta1_geo - offset->theta1_offset_rad);
        angles->theta2_motor_rad = theta2_motor;
        angles->theta3_motor_rad = theta3_motor;
        angles->theta4_motor_rad =
            arm_kinematics_compute_t4_from_t3(offset, theta3_motor);
        angles->theta4_geo_rad =
            angles->theta4_motor_rad + offset->theta4_offset_rad;
        return 0;
    }

    return -2;
}
