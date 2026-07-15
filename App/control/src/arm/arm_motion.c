/*
 * arm_motion.c - Synchronized quintic joint trajectory generator.
 */
#include "arm_motion.h"

#include <math.h>
#include <stddef.h>

#define QUINTIC_VELOCITY_FACTOR      1.875f
#define QUINTIC_ACCELERATION_FACTOR  5.773503f
#define DURATION_MARGIN              1.05f
#define LIMIT_CHECK_SAMPLES          64U
#define LIMIT_ADJUST_ATTEMPTS        10U
#define POSITION_EPSILON_RAD         0.0001f

typedef struct {
    float coefficient[ARM_MOTION_JOINT_COUNT][6];
    float target[ARM_MOTION_JOINT_COUNT];
    arm_motion_sample_t sample;
    arm_motion_limits_t limits;
    uint32_t start_ms;
    float duration_s;
    arm_trajectory_state_t state;
} arm_motion_ctx_t;

static arm_motion_ctx_t s_motion;

static float motion_clampf(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void calculate_coefficients(const arm_motion_sample_t* start,
                                   const float target[ARM_MOTION_JOINT_COUNT],
                                   float duration_s) {
    const float t = duration_s;
    const float t2 = t * t;
    const float t3 = t2 * t;
    const float t4 = t3 * t;
    const float t5 = t4 * t;

    for (uint32_t i = 0U; i < ARM_MOTION_JOINT_COUNT; i++) {
        const float q0 = start->position[i];
        const float v0 = start->velocity[i];
        const float acc0 = start->acceleration[i];
        const float dq = target[i] - q0;

        s_motion.coefficient[i][0] = q0;
        s_motion.coefficient[i][1] = v0;
        s_motion.coefficient[i][2] = 0.5f * acc0;
        s_motion.coefficient[i][3] =
            (20.0f * dq - 12.0f * v0 * t - 3.0f * acc0 * t2) / (2.0f * t3);
        s_motion.coefficient[i][4] =
            (-30.0f * dq + 16.0f * v0 * t + 3.0f * acc0 * t2) / (2.0f * t4);
        s_motion.coefficient[i][5] =
            (12.0f * dq - 6.0f * v0 * t - acc0 * t2) / (2.0f * t5);
    }
}

static void evaluate(float time_s, arm_motion_sample_t* sample) {
    const float t2 = time_s * time_s;
    const float t3 = t2 * time_s;
    const float t4 = t3 * time_s;
    const float t5 = t4 * time_s;

    for (uint32_t i = 0U; i < ARM_MOTION_JOINT_COUNT; i++) {
        const float* c = s_motion.coefficient[i];
        sample->position[i] = c[0] + c[1] * time_s + c[2] * t2 +
                              c[3] * t3 + c[4] * t4 + c[5] * t5;
        sample->velocity[i] = c[1] + 2.0f * c[2] * time_s +
                              3.0f * c[3] * t2 + 4.0f * c[4] * t3 +
                              5.0f * c[5] * t4;
        sample->acceleration[i] = 2.0f * c[2] + 6.0f * c[3] * time_s +
                                  12.0f * c[4] * t2 + 20.0f * c[5] * t3;
    }
}

static float initial_duration(const arm_motion_sample_t* start,
                              const float target[ARM_MOTION_JOINT_COUNT]) {
    float duration = s_motion.limits.min_duration_s;

    for (uint32_t i = 0U; i < ARM_MOTION_JOINT_COUNT; i++) {
        const float distance = fabsf(target[i] - start->position[i]);
        const float vmax = s_motion.limits.max_velocity[i];
        const float amax = s_motion.limits.max_acceleration[i];
        const float velocity_time = QUINTIC_VELOCITY_FACTOR * distance / vmax;
        const float acceleration_time =
            sqrtf(QUINTIC_ACCELERATION_FACTOR * distance / amax);
        const float continuity_time = 2.0f * fabsf(start->velocity[i]) / amax;

        if (velocity_time > duration) duration = velocity_time;
        if (acceleration_time > duration) duration = acceleration_time;
        if (continuity_time > duration) duration = continuity_time;
    }

    return motion_clampf(duration * DURATION_MARGIN,
                         s_motion.limits.min_duration_s,
                         s_motion.limits.max_duration_s);
}

static uint8_t trajectory_within_limits(float duration_s) {
    arm_motion_sample_t check = {0};

    for (uint32_t n = 0U; n <= LIMIT_CHECK_SAMPLES; n++) {
        evaluate(duration_s * (float)n / (float)LIMIT_CHECK_SAMPLES, &check);
        for (uint32_t i = 0U; i < ARM_MOTION_JOINT_COUNT; i++) {
            if (fabsf(check.velocity[i]) >
                s_motion.limits.max_velocity[i] * 1.001f) {
                return 0U;
            }
            if (fabsf(check.acceleration[i]) >
                s_motion.limits.max_acceleration[i] * 1.001f) {
                return 0U;
            }
        }
    }
    return 1U;
}

void arm_motion_init(const arm_motion_limits_t* limits) {
    const arm_motion_limits_t defaults = {
        .max_velocity = { 0.90f, 0.75f, 0.75f, 1.20f },
        .max_acceleration = { 2.70f, 2.25f, 2.25f, 3.60f },
        .min_duration_s = 0.20f,
        .max_duration_s = 15.0f,
    };

    s_motion = (arm_motion_ctx_t){0};
    s_motion.limits = limits ? *limits : defaults;
    for (uint32_t i = 0U; i < ARM_MOTION_JOINT_COUNT; i++) {
        if (s_motion.limits.max_velocity[i] <= 0.0f) {
            s_motion.limits.max_velocity[i] = defaults.max_velocity[i];
        }
        if (s_motion.limits.max_acceleration[i] <= 0.0f) {
            s_motion.limits.max_acceleration[i] = defaults.max_acceleration[i];
        }
    }
    if (s_motion.limits.min_duration_s <= 0.0f) {
        s_motion.limits.min_duration_s = defaults.min_duration_s;
    }
    if (s_motion.limits.max_duration_s < s_motion.limits.min_duration_s) {
        s_motion.limits.max_duration_s = defaults.max_duration_s;
    }
    s_motion.state = ARM_TRAJECTORY_IDLE;
}

void arm_motion_set_limits(const arm_motion_limits_t* limits) {
    if (!limits) return;

    for (uint32_t i = 0U; i < ARM_MOTION_JOINT_COUNT; i++) {
        if (isfinite(limits->max_velocity[i]) &&
            limits->max_velocity[i] > 0.0f) {
            s_motion.limits.max_velocity[i] = limits->max_velocity[i];
        }
        if (isfinite(limits->max_acceleration[i]) &&
            limits->max_acceleration[i] > 0.0f) {
            s_motion.limits.max_acceleration[i] = limits->max_acceleration[i];
        }
    }
    if (isfinite(limits->min_duration_s) && limits->min_duration_s > 0.0f) {
        s_motion.limits.min_duration_s = limits->min_duration_s;
    }
    if (isfinite(limits->max_duration_s) &&
        limits->max_duration_s >= s_motion.limits.min_duration_s) {
        s_motion.limits.max_duration_s = limits->max_duration_s;
    }
}

int arm_motion_start(const arm_motion_sample_t* start,
                     const float target[ARM_MOTION_JOINT_COUNT],
                     uint32_t now_ms) {
    if (!start || !target) return -1;

    float largest_distance = 0.0f;
    for (uint32_t i = 0U; i < ARM_MOTION_JOINT_COUNT; i++) {
        if (!isfinite(start->position[i]) ||
            !isfinite(start->velocity[i]) ||
            !isfinite(start->acceleration[i]) ||
            !isfinite(target[i])) {
            return -1;
        }
        const float distance = fabsf(target[i] - start->position[i]);
        if (distance > largest_distance) largest_distance = distance;
        s_motion.target[i] = target[i];
    }

    s_motion.sample = *start;
    s_motion.start_ms = now_ms;

    if (largest_distance <= POSITION_EPSILON_RAD) {
        for (uint32_t i = 0U; i < ARM_MOTION_JOINT_COUNT; i++) {
            s_motion.sample.position[i] = target[i];
            s_motion.sample.velocity[i] = 0.0f;
            s_motion.sample.acceleration[i] = 0.0f;
        }
        s_motion.duration_s = 0.0f;
        s_motion.state = ARM_TRAJECTORY_FINISHED;
        return 0;
    }

    s_motion.duration_s = initial_duration(start, target);
    uint8_t limits_satisfied = 0U;
    for (uint32_t attempt = 0U; attempt < LIMIT_ADJUST_ATTEMPTS; attempt++) {
        calculate_coefficients(start, target, s_motion.duration_s);
        if (trajectory_within_limits(s_motion.duration_s)) {
            limits_satisfied = 1U;
            break;
        }
        s_motion.duration_s = motion_clampf(s_motion.duration_s * 1.25f,
                                            s_motion.limits.min_duration_s,
                                            s_motion.limits.max_duration_s);
    }

    calculate_coefficients(start, target, s_motion.duration_s);
    if (!limits_satisfied && !trajectory_within_limits(s_motion.duration_s)) {
        s_motion.state = ARM_TRAJECTORY_IDLE;
        return -2;
    }

    s_motion.state = ARM_TRAJECTORY_MOVING;
    return 0;
}

arm_trajectory_state_t arm_motion_update(uint32_t now_ms,
                                         arm_motion_sample_t* sample) {
    if (s_motion.state == ARM_TRAJECTORY_MOVING) {
        const float elapsed_s = (float)(now_ms - s_motion.start_ms) * 0.001f;
        if (elapsed_s >= s_motion.duration_s) {
            for (uint32_t i = 0U; i < ARM_MOTION_JOINT_COUNT; i++) {
                s_motion.sample.position[i] = s_motion.target[i];
                s_motion.sample.velocity[i] = 0.0f;
                s_motion.sample.acceleration[i] = 0.0f;
            }
            s_motion.state = ARM_TRAJECTORY_FINISHED;
        } else {
            evaluate(elapsed_s, &s_motion.sample);
        }
    }

    if (sample) *sample = s_motion.sample;
    return s_motion.state;
}

void arm_motion_stop(const arm_motion_sample_t* hold_sample) {
    if (hold_sample) s_motion.sample = *hold_sample;
    for (uint32_t i = 0U; i < ARM_MOTION_JOINT_COUNT; i++) {
        s_motion.sample.velocity[i] = 0.0f;
        s_motion.sample.acceleration[i] = 0.0f;
    }
    s_motion.state = ARM_TRAJECTORY_IDLE;
}

arm_trajectory_state_t arm_motion_get_state(void) {
    return s_motion.state;
}

void arm_motion_get_sample(arm_motion_sample_t* sample) {
    if (sample) *sample = s_motion.sample;
}

float arm_motion_get_duration(void) {
    return s_motion.duration_s;
}

float arm_motion_get_progress(uint32_t now_ms) {
    if (s_motion.state == ARM_TRAJECTORY_FINISHED) return 1.0f;
    if (s_motion.state != ARM_TRAJECTORY_MOVING ||
        s_motion.duration_s <= 0.0f) {
        return 0.0f;
    }
    return motion_clampf(((float)(now_ms - s_motion.start_ms) * 0.001f) /
                         s_motion.duration_s,
                         0.0f,
                         1.0f);
}
