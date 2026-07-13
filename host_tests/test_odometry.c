#include "chassis_odometry.h"
#include "attitude_estimator.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ASSERT_TRUE(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "assertion failed: %s at %s:%d\n", #condition, __FILE__, __LINE__); \
        return 1; \
    } \
} while (0)

static chassis_odometry_input_t valid_input(void) {
    chassis_odometry_input_t input;
    memset(&input, 0, sizeof(input));
    input.wheel_radius_m = 0.05f;
    input.half_track_m = 0.15f;
    input.wheel_measurement_scale = 1.0f;
    input.imu_ready = 1U;
    input.wheel_online_mask = 0x0FU;
    input.stance_mask = 0x0FU;
    input.allow_velocity_correction = 1U;
    input.allow_yaw_correction = 1U;
    return input;
}

static int test_dynamic_acceleration_is_not_gravity(void) {
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    const float gravity[3] = {0.0f, 0.0f, 9.80665f};
    const float dynamic_accel[3] = {5.0f, 0.0f, 9.80665f};

    ASSERT_TRUE(attitude_estimator_init() == APP_OK);
    for (int i = 0; i < 510; i++) {
        ASSERT_TRUE(attitude_estimator_update(gyro, gravity, 0.002f) == APP_OK);
    }
    ASSERT_TRUE(attitude_estimator_is_calibrated());
    for (int i = 0; i < 500; i++) {
        ASSERT_TRUE(attitude_estimator_update(gyro, dynamic_accel, 0.002f) == APP_OK);
    }
    const attitude_state_t* state = attitude_estimator_get_state();
    ASSERT_TRUE(fabsf(state->roll) < 1e-4f);
    ASSERT_TRUE(fabsf(state->pitch) < 1e-4f);
    return 0;
}

int main(void) {
    if (test_dynamic_acceleration_is_not_gravity()) return 1;

    chassis_odometry_input_t input = valid_input();
    for (int i = 0; i < GAIT_LEG_NUM; i++) input.wheel_velocity_rads[i] = 2.0f;

    chassis_odometry_init();
    for (int i = 0; i < 200; i++) chassis_odometry_update(&input, 0.01f);
    const chassis_odometry_state_t* state = chassis_odometry_get_state();
    ASSERT_TRUE(state->quality_flags & CHASSIS_ODOM_FLAG_IMU_READY);
    ASSERT_TRUE(state->quality_flags & CHASSIS_ODOM_FLAG_WHEEL_VALID);
    ASSERT_TRUE(state->quality_flags & CHASSIS_ODOM_FLAG_WHEEL_CORRECTED);
    ASSERT_TRUE(fabsf(state->vx_m_s - 0.10f) < 0.01f);
    ASSERT_TRUE(state->x_m > 0.15f);

    input.transition_gated = 1U;
    chassis_odometry_init();
    for (int i = 0; i < 100; i++) chassis_odometry_update(&input, 0.01f);
    state = chassis_odometry_get_state();
    ASSERT_TRUE(state->quality_flags & CHASSIS_ODOM_FLAG_TRANSITION_GATED);
    ASSERT_TRUE(!(state->quality_flags & CHASSIS_ODOM_FLAG_WHEEL_CORRECTED));
    ASSERT_TRUE(fabsf(state->vx_m_s) < 1e-6f);
    ASSERT_TRUE(fabsf(state->x_m) < 1e-6f);

    printf("chassis_odometry_host_tests: PASS\n");
    return 0;
}
