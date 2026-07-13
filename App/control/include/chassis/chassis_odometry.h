/* Planar relative odometry driven by IMU yaw rate and wheel feedback. */
#ifndef APP_CONTROL_CHASSIS_ODOMETRY_H_
#define APP_CONTROL_CHASSIS_ODOMETRY_H_

#include <stdint.h>

#include "gait_if.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CHASSIS_ODOM_FLAG_INITIALIZED      = 1U << 0,
    CHASSIS_ODOM_FLAG_IMU_READY        = 1U << 1,
    CHASSIS_ODOM_FLAG_WHEEL_VALID      = 1U << 2,
    CHASSIS_ODOM_FLAG_WHEEL_CORRECTED  = 1U << 3,
    CHASSIS_ODOM_FLAG_ZERO_VELOCITY    = 1U << 4,
    CHASSIS_ODOM_FLAG_TRANSITION_GATED = 1U << 5,
    CHASSIS_ODOM_FLAG_SLIP_GATED       = 1U << 6,
};

typedef struct {
    float x_m;
    float y_m;
    float yaw_rad;
    float vx_m_s;
    float yaw_rate_rad_s;
    float gyro_z_bias_rad_s;
    uint16_t quality_flags;
    uint8_t stance_mask;
    uint8_t wheel_online_mask;
} chassis_odometry_state_t;

typedef struct {
    float gyro_z_rad_s;
    float wheel_velocity_rads[GAIT_LEG_NUM];
    float wheel_radius_m;
    float half_track_m;
    float wheel_measurement_scale;
    uint8_t imu_ready;
    uint8_t wheel_online_mask;
    uint8_t stance_mask;
    uint8_t allow_velocity_correction;
    uint8_t allow_yaw_correction;
    uint8_t zero_velocity;
    uint8_t transition_gated;
    uint8_t slip_gated;
} chassis_odometry_input_t;

void chassis_odometry_init(void);
void chassis_odometry_update(const chassis_odometry_input_t* input, float dt_s);
void chassis_odometry_reset_pose(float x_m, float y_m, float yaw_rad);
const chassis_odometry_state_t* chassis_odometry_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_CHASSIS_ODOMETRY_H_ */
