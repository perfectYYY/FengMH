/* Private Mahony AHRS core adapted from the supplied imu_N reference. */
#ifndef APP_CONTROL_ATTITUDE_MAHONY_AHRS_H_
#define APP_CONTROL_ATTITUDE_MAHONY_AHRS_H_

#include <stdint.h>

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
    float two_kp;
    float two_ki;
    float integral_fb[3];
} mahony_ahrs_t;

void mahony_ahrs_init(mahony_ahrs_t* ahrs, float two_kp, float two_ki);
uint8_t mahony_ahrs_reset_from_accel(mahony_ahrs_t* ahrs,
                                     const float accel[3],
                                     float yaw_rad);
uint8_t mahony_ahrs_update_imu(mahony_ahrs_t* ahrs,
                               const float gyro_rad_s[3],
                               const float accel_mps2[3],
                               float dt_s);
uint8_t mahony_ahrs_get_euler(const mahony_ahrs_t* ahrs,
                              float* roll_rad,
                              float* pitch_rad,
                              float* yaw_rad);
uint8_t mahony_ahrs_set_yaw(mahony_ahrs_t* ahrs, float yaw_rad);

#endif /* APP_CONTROL_ATTITUDE_MAHONY_AHRS_H_ */
