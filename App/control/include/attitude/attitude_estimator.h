/* 6-axis Mahony attitude estimator public interface. */
#ifndef APP_SERVICE_ATTITUDE_ATTITUDE_ESTIMATOR_H_
#define APP_SERVICE_ATTITUDE_ATTITUDE_ESTIMATOR_H_

#include <stdint.h>

#include "attitude_if.h"
#include "err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Reset quaternion, gyro-bias calibration state, and relative yaw. */
app_err_t attitude_estimator_init(void);
uint8_t attitude_estimator_is_initialized(void);

/*
 * Update from BMI088 source-frame measurements. The configured body-axis map
 * is applied internally. gyro[3]: rad/s; accel[3]: m/s^2; dt_s: seconds.
 */
app_err_t attitude_estimator_update(const float gyro[3], const float accel[3], float dt_s);

float attitude_estimator_get_yaw(void);
uint8_t attitude_estimator_is_calibrated(void);
void attitude_estimator_get_gyro_bias(float out_bias[3]);

/* Set relative yaw to zero while preserving the current roll and pitch. */
void attitude_estimator_reset_yaw(void);

const attitude_state_t* attitude_estimator_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_ATTITUDE_ATTITUDE_ESTIMATOR_H_ */
