/*
 * attitude_estimator.h — 姿态估计器
 *
 * 当前实现:
 *   - 陀螺仪 Z 轴积分 → 偏航角 (yaw)
 *   - 陀螺仪 X/Y 积分 + 加速度计低通修正 → roll/pitch
 *   - 静止漂移抑制: gyro 模长 < 阈值时锁定 yaw 积分
 *   - 后续可扩展为完整 AHRS (Mahony / Madgwick)
 *
 * 接口设计:
 *   - 不依赖具体 IMU 型号, 接收 float gyro[3]/accel[3] 即可
 *   - attitude_if.h 定义姿态状态类型, 上层通过此头文件访问
 */
#ifndef APP_SERVICE_ATTITUDE_ATTITUDE_ESTIMATOR_H_
#define APP_SERVICE_ATTITUDE_ATTITUDE_ESTIMATOR_H_

#include "attitude_if.h"
#include "err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化, 偏航角归零 */
app_err_t attitude_estimator_init(void);

/*
 * 更新估计器。
 *   gyro[3]:  rad/s, 三轴角速度
 *   accel[3]: m/s^2, 三轴加速度，用于 roll/pitch 重力方向修正
 *   dt_s:     距上次更新的时间步长 (秒)
 */
app_err_t attitude_estimator_update(const float gyro[3], const float accel[3], float dt_s);

/* 获取当前偏航角 (rad) */
float attitude_estimator_get_yaw(void);

/* 偏航角归零 (用于重置航向参考) */
void attitude_estimator_reset_yaw(void);

/* 获取完整姿态状态 */
const attitude_state_t* attitude_estimator_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_ATTITUDE_ATTITUDE_ESTIMATOR_H_ */
