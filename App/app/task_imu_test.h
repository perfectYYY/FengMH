/*
 * task_imu_test.h - BMI088 board smoke test task
 */
#ifndef APP_APP_TASK_IMU_TEST_H_
#define APP_APP_TASK_IMU_TEST_H_

#ifdef __cplusplus
extern "C" {
#endif

void task_imu_test_init(void);
void task_imu_test_entry(void* arg);

#ifdef __cplusplus
}
#endif

#endif /* APP_APP_TASK_IMU_TEST_H_ */
