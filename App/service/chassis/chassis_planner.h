/*
 * chassis_planner.h — vx/wz 自运动规划
 *
 * 输入上位机线速度和转向角速度，输出动态步态参数与左右轮差速。
 * 本层不依赖 IMU，也不触碰电机外设。
 */
#ifndef APP_SERVICE_CHASSIS_CHASSIS_PLANNER_H_
#define APP_SERVICE_CHASSIS_CHASSIS_PLANNER_H_

#include "err.h"
#include "gait_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float vx_m_s;
    float vy_m_s;
    float wz_rad_s;
} chassis_cmd_plan_t;

typedef struct {
    gait_params_t gait_params;
    float wheel_rads[GAIT_LEG_NUM];
    uint8_t moving;
} chassis_plan_t;

void chassis_planner_init(void);
app_err_t chassis_planner_update(const chassis_cmd_plan_t* cmd,
                                  const gait_params_t* base_trot,
                                  chassis_plan_t* out);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_CHASSIS_CHASSIS_PLANNER_H_ */
