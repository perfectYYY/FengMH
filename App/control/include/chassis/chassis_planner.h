/*
 * chassis_planner.h — vx/wz 自运动规划
 *
 * 输入上位机线速度和转向角速度，输出动态步态参数与共同前进轮速。
 * yaw 转向由步态左右步长差实现，不再使用轮子差速。
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

typedef struct {
    uint8_t enable_gait_turn;    /* 1=启用 yaw 步态转向 */
    uint8_t reserved[3];
    float low_vx_thresh_m_s;     /* |vx| 小于该值时按原地/低速转向处理 */
    float max_wheel_rads;        /* 轮速限幅，<=0 表示不额外限幅 */
    float turn_step_m_per_rad_s; /* yaw(rad/s) 到左右步长差(m)的比例 */
    float max_turn_step_m;       /* yaw 步态转向步长差限幅，<=0 使用默认值 */
    float turn_step_height_m;    /* 原地/低速转向抬脚高度，<=0 使用 base_trot */
    float turn_period_s;         /* 原地/低速转向步态周期，<=0 使用 base_trot */
    float turn_duty;             /* 原地/低速转向支撑占空比，<=0 使用 base_trot */
} chassis_turn_cfg_t;

extern volatile chassis_turn_cfg_t g_chassis_turn_cfg;

void chassis_planner_init(void);
app_err_t chassis_planner_update(const chassis_cmd_plan_t* cmd,
                                  const gait_params_t* base_trot,
                                  chassis_plan_t* out);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_CHASSIS_CHASSIS_PLANNER_H_ */
