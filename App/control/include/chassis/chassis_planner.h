/*
 * chassis_planner.h — vx/wz 自运动规划
 *
 * 输入上位机速度命令，输出每腿动态步态参数与每轮滚动速度。
 * vx/wz 按刚体运动学映射到左右侧局部前后速度；vy 保留输入但暂不生成横移轨迹。
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
    float half_track_m;          /* 腿/轮接触点到机体中心线的横向距离 */
    float max_leg_step_m;        /* 单腿步长限幅，<=0 使用默认值 */
    float turn_step_height_m;    /* 原地/低速转向抬脚高度，<=0 使用 base_gait */
    float turn_period_s;         /* 原地/低速转向步态周期，<=0 使用 base_gait */
    float turn_duty;             /* 原地/低速转向支撑占空比，<=0 使用 base_gait */
} chassis_turn_cfg_t;

typedef struct {
    uint8_t enable;              /* 1=按速度联合调度周期和步幅 */
    uint8_t reserved[3];
    float slow_period_s;         /* 低速移动周期，防止低速步频过低 */
    float fast_period_s;         /* 高速移动周期 */
    float fast_speed_m_s;        /* 达到该速度后使用 fast_period_s */
} chassis_stride_cfg_t;

extern volatile chassis_turn_cfg_t g_chassis_turn_cfg;
extern volatile chassis_stride_cfg_t g_chassis_stride_cfg;

void chassis_planner_init(void);
app_err_t chassis_planner_update(const chassis_cmd_plan_t* cmd,
                                  const gait_params_t* base_gait,
                                  chassis_plan_t* out);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_CHASSIS_CHASSIS_PLANNER_H_ */
