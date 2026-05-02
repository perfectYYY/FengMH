/*
 * steer_controller.h — 转向控制器
 *
 * 模式:
 *   STEER_MODE_OFF      — wz 直通, 不干预步态 (现有行为)
 *   STEER_MODE_YAW      — 偏航角闭环 PID, 输出修正 wz
 *
 * PID 控制律:
 *   yaw_err = wrap_pi(target - current)
 *   wz_out  = kp * yaw_err + ki * ∫err + kd * derr  (clamped)
 */
#ifndef APP_SERVICE_ATTITUDE_STEER_CONTROLLER_H_
#define APP_SERVICE_ATTITUDE_STEER_CONTROLLER_H_

#include "err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STEER_MODE_OFF = 0,       /* wz 直通, 无转向控制 */
    STEER_MODE_YAW,           /* 偏航角闭环 */
} steer_mode_t;

typedef struct {
    steer_mode_t mode;
    float target_yaw;         /* 目标偏航角 (rad) */
    float kp;                 /* 比例增益 */
    float ki;                 /* 积分增益 */
    float kd;                 /* 微分增益 */
    float i_max;              /* 积分限幅 */
    float wz_max;             /* 输出角速度限幅 (rad/s) */
} steer_cfg_t;

app_err_t steer_controller_init(void);
app_err_t steer_controller_set_mode(steer_mode_t mode);
app_err_t steer_controller_set_target_yaw(float yaw_rad);

/*
 * 执行一次控制迭代。
 *   current_yaw: 当前偏航角 (rad)
 *   dt_s:        控制周期 (秒)
 *   返回:         修正后的 wz (rad/s), 若 mode=OFF 则返回 0
 */
float steer_controller_update(float current_yaw, float dt_s);

/* 获取当前配置 (只读) */
const steer_cfg_t* steer_controller_get_cfg(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_ATTITUDE_STEER_CONTROLLER_H_ */
