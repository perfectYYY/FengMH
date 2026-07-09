/*
 * attitude_if.h — 姿态接口定义
 *
 * 定义姿态状态类型, 上层模块 (task_chassis, task_comm) 通过此接口
 * 访问姿态数据, 不依赖具体估计器实现。
 */
#ifndef APP_SERVICE_ATTITUDE_ATTITUDE_IF_H_
#define APP_SERVICE_ATTITUDE_ATTITUDE_IF_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float yaw;      /* 偏航角 (rad), 正方向: 逆时针 (Z-up) */
    float pitch;    /* 俯仰角 (rad), 预留 */
    float roll;     /* 横滚角 (rad), 预留 */
    float yaw_rate;   /* rad/s */
    float pitch_rate; /* rad/s */
    float roll_rate;  /* rad/s */
} attitude_state_t;

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_ATTITUDE_ATTITUDE_IF_H_ */
