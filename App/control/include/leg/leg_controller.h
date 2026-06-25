/*
 * leg_controller.h — 腿控制器：把 gait_output 翻译成"对每条腿的关节/轮电机指令"
 *
 * 持有每条腿的 hip / knee / wheel 电机句柄，把 gait_output_t 的足端
 * 目标先做 IK，再转换成 GO 关节位置指令和 M3508 轮毂指令。
 */
#ifndef APP_SERVICE_LEG_CONTROLLER_H_
#define APP_SERVICE_LEG_CONTROLLER_H_

#include "gait_if.h"
#include "motor_if.h"
#include "motor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    motor_dev_t* hip;
    motor_dev_t* knee;
    motor_dev_t* wheel;
} leg_actuators_t;

typedef struct {
    leg_actuators_t leg[GAIT_LEG_NUM];
    uint32_t        send_cnt;
    uint32_t        miss_cnt;  /* 某条腿电机为空时累加 */
} leg_controller_t;

typedef struct {
    uint8_t  enable;       /* 1=wheel 使用支撑相 MIT 位置积分，0=原速度环 */
    uint8_t  reset;        /* GDB 写 1 重新锁存各轮参考位置 */
    uint8_t  active_mask;  /* bit0..3: 当前处于 MIT 管理的腿 */
    uint8_t  hold_swing;   /* 1=摆动相也保持当前轮角，0=摆动相零电流 */
    float    kp;           /* 输出轴 N·m/rad */
    float    kd;           /* 输出轴 N·m·s/rad */
    float    tau_limit_nm;
    float    pos_err_limit_rad;
    float    theta_ref_rad[GAIT_LEG_NUM];
    uint8_t  ref_valid[GAIT_LEG_NUM];
} leg_wheel_mit_debug_t;

extern volatile leg_wheel_mit_debug_t g_leg_wheel_mit;

void      leg_controller_init(leg_controller_t* lc);
/* 从 motor_registry 中按约定 logical id 装配 */
app_err_t leg_controller_bind_from_registry(leg_controller_t* lc);
/* 设置站立高度 (IK 解算参数) */
void      leg_controller_set_stand_height(float h);
void      leg_controller_set_output_options(uint8_t leg_mask,
                                            uint8_t enable_joints,
                                            uint8_t enable_wheels,
                                            float joint_kp,
                                            float joint_kd);
/* 应用步态输出：先做 IK 解算，再推送给电机 */
app_err_t leg_controller_apply(leg_controller_t* lc, const gait_output_t* o);
app_err_t leg_controller_apply_dt(leg_controller_t* lc, const gait_output_t* o, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
