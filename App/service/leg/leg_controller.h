/*
 * leg_controller.h — 腿控制器：把 gait_output 翻译成"对每条腿的关节/轮电机指令"
 *
 * M2 阶段只做骨架：
 *   - 持有 motor_dev_t* hip / knee / wheel（来自 motor_registry）
 *   - apply(out)：把 gait_leg_target_t 推送给三个电机
 * IK 接入留到 M3 与 kinematics 一起做。
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

void      leg_controller_init(leg_controller_t* lc);
/* 从 motor_registry 中按约定 logical id 装配 */
app_err_t leg_controller_bind_from_registry(leg_controller_t* lc);
/* 设置站立高度 (IK 解算参数) */
void      leg_controller_set_stand_height(float h);
/* 应用步态输出：先做 IK 解算，再推送给电机 */
app_err_t leg_controller_apply(leg_controller_t* lc, const gait_output_t* o);

#ifdef __cplusplus
}
#endif

#endif
