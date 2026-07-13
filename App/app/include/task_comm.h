/*
 * task_comm.h — USB CDC 帧解析任务 + 分发
 */
#ifndef APP_TASK_COMM_H_
#define APP_TASK_COMM_H_

#include <stdint.h>
#include "proto_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

void     task_comm_entry(void* arg);
void     task_comm_init(void);   /* 注册解析器到 bsp_usb_cdc + 装配分发表 */

/* 计数器（host 单测、运行期诊断都能读） */
uint32_t task_comm_good_cnt(void);
uint32_t task_comm_bad_cnt(void);
uint32_t task_comm_dispatch_hit(void);
uint32_t task_comm_dispatch_miss(void);

/* 当前最近一次的 chassis 指令（被 dispatch 时更新；task_chassis 读取） */
typedef struct {
    float vx, vy, wz;
    float target_yaw;        /* 目标偏航角 (rad), BMI088 转向 */
    uint8_t steer_mode;      /* PROTO_STEER_MODE_* */
    uint32_t seq;
} task_comm_chassis_cmd_t;

void task_comm_get_chassis(task_comm_chassis_cmd_t* out);

/* 当前最近一次的 arm target 指令；后续 task_arm 读取 */
typedef struct {
    uint8_t target_type;      /* 0=grasp, 1=place */
    float x_m, y_m, z_m;      /* arm_base, m */
    uint32_t seq;
} task_comm_arm_target_t;

/* ARM_TARGET 接收点调试快照：在 mode gate 和机械臂状态机之前更新。 */
extern volatile uint8_t  debug_comm_arm_target_raw[sizeof(payload_arm_target_t)];
extern volatile uint8_t  debug_comm_arm_target_type;
extern volatile float    debug_comm_arm_target_x_m;
extern volatile float    debug_comm_arm_target_y_m;
extern volatile float    debug_comm_arm_target_z_m;
extern volatile uint32_t debug_comm_arm_target_accept_count;
extern volatile uint32_t debug_comm_arm_target_reject_count;

void task_comm_get_arm_target(task_comm_arm_target_t* out);

/* 当前最近一次的 pump 指令；后续 task_arm 读取 */
typedef struct {
    uint8_t pump_on;          /* 1=vacuum on, 0=release */
    uint32_t seq;
} task_comm_arm_pump_t;

void task_comm_get_arm_pump(task_comm_arm_pump_t* out);

/* 上位机请求的整机模式；后续 safety/mode manager 读取并做本地互锁 */
typedef struct {
    uint8_t mode;             /* PROTO_ROBOT_MODE_* */
    uint32_t seq;
} task_comm_mode_cmd_t;

void task_comm_get_mode_cmd(task_comm_mode_cmd_t* out);

/* 机械臂 feedback 上行；后续 task_arm 以此发送 PROTO_FUNC_ARM_FEEDBACK */
int task_comm_send_arm_feedback(const payload_arm_feedback_t* feedback);
/* 机械臂 J1..J4 原始电机角度上行；FuncID=PROTO_FUNC_ARM_MOTOR_ANGLES。 */
int task_comm_send_arm_motor_angles(const payload_arm_motor_angles_t* angles);

/* 四轮实际转速上行；顺序固定为 FL/FR/RL/RR，单位 rad/s。 */
int task_comm_send_wheel_feedback(void);
int task_comm_send_chassis_diag(void);

/* 最近一次"任何有效帧"的 ms 时戳；用于心跳超时判定 */
uint32_t task_comm_last_rx_ms(void);

#ifdef __cplusplus
}
#endif

#endif
