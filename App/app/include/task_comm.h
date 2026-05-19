/*
 * task_comm.h — USB CDC 帧解析任务 + 分发
 */
#ifndef APP_TASK_COMM_H_
#define APP_TASK_COMM_H_

#include <stdint.h>

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
    uint8_t steer_mode;      /* 0=OFF(现有行为), 1=YAW(偏航闭环) */
    uint32_t seq;
} task_comm_chassis_cmd_t;

void task_comm_get_chassis(task_comm_chassis_cmd_t* out);

/* 最近一次"任何有效帧"的 ms 时戳；用于心跳超时判定 */
uint32_t task_comm_last_rx_ms(void);

#ifdef __cplusplus
}
#endif

#endif
