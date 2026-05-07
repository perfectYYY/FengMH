/*
 * M3508.h
 *
 *  Created on: Apr 25, 2026
 *      Author: FMI
 */

#ifndef INC_3508_MOTOR_H_
#define INC_3508_MOTOR_H_

#include "main.h"
#include "stm32h7xx_hal.h"

#define MOTOR_3508_number 4                               // 当前工程内实际接入并管理的 3508 电机总数
#define MOTOR_HISTORY 20                                 // 每个电机保留的历史数据长度

typedef struct
{
    int init_flag;

    int16_t cur_angle;        // 当前角度，编码器值 0~8191
    float last_angle;         // 上一次角度
    int total_round;          // 多圈计数
    int64_t total_angle;      // 累计角度
    float target_speed;       // 位置环输出目标速度
    int64_t target_angle;     // 位置环目标角度
    int16_t cur_speed;        // 当前速度
    int16_t actual_current;   // 当前电流反馈
    int16_t temperature;      // 电机温度
    int16_t Out_Current;      // 电流输出值

    float filter_speed;       // EMA 滤波速度

    float speed_err_sum;      // 速度环积分
    float position_err_sum;   // 位置环积分
    float torque_err_sum;     // 力矩环积分
    float err_last;           // PID 上一次误差
} Motor_3508_T;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Max_Out;
    float Max_Sum;
} PID_3508_T;

void FDCAN1_Filter_Init(void);
void FDCAN2_Filter_Init(void);
void D3508_Init(void);
void send_current(void);
void PID_Calc_Speed(int i);
void PID_Calc_SetSpeed(int i, float target_speed);
void PID_Calc_Position(int i, float target_angle);
void PID_Calc_Torque(int i, float target_torque);
void D3508_Decode(uint8_t* RxData, uint16_t ID);
int16_t Power_Limit(float desire_current, float rpm);

#endif /* INC_3508_MOTOR_H_ */
