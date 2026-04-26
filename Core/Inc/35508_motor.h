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
    float last_angle;                   // 上一次的角度值，单位为编码器计数
    int total_round;                   // 累计的圈数，用于计算 total_angle
    int64_t total_angle;               // 累计的角度值，单位为编码器计数
    float target_speed;                // 位置环计算出的目标速度，单位 rpm
    int64_t target_angle;              // 位置环的目标角度，单位为编码器计数
    int16_t Out_Current;
    float filter_speed;
    float speed_err_sum;
    float position_err_sum;
    float torque_err_sum;
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