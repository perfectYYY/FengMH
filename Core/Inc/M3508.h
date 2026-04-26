/*
 * M3508.h
 *
 *  Created on: Apr 18, 2026
 *      Author: FMI
 */

#ifndef INC_M3508_H_
#define INC_M3508_H_

#include "main.h"
#include "stm32h7xx_hal.h"

#define MOTOR_3508_number 4                               // 当前工程内实际接入并管理的 3508 电机总数
#define MOTOR_HISTORY 20                                 // 每个电机保留的历史数据长度
#define M3508_ENCODER_COUNTS_PER_REV 8192.0f             // 转子侧单圈编码器计数
#define M3508_CURRENT_COMMAND_RAW_LIMIT 16384.0f         // C620 电流指令 raw 的绝对值上限
#define M3508_CURRENT_LIMIT_A 20.0f                      // raw 满量程对应的电流上限，单位 A
#define M3508_TORQUE_CONSTANT_NM_PER_A 0.01562f          // 转子侧转矩常数 Kt，单位 N*m/A
#define M3508_SPEED_TORQUE_GRADIENT_RAD_S_PER_NM 0.393f  // 转子侧转速-转矩梯度常数，单位 rad/s*N*m

typedef struct
{
    int number;                        // 结构体编号，一般与 motor_id 一一对应
    uint8_t motor_id;                  // 内部电机 ID，目前只使用 0~3
    int32_t now_Pos;                   // 当前累计编码器计数
    int16_t now_W;                     // 当前反馈转速，原始单位仍为 rpm
    int16_t now_current_raw;           // 当前反馈电流原始值，单位语义尚未标定，调试时按 raw 看待
    uint8_t now_temp;                  // 当前温度
    int16_t round_cnt;                 // 当前累计圈数
    float history_current_raw[MOTOR_HISTORY];  // 反馈电流原始值历史
    float history_W[MOTOR_HISTORY];    // 转速历史
    float history_Pos[MOTOR_HISTORY];  // 位置历史
    float history_Temp[MOTOR_HISTORY]; // 温度历史
    uint8_t write_index;               // 环形缓冲区下一次写入索引
    uint8_t count;                     // 当前历史数据有效长度
    uint32_t numb_updates;             // 历史总更新次数
} Motor_3508_Instance;

typedef enum
{
    CAN_3508_CAN1_ID = 0x200,  // CAN1 上 3508 统一控制帧 ID
    CAN_3508_CAN2_ID = 0x200,  // CAN2 上统一控制帧 ID；当前工程 CAN2 的两个 C620 使用 DJI ID 3/4
    CAN_3508_M1_ID = 0x201,    // CAN1 第 1 个 3508 反馈帧 ID
    CAN_3508_M2_ID = 0x202,    // CAN1 第 2 个 3508 反馈帧 ID
    CAN_3508_M3_ID = 0x203,    // CAN2 上 DJI ID 3 的反馈帧 ID，对应内部 ID 2
    CAN_3508_M4_ID = 0x204,    // CAN2 上 DJI ID 4 的反馈帧 ID，对应内部 ID 3
    CAN_3508_M5_ID = 0x205,    // 协议保留 ID，当前这套双总线 1/2 + 3/4 配置不使用
    CAN_3508_M6_ID = 0x206,    // 协议保留 ID，当前这套双总线 1/2 + 3/4 配置不使用
    CAN_3508_M7_ID = 0x207,    // 协议保留 ID，当前工程未使用
    CAN_3508_M8_ID = 0x208     // 协议保留 ID，当前工程未使用
} can_msg_id;

typedef struct
{
    uint16_t ecd;           // 当前编码器原始值，0~8191
    int16_t speed_rpm;      // 当前反馈转速，单位 rpm，且为转子侧机械转速
    int16_t feedback_current_raw;  // 当前反馈电流原始值，就是调试时最该盯的那一项，暂时不要直接按 A 理解
    uint8_t temperate;      // 当前温度
    int16_t given_current;  // 最近一次下发的 raw 电流指令，用于调试
    uint16_t last_ecd;      // 上一次编码器原始值
    uint16_t offset_ecd;    // 编码器零偏
    int32_t total_angle;    // 相对初始位置的累计编码器计数
    int16_t round_cnt;      // 相对初始位置的累计圈数
    uint32_t msg_cnt;       // 累计收到的反馈帧数量
} motor_measure_t;

extern motor_measure_t motor_3508_can[MOTOR_3508_number];           // 原始反馈缓存，兼容旧代码
extern Motor_3508_Instance motor_3508_instance[MOTOR_3508_number];  // 电机历史/当前状态缓存，兼容旧代码

void my_can_filter_init_recv_all(void);
void M3508_Legacy_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void get_motor_measure(motor_measure_t *ptr, uint8_t *data);
HAL_StatusTypeDef set_motor_current_can1(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
HAL_StatusTypeDef set_motor_current_can2(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

float M3508_CurrentRawToAmpere(int16_t current_raw);   // 指令侧 raw 电流 -> A
int16_t M3508_CurrentAmpereToRaw(float current_a);     // 指令侧 A -> raw 电流指令
float M3508_CurrentToTorqueNm(float current_a);        // 指令侧电流模型 -> 转子侧估计力矩，不要直接拿反馈 raw 套这个函数
float M3508_TorqueNmToCurrent(float torque_nm);        // 转子侧力矩 -> 等效电流
float M3508_RpmToRadPerSec(float speed_rpm);           // rpm -> rad/s
float M3508_EncoderCountToRad(int32_t encoder_count);  // 累计编码器计数 -> rad

void PID_M3508_CAN_Init(void);
void PID_M3508_CAN1(int16_t target_rpm1, int16_t target_rpm2, int16_t target_rpm3, int16_t target_rpm4);
void PID_M3508_CAN2(int16_t target_rpm1, int16_t target_rpm2, int16_t target_rpm3, int16_t target_rpm4);

void Motor_3508_Instance_Init(Motor_3508_Instance *inst, int number);
void Motor_3508_Instance_Update(Motor_3508_Instance *inst, int16_t now_current_raw, int16_t W, int32_t Pos, uint8_t Temp);


#endif /* INC_M3508_H_ */
