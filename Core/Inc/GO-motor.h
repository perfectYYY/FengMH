/*
 * GO-motor.h
 *
 *  Created on: Nov 29, 2025
 *      Author: FMI
 */

#ifndef INC_GO_MOTOR_H_
#define INC_GO_MOTOR_H_

#include "main.h"
#include "stm32h7xx_hal.h"

//配置参数
#define MOTOR_NUM         12       // 电机数量

#define MOTOR_HISTORY 20          // 选择记录存储电机反馈的次数：20次记录

//结构体大军：
//RIS_Mode  码，模式
//RIS_Comd  码，发送
//RIS_Fbk   码，接收
//RIS_SendData  码，发送控制结构，有包头、模式、数据以及CRC
//RIS_BackData    码，返回数据结构，有包头、模式、数据以及CRC
//MotorCmd  用户可用函数，发
//MotorBack 用户可用函数，收

//模式控制信息
#pragma pack(1)             //最大对齐 1Byte，就不进行自动对齐了

typedef struct
{
    uint8_t id : 4;      // 电机ID: 0,1...,13,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status : 3;  // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t reserve : 1; // 保留位
} RIS_Mode;            // 控制模式 1Byte

//发，电机状态信息
typedef struct
{
    int16_t tor_des; // 期望关节输出扭矩 unit: N.m      (q8)
    int16_t spd_des; // 期望关节输出速度 unit: rad/s    (q8)
    int32_t pos_des; // 期望关节输出位置 unit: rad      (q15)
    int16_t k_pos;   // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    int16_t k_spd;   // 期望关节阻尼系数 unit: -1.0-1.0 (q15)

} RIS_Comd; // 控制参数 12Byte

//收，电机状态反馈信息
typedef struct
{
    int16_t torque;      // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t speed;       // 实际关节输出速度 unit: rad/s   (q8)
    int32_t pos;         // 实际关节输出位置 unit: rad     (q15)
    int8_t temp;         // 电机温度: -128~127°C
    uint8_t MError : 3;  // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    uint16_t force : 12; // 足端气压传感器数据 12bit (0-4095)
    uint8_t none : 1;    // 保留位
} RIS_Fbk;             // 状态数据 11Byte

//发，电机码，未翻译码结构体框架
typedef struct
{
    uint8_t head[2]; // 包头         2Byte
    RIS_Mode mode; // 电机控制模式  1Byte
    RIS_Comd comd; // 电机期望数据 12Byte
    uint16_t CRC16;  // CRC          2Byte

} RIS_SendData; // 主机控制命令     17Byte

//收，电机码，未翻译码结构体框架
typedef struct
{
    uint8_t head[2]; // 包头         2Byte
    RIS_Mode mode; // 电机控制模式  1Byte
    RIS_Fbk fbk;   // 电机反馈数据 11Byte
    uint16_t CRC16;  // CRC          2Byte

} RIS_BackData; // 电机返回数据     16Byte

#pragma pack()              //回复自动对齐，4/8/16

//发，电机参数结构体
typedef struct
{
    uint8_t id;   // 电机ID，15代表广播数据包
    uint8_t mode; // 0:空闲 1:FOC控制 2:电机标定
    float T;             // 期望关节的输出力矩(电机本身的力矩)(Nm)
    float W;             // 期望关节速度(电机本身的速度)(rad/s)
    float Pos;           // 期望关节位置(rad)
    float K_P;           // 关节刚度系数(0-25.599)
    float K_W;           // 关节速度系数(0-25.599)

    RIS_SendData motor_send_data;

} MotorCmd;

//收，电机参数结构体
typedef struct
{
    uint8_t motor_id; // 电机ID
    uint8_t mode;     // 0:空闲 1:FOC控制 2:电机标定
    int8_t Temp;               // 温度
    uint8_t MError;             // 错误码
    float T;                // 当前实际电机输出力矩(电机本身的力矩)(Nm)
    float W;                // 当前实际电机速度(电机本身的速度)(rad/s)
    float Pos;              // 当前电机位置(rad)
    int correct;            // 接收数据是否完整(1完整，0不完整)
    uint16_t footForce;          // 足端力传感器原始数值

    uint16_t calc_crc;
    uint32_t timeout;       // 通讯超时 数量
    uint32_t bad_msg;       // CRC校验错误 数量
    RIS_BackData motor_back_data; // 电机接收数据结构体

} MotorBack;


//参数指令转码，指令 => 码
void modify_data(MotorCmd *motor_s);
//解码，码 => 指令
void extract_data(MotorBack *motor_r);

// 设置单个电机参数
void MotorController_SetCommand(MotorCmd *motor_s,
                              uint8_t motor_id,
                              uint8_t mode,
							  float torque,
							  float speed,
							  float position,
							  float k_p,
							  float k_w);

//存所用宇树电机回传的数据仓库结构体，用串口和id来进行分组,其实只用了id。。。串口是陪同的
// 电机实例结构体：每个串口上的每个电机一个实例
typedef struct {
	int number;                          //这个结构体的序号，一般与电机的id一一匹配，id为0，number就为0
    uint8_t motor_id;                    // 电机ID (0~11)
    UART_HandleTypeDef *huart;           // 所属串口句柄指针，计算的方法就是[number+3]/3

    // 当前最新值（方便快速访问）
    float now_T;                         // 当前力矩 (Nm)
    float now_W;                         // 当前速度 (rad/s)
    float now_Pos;                       // 当前位置

    // 历史记录（环形缓冲区）
    float history_T[MOTOR_HISTORY];  // 力矩历史
    float history_W[MOTOR_HISTORY];  // 速度历史
    float history_Pos[MOTOR_HISTORY]; // 位置历史

    uint8_t write_index;                 // 下一次写入的位置索引 (0~19)
    uint8_t count;                       // 当前已存入的有效数据点数量（<=20）
    uint32_t numb_updates;               // 总更新次数（用于调试或统计）
} MotorInstance;

//辅助函数,初始化一个电机实例
void MotorInstance_Init(MotorInstance* inst, uint8_t number);

// 更新最新数据，并存入历史环形缓冲区
void MotorInstance_Update(MotorInstance* inst, float T, float W, float Pos);


// 发送单个电机指令
void MotorController_SendCommand(UART_HandleTypeDef *huart,
                              MotorCmd *motor_s);

//单字节 CRC 更新函数
uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);

//CRC 计算
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len);

#endif /* INC_GO_MOTOR_H_ */

