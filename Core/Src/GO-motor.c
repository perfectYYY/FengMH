/*
 * GO-motor.c
 *
 *  Created on: Nov 30, 2025
 *      Author: FMI
 */
#include "GO-motor.h"
#include <string.h>
#include <usart.h>

#define SATURATE(_IN, _MIN, _MAX) \
	{                             \
		if ((_IN) <= (_MIN))      \
			(_IN) = (_MIN);       \
		else if ((_IN) >= (_MAX)) \
			(_IN) = (_MAX);       \
	}

//对数据进行编码
void modify_data(MotorCmd *motor_s){
    motor_s->motor_send_data.head[0] = 0xFE;
	motor_s->motor_send_data.head[1] = 0xEE;

	SATURATE(motor_s->id, 0, 15);
	SATURATE(motor_s->mode, 0, 3);
	SATURATE(motor_s->K_P, 0.0f, 25.599f);
	SATURATE(motor_s->K_W, 0.0f, 25.599f);
	SATURATE(motor_s->T, -127.99f, 127.99f);
	SATURATE(motor_s->W, -804.00f, 804.00f);
	SATURATE(motor_s->Pos, -411774.0f, 411774.0f);

	motor_s->motor_send_data.mode.id = motor_s->id;
	motor_s->motor_send_data.mode.status = motor_s->mode;
	motor_s->motor_send_data.comd.k_pos = motor_s->K_P / 25.6f * 32768;
	motor_s->motor_send_data.comd.k_spd = motor_s->K_W / 25.6f * 32768;
	motor_s->motor_send_data.comd.pos_des = motor_s->Pos / 6.28318f * 32768;
	motor_s->motor_send_data.comd.spd_des = motor_s->W / 6.28318f * 256;
	motor_s->motor_send_data.comd.tor_des = motor_s->T * 256.0f;
	motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);

}

//对数据进行解码
void extract_data(MotorBack *motor_r){

	if (motor_r->motor_back_data.head[0] != 0xFD || motor_r->motor_back_data.head[1] != 0xEE)
	{
		motor_r->correct = 0;
		return;
	}//正常工作
	motor_r->calc_crc = crc_ccitt(0, (uint8_t *)&motor_r->motor_back_data, 14);
	if (motor_r->motor_back_data.CRC16 == motor_r->calc_crc)
	{
		motor_r->motor_id = motor_r->motor_back_data.mode.id;
		motor_r->mode = motor_r->motor_back_data.mode.status;
		motor_r->Temp = motor_r->motor_back_data.fbk.temp;
		motor_r->MError = motor_r->motor_back_data.fbk.MError;
		motor_r->W = ((float)motor_r->motor_back_data.fbk.speed / 256) * 6.28318f;
		motor_r->T = ((float)motor_r->motor_back_data.fbk.torque) / 256;
		motor_r->Pos = 6.28318f * ((float)motor_r->motor_back_data.fbk.pos) / 32768;
		motor_r->footForce = motor_r->motor_back_data.fbk.force;
		motor_r->correct = 1;
		return;
	}
	else
	{
		memset(&motor_r->motor_back_data, 0, sizeof(RIS_BackData));
		motor_r->correct = 0;
		motor_r->bad_msg++;
		return;
	}
}


void MotorController_SetCommand(MotorCmd *motor_s,
                              uint8_t motor_id,
                              uint8_t mode,
							  float torque,
							  float speed,
							  float position,
							  float k_p,
							  float k_w){
    if ( motor_id >= MOTOR_NUM) return;
    memset(motor_s, 0, sizeof(MotorCmd));//清空数据的一步
    motor_s->id = motor_id;
    motor_s->mode = mode;
    motor_s->T = torque;
    motor_s->W = speed;
    motor_s->Pos = position;
    motor_s->K_P = k_p;
    motor_s->K_W = k_w;

    modify_data(motor_s);     // 对发送数据进行编码

}

void MotorController_SendCommand(UART_HandleTypeDef *huart, MotorCmd *motor_s){
    if (motor_s->id >= MOTOR_NUM ) {
        return; // 电机ID错误，退出
    }

    // 启动DMA发送
    HAL_UART_Transmit_DMA(huart, (uint8_t*)&motor_s->motor_send_data, sizeof(motor_s->motor_send_data));
    HAL_Delay(1);
}



const uint16_t crc_ccitt_table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c)
{
	return (crc >> 8) ^ crc_ccitt_table[(crc ^ c) & 0xff];
}

uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len)
{
	while (len--)
		crc = crc_ccitt_byte(crc, *buffer++);
	return crc;
}


//在一切的校验后，将会进行记录的留存，这样就需要对应的结构体MotorInstance。接下来出场的是初始化和存数据函数
// 初始化一个电机实例
void MotorInstance_Init(MotorInstance* inst, uint8_t number) {
    inst->number = (int)number;
	inst->motor_id = number;
    switch((number+3)/3){
      case 1:
    	  inst->huart = &huart3;
          break;
      case 2:
    	  inst->huart = &huart2;
          break;
      case 3:
    	  inst->huart = &huart3;
          break;
      case 4:
    	  inst->huart = &huart2;
          break;
          //之所以进行部分的注释是因为我的板子还没开huart1、3、4，编译会错误，所有就注释一下吧
    }
    inst->now_T = 0.0f;
    inst->now_W = 0.0f;
    inst->now_Pos = 0.0f;
    inst->write_index = 0;
    inst->count = 0;
    inst->numb_updates = 0;

    // 可选：清零历史数组（通常不是必须，但更干净）
    for (int i = 0; i < MOTOR_HISTORY; i++) {
        inst->history_T[i] = 0.0f;
        inst->history_W[i] = 0.0f;
        inst->history_Pos[i] = 0.0f;
    }
}


// 更新最新数据，并存入历史环形缓冲区
void MotorInstance_Update(MotorInstance* inst, float T, float W, float Pos) {
    // 更新当前值
    inst->now_T = T;
    inst->now_W = W;
    inst->now_Pos = Pos;

    // 写入历史
    inst->history_T[inst->write_index] = T;
    inst->history_W[inst->write_index] = W;
    inst->history_Pos[inst->write_index] = Pos;

    // 更新索引和计数
    inst->write_index = (inst->write_index + 1) % MOTOR_HISTORY;
    if (inst->count < MOTOR_HISTORY) {
        inst->count++;
    }

    inst->numb_updates++;
}
