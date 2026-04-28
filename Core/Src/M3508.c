/*
 * M3508.c
 *
 *  Created on: Apr 18, 2026
 *      Author: FMI
 */

#include "M3508.h"
#include "fdcan.h"
#include "pid.h"
#include <math.h>
#include <string.h>

#define M3508_PI                         3.14159265358979323846f
#define M3508_HALF_ENCODER_RANGE         4096
#define M3508_TX_CURRENT_LIMIT           16384
#define M3508_SPEED_PID_KP               3.0f
#define M3508_SPEED_PID_KI               0.3f
#define M3508_SPEED_PID_KD               0.0f
#define M3508_SPEED_PID_MAX_OUT          16000.0f
#define M3508_SPEED_PID_MAX_SUM          10000.0f
#define M3508_PID_DT_S                   0.005f

motor_measure_t motor_3508_can[MOTOR_3508_number];
Motor_3508_Instance motor_3508_instance[MOTOR_3508_number];

static PID_Controller speed_pid[MOTOR_3508_number];

static int16_t M3508_LimitCurrentRaw(float current_raw)
{
    if (current_raw > (float)M3508_TX_CURRENT_LIMIT)
    {
        return M3508_TX_CURRENT_LIMIT;
    }
    if (current_raw < (float)-M3508_TX_CURRENT_LIMIT)
    {
        return -M3508_TX_CURRENT_LIMIT;
    }
    return (int16_t)current_raw;
}

static void M3508_FillTxHeader(FDCAN_TxHeaderTypeDef *tx_header, uint32_t identifier)
{
    tx_header->Identifier = identifier;
    tx_header->IdType = FDCAN_STANDARD_ID;
    tx_header->TxFrameType = FDCAN_DATA_FRAME;
    tx_header->DataLength = FDCAN_DLC_BYTES_8;
    tx_header->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header->BitRateSwitch = FDCAN_BRS_OFF;
    tx_header->FDFormat = FDCAN_CLASSIC_CAN;
    tx_header->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header->MessageMarker = 0;
}

static HAL_StatusTypeDef M3508_SendCurrent(FDCAN_HandleTypeDef *hfdcan,
                                           int16_t iq1,
                                           int16_t iq2,
                                           int16_t iq3,
                                           int16_t iq4)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    M3508_FillTxHeader(&tx_header, CAN_3508_CAN1_ID);

    /* 0x200 控制帧固定 8 字节，每两个字节对应一个 DJI ID 的电流指令。 */
    tx_data[0] = (uint8_t)(iq1 >> 8);
    tx_data[1] = (uint8_t)iq1;
    tx_data[2] = (uint8_t)(iq2 >> 8);
    tx_data[3] = (uint8_t)iq2;
    tx_data[4] = (uint8_t)(iq3 >> 8);
    tx_data[5] = (uint8_t)iq3;
    tx_data[6] = (uint8_t)(iq4 >> 8);
    tx_data[7] = (uint8_t)iq4;

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
}

static HAL_StatusTypeDef M3508_ConfigOneCan(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter;
    HAL_StatusTypeDef status;

    /* 只接收 0x200~0x20F 这一组标准帧，覆盖 M3508 控制/反馈 ID 段。 */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x200;
    filter.FilterID2 = 0x7F0;

    status = HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                          FDCAN_REJECT,
                                          FDCAN_REJECT,
                                          FDCAN_REJECT_REMOTE,
                                          FDCAN_REJECT_REMOTE);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_FDCAN_ConfigFilter(hfdcan, &filter);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_FDCAN_Start(hfdcan);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

static int M3508_MapFeedbackIndex(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier)
{
    if (hfdcan == &hfdcan1)
    {
        if (identifier == CAN_3508_M1_ID || identifier == CAN_3508_M2_ID)
        {
            return (int)(identifier - CAN_3508_M1_ID);
        }
        return -1;
    }

    if (hfdcan == &hfdcan2)
    {
        if (identifier == CAN_3508_M3_ID || identifier == CAN_3508_M4_ID)
        {
            return (int)(identifier - CAN_3508_M1_ID);
        }
        return -1;
    }

    return -1;
}

static int16_t M3508_SpeedPidCalc(uint8_t index, int16_t target_rpm)
{
    float output;

    if (index >= MOTOR_3508_number)
    {
        return 0;
    }

    output = PID_UpdateDt(&speed_pid[index],
                          (float)target_rpm,
                          (float)motor_3508_can[index].speed_rpm,
                          M3508_PID_DT_S);
    return M3508_LimitCurrentRaw(output);
}

void my_can_filter_init_recv_all(void)
{
    if (M3508_ConfigOneCan(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (M3508_ConfigOneCan(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }
}

void M3508_Legacy_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            return;
        }

        if (rx_header.IdType != FDCAN_STANDARD_ID ||
            rx_header.RxFrameType != FDCAN_DATA_FRAME ||
            rx_header.DataLength != FDCAN_DLC_BYTES_8)
        {
            continue;
        }

        /* 根据反馈帧 ID 和来源总线，把 0x201~0x204 映射到内部 0~3 号电机。 */
        int index = M3508_MapFeedbackIndex(hfdcan, rx_header.Identifier);
        if (index < 0)
        {
            continue;
        }

        get_motor_measure(&motor_3508_can[index], rx_data);
        Motor_3508_Instance_Update(&motor_3508_instance[index],
                                   motor_3508_can[index].feedback_current_raw,
                                   motor_3508_can[index].speed_rpm,
                                   motor_3508_can[index].total_angle,
                                   motor_3508_can[index].temperate);
    }
}

__weak void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    M3508_Legacy_RxFifo0Callback(hfdcan, RxFifo0ITs);
}

void get_motor_measure(motor_measure_t *ptr, uint8_t *data)
{
    uint16_t ecd;
    int32_t delta;

    if (ptr == NULL || data == NULL)
    {
        return;
    }

    ecd = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);

    if (ptr->msg_cnt == 0U)
    {
        ptr->ecd = ecd;
        ptr->last_ecd = ecd;
        ptr->offset_ecd = ecd;
        ptr->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
        ptr->feedback_current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
        ptr->temperate = data[6];
        ptr->round_cnt = 0;
        ptr->total_angle = 0;
        ptr->msg_cnt = 1U;
        return;
    }

    ptr->last_ecd = ptr->ecd;
    ptr->ecd = ecd;
    ptr->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    ptr->feedback_current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    ptr->temperate = data[6];

    /* 8192 线编码器跨零点时，delta 会突然大于半圈，用它判断圈数增减。 */
    delta = (int32_t)ptr->ecd - (int32_t)ptr->last_ecd;
    if (delta > M3508_HALF_ENCODER_RANGE)
    {
        ptr->round_cnt--;
    }
    else if (delta < -M3508_HALF_ENCODER_RANGE)
    {
        ptr->round_cnt++;
    }

    ptr->total_angle = ((int32_t)ptr->round_cnt * (int32_t)M3508_ENCODER_COUNTS_PER_REV) +
                       (int32_t)ptr->ecd -
                       (int32_t)ptr->offset_ecd;
    ptr->msg_cnt++;
}

HAL_StatusTypeDef set_motor_current_can1(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    iq1 = M3508_LimitCurrentRaw((float)iq1);
    iq2 = M3508_LimitCurrentRaw((float)iq2);
    iq3 = M3508_LimitCurrentRaw((float)iq3);
    iq4 = M3508_LimitCurrentRaw((float)iq4);

    return M3508_SendCurrent(&hfdcan1, iq1, iq2, iq3, iq4);
}

HAL_StatusTypeDef set_motor_current_can2(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    iq1 = M3508_LimitCurrentRaw((float)iq1);
    iq2 = M3508_LimitCurrentRaw((float)iq2);
    iq3 = M3508_LimitCurrentRaw((float)iq3);
    iq4 = M3508_LimitCurrentRaw((float)iq4);

    return M3508_SendCurrent(&hfdcan2, iq1, iq2, iq3, iq4);
}

float M3508_CurrentRawToAmpere(int16_t current_raw)
{
    return ((float)current_raw / M3508_CURRENT_COMMAND_RAW_LIMIT) * M3508_CURRENT_LIMIT_A;
}

int16_t M3508_CurrentAmpereToRaw(float current_a)
{
    float raw = (current_a / M3508_CURRENT_LIMIT_A) * M3508_CURRENT_COMMAND_RAW_LIMIT;
    return M3508_LimitCurrentRaw(raw);
}

float M3508_CurrentToTorqueNm(float current_a)
{
    return current_a * M3508_TORQUE_CONSTANT_NM_PER_A;
}

float M3508_TorqueNmToCurrent(float torque_nm)
{
    return torque_nm / M3508_TORQUE_CONSTANT_NM_PER_A;
}

float M3508_RpmToRadPerSec(float speed_rpm)
{
    return speed_rpm * 2.0f * M3508_PI / 60.0f;
}

float M3508_EncoderCountToRad(int32_t encoder_count)
{
    return ((float)encoder_count / M3508_ENCODER_COUNTS_PER_REV) * 2.0f * M3508_PI;
}

void PID_M3508_CAN_Init(void)
{
    memset(motor_3508_can, 0, sizeof(motor_3508_can));
    memset(speed_pid, 0, sizeof(speed_pid));

    for (int i = 0; i < MOTOR_3508_number; i++)
    {
        Motor_3508_Instance_Init(&motor_3508_instance[i], i);
        PID_Init(&speed_pid[i],
                 M3508_SPEED_PID_KP,
                 M3508_SPEED_PID_KI,
                 M3508_SPEED_PID_KD,
                 M3508_SPEED_PID_MAX_OUT,
                 -M3508_SPEED_PID_MAX_OUT,
                 M3508_SPEED_PID_MAX_SUM * M3508_PID_DT_S,
                 -M3508_SPEED_PID_MAX_SUM * M3508_PID_DT_S);
    }

    my_can_filter_init_recv_all();
}

void PID_M3508_CAN1(int16_t target_rpm1, int16_t target_rpm2, int16_t target_rpm3, int16_t target_rpm4)
{
    int16_t iq1 = M3508_SpeedPidCalc(0, target_rpm1);
    int16_t iq2 = M3508_SpeedPidCalc(1, target_rpm2);

    (void)target_rpm3;
    (void)target_rpm4;

    /* 当前工程 CAN1 只接 1/2 号 3508，后两个电流槽清零，避免误发。 */
    (void)set_motor_current_can1(iq1, iq2, 0, 0);
}

void PID_M3508_CAN2(int16_t target_rpm1, int16_t target_rpm2, int16_t target_rpm3, int16_t target_rpm4)
{
    int16_t iq3 = M3508_SpeedPidCalc(2, target_rpm3);
    int16_t iq4 = M3508_SpeedPidCalc(3, target_rpm4);

    (void)target_rpm1;
    (void)target_rpm2;

    /* CAN2 上的两个 C620 使用 DJI ID 3/4，所以必须放在 0x200 帧的 4~7 字节。 */
    (void)set_motor_current_can2(0, 0, iq3, iq4);
}

void Motor_3508_Instance_Init(Motor_3508_Instance *inst, int number)
{
    if (inst == NULL)
    {
        return;
    }

    if (number < 0)
    {
        number = 0;
    }
    else if (number >= MOTOR_3508_number)
    {
        number = MOTOR_3508_number - 1;
    }

    memset(inst, 0, sizeof(*inst));
    inst->number = number;
    inst->motor_id = (uint8_t)number;
}

void Motor_3508_Instance_Update(Motor_3508_Instance *inst, int16_t now_current_raw, int16_t W, int32_t Pos, uint8_t Temp)
{
    if (inst == NULL)
    {
        return;
    }

    inst->now_current_raw = now_current_raw;
    inst->now_W = W;
    inst->now_Pos = Pos;
    inst->now_temp = Temp;

    inst->history_current_raw[inst->write_index] = (float)now_current_raw;
    inst->history_W[inst->write_index] = (float)W;
    inst->history_Pos[inst->write_index] = (float)Pos;
    inst->history_Temp[inst->write_index] = (float)Temp;

    inst->write_index = (uint8_t)((inst->write_index + 1U) % MOTOR_HISTORY);
    if (inst->count < MOTOR_HISTORY)
    {
        inst->count++;
    }

    if (inst->motor_id < MOTOR_3508_number)
    {
        inst->round_cnt = motor_3508_can[inst->motor_id].round_cnt;
    }
    inst->numb_updates++;
}
