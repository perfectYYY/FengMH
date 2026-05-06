#include "3508_motor.h"
#include <string.h>
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1; // STM32H7
extern FDCAN_HandleTypeDef hfdcan2; // STM32H7

// 全局变量定义
Motor_3508_T Motors[4];

// PID 参数初始化
PID_3508_T Speed_PID = {
    .Kp = 3.0f,
    .Ki = 0.3f,
    .Kd = 0.0f,
    .Max_Out = 16000.0f,
    .Max_Sum = 10000.0f
};

PID_3508_T Position_PID = {
    .Kp = 1.0f,    // 需调试：越大响应越快，太大会震荡
    .Ki = 0.03f,
    .Kd = 0.15f,    // 需调试：阻尼项，防止超调
    .Max_Out = 6000.0f, // 最大转速限制 (RPM)
    .Max_Sum = 0.0f
};

// 力矩环（电流环）PID参数
PID_3508_T Torque_PID = {
    .Kp = 0.5f,    // 需调试：由于C620自带底层电流环，这里的Kp主要起微调和补偿作用，不需要太大
    .Ki = 0.01f,
    .Kd = 0.0f,
    .Max_Out = 16000.0f,
    .Max_Sum = 8000.0f
};

//功率限制参数
#define POWER_LIMIT 162.0f   // 限制最大功率 162W
#define Kt   0.0156f    // 转矩常数 (Nm/A) 加了减速比
#define Int_Current   819.2f  // 16384/20 = 819.2 (Value/A)  C620电调: 16384 = 20A

// EMA 滤波系数
#define D3508_EMA_ALPHA 0.3f
static const float Ts = 0.005f;

int16_t Power_Limit(float desire_current, float rpm);

// 1.CAN配置
void FDCAN1_Filter_Init(void) {

    // STM32H7 Code (FDCAN)
    FDCAN_FilterTypeDef sFilterConfig;

    // 配置全局过滤器
    // 参数：句柄, 未匹配标准帧处理, 未匹配扩展帧处理, 拒绝远程标准帧, 拒绝远程扩展帧
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    // 也可以配置一个具体的过滤器 (虽然有全局配置其实可以不用)
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x200;//基准ID
    sFilterConfig.FilterID2 = 0x7F0;//掩码

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) Error_Handler();

    // 开启“新消息”中断
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) Error_Handler();
}


void FDCAN2_Filter_Init(void) {

    // STM32H7 Code (FDCAN)
    FDCAN_FilterTypeDef sFilterConfig;

    // 配置全局过滤器
    // 参数：句柄, 未匹配标准帧处理, 未匹配扩展帧处理, 拒绝远程标准帧, 拒绝远程扩展帧
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    // 也可以配置一个具体的过滤器 (虽然有全局配置其实可以不用)
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x200;//基准ID
    sFilterConfig.FilterID2 = 0x7F0;//掩码

    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) Error_Handler();

    // 开启“新消息”中断
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) Error_Handler();
}

// 2.初始化电机数据
void D3508_Init(void) {
    for(int i=0; i<4; i++) {
        Motors[i].init_flag = 0;
        Motors[i].last_angle = 0;
        Motors[i].total_round = 0;
        Motors[i].total_angle = 0;
        Motors[i].target_speed = 0;
        Motors[i].target_angle = 0;
        Motors[i].Out_Current = 0;
        Motors[i].filter_speed = 0.0f;

        Motors[i].speed_err_sum = 0.0f;
        Motors[i].position_err_sum = 0.0f;
        Motors[i].torque_err_sum = 0.0f;
    }
}

// 3. 发送电流
void send_current(void) {
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData1[8] = {0}; // 给 FDCAN1 (放电机1、2)
    uint8_t TxData2[8] = {0}; // 给 FDCAN2 (放电机3、4)

    TxHeader.Identifier = 0x200;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; // 必须是 8 字节
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // FDCAN1 只发电机 1 和 2 (大疆电调ID 1,2 读取 0~3 字节)
    TxData1[0] = Motors[0].Out_Current >> 8;
    TxData1[1] = Motors[0].Out_Current;
    TxData1[2] = Motors[1].Out_Current >> 8;
    TxData1[3] = Motors[1].Out_Current;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData1);

    // FDCAN2 只发电机 3 和 4
    // 【注意】大疆电调 ID 3,4 固定读取 4~7 字节，不能把它们放在 0~3 字节！
    TxData2[4] = Motors[2].Out_Current >> 8;
    TxData2[5] = Motors[2].Out_Current;
    TxData2[6] = Motors[3].Out_Current >> 8;
    TxData2[7] = Motors[3].Out_Current;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData2);
}

// 4.速度环PID 计算
void PID_Calc_Speed(int i)
{
	if (i < 0 || i >= 4) return;
    Motor_3508_T* m = &Motors[i];

    // 计算误差
    float error = m->target_speed - m->cur_speed;

    // 积分累加
    m->speed_err_sum += error;
    m->err_last = error;
    // 积分限幅
    if(m->speed_err_sum > Speed_PID.Max_Sum) m->speed_err_sum = Speed_PID.Max_Sum;
    if(m->speed_err_sum < -Speed_PID.Max_Sum) m->speed_err_sum = -Speed_PID.Max_Sum;

    // 修改为标准的微分项（误差微分）
    float d_term = Speed_PID.Kd * (error - m->err_last) / Ts;
    float output = (Speed_PID.Kp * error) + (Speed_PID.Ki * m->speed_err_sum * Ts) + d_term;

    // 输出限幅
    if(output > Speed_PID.Max_Out) output = Speed_PID.Max_Out;
    if(output < -Speed_PID.Max_Out) output = -Speed_PID.Max_Out;

    //调用功率限制函数
    m->Out_Current = Power_Limit (output, (float)m->filter_speed);

}

//5.计算控制函数

// 速度环计算函数
// 输入：目标速度 target_speed (单位：RPM)
// 输出：写入 Motors[i].Out_Current，供发送函数使用
void PID_Calc_SetSpeed(int i, float target_speed){
	if (i < 0 || i >= 4) return;
    Motor_3508_T* m = &Motors[i];

    // 目标速度限幅，避免直接写入超出速度环设计范围的值
    if(target_speed > Speed_PID.Max_Out) target_speed = Speed_PID.Max_Out;
    if(target_speed < -Speed_PID.Max_Out) target_speed = -Speed_PID.Max_Out;

    m->target_speed = target_speed;

    // 还没收到第一帧有效回传前，不做速度闭环输出
    if (m->init_flag == 0) {
        m->Out_Current = 0;
        return;
    }

    PID_Calc_Speed(i);
}

// 位置环计算函数
// 输入：目标角度 target_angle (单位：总编码器值，8192 = 1圈)
// 输出：写入 Motors[i].target_speed，供速度环使用
void PID_Calc_Position(int i, float target_angle)
{
	if (i < 0 || i >= 4) return;
	Motor_3508_T* m = &Motors[i];

    //  计算位置误差
    // 注意：用 total_angle (多圈累计值) 而不是 cur_angle (0-8192)
	m->target_angle = (int64_t)target_angle;
    float error = m->target_angle - m->total_angle;

    // 死区：对于减速比大的电机，过小的死区会导致末端因机械空程不断抖动
    if (fabs(error) < 25.0f) {
        m->target_speed = 0.0f;
        m->position_err_sum = 0.0f;
        m->Out_Current = 0;
        return;
    }

    // 积分累加
    m->position_err_sum += error;
    m->err_last = error;
    // 积分限幅
    if(m->position_err_sum > Speed_PID.Max_Sum) m->position_err_sum = Speed_PID.Max_Sum;
    if(m->position_err_sum < -Speed_PID.Max_Sum) m->position_err_sum = -Speed_PID.Max_Sum;
    // 位置式 PD 控制
    // 修正 Typo
    float position_speed =(Position_PID.Kp * error) + (Position_PID.Ki * m->position_err_sum * Ts) - (Position_PID.Kd * m->filter_speed);

    // 限幅 (限制最大转速)
    if(position_speed > Position_PID.Max_Out) position_speed = Position_PID.Max_Out;
    if(position_speed < -Position_PID.Max_Out) position_speed = -Position_PID.Max_Out;

    //将计算出的速度 赋值给 速度环的目标
    m->target_speed = position_speed;

    // 调用速度环计算，算出电流
    PID_Calc_Speed(i);
}

// 力矩环计算函数
// 输入：目标力矩 target_torque (单位：Nm)
// 输出：目标电流发送值(-16384~16384)
void PID_Calc_Torque(int i, float target_torque)
{
    if (i < 0 || i >= 4) return;
    Motor_3508_T* m = &Motors[i];

    // 1. 前馈控制：将目标力矩(Nm)直接转化为期望电流发送值(-16384~16384)
    // 公式：电流(A) = 力矩 / Kt；发送值 = 电流(A) * Int_Current
    float target_current_val = (target_torque / Kt) * Int_Current;

    // 2. 闭环补偿：计算实际电流与期望电流的误差
    float error = target_current_val - m->actual_current;

    // 3. 积分累加与限幅
    m->torque_err_sum += error;
    if(m->torque_err_sum > Torque_PID.Max_Sum) m->torque_err_sum = Torque_PID.Max_Sum;
    if(m->torque_err_sum < -Torque_PID.Max_Sum) m->torque_err_sum = -Torque_PID.Max_Sum;

    // 4. PID 输出 = 前馈为主 + PID补偿为辅
    float output = target_current_val + (Torque_PID.Kp * error) + (Torque_PID.Ki * m->torque_err_sum * Ts);

    // 5. 输出限幅
    if(output > Torque_PID.Max_Out) output = Torque_PID.Max_Out;
    if(output < -Torque_PID.Max_Out) output = -Torque_PID.Max_Out;

    // 6. 调用功率限制，保证即使切入力控模式，超高速旋转时也会受到功率墙保护
    m->Out_Current = Power_Limit(output, m->filter_speed);
}

// 6.功率限制
int16_t Power_Limit(float desire_current, float rpm){

	// 计算角速度
	float w = fabs(rpm) *2 *3.14f/60;

	if (w < 0.5f) w = 0.5f;

	// 计算最大允许转矩
	float max_torque = POWER_LIMIT / w;

	//计算最大电流
	float max_current_A=max_torque/Kt;

	float max_current_Value=max_current_A *Int_Current;

	//限流
	if (max_current_Value > 16000.0f) max_current_Value = 16000.0f;

	//输出判断
    if (desire_current > max_current_Value) {
        return (int16_t)max_current_Value;
    } else if (desire_current < -max_current_Value) {
        return (int16_t)(-max_current_Value);
    } else {
        return (int16_t)desire_current;
    }

}

// 7.拆解接收数据
void D3508_Decode(uint8_t* RxData, uint16_t ID) {
    // 检查 ID 范围
    if (ID < 0x201 || ID > 0x204) return;

    uint8_t idx = ID - 0x201;
    Motor_3508_T* m = &Motors[idx];

    // 保存上一次角度
    m->last_angle = m->cur_angle;

    // 解析新数据
    m->cur_angle = (RxData[0] << 8) | RxData[1];
    m->cur_speed = (int16_t)((RxData[2] << 8) | RxData[3]);
    m->actual_current = (int16_t)((RxData[4] << 8) | RxData[5]);
    m->temperature = RxData[6];


    // 【新增核心修复】首次上电初始化保护，防止上电疯转
    if (m->init_flag == 0) {
        m->last_angle = m->cur_angle;       // 同步历史角度，消除初始 delta
        m->total_round = 0;                 // 圈数从 0 开始
        m->total_angle = m->cur_angle;      // 同步绝对总角度
        m->filter_speed = m->cur_speed;     // 速度滤波器赋初值
        m->init_flag = 1;                   // 标记已完成初始化
        return;                             // 第一帧数据不进行跨圈计算，直接返回
    }

    //EMA滤波速度
    m->filter_speed = (D3508_EMA_ALPHA * m->cur_speed) + ((1.0f - D3508_EMA_ALPHA) * m->filter_speed);

    //多圈解算
    int32_t delta = (int32_t)m->cur_angle - (int32_t)m->last_angle;
    if (delta > 4096) {
        m->total_round--;
    } else if (delta < -4096) {
        m->total_round++;
    }
    m->total_angle = m->total_round * 8192 + m->cur_angle;

}


