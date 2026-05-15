/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
 * M3 重构说明：旧 main.c 的"业务初始化 + while(1) 死程序步态"已迁移到 App/。
 *   - 想完全恢复旧行为：编译时定义 USE_LEGACY_MAIN=1
 *   - 默认 USE_LEGACY_MAIN=0：旧 PID/GO/M3508 代码不参与启动，避免与
 *     App 层 RTOS 任务竞写电机；启动后由 freertos.c 调用 app_init() + app_tasks_create()，
 *     脱机时跑 SCRIPT_BUILTIN_STAND_HOLD（详见 App/app/task_chassis.c）。
 *   - 想跑旧"抬腿循环"等死程序步态：用 task_chassis_play_script(&SCRIPT_BUILTIN_WAVE_UP_DOWN, 0.3f)，
 *     不要再回头改 main.c。
 */
#ifndef USE_LEGACY_MAIN
#define USE_LEGACY_MAIN 0
#endif

#if USE_LEGACY_MAIN
#include "GO-motor.h"
#include "gait_plan.h"
#include <string.h>
#include "3508_motor.h"
#else
#include "app_init.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#if USE_LEGACY_MAIN
//8010电机所需结构体
static MotorBack Motor_r;
static MotorInstance motor_instance[MOTOR_NUM];//4路485线，我就记作每组3个电机，从&huart1到&huart4电机依次从【0】到【11】，id与huart的对应办法是（id+3）/3就行
int idx;//转存MotorInstance motor_instance[MOTOR_NUM]专用的整数变量
MotorCmd GO1;//结构体变量，一个用于中转的宇树8010命令的结构体

//M3508电机所需结构体
extern Motor_3508_T Motors[4];

//步态参数
gait_action Gait;

//用于描述腿的机械参数的结构体
leg_size leg;

//人工打点
vector up_down[9];

//电机上电时角度
float GO_begin[12];
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if USE_LEGACY_MAIN
//发送完毕的回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	// 启动DMA接收
    HAL_UART_Receive_DMA(huart, (uint8_t*)&Motor_r.motor_back_data, 16);
}

//接收完毕的回调函数。准备写一个将接收的数据进行归类排布的函数，转存完就清空全局缓存区，11.30号，暂无此函数。12.3号，转存函数已经是写完了，就看测试效果了；效果测试不错。
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	extract_data(&Motor_r);
	idx = ((int)Motor_r.motor_id);
	if (Motor_r.correct && idx >= 0 && idx < MOTOR_NUM) {
		MotorInstance_Update(&motor_instance[idx], Motor_r.T, Motor_r.W, Motor_r.Pos);
	}

	memset(&Motor_r, 0, sizeof(MotorBack));
}

//3508电机的can收发回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
        // 必须使用 while 循环将 FIFO 彻底读空，防止多电机并发导致丢帧
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

                // FDCAN1：接收 3508 的电机 1、2
                if (hfdcan->Instance == FDCAN1) {
                    D3508_Decode(RxData, (uint16_t)RxHeader.Identifier);
                }
                // FDCAN2：接收 3508 的电机 3、4
                else if (hfdcan->Instance == FDCAN2) {
                    D3508_Decode(RxData, (uint16_t)RxHeader.Identifier);
                }
            }
        }
    }
}

void Init_up_down(vector *so){
	so[0].data[0] = 0;
	so[0].data[2] = 0;

	so[1].data[0] = 0;
	so[1].data[2] = 0.01;

	so[2].data[0] = 0;
	so[2].data[2] = 0.03;

	so[3].data[0] = 0;
	so[3].data[2] = 0.04;

	so[4].data[0] = 0;
	so[4].data[2] = 0.05;

	so[5].data[0] = 0;
	so[5].data[2] = 0.04;

	so[6].data[0] = 0;
	so[6].data[2] = 0.03;

	so[7].data[0] = 0;
	so[7].data[2] = 0.01;

	so[8].data[0] = 0;
	so[8].data[2] = 0;
}
#endif /* USE_LEGACY_MAIN */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
#if USE_LEGACY_MAIN
//  PID_M3508_CAN_Init();
  //初始化宇树电机的回传数据，现在是只开了usart2，自己用的时候要用其他串口记得自己进函数里开；
  for(int i = 0; i < 12; i ++){
	  MotorInstance_Init(&motor_instance[i], i);
  }

  //初始化M3508电机数据
  FDCAN1_Filter_Init();
  FDCAN2_Filter_Init();
  D3508_Init();

  //启动定时器中断
  HAL_TIM_Base_Start_IT(&htim2);
  Init_up_down(up_down);

  //初始化狗腿参数,测试时必定修改
  params_init(&leg, 0.1f, 0.15f, 0.04f,
  -PI, 0.0f, -0.75 * PI, 0.0f, 0.095f,
  0.042f, 0.012f,0.09f, 0.008f, 0.55f,
  0.051f, 0.05f, 0.056f, 0.01f, 9.8f);
//
//  //初始化步态数据
//  gait_action_init( &Gait, 0.035, 0.5, 2, 20);
//
//  //计算步态点位
//  compete_gait( &Gait);
  //计算必要角度
  vector ang_ori[9];
  vector ang_mir[9];
  for (int i = 0; i < 9; i++)
  {
      for (int j = 0; j < 3; j++)
      {
          ang_ori[i].data[j] = 0.0f;
      }
      for (int j = 0; j < 3; j++)
      {
          ang_mir[i].data[j] = 0.0f;
      }
  }

  for(int i = 0; i < 9; i++){
	  inverse_kinematics_position(&leg, &up_down[i], &ang_ori[i], -0.20, LEG_TYPE_ORIGINAL);
  }
  for(int i = 0; i < 9; i++){
	  inverse_kinematics_position(&leg, &up_down[i], &ang_mir[i], -0.20, LEG_TYPE_MIRROR);
  }
  HAL_Delay(3000);

  int t = 0;

  for(int n = 0; n < 12; n++){
	  MotorController_SetCommand(&GO1, n, 1, 0, 0, 0, 0, 0);
	  MotorController_SendCommand(&huart3, &GO1);
	  HAL_Delay(1);
	  GO_begin[n] = motor_instance[n].now_Pos;
  }
  for(int n = 0; n < 12; n++){
	  MotorController_SetCommand(&GO1, n, 1, 0, 0, 0, 0, 0);
	  MotorController_SendCommand(&huart2, &GO1);
	  HAL_Delay(1);
	  GO_begin[n] = motor_instance[n].now_Pos;
  }


  //原地站立

  MotorController_SetCommand(&GO1, 0, 1, 0, 0, GO_begin[0] + (ang_mir[0].data[1] * 6.33), 1.5, 0.12);
  MotorController_SendCommand(&huart3, &GO1);

  MotorController_SetCommand(&GO1, 1, 1, 0, 0, GO_begin[1] - ((ang_mir[0].data[2] + (0.75 * Pi)) * 6.33), 2.5, 0.2);
  MotorController_SendCommand(&huart3, &GO1);

  MotorController_SetCommand(&GO1, 6, 1, 0, 0, GO_begin[6] + (ang_mir[0].data[1] * 6.33), 1.5, 0.12);
  MotorController_SendCommand(&huart3, &GO1);

  MotorController_SetCommand(&GO1, 7, 1, 0, 0, GO_begin[7] - ((ang_mir[0].data[2] + (0.75 * Pi)) * 6.33), 2.5, 0.2);
  MotorController_SendCommand(&huart3, &GO1);

  MotorController_SetCommand(&GO1, 3, 1, 0, 0, GO_begin[3] + ((ang_ori[0].data[1] + Pi) * 6.33), 1.5, 0.12);
  MotorController_SendCommand(&huart2, &GO1);

  MotorController_SetCommand(&GO1, 4, 1, 0, 0, GO_begin[4] - ((0.25 * Pi + ang_ori[0].data[2]) * 6.33), 2.5, 0.2);
  MotorController_SendCommand(&huart2, &GO1);

  MotorController_SetCommand(&GO1, 9, 1, 0, 0, GO_begin[9] + ((ang_ori[0].data[1] + Pi) * 6.33), 1.5, 0.12);
  MotorController_SendCommand(&huart2, &GO1);

  MotorController_SetCommand(&GO1, 10, 1, 0, 0, GO_begin[10] - ((0.25 * Pi + ang_ori[0].data[2]) * 6.33), 2.5, 0.2);
  MotorController_SendCommand(&huart2, &GO1);

  HAL_Delay(2000);
#endif /* USE_LEGACY_MAIN */
#if !USE_LEGACY_MAIN
  /*
   * app_init() touches HAL timeouts/delays (for example BMI088 bring-up), so run
   * it before osKernelInitialize() raises BASEPRI and masks the HAL TIM1 tick.
   */
  (void)app_init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#endif /* USE_LEGACY_MAIN */
  while (1)
  {
#if USE_LEGACY_MAIN
	  if(t <= 27){

		  MotorController_SetCommand(&GO1, 0, 1, 0, 0, GO_begin[0] + (ang_mir[t % 7].data[1] * 6.33), 1.5, 0.12);
		  MotorController_SendCommand(&huart3, &GO1);

		  MotorController_SetCommand(&GO1, 1, 1, 0, 0, GO_begin[1] - ((ang_mir[t % 7].data[2] + (0.75 * Pi)) * 6.33), 2.5, 0.2);
		  MotorController_SendCommand(&huart3, &GO1);

		  MotorController_SetCommand(&GO1, 6, 1, 0, 0, GO_begin[6] + (ang_mir[t % 7].data[1] * 6.33), 1.5, 0.12);
		  MotorController_SendCommand(&huart3, &GO1);

		  MotorController_SetCommand(&GO1, 7, 1, 0, 0, GO_begin[7] - ((ang_mir[t % 7].data[2] + (0.75 * Pi)) * 6.33), 2.5, 0.2);
		  MotorController_SendCommand(&huart3, &GO1);

		  HAL_Delay(100);
	  }

	  if(t >= 5 && t <= 32){

		  MotorController_SetCommand(&GO1, 3, 1, 0, 0, GO_begin[3] + ((ang_ori[(t - 5) % 7].data[1] + Pi) * 6.33), 1.5, 0.12);
		  MotorController_SendCommand(&huart2, &GO1);

		  MotorController_SetCommand(&GO1, 4, 1, 0, 0, GO_begin[4] - ((0.25 * Pi + ang_ori[(t - 5) % 7].data[2]) * 6.33), 2.5, 0.2);
		  MotorController_SendCommand(&huart2, &GO1);

		  MotorController_SetCommand(&GO1, 9, 1, 0, 0, GO_begin[9] + ((ang_ori[(t - 5) % 7].data[1] + Pi) * 6.33), 1.5, 0.12);
		  MotorController_SendCommand(&huart2, &GO1);

		  MotorController_SetCommand(&GO1, 10, 1, 0, 0, GO_begin[10] - ((0.25 * Pi + ang_ori[(t - 5) % 7].data[2]) * 6.33), 2.5, 0.2);
		  MotorController_SendCommand(&huart2, &GO1);
//		  HAL_Delay(100);
	  }

	  if(t >= 27 && t <= 32){
		  HAL_Delay(100);
	  }

	  if(t > 500 && t <= 2000){

      PID_Calc_SetSpeed(0, 450);
      PID_Calc_SetSpeed(1, 450);

      HAL_Delay(1);
	  }
	  if(t > 10000){

      PID_Calc_SetSpeed(0, 0);
      PID_Calc_SetSpeed(1, 0);

		  HAL_Delay(1);
	  }
	  t += 1;//简易定时器
#endif /* USE_LEGACY_MAIN */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*
 * HAL 回调桥接：将 FDCAN / UART 中步回调转发给 App 层 BSP。
 * 当 USE_LEGACY_MAIN=0 时，旧代码的回调不再注册，
 * 新架构通过 bsp_fdcan / bsp_uart 接收数据。
 */
#if !USE_LEGACY_MAIN

/* FDCAN RX 回调 → bsp_fdcan */
extern void bsp_fdcan_hal_rxfifo0_cb(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    bsp_fdcan_hal_rxfifo0_cb(hfdcan, RxFifo0ITs);
}

/* UART RX 事件回调 → bsp_uart */
extern void bsp_uart_hal_rx_event(UART_HandleTypeDef *huart, uint16_t size);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    bsp_uart_hal_rx_event(huart, size);
}

/* UART TX 完成回调 → bsp_uart */
extern void bsp_uart_hal_tx_done(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    bsp_uart_hal_tx_done(huart);
}

#endif /* !USE_LEGACY_MAIN */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	// legacy 模式下用 TIM1 稳定发送 3508 电机指令；RTOS 模式只保留 HAL tick。
#if USE_LEGACY_MAIN
	  if (htim->Instance == TIM1)
	  {
		  send_current();
	  }
#endif
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
