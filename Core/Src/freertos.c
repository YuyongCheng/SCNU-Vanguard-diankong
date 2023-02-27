/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "can.h"
#include "pid.h"
#include "kalman.h"
#include "tim.h"
#include "spi.h"
#include "IMU.h"
#include "Chassis.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern uint8_t Message[];
uint8_t uart_rxbuff;//仅供测试
float speed_target_arr[] = {0,30,-15,15};
float angle_target_arr[] = {0,45,30,90,180,150,180};   //测试�???
int spcount = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SendMessage */
osThreadId_t SendMessageHandle;
const osThreadAttr_t SendMessage_attributes = {
  .name = "SendMessage",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ReceiveMessage */
osThreadId_t ReceiveMessageHandle;
const osThreadAttr_t ReceiveMessage_attributes = {
  .name = "ReceiveMessage",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ChangeTarget */
osThreadId_t ChangeTargetHandle;
const osThreadAttr_t ChangeTarget_attributes = {
  .name = "ChangeTarget",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PWM_Test */
osThreadId_t PWM_TestHandle;
const osThreadAttr_t PWM_Test_attributes = {
  .name = "PWM_Test",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal5,
};
/* Definitions for IMU_Read */
osThreadId_t IMU_ReadHandle;
const osThreadAttr_t IMU_Read_attributes = {
  .name = "IMU_Read",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void start_SendMessage(void *argument);
void startReceiveMessage(void *argument);
void fun_ChangeTarget(void *argument);
void Start_PWM_Test(void *argument);
void StartIMU_Read(void *argument);
void StartChassis(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SendMessage */
  SendMessageHandle = osThreadNew(start_SendMessage, NULL, &SendMessage_attributes);

  /* creation of ReceiveMessage */
  ReceiveMessageHandle = osThreadNew(startReceiveMessage, NULL, &ReceiveMessage_attributes);

  /* creation of ChangeTarget */
  ChangeTargetHandle = osThreadNew(fun_ChangeTarget, NULL, &ChangeTarget_attributes);

  /* creation of PWM_Test */
  PWM_TestHandle = osThreadNew(Start_PWM_Test, NULL, &PWM_Test_attributes);

  /* creation of IMU_Read */
  IMU_ReadHandle = osThreadNew(StartIMU_Read, NULL, &IMU_Read_attributes);

  /* creation of Chassis */
  ChassisHandle = osThreadNew(StartChassis, NULL, &Chassis_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_start_SendMessage */
/**
* @brief Function implementing the SendMessage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_SendMessage */
void start_SendMessage(void *argument)
{
  /* USER CODE BEGIN start_SendMessage */
	static int16_t temp_yaw;
	static int16_t temp_pitch;
	static int16_t temp_ammofeed;
  /* Infinite loop */
  for(;;)
  {

	  PID_Origin(&PID_Motor_Angle[Motor_Pitch_ID], Motor[Motor_Pitch_ID].angle, Motor[Motor_Pitch_ID].target_angle);
	  PID_Origin(&PID_Motor_Angle[Motor_Yaw_ID], Motor[Motor_Yaw_ID].angle, Motor[Motor_Yaw_ID].target_angle);

	  PID_Incr(&PID_Motor_Speed[Motor_AmmoFeed_ID],Motor[Motor_AmmoFeed_ID].speed,Motor[Motor_AmmoFeed_ID].target_speed);
	  PID_Incr(&PID_Motor_Speed[Motor_Yaw_ID], Motor[Motor_Yaw_ID].speed, PID_Motor_Angle[Motor_Yaw_ID].Output_float);
   	  PID_Incr(&PID_Motor_Speed[Motor_Pitch_ID],Motor[Motor_Pitch_ID].speed,PID_Motor_Angle[Motor_Pitch_ID].Output_float);

   	  temp_yaw += PID_Motor_Speed[Motor_Yaw_ID].Output;
   	  temp_pitch += PID_Motor_Speed[Motor_Pitch_ID].Output;
   	  temp_ammofeed += PID_Motor_Speed[Motor_AmmoFeed_ID].Output;

   	  Can_TxData[0] = (temp_pitch>>8);
   	  Can_TxData[1] = temp_pitch;

   	  Can_TxData[2] = (temp_yaw>>8);
   	  Can_TxData[3] = temp_yaw;

   	  Can_TxData[4] = (temp_ammofeed>>8);
   	  Can_TxData[5] = temp_ammofeed;


   	  HAL_CAN_AddTxMessage(&hcan1, &Can_cmdHeader[Motor_Pitch_ID], Can_TxData, (uint32_t*)CAN_TX_MAILBOX0);
   	  //HAL_CAN_AddTxMessage(&hcan1, &Can_cmdHeader[Motor_Yaw_ID], Can_TxData, (uint32_t*)CAN_TX_MAILBOX0);
   	  for(int i=0;i<4;i++)
   	  {
   		  Can_TxData[2*i] = Chassis_ctrl[i]>>8;
   		  Can_TxData[2*1+1] = Chassis_ctrl[i];
   	  }
   	  while(HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_READY);
   	  HAL_CAN_AddTxMessage(&hcan1,&Can_cmdHeader[Motor_LeftFront_ID],Can_TxData,(uint32_t*)CAN_TX_MAILBOX0);

    osDelay(5);
  }
  /* USER CODE END start_SendMessage */
}

/* USER CODE BEGIN Header_startReceiveMessage */
/**
* @brief Function implementing the ReceiveMessage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startReceiveMessage */
void startReceiveMessage(void *argument)
{
  /* USER CODE BEGIN startReceiveMessage */
	int data_size = 0;
  /* Infinite loop */
  for(;;)
  {
	if(HAL_UART_GetState(&huart1) ==  HAL_UART_STATE_READY)
	{

		Message[0] = Motor[Motor_Pitch_ID].angle;
		Message[1] = Motor[Motor_Yaw_ID].angle;
		Message[2] = uart_rxbuff;
		data_size = 3;

//		Message[0] = PID_Motor_Speed[1].Output>>8;
//		Message[1] = PID_Motor_Speed[1].Output;
//		Message[2] = PID_Motor_Speed[1].Val_now;
//		Message[3] = PID_Motor_Speed[1].Target_now;
//		Message[4] = (uint16_t)KalmanFilter(&Klm_Motor[0], PID_Motor_Speed[0].Val_now);
//		data_size = 4;

//		Message[0] = BMI088_Gyro.x_MSB_Data;
//		Message[1] = BMI088_Gyro.x_LSB_Data;
//		Message[2] = BMI088_Gyro.y_MSB_Data;
//		Message[3] = BMI088_Gyro.y_LSB_Data;
//		Message[4] = BMI088_Gyro.z_MSB_Data;
//		Message[5] = BMI088_Gyro.z_LSB_Data;
//		data_size = 6;

		HAL_UART_Transmit_DMA(&huart1, Message, data_size);
	}

    osDelay(50);
  }
  /* USER CODE END startReceiveMessage */
}

/* USER CODE BEGIN Header_fun_ChangeTarget */
/**
* @brief Function implementing the ChangeTarget thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fun_ChangeTarget */
void fun_ChangeTarget(void *argument)
{
  /* USER CODE BEGIN fun_ChangeTarget */

  /* Infinite loop */
	for(;;)
  {
//	if(++spcount > 3)
//	{
//		spcount = 0;
//	}
//	PID_Motor_Speed[0].Target_now = speed_target_arr[spcount];
//	if(speed_target_arr[spcount] < 0 )
//	{
//		PID_Motor_Speed[0].Target_now--;
//	}

//	  if(++spcount > 6)
//	{
//		spcount = 0;
//	}
//	PID_Motor_Angle[0].Target_now = angle_target_arr[spcount];
//	if(angle_target_arr[spcount] < 0 )
//	{
//		PID_Motor_Angle[0].Target_now--;
//	}
//		osDelay(2000);
//	  PID_Motor_Speed[2].Target_now = 900;

  }


  /* USER CODE END fun_ChangeTarget */
}

/* USER CODE BEGIN Header_Start_PWM_Test */
/**
* @brief Function implementing the PWM_Test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_PWM_Test */
void Start_PWM_Test(void *argument)
{
  /* USER CODE BEGIN Start_PWM_Test */
	int pwm_count = 500;
  /* Infinite loop */
  for(;;)
  {
	  for(;pwm_count < 1000; pwm_count++)
	  {
		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm_count);
		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, pwm_count);
		  osDelay(2);
	  }
	  for(;pwm_count > 0; pwm_count--)
	  {
	      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm_count);
	  	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, pwm_count);
	  	  osDelay(2);
	  }
    osDelay(5);
  }
  /* USER CODE END Start_PWM_Test */
}

/* USER CODE BEGIN Header_StartIMU_Read */
/**
* @brief Function implementing the IMU_Read thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMU_Read */
void StartIMU_Read(void *argument)
{
  /* USER CODE BEGIN StartIMU_Read */
  /* Infinite loop */
  for(;;)
  {
	 BMI088_read_Gyro(&BMI088_Gyro);
    osDelay(5);
  }
  /* USER CODE END StartIMU_Read */
}

/* USER CODE BEGIN Header_StartChassis */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassis */
void StartChassis(void *argument)
{
  /* USER CODE BEGIN StartChassis */

  /* Infinite loop */
  for(;;)
  {
				/*
				//获取上位机指令中传给底盘的部分，即x、y轴线速度
				Chassis_getCommend();
				*/
				//获取底盘与云台夹角并转换目标速度
				Chassis_angleTransform();

				//底盘特殊状态
				//Follow_flag = true;
				//Swing_flag = true;
	//			if(Follow_flag == true)
	//			{
	//				//底盘跟随
	//				Chassis_Follow();
	//			}
	//			else if(Swing_flag == true)
	//			{
	//				//底盘摇摆
	//				Chassis_Swing();
	//			}
				//底盘移动
				//speed_x_mps = 1.0f;
				Chassis_Move();
    osDelay(1);
  }
  /* USER CODE END StartChassis */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

