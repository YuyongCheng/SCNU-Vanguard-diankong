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
#include "remote.h"
#include "Chassis.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern uint8_t Message[];
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
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
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
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PWM_Test */
osThreadId_t PWM_TestHandle;
const osThreadAttr_t PWM_Test_attributes = {
  .name = "PWM_Test",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal6,
};
/* Definitions for IMU_Read */
osThreadId_t IMU_ReadHandle;
const osThreadAttr_t IMU_Read_attributes = {
  .name = "IMU_Read",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Chassis_task */
osThreadId_t Chassis_taskHandle;
const osThreadAttr_t Chassis_task_attributes = {
  .name = "Chassis_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
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
void Start_Chassis_task(void *argument);

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

  /* creation of Chassis_task */
  Chassis_taskHandle = osThreadNew(Start_Chassis_task, NULL, &Chassis_task_attributes);

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
	static int16_t temp_yaw, temp_pitch, temp_ammofeed;
  /* Infinite loop */
  for(;;)
  {

	  PID_Origin(&PID_Motor_Angle[Motor_Pitch_ID], Motor[Motor_Pitch_ID].angle, Motor[Motor_Pitch_ID].target_angle);
	  PID_Origin(&PID_Motor_Angle[Motor_Yaw_ID], Motor[Motor_Yaw_ID].angle, Motor[Motor_Yaw_ID].target_angle);

	  PID_Incr(&PID_Motor_Speed[Motor_AmmoFeed_ID],Motor[Motor_AmmoFeed_ID].speed,Motor[Motor_AmmoFeed_ID].target_speed);
      PID_Incr(&PID_Motor_Speed[Motor_Yaw_ID], Motor[Motor_Yaw_ID].speed, PID_Motor_Angle[Motor_Yaw_ID].Output);
	  PID_Incr(&PID_Motor_Speed[Motor_Pitch_ID],Motor[Motor_Pitch_ID].speed,PID_Motor_Angle[Motor_Pitch_ID].Output);

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
	  //osDelay(1);
	  Chassis_Move();
	  Can_TxData[0] = Chassis_ctrl[0]>>8;
	  Can_TxData[1] = Chassis_ctrl[0];
 	  Can_TxData[2] = Chassis_ctrl[1]>>8;
 	  Can_TxData[3] = Chassis_ctrl[1];
 	  Can_TxData[4] = Chassis_ctrl[2]>>8;
 	  Can_TxData[5] = Chassis_ctrl[2];
 	  Can_TxData[6] = Chassis_ctrl[3]>>8;
 	  Can_TxData[7] = Chassis_ctrl[3];

	  HAL_CAN_AddTxMessage(&hcan1,&Can_cmdHeader[Motor_LeftFront_ID],Can_TxData,(uint32_t*)CAN_TX_MAILBOX0);
	  osDelay(1);
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
	uint8_t ammo_count=0, ammo_temp = 0;
	GPIO_PinState pinstate = GPIO_PIN_SET;
  /* Infinite loop */
  for(;;)
  {

//		Message[0] = BMI088_Gyro.x_MSB_Data;
//		Message[1] = BMI088_Gyro.x_LSB_Data;
//		Message[2] = BMI088_Gyro.y_MSB_Data;
//		Message[3] = BMI088_Gyro.y_LSB_Data;
//		Message[4] = BMI088_Gyro.z_MSB_Data;
//		Message[5] = BMI088_Gyro.z_LSB_Data;
//		data_size = 6;

	  if(HAL_GPIO_ReadPin(Ammo_Counter_GPIO_Port, Ammo_Counter_Pin) != pinstate)
	  {
		  pinstate = !pinstate;
		  ammo_temp++;
	  }
	  if(ammo_temp >=2)
	  {
		  ammo_count++;
		  ammo_temp = 0;
	  }
	  HAL_UART_Transmit(&huart1, &ammo_count, 1, 100);

    osDelay(5);
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
//		Motor[6].target_angle = RC_Ctl.rc.ch3*2 + 6144;
//		if(Motor[6].target_angle >= 8192) Motor[6].target_angle -=8192;
//		if(Motor[6].target_angle <= 0) Motor[6].target_angle += 8192;
//		Motor[5].target_angle = (RC_Ctl.rc.ch4 >> 1) + 3300;
		//omega = -RC_Ctl.rc.ch3/600*0.8;
		speed_x = RC_Ctl.rc.ch2/600*0.3;
		speed_y = RC_Ctl.rc.ch1/600*0.3;
		Motor[Motor_Yaw_ID].target_angle -= (RC_Ctl.rc.ch3>>5);
		Motor[Motor_Pitch_ID].target_angle += (RC_Ctl.rc.ch4>>5);
		if(Motor[6].target_angle >= 8192) Motor[6].target_angle -=8192;
		if(Motor[6].target_angle <= 0) Motor[6].target_angle += 8192;
		if(Motor[Motor_Pitch_ID].target_angle > 3700)
			Motor[Motor_Pitch_ID].target_angle = 3700;
		if(Motor[Motor_Pitch_ID].target_angle < 2900)
			Motor[Motor_Pitch_ID].target_angle = 2900;
		if(RC_Ctl.rc.sw1 == 2)
		{
			Motor[Motor_AmmoFeed_ID].target_speed = 1200;
		}
		else
		{
			Motor[Motor_AmmoFeed_ID].target_speed = 0;
		}

     osDelay(5);
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

/* USER CODE BEGIN Header_Start_Chassis_task */
/**
* @brief Function implementing the Chassis_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Chassis_task */
void Start_Chassis_task(void *argument)
{
  /* USER CODE BEGIN Start_Chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(5);
  }
  /* USER CODE END Start_Chassis_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

