/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "kalman.h"
#include "IMU.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t Message[] = "noob\n";
uint8_t RC_buff[18];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void Can_MessageConfig(void);
void Can_Filter1Config(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void PWM_Init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Can_MessageConfig();
  Can_Filter1Config();
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan1);
  HAL_TIM_Base_Start_IT(&htim2);
  PID_Clear(&PID_Motor_Speed[0]);
  PID_Clear(&PID_Motor_Angle[0]);
  PID_Init();
  KalmanFilter_Init(&Klm_Motor[0]);
  HAL_UART_Receive_DMA(&huart3, RC_buff, RC_FRAME_LENGTH);//初始化DMA
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//IDLE 中断使能
//  PWM_Init();
  BMI088_init();
  Motor[Motor_Yaw_ID].target_angle = 60;
  Motor[Motor_Pitch_ID].target_angle = 100;
  Motor[Motor_Yaw_ID].target_speed = 0;
  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
//		HAL_UART_Transmit_DMA(&huart1, Can_RxData, sizeof(Can_RxData));
	}
}
void PWM_Init()
{
	  HAL_TIM_Base_Start(&htim1);
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//
	  HAL_Delay(100);
//void SPEED_INIT(int speed){
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);
	  HAL_Delay(1650);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,500);
	  HAL_Delay(1650);
//}
//void SPEED_SET(int speed){//速度极限为1999，但是禁止如此使用，极易损坏发射管，曾经测试的时候一次发射就打废一对摩擦轮了且造成了钢板损坏，所以速度建议在1001~1600左右。
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,650);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,650);
	  HAL_Delay(500);
//	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
//	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
//	  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//	  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}
/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
