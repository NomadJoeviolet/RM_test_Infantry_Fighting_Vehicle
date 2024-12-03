/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DR16.h"
#include "string.h"
#include "motor.h"
#include "pid.h"
#include "motor_message.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define rxBufferLen 18 
#define __HAL_DMA_SET_COUNTER(__HANDLE__, __COUNTER__) ((__HANDLE__)->Instance->CNDTR = (uint16_t)(__COUNTER__))

#define CAN_3508_M1_ID 0x201
#define CAN_3508_M2_ID 0x202
#define CAN_3508_M3_ID 0x203
#define CAN_3508_M4_ID 0x204
#define CAN_6020_M1_ID 0x205
#define CAN_6020_M2_ID 0x206


//#define __HAL_DMA_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNDTR) 

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


char DR16Buffer[rxBufferLen] ;
Controller controller ;//DT7传过来的控制信息(all)
motor_tasks Motor_Tasks ;

//motor_tasks Motor_Tasks ;//所有电机的控制

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void UART_InitDMAReceive(void);//空闲中断初始化

//DMA空闲中断回调函数
void UART_DMAIdleCallback( UART_HandleTypeDef *huart ) {
	if( huart == &huart3 ) {
		__HAL_DMA_DISABLE(huart->hdmarx) ;//停止DMA接收
		int dataLen = rxBufferLen-__HAL_DMA_GET_COUNTER(huart->hdmarx);
		
		DR16_command_process(&controller,(uint8_t*)DR16Buffer);//获取数据
		
		
		memset(DR16Buffer,0,sizeof(DR16Buffer));
		
		gimbal_control(&controller,&Motor_Tasks);
		chasis_moving_encoder( &Motor_Tasks , &controller );
		
		
		
		//重启DMA接收
		__HAL_DMA_SET_COUNTER(huart->hdmarx , rxBufferLen );
		__HAL_DMA_ENABLE(huart->hdmarx) ;//要先重设DMA计数器，再使能DMA
		
	}		
}

void HAL_FDCAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hfdcan)  //FDCAN新消息中断
{  
    FDCAN_RxHeaderTypeDef rxHeader;
	
	
    // 从 FIFO0 读取消息  
	if(hfdcan == &hfdcan1 ) {
		
			uint8_t rxData[8]; // 根据数据长度调整  
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)  
			{  
					// 处理接收到的数据  
					// 例如，可以根据 rxHeader.ID 进行消息分类  
				switch (rxHeader.Identifier ) {
					case CAN_3508_M1_ID :
					{
						fdcan_message_get(&Motor_Tasks.chasis_motor_1,rxData );
						break;
					}
					case CAN_3508_M2_ID :
					{
						fdcan_message_get(&Motor_Tasks.chasis_motor_2,rxData );
						break;
					}
					case CAN_3508_M3_ID :
					{
						fdcan_message_get(&Motor_Tasks.chasis_motor_3,rxData );
						break;
					}
					case CAN_3508_M4_ID :
					{
						fdcan_message_get(&Motor_Tasks.chasis_motor_4,rxData );
						break;
					}
					case CAN_6020_M1_ID :
					{
						fdcan_message_get(&Motor_Tasks.gimbal_yaw_motor,rxData );
						break;
					}
					case CAN_6020_M2_ID :
					{
						fdcan_message_get(&Motor_Tasks.gimbal_pitch_motor,rxData );
						break;
					}
					default:
					{
						break ;
					}
				}
			}
		}  
	
		
}  



void Init_Motors() {
	
	Clear_Cascade_PID( &Motor_Tasks.gimbal_yaw_pid) ;//PID清理
	Init_Cascade_PID( &Motor_Tasks.gimbal_yaw_pid );//PID初始化
	Motor_Tasks.gimbal_yaw_motor.ex_angle = 0 ;
	Motor_Tasks.gimbal_yaw_motor.ex_speed = 0 ;
	
	Clear_Cascade_PID( &Motor_Tasks.gimbal_pitch_pid) ;//PID清理
	Init_Cascade_PID( &Motor_Tasks.gimbal_pitch_pid );//PID初始化
	Motor_Tasks.gimbal_pitch_motor.ex_angle = 0 ;
	Motor_Tasks.gimbal_pitch_motor.ex_speed = 0 ;
	
	//底盘PID的初始化
	Init_chasis_PID(&Motor_Tasks.chasis_pid_1);
	Init_chasis_PID(&Motor_Tasks.chasis_pid_2);
	Init_chasis_PID(&Motor_Tasks.chasis_pid_3);
	Init_chasis_PID(&Motor_Tasks.chasis_pid_4);
	
}

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
  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  /* USER CODE BEGIN 2 */
	UART_InitDMAReceive();
	Init_Motors();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Motor_PID_update(&Motor_Tasks);
		chasis_pid_excute(&Motor_Tasks);
		chasis_motor_message_send(&hfdcan1,&Motor_Tasks);
		HAL_Delay(1);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 21;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void UART_InitDMAReceive(void ) {
	__HAL_UART_CLEAR_IDLEFLAG(&huart3) ;
	__HAL_UART_ENABLE_IT( &huart3 , UART_IT_IDLE );
	
	HAL_UART_Receive_DMA( &huart3 , (uint8_t*)DR16Buffer , rxBufferLen );
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
