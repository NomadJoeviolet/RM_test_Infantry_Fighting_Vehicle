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
#include "fdcan.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "DR16.h"
#include "string.h"
#include "motor.h"
#include "motor_message.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define rxBufferLen 8 

#define CAN_3508_M1_ID 0x201
#define CAN_3508_M2_ID 0x202
#define CAN_3508_M3_ID 0x203
#define CAN_3508_M4_ID 0x204
#define CAN_6020_M1_ID 0x205
#define CAN_6020_M2_ID 0x206
#define CAN_G473_gimbal 0x001

#define PI 3.14159265358979323846
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char DR16Buffer[rxBufferLen] ;
Controller controller ;//DT7传过来的控制信息(all)
motor_tasks Motor_Tasks ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Init_can_filter() {
	
	FDCAN_FilterTypeDef FDCAN1_RXFilter;
	FDCAN1_RXFilter.IdType=FDCAN_STANDARD_ID;
	FDCAN1_RXFilter.FilterIndex=0;
	FDCAN1_RXFilter.FilterType=FDCAN_FILTER_MASK;
	FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
	FDCAN1_RXFilter.FilterID1=0x0000;     
  FDCAN1_RXFilter.FilterID2=0x0000; 
	HAL_FDCAN_ConfigFilter(&hfdcan1,&FDCAN1_RXFilter);
	HAL_FDCAN_Start(&hfdcan1);       
  HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);

	FDCAN_FilterTypeDef FDCAN2_RXFilter;
	FDCAN2_RXFilter.IdType=FDCAN_STANDARD_ID;
	FDCAN2_RXFilter.FilterIndex=0;
	FDCAN2_RXFilter.FilterType=FDCAN_FILTER_MASK;
	FDCAN2_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
	FDCAN2_RXFilter.FilterID1=0x0000;     
  FDCAN2_RXFilter.FilterID2=0x0000; 
	HAL_FDCAN_ConfigFilter(&hfdcan2,&FDCAN2_RXFilter);
	HAL_FDCAN_Start(&hfdcan2);       
  HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	
}

void Init_Motors() {
	
	Clear_Cascade_PID( &Motor_Tasks.gimbal_yaw_pid) ;//PID清理
	Init_Cascade_PID( &Motor_Tasks.gimbal_yaw_pid );//PID初始�?
	Motor_Tasks.gimbal_yaw_motor.ex_angle = 0 ;
	Motor_Tasks.gimbal_yaw_motor.ex_speed = 0 ;
	
	//底盘PID的初始化
	Init_chasis_PID(&Motor_Tasks.chasis_pid_1);
	Init_chasis_PID(&Motor_Tasks.chasis_pid_2);
	Init_chasis_PID(&Motor_Tasks.chasis_pid_3);
	Init_chasis_PID(&Motor_Tasks.chasis_pid_4);
	Init_follow_PID(&Motor_Tasks.follow_pid);
}



void get_gimbal_angle(motor_tasks* mt , uint8_t* rxData );
void get_gimbal_chasis_angle_gap( motor_tasks* mt , uint8_t* rxData );
int flag = 0 ;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan , uint32_t RxFifo0ITs )  //FDCAN新消息中�?
{  
    FDCAN_RxHeaderTypeDef rxHeader;
	
	
    // �? FIFO0 读取消息  
	if(hfdcan == &hfdcan1 ) {
		
			int8_t rxData[8]; // 根据数据长度调整  
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, (uint8_t*)rxData) == HAL_OK)  
			{  
					// 处理接收到的数据  
					// 例如，可以根�? rxHeader.ID 进行消息分类  
				switch (rxHeader.Identifier ) {
					case CAN_3508_M1_ID :
					{
						fdcan_message_get(&Motor_Tasks.chasis_motor_1,(uint8_t*)rxData );
						break;
					}
					case CAN_3508_M2_ID :
					{
						fdcan_message_get(&Motor_Tasks.chasis_motor_2,(uint8_t*)rxData );
						break;
					}
					case CAN_3508_M3_ID :
					{
						fdcan_message_get(&Motor_Tasks.chasis_motor_3,(uint8_t*)rxData );
						break;
					}
					case CAN_3508_M4_ID :
					{
						fdcan_message_get(&Motor_Tasks.chasis_motor_4,(uint8_t*)rxData );
						break;
					}
					case CAN_6020_M1_ID :
					{
						
						fdcan_message_get(&Motor_Tasks.gimbal_yaw_motor,(uint8_t*)rxData );
						
						//获取底盘与云台相差角度
						get_gimbal_chasis_angle_gap( &Motor_Tasks , (uint8_t*)rxData );
						
						//获取初始角度
						if( !flag ) {
							Motor_Tasks.basic_yaw = 0 ;
							//Motor_Tasks.basic_yaw = (float)( (int16_t)(rxData[0]<<8)| (int16_t)rxData[1]) /(8191.0f)*2*(float)PI ;//换位弧度制
							//Motor_Tasks.basic_yaw -= (2*PI-4.13) ;//归零度角，安装时的零度角需要消除
							flag = 1 ;
						}
						break;
					}
					default:
					{
						break ;
					}
				}
			}
		}  
		
	if( hfdcan == &hfdcan2 ) {
		int8_t rxData[rxBufferLen] ;
		memset(rxData,0,sizeof(rxData));
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, (uint8_t*)rxData) == HAL_OK) {
			switch (rxHeader.Identifier ) {
				case CAN_G473_gimbal :
				{
					if(rxData[0] != -1 ) {
						DR16_command_process(&controller,(uint8_t*)rxData);//获取数据
						gimbal_control(&controller,&Motor_Tasks);//云台yaw轴期望值的改变
						chasis_moving_encoder(&Motor_Tasks,&controller);//底盘解算
					}
					else {
						get_gimbal_angle(&Motor_Tasks,(uint8_t*)rxData);//获取云台的yaw轴角度
					}
					break ;
				}
				default:
				{
					break ;
				}
			}
		}
	}
	
}  

void get_gimbal_angle(motor_tasks* mt , uint8_t* rxData ) {
	mt->gimbal_yaw = (float) ((int32_t)(rxData[1]<<16)|(int32_t)(rxData[2]<<8)|(int32_t)(rxData[3]));
	mt->gimbal_yaw = mt->gimbal_yaw/10000.0f ;//弧度制
}

void get_gimbal_chasis_angle_gap( motor_tasks* mt , uint8_t* rxData ) {
	mt->c6020_yaw = (float)(( (int16_t)(rxData[0]<<8)| (int16_t)rxData[1]) -5400.0f) /(8191.0f)*(float)(2*PI) ;//角度制
	//mt->c6020_yaw -= (float )PI ;
	if( mt->c6020_yaw < 0 )
			mt->c6020_yaw += 2*PI ;
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
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  /* USER CODE BEGIN 2 */
	Init_Motors();
	Init_can_filter();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Motor_PID_update(&Motor_Tasks);
		chasis_pid_excute(&Motor_Tasks);
		follow_pid_excute(&Motor_Tasks);
		gimbal_motor_pid_excute(&Motor_Tasks);
		
		chasis_motor_message_send(&hfdcan1,&Motor_Tasks);
		
		for(int i = 1 ; i <= 100000 ; ++ i );
		
		gimbal_motor_message_send(&hfdcan1,&Motor_Tasks);
		
		for(int i = 1 ; i <= 100000 ; ++ i );
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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
