#include "BMI.h"
#include "math.h"
#include "tim.h"
#include "string.h"
#include "fdcan.h"
#include "motor_message.h"

#define BMI088_ACCEL_3G_SEN 0.0008974358974f    //这个数字我也不知道哪来的
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f    //这个数字我也不知道哪来的

#define PI 3.14159265358979323846


float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
float accelerometer[3];
float gyro[3];
float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f};
uint8_t i = 0;
uint8_t buf[8]={0,0,0,0,0,0,0,0};
uint8_t pTxData;
uint8_t pRxData;
 
void Init_BMI(void)
{
    //这9行代码，向地址0x7E处写入0xB6值，加速度计软件复位，使加速度计各个寄存器恢复为默认值
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);    //PA4置0，片选加速度计
    pTxData = (0x7E & 0x7F);    //Bit #0和Bit #1-7，Bit #0是0，表示写
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    pTxData = 0xB6;    //Bit #8-15
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    HAL_Delay(1);    //延时1ms
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);    //PA4置1，取消片选加速度计
 
    //加速度计复位后默认是暂停模式，这9行代码，向地址0x7D处写入0x04值，使加速度计进入正常模式
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);    //PA4置0，片选加速度计
    pTxData = (0x7D & 0x7F);    //Bit #0和Bit #1-7，Bit #0是0，表示写
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    pTxData = 0x04;    //Bit #8-15
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    HAL_Delay(1);    //延时1ms
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);    //PA4置1，取消片选加速度计
 
    //这9行代码，向地址0x14处写入0xB6值，陀螺仪软件复位，使陀螺仪各个寄存器恢复为默认值
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);    //PB0置0，片选陀螺仪
    pTxData = (0x14 & 0x7F);    //Bit #0和Bit #1-7，Bit #0是0，表示写
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    pTxData = 0xB6;    //Bit #8-15
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    HAL_Delay(30);    //延时30ms
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);    //PB0置1，取消片选陀螺仪
		
    HAL_TIM_Base_Start_IT(&htim1);    //使TIM1定时器开始工作，并使能其定时中断
 
}

void get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
	*yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f) *2.0f*1000.0f/2.59f*(float)PI ;
	*pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2])) ;
	*roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f) ;
	
	//限制角度在0-2PI
	while( *yaw > 2*PI )
			*yaw -= 2*PI ;
	while( *yaw < 0 )
			*yaw += 2*PI ;
}
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);    //PA4置0，片选加速度计
    pTxData = (0x12 | 0x80);    //Bit #0和Bit #1-7，Bit #0是1，表示读
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #8-15，无效值
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //等待SPI接收完成
    i = 0;
    while (i < 6)
    {
        HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #16-23，寄存器0x12的值，然后是寄存器0x13、0x14、0x15、0x16、0x17的值
    	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //等待SPI接收完成
    	buf[i] = pRxData;
        i++;
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);    //PA4置1，取消片选加速度计
    accelerometer[0] = ((int16_t)((buf[1]) << 8) | buf[0]) * BMI088_ACCEL_SEN;
    accelerometer[1] = ((int16_t)((buf[3]) << 8) | buf[2]) * BMI088_ACCEL_SEN;
    accelerometer[2] = ((int16_t)((buf[5]) << 8) | buf[4]) * BMI088_ACCEL_SEN;
 
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);    //PB0置0，片选陀螺仪
    pTxData = (0x00 | 0x80);    //Bit #0和Bit #1-7，Bit #0是1，表示读
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    i = 0;
    while (i < 8)
    {
    	HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #8-15，寄存器0x00的值，然后是寄存器0x01、0x02、0x03、0x04、0x05、0x06、0x07的值
    	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //等待SPI接收完成
    	buf[i] = pRxData;
    	i++;
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);    //PB0置1，取消片选陀螺仪
    if(buf[0] == 0x0F)	//buf[0]储存GYRO_CHIP_ID，应该为0x0F，判断我们读取到的是不是陀螺仪的值。
    {
    	gyro[0] = ((int16_t)((buf[3]) << 8) | buf[2]) * BMI088_GYRO_SEN;
    	gyro[1] = ((int16_t)((buf[5]) << 8) | buf[4]) * BMI088_GYRO_SEN;
    	gyro[2] = ((int16_t)((buf[7]) << 8) | buf[6]) * BMI088_GYRO_SEN;
    }
 
	MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accelerometer[0], accelerometer[1], accelerometer[2]);    //对得到的加速度计、陀螺仪数据进行计算得到四元数
	get_angle(quat, INS_angle, INS_angle+1, INS_angle+2);    //对四元数计算得到欧拉角
		
	//乘1000，得弧度制的值
	
		
	//发送
	uint8_t message[8] ;
	memset(message , 0 ,sizeof(message )) ;
	message[0] = -1 ;
	int angle = (int)(INS_angle[0]*10000.0f) ;
	message[1] = (uint8_t)(angle>>16) ;
	message[2] = (uint8_t)(angle>>8) ;
	message[3] = (uint8_t)(angle) ;
	gimbal_chasis_message_send(&hfdcan2,(int8_t*)message);
}
