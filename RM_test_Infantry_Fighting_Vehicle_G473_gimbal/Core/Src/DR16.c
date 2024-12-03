#include "DR16.h"

void DR16_command_process(Controller* controller , uint8_t* buffer ) {
	if( buffer == NULL )
			return ;
	controller->rc.ch0 = ( (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8 ) ) & 0x07FF ;
	controller->rc.ch1 = ( ((uint16_t)buffer[1]>>3) | ((uint16_t)buffer[2]<<5 ) ) & 0x07FF ;
	controller->rc.ch2 = ( ((uint16_t)buffer[2] >> 6 ) | ((uint16_t)buffer[3] << 2 ) | ((uint16_t)buffer[4] << 10 ) ) & 0x07FF ;
  	controller->rc.ch3 = ( ((uint16_t)buffer[4]>>1) | ((uint16_t)buffer[5]<<7) ) & 0x07FF ;
	
	controller->rc.s1 = ((buffer[5]>>4) & 0x000C ) >> 2 ;
	controller->rc.s2 = ((buffer[5]>>4) & 0x0003 ) ; 
	
	//虚拟链路建立后，通过DR16传的信息（电脑鼠标键盘）
	controller->mouse.x = ( (uint16_t)buffer[6] | ((uint16_t)buffer[7] << 8 ) );
	controller->mouse.y = ( (uint16_t)buffer[8] | ((uint16_t)buffer[9] << 8 ) );
	controller->mouse.z =  ( (uint16_t)buffer[10] | ((uint16_t)buffer[11] << 8 ) ) ;
	
	controller->mouse.press_l = buffer[12] ;
	controller->mouse.press_r = buffer[13] ;
	
	controller->key.v = ((uint16_t)buffer[14] | ((uint16_t)buffer[15] << 8 ) ) ;// | ((uint16_t)buffer[15] << 8 );
	
	controller->rc.roll = (buffer[16] | (buffer[17] << 8)) & 0x07FF ;
	
	//解码完成后的优化
	//将重点归为零
	controller->rc.ch0 -= 1024 ;
	controller->rc.ch1 -= 1024 ;
	controller->rc.ch2 -= 1024 ;
	controller->rc.ch3 -= 1024 ;
	//设置死区
	if(controller->rc.ch0<=5 && controller->rc.ch0 >= -5 )
		controller->rc.ch0 = 0 ;
	if(controller->rc.ch1<=5 && controller->rc.ch1 >= -5 )
		controller->rc.ch1 = 0 ;
	if(controller->rc.ch2<=5 && controller->rc.ch2 >= -5 )
		controller->rc.ch2 = 0 ;
	if(controller->rc.ch3<=5 && controller->rc.ch3 >= -5 )
		controller->rc.ch3 = 0 ;
}

