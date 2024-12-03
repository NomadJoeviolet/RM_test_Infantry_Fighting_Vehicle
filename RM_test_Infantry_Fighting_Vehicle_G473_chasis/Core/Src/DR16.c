#include "DR16.h"

void DR16_command_process(Controller* controller , uint8_t* buffer ) {
	if( buffer == NULL )
			return ;
	controller->rc.ch0 = ( (uint16_t)buffer[1] | ((uint16_t)buffer[0] << 8 ) );
	controller->rc.ch2 = ( (uint16_t)buffer[3] | ((uint16_t)buffer[2] << 8 ) );
 	controller->rc.ch3 = ( (uint16_t)buffer[5] | ((uint16_t)buffer[4] << 8 ) );
	
	controller->rc.s1 = buffer[6] ;

	
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

