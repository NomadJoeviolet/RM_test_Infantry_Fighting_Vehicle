#ifndef __DR16_H__
#define __DR16_H__

#include "main.h"

// __packed的结构体内部不能用匿名结构体

typedef struct RC_Control {
		int16_t ch0 ;
		int16_t ch1 ;
		int16_t ch2 ;
		int16_t ch3 ;
		int8_t s1 ;
		int8_t s2 ;
		int16_t roll ;
		
}RC ;

typedef struct Mouse_Control {
		uint16_t x ;
		uint16_t y ;
		uint16_t z ;
		uint16_t press_l ;
		uint16_t press_r ;
}Mouse ;

typedef struct Key_Control {
	uint16_t v ;
}Key ;

//__packed
typedef struct Controller {//packed关键字，不用内存对齐，节省内存
	
	RC rc ;
	Mouse mouse ;
	Key key ;
	float gimbal_angle ;
}Controller ;

void DR16_command_process(Controller* controller , uint8_t* buffer );

#endif
