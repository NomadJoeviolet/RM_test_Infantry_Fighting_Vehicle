#ifndef __PID_H__
#define __PID_H__

#include "main.h"

typedef struct pid_test {//串级pid
	float kp_i , ki_i , kd_i ;
	float kp_e , ki_e , kd_e ;
	float output_e , output_i ;
	float ex_angle , ac_angle ;//单位为度
	float ex_speed , ac_speed ;//单位为rpm
	float error_a , last_error_a ;
	float error_s , last_error_s ;
	float integral_a , integral_s ;
	
	float in_a_max , in_a_min ;//角度环积分限幅
	float in_s_max , in_s_min ;//速度环积分限幅
	
}gimbal_motor_pid;


void Clear_Cascade_PID( struct pid_test* PID );//清理过去数据
void Init_Cascade_PID( struct pid_test* PID );//初始化
void Calculate_Cascade_PID( struct pid_test* PID );

#endif
