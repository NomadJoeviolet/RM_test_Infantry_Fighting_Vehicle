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
	
	
	//想做C++类一样的成员函数的，结果发现指针每次使用前都要指一下函数
	/*void (*Calculate_PID)( struct pid_test* PID );
	void (*Clear_PID)(struct pid_test* PID );
	void (*Get_Value_PID)( struct pid_test* PID );
	void (*Init_PID)(struct pid_test* PID );*/
}gimbal_motor_pid;


void Clear_Cascade_PID( struct pid_test* PID );//清理过去数据
void Init_Cascade_PID( struct pid_test* PID );//初始化
void Calculate_Cascade_PID( struct pid_test* PID );

void Init_chasis_PID(gimbal_motor_pid* PID);
void Calculate_PID(gimbal_motor_pid* PID);

void Init_follow_PID(gimbal_motor_pid* PID);
void Calculate_follow_PID(gimbal_motor_pid* PID);
	
#endif
