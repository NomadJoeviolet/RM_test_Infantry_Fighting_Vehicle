#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "DR16.h"
#include "pid.h"

typedef struct Motor {
	float ac_speed ;//电机反馈得到
	float ac_current ;//电机反馈得到
	float ac_angle ;//电机反馈得到
	float ac_temparature ;//电机反馈得到
	float ex_speed ;//Controller解码+运动学解算得到
	float ex_angle ;//Controller解码+运动学解算得到
	int16_t output ;
}motor;

typedef struct Motor_Tasks {
	
	float basic_yaw ;
	float gimbal_yaw ;
	float chasis_yaw ;
	float c6020_yaw ;
	motor gimbal_yaw_motor ;
	gimbal_motor_pid gimbal_yaw_pid ;
	
	motor chasis_motor_1 ;
	gimbal_motor_pid chasis_pid_1 ;
	
	motor chasis_motor_2 ;
	gimbal_motor_pid chasis_pid_2 ;
	
	motor chasis_motor_3 ;
	gimbal_motor_pid chasis_pid_3 ;
	
	motor chasis_motor_4 ;
	gimbal_motor_pid chasis_pid_4 ;
	
	gimbal_motor_pid follow_pid ;
	
}motor_tasks;

void gimbal_control(Controller*,motor_tasks*);//控制传值

void chasis_moving_encoder( motor_tasks* mt , Controller* con );//运动学解算

void Motor_PID_update(motor_tasks*);//将数据更新进入PID

void chasis_pid_excute(motor_tasks*);

void gimbal_motor_pid_excute(motor_tasks* mt );

void follow_pid_excute(motor_tasks* mt );

#endif
