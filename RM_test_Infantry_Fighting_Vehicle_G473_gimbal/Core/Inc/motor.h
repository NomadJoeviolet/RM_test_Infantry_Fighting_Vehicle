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
	//motor gimbal_yaw_motor ;
	//gimbal_motor_pid gimbal_yaw_pid ;
	
	motor gimbal_pitch_motor ;
	gimbal_motor_pid gimbal_pitch_pid ;
	
}motor_tasks;

void gimbal_control(Controller*,motor_tasks*);//控制传值
void Motor_PID_update(motor_tasks*);//将数据更新进入PID
void gimbal_motor_pid_excute(motor_tasks*);//执行云台pid控制

#endif
