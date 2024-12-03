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
	uint16_t output ;
}motor;

typedef struct Motor_Tasks {
	motor gimbal_yaw_motor ;
	gimbal_motor_pid gimbal_yaw_pid ;
	
	motor gimbal_pitch_motor ;
	gimbal_motor_pid gimbal_pitch_pid ;
	
	motor chasis_motor_1 ;
	gimbal_motor_pid chasis_pid_1 ;
	
	motor chasis_motor_2 ;
	gimbal_motor_pid chasis_pid_2 ;
	
	motor chasis_motor_3 ;
	gimbal_motor_pid chasis_pid_3 ;
	
	motor chasis_motor_4 ;
	gimbal_motor_pid chasis_pid_4 ;
	
}motor_tasks;

void gimbal_control(Controller*,motor_tasks*);//控制传值
void chasis_moving_encoder( motor_tasks* mt , Controller* con );//运动学解算
void Motor_PID_update(motor_tasks*);//将数据更新进入PID

void chasis_pid_excute(motor_tasks*);
//void motors_output(motor_tasks*);

//motor_tasks Motor_Tasks ;//所有电机的控制

#endif
