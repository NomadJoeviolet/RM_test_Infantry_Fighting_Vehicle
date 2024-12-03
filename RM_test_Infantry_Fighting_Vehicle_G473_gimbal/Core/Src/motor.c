#include "motor.h"

void gimbal_control(Controller* con ,motor_tasks* mr_ts ) {//改变期望值
	
	//mr_ts->gimbal_yaw_motor.ex_angle += (float)(1.0f*((float)con->rc.ch0/(float)660.0)) ;
	
	mr_ts->gimbal_pitch_motor.ex_angle -= (float)(3.0f*((float)con->rc.ch1/(float)660.0)) ;//减号，调整方向
	if( mr_ts->gimbal_pitch_motor.ex_angle < 0 )
			mr_ts->gimbal_pitch_motor.ex_angle += 360.0f ;
	if( mr_ts->gimbal_pitch_motor.ex_angle > 360.0f )
			mr_ts->gimbal_pitch_motor.ex_angle -= 360.0f ;
	
	//限制角度
	if(mr_ts->gimbal_pitch_motor.ex_angle <= 160.0f && mr_ts->gimbal_pitch_motor.ex_angle < 180.0f )//在正角度
		mr_ts->gimbal_pitch_motor.ex_angle += (float)(float)(3.0f*((float)con->rc.ch1/(float)660.0));
	if( mr_ts->gimbal_pitch_motor.ex_angle >= 220.0f && mr_ts->gimbal_pitch_motor.ex_angle > 180.0f )//在负角度
		mr_ts->gimbal_pitch_motor.ex_angle += (float)(float)(3.0f*((float)con->rc.ch1/(float)660.0));
	
}


//将数据更新进入PID
void Motor_PID_update(motor_tasks* mt) {
	
	mt->gimbal_pitch_pid.ac_angle = mt->gimbal_pitch_motor.ac_angle ;
	mt->gimbal_pitch_pid.ac_speed = mt->gimbal_pitch_motor.ac_speed ;
	mt->gimbal_pitch_pid.ex_angle = mt->gimbal_pitch_motor.ex_angle ;
	mt->gimbal_pitch_pid.ex_speed = mt->gimbal_pitch_motor.ex_speed ;
	
	
}

void gimbal_motor_pid_excute(motor_tasks* mt ) {
	Motor_PID_update(mt) ;
	Calculate_Cascade_PID( &mt->gimbal_pitch_pid );
	//Calculate_Cascade_PID( &mt->gimbal_yaw_pid );
	mt->gimbal_pitch_motor.output = (int16_t)mt->gimbal_pitch_pid.output_i ;
	//mt->gimbal_yaw_motor.output = (int16_t)mt->gimbal_yaw_pid.output_i ;
	
}

