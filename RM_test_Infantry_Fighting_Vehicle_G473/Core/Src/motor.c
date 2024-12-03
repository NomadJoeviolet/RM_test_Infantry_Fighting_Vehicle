#include "motor.h"

void gimbal_control(Controller* con ,motor_tasks* mr_ts ) {//改变期望值
	mr_ts->gimbal_yaw_motor.ex_angle += (float)(5.0f*((float)con->rc.ch0/(float)32767.0)) ;
	mr_ts->gimbal_pitch_motor.ex_angle += (float)(float)(5.0f*((float)con->rc.ch1/(float)32767.0)) ;
	
	mr_ts->gimbal_yaw_pid.ex_angle = mr_ts->gimbal_yaw_motor.ex_angle ;
	mr_ts->gimbal_pitch_pid.ex_angle = mr_ts->gimbal_pitch_motor.ex_angle ;
}


//将数据更新进入PID
void Motor_PID_update(motor_tasks* mt) {
	
	mt->chasis_pid_1.ac_angle = mt->chasis_motor_1.ac_angle ;
	mt->chasis_pid_1.ac_speed = mt->chasis_motor_1.ac_speed ;
	mt->chasis_pid_1.ex_angle = mt->chasis_motor_1.ex_angle ;
	mt->chasis_pid_1.ex_speed = mt->chasis_motor_1.ex_speed ;
	
	mt->chasis_pid_2.ac_angle = mt->chasis_motor_2.ac_angle ;
	mt->chasis_pid_2.ac_speed = mt->chasis_motor_2.ac_speed ;
	mt->chasis_pid_2.ex_angle = mt->chasis_motor_2.ex_angle ;
	mt->chasis_pid_2.ex_speed = mt->chasis_motor_2.ex_speed ;
	
	mt->chasis_pid_3.ac_angle = mt->chasis_motor_3.ac_angle ;
	mt->chasis_pid_3.ac_speed = mt->chasis_motor_3.ac_speed ;
	mt->chasis_pid_3.ex_angle = mt->chasis_motor_3.ex_angle ;
	mt->chasis_pid_3.ex_speed = mt->chasis_motor_3.ex_speed ;
	
	mt->chasis_pid_4.ac_angle = mt->chasis_motor_4.ac_angle ;
	mt->chasis_pid_4.ac_speed = mt->chasis_motor_4.ac_speed ;
	mt->chasis_pid_4.ex_angle = mt->chasis_motor_4.ex_angle ;
	mt->chasis_pid_4.ex_speed = mt->chasis_motor_4.ex_speed ;
	
	mt->gimbal_pitch_pid.ac_angle = mt->gimbal_pitch_motor.ac_angle ;
	mt->gimbal_pitch_pid.ac_speed = mt->gimbal_pitch_motor.ac_speed ;
	mt->gimbal_pitch_pid.ex_angle = mt->gimbal_pitch_motor.ex_angle ;
	mt->gimbal_pitch_pid.ex_speed = mt->gimbal_pitch_motor.ex_speed ;
	
	mt->gimbal_yaw_pid.ac_angle = mt->gimbal_yaw_motor.ac_angle ;
	mt->gimbal_yaw_pid.ac_speed = mt->gimbal_yaw_motor.ac_speed ;
	mt->gimbal_yaw_pid.ex_angle = mt->gimbal_yaw_motor.ex_angle ;
	mt->gimbal_yaw_pid.ex_speed = mt->gimbal_yaw_motor.ex_speed ;
	
}

//底盘运动学解算
void chasis_moving_encoder( motor_tasks* mt , Controller* con ) {
	float mul = 0 ;
	if( con->rc.s1 == 1 )
			mul = 1 ;
	float w = con->rc.roll*mul ;
	float a = 1 , b = 1 ;
	float v_x , v_y ;
	v_x = ( (float)con->rc.ch2/ 660.0f )*400.0f ;
	v_y = ( (float)con->rc.ch3/ 660.0f )*400.0f ;
	
	mt->chasis_motor_1.ex_speed = v_y-v_x+w*(a+b) ;
	mt->chasis_motor_2.ex_speed = v_y+v_x-w*(a+b) ;
	mt->chasis_motor_3.ex_speed = v_y-v_x-w*(a+b) ;
	mt->chasis_motor_4.ex_speed = v_y+v_x+w*(a+b) ;
	
}


void chasis_pid_excute(motor_tasks* mt ) {
	Calculate_PID(&mt->chasis_pid_1);
	Calculate_PID(&mt->chasis_pid_2);
	Calculate_PID(&mt->chasis_pid_3);
	Calculate_PID(&mt->chasis_pid_4);
	
	mt->chasis_motor_1.output = (int16_t)mt->chasis_pid_1.output_i ;
	mt->chasis_motor_2.output = (int16_t)mt->chasis_pid_2.output_i ;
	mt->chasis_motor_3.output = (int16_t)mt->chasis_pid_3.output_i ;
	mt->chasis_motor_4.output = (int16_t)mt->chasis_pid_4.output_i ;
	
}

