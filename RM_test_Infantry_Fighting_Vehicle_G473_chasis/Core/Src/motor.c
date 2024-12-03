#include "motor.h"
#include "math.h"

#define PI 3.14159265358979323846

void gimbal_control(Controller* con ,motor_tasks* mr_ts ) {//改变期望值
	mr_ts->gimbal_yaw_motor.ex_angle -= (float)(2.0f*((float)con->rc.ch0/(float)660.0)) ;//方向问题，改负
	
	//限制角度
	if( mr_ts->gimbal_yaw_motor.ex_angle > 360.0f )
			mr_ts->gimbal_yaw_motor.ex_angle -= 360 ;
	if( mr_ts->gimbal_yaw_motor.ex_angle < 0 )
		mr_ts->gimbal_yaw_motor.ex_angle += 360 ;
	
	mr_ts->gimbal_yaw_pid.ex_angle = mr_ts->gimbal_yaw_motor.ex_angle ;
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
	
	//mt->gimbal_yaw_pid.ac_angle = mt->gimbal_yaw_motor.ac_angle ;//yaw轴电机反馈数据
	mt->gimbal_yaw_motor.ac_angle = (mt->basic_yaw+mt->gimbal_yaw) ;//yaw轴角度，弧度制
	if(mt->gimbal_yaw_motor.ac_angle > 2*PI )
			mt->gimbal_yaw_motor.ac_angle -= 2*PI ;
	if( mt->gimbal_yaw_motor.ac_angle < 0 )
			mt->gimbal_yaw_motor.ac_angle += 2*PI ;
	mt->gimbal_yaw_motor.ac_angle = mt->gimbal_yaw_motor.ac_angle*360.0f/(float)(2*PI) ;//换成角度制
	
	
	mt->gimbal_yaw_pid.ac_angle = mt->gimbal_yaw_motor.ac_angle ;
	mt->gimbal_yaw_pid.ac_speed = mt->gimbal_yaw_motor.ac_speed ;
	mt->gimbal_yaw_pid.ex_angle = mt->gimbal_yaw_motor.ex_angle ;
	mt->gimbal_yaw_pid.ex_speed = mt->gimbal_yaw_motor.ex_speed ;
	
	mt->follow_pid.ac_angle = mt->c6020_yaw*360.0f/(float)(2*PI) ;//转换为角度制
	
}

//底盘运动学解算
void chasis_moving_encoder( motor_tasks* mt , Controller* con ) {
	float mul = 0 ;
	if( con->rc.s1 == 1 )
			mul = 1 ;
	float w = 2400 ;
	float a = 1 , b = 1 ;
	float v_x_gimbal , v_y_gimbal ;
	float v_x_chasis , v_y_chasis ;
	float v_w_chasis ;
	
	v_x_gimbal = ( (float)con->rc.ch2/ 660.0f )*2500.0f ;//左右，右为正，ch2左右
	v_y_gimbal = ( (float)con->rc.ch3/ 660.0f )*2500.0f ;//前后，前为正，ch3前后
	
	//角度是弧度制
	//float angle = mt->gimbal_yaw_motor.ac_angle/360.0f*(float)(2*PI) ;
	float angle = mt->c6020_yaw ; 
	//
	v_y_chasis = v_y_gimbal*cos(angle)+v_x_gimbal*sin(angle) ;
	v_x_chasis = v_x_gimbal*cos(angle)-v_y_gimbal*sin(angle) ;
	v_w_chasis = (1-mul)*(float)(mt->follow_pid.output_i);
	
	mt->chasis_motor_1.ex_speed = v_y_chasis+v_x_chasis-w*(a+b)*mul +v_w_chasis;
	mt->chasis_motor_2.ex_speed = -v_y_chasis+v_x_chasis-w*(a+b)*mul +v_w_chasis;//这两个电机反装
	mt->chasis_motor_3.ex_speed = -v_y_chasis-v_x_chasis-w*(a+b)*mul +v_w_chasis;//这两个电机反装
	mt->chasis_motor_4.ex_speed = v_y_chasis-v_x_chasis-w*(a+b)*mul +v_w_chasis;
	
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

void gimbal_motor_pid_excute(motor_tasks* mt ) {
	//Motor_PID_update(mt) ;
	//Calculate_Cascade_PID( &mt->gimbal_pitch_pid );
	Calculate_Cascade_PID( &mt->gimbal_yaw_pid );
	//mt->gimbal_pitch_motor.output = (uint16_t)mt->gimbal_pitch_pid.output_i ;
	mt->gimbal_yaw_motor.output = (int16_t)mt->gimbal_yaw_pid.output_i ;
	
}

void follow_pid_excute(motor_tasks* mt ) {
	Calculate_follow_PID(&mt->follow_pid);
}
