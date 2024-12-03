#include "pid.h"

/*typedef struct pid_test {
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
	void (*Calculate_PID)( struct pid_test* PID );
	void (*Clear_PID)(struct pid_test* PID );
	void (*Get_Value_PID)( struct pid_test* PID );
	void (*Init_PID)(struct pid_test* PID );
}motor_pid;*/





//串级PID——————————————————控制云台

void Clear_Cascade_PID( struct pid_test* PID ) {//清空历史数据
	PID->ac_angle = 0 ;//实际角度
	PID->ac_speed = 0 ;//实际速度
	PID->error_a = 0 ;//角度误差
	PID->last_error_a = 0 ;//上次角度误差
	PID->error_s = 0 ;//速度误差
	PID->last_error_s = 0 ;//上次速度误差
	PID->ex_angle = 90 ;//期待角度
	PID->ex_speed = 0 ;//期待速度
	PID->integral_a = 0 ;//角度误差积分
	PID->integral_s = 0 ;//速度误差积分
	PID->output_e = 0 ;//外环输出
	PID->output_i = 0 ;//内环输出
}

void Init_Cascade_PID( struct pid_test* PID ) {
	//Init kp,ki,kd
	PID->kp_e = 20 ;
	PID->ki_e = 0.01 ;
	PID->kd_e = 0.1 ;
	PID->in_a_max = 10000 ;
	PID->in_a_min = -10000 ;
	
	PID->kp_i = 20 ;
	PID->ki_i = 0.1 ;
	PID->kd_i = 1 ;
	PID->in_s_max = 10000 ;
	PID->in_s_min = -10000 ;
	
	//设置目标角度，自己手动输入
	//PID->ex_angle = 30 ;
}	

/*void Get_Value_Cascade_PID_yaw( struct pid_test* PID ) {
	//获取当前的值
	//PID->ac_angle = Motor_Tasks.gimbal_yaw_motor.ac_angle*360.0f/8191.0f ;//单位为度
	//PID->ac_speed = Motor_Tasks.gimbal_yaw_motor.ac_speed ;
	
	
	float gap_angle = PID->ex_angle-PID->ac_angle ;
	 if( gap_angle >= 180 )//角度倒转
		 PID->ac_angle += 360 ;
	 if( gap_angle <= -180 )
		 PID->ac_angle -= 360 ;
}*/

/*void Get_Value_Cascade_PID_pitch( struct pid_test* PID ) {
	//获取当前的值
	//PID->ac_angle = Motor_Tasks.gimbal_pitch_motor.ac_angle*360.0f/8191.0f ;//单位为度
	//PID->ac_speed = Motor_Tasks.gimbal_pitch_motor.ac_speed ;
	
	
	float gap_angle = PID->ex_angle-PID->ac_angle ;
	 if( gap_angle >= 180 )//角度倒转
		 PID->ac_angle += 360 ;
	 if( gap_angle <= -180 )
		 PID->ac_angle -= 360 ;
}*/

void Get_Value_Cascade_PID( struct pid_test* PID ) {
	//获取当前的值
	//PID->ac_angle = Motor_Tasks.gimbal_pitch_motor.ac_angle*360.0f/8191.0f ;//单位为度
	//PID->ac_speed = Motor_Tasks.gimbal_pitch_motor.ac_speed ;
	
	
	float gap_angle = PID->ex_angle-PID->ac_angle ;
	 if( gap_angle >= 180 )//角度倒转
		 PID->ac_angle += 360 ;
	 if( gap_angle <= -180 )
		 PID->ac_angle -= 360 ;
}

void Calculate_In_PID( struct pid_test* PID ) {//内环
	
	PID->last_error_s = PID->error_s ;
	PID->error_s = PID->ex_speed-PID->ac_speed ;
	
	float de_dt = PID->error_s-PID->last_error_s ;
	
	if( PID->integral_s > PID->in_s_max && PID->error_s > 0 )
			PID->integral_s -= PID->error_s ;
	if( PID->integral_s < PID->in_s_min && PID->error_s < 0 )
			PID->integral_s -= PID->error_s ;
	PID->integral_s += PID->error_s ;
	
	PID->output_i = PID->kp_i*PID->error_s + PID->ki_i*PID->integral_s + PID->kd_i*de_dt ;
	if(PID->output_i > 25000 )//控制输出不越界
			PID->output_i = 25000 ;
	if( PID->output_i < -25000 )
			PID->output_i = -25000 ;
}

void Calculate_Ex_PID( struct pid_test* PID ) {//外环
	
	PID->last_error_a = PID->error_a ;
	PID->error_a = PID->ex_angle - PID->ac_angle ;
	
	float de_dt = PID->error_a - PID->last_error_a ;
	
	if( PID->integral_a > PID->in_a_max && PID->error_a > 0 )//积分限幅
		PID->integral_a -= PID->error_a ;
	if( PID->integral_a < PID->in_a_min && PID->error_a < 0 )//积分限幅
		PID->integral_a -= PID->error_a ;
	PID->integral_a += PID->error_a ;
	
	PID->output_e = PID->kp_e*PID->error_a + PID->ki_e*PID->integral_a +PID->kd_e*de_dt ;
	if(PID->output_e>250)//控制输出不越界
			PID->output_e = 250 ;//250pm以内
	if(PID->output_e<-250)
			PID->output_e = -250 ;
	PID->ex_speed = PID->output_e ;
	
}

void Calculate_Cascade_PID( struct pid_test* PID ) {
	//外环角度环
	Calculate_Ex_PID(PID) ;
	
	//内环速度环
	Calculate_In_PID(PID);
}

float test_output = 0.0f;

/*void MotorTask()
{
	Clear_PID (&pid);
	Init_PID(&pid) ;
	for (;;)
	{
		 Write your code here
		
		Get_Value_PID(&pid);
		Calculate_PID (&pid);
		test_output = pid.output_i ;
		
		Motor_SetOutput(&Motor_GimbalYaw, test_output);
		Motor_CAN_SendGroupOutput(&Motor_GimbalMotors);		
		osDelay(1);
	}
}*/




//普通PID——————————————————————控制底盘

void Init_chasis_PID(gimbal_motor_pid* PID) {
	//PID->kp_e = 20 ;
	//PID->ki_e = 0.01 ;
	//PID->kd_e = 0.1 ;
	//PID->in_a_max = 10000 ;
	//PID->in_a_min = -10000 ;
	
	PID->kp_i = 20 ;
	PID->ki_i = 0.1 ;
	PID->kd_i = 1 ;
	PID->in_s_max = 10000 ;
	PID->in_s_min = -10000 ;
}

void Calculate_PID(gimbal_motor_pid* PID) {
	PID->last_error_s = PID->error_s ;
	PID->error_s = PID->ex_speed-PID->ac_speed ;
	
	float de_dt = PID->error_s-PID->last_error_s ;
	
	if( PID->integral_s > PID->in_s_max && PID->error_s > 0 )
			PID->integral_s -= PID->error_s ;
	if( PID->integral_s < PID->in_s_min && PID->error_s < 0 )
			PID->integral_s -= PID->error_s ;
	PID->integral_s += PID->error_s ;
	
	PID->output_i = PID->kp_i*PID->error_s + PID->ki_i*PID->integral_s + PID->kd_i*de_dt ;
	if(PID->output_i > 25000 )//控制输出不越界
			PID->output_i = 25000 ;
	if( PID->output_i < -25000 )
			PID->output_i = -25000 ;
}

