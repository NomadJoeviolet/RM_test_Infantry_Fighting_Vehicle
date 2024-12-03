#include "pid.h"

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
	PID->kp_e = 45 ;
	PID->ki_e = 5.00000006e-08 ;
	PID->kd_e = 12.5 ;
	PID->in_a_max = 10000 ;
	PID->in_a_min = -10000 ;
	
	PID->kp_i = 55 ;
	PID->ki_i = 4.99999999e-07 ;
	PID->kd_i = 15 ;
	PID->in_s_max = 10000 ;
	PID->in_s_min = -10000 ;
	
	//设置目标角度，自己手动输入
	//PID->ex_angle = 30 ;
}	

void Pre_angle_Cascade_PID( struct pid_test* PID ) {
	
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
	if(PID->output_e>350)//控制输出不越界
			PID->output_e = 350 ;//250pm以内
	if(PID->output_e<-350)
			PID->output_e = -350 ;
	PID->ex_speed = PID->output_e ;
	
}

void Calculate_Cascade_PID( struct pid_test* PID ) {
	
	Pre_angle_Cascade_PID(PID) ;
	//外环角度环
	Calculate_Ex_PID(PID) ;
	//内环速度环
	Calculate_In_PID(PID);
}


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




