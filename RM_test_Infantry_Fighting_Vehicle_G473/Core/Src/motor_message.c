#include "motor_message.h"

void chasis_motor_message_send(FDCAN_HandleTypeDef* fdhcan,motor_tasks* motors ) {
	//uint32_t send_mail_box ;
	uint8_t message[8] ;
	
	memset(message,0,sizeof(message));
	message[0] = (motors->chasis_motor_1.output >> 8);
	message[1] = (motors->chasis_motor_1.output );
	message[2] = (motors->chasis_motor_2.output >> 8);
	message[3] = (motors->chasis_motor_2.output );
	message[4] = (motors->chasis_motor_3.output >> 8);
	message[5] = (motors->chasis_motor_3.output );
	message[6] = (motors->chasis_motor_4.output >> 8);
	message[7] = (motors->chasis_motor_4.output );
	
	FDCAN_TxHeaderTypeDef Txheader_gimbal ;
	Txheader_gimbal.DataLength = 0x08 ;
	Txheader_gimbal.Identifier = 0x200 ;
	Txheader_gimbal.TxFrameType = FDCAN_DATA_FRAME ;
	Txheader_gimbal.IdType = FDCAN_STANDARD_ID ;
	
	HAL_FDCAN_AddMessageToTxFifoQ( fdhcan, &Txheader_gimbal , message );
}


void gimbal_motor_message_send(FDCAN_HandleTypeDef* fdhcan,motor_tasks* motors ) {
	
	//uint32_t send_mail_box ;
	uint8_t message[8] ;
	
	memset(message,0,sizeof(message));
	message[0] = (motors->gimbal_yaw_motor.output >>8);
	message[1] = (motors->gimbal_yaw_motor.output);
	message[2] = (motors->gimbal_pitch_motor.output >>8);
	message[2] = (motors->gimbal_pitch_motor.output);
	
	
	FDCAN_TxHeaderTypeDef Txheader_gimbal ;
	Txheader_gimbal.DataLength = 0x08 ;
	Txheader_gimbal.Identifier = 0x1FF ;
	Txheader_gimbal.TxFrameType = FDCAN_DATA_FRAME ;
	Txheader_gimbal.IdType = FDCAN_STANDARD_ID ;
	
	HAL_FDCAN_AddMessageToTxFifoQ( fdhcan, &Txheader_gimbal , message );
	
}

void fdcan_message_get(motor* Motor ,uint8_t* rxData) {
	Motor->ac_angle = (float)((rxData[0]<<8)|rxData[1]) ;
	Motor->ac_current = (float )((rxData[2]<<8)|rxData[3]) ;
	Motor->ac_speed = (float )((rxData[4]<<8)|rxData[5]);
	Motor->ac_temparature = (float )rxData[6] ;
}
