#include "motor_message.h"

/*void chasis_motor_message_send(FDCAN_HandleTypeDef* fdhcan,motor_tasks* motors ) {
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
}*/


void gimbal_motor_message_send(FDCAN_HandleTypeDef* fdhcan,motor_tasks* motors ) {
	int8_t message[8] ;
	
	memset(message,0,sizeof(message));
	
	message[2] = (motors->gimbal_pitch_motor.output >>8);
	message[3] = (motors->gimbal_pitch_motor.output);
	
	
	FDCAN_TxHeaderTypeDef Txheader_gimbal ;
	/*Txheader_gimbal.DataLength = 0x08 ;
	Txheader_gimbal.Identifier = 0x1FF ;
	Txheader_gimbal.TxFrameType = FDCAN_DATA_FRAME ;
	Txheader_gimbal.IdType = FDCAN_STANDARD_ID ;*/
	
		Txheader_gimbal.Identifier=0x1FF;                           //32位ID
    Txheader_gimbal.IdType=FDCAN_STANDARD_ID;                  //标准ID
    Txheader_gimbal.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    Txheader_gimbal.DataLength=0x08;                            //数据长度
		
    Txheader_gimbal.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
    Txheader_gimbal.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    Txheader_gimbal.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    Txheader_gimbal.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
	
	HAL_FDCAN_AddMessageToTxFifoQ( fdhcan, &Txheader_gimbal , (uint8_t*)message );
	
}

void gimbal_chasis_message_transform(Controller* con ,int8_t* message) {
	message[0] = con->rc.ch0 >> 8 ;
	message[1] = con->rc.ch0 ;
	message[2] = con->rc.ch2 >> 8 ;
	message[3] = con->rc.ch2 ;
	message[4] = con->rc.ch3 >> 8 ;
	message[5] = con->rc.ch3 ;
	message[6] = con->rc.s1 ;
	message[7] = 1 ;
}

void gimbal_chasis_message_send(FDCAN_HandleTypeDef* fdhcan, int8_t* message ) {
	
	FDCAN_TxHeaderTypeDef Txheader_gimbal ;
	/*Txheader_gimbal.DataLength = 0x08 ;
	Txheader_gimbal.Identifier = 0x001 ;
	Txheader_gimbal.TxFrameType = FDCAN_DATA_FRAME ;
	Txheader_gimbal.IdType = FDCAN_STANDARD_ID ;*/
	
		Txheader_gimbal.Identifier=0x001;                           //32位ID
    Txheader_gimbal.IdType=FDCAN_STANDARD_ID;                  //标准ID
    Txheader_gimbal.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    Txheader_gimbal.DataLength=0x08;                            //数据长度
		
    Txheader_gimbal.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
    Txheader_gimbal.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    Txheader_gimbal.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    Txheader_gimbal.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
	
	
	HAL_FDCAN_AddMessageToTxFifoQ( fdhcan, &Txheader_gimbal , (uint8_t*)message );
	
}

void fdcan_message_get(motor* Motor ,uint8_t* rxData) {
	
	Motor->ac_angle = (float)( (int16_t)(rxData[0]<<8)| (int16_t)rxData[1]) /(8191.0f)*(360.0f) ;
	Motor->ac_speed = (float )( (int16_t)(rxData[2]<<8) | (int16_t)rxData[3] ) ;
	Motor->ac_current = (float )( (int16_t)(rxData[4]<<8) | (int16_t)rxData[5] );
	Motor->ac_temparature = (float )rxData[6] ;
	
}
