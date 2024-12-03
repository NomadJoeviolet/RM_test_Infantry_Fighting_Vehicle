#include "motor_message.h"

void chasis_motor_message_send(FDCAN_HandleTypeDef* fdhcan,motor_tasks* motors ) {
	//uint32_t send_mail_box ;
	int8_t message[8] ;
	
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
	
		Txheader_gimbal.Identifier=0x200;                           //32位ID
    Txheader_gimbal.IdType=FDCAN_STANDARD_ID;                  //标准ID
    Txheader_gimbal.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    Txheader_gimbal.DataLength=0x08;                            //数据长度
		
    Txheader_gimbal.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
    Txheader_gimbal.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    Txheader_gimbal.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    Txheader_gimbal.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
    Txheader_gimbal.MessageMarker=0; 
	
	HAL_FDCAN_AddMessageToTxFifoQ( fdhcan, &Txheader_gimbal , (uint8_t*)message );

	
}


void gimbal_motor_message_send(FDCAN_HandleTypeDef* fdhcan,motor_tasks* motors ) {
	
	//uint32_t send_mail_box ;
	int8_t message[8] ;
	
	memset(message,0,sizeof(message));
	message[0] = (motors->gimbal_yaw_motor.output >>8);
	message[1] = (motors->gimbal_yaw_motor.output);
	
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

void fdcan_message_get(motor* Motor ,uint8_t* rxData) {
	
	//Motor->c6020_yaw = (float)( (int16_t)(rxData[0]<<8)| (int16_t)rxData[1]) /(8191.0f)*(360.0f) ;//角度制
	Motor->ac_speed = (float )( (int16_t)(rxData[2]<<8) | (int16_t)rxData[3] ) ;
	Motor->ac_current = (float )( (int16_t)(rxData[4]<<8) | (int16_t)rxData[5] );
	Motor->ac_temparature = (float )rxData[6] ;
	
}
