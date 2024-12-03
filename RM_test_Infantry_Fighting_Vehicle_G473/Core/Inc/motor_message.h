#ifndef __MOTOR_MESSAGE_H__
#define __MOTOR_MESSAGE_H__

#include "main.h"
#include "fdcan.h"
#include "motor.h"
#include "string.h"

void chasis_motor_message_send(FDCAN_HandleTypeDef*,motor_tasks*);
void gimbal_motor_message_send(FDCAN_HandleTypeDef*,motor_tasks*);
void fdcan_message_get(motor*,uint8_t*);//获取数据，解码

#endif
