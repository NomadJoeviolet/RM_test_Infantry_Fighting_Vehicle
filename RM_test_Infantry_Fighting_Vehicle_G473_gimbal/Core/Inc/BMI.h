#ifndef __BMI_H__
#define __BMI_H__

#include "main.h"
#include "MahonyAHRS.h"
#include "spi.h"

void Init_BMI(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif
