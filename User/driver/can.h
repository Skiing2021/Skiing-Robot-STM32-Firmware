#ifndef CAN_H
#define CAN_H

#include "stm32f4xx.h"

void CAN_Config(void);
void CAN1_Send_Current(int16_t current1, int16_t current2, int16_t current3, int16_t current4);
void CAN_Filter_Config(void);
#endif
