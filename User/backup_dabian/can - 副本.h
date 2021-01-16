#ifndef CAN_H
#define CAN_H

#include "stm32f4xx.h"

//#define kP 0.1

void CAN_Config(void);
void CAN_Send_Current(uint32_t StdId, int16_t current[]);
void CAN_Depackage(CanRxMsg* RxMessage);

#endif
