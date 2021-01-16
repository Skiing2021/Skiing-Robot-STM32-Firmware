#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

void Delay_Init(void);
void Delay(__IO uint32_t nTime);
void delay_ms(__IO uint32_t nTime);
void delay_us(__IO uint32_t nTime);
void TimingDelay_Decrement(void);

#endif
