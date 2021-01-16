#ifndef LED_H
#define LED_H

#define LED_G_Toggle GPIO_ToggleBits(GPIOF, GPIO_Pin_14)
#define LED_R_Toggle GPIO_ToggleBits(GPIOE, GPIO_Pin_11)
#define LED_R_On GPIO_ResetBits(GPIOE, GPIO_Pin_11)
#define LED_R_Off GPIO_SetBits(GPIOE, GPIO_Pin_11)

void LED_Init(void);

#endif
