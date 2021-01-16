#include "stm32f4xx.h"

CanRxMsg RxMessage;
int16_t motor_speed[8];
__IO int aaa;

static void CAN_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);
}

void CAN_Mode_Config(void){
	CAN_InitTypeDef CAN_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=ENABLE;
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=DISABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_6tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;
	
	CAN_InitStructure.CAN_Prescaler = 5;
	CAN_Init(CAN1, &CAN_InitStructure);
}

static void CAN_Filter_Config(void)
{
	CAN_FilterInitTypeDef CAN_FilterInitStructure;

	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;

	CAN_FilterInitStructure.CAN_FilterIdHigh= 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;

	CAN_FilterInit(&CAN_FilterInitStructure);

	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

static void CAN_NVIC_Config(void){
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void CAN_Config(void){
	CAN_GPIO_Config();
	CAN_Mode_Config();
	CAN_Filter_Config();
	CAN_NVIC_Config();
}

void CAN_Send_Current(uint32_t StdId, int16_t current[]){
	CanTxMsg TxMessage;
	TxMessage.StdId = StdId;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 8;
	for(uint8_t i = 0; i < 8; ++i){
		if(!current[i/2] & 1){
			TxMessage.Data[i] = current[i/2] << 8;
		} else{
			TxMessage.Data[i] = current[i/2];
		}
	}
	CAN_Transmit(CAN1, &TxMessage);
}

void CAN_Depackage(CanRxMsg* RxMessage){
	for(uint8_t i = 0; i < 8; ++i){
		if(RxMessage->StdId == 0x201 + i){
			motor_speed[i] = (RxMessage->Data[2] << 8) + RxMessage->Data[3];
		}
	}
}
