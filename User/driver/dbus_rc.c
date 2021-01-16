//
// Created by caesarw on 11/14/20.
//

#include "dbus_rc.h"
#include "main.h"

volatile unsigned char Remote_RX_Buffer[25];
NDJ6_Controller My_Remote;

/* ----------------------- Function Implements  ---------------------------- */

/******************************************************************************
 * @brief   configure STM32 USART2 port
 * -   USART Parameters
 * -   100Kbps
 * -   8-N-1
 * -   DMA Mode
 * @return  None.
 * @note    This code is fully tested on STM32F405RGT6 Platform, You can port it to other platform.
 */

void RC_Init(void) {
/* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    /* -------------- Configure GPIO ---------------------------------------*/
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        USART_DeInit(USART1);
        USART_InitStructure.USART_BaudRate = 100000;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_Even;
        USART_InitStructure.USART_Mode = USART_Mode_Rx;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART1, &USART_InitStructure);
        USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
//			  USART_ClearFlag(USART1, USART_FLAG_IDLE);
//        USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
        USART_Cmd(USART1, ENABLE);
    }

    /* -------------- Configure NVIC  ---------------------------------------*/
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = RC_NVIC;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    /* -------------- Configure DMA -----------------------------------------*/
    {
        DMA_InitTypeDef DMA_InitStructure;
        DMA_DeInit(DMA2_Stream2);
        DMA_InitStructure.DMA_Channel = DMA_Channel_4;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) Remote_RX_Buffer;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_BufferSize = 18;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA2_Stream2, &DMA_InitStructure);
        DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
        DMA_Cmd(DMA2_Stream2, ENABLE);
    }
}


/******************************************************************************  *
 * @brief   USART2 DMA ISR
 * @return  None.
 * @note    This code is fully tested on STM32F405RGT6 Platform, You can port it to other platform.
 */

void DMA2_Stream2_IRQHandler(void) {
    if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2)) {
        DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);

        // Decoding inputs from raw data
        My_Remote.Joystick.Channel_0 = (Remote_RX_Buffer[0] | (Remote_RX_Buffer[1] << 8)) & 0x07ff;             //!< Channel 0
        My_Remote.Joystick.Channel_1 = ((Remote_RX_Buffer[1] >> 3) | (Remote_RX_Buffer[2] << 5)) & 0x07ff;      //!< Channel 1
        My_Remote.Joystick.Channel_2 = ((Remote_RX_Buffer[2] >> 6) | (Remote_RX_Buffer[3] << 2) | (Remote_RX_Buffer[4] << 10)) & 0x07ff;  //!< Channel 2
        My_Remote.Joystick.Channel_3 = ((Remote_RX_Buffer[4] >> 1) | (Remote_RX_Buffer[5] << 7)) & 0x07ff;      //!< Channel 3
        My_Remote.Joystick.Channel_4 = Remote_RX_Buffer[16] | (Remote_RX_Buffer[17] << 8);                      //!< NULL
        My_Remote.Joystick.Switch_0 = ((Remote_RX_Buffer[5] >> 4) & 0x0003);                                    //!< Switch left
        My_Remote.Joystick.Switch_1 = ((Remote_RX_Buffer[5] >> 4) & 0x000C) >> 2;                               //!< Switch right
        My_Remote.Mouse.x = Remote_RX_Buffer[6] | (Remote_RX_Buffer[7] << 8);                                   //!< Mouse X axis
        My_Remote.Mouse.y = Remote_RX_Buffer[8] | (Remote_RX_Buffer[9] << 8);                                   //!< Mouse Y axis
        My_Remote.Mouse.z = Remote_RX_Buffer[10] | (Remote_RX_Buffer[11] << 8);                                 //!< Mouse Z axis
        My_Remote.Mouse.L_Down = Remote_RX_Buffer[12];                                                          //!< Mouse Left Is Press ?
        My_Remote.Mouse.R_Down = Remote_RX_Buffer[13];                                                          //!< Mouse Right Is Press ?
        My_Remote.Keyboard.Key_Value = Remote_RX_Buffer[14] | (Remote_RX_Buffer[15] << 8);                      //!< Keyboard value

        // Adding offsets
        My_Remote.Joystick.Channel_0 -= RC_CH_VALUE_OFFSET;
        My_Remote.Joystick.Channel_1 -= RC_CH_VALUE_OFFSET;
        My_Remote.Joystick.Channel_2 -= RC_CH_VALUE_OFFSET;
        My_Remote.Joystick.Channel_3 -= RC_CH_VALUE_OFFSET;
        My_Remote.Joystick.Channel_4 -= RC_CH_VALUE_OFFSET;
    }

}
