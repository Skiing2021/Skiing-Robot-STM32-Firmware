//
// Created by caesarw on 11/14/20.
//

#ifndef BUTTON_LIGHT_STM32F4VE_DBUS_RC_H
#define BUTTON_LIGHT_STM32F4VE_DBUS_RC_H

#include "stm32f4xx.h"

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN              ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET           ((uint16_t)1024)
#define RC_CH_VALUE_MAX              ((uint16_t)1684)
/* ************************************************************************* */


/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                     ((uint16_t)1)
#define RC_SW_MID                    ((uint16_t)3)
#define RC_SW_DOWN                   ((uint16_t)2)
/* ************************************************************************* */


/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W         ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S         ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A         ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D         ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q         ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E         ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT     ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL      ((uint16_t)0x01<<7)
/* ************************************************************************* */


/* ----------------------- Data Struct ------------------------------------- */
typedef struct {
    struct {
        int16_t Channel_0;
        int16_t Channel_1;
        int16_t Channel_2;
        int16_t Channel_3;
        int16_t Channel_4;
        uint8_t Switch_0;
        uint8_t Switch_1;
    } Joystick;
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t L_Down;
        uint8_t R_Down;
    } Mouse;
    struct {
        uint16_t Key_Value;
    } Keyboard;
} NDJ6_Controller;

/* ************************************************************************* */

void RC_Init(void);

void DMA2_Stream2_IRQHandler(void);

#endif //BUTTON_LIGHT_STM32F4VE_DBUS_RC_H

// Reference Data Packet Struct
//#pragma pack(1)
//typedef union {
//    struct {
//        struct {
//            uint8_t ch0_h: 8;             //!< Byte 0
//
//            uint8_t ch0_l: 3;             //!< Byte 1
//            uint8_t ch1_h: 5;
//
//            uint8_t ch1_l: 6;             //!< Byte 2
//            uint8_t ch2_h: 2;
//
//            uint8_t ch2_m: 8;             //!< Byte 3
//
//            uint8_t ch2_l: 1;             //!< Byte 4
//            uint8_t ch3_h: 7;
//
//            uint8_t ch3_l: 4;             //!< Byte 5
//            uint8_t Switch_0: 2;
//            uint8_t Switch_1: 2;
//        } Joystick;
//
//        struct {
//            int16_t x;                    //!< Byte 6-7
//            int16_t y;                    //!< Byte 8-9
//            int16_t z;                    //!< Byte 10-11
//            uint8_t L_Down;              //!< Byte 12
//            uint8_t R_Down;              //!< Byte 13
//        } Mouse;
//
//        struct {
//            uint16_t Key_Value;                   //!< Byte 14-15
//        } Keyboard;
//
//        uint16_t resv;                    //!< Byte 16-17
//    };
//    uint8_t buf[18];                      //!< Union --> Byte<0-17>
//} RC_Ctl_Define_t;
