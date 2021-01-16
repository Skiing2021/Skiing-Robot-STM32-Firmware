/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stddef.h"
#include "delay.h"
#include "led.h"
#include "can.h"
#include "dbus_rc.h"
#include "oled.h"
#include "pwm.h"
#include "pid.h"
#include <math.h>
#include "INS_Task.h"
#include "buzzer.h"
#include "control_task.h"

/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern NDJ6_Controller My_Remote;

extern uint8_t MPU6050_Data_Ready;
extern uint8_t Run_Control_Task;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	Delay_Init();
  LED_Init();
	RC_Init();
	//·äÃùÆ÷³õÊ¼»¯
  buzzer_init(30000, 90);
	TIM2_Init(20000, 90);
  TIM4_Init(20000, 90);
  TIM5_Init(20000, 90);
  TIM8_Init(20000, 180);
	//Timer for Control Task, 1ms.
	TIM7_Init();
	
	OLED_Init();
	OLED_ShowString(0,0,(uint8_t*)"Driver Init OK!",16,1);  
	OLED_Refresh();
	
	INS_Init();
	
	Control_Init();
	
	delay_ms(10);
  /* Infinite loop */
  while (1)
  {
		
		if(MPU6050_Data_Ready){
			INS_Task();
			MPU6050_Data_Ready = 0;
		}
		
		if(Run_Control_Task){
			Control_Loop();
			Run_Control_Task = 0;
		}
		
  }
}

//extern int16_t angle, speed;int16_t speed_set;		
//PidTypeDef m2006_pid;
//	CAN_Config();
//	float PID[] = {10, 0, 0};
//	PID_Init(&m2006_pid, PID_POSITION, PID, 10000, 5000);		
//		speed_set = My_Remote.Joystick.Channel_0 * 10;
//		float current_set = PID_Calc(&m2006_pid, speed, speed_set);
//		CAN1_Send_Current(0, 0, current_set, 0);	

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */
