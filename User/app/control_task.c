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
#include "mpu6500driver.h"
#include "usart2.h"

extern NDJ6_Controller My_Remote;

Ski_Control_t ski_control;
uint32_t servo_pulse;

void Control_Init(){
	ski_control.gyro = get_MPU6500_Gyro_Data_Point();
	ski_control.accel = get_MPU6500_Accel_Data_Point();
	
	ski_control.servo[L1].mid = ski_control.servo[L1].set = L1_MID;
	ski_control.servo[L2].mid = ski_control.servo[L2].set = L2_MID;
	ski_control.servo[L3].mid = ski_control.servo[L3].set = L3_MID;
	ski_control.servo[L4].mid = ski_control.servo[L4].set = L4_MID;
	ski_control.servo[L5].mid = ski_control.servo[L5].set = L5_MID;
	ski_control.servo[LARM].mid = ski_control.servo[LARM].set = LARM_MID;
	
	ski_control.servo[R1].mid = ski_control.servo[R1].set = R1_MID;
	ski_control.servo[R2].mid = ski_control.servo[R2].set = R2_MID;
	ski_control.servo[R3].mid = ski_control.servo[R3].set = R3_MID;
	ski_control.servo[R4].mid = ski_control.servo[R4].set = R4_MID;
	ski_control.servo[R5].mid = ski_control.servo[R5].set = R5_MID;
	ski_control.servo[RARM].mid = ski_control.servo[RARM].set = RARM_MID;
	
	ski_control.servo[LT].set = 1000;
	ski_control.servo[RT].set = 1000;
	
	fp32 balance_pid[3] = SKI_BALANCE_PID;
	fp32 gyro_pid[3] = SKI_GYRO_PID;
	PID_Init(&ski_control.ski_balance_pid, PID_POSITION, balance_pid, 10.0f, 0.0f);
	PID_Init(&ski_control.ski_gyro_pid, PID_POSITION, gyro_pid, 10.0f, 0.0f);
}

void Control_Loop(){
	//Set mode
	
	ski_control.ski_last_mode = ski_control.ski_mode;
	if(My_Remote.Joystick.Switch_0 == 1){
		ski_control.ski_mode = SKI_AUTO_MODE;
	} else{
		ski_control.ski_mode = SKI_RC_MODE;
	}
	
	ski_control.ski_yaw_last_mode = ski_control.ski_yaw_mode;
	switch (My_Remote.Joystick.Switch_1){
		case 1:	//Up
			ski_control.ski_yaw_mode = SKI_YAW_PID_GO;
			break;
		case 3:	//Mid
			ski_control.ski_yaw_mode = SKI_YAW_PID_READY;
			break;
		case 2:	//Down
			ski_control.ski_yaw_mode = SKI_YAW_MANUAL;
			break;			
	}
	
	//Feedback update
	ski_control.gravity_angle =  ski_control.accel[2]? atanf(ski_control.accel[0] / ski_control.accel[2]) : 0;
	ski_control.yaw_gyro = ski_control.gyro[2] * 180 / 3.1415926535;
	
	//Set setpoint
	ski_control.gravity_angle_set = 0.0f;
	ski_control.yaw_gyro_set = - My_Remote.Joystick.Channel_0 / 6.0;
	
	//Calculate & control
	ski_control.balance_pid_out = PID_Calc(&ski_control.ski_balance_pid, ski_control.gravity_angle, ski_control.gravity_angle_set);
	ski_control.gyro_pid_out = PID_Calc(&ski_control.ski_gyro_pid, ski_control.yaw_gyro, ski_control.yaw_gyro_set);
	
	if(ski_control.ski_mode == SKI_AUTO_MODE){
		ski_control.feet_pulse_inc = ski_control.yaw_gyro_rx * 2;					 //Range: 256
	} else
	if (ski_control.ski_yaw_mode == SKI_YAW_MANUAL){
		ski_control.feet_pulse_inc = - My_Remote.Joystick.Channel_0 / 4.0; //Range: 165
	} else if(ski_control.ski_yaw_mode == SKI_YAW_PID_READY){
		ski_control.feet_pulse_inc = 0;
	} else if(ski_control.ski_yaw_mode == SKI_YAW_PID_GO){
		ski_control.feet_pulse_inc += ski_control.gyro_pid_out;
	}
	
	ski_control.servo[L1].set += (int16_t)ski_control.balance_pid_out;
	ski_control.servo[R1].set += (int16_t)ski_control.balance_pid_out;
	
	ski_control.servo[L5].set = L5_MID + (int16_t)ski_control.feet_pulse_inc;	
	ski_control.servo[R5].set = R5_MID + (int16_t)ski_control.feet_pulse_inc;

		int16_t left_throttle = 1000;
	int16_t right_throttle = 1000;
	
if(ski_control.ski_mode == SKI_AUTO_MODE){

	left_throttle = 1240;
	right_throttle = 1240;
	
} else{	

	int16_t throttle = My_Remote.Joystick.Channel_3 / 660.0f * 500.0f;
	fp32 throttle_yaw_gain = My_Remote.Joystick.Channel_0 / 660.0f;
	left_throttle = throttle * (1.0 + throttle_yaw_gain);
	right_throttle = throttle * (1.0 - throttle_yaw_gain);
	
	left_throttle += 1000;
	right_throttle += 1000;
	
	if(left_throttle > 1500) left_throttle = 1500;
	if(left_throttle < 1000) left_throttle = 1000;
	
	if(right_throttle > 1500) right_throttle = 1500;
	if(right_throttle < 1000) right_throttle = 1000;
	
}
	
#define MAX_THROTTLE_INC 1
	
	if(left_throttle - ski_control.servo[LT].set > MAX_THROTTLE_INC){
		ski_control.servo[LT].set++;
	} else{
		ski_control.servo[LT].set = left_throttle;
	}
	if(right_throttle - ski_control.servo[RT].set > MAX_THROTTLE_INC){
		ski_control.servo[RT].set++;
	} else{
		ski_control.servo[RT].set = right_throttle;
	}
	
	
//	if(ski_control.servo[L1].mid + (int16_t)ski_control.balance_pid_out - ski_control.servo[L1].set > MAX_PULSE_INCREASEMENT){
//		ski_control.servo[L1].set += MAX_PULSE_INCREASEMENT;
//	}else if(ski_control.servo[L1].mid + (int16_t)ski_control.balance_pid_out - ski_control.servo[L1].set < -MAX_PULSE_INCREASEMENT){
//		ski_control.servo[L1].set -= MAX_PULSE_INCREASEMENT;
//	}else{
//		ski_control.servo[L1].set = ski_control.servo[L1].mid + (int16_t)ski_control.balance_pid_out;
//	}
//	
//	if(ski_control.servo[R1].mid + (int16_t)ski_control.balance_pid_out - ski_control.servo[R1].set > MAX_PULSE_INCREASEMENT){
//		ski_control.servo[R1].set += MAX_PULSE_INCREASEMENT;
//	}else if(ski_control.servo[R1].mid + (int16_t)ski_control.balance_pid_out - ski_control.servo[R1].set < -MAX_PULSE_INCREASEMENT){
//		ski_control.servo[R1].set -= MAX_PULSE_INCREASEMENT;
//	}else{
//		ski_control.servo[R1].set = ski_control.servo[R1].mid + (int16_t)ski_control.balance_pid_out;
//	}
	servo_tester();
	show_debug();
	set_servo_pulse();
}

void servo_tester(){
	ski_control.tester_pulse = 1500 + My_Remote.Joystick.Channel_2 / 660.0f * 1000;
	ski_control.servo[T1].set = ski_control.tester_pulse;
	ski_control.servo[T2].set = ski_control.tester_pulse;
	//ski_control.servo[T3].set = ski_control.tester_pulse;
	//ski_control.servo[T4].set = ski_control.tester_pulse;
}

static void show_debug(){
	static uint8_t oled_cnt = 0;
	if(++oled_cnt % 4 == 0){
		OLED_Clear();
		
		int16_t gyro_line0 = ski_control.gyro[0] * 64;
		int16_t gyro_line1 = ski_control.gyro[1] * 64;
		int16_t gyro_line2 = ski_control.gyro[2] * 64;
		
		LimitRange(gyro_line0, 63, -64)
		LimitRange(gyro_line1, 63, -64)
		LimitRange(gyro_line2, 63, -64)
		
		int16_t accel_x = -ski_control.accel[0] * (float)3.2;
		int16_t accel_y =  ski_control.accel[2] * (float)3.2;
		
		LimitRange(accel_x, 31, -32);
		LimitRange(accel_y, 31, 0);
		
		OLED_DrawLine(64, 18, 64+gyro_line0, 18, 1);
		OLED_DrawLine(64, 22, 64+gyro_line1, 22, 1);
		OLED_DrawLine(64, 26, 64+gyro_line2, 26, 1);
			
		OLED_DrawLine(96, 0, 96 + accel_x, accel_y, 1);
			
		OLED_ShowString(0,0,(uint8_t*)"Pulse:",16,1);
		OLED_ShowNum(48,0,ski_control.tester_pulse,4,16,0);
		OLED_ShowString(0,16,(uint8_t*)"YawSp:",16,1);
		OLED_ShowNum(48,16,ski_control.yaw_gyro,4,16,0);
		OLED_Refresh();
	}//endif
}

/**
	* @brief Set pulse for all servos, STUV->WXYZ->HGFE->DCBA.
	*/
static void set_servo_pulse(){
	
		TIM_SetCompare1(TIM2, ski_control.servo[0].set);
    TIM_SetCompare2(TIM2, ski_control.servo[1].set);
    TIM_SetCompare3(TIM2, ski_control.servo[2].set);
    TIM_SetCompare4(TIM2, ski_control.servo[3].set);

    TIM_SetCompare1(TIM8, ski_control.servo[4].set);
    TIM_SetCompare2(TIM8, ski_control.servo[5].set);
    TIM_SetCompare3(TIM8, ski_control.servo[6].set);
    TIM_SetCompare4(TIM8, ski_control.servo[7].set);

    TIM_SetCompare1(TIM4, ski_control.servo[8].set);
    TIM_SetCompare2(TIM4, ski_control.servo[9].set);
    TIM_SetCompare3(TIM4, ski_control.servo[10].set);
    TIM_SetCompare4(TIM4, ski_control.servo[11].set);

    TIM_SetCompare1(TIM5, ski_control.servo[12].set);
    TIM_SetCompare2(TIM5, ski_control.servo[13].set);
    TIM_SetCompare3(TIM5, ski_control.servo[14].set);
    TIM_SetCompare4(TIM5, ski_control.servo[15].set);
}

//1500 + round(My_Remote.Joystick.Channel_0 / 33.0 * 50);

//extern uint8_t Rxflag;
//extern uint8_t ucTemp;

