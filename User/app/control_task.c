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
	
	ski_control.servo[R1].mid = ski_control.servo[R1].set = R1_MID;
	ski_control.servo[R2].mid = ski_control.servo[R2].set = R2_MID;
	ski_control.servo[R3].mid = ski_control.servo[R3].set = R3_MID;
	ski_control.servo[R4].mid = ski_control.servo[R4].set = R4_MID;
	ski_control.servo[R5].mid = ski_control.servo[R5].set = R5_MID;
	
	static fp32 balance_pid[3] = SKI_BALANCE_PID;
	PID_Init(&ski_control.ski_balance_pid, PID_POSITION, balance_pid, 10.0f, 0.0f);
}

void Control_Loop(){
	//Feedback update
	ski_control.gravity_angle =  ski_control.accel[2]? atanf(ski_control.accel[0] / ski_control.accel[2]) : 0;
	ski_control.yaw_gyro = ski_control.gyro[2];
	
	//Set setpoint
	ski_control.gravity_angle_set = 0.0f;
	ski_control.yaw_gyro_set = 0.0f;
	
	ski_control.balance_pid_out = PID_Calc(&ski_control.ski_balance_pid, ski_control.gravity_angle, ski_control.gravity_angle_set);
	
	ski_control.servo[L1].set += (int16_t)ski_control.balance_pid_out;
	ski_control.servo[R1].set += (int16_t)ski_control.balance_pid_out;
	
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
	
	show_debug();
	set_servo_pulse();
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
		//OLED_ShowNum(48,0,servo_pulse,4,16,0);
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
