#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "main.h"
#include "pid.h"

#define L1 0
#define L2 1
#define L3 2
#define L4 3
#define L5 4
#define LARM 5
#define LT 6

#define R1 8
#define R2 9
#define R3 10
#define R4 11
#define R5 12
#define RARM 13
#define RT 14

#define T1 7
#define T2 15
//#define T3 14
//#define T4 15

#define L1_MID 1840
#define L2_MID 1700
#define L3_MID 750
#define L4_MID 1250
#define L5_MID 1770
#define LARM_MID 1500

#define R1_MID 1380
#define R2_MID 1300
#define R3_MID 2000
#define R4_MID 1750
#define R5_MID 1220
#define RARM_MID 1500

#define SKI_BALANCE_PID {20.0f, 0, 0}
#define SKI_GYRO_PID {0.1f, 0, 0}

#define MAX_PULSE_INCREASEMENT 5

typedef struct{
	uint16_t mid;
	uint16_t set;
}Servo_t;

typedef enum{
	SKI_RAW_MODE = 0,
	SKI_RC_MODE,
	SKI_AUTO_MODE,
}Ski_Mode_e;

typedef enum{
	SKI_YAW_MANUAL = 0,
	SKI_YAW_PID_READY,
	SKI_YAW_PID_GO,
}Ski_Yaw_Mode_e;

typedef struct{
	Ski_Mode_e ski_mode, ski_last_mode;
	Ski_Yaw_Mode_e ski_yaw_mode, ski_yaw_last_mode;
	const fp32* gyro;
	const fp32* accel;
	Servo_t servo[17];
	fp32 gravity_angle;
	fp32 gravity_angle_set;
	fp32 balance_pid_out;
	fp32 yaw_gyro;
	fp32 yaw_gyro_set;
	fp32 yaw_gyro_rx;
	fp32 gyro_pid_out;
	fp32 feet_pulse_inc;
	PidTypeDef ski_balance_pid;
	PidTypeDef ski_gyro_pid;
	uint16_t tester_pulse;
}Ski_Control_t;


void Control_Init(void);
void Control_Loop(void);
static void show_debug(void);
static void set_servo_pulse(void);
void servo_tester(void);

#define LimitRange(input, max, min) \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < min) \
        {                      \
            input = min;      \
        }                      \
    }

#endif
