#ifndef _GIMBAL_TASK_
#define _GIMBAL_TASK_

#include "main.h"
#include <stdbool.h>
#include "kalman.h"

extern uint8_t GimbalRxBuffer[13];
extern uint16_t GimbalRealBuf[7];
extern uint8_t aRxBuffer;
extern uint8_t Uart1_Rx_Cnt;

typedef struct
{
		pid_type_def vison_angle_pid;
	
    pid_type_def yaw_angle_pid;
    pid_type_def pitch_angle_pid;
	
    pid_type_def yaw_speed_pid;
    pid_type_def pitch_speed_pid;

    fp32 Yaw_Send;
    fp32 Pitch_Send;

	  int32_t init_yaw;
    //方向
    int8_t Scout_direction;

} Gimbal_control_t;
extern Gimbal_control_t Gimbal_control;
extern extKalman_t RC_yaw_p, RC_pitch_p;

void PID_Gimbal_Init(void);
void Gimbal_Kalman_Init(void);
void Gimbal_Task(uint16_t GimbalBuf[]);
void Gimbal_RC_Task(void);

#endif
