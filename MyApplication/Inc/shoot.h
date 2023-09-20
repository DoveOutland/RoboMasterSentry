#ifndef _SHOOT_TASK_
#define _SHOOT_TASK_

#include "main.h"
#include "tim.h"

typedef struct
{
    pid_type_def shoot_speed_pid;
    pid_type_def shoot_angle_pid;

    fp32 M2006_SEND;

    int shoot_flag;

} Shoot_control_t;
extern Shoot_control_t Shoot_control;

void Snail_Init(void);
void PID_M2006_Init(void);
void Shoot_task(void);
void Fire_task(uint16_t fire_power);

#endif
