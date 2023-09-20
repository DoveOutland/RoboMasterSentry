#ifndef _CHASSIS_TASK_
#define _CHASSIS_TASK_

#include "main.h"
#include <stdbool.h>
#include "kalman.h"

#define POWERBUFF_HOT    60.F
#define POWERLIMIT_NORMAL 12000U

typedef struct
{
    pid_type_def chassis_angle_pid;
    pid_type_def chassis_speed_pid;

    fp32 Chassis_Send[2]; 

    bool overbuff_flag;

} Chassis_control_t;
extern Chassis_control_t Chassis_control;
extern extKalman_t Chassis_p;
extern first_order_filter_type_t chassis_slow_set;

void PID_3508_Init(void);
void Kalman_3508_Init(void);
void First_Order_Init(void);
void Chassis_task(void);
void Chassis_RC_Task(void);
void Chassis_Power_Control(void);

#endif
