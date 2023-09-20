#include "Chassis_task.h"
#include <math.h>

#define CONSTRAIN(x, min, max)	((x>max)?max:(x<min?min:x))
#define PATH_MAX        1400000  //1432300 总轨道///181910
#define PATH_MIN        1000
#define CHASSIS_SPEED   6500

Chassis_control_t Chassis_control;
extKalman_t Chassis_p;
first_order_filter_type_t chassis_slow_set;

const static fp32 chassis_order_filter[1] = { 10.0f };
float K_Limit = 1;

void PID_3508_Init(void)
{
    static const float chassis_angle_pid[3] = {0.4, 0, 0};
    PID_init(&Chassis_control.chassis_angle_pid, PID_POSITION, chassis_angle_pid, 5000, 1600);

    static const float chassis_speed_pid[3] = {10, 0.02, 0};
    PID_init(&Chassis_control.chassis_speed_pid, PID_POSITION, chassis_speed_pid, 12000, 8000);
}

void Kalman_3508_Init(void)
{
    KalmanCreate(&Chassis_p, 0.1, 0.50);
}

void First_Order_Init(void)
{
    first_order_filter_init(&chassis_slow_set, 0.01, chassis_order_filter);
}

void Chassis_task(void)
{
    static float chassis_taget = 0;
    static uint8_t chassis_dir = 1;
    static double sin_rand = 0;

    float chassis_speedset = 0;
    int rand_num;

    sin_rand = sin_rand + 0.001;
    rand_num = 50 * sin(sin_rand);

    if(PITCH_6020_DATA.angle_sum != 0)
    {
        if(chassis_dir == 1)
        {
            chassis_taget = M3508_Receive[0].angle_sum + CHASSIS_SPEED + abs(rand_num);
            if(chassis_taget > PATH_MAX)
            {
                chassis_taget = PATH_MAX;
                chassis_dir = 0;
            }
        }
        else if(chassis_dir == 0)
        {
            chassis_taget = M3508_Receive[0].angle_sum - CHASSIS_SPEED - abs(rand_num);
            if(chassis_taget < PATH_MIN)
            {
                chassis_taget = PATH_MIN;
                chassis_dir = 1;
            }
        }
        chassis_taget = CONSTRAIN(chassis_taget, PATH_MIN, PATH_MAX);
        chassis_speedset = PID_calc(&Chassis_control.chassis_angle_pid, M3508_Receive[0].angle_sum, chassis_taget);
        Chassis_control.Chassis_Send[0] = PID_calc(&Chassis_control.chassis_speed_pid, M3508_Receive[0].speed, chassis_speedset);
        Chassis_control.Chassis_Send[1] = PID_calc(&Chassis_control.chassis_speed_pid, M3508_Receive[1].speed, chassis_speedset);
        Chassis_Power_Control();
        CAN_cmd_chassis(Chassis_control.Chassis_Send[0], Chassis_control.Chassis_Send[1], 0, 0);
    }
}

void Chassis_RC_Task(void)
{
    float chassis_speedset = 0;
    chassis_speedset = rc_ctrl.rc.ch[2] * 1.5;

    Chassis_control.Chassis_Send[0] = PID_calc(&Chassis_control.chassis_speed_pid, M3508_Receive[0].speed, chassis_speedset);
    Chassis_control.Chassis_Send[1] = PID_calc(&Chassis_control.chassis_speed_pid, M3508_Receive[1].speed, chassis_speedset);
    Chassis_Power_Control();
    CAN_cmd_chassis(Chassis_control.Chassis_Send[0], Chassis_control.Chassis_Send[1], 0, 0);
}

void Chassis_Power_Control(void)
{
    static int16_t Cold_time = 0;

    if(judge_sensor.info->PowerHeatData.chassis_power_buffer < POWERBUFF_HOT)
    {
        Cold_time = 0;
        Chassis_control.overbuff_flag = true;
        K_Limit = (float)(judge_sensor.info->PowerHeatData.chassis_power_buffer / POWERBUFF_HOT);
        Chassis_control.chassis_speed_pid.max_out = POWERLIMIT_NORMAL * K_Limit * K_Limit;
    }
    else
    {
        Cold_time++;
        if(Cold_time >= 100)
        {
            Cold_time = 0;
            Chassis_control.overbuff_flag = false;
            Chassis_control.chassis_speed_pid.max_out = POWERLIMIT_NORMAL;
            K_Limit = 1;
        }
    }
}
