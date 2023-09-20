#include "Gimbal_task.h"
#include "usart.h"

#define CONSTRAIN(x, min, max)	((x>max)?max:(x<min?min:x))
#define YAW_MAX        -3000  //1500
#define YAW_MIN        -5000  //-1300
#define PITCH_MAX      3800
#define PITCH_MIN      3000
#define YAW_SPEED      20
#define PITCH_SPEED    10

uint8_t GimbalRxBuffer[13] = { 0 };
uint16_t GimbalRealBuf[7] = { 0 };
uint8_t aRxBuffer;
uint8_t Uart1_Rx_Cnt = 0;

Gimbal_control_t Gimbal_control;
extKalman_t RC_yaw_p, RC_pitch_p;

void Gimbal_Kalman_Init(void)
{
    KalmanCreate(&RC_pitch_p, 1, 2);
    KalmanCreate(&RC_yaw_p, 1, 2);
}

void PID_Gimbal_Init(void)
{
    static const fp32 vison_ang_pid[3] = { 0.31, 0.0, 0.0 };//0.011, 0.0, 0.005
    PID_init(&Gimbal_control.vison_angle_pid, PID_POSITION, vison_ang_pid, 1500, 50);

    //yaw
    static const fp32 yaw_ang_pid[3] = { 10.0, 0.0, 0.0 };
    PID_init(&Gimbal_control.yaw_angle_pid, PID_POSITION, yaw_ang_pid, 16000, 8000);
    static const fp32 Yaw_speed_pid[3] = { 18.0, 0.1, 0.0 };
    PID_init(&Gimbal_control.yaw_speed_pid, PID_POSITION, Yaw_speed_pid, 25000, 5000);

    //pitch
    static const fp32 pitch_ang_pid[3] = { 15.0, 0.0, 0.0 };
    PID_init(&Gimbal_control.pitch_angle_pid, PID_POSITION, pitch_ang_pid, 16000, 8000);
    static const fp32 Pitch_speed_pid[3] = { 12.0, 0.3, 0.0 };
    PID_init(&Gimbal_control.pitch_speed_pid, PID_POSITION, Pitch_speed_pid, 28000, 8000);

    Gimbal_control.init_yaw = YAW_6020_DATA.angle_sum;
}

void Gimbal_Task(uint16_t GimbalBuf[])
{
    static fp32 yaw_taget, pitch_taget = 0;
    static uint8_t yaw_dir = 0, pitch_dir = 1;

    fp32 yaw_speedset = 0, pitch_speedset = 0;

    if(PITCH_6020_DATA.angle_sum != 0)
    {
        if(GimbalRealBuf[0] == 1)
        {
            //yaw
            if(GimbalBuf[2] == 0)
            {
                yaw_speedset = PID_calc(&Gimbal_control.vison_angle_pid, GimbalRealBuf[3], 0.0);
                yaw_speedset = yaw_speedset;
            }
            else
            {
                yaw_speedset = PID_calc(&Gimbal_control.vison_angle_pid, GimbalRealBuf[3], 0.0);
                yaw_speedset = -yaw_speedset;
            }
            //pitch
            if(GimbalBuf[4] == 0)
            {
                pitch_speedset = PID_calc(&Gimbal_control.vison_angle_pid, GimbalRealBuf[5], 0.0);
                pitch_speedset = pitch_speedset / 1.8;
            }
            else
            {
                pitch_speedset = PID_calc(&Gimbal_control.vison_angle_pid, GimbalRealBuf[5], 0.0);
                pitch_speedset = -pitch_speedset / 1.8;
            }

            if(GimbalBuf[3] < 100 && GimbalBuf[5] < 100)
            {
                Shoot_control.shoot_flag = 1;
            }
            else
            {
                Shoot_control.shoot_flag = 0;
            }
            Gimbal_control.Yaw_Send = PID_calc(&Gimbal_control.yaw_speed_pid, YAW_6020_DATA.speed, yaw_speedset);
            Gimbal_control.Pitch_Send = PID_calc(&Gimbal_control.pitch_speed_pid, PITCH_6020_DATA.speed, pitch_speedset);
        }
        else
        {
            yaw_taget = YAW_6020_DATA.angle_sum;
            pitch_taget = PITCH_6020_DATA.angle_sum;
            Shoot_control.shoot_flag = 0;
            //yaw_taget测试 2700-5600
            if(yaw_dir == 1)//向右侦察
            {
                yaw_taget = YAW_6020_DATA.angle_sum + YAW_SPEED;
                if(YAW_6020_DATA.angle_sum > YAW_MAX)
                {
                    yaw_taget = YAW_MAX;
                    yaw_dir = 0;
                }
            }
            else if(yaw_dir == 0)  //向左侦察
            {
                yaw_taget = YAW_6020_DATA.angle_sum - YAW_SPEED;
                if(YAW_6020_DATA.angle_sum < YAW_MIN)
                {
                    yaw_taget = YAW_MIN;
                    yaw_dir = 1;
                }
            }
            //pitch_taget测试:3000-3700之间/联盟赛:14400-16800
            if(pitch_dir == 1)//向右侦察
            {
                pitch_taget = PITCH_6020_DATA.angle_sum + PITCH_SPEED;
                if(PITCH_6020_DATA.angle_sum > PITCH_MAX)
                {
                    pitch_taget = PITCH_MAX;
                    pitch_dir = 0;
                }
            }
            else if(pitch_dir == 0)  //向左侦察
            {
                pitch_taget = PITCH_6020_DATA.angle_sum - PITCH_SPEED;
                if(PITCH_6020_DATA.angle_sum < PITCH_MIN)
                {
                    pitch_taget = PITCH_MIN;
                    pitch_dir = 1;
                }
            }
            yaw_taget = CONSTRAIN(yaw_taget, YAW_MIN, YAW_MAX);
            pitch_taget = CONSTRAIN(pitch_taget, PITCH_MIN, PITCH_MAX);
            yaw_speedset = PID_calc(&Gimbal_control.yaw_angle_pid, YAW_6020_DATA.angle_sum, yaw_taget);
            pitch_speedset = PID_calc(&Gimbal_control.pitch_angle_pid, PITCH_6020_DATA.angle_sum, pitch_taget);
            Gimbal_control.Yaw_Send = PID_calc(&Gimbal_control.yaw_speed_pid, YAW_6020_DATA.speed, yaw_speedset);
            Gimbal_control.Pitch_Send = PID_calc(&Gimbal_control.pitch_speed_pid, PITCH_6020_DATA.speed, pitch_speedset);
        }
    }
}

void Gimbal_RC_Task(void)
{
    float yaw_speed = 0, pitch_speed = 0;
    yaw_speed = rc_ctrl.rc.ch[0] * 1.5;
    pitch_speed = rc_ctrl.rc.ch[1] * 1.5;

    Gimbal_control.Yaw_Send = PID_calc(&Gimbal_control.yaw_speed_pid, YAW_6020_DATA.speed, yaw_speed);
    Gimbal_control.Pitch_Send = PID_calc(&Gimbal_control.pitch_speed_pid, PITCH_6020_DATA.speed, pitch_speed);
}
