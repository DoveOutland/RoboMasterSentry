#include "shoot.h"

Shoot_control_t Shoot_control;

void Snail_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_GPIO_TogglePin(RLED_GPIO_Port, RLED_Pin);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 2000);
    HAL_Delay(2000);
    HAL_GPIO_TogglePin(RLED_GPIO_Port, RLED_Pin);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
    HAL_Delay(2000);
}
void PID_M2006_Init(void)
{
    //位置环
    static const fp32 angle_pid[3] = {0.1, 0, 0};
    PID_init(&Shoot_control.shoot_angle_pid, PID_POSITION, angle_pid, 8000, 3000);

    //速度环
    static const fp32 speed_pid[3] = {5.5, 0.02, 0};
    PID_init(&Shoot_control.shoot_speed_pid, PID_POSITION, speed_pid, 8000, 6000);

    Shoot_control.shoot_flag = 0;
}

void Shoot_task(void)
{
    if(Shoot_control.shoot_flag == 1)
    {
        fp32 shoot_speedset;
        if(TRIGGER_DATA.angle_sum < -1556290)
        {
            TRIGGER_DATA.angle_sum = 0;
        }
        shoot_speedset = PID_calc(&Shoot_control.shoot_angle_pid, TRIGGER_DATA.angle_sum, TRIGGER_DATA.angle_sum - 36859.0);
        Shoot_control.M2006_SEND = PID_calc(&Shoot_control.shoot_speed_pid, TRIGGER_DATA.speed, shoot_speedset);
    }
    else
    {
        fp32 shoot_speedset;
        shoot_speedset = PID_calc(&Shoot_control.shoot_angle_pid, TRIGGER_DATA.angle_sum, TRIGGER_DATA.angle_sum);
        Shoot_control.M2006_SEND = PID_calc(&Shoot_control.shoot_speed_pid, TRIGGER_DATA.speed, shoot_speedset);
    }

}
void Fire_task(uint16_t fire_power)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1100 + fire_power);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1200 + fire_power);
}
