/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int snail_flag = 1;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_USART6_UART_Init();
    MX_USART1_UART_Init();
    MX_TIM6_Init();
    MX_USART3_UART_Init();
    MX_TIM1_Init();
    /* USER CODE BEGIN 2 */

    //遥控器
    remote_control_init();

    //自瞄,裁判系统开启
    Judge_Init_DMA();

    //PID初始化
    PID_3508_Init();
    PID_Gimbal_Init();
    PID_M2006_Init();

    //滤波初始化
    Gimbal_Kalman_Init();
    Kalman_3508_Init();

    //CAN开启
    can_filter_init();

    //任务运行
    HAL_TIM_Base_Start_IT(&htim6);

    //摩擦轮测试
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    //OFF fric
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
    HAL_Delay(100);
    while(rc_ctrl.rc.ch[3] < 600)//左y杆不动，右y杆向上打
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2000);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 2000);
        if(rc_ctrl.rc.ch[1] > 600)
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
            break;
        }
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while(1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        Fire_task(50);
        HAL_Delay(100);
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    static int TaskLab = 0;

    if(htim == (&htim6))
    {
        TaskLab ++;

        /* Task 1     10ms*/
        if(TaskLab % 2 == 0)
        {
            if(rc_ctrl.rc.s[0] == RC_SW_UP)
            {
//                Shoot_task();
//                Fire_task(65);
                Gimbal_Task(GimbalRealBuf);
                CAN_cmd_gimbal(Gimbal_control.Yaw_Send, Gimbal_control.Pitch_Send, 0, 0); //电机赋值
                Chassis_task();
            }
            if(rc_ctrl.rc.s[0] == RC_SW_MID)
            {
                Fire_task(65);
                Gimbal_RC_Task();
                Shoot_control.M2006_SEND = PID_calc(&Shoot_control.shoot_speed_pid, TRIGGER_DATA.speed, -1000);
                CAN_cmd_gimbal(Gimbal_control.Yaw_Send, Gimbal_control.Pitch_Send, Shoot_control.M2006_SEND, 0);
                Chassis_RC_Task();
            }
            if(rc_ctrl.rc.s[0] == RC_SW_DOWN)
            {
                PID_clear(&Gimbal_control.vison_angle_pid);
                PID_clear(&Gimbal_control.yaw_angle_pid);
                PID_clear(&Gimbal_control.yaw_speed_pid);
                PID_clear(&Gimbal_control.pitch_angle_pid);
                PID_clear(&Gimbal_control.pitch_speed_pid);
                CAN_cmd_gimbal(0, 0, 0, 0);
                CAN_cmd_chassis(0, 0, 0, 0);
            }
        }

        /* Task 2    500ms*/
        if(TaskLab % 100 == 0) //
        {
            HAL_GPIO_TogglePin(BLED_GPIO_Port, BLED_Pin);

            TaskLab = 0;
        }
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while(1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

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
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
