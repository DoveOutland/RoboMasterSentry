#include "CAN_receive.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

#define abs(x) 					((x)>0? (x):(-(x)))

static motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

motor_DATA_t YAW_6020_DATA;
motor_DATA_t PITCH_6020_DATA;
motor_DATA_t M3508_Receive[4];
motor_DATA_t TRIGGER_DATA;

static void motor_check(motor_DATA_t* motor)
{
    int16_t err;

//    if(!motor->init_flag)
//    {
//        motor->init_flag = 1;
//        motor->angle_prev = motor->angel;
//        motor->angle_sum = 0;
//    }

    err = motor->angel - motor->angle_prev;

    if(abs(err) > 4095)
    {
        if(err >= 0)
        {
            motor->angle_sum += -8191 + err;
        }
        else
        {
            motor->angle_sum += 8191 + err;
        }
    }
    else
    {
        motor->angle_sum += err;
    }

    motor->angle_prev = motor->angel;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch(rx_header.StdId)
    {
    case CAN_3508_M1_ID:
        M3508_Receive[0].angel = rx_data[0] << 8 | rx_data[1];
        M3508_Receive[0].speed = rx_data[2] << 8 | rx_data[3];
        M3508_Receive[0].current = rx_data[4] << 8 | rx_data[5];
        motor_check(&M3508_Receive[0]);
        break;

    case CAN_3508_M2_ID:
        M3508_Receive[1].angel = rx_data[0] << 8 | rx_data[1];
        M3508_Receive[1].speed = rx_data[2] << 8 | rx_data[3];
        M3508_Receive[1].current = rx_data[4] << 8 | rx_data[5];
				motor_check(&M3508_Receive[1]);
        break;

    case CAN_3508_M3_ID:
        M3508_Receive[2].angel = rx_data[0] << 8 | rx_data[1];
        M3508_Receive[2].speed = rx_data[2] << 8 | rx_data[3];
        M3508_Receive[2].current = rx_data[4] << 8 | rx_data[5];
        break;

    case CAN_3508_M4_ID:
        M3508_Receive[3].angel = rx_data[0] << 8 | rx_data[1];
        M3508_Receive[3].speed = rx_data[2] << 8 | rx_data[3];
        M3508_Receive[3].current = rx_data[4] << 8 | rx_data[5];
        break;

    case CAN_YAW_MOTOR_ID:
        YAW_6020_DATA.angel = rx_data[0] << 8 | rx_data[1];
        YAW_6020_DATA.speed = rx_data[2] << 8 | rx_data[3];
        YAW_6020_DATA.current = rx_data[4] << 8 | rx_data[5];
        motor_check(&YAW_6020_DATA);
        break;

    case CAN_PITCH_MOTOR_ID:
        PITCH_6020_DATA.angel = rx_data[0] << 8 | rx_data[1];
        PITCH_6020_DATA.speed = rx_data[2] << 8 | rx_data[3];
        PITCH_6020_DATA.current = rx_data[4] << 8 | rx_data[5];
        motor_check(&PITCH_6020_DATA);
        break;

    case CAN_TRIGGER_MOTOR_ID:
        TRIGGER_DATA.angel = rx_data[0] << 8 | rx_data[1];
        TRIGGER_DATA.speed = rx_data[2] << 8 | rx_data[3];
        TRIGGER_DATA.current = rx_data[4] << 8 | rx_data[5];
        motor_check(&TRIGGER_DATA);
        break;

    default:
    {
        break;
    }
    }
}

void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &gimbal_tx_message, gimbal_can_send_data,
                         &send_mail_box);
}

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3,
                     int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message,
                         chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message,
                         chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t* get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}
/**
 * @brief          返回pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t* get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}
/**
 * @brief          返回拨弹电机 2006电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t* get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}
/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t* get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;

}
/**
 * 任意电机测试函数
 * Value       : 电机直接赋值
 * SEND_ID     : 电机发送ID		0x1ff / 0x200
 * MOTOR_ID    : 电机ID   		范围：1-8
 * CAN_ID      : CAN编号   		1 / 2
 */
void CAN_MOTOR_CHECK(int16_t Value, int16_t SEND_ID, int16_t MOTOR_ID,
                     int CAN_ID)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = SEND_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    if(MOTOR_ID == 1 || MOTOR_ID == 5)
    {
        chassis_can_send_data[0] = Value >> 8;
        chassis_can_send_data[1] = Value;
    }
    else if(MOTOR_ID == 2 || MOTOR_ID == 6)
    {
        chassis_can_send_data[2] = Value >> 8;
        chassis_can_send_data[3] = Value;
    }
    else if(MOTOR_ID == 3 || MOTOR_ID == 7)
    {
        chassis_can_send_data[4] = Value >> 8;
        chassis_can_send_data[5] = Value;
    }
    else if(MOTOR_ID == 4 || MOTOR_ID == 8)
    {
        chassis_can_send_data[6] = Value >> 8;
        chassis_can_send_data[7] = Value;
    }

    if(CAN_ID == 1)
    {
        HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data,
                             &send_mail_box);
    }
    else if(CAN_ID == 2)
    {
//		HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data,
//				&send_mail_box);
    }
    else
    {
        HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data,
                             &send_mail_box);
    }
}






