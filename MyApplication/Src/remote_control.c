#include "remote_control.h"
#include "main.h"
#include <string.h>

//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;


//取正函数
static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t* sbus_buf, RC_ctrl_t* rc_ctrl);

//remote control data
//遥控器控制变量
RC_ctrl_t rc_ctrl;
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t* get_remote_control_point(void)
{
    return &rc_ctrl;
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if(RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if(RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if(RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if(RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if(rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if(rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

//串口中断
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);

                //记录数据接收时间
//                detect_hook(DBUS_TOE);
            }
        }
        else
        {
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                //记录数据接收时间
//                detect_hook(DBUS_TOE);
            }
        }
    }
}

//取正函数
static int16_t RC_abs(int16_t value)
{
    if(value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */

static void sbus_to_rc(volatile const uint8_t* sbus_buf, RC_ctrl_t* rc_ctrl)
{
    if(sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

/* ----------------------- Rc Sensor-------------------------------- */
/* ----------------------- Rc Sensor-------------------------------- */
/* ----------------------- Rc Sensor-------------------------------- */

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void rc_sensor_check(rc_sensor_t* rc_sen);
static void rc_sensor_heart_beat(rc_sensor_t* rc_sen);
// 遥控器信息
rc_sensor_info_t 	rc_sensor_info =
{
    .offline_max_cnt = 60,
};

// 遥控器传感器
rc_sensor_t	rc_sensor =
{
    .info = &rc_sensor_info,
    .check = rc_sensor_check,
    .heart_beat = rc_sensor_heart_beat,
};

/**
 *	@brief	遥控器数据检查
 */
static void rc_sensor_check(rc_sensor_t* rc_sen)
{
    rc_sensor_info_t* rc_info = rc_sen->info;
    if(!rc_info->init_flag)
    {
        rc_info->init_flag = true;
        rc_info->s1_pre = rc_info->s1;
        rc_info->s2_pre = rc_info->s2;
    }

    if(abs(rc_info->ch0) > 660 ||
            abs(rc_info->ch1) > 660 ||
            abs(rc_info->ch2) > 660 ||
            abs(rc_info->ch3) > 660)
    {
        rc_info->ch0 = 0;
        rc_info->ch1 = 0;
        rc_info->ch2 = 0;
        rc_info->ch3 = 0;
        rc_info->s1 = RC_SW_MID;
        rc_info->s2 = RC_SW_MID;
        rc_info->thumbwheel = 0;
    }
    else
    {
    }
    /*用完记得清0*/
    if(rc_info->s1_pre != rc_info->s1) //s1切换记录
    {
        switch(rc_info->s1)
        {
        case RC_SW_UP:
            if(rc_info->s1_pre == RC_SW_MID)
                rc_info->s1_siwtch_up = true;
            break;
        case RC_SW_MID:
            if(rc_info->s1_pre == RC_SW_UP)
                rc_info->s1_switch_uptomid = true;
            if(rc_info->s1_pre == RC_SW_DOWN)
                rc_info->s1_switch_downtomid = true;
            break;
        case RC_SW_DOWN:
            if(rc_info->s1_pre == RC_SW_MID)
                rc_info->s1_siwtch_down = true;
            break;
        default:
            break;
        }
    }
    if(rc_info->s2_pre != rc_info->s2) //s2切换记录
    {
        switch(rc_info->s2)
        {
        case RC_SW_UP:
            if(rc_info->s2_pre == RC_SW_MID)
                rc_info->s2_siwtch_up = true;
            break;
        case RC_SW_MID:
            if(rc_info->s2_pre == RC_SW_UP)
                rc_info->s2_switch_uptomid = true;
            if(rc_info->s2_pre == RC_SW_DOWN)
                rc_info->s2_switch_downtomid = true;
            break;
        case RC_SW_DOWN:
            if(rc_info->s2_pre == RC_SW_MID)
                rc_info->s2_siwtch_down = true;
            break;
        default:
            break;
        }
    }
    /****************/
    rc_info->s1_pre = rc_info->s1;
    rc_info->s2_pre = rc_info->s2;	//记录上次的值
}

/**
 *	@brief	遥控器心跳包
 */
static void rc_sensor_heart_beat(rc_sensor_t* rc_sen)
{
    rc_sensor_info_t* rc_info = rc_sen->info;

    rc_info->offline_cnt++;
    if(rc_info->offline_cnt > rc_info->offline_max_cnt)
    {
        rc_info->offline_cnt = rc_info->offline_max_cnt;
        rc_info->init_flag = false;//需要重新初始化
    }
    else
    {
        /* 离线->在线 */
    }
}

float DeathZoom(float input, float center, float death)
{
    if(abs(input - center) < death)
        return center;
    return input;
}

bool RC_IsChannelReset(void)
{
    if((DeathZoom(rc_sensor_info.ch0, 0, 50) == 0) &&
            (DeathZoom(rc_sensor_info.ch1, 0, 50) == 0) &&
            (DeathZoom(rc_sensor_info.ch2, 0, 50) == 0) &&
            (DeathZoom(rc_sensor_info.ch3, 0, 50) == 0))
    {
        return true;
    }
    return false;
}

void RC_ResetData(rc_sensor_t* rc)
{
    // 通道值强行设置成中间值(不拨动摇杆的状态)
    rc->info->ch0 = 0;
    rc->info->ch1 = 0;
    rc->info->ch2 = 0;
    rc->info->ch3 = 0;
    // 左右开关选择强行设置成中间值状态
    rc->info->s1 = RC_SW_MID;
    rc->info->s2 = RC_SW_MID;
    // 鼠标
    rc->info->mouse_vx = 0;
    rc->info->mouse_vy = 0;
    rc->info->mouse_vz = 0;
    rc->info->mouse_btn_l = 0;
    rc->info->mouse_btn_r = 0;
    // 键盘
    rc->info->key_v = 0;
    // 左拨轮
    rc->info->thumbwheel = 0;
}
