#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"
#include <stdbool.h>
#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/*
rc_ctrl.rc.ch[x]: -660——660

					3															1
					|                   					|
					|  +                       		|  +
    ------o------	2			          ------o------ 0
			-		|                          -	|
					|                         		|
*/
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */
typedef   struct
{
    struct
    {
        int16_t ch[5];
        char s[2];
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    struct
    {
        uint16_t v;
    } key;

} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

extern void remote_control_init(void);
extern const RC_ctrl_t* get_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
extern void sbus_to_usart1(uint8_t* sbus);

extern RC_ctrl_t rc_ctrl;




/* ----------------------- Rc Sensor-------------------------------- */
/* ----------------------- Rc Sensor-------------------------------- */
/* ----------------------- Rc Sensor-------------------------------- */
/* ----------------------- Function Definition-------------------------------- */
/* 遥控摇杆通道偏移值 */
#define		RC_SW1_VALUE				(rc_sensor_info.s1)
#define		RC_SW2_VALUE				(rc_sensor_info.s2)
#define		RC_LEFT_CH_LR_VALUE			(rc_sensor_info.ch2)
#define		RC_LEFT_CH_UD_VALUE			(rc_sensor_info.ch3)
#define		RC_RIGH_CH_LR_VALUE			(rc_sensor_info.ch0)
#define		RC_RIGH_CH_UD_VALUE			(rc_sensor_info.ch1)
#define		RC_THUMB_WHEEL_VALUE		(rc_sensor_info.thumbwheel)

/* 检测遥控器开关状态 */
#define    IF_RC_SW1_UP      (rc_sensor_info.s1 == RC_SW_UP)
#define    IF_RC_SW1_MID     (rc_sensor_info.s1 == RC_SW_MID)
#define    IF_RC_SW1_DOWN    (rc_sensor_info.s1 == RC_SW_DOWN)
#define    IF_RC_SW2_UP      (rc_sensor_info.s2 == RC_SW_UP)
#define    IF_RC_SW2_MID     (rc_sensor_info.s2 == RC_SW_MID)
#define    IF_RC_SW2_DOWN    (rc_sensor_info.s2 == RC_SW_DOWN)

/* 获取鼠标三轴的移动速度 */
#define    MOUSE_X_MOVE_SPEED    (rc_sensor_info.mouse_vx)
#define    MOUSE_Y_MOVE_SPEED    (rc_sensor_info.mouse_vy)
#define    MOUSE_Z_MOVE_SPEED    (rc_sensor_info.mouse_vz)


/* 检测鼠标按键状态
   按下为1，没按下为0*/
#define    IF_MOUSE_PRESSED_LEFT    (rc_sensor_info.mouse_btn_l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (rc_sensor_info.mouse_btn_r == 1)


/* 检测键盘按键状态
   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define    IF_KEY_PRESSED         (  rc_sensor_info.key_v  )
#define    IF_KEY_PRESSED_W       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_SHIFT) != 0 )

/* Exported types ------------------------------------------------------------*/
#define abs(x) 					((x)>0? (x):(-(x)))

typedef struct rc_sensor_info_struct
{
    int16_t 	ch0;
    int16_t 	ch1;
    int16_t 	ch2;
    int16_t 	ch3;
    uint8_t  	s1;
    uint8_t  	s2;
    int16_t		mouse_vx;
    int16_t 	mouse_vy;
    int16_t 	mouse_vz;
    uint8_t 	mouse_btn_l;
    uint8_t 	mouse_btn_r;
    uint16_t	key_v;
    int16_t 	thumbwheel;

    uint8_t     s1_pre;
    uint8_t     s2_pre;
    bool     s1_siwtch_up;
    bool     s1_switch_uptomid;
    bool     s1_switch_downtomid;
    bool     s1_siwtch_down;
    bool     s2_siwtch_up;
    bool     s2_switch_uptomid;
    bool     s2_switch_downtomid;
    bool     s2_siwtch_down;
    uint8_t     init_flag;
    int16_t		offline_cnt;
    int16_t		offline_max_cnt;
} rc_sensor_info_t;

typedef struct rc_sensor_struct
{
    rc_sensor_info_t*	info;
    void	(*init)(struct rc_sensor_struct* self);
    void	(*update)(struct rc_sensor_struct* self, uint8_t* rxBuf);
    void	(*check)(struct rc_sensor_struct* self);
    void	(*heart_beat)(struct rc_sensor_struct* self);
} rc_sensor_t;

extern rc_sensor_info_t rc_sensor_info;
extern rc_sensor_t 		rc_sensor;
/* Exported functions --------------------------------------------------------*/
float DeathZoom(float input, float center, float death);
bool RC_IsChannelReset(void);
void RC_ResetData(rc_sensor_t* rc);
































#endif
