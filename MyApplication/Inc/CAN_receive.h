#ifndef _CAN_RECEIVE_H_
#define _CAN_RECEIVE_H_

#include "main.h"
#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PITCH_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
} can_msg_id_e;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct
{
    uint16_t angel;
    int16_t speed;
    int16_t current;
    uint16_t	angle_prev;
    int32_t		angle_sum;
    uint8_t		init_flag;

    uint8_t temperate;
    int16_t last_ecd;
} motor_DATA_t;

extern motor_DATA_t YAW_6020_DATA;
extern motor_DATA_t PITCH_6020_DATA;
extern motor_DATA_t M3508_Receive[4];
extern motor_DATA_t TRIGGER_DATA;
void can_filter_init(void);
void CAN_MOTOR_CHECK(int16_t Value, int16_t SEND_ID, int16_t MOTOR_ID, int CAN_ID);
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void CAN_cmd_chassis_reset_ID(void);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern const motor_measure_t* get_yaw_gimbal_motor_measure_point(void);
extern const motor_measure_t* get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t* get_trigger_motor_measure_point(void);
extern const motor_measure_t* get_chassis_motor_measure_point(uint8_t i);

#endif
