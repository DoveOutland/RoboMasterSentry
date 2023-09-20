#ifndef __JUDGE_H
#define __JUDGE_H
#include "main.h"

//帧头结构定义
typedef __packed struct
{
    uint8_t  SOF;
    uint16_t DataLength;
    uint8_t  Seq;
    uint8_t  CRC8;
} xFrameHeader;

//数据cmdid
typedef enum
{
    GAME_STATE_DATA_ID = 0x0001,  //比赛状态数据， 1Hz 周期发送
    GAME_RESULT_ID     = 0x0002, //比赛结果数据，比赛结束后发送 [6]
    ROBO_survive_DATA	 = 0x0003, //比赛机器人存活数据， 1Hz 周期发送

    REAL_FIELD_DATA_ID = 0x0101, //场地事件数据，事件改变后发送[5]
    FIELD_depot_DATA	 = 0x0102, //场地补给站动作标识数据，动作改变后发送

    ROBO_STATE_DATA_ID = 0x0201, //机器人状态数据， 10Hz 周期发送 [1]
    HEAT_POWER_DATA_ID = 0x0202, //实时功率热量数据， 50Hz 周期发送 [4]
    GAME_POSITION_ID	 = 0x0203, //机器人位置数据， 10Hz 发送 [8]
    GAIN_BUFF_ID    	 = 0x0204, //机器人增益数据， 增益状态改变后发送 [7]

    HURT_STATE_DATA		 = 0x0206, //伤害状态数据，伤害发生后发送
    REAL_SHOOT_DATA_ID = 0x0207, //实时射击数据，子弹发射后发送		[3]
    STU_CUSTOM_DATA_ID = 0x0301, //机器人间交互数据，发送方触发发送，上限 10Hz [100]
} judge_data_id_e;

/*-------------------接口协议说明-------------------*/

//1.比赛状态数据： 0x0001。 发送频率： 1Hz
typedef __packed struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_status_t;

//2.比赛结果数据： 0x0002。 发送频率：比赛结束后发送
typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;

//3.机器人血量数据：0x0003。发送频率：1Hz
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

//4. 场地事件数据：0x0101。发送频率：1Hz
typedef __packed struct
{
    uint32_t event_type;
} ext_event_data_t;

//5. 补给站动作标识：0x0102。发送频率：动作改变后发送, 发送范围：己方机器人
typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


//7. 比赛机器人状态： 0x0201。 发送频率： 10Hz
typedef __packed struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;

    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;

    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;

    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;

    uint16_t chassis_power_limit;

    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;
//typedef __packed struct // 旧版协议
//{
//    uint8_t robot_id;
//    uint8_t robot_level;
//    uint16_t remain_HP;
//    uint16_t max_HP;
//
//    uint16_t shooter_heat0_cooling_rate;
//    uint16_t shooter_heat0_cooling_limit;
//    uint16_t shooter_heat1_cooling_rate;
//    uint16_t shooter_heat1_cooling_limit;
//    uint8_t shooter_heat0_speed_limit;
//    uint8_t shooter_heat1_speed_limit;
//
//    uint8_t max_chassis_power;
//
//    uint8_t mains_power_gimbal_output : 1;
//    uint8_t mains_power_chassis_output : 1;
//    uint8_t mains_power_shooter_output : 1;
//} ext_game_robot_status_t;

//8. 实时功率热量数据：0x0202。发送频率：50Hz
typedef __packed struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;

    uint16_t chassis_power_buffer;

    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;
//typedef __packed struct // 旧版协议
//{
//    uint16_t chassis_volt;
//    uint16_t chassis_current;
//    float chassis_power;

//    uint16_t chassis_power_buffer;

//    uint16_t shooter_heat0;
//    uint16_t shooter_heat1;
//    uint16_t mobile_shooter_heat2;
//} ext_power_heat_data_t;

//9. 机器人位置：0x0203。发送频率：10Hz
typedef __packed struct
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

//10. 机器人增益： 0x0204。 发送频率：状态改变后发送
typedef __packed struct
{
    uint8_t power_rune_buff;
} ext_buff_t;

//11. 空中机器人能量状态： 0x0205。 发送频率： 10Hz
typedef __packed struct
{
    uint16_t energy_point;
    uint8_t attack_time;
} ext_aerial_robot_energy_t;
//typedef __packed struct
//{
//    uint8_t attack_time;
//} aerial_robot_energy_t;

//12. 伤害状态： 0x0206。 发送频率：伤害发生后发送
typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

//13. 实时射击信息： 0x0207。 发送频率：射击后发送
typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

/*------------------机器人间交互数据------------------*/

//交互数据接收信息： 0x0301。 发送频率：上限 10Hz
typedef __packed struct
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

//交互数据机器人间通信： 0x0301。发送频率：上限 10Hz
typedef __packed struct
{
    uint8_t data[30];
} robot_interactive_data_t;

//飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人。
typedef __packed struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
} ext_dart_status_t;

//人工智能挑战赛加成与惩罚区状态：0x0005。发送频率：1Hz 周期发送，发送范围：所有机器人。
typedef __packed struct
{
    uint8_t F1_zone_status: 1;
    uint8_t F1_zone_buff_debuff_status: 3;
    uint8_t F2_zone_status: 1;
    uint8_t F2_zone_buff_debuff_status: 3;
    uint8_t F3_zone_status: 1;
    uint8_t F3_zone_buff_debuff_status: 3;
    uint8_t F4_zone_status: 1;
    uint8_t F4_zone_buff_debuff_status: 3;
    uint8_t F5_zone_status: 1;
    uint8_t F5_zone_buff_debuff_status: 3;
    uint8_t F6_zone_status: 1;
    uint8_t F6_zone_buff_debuff_status: 3;
} ext_ICRA_buff_debuff_zone_status_t;

//裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送，发送范围：己方机器人。;
typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;

//飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人。;
typedef __packed struct
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

//子弹剩余发射数：0x0208。发送频率：1Hz 周期发送，空中机器人，哨兵机器人以及 ICRA 机器人主控发送，发送范围：单一机器人。
typedef __packed struct
{
    uint16_t bullet_remaining_num;
} ext_bullet_remaining_t;

//机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人。
typedef __packed struct
{
    uint32_t rfid_status;
} ext_rfid_status_t;

//飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人。
typedef __packed struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint8_t first_dart_speed;
    uint8_t second_dart_speed;
    uint8_t third_dart_speed;
    uint8_t fourth_dart_speed;
    uint16_t last_dart_launch_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

//客户端删除图形 机器人间通信：0x0301。
typedef __packed struct
{
    uint8_t operate_tpye;
    uint8_t layer;
} ext_client_custom_graphic_delete_t;

//图形数据
typedef __packed struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye: 3;
    uint32_t graphic_tpye: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t start_angle: 9;
    uint32_t end_angle: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    uint32_t radius: 10;
    uint32_t end_x: 11;
    uint32_t end_y: 11;
} graphic_data_struct_t;

//客户端绘制一个图形 机器人间通信：0x0301。
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

//客户端绘制二个图形 机器人间通信：0x0301。
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

//客户端绘制五个图形 机器人间通信：0x0301。
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

//客户端绘制七个图形 机器人间通信：0x0301。
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

//客户端绘制字符 机器人间通信：0x0301。
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;

unsigned char Get_CRC8_Check_Sum(unsigned char* pchMessage, unsigned int dwLength, unsigned char ucCRC8);
uint16_t Get_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);

unsigned int Verify_CRC8_Check_Sum(unsigned char* pchMessage, unsigned int dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);

void Append_CRC8_Check_Sum(unsigned char* pchMessage, unsigned int dwLength);
void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);

/*------------------裁判系统数据接收结构体------------------*/
//0001 Frame1 机器人状态包
typedef __packed struct
{
    uint8_t SOF;
    uint16_t DataLength;
    uint8_t Seq;
    uint8_t CRC8;
    uint16_t CmdID;
    ext_game_robot_status_t extGameRobotState;
    uint16_t CRC16Tail;
} Frame1;

//0002 Frame2 伤害数据包
typedef __packed struct
{
    uint8_t SOF;
    uint16_t DataLength;
    uint8_t Seq;
    uint8_t CRC8;
    uint16_t CmdID;
    ext_robot_hurt_t extRobotHurt;
    uint16_t CRC16Tail;
} Frame2;

//0003 Frame3 实时射击数据包
typedef __packed struct
{
    uint8_t SOF;
    uint16_t DataLength;
    uint8_t Seq;
    uint8_t CRC8;
    uint16_t CmdID;
    ext_shoot_data_t extShootData;
    uint16_t CRC16Tail;
} Frame3;

//0004 Frame4 实时功率热量包
typedef __packed struct
{
    uint8_t   SOF;
    uint16_t  DataLength;
    uint8_t   Seq;
    uint8_t   CRC8;
    uint16_t  CmdID;
    ext_power_heat_data_t extPowerHeatData;
    uint16_t CRC16Tail;
} Frame4;

extern Frame1 Robot_State;
extern Frame4 Power_Heat;

extern xFrameHeader	FrameHeader;//帧头信息
extern int Judge_ID;
extern int J_CMD;
extern uint8_t jbus_rx_buffer[200];

void Judge_Data_Receive(uint8_t* ReadFromUsart);

#endif

