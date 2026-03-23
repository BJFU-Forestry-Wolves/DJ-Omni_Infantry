/*
 *  Project      : Infantry_Neptune
 * 
 *  file         : referee_periph.h
 *  Description  : This document contains the data receiving and sending of the referee system
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 09:27:53
 */


#ifndef REFEREE_PERIPH_H
#define REFEREE_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "util_uart.h"
#include "alg_crc.h"
#include "string.h"

extern UART_HandleTypeDef* Const_Referee_UART_HANDLER;
extern const uint8_t PARSE_FAILED, PARSE_SUCCEEDED;
/********** START OF REFEREE CMD STRUCT DEFINITION **********/

typedef __packed struct {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
	uint64_t SyncTimeStamp; 
} ext_game_status_t;

typedef __packed struct {
    uint8_t winner;
} ext_game_result_t;

typedef __packed struct {
    uint16_t ally_1_robot_HP; //1号英雄 
    uint16_t ally_2_robot_HP;  
    uint16_t ally_3_robot_HP; //3号步兵 
    uint16_t ally_4_robot_HP;  
    uint16_t reserved;  
    uint16_t ally_7_robot_HP; //7号哨兵 
    uint16_t ally_outpost_HP;  
    uint16_t ally_base_HP; 
} ext_game_robot_HP_t;

typedef __packed struct { 
  uint8_t level; 
  uint8_t offending_robot_id; 
  uint8_t count; 
}ext_referee_warning_t;      //裁判系统警告


typedef __packed struct {
    uint32_t event_type;
} ext_event_data_t;

typedef __packed struct {
    uint8_t supply_projectile_id; 
    uint8_t supply_robot_id; 
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


typedef __packed struct {
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;


typedef __packed struct { 
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value; //机器人射击热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;  //机器人射击热量上限
    uint16_t chassis_power_limit;        //机器人底盘功率上限 
    uint8_t power_management_gimbal_output : 1; 
    uint8_t power_management_chassis_output : 1;  
    uint8_t power_management_shooter_output : 1; 
} ext_game_robot_status_t;

typedef __packed struct {
    uint16_t buffer_energy;            //缓存能量
    uint16_t shooter_17mm_barrel_heat; //当前热量
} ext_power_heat_data_t;

typedef __packed struct {
    float x; 
    float y; 
    float angle;                        //本机器人测速模块的朝向
} ext_game_robot_pos_t;


typedef __packed struct {
    uint8_t armor_id : 4; 
    uint8_t HP_deduction_reason : 4;
} ext_robot_hurt_t;                   //是谁打了我

typedef __packed struct {
   uint8_t bullet_type;  
  uint8_t shooter_number; 
  uint8_t launching_frequency;  
  float initial_speed; 
} ext_shoot_data_t;                   //实时射击数据


typedef __packed struct {
    uint16_t data_cmd_id; 
    uint16_t sender_ID; 
    uint16_t receiver_ID;
} robot_interaction_data_t;


typedef __packed struct { 
    uint8_t operate_type; 
    uint8_t layer; 
} ext_client_custom_graphic_delete_t;





typedef __packed struct { 
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

/********** END OF REFEREE CMD STRUCT DEFINITION **********/


typedef enum {
    Referee_STATE_NULL      = 0,
    Referee_STATE_CONNECTED = 1,
    Referee_STATE_LOST      = 2,
    Referee_STATE_ERROR     = 3,
    Referee_STATE_PENDING   = 4
} Referee_RefereeStateEnum;

typedef struct {
    Referee_RefereeStateEnum state;             // 裁判系统当前状态
    uint32_t last_update_time;                  // 裁判系统上次更新时间
    
    uint16_t client_id;                         // 客户端ID   
	uint8_t game_type;                        //  游戏类型,    1:RoboMaster 机甲大师赛；
	                                            //              2:RoboMaster 机甲大师单项赛；
                                                //      	    3：ICRA RoboMaster 人工智能挑战赛
	                                            //              4：RoboMaster 联盟赛3V3
	                                            //              5：RoboMaster 联盟赛1V1
    uint8_t game_progress;                      //  当前比赛阶段,0：未开始比赛；
                                                //              1：准备阶段；
                                                //              2：自检阶段；
                                                //              3：5s倒计时；
                                                //              4：对战中；
                                                //              5：比赛结算中
    uint16_t stage_remain_time;                 
    uint64_t SyncTimeStamp;                    
	uint8_t winner;                             //谁赢了0平局，1红胜，2蓝胜
	struct {
        uint16_t  Hero_HP;  //Channel 0 channel 1 channel 2 channel 3 paddle wheel
        uint16_t  Sentinel_HP;   //Switch 0 switch 1
    } Teammate_HP;                      
    struct {
        uint8_t  level;  //判罚等级
        uint8_t  offending_robot_id;//哪个蠢蛋被判罚了
		uint8_t  count;  //判罚次数
    } waring;
	struct{
    uint8_t robot_id;
    uint8_t robot_level; 
    uint16_t current_HP; 
	uint16_t shooter_limit;                      //热量上限
    uint8_t max_chassis_power;                  //底盘功率上限
    uint8_t mains_power_gimbal_output;          //云台是否有输出
    uint8_t mains_power_chassis_output;         //底盘是否有输出
    uint8_t mains_power_shooter_output;         //发射是否有输出
    uint16_t buffer_energy;                      //当前底盘缓存能量
    uint16_t shoot_heat;                         //当前射击热量
    }status ;
    float x; 
    float y; 
    float angle;
    
	struct {
        uint8_t  armor_id:4;  
        uint8_t  HP_deduction_reason:4;
    } who_shoot_me;
      
	struct {
        uint8_t  type;                         //子弹类型
        uint8_t  number;                       //子弹发射机构？SBdJ写的什么东西
		uint8_t	 frequency;                    //弹丸射速
		uint8_t  speed ;                       //弹丸初速度
    } what_can_I_shoot;
		
} Referee_RefereeDataTypeDef;

typedef uint8_t (*Referee_RefereeCmdParseFuncDef)(Referee_RefereeDataTypeDef* referee, void *data_ptr);

typedef struct {
    uint16_t cmd_id;                            // 指令ID
    uint8_t data_length;                        // 数据帧长度
    Referee_RefereeCmdParseFuncDef parse_func;  // 解析函数指针
} Referee_RefereeCmdTypeDef;

typedef struct {
    uint16_t robot_id;
    uint16_t client_id;
} Referee_RobotAndClientIDTypeDef;

typedef union {
    struct {
        uint32_t radius:10; 
        uint32_t end_x:11; 
        uint32_t end_y:11;
    } graphic_data;
    uint32_t ui32_data;
    float float_data;
    int32_t int_data;
} Referee_GraphicDataConverterUnion;




extern const uint16_t Const_Referee_RX_BUFF_LEN;
extern const uint16_t Const_Referee_REMOTE_OFFLINE_TIME;


extern Referee_RefereeDataTypeDef Referee_RefereeData;


Referee_RefereeDataTypeDef* Referee_GetRefereeDataPtr(void);
void Referee_ResetRefereeData(void);
void Referee_InitReferee(void);
uint16_t Referee_GetClientIDByRobotID(uint8_t robot_id);
void Referee_SendInteractiveData(uint16_t data_cmd_id, uint16_t receiver_ID, const uint8_t *interactive_data, uint16_t interactive_data_length);
void Referee_SendRobotCustomData(uint16_t data_cmd_id, uint16_t receiver_ID, const uint8_t *data, uint16_t data_length);
uint8_t Referee_IsRefereeOffline(void);
uint8_t Referee_CheckDataLengthByCmdID(uint16_t cmd_id, uint16_t data_length);
uint8_t Referee_ParseRobotCustomData(uint8_t* data, uint16_t data_length);
uint8_t Referee_ParseRefereeCmd(uint16_t cmd_id, uint8_t* data, uint16_t data_length);
void Referee_DecodeRefereeData(uint8_t* buff, uint16_t rxdatalen);
void Referee_RXCallback(UART_HandleTypeDef* huart);

#endif

#ifdef __cplusplus
}
#endif
