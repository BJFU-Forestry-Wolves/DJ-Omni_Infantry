/*
 *  Project      : Infantry_Neptune
 * 
 *  file         : referee_periph.c
 *  Description  : This document contains the data receiving and sending of the referee system
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 10:10:43
 */

#include "periph_referee.h"
#include "periph_remote.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>


UART_HandleTypeDef* Const_Referee_UART_HANDLER          = &huart6;
lqw_bref_data bref_data;
const uint16_t Const_Referee_TX_BUFF_LEN            = 300;
const uint16_t Const_Referee_RX_BUFF_LEN            = 300;
const uint16_t Const_Referee_REMOTE_OFFLINE_TIME    = 1000;

uint8_t Referee_TxData[Const_Referee_TX_BUFF_LEN];
uint8_t Referee_RxData[Const_Referee_RX_BUFF_LEN];
Referee_RefereeDataTypeDef Referee_RefereeData;

const uint8_t PARSE_FAILED = 0, PARSE_SUCCEEDED = 1;


/********** REFEREE CMD PARSER FUNCTION **********/


uint8_t P_ext_game_status(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_game_status_t *struct_ptr = data_ptr;
    
    referee->game_type = struct_ptr->game_type;
    referee->game_progress = struct_ptr->game_progress;
    referee->stage_remain_time = struct_ptr->stage_remain_time;
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_game_result(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_game_result_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_game_robot_HP(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_game_robot_HP_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_dart_status(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_dart_status_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_event_data(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_event_data_t *struct_ptr = data_ptr;
    
    referee->event_type = struct_ptr->event_type;
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_supply_projectile_action(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_supply_projectile_action_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_referee_warning(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_referee_warning_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_dart_remaining_time(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_dart_remaining_time_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_game_robot_status(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_game_robot_status_t *struct_ptr = data_ptr;
    
    referee->robot_level = struct_ptr->robot_level;
    referee->remain_HP = struct_ptr->remain_HP;
    referee->max_chassis_power = struct_ptr->chassis_power_limit;
    referee->mains_power_gimbal_output = struct_ptr->mains_power_gimbal_output;
    referee->mains_power_chassis_output = struct_ptr->mains_power_chassis_output;//底盘功率上限
    referee->mains_power_shooter_output = struct_ptr->mains_power_shooter_output;
    referee->shooter_heat0_cooling_rate = struct_ptr->shooter_id1_17mm_cooling_rate;
    referee->shooter_heat1_cooling_rate = struct_ptr->shooter_id2_17mm_cooling_rate;
    referee->shooter_heat0_cooling_limit = struct_ptr->shooter_id1_17mm_cooling_limit;
    referee->shooter_heat1_cooling_limit = struct_ptr->shooter_id2_17mm_cooling_limit;
    referee->shooter_heat0_speed_limit = struct_ptr->shooter_id1_17mm_speed_limit;
    referee->shooter_heat1_speed_limit = struct_ptr->shooter_id2_17mm_speed_limit;
    
    if (referee->robot_id != struct_ptr->robot_id) {
        referee->robot_id = struct_ptr->robot_id;
        referee->client_id = Referee_GetClientIDByRobotID(referee->robot_id);
    }
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_power_heat_data(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_power_heat_data_t *struct_ptr = data_ptr;
    referee->chassis_volt = struct_ptr->chassis_volt;
    referee->chassis_current = struct_ptr->chassis_current;
    referee->chassis_power = struct_ptr->chassis_power;
    referee->chassis_power_buffer = struct_ptr->chassis_power_buffer;
    referee->shooter_heat0 = struct_ptr->shooter_heat0;
    referee->shooter_heat1 = struct_ptr->shooter_heat1;
    referee->mobile_shooter_heat2 = struct_ptr->mobile_shooter_heat2;
    
    // Referee_DrawingTimeBaseCallback();
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_game_robot_pos(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_game_robot_pos_t *struct_ptr = data_ptr;
    
    referee->x = struct_ptr->x;
    referee->y = struct_ptr->y;
    referee->z = struct_ptr->z;
    referee->yaw = struct_ptr->yaw;
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_buff(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_buff_t *struct_ptr = data_ptr;
    
    referee->power_rune_buff = struct_ptr->power_rune_buff;
    
    return PARSE_SUCCEEDED;
}

uint8_t P_aerial_robot_energy(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    // aerial_robot_energy_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_robot_hurt(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_robot_hurt_t *struct_ptr = data_ptr;

    // Hurt Callback
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_shoot_data(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_shoot_data_t *struct_ptr = data_ptr;
	
		referee->bullet_freq = struct_ptr->bullet_freq;
		referee->bullet_speed = struct_ptr->bullet_speed; 
		referee->bullet_type = struct_ptr->bullet_type;
    // Shoot Callback
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_bullet_remaining(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_bullet_remaining_t *struct_ptr = data_ptr;
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_rfid_status(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_rfid_status_t *struct_ptr = data_ptr;
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_dart_cmd(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_dart_client_cmd_t *struct_ptr = data_ptr;
        
    return PARSE_SUCCEEDED;
}


/********** END OF REFEREE CMD PARSER FUNCTION **********/


const uint16_t Const_Referee_FRAME_HEADER_SOF       = 0xA5;     // 裁判系统指令帧头长度
const Referee_RobotAndClientIDTypeDef   // 机器人ID及对应客户端ID，0表示无对应客户端
    HERO_RED        = {1,   0x0101},    // 英雄(红)
    ENGINEER_RED    = {2,   0x0102},    // 工程(红)
    INFANTRY3_RED   = {3,   0x0103},    // 步兵3(红)
    INFANTRY4_RED   = {4,   0x0104},    // 步兵4(红)
    INFANTRY5_RED   = {5,   0x0105},    // 步兵5(红)
    AERIAL_RED      = {6,   0x0106},    // 空中(红)
    SENTRY_RED      = {7,   0},         // 哨兵(红)
    HERO_BLUE       = {101, 0x0165},    // 英雄(蓝)
    ENGINEER_BLUE   = {102, 0x0166},    // 工程(蓝)
    INFANTRY3_BLUE  = {103, 0x0167},    // 步兵3(蓝)
    INFANTRY4_BLUE  = {104, 0x0168},    // 步兵4(蓝)
    INFANTRY5_BLUE  = {105, 0x0169},    // 步兵5(蓝)
    AERIAL_BLUE     = {106, 0x016A},    // 空中(蓝)
    SENTRY_BLUE     = {107, 0};         // 哨兵(蓝)
        
const uint16_t Const_Referee_CMD_NUM                = 20;       // 裁判系统指令个数（不含交互指令）
const Referee_RefereeCmdTypeDef Const_Referee_CMD_LIST[Const_Referee_CMD_NUM] = {           // 裁判系统消息命令ID列表
    {0x0001,    11, &P_ext_game_status},                // 比赛状态数据，1Hz 周期发送
    {0x0002,    1,  &P_ext_game_result},                // 比赛结果数据，比赛结束后发送
    {0x0003,    28, &P_ext_game_robot_HP},              // 比赛机器人血量数据，1Hz 周期发送
    {0x0004,    3,  &P_ext_dart_status},                // 飞镖发射状态，飞镖发射时发送
    {0x0005,    11, NULL},                              // （未使用）人工智能挑战赛加成与惩罚区状态，1Hz周期发送
    {0x0101,    4,  &P_ext_event_data},                 // 场地事件数据，事件改变后发送
    {0x0102,    3,  &P_ext_supply_projectile_action},   // 场地补给站动作标识数据，动作改变后发送
    {0x0103,    2,  NULL},                              // （已废弃）请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）
    {0x0104,    2,  &P_ext_referee_warning},            // 裁判警告数据，警告发生后发送
    {0x0105,    1,  &P_ext_dart_remaining_time},        // 飞镖发射口倒计时，1Hz周期发送
    {0x0201,    15, &P_ext_game_robot_status},          // 机器人状态数据，10Hz 周期发送
    {0x0202,    14, &P_ext_power_heat_data},            // 实时功率热量数据，50Hz 周期发送
    {0x0203,    16, &P_ext_game_robot_pos},             // 机器人位置数据，10Hz 发送
    {0x0204,    1,  &P_ext_buff},                       // 机器人增益数据，1Hz 周期发送
    {0x0205,    3,  &P_aerial_robot_energy},            // 空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送
    {0x0206,    1,  &P_ext_robot_hurt},                 // 伤害状态数据，伤害发生后发送
    {0x0207,    6,  &P_ext_shoot_data},                 // 实时射击数据，子弹发射后发送
    {0x0208,    2,  &P_ext_bullet_remaining},           // 弹丸剩余发射数，仅空中机器人，哨兵机器人以及ICRA机器人发送，1Hz周期发送
    {0x0209,    4,  &P_ext_rfid_status},                // 机器人RFID状态，1Hz周期发送
    {0x020A,    12, &P_ext_dart_cmd}                    // 飞镖机器人客户端指令书，10Hz周期发送
};

const Referee_RefereeCmdTypeDef Const_Referee_CMD_INTERACTIVE       = {0x0301, 8, NULL};    // 机器人间交互数据，发送方触发发送
// 注：这里的6是交互数据帧头的长度，因为交互数据帧是不定长的
//const uint16_t Const_Referee_DATA_CMD_ID_CLIENT_CUSTOM_DATA       = 0xD180;               // （已废弃）客户端自定义数据内容ID
const uint16_t Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND    = 0x0200;               // 机器人间交互数据内容ID下界
const uint16_t Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND    = 0x02FF;               // 机器人间交互数据内容ID上界
const uint16_t Const_Referee_DATA_INTERACTIVE_DATA_MAX_LENGTH       = 113 - 1;              // 机器人间交互数据内容最大长度
const uint16_t Const_Referee_GRAPHIC_BUFFER_MAX_LENGTH              = 21;                   // 图形缓冲区最大长度
const Referee_RefereeCmdTypeDef Const_Referee_DATA_CMD_ID_LIST[6]   = {                     // 裁判系统交互数据内容ID
    {0x0100,    2,      NULL},              // 客户端删除图形
    {0x0101,    15,     NULL},              // 客户端绘制一个图形
    {0x0102,    30,     NULL},              // 客户端绘制二个图形
    {0x0103,    75,     NULL},              // 客户端绘制五个图形
    {0x0104,    105,    NULL},              // 客户端绘制七个图形
    {0x0110,    45,     NULL}               // 客户端绘制字符图形
};

graphic_data_struct_t Referee_dummyGraphicCmd = {{0x00, 0x00, 0x00}, Draw_OPERATE_NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


/**
  * @brief      获取裁判系统数据对象的指针
  * @param      无
  * @retval     指针指向裁判系统数据对象
  */
Referee_RefereeDataTypeDef* Referee_GetRefereeDataPtr() {
    return &Referee_RefereeData;
}


/**
  * @brief      重置裁判系统数据对象
  * @param      无
  * @retval     无
  */
void Referee_ResetRefereeData() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    Referee_RefereeStateEnum state = referee->state;        // backup state
    uint32_t last_update_time = referee->last_update_time;  // backup time
    memset(referee, 0, sizeof(Referee_RefereeDataTypeDef));
    referee->state = state;
    referee->last_update_time = last_update_time;
}


/**
  * @brief      初始化裁判系统
  * @param      无
  * @retval     无
  */
void Referee_InitReferee() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    Referee_ResetRefereeData();
    referee->last_update_time = HAL_GetTick();
    Uart_InitUartDMA(Const_Referee_UART_HANDLER);
    Uart_ReceiveDMA(Const_Referee_UART_HANDLER, Referee_RxData, Const_Referee_RX_BUFF_LEN);
}


/**
  * @brief      通过机器人ID获取对应客户端ID
  * @param      robot_id: 机器人ID
  * @retval     客户端ID
  */
uint16_t Referee_GetClientIDByRobotID(uint8_t robot_id) {
    if (robot_id == 7 || robot_id == 107) return 0;
    if ((robot_id >= 1 && robot_id <= 6) || (robot_id >= 101 && robot_id <= 106)) 
        return robot_id + 0x100;
    return 0;
}


/**
  * @brief      裁判系统交互数据发送函数（阻塞）
  * @param      data_cmd_id: 数据内容ID
  * @param      receiver_ID: 接受者ID
  * @param      interactive_data: 交互数据帧
  * @param      interactive_data_length: 交互数据帧长度
  * @retval     无
  */
void Referee_SendInteractiveData(uint16_t data_cmd_id, uint16_t receiver_ID, const uint8_t *interactive_data, uint16_t interactive_data_length) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    static uint8_t seq = 0;
    uint8_t *buf = Referee_TxData;
    buf[0] = Const_Referee_FRAME_HEADER_SOF;
    
    uint16_t *data_length_ptr = (void *) (buf + 1);
    *data_length_ptr = interactive_data_length + Const_Referee_CMD_INTERACTIVE.data_length;
    
    uint8_t *seq_ptr = (void *) (buf + 3);
    *seq_ptr = seq;   // not obvious in doc
    seq = (seq + 1) % 256;

    uint8_t *crc8_ptr = (void *) (buf + 4);
    *crc8_ptr = CRC_GetCRC8CheckSum(buf, 4, CRC8_INIT);
    
    buf[5] = 0x01;
    buf[6] = 0x03;
    
    ext_student_interactive_header_data_t *header = (void *) (buf + 7);
    header->data_cmd_id  = data_cmd_id;
    header->receiver_ID  = receiver_ID;
    header->sender_ID    = (uint16_t)referee->robot_id;
    
    memcpy(buf + 5 + Const_Referee_CMD_INTERACTIVE.data_length, interactive_data, interactive_data_length);
    
    uint16_t *crc16_ptr = (void *) (buf + 5 + 2 + *data_length_ptr);
    *crc16_ptr = CRC_GetCRC16CheckSum(buf, 5 + 2 + *data_length_ptr, CRC16_INIT);
    
    uint16_t tx_size = 5 + 2 + *data_length_ptr + 2;
    Uart_SendMessage(Const_Referee_UART_HANDLER, buf, tx_size, 100);
}









/**
  * @brief      机器人间交互数据发送函数
  * @param      data_cmd_id: 数据内容ID
  * @param      receiver_ID: 接受者ID
  * @param      data: 数据帧
  * @param      data_length: 数据帧长度
  * @retval     无
  */
void Referee_SendRobotCustomData(uint16_t data_cmd_id, uint16_t receiver_ID, const uint8_t *data, uint16_t data_length) {
    if (data_cmd_id < Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND || 
        data_cmd_id > Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND)  return;                   // wrong data cmd id
    if (receiver_ID == 0 || (receiver_ID > 10 && receiver_ID < 110) || receiver_ID > 107) return;   // wrong receiver id
    if (data_length > Const_Referee_DATA_INTERACTIVE_DATA_MAX_LENGTH) return;                       // interactive data too long
    Referee_SendInteractiveData(data_cmd_id, receiver_ID, data, data_length);
}









/**
  * @brief      判断裁判系统是否离线
  * @param      无
  * @retval     是否离线（1为是，0为否）
  */
uint8_t Referee_IsRefereeOffline() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    uint32_t now = HAL_GetTick();
    if ((now - referee->last_update_time) > Const_Referee_REMOTE_OFFLINE_TIME)
        referee->state = Referee_STATE_LOST;
    return referee->state == Referee_STATE_LOST;
}


/**
  * @brief      机器人间交互数据解析函数
  * @param      data: 数据帧
  * @param      data_length: 数据帧长度
  * @retval     解析结果（0为失败，1为成功）
  */
uint8_t Referee_ParseRobotCustomData(uint8_t* data, uint16_t data_length) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    
    //if (data_length != Const_Referee_CMD_INTERACTIVE.data_length) return PARSE_FAILED;      // wrong data length
    
    ext_student_interactive_header_data_t *header_struct_ptr = (void *) data;
    if (header_struct_ptr->data_cmd_id < Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND || 
        header_struct_ptr->data_cmd_id > Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND) 
        return PARSE_FAILED;    // wrong data cmd id
    if (header_struct_ptr->receiver_ID != referee->robot_id) return PARSE_FAILED;           // wrong receiver id
    
    //uint8_t interactive_data_ptr = data + Const_Referee_CMD_INTERACTIVE.data_length;
    
    // Interactive Data Recieve Callback
    
    return PARSE_SUCCEEDED;
}


/**
  * @brief      裁判系统数据解析函数
  * @param      cmd_id: 命令ID
  * @param      data: 数据帧
  * @param      data_length: 数据帧长度
  * @retval     解析结果（0为失败，1为成功）
  */
uint8_t Referee_ParseRefereeCmd(uint16_t cmd_id, uint8_t* data, uint16_t data_length) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
	 /*判断是否为机器人间交互*/
    if (cmd_id == Const_Referee_CMD_INTERACTIVE.cmd_id) return Referee_ParseRobotCustomData(data, data_length);
    
    for (int i = 0; i < Const_Referee_CMD_NUM; ++i) {
        if (cmd_id == Const_Referee_CMD_LIST[i].cmd_id) {
            //if (data_length != Const_Referee_CMD_LIST[i].data_length) return PARSE_FAILED;  // wrong data length
            if (Const_Referee_CMD_LIST[i].parse_func == NULL) return PARSE_FAILED;          // unsupported cmd
            return (*(Const_Referee_CMD_LIST[i].parse_func))(referee, data);                // parse cmd
        }
    }
    
    return PARSE_FAILED;    // unknown cmd
}

/**
  * @brief 
  * @param cmd_id: [输入/出] 
**			 data_lengh: [输入/出] 
**			 data_buffer: [输入/出] 
  * @return NULL
  * @retval NULL
  */
void lqw_update_referee(uint16_t cmd_id,uint16_t data_lengh,uint8_t * data_buffer  ){
	switch(cmd_id){
		case 0x207:{
		  //bref_data.init_speed=(float)((uint32_t)(&data_buffer[3]));
			memcpy(&bref_data.init_speed, &data_buffer[3], sizeof(float));
		}
		case 0x201:{
		  memcpy(&bref_data.chass_power,&data_buffer[10],sizeof(uint16_t));
			memcpy(&bref_data.robot_id,&data_buffer[0],sizeof(uint8_t));
		}
		//case 
	}


}
/**
  * @brief      裁判系统串口数据解码函数
  * @param      buff: 数据缓冲区
  * @param      rxdatalen: 数据长度
  * @retval     无
  */
void Referee_DecodeRefereeData(uint8_t* buff, uint16_t rxdatalen) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    
    referee->state              = Referee_STATE_PENDING;    // 加锁防止互斥读写
    referee->last_update_time   = HAL_GetTick();            // 时间戳
    
    if (buff[0] != Const_Referee_FRAME_HEADER_SOF) {
        referee->state          = Referee_STATE_ERROR;
        return;
    }
    
    if (!CRC_VerifyCRC8CheckSum(buff, 5)) {
        referee->state          = Referee_STATE_ERROR;
        return;
    }
    
    uint16_t data_length = (uint16_t) buff[2] << 8 | buff[1];
    uint8_t seq = buff[3];
    if (seq == 0) {
        referee->state          = Referee_STATE_ERROR;
        return;
    }
    if (!CRC_VerifyCRC16CheckSum(buff, data_length + 9)) {
        referee->state          = Referee_STATE_ERROR;
        return;
    }
    
    uint16_t cmd_id = (uint16_t) buff[6] << 8 | buff[5];
    if (!Referee_ParseRefereeCmd(cmd_id, buff + 7, data_length)) {  //这个是cmd命令的作用处
        referee->state          = Referee_STATE_ERROR;
        return;
    }
    lqw_update_referee(cmd_id,data_length,&buff[7]);
    referee->state              = Referee_STATE_CONNECTED;  // 解锁
}


/**
  * @brief      裁判系统串口回调函数
  * @param      huart: 指针指向串口句柄
  * @retval     无
  */
void Referee_RXCallback(UART_HandleTypeDef* huart) {
	

    uint16_t rxdatalen = Const_Referee_RX_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
   for (uint16_t i = 0; i < rxdatalen; i++){
       if (Referee_RxData[i] == Const_Referee_FRAME_HEADER_SOF){
           Referee_DecodeRefereeData(Referee_RxData + i, rxdatalen);
		 }
	 
    }

	 __HAL_UART_CLEAR_IDLEFLAG(huart);
    /* restart dma transmission */
 //   __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_Referee_RX_BUFF_LEN);
    //HAL_DMA_Start(huart->hdmarx,(uint32_t)&huart->Instance->DR,(uint32_t)Referee_RxData,Const_Referee_RX_BUFF_LEN);
 //   __HAL_DMA_ENABLE(huart->hdmarx);
}
