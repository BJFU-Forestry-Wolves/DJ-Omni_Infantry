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
#include <stdio.h>
#include <stdint.h>
#include <string.h>

UART_HandleTypeDef* Const_Referee_UART_HANDLER          = &huart6;
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
    referee->SyncTimeStamp =   struct_ptr->SyncTimeStamp;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_game_result(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_game_result_t *struct_ptr = data_ptr;
	referee->winner = struct_ptr->winner;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_game_robot_HP(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_game_robot_HP_t *struct_ptr = data_ptr;
	referee->Teammate_HP.Hero_HP =struct_ptr->ally_1_robot_HP;
	referee->Teammate_HP.Sentinel_HP =struct_ptr->ally_7_robot_HP;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_referee_warning(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_referee_warning_t *struct_ptr = data_ptr;
	referee->waring.level =struct_ptr->level;
	referee->waring.offending_robot_id = struct_ptr->offending_robot_id ;
	referee->waring.count = struct_ptr->count;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_game_robot_status(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_game_robot_status_t *struct_ptr = data_ptr;
	

	referee->status.robot_id = struct_ptr->robot_id;
    referee->status.robot_level = struct_ptr->robot_level;
    referee->status.current_HP =  struct_ptr->current_HP;
    referee->status.max_chassis_power = struct_ptr->chassis_power_limit;
    referee->status.mains_power_gimbal_output = struct_ptr->power_management_gimbal_output;
    referee->status.mains_power_chassis_output = struct_ptr->power_management_chassis_output;//底盘功率上限
    referee->status.mains_power_shooter_output = struct_ptr->power_management_shooter_output;
    referee->status.shooter_limit = struct_ptr->shooter_barrel_heat_limit;
    referee->client_id = Referee_GetClientIDByRobotID(referee->status.robot_id);   
    if (referee->status.robot_id != struct_ptr->robot_id) {
        referee->status.robot_id = struct_ptr->robot_id;
        referee->client_id = Referee_GetClientIDByRobotID(referee->status.robot_id);		
    }
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_power_heat_data(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_power_heat_data_t *struct_ptr = data_ptr;
    referee->status.shoot_heat = struct_ptr->shooter_17mm_barrel_heat;
    referee->status.buffer_energy = struct_ptr->buffer_energy;
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_game_robot_pos(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_game_robot_pos_t *struct_ptr = data_ptr;
    
    referee->x = struct_ptr->x;
    referee->y = struct_ptr->y;
    referee->angle = struct_ptr->angle;   
    
    return PARSE_SUCCEEDED;
}


uint8_t P_ext_robot_hurt(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_robot_hurt_t *struct_ptr = data_ptr;

	 referee->who_shoot_me.armor_id = struct_ptr->armor_id ;
	 referee->who_shoot_me.HP_deduction_reason =struct_ptr->HP_deduction_reason;
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_shoot_data(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_shoot_data_t *struct_ptr = data_ptr;
	
	  referee->what_can_I_shoot.type = struct_ptr->bullet_type;
	  referee->what_can_I_shoot.number = struct_ptr->shooter_number; 
      referee->what_can_I_shoot.frequency = struct_ptr->launching_frequency;
      referee->what_can_I_shoot.speed =      struct_ptr->initial_speed;
    
    return PARSE_SUCCEEDED;
}


/********** END OF REFEREE CMD PARSER FUNCTION **********/


const uint16_t Const_Referee_FRAME_HEADER_SOF       = 0xA5;     // 裁判系统指令帧头长度
const Referee_RobotAndClientIDTypeDef RobotClientTable[] = {
    {1,   0x0101}, {2,   0x0102}, {3,   0x0103}, {4,   0x0104}, {5,   0x0105}, {6,   0x0106}, {7,   0}, // 红方
    {101, 0x0165}, {102, 0x0166}, {103, 0x0167}, {104, 0x0168}, {105, 0x0169}, {106, 0x016A}, {107, 0}  // 蓝方
};
#define ROBOT_CLIENT_TABLE_SIZE (sizeof(RobotClientTable) / sizeof(Referee_RobotAndClientIDTypeDef))   //计算表的大小     
const uint16_t Const_Referee_CMD_NUM                = 20;       // 裁判系统指令个数（不含交互指令）
const Referee_RefereeCmdTypeDef Const_Referee_CMD_LIST[Const_Referee_CMD_NUM] = {           // 裁判系统消息命令ID列表
    {0x0001,    11, &P_ext_game_status},                // 比赛状态数据，1Hz 周期发送
    {0x0002,    1,  &P_ext_game_result},                // 比赛结果数据，比赛结束后发送
    {0x0003,    16, &P_ext_game_robot_HP},              // 比赛机器人血量数据，1Hz 周期发送
//    {0x0101,    4,  &P_ext_event_data},                 // 场地事件数据，事件改变后发送
    {0x0104,    3,  &P_ext_referee_warning},            // 裁判警告数据，警告发生后发送   
    {0x0201,    13, &P_ext_game_robot_status},          // 机器人状态数据，10Hz 周期发送
    {0x0202,    14, &P_ext_power_heat_data},            // 实时功率热量数据，50Hz 周期发送
    {0x0203,    16, &P_ext_game_robot_pos},             // 机器人位置数据，10Hz 发送
//    {0x0204,    8,  &P_ext_buff},                       // 机器人增益数据，1Hz 周期发送
    {0x0206,    1,  &P_ext_robot_hurt},                 // 伤害状态数据，伤害发生后发送
    {0x0207,    7,  &P_ext_shoot_data},                 // 实时射击数据，子弹发射后发送
//    {0x0209,    5,  &P_ext_rfid_status},	// 机器人RFID状态，3Hz周期发送
//	{0x0308,    34,  &P_ext_rfid_status},
};

const Referee_RefereeCmdTypeDef Const_Referee_CMD_INTERACTIVE       = {0x0301, 8, NULL};    // 机器人间交互数据，发送方触发发送
// 注：这里的6是交互数据帧头的长度，因为交互数据帧是不定长的
//const uint16_t Const_Referee_DATA_CMD_ID_CLIENT_CUSTOM_DATA       = 0xD180;               // （已废弃）客户端自定义数据内容ID
const uint16_t Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND    = 0x0200;               // 机器人间交互数据内容ID下界
const uint16_t Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND    = 0x02FF;               // 机器人间交互数据内容ID上界
const uint16_t Const_Referee_DATA_INTERACTIVE_DATA_MAX_LENGTH       = 113 - 1;              // 机器人间交互数据内容最大长度




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
  for (int i = 0; i < ROBOT_CLIENT_TABLE_SIZE; i++) {
        if (RobotClientTable[i].robot_id == robot_id) {
            return RobotClientTable[i].client_id;
        }
    }
   return 0; // 未找到匹配的 ID
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
    
    robot_interaction_data_t *header = (void *) (buf + 7);
    header->data_cmd_id  = data_cmd_id;                   //0x0301后放入子内容，绘制多少个图形
    header->receiver_ID  = receiver_ID;                    //接收者ID
    header->sender_ID    = (uint16_t)referee->status.robot_id;    //发送者ID
    
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
    
    robot_interaction_data_t *header_struct_ptr = (void *) data;
    if (header_struct_ptr->data_cmd_id < Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND || 
        header_struct_ptr->data_cmd_id > Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND) 
        return PARSE_FAILED;    // wrong data cmd id
    if (header_struct_ptr->receiver_ID != referee->status.robot_id) return PARSE_FAILED;           // wrong receiver id
    
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


uint8_t testm=0;
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
		testm=2;
        return;
    }
    
    uint16_t data_length = (uint16_t) buff[2] << 8 | buff[1];
    uint8_t seq = buff[3];

    if (!CRC_VerifyCRC16CheckSum(buff, data_length + 9)) {
        referee->state          = Referee_STATE_ERROR;
		testm=4;
        return;
    }
    
    uint16_t cmd_id = (uint16_t) buff[6] << 8 | buff[5];
	Referee_ParseRefereeCmd(cmd_id, buff + 7, data_length);
 //   if (!Referee_ParseRefereeCmd(cmd_id, buff + 7, data_length)) {  //这个是cmd命令的作用处
 //       referee->state          = Referee_STATE_ERROR;
 //       return;
 //   }        可以先不判断，因为有些cmd我们根本不需要用
    referee->state              = Referee_STATE_CONNECTED;  // 解锁
}


/**
  * @brief      裁判系统串口回调函数
  * @param      huart: 指针指向串口句柄
  * @retval     无
  */
void Referee_RXCallback(UART_HandleTypeDef* huart) {
    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* handle dbus data dbus_buf from DMA */

    uint16_t rxdatalen = Const_Referee_RX_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
   for (uint16_t i = 0; i < rxdatalen; i++){
       if (Referee_RxData[i] == Const_Referee_FRAME_HEADER_SOF){
		   uint16_t pkg_data_len = (uint16_t)Referee_RxData[i+2] << 8 | Referee_RxData[i+1];
           uint16_t total_len = pkg_data_len + 9;
           Referee_DecodeRefereeData(Referee_RxData + i, rxdatalen - i);
		   i += total_len;
        }
    }
   //Referee_DecodeRefereeData(Referee_RxData, rxdatalen);
	
    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_Referee_RX_BUFF_LEN);
    //HAL_DMA_Start(huart->hdmarx,(uint32_t)&huart->Instance->DR,(uint32_t)Referee_RxData,Const_Referee_RX_BUFF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}
