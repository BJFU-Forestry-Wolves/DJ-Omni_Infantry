/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_receive.c
 *  Description  : This file is for receive communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:10:30
 *  LastEditTime : 2023-05-06 08:43:35
 */


#include "protocol_common.h"
#include "protocol_receive.h"
#include "lib_buff.h"
#include "sys_dwt.h"


static void _set_state_(uint8_t *buff);
static void _send_cap_data(uint8_t *buff);

const uint32_t CMD_CHASSIS_SET_STATE               = 0x203;
const uint32_t CMD_CHASSIS_SET_CAP                 = 0x204;


Protocol_ReceiveEntry ProtocolCmd_Receive[Const_Protocol_Receive_BUFF_SIZE] = {
    {CMD_CHASSIS_SET_STATE,           &_set_state_         },
    {CMD_CHASSIS_SET_CAP  ,           &_send_cap_data     },
};

uint32_t dt_tick1;
float dt_1;
static void _set_state_(uint8_t *buff) {
    dt_1 = DWT_GetDeltaT(&dt_tick1);
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->bal_real_mode = (WheelLeg_BalanceModeEnum)buff[0];
}

uint32_t dt_tick2;
float dt_2;
static void _send_cap_data(uint8_t *buff) {
    dt_2 = DWT_GetDeltaT(&dt_tick2);
}
