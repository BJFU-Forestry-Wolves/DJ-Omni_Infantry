/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_common.c
 *  Description  : This file contains Bus communication control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:02:53
 *  LastEditTime : 2023-05-06 08:34:28
 */


#include "protocol_common.h"
#include "protocol_receive.h"
#include "protocol_transmit.h"
#include "module_gimbal.h"
#include "lib_buff.h"
#include "stdlib.h"
#include "sys_dwt.h"
#include "util_can.h"

const uint16_t Const_Protocol_OFFLINE_TIME       = 500; 


uint8_t BusComm_TxData[100];
uint8_t BusComm_RxData[100];

// Dual bus communication protocol

Protocol_DataTypeDef Protocol_Data;


/**
  * @brief      Inter bus communication initialization
  * @param      NULL
  * @retval     NULL
  */
void Protocol_InitProtocol() {
    Protocol_ResetProtocolData();
}


/**
  * @brief      Gets the pointer to the bus communication data object
  * @param      NULL
  * @retval     Pointer to bus communication data object
  */
Protocol_DataTypeDef* Protocol_GetBusDataPtr() {
    return &Protocol_Data;
}


/**
  * @brief      Check whether the dual bus communication is offline
  * @param      NULL
  * @retval     NULL
  */
uint8_t Protocol_IsProtocolOffline() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    if (HAL_GetTick() - buscomm->last_update_time > Const_Protocol_OFFLINE_TIME) {
        buscomm->state = Protocol_STATE_LOST;
        return Protocol_STATE_LOST;
    }
    return Protocol_STATE_CONNECTED;
}

uint32_t block;
/**
  * @brief      Protocol send block error handler
  * @param      NULL
  * @retval     NULL
  */
void Protocol_SendBlockError() {
    block++;
}


/**
  * @brief      Reset inter bus communication data object
  * @param      NULL
  * @retval     NULL
  */
void Protocol_ResetProtocolData() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->last_update_time = HAL_GetTick();
}


/**
  * @brief      Data sending function of serial port in inter bus communication
  * @param      NULL
  * @retval     NULL
  */
void Protocol_SendProtocolData() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->state = Protocol_STATE_PENDING;
    uint32_t out_time = HAL_GetTick();
    static int j = 0;
    static int i = 0;
        if (i >= 3) {
			j = 0;
			i = 0;
		}
    for (; i < Const_Protocol_Transmit_BUFF_SIZE; i++) {
        if ((i == 2) && (j == 0)) {
            j = 1; break;
        }
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) {
            if (HAL_GetTick() - out_time >= 2) {
                Protocol_SendBlockError();
                return;
            }
        }
        if (ProtocolCmd_Send[i].bus_func != NULL) {
            ProtocolCmd_Send[i].bus_func(ProtocolCmd_Send[i].stdid, BusComm_TxData);
        }
    }
    buscomm->state = Protocol_STATE_CONNECTED;
}


/**
  * @brief      Data decoding function of serial port in inter bus communication
  * @param      buff: Data buffer
  * @param      rxdatalen: data length
  * @retval     NULL
  */
void Protocol_DecodeData(uint32_t stdid, uint8_t buff[], uint32_t rxdatalen) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->last_update_time = HAL_GetTick();

    memcpy(BusComm_RxData, buff, rxdatalen);
    // Data decode
    for (int i = 0; i < Const_Protocol_Receive_BUFF_SIZE; i++) {
        if ((stdid == ProtocolCmd_Receive[i].cmd_id) && (ProtocolCmd_Receive[i].bus_func != NULL)) {
           ProtocolCmd_Receive[i].bus_func(buff);
        }
    }
}



void Protocol_SetChaMode(WheelLeg_ChassisModeEnum cha_mode) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->cha_mode = cha_mode;
}

void Protocol_SetBalMode(WheelLeg_BalanceModeEnum bal_mode) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->bal_mode = bal_mode;
}

void Protocol_SetYawState(uint8_t ctrl, uint8_t out) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->yaw_control = ctrl != 0 ? 1 : 0;
    buscomm->yaw_output = out != 0 ? 1 : 0;
}

void Protocol_SetChassisRef(float len, float dx, float yaw_ref) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->yaw_ref += Gimbal_LimitYaw(yaw_ref);
    buscomm->dx = dx;
    buscomm->leg_len = len;
}
