/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_can.c
 *  Description  : This file contains the functions of CAN
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-05-06 19:51:17
 */


#include "util_can.h"
#include "periph_motor.h"
#include "protocol_common.h"
#include "module_dm4310.h"

CAN_RxHeaderTypeDef Can_RxHeader;
#define Const_Can_RX_BUFF_LEN  200
uint8_t Can_RxData[Const_Can_RX_BUFF_LEN];


/**
 * @brief        : CAN Error handle handling
 * @param         [uint32_t] ret
 * @return        [type]
 */
void Can_ErrorHandler(uint32_t ret) {
    //Log_DebugPrintf("Error: CAN Error!\n");
    while (1) {
        return;
    }
}


/**
 * @brief        : Initialize can transmitter
 * @param         [CAN_TxHeaderTypeDef] *pheader
 * @param         [uint32_t] stdid
 * @param         [uint32_t] extid
 * @param         [uint32_t] dlc
 * @return        [type]
 */
void Can_InitTxHeader(CAN_TxHeaderTypeDef *pheader, uint32_t stdid, uint32_t extid, uint32_t dlc) {
    pheader->StdId = stdid;
    pheader->ExtId = extid;
    pheader->RTR = CAN_RTR_DATA;
    pheader->IDE = CAN_ID_STD;          //你TM用标准帧，给扩展帧设置个0x01是何以味 (---某源)
    pheader->DLC = dlc;
    pheader->TransmitGlobalTime = DISABLE;
}


/**
 * @brief        : Initialize can filter and enable CAN Bus Transceiver
 * @param         [CAN_HandleTypeDef*] phcan
 * @return        [type]
 */
void Can_InitFilterAndStart(CAN_HandleTypeDef* phcan) {
    CAN_FilterTypeDef sFilterConfig;

    if (phcan == &hcan1)
        sFilterConfig.FilterBank = 0;
    else
        sFilterConfig.FilterBank = 14;
    
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    uint32_t ret = HAL_CAN_ConfigFilter(phcan, &sFilterConfig);
    if (ret != HAL_OK) {
        Can_ErrorHandler(ret);
    }
    
    ret = HAL_CAN_Start(phcan);
    if (ret != HAL_OK) {
        Can_ErrorHandler(ret);
    }
    
    ret = HAL_CAN_ActivateNotification(phcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    if (ret != HAL_OK) {
        Can_ErrorHandler(ret);
    }
    
}


/**
 * @brief        : Sending information to can bus
 * @param         [CAN_HandleTypeDef*] phcan
 * @param         [CAN_TxHeaderTypeDef*] pheader
 * @param         [uint8_t] txdata
 * @return        [type]
 */
void Can_SendMessage(CAN_HandleTypeDef* phcan, CAN_TxHeaderTypeDef* pheader, uint8_t txdata[]) {
    uint32_t mailbox;
    /* Start the Transmission process */
    uint32_t ret = HAL_CAN_AddTxMessage(phcan, pheader, txdata, &mailbox);
    if (ret != HAL_OK) {
        /* Transmission request Error */
        Can_ErrorHandler(ret);
    }
}

/**
************************************************************************
* @brief:      	canx_bsp_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hcan: CAN句柄
* @param:       id: 	CAN设备ID
* @param:       data: 发送的数据
* @param:       len:  发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{	
	CAN_TxHeaderTypeDef	tx_header;
	
	tx_header.StdId = id;
	tx_header.ExtId = 0;
	tx_header.IDE   = 0;
	tx_header.RTR   = 0;
	tx_header.DLC   = len;
  /*找到空的发送邮箱，把数据发送出去*/
	if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
		if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) {
			HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX2);
    }
  }
  return 0;
}

/**
************************************************************************
* @brief:      	canx_bsp_receive(CAN_HandleTypeDef *hcan, uint8_t *buf)
* @param:       hcan: CAN句柄
* @param[out]:  rec_id: 	接收到数据的CAN设备ID
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t canx_receive(CAN_HandleTypeDef *hcan, uint16_t *rec_id, uint8_t *buf)
{	
	CAN_RxHeaderTypeDef rx_header;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, buf) == HAL_OK)
	{
		*rec_id = rx_header.StdId;
		return rx_header.DLC; //接收数据长度
	}
	else
		return 0;
}

/**
  * @brief 
  * @param phcan: [输入/出] 
  * @return NULL
  * @retval NULL
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *phcan) {
		/* Get RX message */
    uint32_t ret = HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO1, &Can_RxHeader, Can_RxData);
    if (ret != HAL_OK) {
        /* Reception Error */
        Can_ErrorHandler(ret);
    }
    Can_RxMessageCallback(phcan, &Can_RxHeader, Can_RxData);
}


/**
 * @brief        : HAL_CAN_ Rx Fifo0 Message Pending Call back
 * @param         [CAN_HandleTypeDef] *phcan
 * @return        [type]
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *phcan) {
		/* Get RX message */
    uint32_t ret = HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &Can_RxHeader, Can_RxData);
    if (ret != HAL_OK) {
        /* Reception Error */
        Can_ErrorHandler(ret);
    }
    Can_RxMessageCallback(phcan, &Can_RxHeader, Can_RxData);		
}


/**
 * @brief        : Can bus data receiving callback function that updates the motor status according to the received information
 * @param         [CAN_HandleTypeDef*] phcan
 * @param         [CAN_RxHeaderTypeDef*] rxheader
 * @param         [uint8_t] rxdata
 * @return        [type]
 */
void Can_RxMessageCallback(CAN_HandleTypeDef* phcan, CAN_RxHeaderTypeDef* rxheader, uint8_t rxdata[]) {
    if (phcan == &hcan1) {
		
	  if( rxheader -> StdId==0x00)
	  {
		dm_motor_fbdata(&motor[Motor1], rxdata); 
        receive_motor_data(&motor[Motor1], rxdata);
	  }else  
        {
			Motor_EncoderDecodeCallback(phcan, rxheader -> StdId, rxdata, rxheader -> DLC);
		} 
		
		
		 
    }
    if (phcan == &hcan2) {
//        Protocol_DecodeData(rxheader->StdId, rxdata, rxheader->DLC);
        Motor_EncoderDecodeCallback(phcan, rxheader -> StdId, rxdata, rxheader -> DLC);
    }
}
