/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : periph_remote.c
 *  Description  : This file contains RoboMaster remote relevant function
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-05-05 09:41:23
 */


#include "periph_remote.h"
#include "main.h"
#include "usart.h"
extern DMA_HandleTypeDef hdma_usart3_rx;
UART_HandleTypeDef* Const_Remote_UART_HANDLER       = &huart3;
HAL_StatusTypeDef errcode_1;
/*          Remote control related constants    */
const uint16_t Const_Remote_RX_BUFF_LEN             = 63 ;
const uint16_t Const_Remote_RX_FRAME_LEN            = 21;
const uint16_t Const_Remote_CHANNEL_VALUE_LIMIT     = 640;
const uint16_t Const_Remote_CHANNEL_VALUE_OFFSET    = 1024;
const uint16_t Const_Remote_CHANNEL_ERROR_LIMIT     = 700;
const uint16_t Const_Remote_REMOTE_OFFLINE_TIME     = 1000;

uint8_t Remote_RxData[Const_Remote_RX_BUFF_LEN];
Remote_RemoteDataTypeDef Remote_RemoteData;


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//      if(huart==&huart3){
//      Remote_DecodeRemoteData(Remote_RxData, 18);
//			HAL_UART_Receive_IT(&huart3,Remote_RxData,18);
//			}

//}
/**
  * @brief      Gets the pointer of the remote control object
  * @param      NULL
  * @retval     Pointer to remote control object
  */
Remote_RemoteDataTypeDef* Remote_GetRemoteDataPtr() {
    return &Remote_RemoteData;
}


/**
  * @brief      Initialize remote control
  * @param      NULL
  * @retval     NULL
  */

void Remote_InitRemote() {
    Remote_ResetRemoteData();
    Remote_RemoteData.last_update_time = HAL_GetTick();
    Uart_InitUartDMA(Const_Remote_UART_HANDLER);
    Uart_ReceiveDMA(Const_Remote_UART_HANDLER, Remote_RxData, Const_Remote_RX_BUFF_LEN);
}


/**
  * @brief      Switch position state
  * @param      sw: Original switch value
  * @retval     Switch position status
  */
Remote_SwitchStateEnum Remote_ToSwitchState(uint8_t sw) {
    return (Remote_SwitchStateEnum) sw;
}


/**
  * @brief      Judge whether the remote control is offline
  * @param      rc: pointer to remote control object
  * @retval     Offline or not (1 is yes, 0 is no)
  */
uint8_t Remote_IsRemoteOffline() {
    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();
    uint32_t now = HAL_GetTick();
    if ((now - rc->last_update_time) > Const_Remote_REMOTE_OFFLINE_TIME)
        rc->state = Remote_STATE_LOST;
    return rc->state == Remote_STATE_CONNECTED;
}

uint16_t testrxdatalen;
/**
  * @brief      Remote control receiving callback function
  * @param      huart: Pointer to UART handle
  * @retval     NULL
  */
void Remote_RXCallback(UART_HandleTypeDef* huart) {
    
    /* handle uart data from DMA */
    uint16_t rxdatalen = Const_Remote_RX_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
	testrxdatalen =  rxdatalen;
	for (uint16_t i = 0; i < rxdatalen; i++){
       if (Remote_RxData[i] == 0xA9){
          Remote_DecodeRemoteData(Remote_RxData+i, rxdatalen-i);
	   }
   }
    /* restart dma transmission */
   __HAL_UART_CLEAR_IDLEFLAG(huart);
}



/**
  * @brief      Remote control keyboard data decoding
  * @param      key: Remote control keyboard data object
  * @param      v: Original remote control keyboard data value
  * @retval     NULL
  */
void Remote_DecodeKeyboardData(Remote_KeyboardTypeDef* key, uint16_t v) {
    const uint16_t KEY_MASK_W       = 1 << 0;
    const uint16_t KEY_MASK_S       = 1 << 1;
    const uint16_t KEY_MASK_A       = 1 << 2;
    const uint16_t KEY_MASK_D       = 1 << 3;
    const uint16_t KEY_MASK_SHIFT   = 1 << 4;
    const uint16_t KEY_MASK_CTRL    = 1 << 5;
    const uint16_t KEY_MASK_Q       = 1 << 6;
    const uint16_t KEY_MASK_E       = 1 << 7;
    const uint16_t KEY_MASK_R       = 1 << 8;
    const uint16_t KEY_MASK_F       = 1 << 9;
    const uint16_t KEY_MASK_G       = 1 << 10;
    const uint16_t KEY_MASK_Z       = 1 << 11;
    const uint16_t KEY_MASK_X       = 1 << 12;
    const uint16_t KEY_MASK_C       = 1 << 13;
    const uint16_t KEY_MASK_V       = 1 << 14;
    const uint16_t KEY_MASK_B       = 1 << 15;

    key->w      = (v & KEY_MASK_W    ) > 0;
    key->s      = (v & KEY_MASK_S    ) > 0;
    key->a      = (v & KEY_MASK_A    ) > 0;
    key->d      = (v & KEY_MASK_D    ) > 0;
    key->shift  = (v & KEY_MASK_SHIFT) > 0;
    key->ctrl   = (v & KEY_MASK_CTRL ) > 0;
    key->q      = (v & KEY_MASK_Q    ) > 0;
    key->e      = (v & KEY_MASK_E    ) > 0;
    key->r      = (v & KEY_MASK_R    ) > 0;
    key->f      = (v & KEY_MASK_F    ) > 0;
    key->g      = (v & KEY_MASK_G    ) > 0;
    key->z      = (v & KEY_MASK_Z    ) > 0;
    key->x      = (v & KEY_MASK_X    ) > 0;
    key->c      = (v & KEY_MASK_C    ) > 0;
    key->v      = (v & KEY_MASK_V    ) > 0;
    key->b      = (v & KEY_MASK_B    ) > 0;
}

int16_t TEST;
/**
  * @brief      Remote control decoding function
  * @param      rc: The pointer points to the remote control data object
  * @param      buff: data buff
  * @param      rxdatalen: Data length
  * @retval     NULL
  */
void Remote_DecodeRemoteData(uint8_t* buff, int rxdatalen) {
    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();

    if (rxdatalen != Const_Remote_RX_FRAME_LEN) {
        return;                                     //Data length error
    }
    // 2. 帧头校验：固定值为 0xA9 和 0x53 
    if (buff[0] != 0xA9 || buff[1] != 0x53) {
        return;
    }
    rc->state           = Remote_STATE_PENDING;
    rc->last_update_time = HAL_GetTick();   
    rc->remote.ch[0] = Remote_CancelChannelOffset(((uint16_t)buff[2] | (uint16_t)buff[3] << 8) & 0x07FF); 
    TEST =	Remote_CancelChannelOffset(((uint16_t)buff[2] | (uint16_t)buff[3] << 8) & 0x07FF);
    rc->remote.ch[1] = Remote_CancelChannelOffset(((uint16_t)buff[3] >> 3 | (uint16_t)buff[4] << 5) & 0x07FF);    
    rc->remote.ch[2] = Remote_CancelChannelOffset(((uint16_t)buff[4] >> 6 | (uint16_t)buff[5] << 2 | (uint16_t)buff[6] << 10) & 0x07FF);    
    rc->remote.ch[3] = Remote_CancelChannelOffset(((uint16_t)buff[6] >> 1 | (uint16_t)buff[7] << 7) & 0x07FF);
    rc->remote.s[0] = Remote_ToSwitchState((buff[7] >> 4) & 0x03);
	rc->keskey.pause_btn= (buff[7] >> 6) & 0x01; //暂停按钮
    rc->keskey.custom_l = (buff[7] >> 7) & 0x01; //自定义按键-左
    rc->keskey.custom_r = (buff[8] & 0x01);      //自定义按键-右
	rc->keskey.trigger_btn = (buff[9] >> 4) & 0x01; //扳机键
	// 拨轮数据 (Channel 4) [cite: 296]
    rc->remote.ch[4] = ((uint16_t)buff[8] >> 1 | (uint16_t)buff[9] << 7) & 0x07FF;
    rc->remote.ch[4] = Remote_CancelChannelOffset(rc->remote.ch[4]);
	
    rc->mouse.x = ((int16_t)buff[10] | (int16_t)buff[11] << 8);
    rc->mouse.y = ((int16_t)buff[12] | (int16_t)buff[13] << 8);
    rc->mouse.z = ((int16_t)buff[14] | (int16_t)buff[15] << 8);
    // 鼠标按键 [cite: 296]
    rc->mouse.l = buff[16] & 0x03;
    rc->mouse.r = (buff[16] >> 2) & 0x03;
	uint8_t mouse_mid = (buff[16] >> 4) & 0x03;
   /* --- 键盘数据解析 (偏移量 136 bits -> buff[17]) --- [cite: 296] */
    Remote_DecodeKeyboardData(&(rc->key), ((uint16_t)buff[17]) | ((uint16_t)buff[18] << 8));
   
    rc->state           = Remote_STATE_CONNECTED;
}


/**
  * @brief      Initialize remote control data
  * @param      rc: Pointer to remote control object
  * @retval     NULL
  */
void Remote_ResetRemoteData() {
    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();
    for (int i = 0; i < 5; ++i)
        rc->remote.ch[i] = 0;
    for (int i = 0; i < 2; ++i)
        rc->remote.s[i] = Remote_ToSwitchState(0);
    rc->mouse.x = 0;
    rc->mouse.y = 0;
    rc->mouse.z = 0;
    rc->mouse.l = 0;
    rc->mouse.r = 0;
    Remote_DecodeKeyboardData(&(rc->key), 0);
}


/**
  * @brief      Remove remote control offset
  * @param      ch: Original channel value
  * @retval     True value
  */
int16_t Remote_CancelChannelOffset(uint16_t ch) {
    return (int16_t) ch - Const_Remote_CHANNEL_VALUE_OFFSET;
}
