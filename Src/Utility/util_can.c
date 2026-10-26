/*
 * 文件：util_can.c
 * CAN硬件收发封装与分发。区分0x006达妙单电机帧和0x1FF普通电机组帧。
 * 本轮只整理注释，保持上一版恢复及局部命名修正后的可执行代码不变。
 */

#include "util_can.h"
#include "periph_motor.h"
#include "protocol_common.h"
#include "module_dm4310.h"

CAN_RxHeaderTypeDef Can_RxHeader;
#define Const_Can_RX_BUFF_LEN  200
uint8_t Can_RxData[Const_Can_RX_BUFF_LEN];

/**
 * 当前错误入口实际立即return，不打印、不等待也不重试；不要将while(1)理解为死循环处理。
 */
void Can_ErrorHandler(uint32_t ret) {
    //Log_DebugPrintf("Error: CAN Error!\n");
    while (1) {
        return;
    }
}

/**
 * 生成标准数据帧头；虽然保存extid字段，IDE固定为CAN_ID_STD，因此不是扩展帧发送。
 */
void Can_InitTxHeader(CAN_TxHeaderTypeDef *pheader, uint32_t stdid, uint32_t extid, uint32_t dlc) {
    pheader->StdId = stdid;
    pheader->ExtId = extid;
    pheader->RTR = CAN_RTR_DATA;
    pheader->IDE = CAN_ID_STD;
    pheader->DLC = dlc;
    pheader->TransmitGlobalTime = DISABLE;
}

/**
 * CAN1使用滤波器组0，CAN2使用组14；全零掩码接收所有ID到FIFO0。
 * 启动CAN并使能FIFO0接收中断，实际电机分流在Can_RxMessageCallback内完成。
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
 * 提交发送请求到CAN硬件邮箱；失败时调用当前空错误处理入口，不会自动重试。
 * HAL_OK仅表示请求成功提交，不代表已收到电机反馈。
 */
void Can_SendMessage(CAN_HandleTypeDef* phcan, CAN_TxHeaderTypeDef* pheader, uint8_t txdata[]) {
    uint32_t mailbox;

    uint32_t ret = HAL_CAN_AddTxMessage(phcan, pheader, txdata, &mailbox);
    if (ret != HAL_OK) {

        Can_ErrorHandler(ret);
    }
}

/**
 * 达妙等接口使用的标准帧发送封装；返回HAL_CAN_AddTxMessage状态，未等待总线发送完成。
 */
uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;
    Can_InitTxHeader(&tx_header, id, 0, len);
    return (uint8_t)HAL_CAN_AddTxMessage(hcan, &tx_header, data, &mailbox);
}

/**
 * 主动从FIFO0取一帧，成功时输出标准ID并返回DLC，失败返回0；不要与同FIFO中断接收争抢帧。
 */
uint8_t canx_receive(CAN_HandleTypeDef *hcan, uint16_t *rec_id, uint8_t *buf)
{
	CAN_RxHeaderTypeDef rx_header;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, buf) == HAL_OK)
	{
		*rec_id = rx_header.StdId;
		return rx_header.DLC;
	}
	else
		return 0;
}

/**
 * 取出FIFO1的一帧并分发；当前初始化实际只开启FIFO0接收通知。
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *phcan) {

    uint32_t ret = HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO1, &Can_RxHeader, Can_RxData);
    if (ret != HAL_OK) {

        Can_ErrorHandler(ret);
    }
    Can_RxMessageCallback(phcan, &Can_RxHeader, Can_RxData);
}

/**
 * 取出FIFO0的一帧并转交统一分发；当前实现接收失败后仍会继续分发原缓冲区。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *phcan) {

    uint32_t ret = HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &Can_RxHeader, Can_RxData);
    if (ret != HAL_OK) {

        Can_ErrorHandler(ret);
    }
    Can_RxMessageCallback(phcan, &Can_RxHeader, Can_RxData);
}

/**
 * CAN1/标准ID0x06分给达妙P轴，其他帧按DJI反馈ID/电容ID处理；CAN2走普通电机分发。
 * 0x06匹配驱动Master ID，不是组帧0x1FF，也不是对主控发送ID字段的自动读取。
 * 现代码对0x06帧先调用普通反馈解码，再调用参数应答解码；未严格区分两类报文及检查DLC。
 */
void Can_RxMessageCallback(CAN_HandleTypeDef* phcan, CAN_RxHeaderTypeDef* rxheader, uint8_t rxdata[]) {
    if (phcan == &hcan1) {

	  if( rxheader -> StdId==0x06)  // 以驱动Master ID分流反馈；当前未额外验证帧类型和数据长度。
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
