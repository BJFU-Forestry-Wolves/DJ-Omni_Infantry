/*
 *  Project      :  
 *  
 *  FilePath     : util_debug.h
 *  Description  : 
 *  LastEditors  : Mr.Lee
 *  Date         : 2024年10月3日23:18:23
 *  LastEditTime : 
 */


#ifndef UTIL_DEBUG_H
#define UTIL_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#define VOFA_DATAPACK_HEAD      '='     /* VOFA命令帧头 */
#define VOFA_DATAPACK_END       '\n'    /* VOFA命令帧尾 */
#define VOFA_DATAPACK_MAXLEN    200     /* VOFA命令最大长度 */

#define VOFA_JUSTFLOAT_CHANNEL_NUM  7       // 发送5个通道数据（对应原来的param1-param5）
#define VOFA_JUSTFLOAT_FLOAT_BYTES  4       // 每个float占4字节
#define VOFA_JUSTFLOAT_TAIL_BYTES   4       // 帧尾占4字节
#define VOFA_JUSTFLOAT_TOTAL_LEN    (VOFA_JUSTFLOAT_CHANNEL_NUM * VOFA_JUSTFLOAT_FLOAT_BYTES + VOFA_JUSTFLOAT_TAIL_BYTES)

// JustFloat帧尾（小端字节序：对应32位值0x7F800000）
static const uint8_t VOFA_JUSTFLOAT_TAIL[VOFA_JUSTFLOAT_TAIL_BYTES] = {0x00, 0x00, 0x80, 0x7F};


// 定义接收缓冲区大小（根据上位机发送的数据长度调整，如256字节足够大部分场景）
#define UART6_RX_BUF_SIZE 256

// 全局接收缓冲区（存储原始数据）
extern uint8_t Uart6_Rx_Buf[UART6_RX_BUF_SIZE];
// 有效数据长度（空闲中断中获取，记录上位机实际发送的字节数）
extern uint16_t Uart6_Rx_Len;
// 接收完成标志位（用于主循环/其他函数判断是否有新数据）
extern uint8_t Uart6_Rx_Complete_Flag;
extern volatile uint8_t g_uart6_print_flag ; 

uint16_t DMACurrentDataCounter(DMA_Stream_TypeDef *dma_stream);



void usart_printf(const char *fmt,...);
void Debug_Init();
void Debug_RXCallback(UART_HandleTypeDef* huart);
void vofa_justfloat_send(float *data_buf, uint8_t channel_num);

#endif

#ifdef __cplusplus
}

#endif
