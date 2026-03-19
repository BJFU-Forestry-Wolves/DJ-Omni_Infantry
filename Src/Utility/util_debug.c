/*
 *  Project      : 此文件第一代作者是宪哥
 *
 *  FilePath     : util_debug.c
 *  Description  :
 *  LastEditors  : Mr.Lee
 *  Date         : 2024年10月3日23:18:23
 *  LastEditTime :
 */


/**
* @file util_debug.c
* @brief 第二代作者是Yuyuan，
* @details 此文件用于VOFA调试
* @author Yuyuan
* @version V1.0
* @date 2026-01-02  15：55
* @attention  函数在app_Debug.c里面调用，不用修改直接可以使用，内置了FireWater，JustFloat两种协议，还有命令解析
*             也就是说，你不仅可以打印出想要的数据在VOFA上面形成波形图（可视化PID），而且你还能用VOFA的命令实时改变PID，根据波形图迅速调整
*              你的KP,KD,KI。还不快TMD谢谢YUYUAN
*/
#include "util_debug.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "app_Debug.h"
#include "app_remote.h"
#include "app_autoaim.h"


#define Const_Debug_RX_BUFF_LEN             	54
#define 			Debug_Uart				huart6
#define PRINT_TX_BUF_SIZE 256              // 发送缓冲区大小
#define VOFA_JUSTFLOAT_TX_BUF_SIZE  64  // 足够容纳8个通道+帧尾

UART_HandleTypeDef *Const_Debug_UART_HANDLER       = &huart6;
uint8_t Debug_RxData[54];


static uint8_t g_print_tx_buf[PRINT_TX_BUF_SIZE];
static volatile uint8_t g_dma_tx_busy = 0; // DMA发送忙标记：0-空闲，1-忙碌
volatile uint8_t g_uart6_print_flag = 0; // 10ms打印标志位：0-未到时机，1-可打印


uint8_t g_ucVofaBuf[VOFA_DATAPACK_MAXLEN];
static float vofa_get_data(uint8_t, uint8_t);
static void vofa_set_sram_data(uint8_t, float);
static uint8_t g_vofa_justfloat_tx_buf[VOFA_JUSTFLOAT_TX_BUF_SIZE];



/**
  * @brief 用于VOFA的FireWater协议，但是会大量占用CPU资源，而且freertos会被卡住
  *         不过YUYUAN还是舍不得删掉，就保留了这个函数
  * @param fmt
  * @return NULL
  * @retval NULL
  */
//使用方法：usart_printf("%d,%d,%d,%d,%d\r\n",param1,param2,param3,param4,param5);      \r\n不可以删掉
void usart_printf(const char *fmt, ...)
{
    va_list ap;
    uint16_t len;

    // 若DMA正在发送，直接返回
    if (g_dma_tx_busy == 1)
    {
        return;
    }

    // 1. 格式化数据到发送缓冲区（使用vsnprintf避免缓冲区溢出）
    va_start(ap, fmt);
    len = vsnprintf((char *)g_print_tx_buf, PRINT_TX_BUF_SIZE - 1, fmt, ap);
    va_end(ap);
    g_print_tx_buf[len] = '\0'; // 确保字符串结束

    // 2. 标记DMA发送忙，启动DMA非阻塞发送
    g_dma_tx_busy = 1;
    HAL_UART_Transmit_DMA(&Debug_Uart, g_print_tx_buf, len);
}





/**
  * @brief 适配VOFA的JustFloat协议，发送速度快，不会影响freertos
  * @param  
  * @return NULL
  * @retval NULL
  */
void vofa_justfloat_send(float *data_buf, uint8_t channel_num)
{
    // 1. 合法性校验
    if (data_buf == NULL || channel_num == 0 || g_dma_tx_busy == 1)
    {
        return; // 空指针/无通道/DMA忙碌时直接返回
    }

    // 2. 计算总长度（通道数据+帧尾）
    uint16_t total_len = channel_num * 4 + 4;
    if (total_len > VOFA_JUSTFLOAT_TX_BUF_SIZE)
    {
        return; // 缓冲区不足，避免溢出
    }

    // 3. 组装float数据（二进制拷贝，小端序直接映射）
    uint8_t *tx_ptr = g_vofa_justfloat_tx_buf;
    for (uint8_t i = 0; i < channel_num; i++)
    {
        // 将float强制转换为uint8_t指针，逐字节拷贝
        memcpy(tx_ptr, &data_buf[i], 4);
        tx_ptr += 4; // 指针偏移4字节，指向下一个通道
    }

    // 4. 拼接帧尾
    memcpy(tx_ptr, VOFA_JUSTFLOAT_TAIL, 4);

    // 5. 启动DMA发送
    g_dma_tx_busy = 1;
    HAL_UART_Transmit_DMA(&Debug_Uart, g_vofa_justfloat_tx_buf, total_len);
}



 //vofa发送给单片机的数据 ，这些函数你不需要动
void vofa_set_data(uint8_t _rx_byte)     
{
    static uint8_t end_pos = 0;
    static uint8_t head_pos = 0;


    g_ucVofaBuf[end_pos] = _rx_byte;

    if (_rx_byte == VOFA_DATAPACK_HEAD)
    {
        head_pos = end_pos;
    }
    else if(_rx_byte == VOFA_DATAPACK_END && g_ucVofaBuf[head_pos] == VOFA_DATAPACK_HEAD)
    {

        float VofaData = vofa_get_data(head_pos, end_pos);


        vofa_set_sram_data(head_pos, VofaData);


        end_pos = 0;
        head_pos = 0;
        memset(g_ucVofaBuf, 0x00, VOFA_DATAPACK_MAXLEN);
    }


    if(++end_pos > VOFA_DATAPACK_MAXLEN)
    {
        end_pos = 0;
        memset(g_ucVofaBuf, 0x00, VOFA_DATAPACK_MAXLEN);
    }
}



static float vofa_get_data(uint8_t _head, uint8_t _end)
{
    char _VofaData[_end - (_head + 1)];
    for(uint8_t i = 0; i < _end; i++)
        _VofaData[i] = g_ucVofaBuf[i + _head + 1];

    return atof(_VofaData);
}



/**
  * @brief  按照此格式,YP=%.2f\n，发送这个命令YP识别，\n结束，中间是VOFA发送的数据，会自动替换成需要改变的数据
  * @param 
  * @return NULL
  * @retval NULL
  */
static void vofa_set_sram_data(uint8_t _head, float _data)
{
    uint8_t _id_pos1 = _head - 2;   /* 锟斤拷锟捷号的碉拷1位 - P/I/... */
    uint8_t _id_pos2 = _head - 1;   /* 锟斤拷锟捷号的碉拷2位 - 1/2/3/4/...*/


    /* P1 - g_tAnglePID.kp */
    if (g_ucVofaBuf[_id_pos1] == 'Y' && g_ucVofaBuf[_id_pos2] == 'P')
    {
        YAKp = _data;
    }
    /* D1 - g_tAnglePID.kd */
    else if (g_ucVofaBuf[_id_pos1] == 'Y' && g_ucVofaBuf[_id_pos2] == 'D')
    {
        YAKd = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'A' && g_ucVofaBuf[_id_pos2] == 'P')
    {
        PAKd = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'S' && g_ucVofaBuf[_id_pos2] == 'P')
    {
        PSKd = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'P' && g_ucVofaBuf[_id_pos2] == 'S')
    {
        PSKp = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'P' && g_ucVofaBuf[_id_pos2] == 'A')
    {
        PAKp = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'M' && g_ucVofaBuf[_id_pos2] == 'T')
    {
        visionDataGet.yaw_angle.yaw_predict = _data;
    }



}

/**
  * @brief   利用UART6的空闲中断DMA，这个函数用于使能DMA
  * @param : [输入/出]
  * @return NULL
  * @retval NULL
  */
void Debug_Init()
{

    /*	初始化DMA通道	*/    //其实这是空闲中断的配置
    __HAL_UART_CLEAR_IDLEFLAG(Const_Debug_UART_HANDLER);
    __HAL_UART_ENABLE_IT(Const_Debug_UART_HANDLER, UART_IT_IDLE);



    uint32_t tmp1 = 0;


    tmp1 = Const_Debug_UART_HANDLER->RxState;
    if(tmp1 == HAL_UART_STATE_READY)
    {

        Const_Debug_UART_HANDLER->pRxBuffPtr = Debug_RxData;
        Const_Debug_UART_HANDLER->RxXferSize = 54;
        Const_Debug_UART_HANDLER->ErrorCode  = HAL_UART_ERROR_NONE;
        /* 打开DMA接收通道 */
        HAL_DMA_Start(Const_Debug_UART_HANDLER->hdmarx, (uint32_t)&Const_Debug_UART_HANDLER->Instance->DR, (uint32_t)Debug_RxData, 54);
        /*
         * 启动 DMA 控制器，使其将 UART 接收到的数据自动存储到 Debug_RxData 缓冲区
         * USART_CR3_DMAR 是 UART 控制寄存器 3 (CR3) 中的一个标志位，用来启用 DMA 接收请求。
         */
        SET_BIT(Const_Debug_UART_HANDLER->Instance->CR3, USART_CR3_DMAR);

    }
}

/**
  * @brief      Debug接收回调函数
  * @param      你可以自己ctrl+F查找在哪里调用的
  * @retval     当UART6接收到VOFA发来的数据，就会执行这个函数
  */
int test_lens;
void Debug_RXCallback(UART_HandleTypeDef *huart)
{
    /* 关闭DMA */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* 处理DMA中传输回来的数据 */
    int rxdatalen = Const_Debug_RX_BUFF_LEN - DMACurrentDataCounter(huart->hdmarx->Instance);
    test_lens = rxdatalen;
    if (test_lens > 0)   // 避免长度为0时无效循环
    {
        for (int i = 0; i < test_lens; i++)
        {
            vofa_set_data(Debug_RxData[i]); // 逐个字节传入VOFA解析函数
        }
        //    }
        /* 重新打开DMA */
        __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_Debug_RX_BUFF_LEN);
        __HAL_DMA_ENABLE(huart->hdmarx);
    }
}


/**
  * @brief      获取当前DMAy Streamx传输中剩余的数据单元数
  * @param      dma_stream: 指向 `DMA_Stream_TypeDef` 结构的指针，
  *             其中y可以是1或2，表示选择的DMA控制器，
  *             x可以是0到7，表示选择的DMA Stream。
  * @retval     当前DMAy Streamx传输中剩余的数据单元数。
  * @note       该函数通过读取DMA流结构体的NDTR寄存器值，获取当前DMA传输中尚未传输的数据单元数。
  *             NDTR寄存器（Number of Data to Transfer Register）存储了DMA传输剩余的数据单元数，
  *             该寄存器的值会在每次数据传输后自动递减。
  */
uint16_t DMACurrentDataCounter(DMA_Stream_TypeDef *dma_stream)
{
    /* 返回当前DMAy Streamx传输中剩余的数据单元数 */
    return ((uint16_t)(dma_stream->NDTR));
}





/**
  * @brief  用于UART6的TX的回调，重置DMA状态，让DMA继续搬运数据
  * @param huart: [输入/出]
  * @return NULL
  * @retval NULL
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 仅处理USART6的发送完成回调
    if (huart->Instance == Debug_Uart.Instance)
    {
        g_dma_tx_busy = 0; // 重置DMA发送状态，允许下一次打印
    }
}

