#include "app_autoaim.h"
#include "periph_referee.h"
//--------------------------------------------------------------------------------------------------------
//																						外部变量和函数
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

//extern shoot_control_t shoot_control;
//extern gimbal_control_t gimbal_control;
extern INS_INSTypeDef INS;
extern fp32 INS_angle[3];
extern fp32 INS_accel[3];
extern bool_t gimbal_cmd_to_chassis_stop(void);
extern int shoot_speed_add_little;
//--------------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------------
//																						变量定义
__align(8) uint8_t USART1_TX_BUF[USART1_MAX_SEND_LEN]; 	 //发送缓冲,最大USART1_MAX_SEND_LEN(400)字节
uint8_t USART1_RX_BUF[USART1_MAX_RECV_LEN];      //接收缓冲,最大USART_REC_LEN(400)个字节.
uint16_t USART1_RX_STA=0;						 //接收状态标记	

uint32_t usart1_this_time_rx_len = 0;              //USART1收到的数据个数
uint8_t ChariqotRecognition_data[2][ChariotRecognition_data_dma_buf_len];   //DMA接收妙算数据的双缓存数组
int32_t ChariotRecognitionTemp[2] = {0,0};      //解析角度内容的数组
int16_t ChariotRecognitionDirection[2];  //接受摄像头传来的大装甲数据
int16_t Chariot_Rec_Dir_rotate[2];       //接受摄像头传来的大装甲和小装甲数据
int CameraDetectTarget_Flag = 0;     //摄像头发现目标标志

int TempShootingFlag=0;      //发弹标志,修改该标志，可选择是否发弹

float last_ChariotRecognition_pitch = 0.0f;        //上一个pitch角度值
float ChariotRecognition_pitch = 0.0f;             //pitch角度值
float last_ChariotRecognition_yaw = 0.0f;          //上一个yaw轴角度值
float ChariotRecognition_yaw = 0.0f;               //yaw角度值

float YawCurrentPositionSave   = 0.0f;     //保存当前Yaw轴位置
float PitchCurrentPositionSave = 0.0f;     //保存当前Pitch轴位置

uint16_t last_Target_Distance = 0;         //上次摄像头与目标的距离
uint16_t Target_Distance = 150;            //摄像头与目标的距离

uint16_t Distance_buf[10];     //距离缓冲区
uint8_t Dis_Buf_Index = 0;
uint8_t Pitch_Add_Angle = 0;
uint8_t enter_CNT = 0;
int Armor_R_Flag_Before=0;
int Armor_R_Flag_Behind=0;

int GM_Rotete_flag_Before=0;        //前固定摄像头识别目标
int GM_Rotete_flag_Behind=0;        //后固定摄像头识别目标
int Time_count=0;

CRringBuffer_t CR_ringBuffer;

float CR_yaw_Angle[20];
uint8_t CR_yaw_Angle_Index = 0;
uint8_t CR_yaw_Angle_CNT   = 0;
int8_t loop_j;

char Sendtosight[Sendtosight_len];            //发送给视觉

int friction_wheel_count = 0;
float kalman_yaw = 0;
float kalman_pitch = 0;
float kalman_yaw_feedforward = 0;
uint8_t update_flag = 1;
int Last_CameraDetectTarget_Flag=0;
float E_TEST=0;
float E_TEST1=0;
float E_TEST2=0;
float E_TEST3=0;
int camera_send_flag = 0;

int autoaim_mode_flag
	= 0;   //自瞄状态开启/关闭标志
//--------------------------------------------------------------------------------------------------------




//--------------------------------------------------------------------------------------------------------
//																						串口处理器函数
//void USART1_IRQHandler(void)
//{  
//	 if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart1);
//    }

//    else if(USART1->SR & UART_FLAG_IDLE)
//    {
//        static uint8_t this_time_rx_len = 0;
//			
//        __HAL_UART_CLEAR_PEFLAG(&huart1);

//        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            /* Current memory buffer used is Memory 0 */

//            //disable DMA
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = ChariotRecognition_data_dma_buf_len - hdma_usart1_rx.Instance->NDTR;//NDTR=15 buf_len =27,rxlen=12 然后就不接收了 应该把buf改成30


//            //reset set_data_lenght
//            //重新设定数据长度
//            hdma_usart1_rx.Instance->NDTR = (uint16_t)ChariotRecognition_data_dma_buf_len;

//            //set memory buffer 1
//            //设定缓冲区1
//            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
//            
//            //enable DMA
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);
//	   
//            if(this_time_rx_len == ChariotRecognition_data_len )
//            {
//							GetVisionData(&visionDataGet ,ChariqotRecognition_data[0]);
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //disable DMA
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = ChariotRecognition_data_dma_buf_len - hdma_usart1_rx.Instance->NDTR;//NDTR=21 

//            //reset set_data_lenght
//            //重新设定数据长度
//           hdma_usart1_rx.Instance->NDTR = (uint16_t)ChariotRecognition_data_dma_buf_len;

//            //set memory buffer 0
//            //设定缓冲区0
//            hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
//            
//            //enable DMA
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);

//            if(this_time_rx_len == ChariotRecognition_data_len)
//            {
//							GetVisionData(&visionDataGet ,ChariqotRecognition_data[0]);
//            }
//        }
//    }

//}

//--------------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------------
//																							机器人ID，机器人等级，射速......
State_distance state_distacne=closedistance;

extern uint8_t get_robot_id(void);
extern uint8_t get_robot_level(void);

int count_Sendtosight = 0;
//extern int windmill_mode;
float yaw_add_little = 0;
float pitch_add_little = 0;

int shoot_num = 0;
float real_shoot_speed_save[5] = {1.36,1.36,1.36,1.36,1.36};
float remaining_shoot_num = 0;
float last_remaining_shoot_num = 0;
float average_shoot_speed = 1.36;
float last_shoot_speed = 0.0f;
//--------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------
//             求5次发弹的平均值，实时更新,暂时没有写好，完善图传链路时修改
void shoot_speed_sending_culculate(void)
{
//	remaining_shoot_num = get_bullet_remaining_num();
  remaining_shoot_num = 1.36f;
	if(last_remaining_shoot_num - remaining_shoot_num)
	{
		shoot_num++;
	}
	
	if(shoot_num)
	{
//		real_shoot_speed_save[shoot_num%5] = get_bullet_speed();
		real_shoot_speed_save[shoot_num%5] = 1.36f;
	}
	
	average_shoot_speed = (real_shoot_speed_save[0] + real_shoot_speed_save[1] + real_shoot_speed_save[2] + real_shoot_speed_save[3] + real_shoot_speed_save[4]) / 5;	
}
//--------------------------------------------------------------------------------------------------------





VisionDataSend_Typedef visionDataSend;					    //发送给视觉组的数据
VisionDataGet_Typedef  visionDataGet;               //接收来自视觉组的数据
uint8_t RXdata[15];                                 //接收视觉组的数组
//             发送数据初始化
void SendVisionData_Init(void)
{
	visionDataSend.head 					         = 0xAA;		   //帧头
	visionDataSend.uphead 					         = 0xFF;		   //帧头
	visionDataSend.payload_len 					     = 22;              //数据长度
	visionDataSend.seq 					             = 0;
	visionDataSend.color 					         = 0x02;		   //初始颜色 0为己方红色 1为己方蓝色 2为己方红蓝双色（调试模式）
	visionDataSend.mode 					         = 0x01;		   //模式 0为自瞄 1为小能量机关 2为大能量机关 3为打击哨兵模式 4为小陀螺模式 5为录像模式 6为无人机模式 7为哨兵模式 8为雷达模式 其余情况默认为自瞄模式
	visionDataSend.robotID 					       = 0x03;		   //当前机器人ID 0为英雄 1为工程 2为步兵 3为无人机 4为哨兵
	visionDataSend.q[0].value                              =0    ;
	visionDataSend.q[1].value                              =0    ;
	visionDataSend.q[2].value                              =0    ;
	visionDataSend.q[3].value                              =0    ;
	visionDataSend.yaw_angle.yaw_gyro   	 =  5 ;			   //yaw轴陀螺仪角度   初始值
	visionDataSend.pitch_angle.pitch_gyro	 =  6;			   //pitch轴陀螺仪角度 初始值
//	visionDataSend.yaw_acc_data.yaw_acc 	 = 0;					 //yaw轴速度				  初始值
//	visionDataSend.pitch_acc_data.pitch_acc= 0;					 //pitch轴速度		    初始值
	visionDataSend.shoot_speed.value             = 0;		       //弹丸发射速度 			初始值
//	visionDataSend.CRCcode                 = 0;          //CRC校验
	visionDataSend.end 						         = 0x0D;		   //帧尾（某不愿意透露姓名的Q姓队长规定）
}
//             收发数据的串口初始化
void MyUART_Init(void)
{
	SendVisionData_Init();
}
//                向视觉发送数据 
void SendVisionData(VisionDataSend_Typedef* RAW_Data)		//调用该函数发送视觉信息给视觉组
{
	uint8_t data_buffer[28];
	static uint8_t delay_num;
	delay_num++;
	while(delay_num ==10)
	{
	static uint8_t frame_seq = 0;
	RAW_Data->seq = frame_seq++;
	data_buffer[0]  = RAW_Data->head;												//帧头
	data_buffer[1]  = RAW_Data->uphead;
	data_buffer[2]  = RAW_Data->payload_len;		
	data_buffer[3]  = RAW_Data->seq;	
	data_buffer[4]  = RAW_Data->color;								   	  //颜色 0为己方红色 1为己方蓝色 2为己方红蓝双色（调试模式）
	data_buffer[5]  = RAW_Data->mode;												//模式 0为自瞄 1为小能量机关 2为大能量机关 3为打击哨兵模式 4为小陀螺模式 5为录像模式 6为无人机模式 7为哨兵模式 8为雷达模式 其余情况默认为自瞄模式
	data_buffer[6]  = RAW_Data->robotID;                               //当前机器人ID 0为英雄 1为工程 2为步兵 3为无人机 4为哨兵
    data_buffer[7] = RAW_Data->shoot_speed.send[0];
	data_buffer[8] = RAW_Data->shoot_speed.send[1];	
    data_buffer[9] = RAW_Data->q[0].byte[0];                     //四元数W
	data_buffer[10] = RAW_Data->q[0].byte[1];
	data_buffer[11] = RAW_Data->q[1].byte[0];                     //四元数X
	data_buffer[12] = RAW_Data->q[1].byte[1];
	data_buffer[13] = RAW_Data->q[2].byte[0];                     //四元数Y
	data_buffer[14] = RAW_Data->q[2].byte[1];
	data_buffer[15] = RAW_Data->q[3].byte[0];                     //四元数Z
	data_buffer[16] = RAW_Data->q[3].byte[1];
	data_buffer[17]  = RAW_Data->yaw_angle.yaw_angle_send[0];
	data_buffer[18]  = RAW_Data->yaw_angle.yaw_angle_send[1];
	data_buffer[19]  = RAW_Data->pitch_angle.pitch_angle_send[0];
	data_buffer[20]  = RAW_Data->pitch_angle.pitch_angle_send[1];
	data_buffer[21]  = RAW_Data->yaw_vel.yaw_vel_send[0];
    data_buffer[22]  = RAW_Data->yaw_vel.yaw_vel_send[1];
    data_buffer[23]  = RAW_Data->pitch_vel.pitch_vel_send[0];
    data_buffer[24]  = RAW_Data->pitch_vel.pitch_vel_send[1];
//	data_buffer[8]  = RAW_Data->yaw_acc_data.yaw_acc_send[0];           新版视觉代码不需要加速度了
//	data_buffer[9]  = RAW_Data->yaw_acc_data.yaw_acc_send[1];           哈吉桢不让发
//	data_buffer[10] = RAW_Data->pitch_acc_data.pitch_acc_send[0];
//	data_buffer[11] = RAW_Data->pitch_acc_data.pitch_acc_send[1];
    uint16_t crc_val = Get_CRC16_CCITT(data_buffer, 25);

	data_buffer[25] = (uint8_t)(crc_val & 0xFF);         // 低位
    data_buffer[26] = (uint8_t)((crc_val >> 8) & 0xFF);  // 高位
	data_buffer[27] = RAW_Data->end;								//帧尾
	HAL_UART_Transmit(&huart1, data_buffer, 28, 0xff);//通过DMA一次性发送数据														//调用API发送
	delay_num =0;
	}

//#ifdef UART_DEBUG_MODE
//	printf("\n----------SendVisionData Start-------\n\n");
//	printf("\n---------SendVisionData Finish-------\n");
//#endif
}
//    处理从视觉接收的数据
//    myVisionDataGet    和   RAW_Data
void GetVisionData(VisionDataGet_Typedef* myVisionDataGet ,uint8_t RAW_Data[30])			//调用该函数获取视觉信息，视觉信息会传到 形参结构体 里
{
	
	// 1. 拼接视觉发来的两个 CRC 字节 (假设低位在前 RAW_Data[12]，高位在后 RAW_Data[13])
    uint16_t received_crc = (uint16_t)(RAW_Data[13] << 8) | RAW_Data[12];

    // 2. 计算接收到的数据部分的 CRC16 (校验范围：Index 0 到 11，共 12 个字节)
    // 注意：校验范围千万不能包含 CRC 字节本身
    uint16_t calculated_crc = Get_CRC16_CCITT(RAW_Data, 12);

    // 3. 校验对比
    if (calculated_crc != received_crc) 
    {
        // 如果校验不通过，直接跳出函数，不更新结构体数据，防止干扰导致云台乱晃
        return; 
    }
	
	myVisionDataGet->head =        RAW_Data[0];
	myVisionDataGet->uphead =      RAW_Data[1];
	myVisionDataGet->payload_len = RAW_Data[2];
	myVisionDataGet->seq =         RAW_Data[3];
	myVisionDataGet->detect_signal=RAW_Data[4];
	myVisionDataGet->shoot_msg =   RAW_Data[5];
	myVisionDataGet->yaw_angle.yaw_angle_get[0] =     RAW_Data[6];
	myVisionDataGet->yaw_angle.yaw_angle_get[1] =     RAW_Data[7];
    myVisionDataGet->pitch_angle.pitch_angle_get[0] = RAW_Data[8];
	myVisionDataGet->pitch_angle.pitch_angle_get[1] = RAW_Data[9];
// myVisionDataGet->x_data.x_get[0] = RAW_Data[7];
//	myVisionDataGet->x_data.x_get[1] = RAW_Data[8];
//	myVisionDataGet->y_data.y_get[0] = RAW_Data[9];
//	myVisionDataGet->y_data.y_get[1] = RAW_Data[10];
	myVisionDataGet->depth_data.depth_get[0] = RAW_Data[10];
	myVisionDataGet->depth_data.depth_get[1] = RAW_Data[11];
	myVisionDataGet->CRCcode = received_crc;
	myVisionDataGet->end = RAW_Data[14];
	
 if(RAW_Data[0] != 0XAA)  myVisionDataGet->head = 0;
 if(RAW_Data[1] == 0X0F)  myVisionDataGet->detect_signal = 0;
 if(RAW_Data[2] == 0X0A)  myVisionDataGet->shoot_msg =0;
// if(RAW_Data[3] == 0)  myVisionDataGet->yaw_angle.yaw_angle_get[0] = 0;
// if(RAW_Data[4] == 0)  myVisionDataGet->yaw_angle.yaw_angle_get[1] = 0;
// if(RAW_Data[5] == 0)  myVisionDataGet->pitch_angle.pitch_angle_get[0] = 0;
// if(RAW_Data[6] == 0)  myVisionDataGet->pitch_angle.pitch_angle_get[1] = 0;
// if(RAW_Data[7] == 0)  myVisionDataGet->x_data.x_get[0] = 0;
// if(RAW_Data[8] == 0)  myVisionDataGet->x_data.x_get[1] = 0;
// if(RAW_Data[9] == 0)  myVisionDataGet->y_data.y_get[0] = 0;
// if(RAW_Data[10] == 0)  myVisionDataGet->y_data.y_get[1] = 0;
// if(RAW_Data[11] == 0)  myVisionDataGet->depth_data.depth_get[0] = 0;
// if(RAW_Data[12] == 0)  myVisionDataGet->depth_data.depth_get[1] = 0;
 if(RAW_Data[14] != 0X0D)  myVisionDataGet->end = 0;


}

void Tidy_send_vision(VisionDataSend_Typedef *visionDataSend)
{
	 Referee_RefereeDataTypeDef *referee = Referee_GetRefereeDataPtr();
	//visionDataSend.color 					         = 0x02;		//初始颜色 0为己方红色 1为己方蓝色 2为己方红蓝双色（调试模式）
	//visionDataSend.mode 					         = 0x01;		//模式 0为自瞄 1为小能量机关 2为大能量机关 3为打击哨兵模式 4为小陀螺模式 5为录像模式 6为无人机模式 7为哨兵模式 8为雷达模式 其余情况默认为自瞄模式

	visionDataSend->q[0].value = (int16_t)(INS.q[0] * 10000.0f);
    visionDataSend->q[1].value = (int16_t)(INS.q[1] * 10000.0f);
    visionDataSend->q[2].value = (int16_t)(INS.q[2] * 10000.0f);
    visionDataSend->q[3].value = (int16_t)(INS.q[3] * 10000.0f);
	
	

	visionDataSend->yaw_angle.yaw_gyro      = (int16_t)(INS.Yaw* 100);
	visionDataSend->pitch_angle.pitch_gyro  = (int16_t)(INS.Roll * 100);
	visionDataSend->yaw_vel.yaw_gyro_vel      = (int16_t)(INS.Gyro[2]* 100);
	visionDataSend->pitch_vel.pitch_gyro_vel  = (int16_t)(INS.Gyro[1] * 100);  //INS.Gyro[0]为raw，1为pitch，2为yaw
//	visionDataSend->yaw_acc_data.yaw_acc 	= (int16_t)(INS.Accel[0] * 10000);
//	visionDataSend->pitch_acc_data.pitch_acc= (int16_t)(INS.Accel[2] * 10000);
	visionDataSend->shoot_speed.value              =  100*100;                           //(uint8_t)(last_shoot_speed*10.0);
//	if(bref_data.init_speed>=25){visionDataSend->shoot_speed=(uint8_t)25*10;	}
//	else{
//	visionDataSend->shoot_speed              =(uint8_t)bref_data.init_speed*10;	
//	}
	visionDataSend->robotID=(uint8_t)bref_data.robot_id;
	if(referee->bullet_speed != 0)
	last_shoot_speed = referee->bullet_speed;
	else if(referee->bullet_speed == 0)
	last_shoot_speed = 15.0f;

//  visionDataSend.CRCcode                 = 0;       //CRC校验;
}

uint8_t get_shoot_msg(VisionDataGet_Typedef *visionDataGet)
{
	uint8_t shoot_msg = 1;
	static uint8_t shoot_msg_buf[SHOOT_MSG_BUF_SIZE] = {0};
	shoot_msg_buf[0] = visionDataGet->shoot_msg;
	for(uint8_t shoot_msg_buf_num = 1;shoot_msg_buf_num < SHOOT_MSG_BUF_SIZE; shoot_msg_buf_num++)
	{
		shoot_msg_buf[shoot_msg_buf_num] = shoot_msg_buf[shoot_msg_buf_num - 1];
	}
	for(uint8_t shoot_msg_buf_num = 0;shoot_msg_buf_num < SHOOT_MSG_BUF_SIZE; shoot_msg_buf_num++)
	{
		shoot_msg = shoot_msg * shoot_msg_buf[shoot_msg_buf_num];
	}
	return shoot_msg;
}

/**
 * @brief CRC16-CCITT 计算函数
 * @param data 字节数组指针
 * @param length 要计算的字节长度
 * @return uint16_t 计算得到的校验和
 */
uint16_t Get_CRC16_CCITT(uint8_t *data, uint32_t length) {
    uint16_t crc = 0xFFFF; // 初始值
    for (uint32_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021; // 多项式 0x1021
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
