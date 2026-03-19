/**
 * @file app_Debug.c
 * @brief 这个文件用来调试
 * @details 
 * @author Yuyuan
 * @version V1.0
 * @date 2026-01-02
 * @attention 要在CubeMX里面把优先级换成空闲
 */

#include "main.h"
#include "app_Debug.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "util_debug.h"
#include "sys_const.h"
#include "module_gimbal.h"
#include "app_ins.h"
#include "app_autoaim.h"
#include "app_remote.h"
#include "periph_DMmotor.h"
#include "module_dm4310.h"




float PAKp = 0.05;
float PAKd = 0.01;
float PAKi = 0;


float PSKp = 0.1;
float PSKd = 0;
float PSKi = 0;



float YAKp = 0.4;
float YAKd = 0.12;
float YAKi = 0.011;


float YSKp = 12.0;
float YSKd = 0;
float YSKi = 0;

int Target = 100  ;



/**
  * @brief 这是一个Debug任务
  * @param argument: [输入/出] 
  * @return NULL
  * @retval NULL
  */
void Debug_Task(void const *argument)
{

    for(;;)
    {
        GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
        GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

        INS_INSTypeDef *ins = INS_GetINSPtr();

		
        //PID_GimbalYawVision_SetAllParam(YAKp, YAKi, YAKd);
        PID_ChangePID(&gimbalpitch->angPIDParam);
		PID_ChangePID(&gimbalpitch->spdPIDParam);
		//在TIM3的定时中断里面，每10ms就会g_uart6_print_flag给这个置为1，也就是10ms打印一次数据
        if (g_uart6_print_flag == 1)
        {

            float send_data[VOFA_JUSTFLOAT_CHANNEL_NUM] =
            {
                motor[Motor1].ctrl.pos_set,                
                gimbalpitch->pitch_ref,                
                ins->Roll + Const_PITCH_MOTOR_INIT_OFFSETf,               

                PSKp,
                PSKd,
				PAKp,
				PAKd
              
            };

            //		usart_printf("%d,%d,%d,%d,%d\r\n",param1,param2,param3,param4,param5);
            vofa_justfloat_send(send_data, VOFA_JUSTFLOAT_CHANNEL_NUM);
            g_uart6_print_flag = 0;
        }
        osDelay(2);
    }



}






/**
  * @brief  这个函数用于改变电机的PID
  * @param pparam: [输入/出] 
  * @return NULL
  * @retval NULL
  */
void PID_ChangePID(PID_PIDParamTypeDef *pparam)
{
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

  
    if (pparam == NULL)
    {
        return;
    }

   
    if (pparam == &gimbalpitch->spdPIDParam)
    {
        pparam->kp = PSKp;
        pparam->kd = PSKd;
        pparam->ki = PSKi;
    }
  
    else if (pparam == &gimbalpitch->angPIDParam)
    {
        pparam->kp = PAKp;
        pparam->kd = PAKd;
        pparam->ki = PAKi;
    }
    else if (pparam == &gimbalyaw->spdPIDParam)
    {
        pparam->kp = YSKp;
        pparam->kd = YSKd;
        pparam->ki = YSKi;
    }
    else if (pparam == &gimbalyaw->angPIDParam)
    {
        pparam->kp = YAKp;
        pparam->kd = YAKd;
        pparam->ki = YAKi;
    }



}










/**
 * @brief  这个函数用于改变视觉发送的数据的增量式PID
 * @param new_kp: [输入/出] 
 *			 new_ki: [输入/出] 
 *			 new_kd: [输入/出] 
 * @return NULL
 * @retval NULL 
 */
//void PID_GimbalYawVision_SetAllParam(float new_kp, float new_ki, float new_kd)
//{
//    PID_GimbalYawVisionTypeDef *yaw_pid = GimbalYaw_GetGimbalYaw();
//    if(yaw_pid != NULL)
//    {
//        yaw_pid->kp = new_kp;
//        yaw_pid->ki = new_ki;
//        yaw_pid->kd = new_kd;
//    }
//}
