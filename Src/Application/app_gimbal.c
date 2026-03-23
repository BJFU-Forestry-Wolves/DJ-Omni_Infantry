/*
 *  Project      : Polaris
 * 
 *  file         : app_gimbal.c
 *  Description  : This file contains Gimbal Pitch control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 13:13:54
 */


#include "app_gimbal.h"
#include "module_gimbal.h"
#include "app_autoaim.h"
#include "periph_DMmotor.h"
#include "module_dm4310.h"
#include "util_can.h"

/**
  * @brief          Gimbal task
  * @param          NULL
  * @retval         NULL
  */
void Gimbal_Task(void const * argument) {

				  MyUART_Init();
	            
    for(;;) {
			  Tidy_send_vision(&visionDataSend);  //从陀螺仪上面取数据
			  SendVisionData(&visionDataSend);    //将数据发给视觉
		dm_motor_detect(&motor[Motor1]);

       // GimbalPitch_Control();
		//	  GimbalYaw_Control();
        GimbalPitch_Output();
		//dm_motor_ctrl_send(&hcan1, &motor[Motor1]);
		
      osDelay(2);
    }
}
