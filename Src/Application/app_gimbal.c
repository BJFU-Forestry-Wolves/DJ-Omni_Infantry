/*
 * 达妙P轴空载慢速试转。
 * DM_SLOW_TEST_ENABLE=1：本文件内的独立试转；=0：恢复下方原云台任务。
 * 保留原控制函数、PID、组名称及初始化设置；不下载或自动启动电机。
 * 中文编码为GBK，Keil选择Chinese GB2312 (Simplified)。
 */
#include "app_gimbal.h"
#include "module_gimbal.h"
#include "app_autoaim.h"
#include "periph_DMmotor.h"
#include "module_dm4310.h"
#include "util_can.h"
#include "util_debug.h"
#include "FreeRTOS.h"
#include "task.h"



void Gimbal_Task(void const * argument) {
    MyUART_Init();

    for (;;) {
        Tidy_send_vision(&visionDataSend);
        SendVisionData(&visionDataSend);
        dm_motor_detect(&motor[Motor1]);
        GimbalPitch_Control();
        GimbalYaw_Control();
        GimbalYaw_Output();
        dm_motor_ctrl_send(&hcan1, &motor[Motor1]);
        osDelay(2);
    }
}
