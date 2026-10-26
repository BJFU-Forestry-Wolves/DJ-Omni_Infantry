/*
 * 文件：app_init.c
 * 系统初始化入口：先建立硬件与CAN，再初始化控制对象和使能达妙。
 * 本轮只整理注释，保持上一版恢复及局部命名修正后的可执行代码不变。
 */

#include "periph_DMmotor.h"
#include "module_dm4310.h"
#include "app_init.h"
#include "sys_softTimer.h"
#include "sys_dwt.h"
#include "alg_pid.h"
#include "util_can.h"
#include "periph_motor.h"
#include "periph_bmi088.h"
#include "periph_remote.h"
#include "periph_referee.h"
#include "periph_servo.h"
#include "module_gimbal.h"
#include "module_shoot.h"
#include "module_chassis.h"
#include "module_referee.h"
#include "app_ins.h"
#include "app_remote.h"
#include "protocol_common.h"
#include "periph_draw.h"
#include "module_power.h"

/**
 * 按依赖顺序完成硬件与控制模块初始化。
 * CAN滤波、启动及接收中断先就绪，再初始化达妙对象并发送一次使能帧。
 * 恢复版本不再写临时100ms超时参数；驱动端超时需另外配置并保存。
 */
void Init_InitAll() {
    DWT_Init(168);

    BMI088_Init(0);
    Remote_InitRemote();

    INS_Init();

	Motor_InitAllMotors();
    Can_InitFilterAndStart(&hcan1);
    Can_InitFilterAndStart(&hcan2);

    Referee_InitReferee();
	Referee_Setup();

	GimbalPitch_InitGimbalPitch();
	GimbalYaw_InitGimbalYaw();
    Shooter_InitShooter();
		Chassis_InitChassis();
    Remote_RemotrControlInit();
    Protocol_InitProtocol();

dm_motor_init();



// 使能电机
dm_motor_enable(&hcan1, &motor[Motor1]);

	SuperCap_InitHeader(&CapHeader) ;
	SuperCap_SetPower(&hcan1,&CapHeader,70);

}

/**
 * 启动软件定时器后挂起自身；只有外部恢复此任务后才会继续执行。
 */
void Init_Task(void const * argument) {
    SoftTimer_StartAll();
    forever {
        vTaskSuspend(Init_TaskHandleHandle);
      osDelay(25);
    }
}
