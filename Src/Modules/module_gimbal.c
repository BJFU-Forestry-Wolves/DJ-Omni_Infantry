/*
 *  Project      : Polaris
 * 
 *  file         : cha_gimbal_ctrl.c
 *  Description  : This file contains Gimbal Pitch control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 16:27:28
 */

#include "alg_pid.h"
#include "cmsis_os.h"
#include "sys_const.h"
#include "module_gimbal.h"
#include "protocol_common.h"
#include "app_ins.h"
#include "periph_DMmotor.h"
#include "module_dm4310.h"
#include "util_can.h"
#include "app_remote.h"

GimbalPitch_GimbalPitchTypeDef GimbalPitch_gimbalPitchControlData;
GimbalYaw_GimbalYawTypeDef GimbalYaw_gimbalYawControlData;

float TestM;

/**
  * @brief      Gimbal pitch control initialization
  * @param      NULL
  * @retval     NULL
  */
void GimbalPitch_InitGimbalPitch() {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
	
	
    gimbalpitch->control_state = 1;
    gimbalpitch->output_state = 1;
    gimbalpitch->pitch_ref = 0;
    gimbalpitch->pitch_count = 0;
	gimbalpitch->pitch_ref_smooth =0;
    PID_InitPIDParam(&gimbalpitch->spdPIDParam, Const_GimbalPitchSpdParam[0][0], Const_GimbalPitchSpdParam[0][1], Const_GimbalPitchSpdParam[0][2], Const_GimbalPitchSpdParam[0][3], 
                    Const_GimbalPitchSpdParam[0][4], Const_GimbalPitchSpdParam[1][0], Const_GimbalPitchSpdParam[1][1], Const_GimbalPitchSpdParam[2][0], Const_GimbalPitchSpdParam[2][1], 
                    Const_GimbalPitchSpdParam[3][0], Const_GimbalPitchSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&gimbalpitch->angPIDParam, Const_GimbalPitchAngParam[0][0], Const_GimbalPitchAngParam[0][1], Const_GimbalPitchAngParam[0][2], Const_GimbalPitchAngParam[0][3], 
                    Const_GimbalPitchAngParam[0][4], Const_GimbalPitchAngParam[1][0], Const_GimbalPitchAngParam[1][1], Const_GimbalPitchAngParam[2][0], Const_GimbalPitchAngParam[2][1], 
                    Const_GimbalPitchAngParam[3][0], Const_GimbalPitchAngParam[3][1], PID_POSITION);     
	
	  
}

/**
  * @brief      Gimbal yaw control initialization
  * @param      NULL
  * @retval     NULL
  */
void GimbalYaw_InitGimbalYaw() {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();	
	
    gimbalyaw->control_state = 1;
    gimbalyaw->output_state = 1;
    gimbalyaw->yaw_ref = 0;
    gimbalyaw->yaw_count = 0;

    PID_InitPIDParam(&gimbalyaw->spdPIDParam, Const_GimbalYawSpdParam[0][0], Const_GimbalYawSpdParam[0][1], Const_GimbalYawSpdParam[0][2], Const_GimbalYawSpdParam[0][3], 
                    Const_GimbalYawSpdParam[0][4], Const_GimbalYawSpdParam[1][0], Const_GimbalYawSpdParam[1][1], Const_GimbalYawSpdParam[2][0], Const_GimbalYawSpdParam[2][1], 
                    Const_GimbalYawSpdParam[3][0], Const_GimbalYawSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&gimbalyaw->angPIDParam, Const_GimbalYawAngParam[0][0], Const_GimbalYawAngParam[0][1], Const_GimbalYawAngParam[0][2], Const_GimbalYawAngParam[0][3], 
                    Const_GimbalYawAngParam[0][4], Const_GimbalYawAngParam[1][0], Const_GimbalYawAngParam[1][1], Const_GimbalYawAngParam[2][0], Const_GimbalYawAngParam[2][1], 
                    Const_GimbalYawAngParam[3][0], Const_GimbalYawAngParam[3][1], PID_POSITION); 
	
   					
}

/**
  * @brief      Get the pointer of gimbal control object
  * @param      NULL
  * @retval     Pointer to gimbal control object
  */
GimbalPitch_GimbalPitchTypeDef* GimbalPitch_GetGimbalPitchPtr() {
    return &GimbalPitch_gimbalPitchControlData;
}
/**
  * @brief      Get the pointer of gimbal control object
  * @param      NULL
  * @retval     Pointer to gimbal control object
  */
GimbalYaw_GimbalYawTypeDef* GimbalYaw_GetGimbalYawPtr() {
    return &GimbalYaw_gimbalYawControlData;
}


/**
  * @brief      Set the gimbal control output calculation enabled state
  * @param      state: Enabled, 1 is enabled, 0 is disabled
  * @retval     NULL
  */
void GimbalPitch_SetGimbalPitchControlState(uint8_t state) {
    GimbalPitch_GimbalPitchTypeDef *gimbalPitch = GimbalPitch_GetGimbalPitchPtr();

    gimbalPitch->control_state = state;
}
/**
  * @brief      Set the gimbal control output calculation enabled state
  * @param      state: Enabled, 1 is enabled, 0 is disabled
  * @retval     NULL
  */
void GimbalYaw_SetGimbalYawControlState(uint8_t state) {
    GimbalYaw_GimbalYawTypeDef *gimbalYaw = GimbalYaw_GetGimbalYawPtr();

    gimbalYaw->control_state = state;
}


/**
  * @brief      Set gimbal control output enable status
  * @param      state: Enabled, 1 is enabled, 0 is disabled
  * @retval     NULL
  */
void GimbalPitch_SetGimbalPitchOutputState(uint8_t state) {
    GimbalPitch_GimbalPitchTypeDef *gimbalPitch = GimbalPitch_GetGimbalPitchPtr();

    gimbalPitch->output_state = state;
}
/**
  * @brief      Set gimbal control output enable status
  * @param      state: Enabled, 1 is enabled, 0 is disabled
  * @retval     NULL
  */
void GimbalYaw_SetGimbalYawOutputState(uint8_t state) {
    GimbalYaw_GimbalYawTypeDef *gimbalYaw = GimbalYaw_GetGimbalYawPtr();

    gimbalYaw->output_state = state;
}



/**
  * @brief      Set the target value of gimbal pitch
  * @param      pitch_ref: gimbal pitch target value
  * @retval     NULL
  */
void GimbalPitch_SetPitchRef(float pitch_ref) {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
//    float UMAX_PITCH_DEG = Const_DMPITCH_UMAXANGLE;
//	float DMAX_PITCH_DEG = Const_DMPITCH_DMAXANGLE;
	
    gimbalpitch->pitch_ref += pitch_ref;
	LimitMaxMin(gimbalpitch->pitch_ref, 0.75,0.2);
}
/**
  * @brief      Set the target value of gimbal yaw
  * @param      yaw_ref: gimbal yaw target value
  * @retval     NULL
  */
void GimbalYaw_SetYawRef(float yaw_ref) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
    
    gimbalyaw->yaw_ref = yaw_ref;
}                                                             //Pitch加和，而Yaw直接赋值



/**
* @brief      Yaw angle limit
* @param      ref: Yaw set ref
* @retval     Limited ywa ref
*/
float Gimbal_LimitYaw(float ref) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();

	if (buscomm->cha_mode  == Cha_Gyro)
        return ref;
	else if (((ins->YawTotalAngle - buscomm->yaw_ref < -Const_YAW_MAXANGLE) && (ref > 0)) || 
             ((ins->YawTotalAngle - buscomm->yaw_ref >  Const_YAW_MAXANGLE) && (ref < 0))) 
        return 0.0f;
    else return ref;
}



/**
  * @brief      Setting IMU yaw position feedback
  * @param      imu_yaw_position_fdb: IMU Yaw Position feedback
  * @retval     NULL
  */
void GimbalYaw_SetIMUYawPositionFdb(float imu_yaw_position_fdb) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->yaw_position_fdb = imu_yaw_position_fdb;
}


/**
  * @brief      Setting IMU yaw speed feedback
  * @param      imu_yaw_speed_fdb: IMU Yaw Speed feedback
  * @retval     NULL
  */
void GimbalYaw_SetIMUYawSpeedFdb(float imu_yaw_speed_fdb) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->yaw_speed_fdb = imu_yaw_speed_fdb;
}


/**
  * @brief      Control function of gimbal pitch
  * @param      NULL
  * @retval     NULL
  */
float out_put_data;


void GimbalPitch_Control() {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();

    if (gimbalpitch->control_state != 1) return;
	
	 // 滤波系数：0.05~0.2 之间，越小越平滑，响应越慢
    // 推荐默认 0.1f，平滑效果好，不卡顿
    gimbalpitch->filter_alpha = 0.08f;
	// dm_motor_init_test();
    //float imu_error = ins->Roll * PI / 180.0f + Const_PITCH_MOTOR_INIT_OFFSETf;


    gimbalpitch->pitch_ref_smooth += gimbalpitch->filter_alpha *(gimbalpitch->pitch_ref - gimbalpitch->pitch_ref_smooth);
      float target_pos = gimbalpitch->pitch_ref_smooth;
   

    
   // motor[Motor1].ctrl.vel_set = v_imu;
	motor[Motor1].ctrl.pos_set =Gimbal_DMLimitPitch(target_pos) ;
			
}

float gimbalyaw_fdb;
/**
  * @brief      Control function of gimbal yaw
  * @param      NULL
  * @retval     NULL
  */
void GimbalYaw_Control() {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();
	
    if (gimbalyaw->control_state != 1) return;


	
    PID_SetPIDRef(&gimbalyaw->angPID, gimbalyaw->yaw_ref);
    PID_SetPIDFdb(&gimbalyaw->angPID, ins->YawTotalAngle);//用的totalangle
    PID_CalcPID(&gimbalyaw->angPID, &gimbalyaw->angPIDParam);
	gimbalyaw_fdb = ins->YawTotalAngle;
    PID_SetPIDRef(&gimbalyaw->spdPID, PID_GetPIDOutput(&gimbalyaw->angPID));
    PID_SetPIDFdb(&gimbalyaw->spdPID, ins->Gyro[Z_INS]);
    PID_CalcPID(&gimbalyaw->spdPID, &gimbalyaw->spdPIDParam);   

    Motor_SetMotorOutput(&Motor_YawMotor, PID_GetPIDOutput(&gimbalyaw->spdPID));
}


/**
  * @brief      Gimbal pitch output function
  * @param      NULL
  * @retval     NULL
  */
void GimbalPitch_Output() {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
	
    if (gimbalpitch->output_state != 1) return;
	  if (gimbalyaw->output_state != 1) return;
    Motor_SendMotorGroupOutput(&Motor_PitchMotors);
	//  Motor_SendMotorGroupOutput(&Motor_PitchMotors1);
}



float Gimbal_DMLimitPitch(float ref) 
{
	Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
	INS_INSTypeDef *ins = INS_GetINSPtr();
	   
    
	if (ref > Const_DMPITCH_UMAXANGLE)
	{	
        return Const_DMPITCH_UMAXANGLE;
	}
	else if(ref < Const_DMPITCH_DMAXANGLE)
	{
			return Const_DMPITCH_DMAXANGLE;
	}else
          return ref;	
	
}
