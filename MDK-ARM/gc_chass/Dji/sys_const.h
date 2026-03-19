/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_const.h
 *  Description  : This file include all required constants
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:21:16
 *  LastEditTime : 2023-05-07 10:55:18
 */


#ifndef SYS_CONST_H
#define SYS_CONST_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "main.h"

extern float Const_SERVO_INIT_OFFSET;


extern const float Const_Shooter15mpers;
extern const float Const_Shooter18mpers;
extern const float Const_Shooter22mpers;
extern const float Const_Shooter30mpers;

extern const float Const_ShooterLockedCurrent;     
extern const float Const_ShooterLockedSpeed;       
extern const float Const_ShooterLockedTime;        
extern const float Const_ShooterRelockedTime;      
extern const float Const_ShooterLockedReverseSpeed;

extern const float Const_FeederSlowSpeed;
extern const float Const_FeederFastSpeed;
extern const float Const_FeederWaitSpeed;

extern const float Const_HeatCtrlFastLimit;  
extern const float Const_HeatCtrlSlowLimit;  
extern const float Const_HeatCtrlWaitLimit;  
extern const float Const_HeatCtrlSingleCount;
extern const float Const_HeatCtrlStopLimit;  

extern const float Const_ShooterSlowSpeed;
extern const float Const_ShooterFastSpeed;

extern const float MOUSE_PITCH_ANGLE_TO_FACT;   
extern float MOUSE_YAW_ANGLE_TO_FACT;     
extern const float MOUSE_CHASSIS_TO_FACT;
extern const float MOUSE_CHASSIS_ACCELERATE;        
extern float MOUSE_CHASSIS_MAX_SPEED;     
extern const float MOUSE_LEG_LEN_DLEN;

extern const float Const_WHEELLEG_REMOTE_YAW_GAIN;
extern const float Const_WHEELLEG_REMOTE_X_GAIN;  
extern const float Const_WHEELLEG_REMOTE_LEN_GAIN;

extern const float REMOTE_PITCH_ANGLE_TO_REF;

extern float Const_PITCH_UMAXANGLE;         
extern float Const_PITCH_UMAXANGLE_GRYO;    
extern float Const_PITCH_DMAXANGLE;         
extern float Const_YAW_MAXANGLE;            
extern float Const_PITCH_MOTOR_INIT_OFFSETf;
extern float Const_YAW_AUTOADD;	

extern const float Const_GimbalPitchSpdParam[4][5];
extern const float Const_GimbalPitchAngParam[4][5];
extern const float Const_GimbalYawSpdParam[4][5];
extern const float Const_GimbalYawAngParam[4][5];

extern const float Const_ShootLeftParam[4][5];
extern const float Const_ShootRightParam[4][5];
extern const float Const_FeedAngParam[4][5];
extern const float Const_FeedSpdParam[4][5];


extern const float QuaternionEKF_F[36];                                   
extern float QuaternionEKF_P[36];

#endif

#ifdef __cplusplus
}
#endif
