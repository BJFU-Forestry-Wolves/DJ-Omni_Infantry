/*
 *  Project      : 
 * 
 *  file         : app_Debug.c
 *  Description  : 本文件用来给VOFA当奴隶
 *  LastEditors  : Yuyuan
 *  Date         : 2025年12月22日19:31:49
 *  LastEditTime : 
 */

#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif



#include "alg_math.h"
#include "alg_pid.h"


extern float PAKp ;
extern float PAKd ;
extern float PAKi ;
	
extern float PSKp ;
extern float PSKd ;
extern float PSKi ;

extern float YSKp ;
extern float YSKd ;
extern float YSKi ;

extern float YAKp ;
extern float YAKd ;
extern float YAKi ;


extern int Target;

void Debug_Task(void const * argument);
void PID_ChangePID(PID_PIDParamTypeDef* pparam);
void PID_GimbalYawVision_SetAllParam(float new_kp, float new_ki, float new_kd);
#endif

#ifdef __cplusplus
}
#endif
