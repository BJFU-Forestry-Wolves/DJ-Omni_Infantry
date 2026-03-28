/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : alg_pid.h
 *  Description  : This file contains PID algorithm function
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-02-08 10:51:07
 */


#ifndef PID_ALG_H
#define PID_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "alg_math.h"
#include "alg_filter.h"


typedef enum {
    PID_POSITION = 0u,
    PID_DELTA = 1u,
	  PID_Accel=2u
} PID_ModeEnum;

typedef struct {
    float ref;
    float fdb;
    float err[3];
    float err_lim;          // Integral anti-windup 
    float err_fdf[3];       // Feedforard
    float out_fdf;          // Feedforard out put
    float sum;
    float output;

    Filter_LowPassTypeDef d_fil;
    Filter_LowPassTypeDef delta_fil;

    Filter_LowPassTypeDef kf1_fil;
    Filter_LowPassTypeDef kf2_fil;
    float err_watch;
} PID_PIDTypeDef;

typedef struct {
    PID_ModeEnum pid_mode;

    // PID
    float kp;                   // pid param
    float ki;
    float kd;
    
    float sum_max;              // sum limit    
    float output_max;           // out limit
    
    Filter_LowPassParamTypeDef d_fil_param;         // Kd filter
    Filter_LowPassParamTypeDef delta_fil_param;     // delta filter ref filter

    // Feedforward
    float kf_1;                                     // feedforward param
    float kf_2;         
    
    Filter_LowPassParamTypeDef kf1_fil_param;      // feed forawrd filter
    Filter_LowPassParamTypeDef kf2_fil_param;
} PID_PIDParamTypeDef;

typedef struct {
    float kp;            // 比例系数（核心调响应速度）
    float ki;            // 积分系数（补微小偏差，默认0）
    float kd;            // 微分系数（核心抑超调）
    float raw_bias;      // 视觉原始偏差(yaw_predict)
    float real_bias;     // 还原后真实角度偏差(°)
    float err[3];        // 偏差缓存 [0]当前 [1]上一周期 [2]上上周期
    float single_inc;    // PID输出：单次2ms最优累加量(°)
    float inc_max;       // 单次累加量最大限幅(°)
    float bias_deadband; // 偏差死区阈值(°)
} PID_GimbalYawVisionTypeDef;

void PID_InitPIDParam(PID_PIDParamTypeDef* pparam, float kp, float ki, float kd, float sum_max, float output_max, 
                      float kd_fil_param, float delta_fil_param, float kf_1, float kf_2, float kf1_fil_param, 
                      float kf2_fil_param,PID_ModeEnum pid_mode);
float PID_GetPIDRef(PID_PIDTypeDef* pid);
void PID_SetPIDRef(PID_PIDTypeDef* pid, float ref);
void PID_AddPIDRef(PID_PIDTypeDef* pid, float inc);
float PID_GetPIDFdb(PID_PIDTypeDef* pid);
void PID_SetPIDFdb(PID_PIDTypeDef* pid, float fdb);
float PID_GetPIDOutput(PID_PIDTypeDef* pid);
void PID_ClearPID(PID_PIDTypeDef* pid);
void PID_CalcPID(PID_PIDTypeDef* pid, PID_PIDParamTypeDef* para);
float PID_GimbalYawVisionPID_Calc(PID_GimbalYawVisionTypeDef *yaw_pid, int16_t raw_yaw_predict);
void PID_GimbalYawVisionPID_Init(PID_GimbalYawVisionTypeDef *yaw_pid, float kp, float ki, float kd, float inc_max, float bias_deadband);

#ifdef __cplusplus
}
#endif

#endif
