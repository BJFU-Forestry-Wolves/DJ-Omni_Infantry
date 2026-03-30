/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : alg_pid.c
 *  Description  : This file contains PID algorithm function
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:06
 *  LastEditTime : 2023-01-23 04:55:45
 */


#include "alg_pid.h"
#include "stddef.h"
#include "string.h"


/**
  * @brief      Initialize PID control parameters
  * @param      param: The pointer points to PID control parameters
  * @param      kp: P factor
  * @param      ki: I factor
  * @param      kd: D factor
  * @param      sum_max: Integral limiting
  * @param      output_max: Output limiting
  * @param      kd_fil_frq : PID Kd out filter
  * @param      delta_fil_frq: Delta PID ref filter
  * @param      kf_1: Feedforward of first order param
  * @param      kf_2: Feedforward of second order param
  * @param      kf1_fil_frq: Feedforward first order filter
  * @param      kf2_fil_frq: Feedforward second order filter
  * @param      period: PID calculate period
  * @param      pid_mode: PID calculate mode
  * @retval     NULL
  */
void PID_InitPIDParam(PID_PIDParamTypeDef* pparam, float kp, float ki, float kd, float sum_max, float output_max, 
                      float kd_fil_param, float delta_fil_param, float kf_1, float kf_2, float kf1_fil_param, 
                      float kf2_fil_param,PID_ModeEnum pid_mode) {

    pparam->pid_mode = pid_mode;
    pparam->kp = kp;
    pparam->ki = ki;
    pparam->kd = kd;

    pparam->sum_max    = sum_max;
    pparam->output_max = output_max;

    Filter_LowPassInit(kd_fil_param, &pparam->d_fil_param);
    Filter_LowPassInit(delta_fil_param, &pparam->delta_fil_param);

    pparam->kf_1 = kf_1;
    pparam->kf_2 = kf_2;

    Filter_LowPassInit(kf1_fil_param, &pparam->kf1_fil_param);
    Filter_LowPassInit(kf2_fil_param, &pparam->kf2_fil_param);
    
}


/**
  * @brief      Get PID control value
  * @param      pid: The pointer points to the PID controller
  * @retval     Set value of PID control quantity
  */
float PID_GetPIDRef(PID_PIDTypeDef* pid) {
    return pid->ref;
}


/**
  * @brief      Setting PID control value
  * @param      pid: The pointer points to the PID controller
  * @param      ref: set value
  * @retval     NULL
  */
void PID_SetPIDRef(PID_PIDTypeDef* pid, float ref) {
    pid->ref = ref;
}


/**
  * @brief      Increase the set value of PID control value (the increment can be negative)
  * @param      pid: The pointer points to the PID controller
  * @param      inc: Increment value
  * @retval     NULL
  */
void PID_AddPIDRef(PID_PIDTypeDef* pid, float inc) {
    pid->ref += inc;
}


/**
  * @brief      Obtaining feedback value of PID control quantity
  * @param      pid: The pointer points to the PID controller
  * @retval     Feedback value of PID control quantity
  */
float PID_GetPIDFdb(PID_PIDTypeDef* pid) {
    return pid->fdb;
}


/**
  * @brief      Setting feedback value of PID control quantity
  * @param      pid: The pointer points to the PID controller
  * @param      fdb: Feedback value
  * @retval     NULL
  */
void PID_SetPIDFdb(PID_PIDTypeDef* pid, float fdb) {
    pid->fdb = fdb;
}


/**
  * @brief      Obtain PID control output value
  * @param      pid: The pointer points to the PID controller
  * @retval     Output value of PID control quantity
  */
float PID_GetPIDOutput(PID_PIDTypeDef* pid) {
    return pid->output;
}


/**
  * @brief      Initialize PID controller
  * @param      pid: The pointer points to the PID controller
  * @retval     NULL
  */
void PID_ClearPID(PID_PIDTypeDef* pid) {
    pid->ref        = 0;
    pid->fdb        = 0;
    pid->err[0]     = 0;
    pid->err[1]     = 0;
    pid->err[2]     = 0;
    pid->err_lim    = 0;
    pid->err_fdf[0] = 0;
    pid->err_fdf[1] = 0;
    pid->err_fdf[2] = 0;
    pid->out_fdf    = 0;
    pid->sum        = 0;
    pid->output     = 0;


    pid->err_watch  = 0;
}


/**
  * @brief      Calculation of PID control quantity
  * @param      pid: The pointer points to the PID controller
  * @param      para: The pointer points to PID control parameters
  * @retval     NULL
  */
void PID_CalcPID(PID_PIDTypeDef* pid, PID_PIDParamTypeDef* pparam) {

    // Position Pid calculate
    if (pparam->pid_mode == PID_POSITION) {
        float dError, Error, ref_dError, ref_ddError;

        // Calculate the differential value
        Error = pid->ref - pid->fdb;
        pid->err[2] = pid->err[1];
        pid->err[1] = pid->err[0];
        pid->err[0] = Error;

        dError = Math_Differential(pid->err, 1, 1);


        pid->err_fdf[2] = pid->err_fdf[1];
        pid->err_fdf[1] = pid->err_fdf[0];
        pid->err_fdf[0] = pid->ref;

        ref_dError = Math_Differential(pid->err_fdf, 1, 1);
        ref_ddError = Math_Differential(pid->err_fdf, 2, 1);

        pid->err_watch = Error;
        // Calculate the integral and integra anti-windup 
        if (pparam->kp == 0)
            pid->sum = pid->sum + Error;
        else
            pid->sum = pid->sum + Error + pid->err_lim / pparam->kp;

        // Integral limiting
        LimitMax(pid->sum, pparam->sum_max);

        // Calculation results kf1_filter
        pid->out_fdf = Filter_LowPass((pparam->kf_1 * ref_dError), &pparam->kf1_fil_param, &pid->kf1_fil) + Filter_LowPass((pparam->kf_2 * ref_ddError), &pparam->kf2_fil_param, &pid->kf2_fil);
        pid->output  = pparam->kp * Error + pparam->ki * pid->sum + pparam->kd * Filter_LowPass(dError, &pparam->d_fil_param, &pid->d_fil) + pid->out_fdf;        
    }
    

    else if (pparam->pid_mode == PID_DELTA) {
        float dError, ddError, Error, ref_dError, ref_ddError;

        // Calculate the difference
        Error = pid->ref - pid->fdb;
        pid->err[2] = pid->err[1];
        pid->err[1] = pid->err[0];
        pid->err[0] = Error;
        dError  = Filter_LowPass(Math_Differential(pid->err, 1, 1), &pparam->delta_fil_param, &pid->delta_fil);
        ddError = Math_Differential(pid->err, 2, 1);


        pid->err_fdf[2] = pid->err_fdf[1];
        pid->err_fdf[1] = pid->err_fdf[0];
        pid->err_fdf[0] = pid->ref;
        
        ref_dError = Math_Differential(pid->err_fdf, 1, 1);
        ref_ddError = Math_Differential(pid->err_fdf, 2, 1);
    
        pid->err_watch = Error;

        // Calculate the integral and integral anti-windup 
        if (pparam->kp == 0)
            pid->sum = Error;
        else
            pid->sum = Error + pid->err_lim / pparam->kp;

        // Integral limiting
        LimitMax(pid->sum, pparam->sum_max);
    
        // Calculation results kf1_filter
        pid->out_fdf =  Filter_LowPass((pparam->kf_1 * ref_dError), &pparam->kf1_fil_param, &pid->kf1_fil) + Filter_LowPass((pparam->kf_2 * ref_ddError), &pparam->kf2_fil_param, &pid->kf2_fil);
        pid->output += (pparam->kp * dError + pparam->ki * pid->sum + pparam->kd * Filter_LowPass(ddError, &pparam->d_fil_param, &pid->d_fil));
        pid->output += pid->out_fdf;
    }    

        // Output limiting
    float temp_limit = pid->output;
    LimitMax(pid->output, pparam->output_max);
    pid->err_lim = pid->output - temp_limit;   
}


void PID_GimbalYawVisionPID_Init(PID_GimbalYawVisionTypeDef *yaw_pid, float kp, float ki, float kd, float inc_max, float bias_deadband)
{
    if (yaw_pid == NULL) return;
    
    // 初始化PID核心参数
    yaw_pid->kp = kp;
    yaw_pid->ki = ki;
    yaw_pid->kd = kd;
    
    // 初始化限幅/死区参数
    yaw_pid->inc_max = inc_max;         //单次最大累加度数
    yaw_pid->bias_deadband = bias_deadband;    //小于死区直接输出0°
    
    // 初始化偏差缓存
    yaw_pid->raw_bias = 0.0f;
    yaw_pid->real_bias = 0.0f;
    yaw_pid->err[0] = 0.0f;
    yaw_pid->err[1] = 0.0f;
    yaw_pid->err[2] = 0.0f;
    yaw_pid->single_inc = 0.0f;
}



/**
 * @brief 云台YAW轴视觉偏差PID计算核心函数
 * @核心逻辑：增量式PID → 输入总偏差 → 输出单次最优累加量
 * @适配点：1.自动×0.01还原真实角度 2.死区滤波 3.单次累加量限幅 4.历史偏差缓存
 */
float PID_GimbalYawVisionPID_Calc(PID_GimbalYawVisionTypeDef *yaw_pid, int16_t raw_yaw_predict)
{
    if (yaw_pid == NULL) return 0.0f;
    
    // -------------------------- 步骤1：数据预处理 还原真实角度偏差 --------------------------
    // raw_yaw_predict ×0.01 → 还原为云台可识别的真实角度偏差(°)
    yaw_pid->raw_bias = (float)raw_yaw_predict;
    yaw_pid->real_bias = yaw_pid->raw_bias * 0.01f;
    
    // -------------------------- 步骤2：偏差死区滤波（消除微小抖动） --------------------------
    if (fabs(yaw_pid->real_bias) < yaw_pid->bias_deadband)
    {
        yaw_pid->single_inc = 0.0f;
        yaw_pid->err[0] = 0.0f;
        yaw_pid->err[1] = 0.0f;
        yaw_pid->err[2] = 0.0f;
        return yaw_pid->single_inc;
    }
    
    // -------------------------- 步骤3：更新偏差缓存（k、k-1、k-2） --------------------------
    yaw_pid->err[2] = yaw_pid->err[1];  // 上上周期偏差 e(k-2)
    yaw_pid->err[1] = yaw_pid->err[0];  // 上一周期偏差 e(k-1)
    yaw_pid->err[0] = yaw_pid->real_bias;// 当前周期偏差 e(k)
    
    // -------------------------- 步骤4：增量式PID核心计算（专属你的累加场景） --------------------------
    // 增量式PID公式：ΔU(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
    float delta_err1 = yaw_pid->err[0] - yaw_pid->err[1];          // e(k)-e(k-1)
    float delta_err2 = yaw_pid->err[0] - 2*yaw_pid->err[1] + yaw_pid->err[2]; // e(k)-2e(k-1)+e(k-2)
    
    yaw_pid->single_inc = yaw_pid->kp * delta_err1          // P环节：动态调响应速度
                        + yaw_pid->ki * yaw_pid->err[0]     // I环节：补微小偏差
                        + yaw_pid->kd * delta_err2;         // D环节：抑制超调/抖动
    
    // -------------------------- 步骤5：单次累加量限幅（硬件保护，杜绝冲击） --------------------------
    LimitMax(yaw_pid->single_inc, yaw_pid->inc_max);
    
    // -------------------------- 返回：本次2ms最优累加量 --------------------------
    static float filted_inc = 0.0f;
    #define FILTER_ALPHA 0.6f  // 滤波系数（0.1~0.3，越小越平滑）
    filted_inc = filted_inc * (1 - FILTER_ALPHA) + yaw_pid->single_inc * FILTER_ALPHA;
    yaw_pid->single_inc = filted_inc;

    return yaw_pid->single_inc;
}
