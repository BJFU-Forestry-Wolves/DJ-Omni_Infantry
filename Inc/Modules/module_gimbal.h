/*
 * 文件：module_gimbal.h
 * 云台控制数据与接口。P轴和Yaw轴的参考值来源、单位和开关含义不同。
 * 本轮只整理注释，保持上一版恢复及局部命名修正后的可执行代码不变。
 */

#ifndef MODULE_GIMBAL_H
#define MODULE_GIMBAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "periph_motor.h"
#include "alg_math.h"
#include "alg_pid.h"

typedef struct {
    float pitch_ref;                                     // P轴累加后的目标角，按rad参与控制。
    uint8_t pitch_ref_limit_status;
    uint8_t pitch_count;

    uint8_t control_state;
    uint8_t output_state;
    uint8_t pending_state;

    PID_PIDTypeDef spdPID;  // 速度环PID对象；P轴当前主控制路径不使用其输出。
    PID_PIDParamTypeDef spdPIDParam;
    PID_PIDTypeDef angPID;  // 角度环PID对象；Yaw外环使用，P轴当前保留但未用于MIT目标生成。
    PID_PIDParamTypeDef angPIDParam;

    float pitch_ref_smooth;      // P轴参考的一阶平滑状态，单位rad。
    float filter_alpha;          // 一阶平滑系数；当前控制函数每轮写0.1。

} GimbalPitch_GimbalPitchTypeDef;

typedef struct {
    float yaw_ref;                                     // Yaw角度环参考，与INS累计Yaw角使用同一单位。
    float yaw_position_fdb;
    float yaw_speed_fdb;
    uint8_t yaw_ref_limit_status;
    uint8_t yaw_count;

    uint8_t control_state;
    uint8_t output_state;
    uint8_t pending_state;

    PID_PIDTypeDef spdPID;  // 速度环PID对象；P轴当前主控制路径不使用其输出。
    PID_PIDParamTypeDef spdPIDParam;
    PID_PIDTypeDef angPID;  // 角度环PID对象；Yaw外环使用，P轴当前保留但未用于MIT目标生成。
    PID_PIDParamTypeDef angPIDParam;

} GimbalYaw_GimbalYawTypeDef;

extern GimbalPitch_GimbalPitchTypeDef GimbalPitch_gimbalPitchControlData;
extern GimbalYaw_GimbalYawTypeDef GimbalYaw_gimbalYawControlData;

void GimbalPitch_InitGimbalPitch(void);
GimbalPitch_GimbalPitchTypeDef* GimbalPitch_GetGimbalPitchPtr(void);
void GimbalPitch_SetGimbalPitchControlState(uint8_t state);
void GimbalPitch_SetGimbalPitchOutputState(uint8_t state);
void GimbalPitch_SetPitchRef(float pitch_ref);
float Gimbal_LimitYaw(float ref);
void GimbalPitch_Control(void);

void GimbalYaw_InitGimbalYaw(void);
GimbalYaw_GimbalYawTypeDef* GimbalYaw_GetGimbalYawPtr(void);
void GimbalYaw_SetGimbalYawControlState(uint8_t state);
void GimbalYaw_SetGimbalYawOutputState(uint8_t state);
void GimbalYaw_SetYawRef(float yaw_ref);
void GimbalYaw_SetIMUYawPositionFdb(float imu_yaw_position_fdb);
void GimbalYaw_SetIMUYawSpeedFdb(float imu_yaw_speed_fdb);
void GimbalYaw_Control(void);
void GimbalYaw_Output(void);
float Gimbal_DMLimitPitch(float ref);

#endif

#ifdef __cplusplus
}
#endif
