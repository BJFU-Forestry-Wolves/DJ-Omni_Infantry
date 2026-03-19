/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_const.c
 *  Description  : This file include all required constants
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:20:29
 *  LastEditTime : 2023-05-07 10:55:49
 */


#include "sys_const.h"

float Const_SERVO_INIT_OFFSET = 0.053f;

const float Const_Shooter15mpers        = 198.0f;
const float Const_Shooter18mpers        = 220.0f;
const float Const_Shooter22mpers        = 260.0f;
const float Const_Shooter30mpers        = 378.0f;

const float Const_ShooterLockedCurrent              = 3000.0f;
const float Const_ShooterLockedSpeed                = 20.0f;
const float Const_ShooterLockedTime                 = 200.0f;
const float Const_ShooterRelockedTime               = 500.0f;
const float Const_ShooterLockedReverseSpeed         = 0.0f;

const float Const_FeederSlowSpeed                   = 50.0f;
const float Const_FeederFastSpeed                   = 100.0f;
const float Const_FeederWaitSpeed                   = 10.0f;

const float Const_HeatCtrlFastLimit                 = 75;
const float Const_HeatCtrlSlowLimit                 = 40;
const float Const_HeatCtrlWaitLimit                 = 10;
const float Const_HeatCtrlSingleCount               = 10;
const float Const_HeatCtrlStopLimit                 = 10;

const float Const_ShooterSlowSpeed                  = 150.0f;
const float Const_ShooterFastSpeed                  = 225.0f;

const float MOUSE_PITCH_ANGLE_TO_FACT             = 0.008f;
float MOUSE_YAW_ANGLE_TO_FACT               	  = 0.0025f;
const float MOUSE_CHASSIS_TO_FACT                 = 0.00022;       
const float MOUSE_CHASSIS_ACCELERATE              = 0.02f;
float MOUSE_CHASSIS_MAX_SPEED               	  = 12;
const float MOUSE_LEG_LEN_DLEN                    = 0.00018;

const float Const_WHEELLEG_REMOTE_YAW_GAIN              = 0.0003;
const float Const_WHEELLEG_REMOTE_X_GAIN                = 0.000022;
const float Const_WHEELLEG_REMOTE_LEN_GAIN              = 0.00000028;

const float REMOTE_PITCH_ANGLE_TO_REF                   = 0.0005f;

const float REMOTE_CHASSIS_VX_GAIN                  = 0.5f;
const float REMOTE_CHASSIS_VY_GAIN									= -0.5f;
const float REMOTE_CHASSIS_SEP_WZ_GAIN							= -0.25f;
const float REMOTE_CHASSIS_FOLLOW_WZ_GAIN						= 5.0f;
const float REMOTE_CHASSIS_FOLLOW_WZ_MAX						= 175.0f;
const float REMOTE_CHASSIS_XTL_WZ_GAIN							= 1.0f;
const float CHASSIS_YAW_ANGLE_OFFSET								= 210.0f;
const float CHASSIS_XTL_WZ													= 100.0f;

float Const_PITCH_UMAXANGLE                       = 30.0f;
float Const_PITCH_UMAXANGLE_GRYO                  = 28.0f;  
float Const_PITCH_DMAXANGLE                       = -31.5f;
float Const_YAW_MAXANGLE                          = 40.0f; 
float Const_PITCH_MOTOR_INIT_OFFSETf              = 0.0f;

// pitch gimbal param
const float Const_GimbalPitchSpdParam[4][5] = {
    {5.0f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_GimbalPitchAngParam[4][5] = {
    {0.5f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
// yaw gimbal param
const float Const_GimbalYawSpdParam[4][5] = {
    {8.0f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_GimbalYawAngParam[4][5] = {
    {0.4f, 0, 0, 0, 5.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};


// shooter param
const float Const_ShootLeftParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ShootRightParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_FeedAngParam[4][5] = {
    {0.8f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_FeedSpdParam[4][5] = {
    {0.6f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

// Chassis param
const float Const_ChassisFontRightAngParam[4][5] = {
    {0.8f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisFontRightSpdParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisFontLeftAngParam[4][5] = {
    {0.8f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisFontLeftSpdParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackLeftAngParam[4][5] = {
    {0.8f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackLeftSpdParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackRightAngParam[4][5] = {
    {0.8f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackRightSpdParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                   0, 1, 0, 0, 0, 0,
                                   0, 0, 1, 0, 0, 0,
                                   0, 0, 0, 1, 0, 0,
                                   0, 0, 0, 0, 1, 0,
                                   0, 0, 0, 0, 0, 1};

float QuaternionEKF_P[36] = {100000,    0.1,    0.1,    0.1,    0.1,    0.1,
                                0.1, 100000,    0.1,    0.1,    0.1,    0.1,
                                0.1,    0.1, 100000,    0.1,    0.1,    0.1,
                                0.1,    0.1,    0.1, 100000,    0.1,    0.1,
                                0.1,    0.1,    0.1,    0.1,    100,    0.1,
                                0.1,    0.1,    0.1,    0.1,    0.1,    100};
