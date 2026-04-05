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


const float Const_Shooter22mpers        = 320.0f;


const float Const_ShooterLockedCurrent              = 9000.0f;//堵转电流
const float Const_ShooterLockedSpeed                = 10.0f;
const float Const_ShooterLockedTime                 = 20.0f;
const float Const_ShooterRelockedTime               = 80.0f;
const float Const_ShooterLockedReverseSpeed         = 0.0f;

const float Const_FeederSlowSpeed                   = -70.0f;
const float Const_FeederFastSpeed                   = -70.0f;
const float Const_FeederVeryFastSpeed               = -70.0f;
const float Const_FeederWaitSpeed                   = 10.0f;

const float Const_HeatCtrlFastLimit                 = 75;
const float Const_HeatCtrlSlowLimit                 = 40;
const float Const_HeatCtrlWaitLimit                 = 10;
const float Const_HeatCtrlSingleCount               = 10;
const float Const_HeatCtrlStopLimit                 = 10;

const float Const_ShooterSlowSpeed                  = 0.0f;
const float Const_ShooterFastSpeed                  = 0.0f;

const float MOUSE_PITCH_ANGLE_TO_FACT             = 0.0025f;
float MOUSE_YAW_ANGLE_TO_FACT               	  = 0.02f;      //0.0025
const float MOUSE_CHASSIS_TO_FACT                 = 0.00022;       
const float MOUSE_CHASSIS_ACCELERATE              = 0.02f;
float MOUSE_CHASSIS_MAX_SPEED               	  = 12;
const float MOUSE_LEG_LEN_DLEN                    = 0.00018;

const float Const_WHEELLEG_REMOTE_YAW_GAIN              = 0.00035;  //0.0003
const float Const_WHEELLEG_REMOTE_X_GAIN                = 0.000022;
const float Const_WHEELLEG_REMOTE_LEN_GAIN              = 0.00000028;

const float REMOTE_PITCH_ANGLE_TO_REF                   = 0.0005f;
const float REMOTE_DMPITCH_ANGLE                         = 0.0004f; //0.000018f

const float REMOTE_CHASSIS_VX_GAIN                  = 0.7f;               //0.75
const float REMOTE_CHASSIS_VY_GAIN									= -0.7f;
const float REMOTE_CHASSIS_SEP_WZ_GAIN							= -0.25f;
const float REMOTE_CHASSIS_FOLLOW_WZ_GAIN						= 4.5f;    //跟随响应速度    原定4.3
const float REMOTE_CHASSIS_FOLLOW_WZ_MAX						= 120.0f;
const float REMOTE_CHASSIS_XTL_WZ_GAIN							= 1.2f;
const float CHASSIS_YAW_ANGLE_OFFSET							= 170.62f;    //269.62
const float CHASSIS_XTL_WZ										= 180.0f;     //180

float Const_PITCH_UMAXANGLE                       = 10.0f;   //10.4
float Const_PITCH_UMAXANGLE_GRYO                  = 10.0f;    //40.4
float Const_PITCH_DMAXANGLE                       = -10.0f;
float Const_YAW_MAXANGLE                          = 40.0f; 


/******************************************DMmotor***********************************************/
float KP_MIN                                                =0.0f  ;
float KP_MAX                                                =500.0f;
float KD_MIN                                                =0.0f  ;
float KD_MAX                                                =5.0f   ;
float Const_DMPITCH_UMAXANGLE                       = 0.8f;
float Const_DMPITCH_DMAXANGLE                       = -0.2f;
float Const_PITCH_MOTOR_INIT_OFFSETf              = 0.16f;     
float DM_KP                                        = 130  ;
float DM_KD	                                       = 1.5 ;
float DMpitchrefMax                                = 1.08 ;
float DMpitchrefMin	                               = -0.4 ;
float VisionPitch                                  =0.005f;
/************************************************************************************************/
// pitch gimbal param
const float Const_GimbalPitchSpdParam[4][5] = {
   {0.03f, 0.0f, -0.00f, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_GimbalPitchAngParam[4][5] = {
    {1.1f, 0.1f, -0.2f, 0, 15.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

//const float Const_GimbalPitchSpdParam1[4][5] = {
//    {0.5f, 0.001f, -0.0f, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

//const float Const_GimbalPitchAngParam1[4][5] = {
//   {1.1f, 0.1f, -0.2f, 5.0f, 15.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

// yaw gimbal param
const float Const_GimbalYawSpdParam[4][5] = {
    {2.5f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_GimbalYawAngParam[4][5] = {
    {0.45f, 0.01, 0.35f, 0, 10.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_GimbalYawVision[5] ={0.0033f, 0.0f, 0.0f, 0.5, 0.12f} ;

// shooter param
const float Const_ShootLeftParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ShootRightParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_FeedAngParam[4][5] = {
    {0.4f, 0.1f, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_FeedSpdParam[4][5] = {
    {0.4f, 0.1f, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

// Chassis param
const float Const_ChassisFontRightAngParam[4][5] = {
    {0.8f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisFontRightSpdParam[4][5] = {
    {0.06f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisFontRightAccelParam[4][5]={
    {0.002f,0.001,0.004,10,20},{0.1f,-1},{0,0},{0,0}
};
const float Const_ChassisFontLeftAngParam[4][5] = {
    {0.8f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisFontLeftSpdParam[4][5] = {
    {0.06f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackLeftAngParam[4][5] = {
    {0.8f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackLeftSpdParam[4][5] = {
    {0.06f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackRightAngParam[4][5] = {
    {0.8f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackRightSpdParam[4][5] = {
    {0.06f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

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
const float Power_Limit[4]={0,0,50,0.2};
