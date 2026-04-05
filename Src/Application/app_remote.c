/*
 *  Project      : Polaris
 * 
 *  file         : app_remote.c
 *  Description  : This file contains Remote control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-07 11:26:15
 */


#include "sys_const.h"
#include "protocol_common.h"
#include "app_remote.h"
#include "app_referee.h"
#include "module_shoot.h"
#include "module_gimbal.h"
#include "module_chassis.h"
#include "module_referee.h"
#include "periph_servo.h"
#include "cmsis_os.h"
#include "app_autoaim.h"
#include "periph_DMmotor.h"
#include "module_dm4310.h"
#include "util_can.h"
#include "alg_math.h"
#include "app_referee.h"
#include "periph_draw.h"


#define REMOTE_TASK_PERIOD  1
#define ENCODER_LIMIT 500
Remote_RemoteControlTypeDef Remote_remoteControlData;
Math_SlopeParamTypeDef Remote_ChassisFBSlope;
PID_GimbalYawVisionTypeDef Gimbal_YawVisionPID;
float last_encoder_angle=0;
float encoder_angle=0;
float get_abs(float a){
    if(a>=0){
		return a;
		}
		else return(0-a);

}
float limit_siqu(float last_angle,float angle){
       if(get_abs((int16_t)(angle-last_angle))<=ENCODER_LIMIT){ 
			 return last_angle;}
	     else{
			 return angle;
			 }

}

/**
  * @brief          Remote task
  * @param          NULL
  * @retval         NULL
  */
void Remote_Task(void const * argument) {

    forever {
        Remote_ControlCom();
      osDelay(REMOTE_TASK_PERIOD);
    }
}


/**
  * @brief      Remote Control Init
  * @param      NULL
  * @retval     NULL
  */
void Remote_RemotrControlInit() {
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    
    Math_InitSlopeParam(&Remote_ChassisFBSlope, MOUSE_CHASSIS_ACCELERATE, MOUSE_CHASSIS_ACCELERATE);
	
	
}


/**
  * @brief      Gets the pointer to the remote control data object
  * @param      NULL
  * @retval     Pointer to remote control data object
  */
Remote_RemoteControlTypeDef* Remote_GetControlDataPtr() {
    return &Remote_remoteControlData;
}


/**
* @brief      Remote control command
* @param      NULL
* @retval     NULL
*/
Remote_RemoteDataTypeDef *testdata;
void Remote_ControlCom() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
		testdata = data;
    control_data->pending = 1;

    switch (data->remote.s[0]) {
    /*      right switch control mode   */
        case Remote_SWITCH_UP: {
            /* right switch up is remote normal mode */
            Remote_RemoteProcess();
            break;
        }
        case Remote_SWITCH_MIDDLE: {
            /* right switch mid is keymouse mode    */
            Remote_RemoteShooterModeSet();
            break;
        }
        case Remote_SWITCH_DOWN: {
            /* right switch down is auto aim mode   */
            Remote_KeyMouseProcess();
            Remote_MouseShooterModeSet();
            break;
        }
        default:
            break;
    }

    control_data->pending = 0;
}


/**
* @brief      Mouse shoot mode set
* @param      NULL
* @retval     NULL
*/

/**
* @brief      Remote shoot mode set
* @param      NULL
* @retval     NULL
*/
float testvision;
float testvision2;
void Remote_RemoteShooterModeSet() {


		Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
		Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
		
	  
	PID_GimbalYawVisionPID_Init(&Gimbal_YawVisionPID, 
                                Const_GimbalYawVision[0],   // Kp   (? Debug ? YAKp ?????)
                                Const_GimbalYawVision[1],   // Ki
                                Const_GimbalYawVision[2],   // Kd
                                Const_GimbalYawVision[3],   // inc_max ????????(?)??=????
                                Const_GimbalYawVision[4]);  // bias_deadband ??(?)????????
    switch (data->remote.s[1]) {
    /*      left switch control mode   */
        case Remote_SWITCH_UP: {
            /* left switch up is fast shooting */
            Shooter_ChangeShooterMode(Shoot_NULL);
            Shooter_ChangeFeederMode(Feeder_FINISH);
            break;
        }
        case Remote_SWITCH_MIDDLE: {
            /* left switch mid is stop shooting    */
            Shooter_ChangeShooterMode(Shoot_FAST);
            Shooter_ChangeFeederMode(Feeder_FINISH);
					  
            break;
        }
        case Remote_SWITCH_DOWN: {
            /* left switch down is slow shooting   */
            Shooter_ChangeShooterMode(Shoot_FAST);
            //Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
					Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
        }
        default:
            break;
    }
		
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
	INS_INSTypeDef *ins = INS_GetINSPtr();
		gimbalpitch->output_state = 1;
		gimbalyaw->output_state = 1;
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
	float yaw_single_inc = PID_GimbalYawVisionPID_Calc(&Gimbal_YawVisionPID, visionDataGet.yaw_angle.yaw_predict);
	float yaw_total_add = (float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN+yaw_single_inc;
	buscomm->yaw_ref += yaw_total_add;
	testvision =yaw_total_add;
	testvision2 = yaw_single_inc;
	GimbalYaw_SetYawRef(buscomm->yaw_ref);
	
    float pitch_ref;	
    pitch_ref = (float)data->remote.ch[3] * REMOTE_DMPITCH_ANGLE + (float)visionDataGet.pitch_angle.pitch_predict*0.01f*VisionPitch;      //(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.02*-0.06f;      
	float cospitch = pitch_ref*PI/180;   //角度转为弧度

	GimbalPitch_SetPitchRef(cospitch);
	
	
	
		Chassis_SetChassisMode(Chassis_SEP);
		Chassis_SetChassisRef((float)data->remote.ch[1]  , (float)data->remote.ch[0] , 0);
		
		
}


/**
* @brief      Remote control process
* @param      NULL
* @retval     NULL

*/
float test_pitch_ref;
float test_pitch_limit;
float test_pitch_set;
void Remote_RemoteProcess() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();

    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
		gimbalpitch->output_state = 0;
		gimbalyaw->output_state = 0;
		
    switch (data->remote.s[1]) {
    /*      left switch control mode   */
        case Remote_SWITCH_UP: {
	        gimbalpitch->output_state = 1;
	        gimbalyaw->output_state = 1;
	        Chassis_SetChassisMode(Chassis_FOLLOW);
					last_encoder_angle=encoder_angle;
					encoder_angle=(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET);
	        Chassis_SetChassisRef((float)data->remote.ch[1]  , (float)data->remote.ch[0] ,encoder_angle); //(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET));
          buscomm->yaw_ref += Gimbal_LimitYaw((float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN);
		      GimbalYaw_SetYawRef(buscomm->yaw_ref);
          float pitch_ref;
          pitch_ref = (float)data->remote.ch[3] * REMOTE_DMPITCH_ANGLE+(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
          float cospitch = pitch_ref*PI/180;   //角度转为弧度
	      GimbalPitch_SetPitchRef(Gimbal_DMLimitPitch(cospitch));
            break;
        }
        case Remote_SWITCH_MIDDLE: {
	        gimbalpitch->output_state = 1;
	        gimbalyaw->output_state = 1;
	        Chassis_SetChassisMode(Chassis_SEP);
					last_encoder_angle=encoder_angle;
					encoder_angle=(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET);
	        Chassis_SetChassisRef((float)data->remote.ch[1]  , (float)data->remote.ch[0] ,(float)data->remote.ch[2]); //(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET));
          buscomm->yaw_ref += Gimbal_LimitYaw((float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN);
		      GimbalYaw_SetYawRef(buscomm->yaw_ref);
          float pitch_ref;
          pitch_ref = (float)data->remote.ch[3] * REMOTE_DMPITCH_ANGLE;
			float cospitch = pitch_ref*PI/180;   //角度转为弧度               																	
	      GimbalPitch_SetPitchRef(Gimbal_DMLimitPitch(cospitch));  
					break;
        }
        case Remote_SWITCH_DOWN: {
            /* left switch down is slow shooting   */
	          gimbalpitch->output_state = 1;
	          gimbalyaw->output_state = 1;
		        Chassis_SetChassisYawAngle(Motor_YawMotor.encoder.limited_angle,CHASSIS_YAW_ANGLE_OFFSET);
		        Chassis_SetChassisMode(Chassis_XTL);
		        Chassis_SetChassisRef((float)data->remote.ch[1]  , (float)data->remote.ch[0] , CHASSIS_XTL_WZ);
          buscomm->yaw_ref += (float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN;
		GimbalYaw_SetYawRef(buscomm->yaw_ref);
    float pitch_ref;
    pitch_ref = (float)data->remote.ch[3] * REMOTE_DMPITCH_ANGLE;
	float cospitch = pitch_ref*PI/180;   //角度转为弧度
	GimbalPitch_SetPitchRef(Gimbal_DMLimitPitch(cospitch));
							
            break;
        }
        default:
            break;
    }
	
    
}

/******************************************键鼠控制模式******************************************************/
/*************************************此部分单独写，仅赛场使用************************************************/

#define CHASSIS_ACCEL_STEP 0.9f  // 【关键参数】每控制周期速度变化量
                                 // 示例：若控制周期=10ms(100Hz)，加速至320需 320/8.0/100 = 0.4秒
                                 // 调大 → 加速更快；调小 → 加速更平滑


static uint32_t tick_b = 0, tick_q = 0,tick_v = 0,tick_e = 0;
static uint8_t last_b = 0, last_q = 0,last_v = 0,last_e = 0;
static uint8_t delay_step = 0;      // 步骤状态机
static uint32_t delay_start_tick = 0;
uint8_t q_mode = 0;
float autoaimtest;
LowPassFilter pitch_filter = {0.0f, 0.05f};
LowPassFilter  yaw_filter = {0.0f, 0.08f};

float autoaim_pitchz;
void Remote_KeyMouseProcess() { 
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    GimbalPitch_GimbalPitchTypeDef *gimbal = GimbalPitch_GetGimbalPitchPtr();
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
	
	PID_GimbalYawVisionPID_Init(&Gimbal_YawVisionPID, 
                                Const_GimbalYawVision[0],   // Kp   (? Debug ? YAKp ?????)
                                Const_GimbalYawVision[1],   // Ki
                                Const_GimbalYawVision[2],   // Kd
                                Const_GimbalYawVision[3],   // inc_max ????????(?)??=????
                                Const_GimbalYawVision[4]);  // bias_deadband ??(?)????????
    //chassis control
		// 静态变量：保存跨周期的当前速度（上电/复位后自动初始化为0）
      static float current_chassis_vx = 0.0f;
      static float current_chassis_vy = 0.0f;
			
	  float target_vx = 0.0f;
      float target_vy = 0.0f;
		
		if (data->key.w == 1)      target_vx =  220.0f;  // W: 前进
        else if (data->key.s == 1) target_vx = -220.0f; // S: 后退

        if (data->key.d == 1)      target_vy =  220.0f;  // D: 右移
        else if (data->key.a == 1) target_vy = -220.0f; // A: 左移
			
			// 2. 当前速度向目标速度线性渐变（含防超调保护）
           // VX 轴
       if (current_chassis_vx < target_vx) 
	   {
        current_chassis_vx += CHASSIS_ACCEL_STEP;
       if (current_chassis_vx > target_vx) current_chassis_vx = target_vx;
       } 
	   else if (current_chassis_vx > target_vx) {
    current_chassis_vx -= CHASSIS_ACCEL_STEP;
    if (current_chassis_vx < target_vx) current_chassis_vx = target_vx;
       }

           // VY 轴
      if (current_chassis_vy < target_vy) 
	  {
        current_chassis_vy += CHASSIS_ACCEL_STEP;
       if (current_chassis_vy > target_vy) current_chassis_vy = target_vy;
        }
        else if (current_chassis_vy > target_vy) {
    current_chassis_vy -= CHASSIS_ACCEL_STEP;
    if (current_chassis_vy < target_vy) current_chassis_vy = target_vy;
       }
//chassis control END		
		if(data->key.shift == 1)
		{
		  Chassis_SetChassisYawAngle(Motor_YawMotor.encoder.limited_angle,CHASSIS_YAW_ANGLE_OFFSET);
		  Chassis_SetChassisMode(Chassis_XTL);
		  Chassis_SetChassisRef(current_chassis_vx  , current_chassis_vy , CHASSIS_XTL_WZ);
		}
		else if(data->key.shift == 0)
		{
			Chassis_SetChassisMode(Chassis_FOLLOW);
			Chassis_SetChassisRef(current_chassis_vx  , current_chassis_vy , (float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET));	
		}

		if(Is_Key_Triggered(data->key.q, &last_q, &tick_q, 300))
		{
			 q_mode = (q_mode + 1) % 3;
		}
		if(Is_Key_Triggered(data->key.e, &last_e, &tick_e, 300))
		{
			 q_mode = (q_mode + 2) % 3;
		}		
		//autoaim control
		float autoaim_yaw;
		float autoaim_pitch;
		if(data->mouse.r == 1)
		{
			autoaim_yaw = PID_GimbalYawVisionPID_Calc(&Gimbal_YawVisionPID, visionDataGet.yaw_angle.yaw_predict);
			autoaim_pitch = (float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
			
		}
		else if(data->mouse.r == 0)
		{
			autoaim_yaw = 0.0f;
			autoaim_pitch = 0.0f;
		}
		
		
		//gimbal control
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
		gimbalpitch->output_state = 1;
		gimbalyaw->output_state = 1;
		float target_yaw = buscomm->yaw_ref + (float)data->mouse.x * -MOUSE_YAW_ANGLE_TO_FACT;
		float alpha = 0.1f; // 调整这个值，通常在0.01到0.3之间
		float raw_ref = alpha * target_yaw + (1.0f - alpha) * buscomm->yaw_ref;
		buscomm->yaw_ref = raw_ref+autoaim_yaw;
		autoaimtest = autoaim_yaw;
		GimbalYaw_SetYawRef(buscomm->yaw_ref);
    float pitch_ref;
     pitch_ref = (float)data->mouse.y * MOUSE_PITCH_ANGLE_TO_FACT;
		float alphb = 0.10f;
	float raw_pitch	 = alphb*pitch_ref+ (1.0f - alphb)*pitch_ref+autoaim_pitch ;
		autoaim_pitchz = autoaim_pitch;
	float cospitch = raw_pitch*PI/180;   //角度转为弧度
	GimbalPitch_SetPitchRef(cospitch);


		//shoot control(fric)
    if (data->key.f == 1)      
        Shooter_ChangeShooterMode(Shoot_FAST);
    if (data->key.g == 1)      
        Shooter_ChangeShooterMode(Shoot_NULL);
	
	
/**************************************************for draw************************************************************/	
		if (Is_Key_Triggered(data->key.b, &last_b, &tick_b, 300))
		{
			Referee_Setup();
		}

		if (Is_Key_Triggered(data->key.v, &last_v, &tick_v, 600))
		{
			Draw_ClearAll();
		}	
}
int count_cqie = 0;
int test_count ;
void Remote_MouseShooterModeSet() {
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();

    static int count_mouse_L = 0;
    if (data->mouse.l == 1){
		count_mouse_L++;
    if (count_mouse_L >= 3) {
        count_mouse_L = 3;
        count_cqie = 1;      
        switch (q_mode) {
            case 0:
                Shooter_ChangeFeederMode(Feeder_LOW_CONTINUE);
                break;
                
            case 1:
                Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
                break;
                
            case 2:
                Shooter_ChangeFeederMode(Feeder_VERY_FAST_CONTINUE);
                break;
                
            default:
                // 防御性编程：万一 q_mode 出现意外值，默认使用中速
                Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
                break;
        }
    }}else {
        if (0 < count_mouse_L && count_mouse_L < 3) {
            Shooter_SingleShootReset();
            Shooter_ChangeFeederMode(Feeder_SINGLE);
             
        }
        else 
        {
            // 没有按键或长按结束：进入完成/停止状态
            Shooter_ChangeFeederMode(Feeder_FINISH);
			count_cqie =0;
        }
        count_mouse_L = 0;
    }
		
		test_count = count_mouse_L;
}


/**
 * @brief 按键边沿检测与冷却逻辑集成函数
 * * @param current_state 当前按键的状态 (1为按下, 0为松开)
 * @param last_state    指向记录上次状态的变量指针
 * @param last_tick     指向记录上次触发时间的变量指针
 * @param cool_time     设定的冷却时间 (ms)
 * @return uint8_t      1: 触发成功, 0: 未触发
 */
uint8_t Is_Key_Triggered(uint8_t current_state, uint8_t *last_state, uint32_t *last_tick, uint32_t cool_time) 
{
    uint8_t triggered = 0;
    uint32_t now = HAL_GetTick();

    // 条件：当前是按下(1) && 上次是松开(0) && 时间差超过冷却时间
    if (current_state == 1 && *last_state == 0 && (now - *last_tick) > cool_time) 
    {
        *last_tick = now;  // 更新时间戳
        triggered = 1;     // 标记触发成功
    }

    *last_state = current_state; // 实时更新状态用于下次对比
    return triggered;
}

/**
 * @brief 一阶低通滤波函数
 * @param filter 滤波器结构体指针
 * @param input  当前时刻的原始输入值
 * @return float 滤波后的平滑输出
 */
float apply_low_pass_filter(LowPassFilter *filter, float input) {
    // 公式：out = alpha * input + (1 - alpha) * last_out
    filter->last_out = filter->alpha * input + (1.0f - filter->alpha) * filter->last_out;
    return filter->last_out;
}