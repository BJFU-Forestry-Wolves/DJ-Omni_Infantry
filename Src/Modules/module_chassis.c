#include "cmsis_os.h"
#include "sys_const.h"
#include "module_chassis.h"
#include "alg_power_limting.h"
Chassis_ChassisTypeDef Chassis_ControlData[4];
Chassis_StatusTypeDef Chassis_StatusData;

void Chassis_InitChassis(){
		Chassis_ChassisTypeDef *chassis = Chassis_ChassisPtr();

	  for(int i = 0; i < 4 ; i++)
		{
    chassis[i].control_state = 1;
    chassis[i].output_state = 1;
    chassis[i].chassis_ref = 0;
    chassis[i].chassis_count = 0;		
		}
		
		PID_InitPIDParam(&chassis[0].spdPIDParam, Const_ChassisFontRightSpdParam[0][0], Const_ChassisFontRightSpdParam[0][1], Const_ChassisFontRightSpdParam[0][2], Const_ChassisFontRightSpdParam[0][3], 
                    Const_ChassisFontRightSpdParam[0][4], Const_ChassisFontRightSpdParam[1][0], Const_ChassisFontRightSpdParam[1][1], Const_ChassisFontRightSpdParam[2][0], Const_ChassisFontRightSpdParam[2][1], 
                    Const_ChassisFontRightSpdParam[3][0], Const_ChassisFontRightSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&chassis[0].angPIDParam, Const_ChassisFontRightAngParam[0][0], Const_ChassisFontRightAngParam[0][1], Const_ChassisFontRightAngParam[0][2], Const_ChassisFontRightAngParam[0][3], 
                    Const_ChassisFontRightAngParam[0][4], Const_ChassisFontRightAngParam[1][0], Const_ChassisFontRightAngParam[1][1], Const_ChassisFontRightAngParam[2][0], Const_ChassisFontRightAngParam[2][1], 
                    Const_ChassisFontRightAngParam[3][0], Const_ChassisFontRightAngParam[3][1], PID_POSITION);
    PID_InitPIDParam(&chassis[0].AccelPIDParam,Const_ChassisFontRightAccelParam[0][0],Const_ChassisFontRightAccelParam[0][1],Const_ChassisFontRightAccelParam[0][2],Const_ChassisFontRightAccelParam[0][3],
		                Const_ChassisFontRightAccelParam[0][4],Const_ChassisFontRightAccelParam[1][0],Const_ChassisFontRightAccelParam[1][1],Const_ChassisFontRightAccelParam[2][0],Const_ChassisFontRightAccelParam[2][1],
		                Const_ChassisFontRightAccelParam[3][0],Const_ChassisFontRightAccelParam[3][1],PID_Accel);
		PID_InitPIDParam(&chassis[1].spdPIDParam, Const_ChassisFontLeftSpdParam[0][0], Const_ChassisFontLeftSpdParam[0][1], Const_ChassisFontLeftSpdParam[0][2], Const_ChassisFontLeftSpdParam[0][3], 
                    Const_ChassisFontLeftSpdParam[0][4], Const_ChassisFontLeftSpdParam[1][0], Const_ChassisFontLeftSpdParam[1][1], Const_ChassisFontLeftSpdParam[2][0], Const_ChassisFontLeftSpdParam[2][1], 
                    Const_ChassisFontLeftSpdParam[3][0], Const_ChassisFontLeftSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&chassis[1].angPIDParam, Const_ChassisFontLeftAngParam[0][0], Const_ChassisFontLeftAngParam[0][1], Const_ChassisFontLeftAngParam[0][2], Const_ChassisFontLeftAngParam[0][3], 
                    Const_ChassisFontLeftAngParam[0][4], Const_ChassisFontLeftAngParam[1][0], Const_ChassisFontLeftAngParam[1][1], Const_ChassisFontLeftAngParam[2][0], Const_ChassisFontLeftAngParam[2][1], 
                    Const_ChassisFontLeftAngParam[3][0], Const_ChassisFontLeftAngParam[3][1], PID_POSITION); 
    PID_InitPIDParam(&chassis[1].AccelPIDParam,Const_ChassisFontRightAccelParam[0][0],Const_ChassisFontRightAccelParam[0][1],Const_ChassisFontRightAccelParam[0][2],Const_ChassisFontRightAccelParam[0][3],
		                Const_ChassisFontRightAccelParam[0][4],Const_ChassisFontRightAccelParam[1][0],Const_ChassisFontRightAccelParam[1][1],Const_ChassisFontRightAccelParam[2][0],Const_ChassisFontRightAccelParam[2][1],
		                Const_ChassisFontRightAccelParam[3][0],Const_ChassisFontRightAccelParam[3][1],PID_Accel);
		PID_InitPIDParam(&chassis[2].spdPIDParam, Const_ChassisBackLeftSpdParam[0][0], Const_ChassisBackLeftSpdParam[0][1], Const_ChassisBackLeftSpdParam[0][2], Const_ChassisBackLeftSpdParam[0][3], 
                    Const_ChassisBackLeftSpdParam[0][4], Const_ChassisBackLeftSpdParam[1][0], Const_ChassisBackLeftSpdParam[1][1], Const_ChassisBackLeftSpdParam[2][0], Const_ChassisBackLeftSpdParam[2][1], 
                    Const_ChassisBackLeftSpdParam[3][0], Const_ChassisBackLeftSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&chassis[2].angPIDParam, Const_ChassisBackLeftAngParam[0][0], Const_ChassisBackLeftAngParam[0][1], Const_ChassisBackLeftAngParam[0][2], Const_ChassisBackLeftAngParam[0][3], 
                    Const_ChassisBackLeftAngParam[0][4], Const_ChassisBackLeftAngParam[1][0], Const_ChassisBackLeftAngParam[1][1], Const_ChassisBackLeftAngParam[2][0], Const_ChassisBackLeftAngParam[2][1], 
                    Const_ChassisBackLeftAngParam[3][0], Const_ChassisBackLeftAngParam[3][1], PID_POSITION); 
    PID_InitPIDParam(&chassis[2].AccelPIDParam,Const_ChassisFontRightAccelParam[0][0],Const_ChassisFontRightAccelParam[0][1],Const_ChassisFontRightAccelParam[0][2],Const_ChassisFontRightAccelParam[0][3],
		                Const_ChassisFontRightAccelParam[0][4],Const_ChassisFontRightAccelParam[1][0],Const_ChassisFontRightAccelParam[1][1],Const_ChassisFontRightAccelParam[2][0],Const_ChassisFontRightAccelParam[2][1],
		                Const_ChassisFontRightAccelParam[3][0],Const_ChassisFontRightAccelParam[3][1],PID_Accel);
		PID_InitPIDParam(&chassis[3].spdPIDParam, Const_ChassisBackRightSpdParam[0][0], Const_ChassisBackRightSpdParam[0][1], Const_ChassisBackRightSpdParam[0][2], Const_ChassisBackRightSpdParam[0][3], 
                    Const_ChassisBackRightSpdParam[0][4], Const_ChassisBackRightSpdParam[1][0], Const_ChassisBackRightSpdParam[1][1], Const_ChassisBackRightSpdParam[2][0], Const_ChassisBackRightSpdParam[2][1], 
                    Const_ChassisBackRightSpdParam[3][0], Const_ChassisBackRightSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&chassis[3].angPIDParam, Const_ChassisBackRightAngParam[0][0], Const_ChassisBackRightAngParam[0][1], Const_ChassisBackRightAngParam[0][2], Const_ChassisBackRightAngParam[0][3], 
                    Const_ChassisBackRightAngParam[0][4], Const_ChassisBackRightAngParam[1][0], Const_ChassisBackRightAngParam[1][1], Const_ChassisBackRightAngParam[2][0], Const_ChassisBackRightAngParam[2][1], 
                    Const_ChassisBackRightAngParam[3][0], Const_ChassisBackRightAngParam[3][1], PID_POSITION); 
		PID_InitPIDParam(&chassis[3].AccelPIDParam,Const_ChassisFontRightAccelParam[0][0],Const_ChassisFontRightAccelParam[0][1],Const_ChassisFontRightAccelParam[0][2],Const_ChassisFontRightAccelParam[0][3],
		                Const_ChassisFontRightAccelParam[0][4],Const_ChassisFontRightAccelParam[1][0],Const_ChassisFontRightAccelParam[1][1],Const_ChassisFontRightAccelParam[2][0],Const_ChassisFontRightAccelParam[2][1],
		                Const_ChassisFontRightAccelParam[3][0],Const_ChassisFontRightAccelParam[3][1],PID_Accel);
};


Chassis_ChassisTypeDef* Chassis_ChassisPtr(){
    return Chassis_ControlData;
}

Chassis_StatusTypeDef* Chassis_StatusPtr(){
		return &Chassis_StatusData;
}

void Chassis_SetChassisControlState(uint8_t state)
{
		Chassis_ChassisTypeDef *chassis = Chassis_ChassisPtr();
	 	for(int i = 0; i < 4 ; i++)
		{
    chassis[i].control_state = state;
		}
}

void Chassis_SetChassisOutputState(uint8_t state)
{
		Chassis_ChassisTypeDef *chassis = Chassis_ChassisPtr();
	 	for(int i = 0; i < 4 ; i++)
		{
    chassis[i].output_state = state;
		}
}

void Chassis_SetChassisMode(Chassis_ModeEnum mode)
{
		Chassis_StatusTypeDef *chassis_status = Chassis_StatusPtr();
		chassis_status->chassis_mode = mode;
}

void Chassis_SetChassisYawAngle(float yaw_angle,float yaw_angle_offset){
		Chassis_StatusTypeDef *chassis_status = Chassis_StatusPtr();
		if( (yaw_angle >= 0.0f) && (yaw_angle <= yaw_angle_offset) )
		{
				chassis_status->Chassis_Yaw_Angle = yaw_angle - yaw_angle_offset + 360.0f;
		}
		else if( (yaw_angle > yaw_angle_offset) && (yaw_angle <= 360.0f) )
		{
				chassis_status->Chassis_Yaw_Angle = yaw_angle - yaw_angle_offset;
		}
		
		if( (chassis_status->Chassis_Yaw_Angle >= 180.0f) && (chassis_status->Chassis_Yaw_Angle <= 360.0f) )
		{
				chassis_status->Chassis_Yaw_Angle = chassis_status->Chassis_Yaw_Angle - 360.0f;
		}
		chassis_status->Chassis_Yaw_Rad = Math_Angle2Rad(chassis_status->Chassis_Yaw_Angle);
}
void Chassis_setAccel(float Forward_Accel){
    


}
void Chassis_SetChassisRef(float RC_Vx,float RC_Vy,float RC_Wz){
		Chassis_StatusTypeDef *chassis_status = Chassis_StatusPtr();
	
		switch(chassis_status->chassis_mode)
		{
      case Chassis_NULL:
					chassis_status->Chassis_Vx = 0.0f;
					chassis_status->Chassis_Vy = 0.0f;
					chassis_status->Chassis_Wz = 0.0f;
				  chassis_status->Chassis_FontRight_Speed = 0.0f;
				  chassis_status->Chassis_FontLeft_Speed  = 0.0f;
				  chassis_status->Chassis_BackLeft_Speed  = 0.0f;
				  chassis_status->Chassis_BackRight_Speed = 0.0f;	
          break;
			
      case Chassis_SEP:
	      	chassis_status->Chassis_Vx = RC_Vx * REMOTE_CHASSIS_VX_GAIN;
	      	chassis_status->Chassis_Vy = RC_Vy * REMOTE_CHASSIS_VY_GAIN;
					chassis_status->Chassis_Wz = RC_Wz * REMOTE_CHASSIS_SEP_WZ_GAIN;
				  chassis_status->Chassis_FontRight_Speed = -chassis_status->Chassis_Vx -chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;
				  chassis_status->Chassis_FontLeft_Speed  =  chassis_status->Chassis_Vx -chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;
				  chassis_status->Chassis_BackLeft_Speed  =  chassis_status->Chassis_Vx +chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;
				  chassis_status->Chassis_BackRight_Speed = -chassis_status->Chassis_Vx +chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;			
          break;
			
      case Chassis_FOLLOW:
	      	chassis_status->Chassis_Vx = RC_Vx * REMOTE_CHASSIS_VX_GAIN;
	      	chassis_status->Chassis_Vy = RC_Vy * REMOTE_CHASSIS_VY_GAIN;
					chassis_status->Chassis_Wz = RC_Wz * REMOTE_CHASSIS_FOLLOW_WZ_GAIN;
			    if(chassis_status->Chassis_Wz > REMOTE_CHASSIS_FOLLOW_WZ_MAX)
					{
						chassis_status->Chassis_Wz = REMOTE_CHASSIS_FOLLOW_WZ_MAX;
					}
					else if(chassis_status->Chassis_Wz < -REMOTE_CHASSIS_FOLLOW_WZ_MAX)
					{
						chassis_status->Chassis_Wz = -REMOTE_CHASSIS_FOLLOW_WZ_MAX;
					}
				  chassis_status->Chassis_FontRight_Speed = -chassis_status->Chassis_Vx -chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;
				  chassis_status->Chassis_FontLeft_Speed  =  chassis_status->Chassis_Vx -chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;
				  chassis_status->Chassis_BackLeft_Speed  =  chassis_status->Chassis_Vx +chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;
				  chassis_status->Chassis_BackRight_Speed = -chassis_status->Chassis_Vx +chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;			
          break;
			
      case Chassis_XTL:
	      	chassis_status->Chassis_Vx = RC_Vx * REMOTE_CHASSIS_VX_GAIN;
	      	chassis_status->Chassis_Vy = RC_Vy * REMOTE_CHASSIS_VY_GAIN;
					chassis_status->Chassis_Wz = RC_Wz * REMOTE_CHASSIS_XTL_WZ_GAIN;
          chassis_status->Chassis_FontRight_Speed = -chassis_status->Chassis_Vx * arm_cos_f32(-chassis_status->Chassis_Yaw_Rad + PI/2) 
																										-chassis_status->Chassis_Vy * arm_sin_f32(-chassis_status->Chassis_Yaw_Rad)
																									  -chassis_status->Chassis_Vy * arm_cos_f32(-chassis_status->Chassis_Yaw_Rad)
																										-chassis_status->Chassis_Vx * arm_sin_f32(-chassis_status->Chassis_Yaw_Rad + PI/2)			
																										-chassis_status->Chassis_Wz;
			   chassis_status->Chassis_FontLeft_Speed   = -chassis_status->Chassis_Vx * arm_cos_f32(-chassis_status->Chassis_Yaw_Rad + PI/2)
			                                              +chassis_status->Chassis_Vy * arm_sin_f32(-chassis_status->Chassis_Yaw_Rad)
			                                              -chassis_status->Chassis_Vy * arm_cos_f32(-chassis_status->Chassis_Yaw_Rad)
			                                              +chassis_status->Chassis_Vx * arm_sin_f32(-chassis_status->Chassis_Yaw_Rad + PI/2)
																										-chassis_status->Chassis_Wz;
			   chassis_status->Chassis_BackLeft_Speed   = +chassis_status->Chassis_Vx * arm_cos_f32(-chassis_status->Chassis_Yaw_Rad + PI/2)
			                                              +chassis_status->Chassis_Vy * arm_sin_f32(-chassis_status->Chassis_Yaw_Rad)
			                                              +chassis_status->Chassis_Vy * arm_cos_f32(-chassis_status->Chassis_Yaw_Rad)
			                                              +chassis_status->Chassis_Vx * arm_sin_f32(-chassis_status->Chassis_Yaw_Rad + PI/2)
																										-chassis_status->Chassis_Wz;
			   chassis_status->Chassis_BackRight_Speed  = +chassis_status->Chassis_Vx * arm_cos_f32(-chassis_status->Chassis_Yaw_Rad + PI/2)
			                                              -chassis_status->Chassis_Vy * arm_sin_f32(-chassis_status->Chassis_Yaw_Rad)
			                                              +chassis_status->Chassis_Vy * arm_cos_f32(-chassis_status->Chassis_Yaw_Rad)
			                                              -chassis_status->Chassis_Vx * arm_sin_f32(-chassis_status->Chassis_Yaw_Rad + PI/2)
																										-chassis_status->Chassis_Wz;		
          break;
      
			case Chassis_Fp:
				  chassis_status->Chassis_Vx = RC_Vx * REMOTE_CHASSIS_VX_GAIN;
	      	chassis_status->Chassis_Vy = RC_Vy * REMOTE_CHASSIS_VY_GAIN;
					chassis_status->Chassis_Wz = RC_Wz * REMOTE_CHASSIS_SEP_WZ_GAIN;
				  chassis_status->Chassis_FontRight_Speed = -chassis_status->Chassis_Vx -chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;
				  chassis_status->Chassis_FontLeft_Speed  =  chassis_status->Chassis_Vx -chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;
				  chassis_status->Chassis_BackLeft_Speed  =  chassis_status->Chassis_Vx +chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;
				  chassis_status->Chassis_BackRight_Speed = -chassis_status->Chassis_Vx +chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;
			default:
          break;
		}

	

}


void Chassis_Control()
{
		Chassis_ChassisTypeDef *chassis = Chassis_ChassisPtr();
		Chassis_StatusTypeDef *chassis_status = Chassis_StatusPtr();
	
		for(int i = 0 ; i < 4 ; i++ )
    {
			if(chassis[i].control_state != 1) return;
    }
		//FontRight
//    PID_SetPIDRef(&chassis[0].angPID, chassis[0].chassis_ref);
//    PID_SetPIDFdb(&chassis[0].angPID, Motor_ChassisFontRightMotor.encoder.consequent_angle);
//    PID_CalcPID(&chassis[0].angPID, &chassis[0].angPIDParam);
//    PID_SetPIDRef(&chassis[0].AccelPID,chassis_status->Chassis_FontRight_Speed);
//		PID_SetPIDRef(&chassis[0].AccelPID,Motor_ChassisFontRightMotor.encoder.speed-Motor_ChassisFontRightMotor.encoder.last_speed);
 //   PID_CalcPID(&chassis[0].AccelPID,&chassis[0].AccelPIDParam);
		PID_SetPIDRef(&chassis[0].spdPID, chassis_status->Chassis_FontRight_Speed);
    PID_SetPIDFdb(&chassis[0].spdPID, Motor_ChassisFontRightMotor.encoder.speed);
    PID_CalcPID(&chassis[0].spdPID, &chassis[0].spdPIDParam);   
		
    Motor_SetMotorOutputChassic(&Motor_ChassisFontRightMotor, PID_GetPIDOutput(&chassis[0].spdPID));
		//FontLeft
//	PID_SetPIDRef(&chassis[1].angPID, chassis[1].chassis_ref);
 //   PID_SetPIDFdb(&chassis[1].angPID, Motor_ChassisFontLeftMotor.encoder.consequent_angle);
//    PID_CalcPID(&chassis[1].angPID, &chassis[1].angPIDParam);

    PID_SetPIDRef(&chassis[1].spdPID, chassis_status->Chassis_FontLeft_Speed);
    PID_SetPIDFdb(&chassis[1].spdPID, Motor_ChassisFontLeftMotor.encoder.speed);
    PID_CalcPID(&chassis[1].spdPID, &chassis[1].spdPIDParam);   

    Motor_SetMotorOutputChassic(&Motor_ChassisFontLeftMotor, PID_GetPIDOutput(&chassis[1].spdPID));
		//BackLeft
//	PID_SetPIDRef(&chassis[2].angPID, chassis[2].chassis_ref);
//    PID_SetPIDFdb(&chassis[2].angPID, Motor_ChassisBackLeftMotor.encoder.consequent_angle);
//    PID_CalcPID(&chassis[2].angPID, &chassis[2].angPIDParam);

    PID_SetPIDRef(&chassis[2].spdPID, chassis_status->Chassis_BackLeft_Speed);
    PID_SetPIDFdb(&chassis[2].spdPID, Motor_ChassisBackLeftMotor.encoder.speed);
    PID_CalcPID(&chassis[2].spdPID, &chassis[2].spdPIDParam);   

    Motor_SetMotorOutputChassic(&Motor_ChassisBackLeftMotor, PID_GetPIDOutput(&chassis[2].spdPID));
		//BackRight		
//	PID_SetPIDRef(&chassis[3].angPID, chassis[3].chassis_ref);
 //   PID_SetPIDFdb(&chassis[3].angPID, Motor_ChassisBackRightMotor.encoder.consequent_angle);
//    PID_CalcPID(&chassis[3].angPID, &chassis[3].angPIDParam);

    PID_SetPIDRef(&chassis[3].spdPID, chassis_status->Chassis_BackRight_Speed);
    PID_SetPIDFdb(&chassis[3].spdPID, Motor_ChassisBackRightMotor.encoder.speed);
    PID_CalcPID(&chassis[3].spdPID, &chassis[3].spdPIDParam);   

    Motor_SetMotorOutputChassic(&Motor_ChassisBackRightMotor, PID_GetPIDOutput(&chassis[3].spdPID));
	
}


void Chassis_Output()
{
		Chassis_ChassisTypeDef *chassis = Chassis_ChassisPtr();

		for(int i = 0 ; i < 4 ; i++ )
    { 
			//Motor_ChassisMotors.motor_handle[i]->output=Chassic_pl.K*Motor_ChassisMotors.motor_handle[i]->output;
			if(chassis[i].output_state != 1) return;
    }
    Motor_SendMotorGroupOutput(&Motor_ChassisMotors);
}
