#ifndef MODULE_CHASSIS
#define MODULE_CHASSIS

#ifdef __cplusplus
extern "C" {
#endif

#include "periph_motor.h"
#include "alg_math.h"
#include "alg_pid.h"

typedef enum {
		Chassis_NULL = 0u,
		Chassis_SEP = 1u,
		Chassis_FOLLOW = 2u,
		Chassis_XTL = 3u,
	  Chassis_Fp=4u
}Chassis_ModeEnum;	
	
typedef struct {
    float chassis_ref;                                  
    float chassis_position_fdb;                         
    float chassis_speed_fdb;                            
    uint8_t chassis_ref_limit_status;                   
    uint8_t chassis_count;

    uint8_t control_state;                          
    uint8_t output_state;                          
    uint8_t pending_state;                          

    PID_PIDTypeDef spdPID;
    PID_PIDParamTypeDef spdPIDParam;
    PID_PIDTypeDef angPID;
    PID_PIDParamTypeDef angPIDParam;
    PID_PIDTypeDef AccelPID;
	  PID_PIDParamTypeDef AccelPIDParam;
} Chassis_ChassisTypeDef;

typedef struct {
	  Chassis_ModeEnum chassis_mode;
	
		float Chassis_Vx;
		float Chassis_Vy;
		float Chassis_Wz;
	
		float Chassis_Yaw_Angle;													
		float Chassis_Yaw_Rad;
		
		float Chassis_FontRight_Speed;
		float Chassis_FontLeft_Speed;
		float Chassis_BackLeft_Speed;
		float Chassis_BackRight_Speed;
	
} Chassis_StatusTypeDef;

extern Chassis_ChassisTypeDef Chassis_ControlData[4];
extern Chassis_StatusTypeDef Chassis_StatusData;

void Chassis_InitChassis(void);

Chassis_ChassisTypeDef* Chassis_ChassisPtr(void);
void Chassis_SetChassisControlState(uint8_t state);
void Chassis_SetChassisOutputState(uint8_t state);

Chassis_StatusTypeDef* Chassis_StatusPtr(void);
void Chassis_SetChassisMode(Chassis_ModeEnum mode);
void Chassis_SetChassisYawAngle(float yaw_angle,float yaw_angle_offset);
void Chassis_SetChassisRef(float RC_Vx,float RC_Vy,float RC_Wz);

void Chasssis_SetChasssisFontRightRef(float chassisfontright_ref);
void Chassis_SetChassisFontLeftRef(float chassisfontleft_ref);
void Chassis_SetChassisBackLeftRef(float chassisbackleft_ref);
void Chassis_SetChassisBackRightRef(float chassisbackright_ref);

void Chassis_Control(void);
void Chassis_Output(void);


#endif
	
#ifdef __cplusplus
}
#endif

