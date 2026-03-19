#include "alg_power_limting.h"
#include "arm_math.h"
#include "periph_referee.h"
#define filter_a 0.8
Chassic_Tyre_Power tyres[4];
Power_Limiting Chassic_pl;
float My_fabs(float a){
     if(a>=0){
		 return a;
		 }else return -a;

}
void Power_limiting_Init(void){
     for(uint16_t i=0;i<4;i++){
		 tyres[i].a=Power_Limit[0];
		 tyres[i].Ct=Power_Limit[1];
		 tyres[i].K1=Power_Limit[2];
		 tyres[i].K2=Power_Limit[3];
		 tyres[i].Power_current=0;
		 Chassic_pl.K_last=0;
		 }

}
void Power_Calculate(Chassic_Tyre_Power* Pl_tyre,Motor_MotorTypeDef* Cal_Motor){
     Pl_tyre->Power_current=Pl_tyre->K1*Cal_Motor->encoder.current*Cal_Motor->encoder.current+(Cal_Motor->encoder.speed*Cal_Motor->encoder.current)/9.55+Pl_tyre->a;
     //Pl_tyre->Power_current=My_fabs(Cal_Motor->encoder.current)+Pl_tyre->K1*
}
uint16_t test_max;
float Power_const=30000;
void Chassic_Power_Control(void){
	  double Speed_Cmd_Sum; double T_Cmd_sum; double Speed_2Sum;
	  Power_const=bref_data.chass_power*386.6;
    for(uint16_t i=0;i<4;i++){
		Speed_Cmd_Sum+=Motor_ChassisMotors.motor_handle[i]->encoder.speed*(Motor_ChassisMotors.motor_handle[i]->output);
		T_Cmd_sum+=(Motor_ChassisMotors.motor_handle[i]->output)*(Motor_ChassisMotors.motor_handle[i]->output);
		Speed_2Sum+=Motor_ChassisMotors.motor_handle[i]->encoder.speed*Motor_ChassisMotors.motor_handle[i]->encoder.speed;
		}
		test_max=T_Cmd_sum;
		Chassic_pl.K=10*(-Speed_Cmd_Sum+sqrt(Speed_Cmd_Sum*Speed_Cmd_Sum-4*Power_Limit[2]*T_Cmd_sum*(Power_Limit[3]*Speed_2Sum-Power_const)));
		if(T_Cmd_sum!=0&&Speed_Cmd_Sum*Speed_Cmd_Sum-4*Power_Limit[2]*T_Cmd_sum*(Power_Limit[3]*Speed_2Sum-Power_const)>=0){
		Chassic_pl.K/=100*2*Power_Limit[2]*T_Cmd_sum;
		if(Chassic_pl.K>=0){Chassic_pl.K=Chassic_pl.K;}else Chassic_pl.K=0-Chassic_pl.K;
		}
		else Chassic_pl.K=0;
		Chassic_pl.K=0.5f*Chassic_pl.K_last+(1-0.5f)*Chassic_pl.K;
		//if(Chassic_pl.K>=0){Chassic_pl.K=Chassic_pl.K;}else Chassic_pl.K=0-Chassic_pl.K;
		Chassic_pl.K_last=Chassic_pl.K;
	  
}	
void Tyres_Power_Cal(){
    Power_Calculate(&tyres[0],&Motor_ChassisFontRightMotor);
	  Power_Calculate(&tyres[1],&Motor_ChassisFontLeftMotor);
	  Power_Calculate(&tyres[2],&Motor_ChassisBackLeftMotor);
	  Power_Calculate(&tyres[3],&Motor_ChassisBackRightMotor);

}