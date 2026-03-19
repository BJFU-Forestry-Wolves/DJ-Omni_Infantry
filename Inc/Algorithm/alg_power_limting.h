#ifndef __ALG_POWER_LIMTING_H
#define __ALG_POWER_LIMTING_H
#include "main.h"
#include "periph_servo.h"
#include "module_chassis.h"
#include "periph_referee.h"
#include "sys_const.h"
typedef struct {
	 double Power_current;
	 double Ct;
	 double K1;
	 double K2;
	 double a;


}Chassic_Tyre_Power;
typedef struct {
	double Power_limiting;
	double Power_Current;
  Chassic_Tyre_Power* chass_tyre[4];
	double K;//功率缩放系数
	double K_last;

}Power_Limiting;
extern  Chassic_Tyre_Power tyres[4];
extern  Power_Limiting Chassic_pl;
void Power_limiting_Init(void);
void Power_Calculate(Chassic_Tyre_Power* Pl_tyre,Motor_MotorTypeDef* Cal_Motor);
void Chassic_Power_Control(void);
void Tyres_Power_Cal(void);
#endif