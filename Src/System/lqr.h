#ifndef __LQR_H
#define __LQR_H
#include "alg_math.h"
#include "main.h"
//#include "app_init.h"
//void atriax_nit(void);
typedef struct {
     float desire;
	   mat K;
	   float output;
}lqr;
void mat_init(void);
void Lqr_init(void);
void lqr_set_desire(float set_angle,lqr * follow_path);
void lqr_calculate(lqr * calculate);
#endif