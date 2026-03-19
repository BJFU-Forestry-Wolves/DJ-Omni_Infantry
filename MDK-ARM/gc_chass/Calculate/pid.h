#ifndef __PID_H
#define __PID_H
#include "main.h"
typedef struct{
  double error;
  double last_error;
  double kp;
  double ki;
  double kd;
  double index;
  double index_max;  
	double output;
	double max_output;
	
}my_pid;
void pid_init(my_pid * init,double kp1,double ki1,double kd1,double index_max1,double max_out);
void pid_cal(my_pid * cal,double ref,double set);
double pid_get_output(my_pid* ret);
#endif