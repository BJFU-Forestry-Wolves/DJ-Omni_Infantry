#include "pid.h"
#include "math.h"
void pid_init(my_pid * init,double kp1,double ki1,double kd1,double index_max1,double max_out){
	init->kp=kp1;
	init->ki=ki1;
	init->kd=kd1;
	init->index_max=index_max1;
	init->max_output=max_out;
	init->error=0;
	init->index=0;
	init->last_error=0;
	init->output=0;
}
void pid_cal(my_pid * cal,double ref,double set){
  cal->last_error=cal->error;
	cal->error=set-ref;
	cal->index+=cal->error;
	if(cal->index>=cal->index_max){
	cal->index=cal->index_max;
	}
	if(cal->index<=0-cal->index_max){
	cal->index=0-cal->index_max;
	}
	cal->output=cal->kp*(cal->error)+cal->ki*(cal->index)+cal->kp*(cal->error-cal->last_error);
	if(cal->output>=cal->max_output){
	cal->output=cal->max_output;
	}
	if(cal->output<=0-cal->max_output){
	cal->output=0-cal->max_output;
	}
	
}
double pid_get_output(my_pid* ret){
  return ret->output;

}