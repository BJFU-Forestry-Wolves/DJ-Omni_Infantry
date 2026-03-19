#ifndef __CHASSIC_TASK_H
#define __CHASSIC_TASK_H
#include "main.h"
#include "freertos.h"
#include "tim.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "pid.h"
void chassic_task(void const * argument);
void chassic_init(void );
void chassic_set_speed(void);
void tyres_get_output(void);
void chassic_mode_change(void);
void chassic_send_output(void);
void chassic_get_feed(void);
void ENcoder_Set(signed int value1, signed int value2,signed int value3,signed int value4);
void task_mode_change(void);
void gc_map_set_encoder(void);
typedef enum{
   speed_control,
	 encoder_control
}chassic_mode;
typedef struct{
    uint16_t dir;//0正转
	   signed int speed;
	   signed encoder;
	   signed last_encoder;
	  //uint16_t set_dir;
	    signed  int set_speed;
	    signed  int set_encoder;
	  my_pid * tyres_pid;//速度环
	  my_pid * encoder_pid;//位置环
}tyre;
typedef enum{
    task_num1,
	  task_num2,
	  task_num3,
	  task_num4
}task_num;
typedef struct{
    double V_x;
	  double V_y;
	  double V_z;
	  chassic_mode * Run_mode;//1速度控制 2位置控制
}chassic_move;
extern tyre chass_tyre[4];
void Chassic_Control_with_Angle(double Vx,double Vy,double Wz,double Angle_set);
void My_ins_end(void);
void My_ins_Restart(void);
#endif