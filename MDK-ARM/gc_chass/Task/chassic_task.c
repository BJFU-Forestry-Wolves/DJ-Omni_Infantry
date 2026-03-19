#include "chassic_task.h"
#include "pid.h"
#include "Angle_cal_task.h"
#include "math.h"
#include "app_ins.h"
#include "gpio.h"

tyre chass_tyre[4];
chassic_move chassic;//µ×ÅÌÔË¶¯·½Ïò
my_pid tyres_pid[4]; //ÂÖÌ¥
my_pid encoder_pid[4]; //±àÂëÆ÷
chassic_mode cha_run_mode=speed_control;
uint16_t task_finshed=0;//
task_num gc_map=task_num1;
extern double Yaw_Angle;
void chassic_task(void const * argument){
	 chassic_init();
	 osDelay(10);
	 gc_map_set_encoder();
   while(1){
		 chassic_mode_change();	
		 //chassic_set_speed();
		 //gc_map_set_encoder();
		 task_mode_change();
		 gc_map_set_encoder();
		 tyres_get_output();
		 chassic_send_output();
		 
//		 usart_printf("debug\r\n");
		 osDelay(5);
	 }


}
void chassic_init(void ){
	  for(int i=0;i<4;i++){
		chass_tyre[i].dir=0;
		chass_tyre[i].encoder=0;
		chass_tyre[i].speed=0;
		//chass_tyre[i].set_dir=0;
		chass_tyre[i].set_encoder=0;
		chass_tyre[i].set_speed=0;
		chass_tyre[i].tyres_pid=&tyres_pid[i];
		chass_tyre[i].encoder_pid=&encoder_pid[i];
		pid_init(&tyres_pid[i],17,3,0,200,500);//speed pid 
		pid_init(&encoder_pid[i],5.2,0.4,-0.2,300,300);//encoder pid
		}
		chassic.V_x=0;
		chassic.V_y=0;
		chassic.V_z=0;
		chassic.Run_mode=&cha_run_mode;
		HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_SET);
}
void chassic_set_speed(void){
//   case Chassis_SEP:
//	     chassis_status->Chassis_Vx = RC_Vx * REMOTE_CHASSIS_VX_GAIN;
//	     chassis_status->Chassis_Vy = RC_Vy * REMOTE_CHASSIS_VY_GAIN;
//			 chassis_status->Chassis_Wz = RC_Wz * REMOTE_CHASSIS_SEP_WZ_GAIN;
//			 chassis_status->Chassis_FontRight_Speed = -chassis_status->Chassis_Vx -chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;
//			 chassis_status->Chassis_FontLeft_Speed  =  chassis_status->Chassis_Vx -chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;
//				 hassis_status->Chassis_BackLeft_Speed  =  chassis_status->Chassis_Vx +chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;
//				  chassis_status->Chassis_BackRight_Speed = -chassis_status->Chassis_Vx +chassis_status->Chassis_Vy -chassis_status->Chassis_Wz;
    if(*chassic.Run_mode==speed_control){
	 chassic.V_x=0;
	 chassic.V_y=0;
	 chassic.V_z=0;
	 chass_tyre[0].set_speed=0-chassic.V_x-chassic.V_y-chassic.V_z;
	 chass_tyre[1].set_speed=0+chassic.V_x-chassic.V_y-chassic.V_z;
	 chass_tyre[2].set_speed=0+chassic.V_x+chassic.V_y-chassic.V_z;
	 chass_tyre[3].set_speed=0-chassic.V_x+chassic.V_y-chassic.V_z;
		}
		//strght forword
		if(*chassic.Run_mode==encoder_control){
		chass_tyre[0].set_encoder=750;//-sta
		chass_tyre[1].set_encoder=750;//+ sta
		chass_tyre[2].set_encoder=750;// + sta
		chass_tyre[3].set_encoder=-750;//+sta
		}

}
void tyres_get_output(void){
    if(*chassic.Run_mode==speed_control){
		 for(int i=0;i<4;i++){
		 pid_cal(chass_tyre[i].tyres_pid,chass_tyre[i].speed,chass_tyre[i].set_speed);
		 }
		}
		if(*chassic.Run_mode==encoder_control){
		 for(int i=0;i<4;i++){
		 pid_cal(chass_tyre[i].encoder_pid,chass_tyre[i].encoder,chass_tyre[i].set_encoder);
		 }
		}
		
	 
	 
	
	
	 
	 
}
void chassic_mode_change(void){
    *chassic.Run_mode=encoder_control;
}


void chassic_send_output(void){
	if(*chassic.Run_mode==speed_control){
		if(pid_get_output(chass_tyre[0].tyres_pid)>=0){HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
		}
		else{HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);
		}
		if(pid_get_output(chass_tyre[1].tyres_pid)>=0){HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);
		}
		else{HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_SET);
		}
		if(pid_get_output(chass_tyre[2].tyres_pid)>=0){HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET);
		}
		else{HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_SET);
		}
		if(pid_get_output(chass_tyre[3].tyres_pid)>=0){HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_3,GPIO_PIN_RESET);
		}
		else{HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_3,GPIO_PIN_SET);
		}
    __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,fabs(pid_get_output(chass_tyre[0].tyres_pid)));
	  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,fabs(pid_get_output(chass_tyre[1].tyres_pid)));
	  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,fabs(pid_get_output(chass_tyre[2].tyres_pid)));
	  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,fabs(pid_get_output(chass_tyre[3].tyres_pid)));
//		    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,500);
//	  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,500);
//	  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,500);
//	  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,500);
	}
		if(*chassic.Run_mode==encoder_control){
		if(pid_get_output(chass_tyre[0].encoder_pid)>=0){HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
		}
		else{HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);
		}
		if(pid_get_output(chass_tyre[1].encoder_pid)>=0){HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);
		}
		else{HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_SET);
		}
		if(pid_get_output(chass_tyre[2].encoder_pid)>=0){HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET);
		}
		else{HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_SET);
		}
		if(pid_get_output(chass_tyre[3].encoder_pid)>=0){HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_3,GPIO_PIN_RESET);
		}
		else{HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_3,GPIO_PIN_SET);
		}
    __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,fabs(pid_get_output(chass_tyre[0].encoder_pid)));
	  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,fabs(pid_get_output(chass_tyre[1].encoder_pid)));
	  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,fabs(pid_get_output(chass_tyre[2].encoder_pid)));
	  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,fabs(pid_get_output(chass_tyre[3].encoder_pid)));
//
//		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,0);
//	  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,0);
//	  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,0);
//	  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,0);

	}
}

void chassic_get_feed(){
//  if(*chassic.Run_mode==speed_control){
//	 chass_tyre[0].speed=(signed int)__HAL_TIM_GET_COUNTER(&htim1);
//	 __HAL_TIM_SET_COUNTER(&htim1,0);
//			 chass_tyre[1].speed=(signed int)__HAL_TIM_GET_COUNTER(&htim2);
//	 __HAL_TIM_SET_COUNTER(&htim2,0);
//			 chass_tyre[2].speed=(signed int)__HAL_TIM_GET_COUNTER(&htim4);
//	 __HAL_TIM_SET_COUNTER(&htim4,0);
//			 chass_tyre[3].speed=(signed int)__HAL_TIM_GET_COUNTER(&htim8);
//	 __HAL_TIM_SET_COUNTER(&htim8,0);
//	}
//	if(*chassic.Run_mode==encoder_control){
//	for(uint16_t i=0;i<4;i++){
//	   chass_tyre[i].speed=0;
//	}
//	uint16_t temp_speed;
		for(uint16_t i=0;i<4;i++){
		 chass_tyre[0].last_encoder=chass_tyre[0].encoder;
		}
	  uint16_t temp;
	  temp=__HAL_TIM_GET_COUNTER(&htim1);
	  if(temp>32767){chass_tyre[0].encoder=temp-65535;
		}
	  else{
	    chass_tyre[0].encoder=temp;
	  }
		
    temp=__HAL_TIM_GET_COUNTER(&htim3);
	  if(temp>32767){chass_tyre[1].encoder=temp-65535;
		}
	  else{
	    chass_tyre[1].encoder=temp;
	  }		
		
		temp=__HAL_TIM_GET_COUNTER(&htim4);
	  if(temp>32767){chass_tyre[2].encoder=temp-65535;
		}
	  else{
	    chass_tyre[2].encoder=temp;
	  }		
		
		temp=__HAL_TIM_GET_COUNTER(&htim8);
	  if(temp>32767){chass_tyre[3].encoder=temp-65535;
		}
	  else{
	    chass_tyre[3].encoder=temp;
	  }	
		for(uint16_t i=0;i<4;i++){
		chass_tyre[i].speed=chass_tyre[i].encoder-chass_tyre[i].last_encoder;
		}
//	uint16_t temp;
//	temp=( int32_t)__HAL_TIM_GET_COUNTER(&htim1);
//	if(temp>32767){chass_tyre[0].encoder=temp-65535;}
//	else{
//	chass_tyre[0].encoder=temp;
//	}
//	temp=__HAL_TIM_GET_COUNTER(&htim3);
//	if(temp>32767){chass_tyre[1].encoder=temp-65535;}
//	else{
//	chass_tyre[1].encoder=temp;
//	}
//	temp=( int32_t)__HAL_TIM_GET_COUNTER(&htim4);
//	if(temp>32767){chass_tyre[2].encoder=temp-65535;}
//	else{
//	chass_tyre[2].encoder=temp;
//	}
//	temp=( int32_t)__HAL_TIM_GET_COUNTER(&htim8);
//	if(temp>32767){chass_tyre[3].encoder=temp-65535;}
//	else{
//	chass_tyre[3].encoder=temp;
//	}
	
//	chass_tyre[1].encoder=(int32_t)__HAL_TIM_GET_COUNTER(&htim2);
//	if(chass_tyre[1].encoder>32767){chass_tyre[1].encoder=chass_tyre[1].encoder-65535;}
//	chass_tyre[2].encoder=(int32_t)__HAL_TIM_GET_COUNTER(&htim4);
//	if(chass_tyre[2].encoder>32767){chass_tyre[2].encoder=chass_tyre[2].encoder-65535;}
//	chass_tyre[3].encoder=(int32_t)__HAL_TIM_GET_COUNTER(&htim8);
//	if(chass_tyre[3].encoder>32767){chass_tyre[3].encoder=chass_tyre[3].encoder-65535;}
	
	
}
double my_fabs(double a,double b){
   if((a-b)>=0){
	  return (a-b);
	 }
	 else return (b-a);
 
}
void task_mode_change(){
	for(uint16_t j=0;j<5;j++){
    
		if(my_fabs(chass_tyre[0].set_encoder,chass_tyre[0].encoder)<=2&&my_fabs(chass_tyre[1].set_encoder,chass_tyre[1].encoder)<=2&&my_fabs(chass_tyre[2].set_encoder,chass_tyre[2].encoder)<=2&&my_fabs(chass_tyre[3].set_encoder,chass_tyre[3].encoder)<=2){
		task_finshed++;
		
		
		}
	}
	if(task_finshed>=4){
	   task_finshed=1;
		if(gc_map==task_num4){
		gc_map=task_num1;
		}
		else gc_map++;
		for(int i=0;i<4;i++){
		chass_tyre[i].encoder=0;
		}
		__HAL_TIM_SetCounter(&htim1,0);
		__HAL_TIM_SetCounter(&htim3,0);
		__HAL_TIM_SetCounter(&htim4,0);
		__HAL_TIM_SetCounter(&htim8,0);
	}
	else{
	   task_finshed=0;
	}
		if(task_finshed==1){
		
		task_finshed=0;
		}

}
void gc_map_set_encoder(){
    if(gc_map==task_num1){
		//ENcoder_Set(2000,-2000,-2000,-2000);
	  Chassic_Control_with_Angle(1000,0,0,0);
		}
		if(gc_map==task_num2){
//		ENcoder_Set(1500,1500,-1500,1500);
		Chassic_Control_with_Angle(0,0,0,90);
		}
		if(gc_map==task_num3){
		ENcoder_Set(1500,1500,-1500,1500);
		}
		if(gc_map==task_num4){
		ENcoder_Set(0,0,0,0);
		}

}

void ENcoder_Set(signed int value1, signed int value2,signed int value3,signed int value4){
    chass_tyre[0].set_encoder=value1;
	  chass_tyre[1].set_encoder=value2;
	  chass_tyre[2].set_encoder=value3;
	  chass_tyre[3].set_encoder=value4;
	  


}
void Chassic_Control_with_Angle(double Vx,double Vy,double Wz,double Angle_set){
    chass_tyre[0].set_encoder=(-Vx-Vy-Wz-(Angle_set-Yaw_Angle));
	  chass_tyre[1].set_encoder=(Vx-Vy-Wz-(Angle_set-Yaw_Angle));
	  chass_tyre[2].set_encoder=(Vx+Vy-Wz-(Angle_set-Yaw_Angle));
	  chass_tyre[3].set_encoder=-(-Vx+Vy-Wz-(Angle_set-Yaw_Angle));
	  //*chassic.Run_mode=speed_control;
	  
	  //pid_get_output();

}



void My_ins_end(void){
     Ins_reset

};



void My_ins_Restart(void){
     Ins_set
	   INS_Init();
	   Init_MPU9250();

}