#include "app_power_limiting.h"
void Power_Limting_Task(void const * argument){
	   Power_limiting_Init();
     while(1){
			Tyres_Power_Cal();
			Chassic_Power_Control();
		  osDelay(10);
		 }

}