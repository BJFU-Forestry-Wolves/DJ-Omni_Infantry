#include "app_referee.h"
#include "periph_draw.h"

static uint8_t referee_reset_flag = 0;

void Referee_Task(void const * argument) {

	
	forever{
		
		
		Referee_Update();
		osDelay(200);
	}
}

