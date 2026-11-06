#include "app_chassis.h"
#include "module_chassis.h"
#include "module_power.h"
#include "periph_Ina226.h"

/**
  * @brief          Chassis task
  * @param          NULL
  * @retval         NULL
  */
void Chassis_Task(void const * argument) {

    for(;;) {
		SuperCap_CheckPower(&SuperCap);
		Ina226_Update();   /* 刷新功率计在线判据/状态/k_safe；不调用则 is_online 永远停在 1 */
			Chassis_Control();
      Chassis_Output();
      osDelay(2);
    }
}
