#ifndef MODULE_DM4310_H
#define MODULE_DM4310_H


#ifdef __cplusplus
extern "C" {
#endif


#include "periph_DMmotor.h"
extern DMmotor_t motor[num];


typedef union
{
	float f_val;
	uint32_t u_val;
	uint8_t b_val[4];
}float_type_u;

void read_all_motor_data(DMmotor_t *motor);
void receive_motor_data(DMmotor_t *motor, uint8_t *data);
void can1_rx_callback(void);
#endif

#ifdef __cplusplus
}
#endif