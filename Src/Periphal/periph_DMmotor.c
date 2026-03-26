
#include "periph_DMmotor.h"
#include "util_can.h"
#include "sys_const.h"


DMmotor_t motor[num];


/**
************************************************************************
* @brief:      	dm4310_motor_init: DM4310????????????
* @param:      	void
* @retval:     	void
* @details:    	?????1??DM4310???????????????????????????
*               ????ID???????????????????????
************************************************************************
**/
void dm_motor_init(void)
{
	// ????Motor1???????
	motor[Motor1].id = 0x01;			// 电机ID设置为0x01   stdid  主控->电机 
	motor[Motor1].mst_id = 0x00;		// Master ID设置为0x00       电机->主控
	motor[Motor1].tmp.read_flag = 1;
	motor[Motor1].ctrl.mode 	= mit_mode;		// 使用MIT模式
	motor[Motor1].ctrl.pos_set 	= 0.4f;			// 初始目标位置  pos范围是-1到+1 ，角度=pos*716.2° 0.32
	motor[Motor1].ctrl.vel_set 	= 0.0f;			// 速度前馈  6.5
	motor[Motor1].ctrl.kp_set 	= DM_KP;		// 位置增益Kp=20
	motor[Motor1].ctrl.kd_set 	= DM_KD;			// 阻尼增益Kd=0.3
	motor[Motor1].ctrl.tor_set 	= 0.1f;			// 扭矩前馈
	motor[Motor1].ctrl.cur_set 	= 0.0f;			// MIT模式不使用此参数
	motor[Motor1].tmp.PMAX		= 12.5f;		// 位置映射范围    此参数不应该修改
	motor[Motor1].tmp.VMAX		= 30.0f;		// 速度映射范围
	motor[Motor1].tmp.TMAX		= 10.0f;		// 扭矩映射范围
}


void dm_motor_init_test(void)
{
	motor[Motor1].ctrl.kp_set 	= DM_KP;		// 位置增益Kp=20
	motor[Motor1].ctrl.kd_set 	= DM_KD;			// 阻尼增益Kd=0.3
}
/**
************************************************************************
* @brief:      	dm4310_enable: 启用DM4310电机控制模式函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式启用相应的模式，通过CAN总线发送启用命令
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/
void dm_motor_enable(CAN_HandleTypeDef* hcan, DMmotor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			enable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case pos_mode:
			enable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case spd_mode:
			enable_motor_mode(hcan, motor->id, SPD_MODE);
			break;
		case psi_mode:
			enable_motor_mode(hcan, motor->id, PSI_MODE);
			break;
	}	
}


/**
************************************************************************
* @brief:      	dm4310_disable: 禁用DM4310电机控制模式函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式禁用相应的模式，通过CAN总线发送禁用命令
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/
void dm_motor_disable(CAN_HandleTypeDef* hcan, DMmotor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			disable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case pos_mode:
			disable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case spd_mode:
			disable_motor_mode(hcan, motor->id, SPD_MODE);
			break;
		case psi_mode:
			disable_motor_mode(hcan, motor->id, PSI_MODE);
			break;
	}	
	dm_motor_clear_para(motor);
}

/**
************************************************************************
* @brief:      	dm4310_clear: 清除DM4310电机控制参数函数
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	将DM4310电机的命令参数和控制参数清零，包括位置、速度、
*               比例增益(KP)、微分增益(KD)和扭矩
************************************************************************
**/
void dm_motor_clear_para(DMmotor_t *motor)
{
	motor->ctrl.kd_set 	= 0;
	motor->ctrl.kp_set	= 0;
	motor->ctrl.pos_set = 0;
	motor->ctrl.vel_set = 0;
	motor->ctrl.tor_set = 0;
	motor->ctrl.cur_set = 0;
}

/**
************************************************************************
* @brief:      	enable_motor_mode: 启用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要开启的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
void enable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
void disable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	dm4310_ctrl_send: 发送DM4310电机控制命令函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式发送相应的命令到DM4310电机
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/
void dm_motor_ctrl_send(CAN_HandleTypeDef* hcan, DMmotor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			mit_ctrl(hcan, motor, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
			break;
		case pos_mode:
			//pos_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
			break;
		case spd_mode:
			//spd_ctrl(hcan, motor->id, motor->ctrl.vel_set);
			break;
		case psi_mode:
			//psi_ctrl(hcan, motor->id,motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.cur_set);
			break;
	}	
}
/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void mit_ctrl(CAN_HandleTypeDef* hcan, DMmotor_t *motor, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
	vel_tmp = float_to_uint(vel, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
	tor_tmp = float_to_uint(tor, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
	kp_tmp  = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,  KD_MIN, KD_MAX, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩以及相关温度参数
************************************************************************
**/
void dm_motor_fbdata(DMmotor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, -motor->tmp.PMAX, motor->tmp.PMAX, 16); // (-12.5,12.5)
	motor->para.vel = uint_to_float(motor->para.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12); // (-18.0,18.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
}


void dm_motor_detect(DMmotor_t *motor)
{
	if(motor->para.state==1)
		return;
	else
	{
	  dm_motor_enable(&hcan1, &motor[Motor1]);	
	}
}
 /**
************************************************************************
* @brief:      	read_motor_data: 发送读取寄存器命令
* @param[in]:   id:    电机can id
* @param[in]:   rid:   寄存器地址
* @retval:     	void
* @details:    	读取电机参数
************************************************************************
**/
void read_motor_data(uint16_t id, uint8_t rid) 
{
	uint8_t can_id_l = id & 0xFF;       // 低 8 位
    uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位
	
	uint8_t data[4] = {can_id_l, can_id_h, 0x33, rid};
	canx_send_data(&hcan1, 0x7FF, data, 4);
}

/**
************************************************************************
* @brief:      	read_motor_ctrl_fbdata: 发送读取电机反馈数据的命令
* @param[in]:   id:    电机can id
* @retval:     	void
* @details:    	读取电机控制反馈的数据
************************************************************************
**/
void read_motor_ctrl_fbdata(uint16_t id) 
{
	uint8_t can_id_l = id & 0xFF;       // 低 8 位
    uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位
	
	uint8_t data[4] = {can_id_l, can_id_h, 0xCC, 0x00};
	canx_send_data(&hcan1, 0x7FF, data, 4);
}
/**
************************************************************************
* @brief:      	write_motor_data: 发送写寄存器命令
* @param[in]:   id:    电机can id
* @param[in]:   rid:   寄存器地址
* @param[in]:   d0-d3: 写入的数据
* @retval:     	void
* @details:    	向寄存器写入数据
************************************************************************
**/
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;
	
	uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3};
	canx_send_data(&hcan1, 0x7FF, data, 8);
}
/**
************************************************************************
* @brief:      	save_motor_data: 发送保存命令
* @param[in]:   id:    电机can id
* @param[in]:   rid:   寄存器地址
* @retval:     	void
* @details:    	保存写入的电机参数
************************************************************************
**/
void save_motor_data(uint16_t id, uint8_t rid) 
{
	uint8_t can_id_l = id & 0xFF;       // 低 8 位
    uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位
	
	uint8_t data[4] = {can_id_l, can_id_h, 0xAA, 0x01};
	canx_send_data(&hcan1, 0x7FF, data, 4);
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}