/*
 * 文件：periph_DMmotor.c
 * 当前达妙P轴驱动：MIT组帧、使能/失能、反馈解码和寄存器请求。
 * 本轮只整理注释，保持上一版恢复及局部命名修正后的可执行代码不变。
 */

#include "periph_DMmotor.h"
#include "util_can.h"
#include "sys_const.h"

DMmotor_t motor[num];

/**
 * 初始化唯一达妙对象motor[Motor1]的地址、MIT模式、默认控制值与协议映射范围。
 * 当前实物为DM-J4340P-2EC，控制ID=6，反馈Master ID=6。
 * 这里只配置主控内存，不会把ID、模式、范围或超时写进驱动器。
 */
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

/**
 * 重新装载全局DM_KP和DM_KD。名称虽含test，但原P轴控制每轮都会调用。
 */
void dm_motor_init_test(void)
{
	motor[Motor1].ctrl.kp_set 	= DM_KP;		   // MIT位置刚度增益，当前来自全局DM_KP。
	motor[Motor1].ctrl.kd_set 	= DM_KD;			   // MIT速度阻尼增益，当前来自全局DM_KD。
}

/**
 * 按ctrl.mode选定模式地址偏移，并发送达妙使能命令；不等于持续控制。
 */
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
 * 发送达妙失能帧，然后清零本地控制值。
 * 若周期任务仍运行dm_motor_detect，后者可能再次使能电机。
 */
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
 * 只清零本地位置、速度、KP、KD、前馈转矩和电流给定。
 * 不发送CAN，也不修改驱动器状态、控制模式或ID；全零控制仍不等于失能。
 */
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
 * 标准数据帧：ID=motor_id+模式偏移，8字节FF FF FF FF FF FF FF FC。
 * MIT偏移为0，当前控制ID6对应标准ID0x006；驱动回应需由接收路径确认。
 */
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
 * 标准数据帧：ID=motor_id+模式偏移，8字节FF FF FF FF FF FF FF FD。
 */
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
 * 根据ctrl.mode分发周期控制。当前仅MIT分支真正发送，其他模式的发送语句仍被注释。
 */
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
 * 将位置16位以及速度/KP/KD/前馈转矩各12位拼成8字节MIT控制帧。
 * 位置rad、速度rad/s、转矩N*m；PMAX/VMAX/TMAX必须与驱动端一致。
 * 典型MIT控制关系为KP*(目标位置-反馈位置)+KD*(目标速度-反馈速度)+前馈转矩。
 * float_to_uint本身不钳位，调用方应保证所有输入在映射范围内。
 */
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
 * 解析普通8字节MIT状态反馈：D0高4位为状态，低4位为电机ID。
 * 状态0=失能，1=使能，0xD=通信超时；再解码位置、速度、转矩和两处温度。
 * 仅适用于普通反馈，参数读写应答应由上层正确区分，不能依靠本函数识别。
 */
void dm_motor_fbdata(DMmotor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
	motor->para.vel = uint_to_float(motor->para.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
	motor->para.tor = uint_to_float(motor->para.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
}

/**
 * 反馈状态为1时不处理；其他所有状态都会尝试再次使能。
 * 当前发送对象写死为hcan1上的motor[Motor1]，并非完整的故障诊断或通用多电机管理。
 */
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
 * 在CAN1用标准ID0x7FF发送4字节读寄存器请求：ID低字节、高位、0x33、RID。
 */
void read_motor_data(uint16_t id, uint8_t rid)
{
	uint8_t can_id_l = id & 0xFF;
    uint8_t can_id_h = (id >> 8) & 0x07;

	uint8_t data[4] = {can_id_l, can_id_h, 0x33, rid};
	canx_send_data(&hcan1, 0x7FF, data, 4);
}

/**
 * 在CAN1用标准ID0x7FF发送0xCC请求，主动查询电机控制反馈。
 */
void read_motor_ctrl_fbdata(uint16_t id)
{
	uint8_t can_id_l = id & 0xFF;
    uint8_t can_id_h = (id >> 8) & 0x07;

	uint8_t data[4] = {can_id_l, can_id_h, 0xCC, 0x00};
	canx_send_data(&hcan1, 0x7FF, data, 4);
}

/**
 * 在CAN1发送8字节参数写入请求，命令0x55；四个数据字节由调用者按寄存器类型编码。
 * 注意现有ID拆分使用低4位和右移4位，ID6可用，但不是通用11位CAN ID编码。本次仅说明，不修改实现。
 * 写入运行参数不等于保存到驱动Flash。
 */
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;

	uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3};
	canx_send_data(&hcan1, 0x7FF, data, 8);
}

/**
 * 通过0x7FF发送[ID低字节,ID高位,0xAA,0x01]保存参数请求。
 * 现有rid形参未参与报文；驱动器保存参数应在失能状态进行。
 */
void save_motor_data(uint16_t id, uint8_t rid)
{
	uint8_t can_id_l = id & 0xFF;
    uint8_t can_id_h = (id >> 8) & 0x07;

	uint8_t data[4] = {can_id_l, can_id_h, 0xAA, 0x01};
	canx_send_data(&hcan1, 0x7FF, data, 4);
}

/**
 * 把[x_min,x_max]线性映射到[0,2^bits-1]并截断为整数。
 * 函数未做上下界钳位，也未防止x_max=x_min；超范围值可能打包错误。
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{

	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
 * 将无符号编码值按相同线性范围还原成浮点物理量。
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{

	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
