/*
 * 文件：periph_motor.c
 * 普通电机组收发、编码器解码及历史达妙适配接口。
 * 本轮只整理注释，保持上一版恢复及局部命名修正后的可执行代码不变。
 */

#include "periph_motor.h"
#include "sys_const.h"
#include "stdio.h"
#include "stdlib.h"
#include "alg_power_limting.h"
#include "module_dm4310.h"
#include "module_power.h"

const uint32_t Const_Motor_MOTOR_OFFLINE_TIME = 200;
const uint32_t Const_Motor_MOTOR_TX_EXTID = 0x00;
const uint32_t Const_Motor_MOTOR_TX_DLC = 8;
const uint32_t Const_Motor_MOTOR_RX_DLC = 8;

Motor_MotorGroupTypeDef *Motor_groupHandle[MOTOR_GROUP_NUM];

Motor_MotorGroupTypeDef Motor_ChassisMotors;
Motor_MotorGroupTypeDef Motor_GimbalMotors;          // 历史名称：实际为CAN1/0x1FF的Yaw+拨弹共享组，不是达妙P轴。
Motor_MotorGroupTypeDef Motor_ShootMotors;
//Motor_MotorGroupTypeDef Motor_DM4310s;
//Motor_MotorGroupTypeDef Motor_PitchMotors1;

Motor_MotorTypeDef Motor_ChassisFontRightMotor;
Motor_MotorTypeDef Motor_ChassisFontLeftMotor;
Motor_MotorTypeDef Motor_ChassisBackLeftMotor;
Motor_MotorTypeDef Motor_ChassisBackRightMotor;
Motor_MotorTypeDef Motor_PitchMotor;  // 旧普通电机P轴对象，当前初始化被注释；不是达妙motor[Motor1]。
Motor_MotorTypeDef Motor_YawMotor;
Motor_MotorTypeDef Motor_ShootLeftMotor;
Motor_MotorTypeDef Motor_ShootRightMotor;
Motor_MotorTypeDef Motor_FeedMotor;
//Motor_MotorTypeDef Motor_DMMotor;

/**
 * 先处理超级电容反馈，再遍历已注册电机组，以CAN句柄和反馈ID匹配并调用各电机解码器。
 */
void Motor_EncoderDecodeCallback(CAN_HandleTypeDef* phcan, uint32_t stdid, uint8_t rxdata[], uint32_t len) {
	if (stdid == 0x211) {
        Parse_SuperCap_Feedback(rxdata,&SuperCap);
    }

    for (int i = 0; i < MOTOR_GROUP_NUM; i++) {
        for (int j = 0; j < Motor_groupHandle[i]->motor_num; j++) {
            if (Motor_groupHandle[i]->motor_handle[j] == NULL) continue;
            if ((phcan == Motor_groupHandle[i]->can_handle)
                  && (stdid == Motor_groupHandle[i]->motor_handle[j]->id) && phcan != NULL) {
                Motor_groupHandle[i]->motor_handle[j]->callback(Motor_groupHandle[i]->motor_handle[j], rxdata, len);
            }
        }
    }
}

/**
 * 登记3个电机组及其成员。
 * CAN1/0x1FF共享组：第3通道拨弹0x207，第4通道Yaw 0x208。
 * CAN2/0x200组用于摩擦轮；CAN1/0x200组用于底盘。达妙P轴不通过这些DJI电机组发送。
 */
void Motor_InitAllMotors() {
    Motor_groupHandle[0] = &Motor_GimbalMotors;
    Motor_InitMotorGroup(&Motor_GimbalMotors, Motor_TYPE_RM6020, 4, &hcan1, 0x1FF);
		Motor_InitMotor(&Motor_YawMotor,Motor_TYPE_RM6020,0x208,0.1,gm6020_encoder_callback);

		Motor_InitMotor(&Motor_FeedMotor, Motor_TYPE_RM2006, 0X207, 0.1, rm2006_encoder_callback);
	  Motor_GimbalMotors.motor_handle[3] = &Motor_YawMotor;  // 第4通道：Yaw反馈ID0x208，输出占本组D6/D7。
		
	  Motor_GimbalMotors.motor_handle[2] = &Motor_FeedMotor;  // 第3通道：拨弹反馈ID0x207，输出占本组D4/D5。

    Motor_groupHandle[1] = &Motor_ShootMotors;
    Motor_InitMotorGroup(&Motor_ShootMotors, Motor_TYPE_RM3508, 2, &hcan2, 0x200);
    Motor_InitMotor(&Motor_ShootRightMotor, Motor_TYPE_RM3508, 0x201, 0.1, rm3508_encoder_callback);
    Motor_InitMotor(&Motor_ShootLeftMotor, Motor_TYPE_RM3508, 0x202, 0.1, rm3508_encoder_callback);
	  //Motor_InitMotor(&Motor_PitchMotor, Motor_TYPE_RM6020, 0x206, 0.1, gm6020_encoder_callback);
    //Motor_InitMotor(&Motor_FeedMotor, Motor_TYPE_RM2006, 0X201, 0.1, rm2006_encoder_callback);
	  //Motor_ShootMotors.motor_handle[0] = &Motor_FeedMotor;
    Motor_ShootMotors.motor_handle[1] = &Motor_ShootLeftMotor;
    Motor_ShootMotors.motor_handle[0] = &Motor_ShootRightMotor;

		Motor_groupHandle[2] = &Motor_ChassisMotors;
    Motor_InitMotorGroup(&Motor_ChassisMotors, Motor_TYPE_RM3508, 4, &hcan1, 0x200);
	  Motor_InitMotor(&Motor_ChassisFontRightMotor,Motor_TYPE_RM3508,0x201,0.1,rm3508_encoder_callback);
	  Motor_InitMotor(&Motor_ChassisFontLeftMotor,Motor_TYPE_RM3508,0x202,0.1,rm3508_encoder_callback);
	  Motor_InitMotor(&Motor_ChassisBackLeftMotor,Motor_TYPE_RM3508,0x203,0.1,rm3508_encoder_callback);
	  Motor_InitMotor(&Motor_ChassisBackRightMotor,Motor_TYPE_RM3508,0x204,0.1,rm3508_encoder_callback);
	  Motor_ChassisMotors.motor_handle[0] = &Motor_ChassisFontRightMotor;
    Motor_ChassisMotors.motor_handle[1] = &Motor_ChassisFontLeftMotor;
    Motor_ChassisMotors.motor_handle[2] = &Motor_ChassisBackLeftMotor;
    Motor_ChassisMotors.motor_handle[3] = &Motor_ChassisBackRightMotor;

}

/**
 * 初始化单个电机对象的ID、类型、反馈回调和滤波参数；输出初值为0。
 */
void Motor_InitMotor(Motor_MotorTypeDef* pmotor, Motor_MotorTypeEnum type, uint16_t id, float fil_param,
                     Motor_EncoderCallbackFuncTypeDef callback) {
    if (pmotor == NULL) return;
    pmotor->last_update_time = 0;
    pmotor->type = type;
    pmotor->id = id;
    pmotor->init = 0;
    pmotor->is_online = 0;
    pmotor->output = 0;
    pmotor->callback = callback;
    Filter_LowPassInit(fil_param, &pmotor->fdb_fil_param);
}

/**
 * 保存发送CAN句柄和标准帧头，并清空4个通道的电机指针。
 * 之后由Motor_InitAllMotors按报文通道顺序填入实际成员。
 */
void Motor_InitMotorGroup(Motor_MotorGroupTypeDef* pgroup, Motor_MotorTypeEnum type, uint8_t motor_num, CAN_HandleTypeDef* phcan, uint16_t stdid) {
    if (pgroup == NULL) return;
    pgroup->motor_num = motor_num;
    pgroup->type = type;

    if (phcan == NULL) return;
    pgroup->can_handle = phcan;
    Can_InitTxHeader(&(pgroup->can_header), stdid, Const_Motor_MOTOR_TX_EXTID, Const_Motor_MOTOR_TX_DLC);

    for (int i = 0; i < 4; ++i) {
        pgroup->motor_handle[i] = NULL;
    }
}

/**
 * 更新电机输出缓存，不立即发送。
 */
void Motor_SetMotorOutput(Motor_MotorTypeDef* pmotor, float output) {
    pmotor->output = output;
}
/**
 * 更新底盘电机输出缓存；具体发送仍由电机组输出函数完成。
 */
void Motor_SetMotorOutputChassic(Motor_MotorTypeDef* pmotor, float output) {
	  pmotor->output = output;
}

/**
 * 对已连接的RM3508/RM2006/RM6020，将output乘1000并转成有符号16位报文值。
 * 返回uint16_t用于拆字节，负数按16位补码传输；函数未执行幅值限制。
 */
uint16_t Motor_GetMotorOutput(Motor_MotorTypeDef* pmotor) {
    if (pmotor == NULL) return 0;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED) return 0;

    int16_t ret = 0;
    if (pmotor->type == Motor_TYPE_RM3508 || pmotor->type == Motor_TYPE_RM2006 || pmotor->type ==Motor_TYPE_RM6020) {
        ret = (int16_t)(pmotor->output * 1000.0f);
        return (uint16_t)ret;
    }
    return 0;
}

/**
 * 按motor_handle[0..3]顺序将4个16位输出拼为8字节，并使用组内帧头发送。
 * 空通道经Motor_GetMotorOutput返回0；共享组必须保留所有成员的通道位置。
 */
void Motor_SendMotorGroupOutput(Motor_MotorGroupTypeDef *pgroup) {
    if (pgroup == NULL) return;
    uint8_t txdata[64];
	memset(txdata, 0, 64);
    txdata[0] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[0]) >> 8);
    txdata[1] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[0]);
    txdata[2] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[1]) >> 8);
    txdata[3] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[1]);
    txdata[4] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[2]) >> 8);
    txdata[5] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[2]);
    txdata[6] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[3]) >> 8);
    txdata[7] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[3]);
    Can_SendMessage(pgroup->can_handle, &(pgroup->can_header), txdata);
}

/**
 * 按注册顺序发送全部电机组；云台局部发送与此函数若同时使用会重复发送相同组。
 */
void Motor_SendMotorGroupsOutput() {
    for (int i = 0; i < MOTOR_GROUP_NUM; i++) {
        Motor_SendMotorGroupOutput(Motor_groupHandle[i]);
    }
}

/**
 * 遍历每组4个槽位；遇到Motor_IsMotorOffline返回0即返回0，否则返回1。
 * 保留原有返回语义和空槽参与判断的行为，不根据函数名推断含义。
 */
uint8_t Motor_IsAnyMotorOffline() {
    for (int i = 0; i < MOTOR_GROUP_NUM; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!Motor_IsMotorOffline(Motor_groupHandle[i]->motor_handle[j])) {
                return 0;
            }
        }
    }
    return 1;
}

/**
 * 按最近反馈时间判断：现实现返回1表示未超过离线阈值，即在线。
 * 这个返回语义与函数名Offline相反，调用者应以实际表达式为准。
 */
uint8_t Motor_IsMotorOffline(Motor_MotorTypeDef* pmotor) {
    if (pmotor == NULL) return 0;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED) return 0;
    uint32_t now = HAL_GetTick();
    return (now - pmotor->last_update_time) < Const_Motor_MOTOR_OFFLINE_TIME;
}

/**
 * 解码RM2006的角度、速度和电流；速度除36，连续输出角每电机转累加10度。
 * 该回调使用36:1减速换算，并更新last_update_time；其他字段更新情况以函数体为准。
 */
void rm2006_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    if (pmotor == NULL) return;
    if (len != 8) return;

    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    pmotor->encoder.angle   = (float)((int16_t)((uint16_t)rxbuff[0] << 8 | (uint16_t)rxbuff[1]));
    pmotor->encoder.speed   = (float)((int16_t)((uint16_t)rxbuff[2] << 8 | (uint16_t)rxbuff[3])) / 36.0f;
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxbuff[4] << 8 | (uint16_t)rxbuff[5]));
    pmotor->encoder.temp = 0;
    pmotor->init = 1;

    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;
    if (diff < -4096)
        pmotor->encoder.round_count++;
    else if (diff > 4096)
        pmotor->encoder.round_count--;

    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 10.0f +
                                       (float)pmotor->encoder.angle / 8192.0f * 10.0f;
    if (pmotor->encoder.round_count > 10000) {
        pmotor->encoder.consequent_angle -= 10 * pmotor->encoder.round_count;
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.round_count < -10000) {
        pmotor->encoder.consequent_angle += 10 * pmotor->encoder.round_count;
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;
    pmotor->last_update_time = HAL_GetTick();
}

/**
 * 解码GM6020角度、速度、电流，计算连续角和单圈角（度）。
 * 注意现有跨圈判断位于本帧角度赋值之前，且未更新last_update_time；本次不调整算法。
 */
void gm6020_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {

    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;
    if (diff < -5500)
        pmotor->encoder.round_count++;
    else if (diff > 5500)
        pmotor->encoder.round_count--;

    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    pmotor->encoder.angle   = (float)((int16_t)((uint16_t)rxbuff[0] << 8 | (uint16_t)rxbuff[1]));
    pmotor->encoder.speed   = (float)((int16_t)((uint16_t)rxbuff[2] << 8 | (uint16_t)rxbuff[3]));
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxbuff[4] << 8 | (uint16_t)rxbuff[5]));
    pmotor->encoder.temp = 0;
    pmotor->init = 1;

    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 360.0f +
                                       (float)pmotor->encoder.angle / 8192.0f * 360.0f;
    pmotor->encoder.limited_angle = (float)pmotor->encoder.angle / 8192.0f * 360.0f;
}

/**
 * 解码RM3508反馈；速度除19，连续角当前按电机转子角度累加。
 * 速度和角度的减速比处理并不完全相同，注释按现有表达式说明，不改变计算。
 */
void rm3508_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    if (pmotor == NULL) return;

    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
	  pmotor->encoder.last_speed=pmotor->encoder.speed;
    pmotor->encoder.angle   = (float)((int16_t)((uint16_t)rxbuff[0] << 8 | (uint16_t)rxbuff[1]));
    pmotor->encoder.speed   = (float)((int16_t)((uint16_t)rxbuff[2] << 8 | (uint16_t)rxbuff[3]))/19;
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxbuff[4] << 8 | (uint16_t)rxbuff[5]));
    pmotor->encoder.temp    = (float)((int16_t)((uint16_t)rxbuff[6]));

    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;
    if (diff < -4096)
        pmotor->encoder.round_count++;
    else if (diff > 4096)
        pmotor->encoder.round_count--;

    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 360.0f  +
                                       (float)pmotor->encoder.angle / 8192.0f * 360.0f;
    if (pmotor->encoder.round_count > 10000) {
        pmotor->encoder.consequent_angle -= 10 * pmotor->encoder.round_count;
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.round_count < -10000) {
        pmotor->encoder.consequent_angle += 10 * pmotor->encoder.round_count;
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;

    pmotor->last_update_time = HAL_GetTick();
}
/**
 * 旧通用电机接口适配：直接转交全局达妙motor[Motor1]解析。当前主路径为CAN接收分发。
 */
void dm4340_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len)
{

        dm_motor_fbdata(&motor[Motor1], rxbuff);
        receive_motor_data(&motor[Motor1], rxbuff);

}

/**
 * 旧通用达妙对象初始化接口，仅初始化Motor_MotorTypeDef，与dm_motor_init不是同一个数据结构。
 */
void dm4340_motor_init(Motor_MotorTypeDef* pmotor, Motor_MotorTypeEnum type, uint16_t id, float fil_param,
                     Motor_EncoderCallbackFuncTypeDef callback)
{
    if (pmotor == NULL) return;
    pmotor->last_update_time = 0;
    pmotor->type = type;
    pmotor->id = id;
    pmotor->init = 0;
    pmotor->is_online = 0;
    pmotor->output = 0;
    pmotor->callback = callback;
    Filter_LowPassInit(fil_param, &pmotor->fdb_fil_param);
}

/**
 * 旧打包接口的浮点编码，先钳位再线性映射；与periph_DMmotor.c中未钳位的版本不同。
 */
static uint32_t dm4310_float_to_uint(float value, float min_val, float max_val, uint8_t bits)
{
    value = fmaxf(min_val, fminf(max_val, value));
    float normalized = (value - min_val) / (max_val - min_val);
    uint32_t max_int = (1 << bits) - 1;
    return (uint32_t)(normalized * max_int);
}

/**
 * 旧固定参数MIT转矩接口，地址固定0x01，映射固定12.5/45/18。
 * 与当前ID6及映射12.5/10/28不一致，当前P轴任务使用dm_motor_ctrl_send而非此函数。
 * 保留原报文代码，不把这个历史接口解释成当前实物配置。
 */
void dm4340_motor_send_output(float torque)
{
    uint8_t txdata[64];
		memset(txdata, 0, 64);
		CAN_HandleTypeDef* can_handle = &hcan1;
    CAN_TxHeaderTypeDef can_header;
    Can_InitTxHeader(&can_header, 0x01, Const_Motor_MOTOR_TX_EXTID, Const_Motor_MOTOR_TX_DLC);

		uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
		pos_tmp = dm4310_float_to_uint(0, -12.5f, 12.5f, 16);
    vel_tmp = dm4310_float_to_uint(0, -45.0f, 45.0f, 12);
		tor_tmp = dm4310_float_to_uint(torque, -18.0f, 18.0f, 12);
    kp_tmp = dm4310_float_to_uint(0, 0.0f, 500.0f, 12);
    kd_tmp = dm4310_float_to_uint(0, 0.0f, 5.0f, 12);
    txdata[0] = (uint8_t)(pos_tmp	>> 8);
    txdata[1] = (uint8_t)(pos_tmp);
    txdata[2] = (uint8_t)(vel_tmp	>> 4);
    txdata[3] = (uint8_t)(((vel_tmp&0xF)<<4) | (kp_tmp>>8));
    txdata[4] = (uint8_t)(kp_tmp);
    txdata[5] = (uint8_t)(kp_tmp >> 4);
    txdata[6] = (uint8_t)(((kd_tmp&0xF)<<4) | (tor_tmp >> 8));
    txdata[7] = (uint8_t)(tor_tmp);
    Can_SendMessage(&hcan1, &can_header, txdata);
}

//void dm4340_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len)
//{
//    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
//	  pmotor->encoder.last_speed=pmotor->encoder.speed;
//    pmotor->encoder.angle   = (float)((int16_t)((uint16_t)rxbuff[1] << 8 | (uint16_t)rxbuff[2]));
//    pmotor->encoder.speed   = (float)((int16_t)((uint16_t)rxbuff[3] << 4 | (uint16_t)rxbuff[4] >> 4));
//    pmotor->encoder.current = (float)((int16_t)((uint16_t)(rxbuff[4] & 0x0F) << 8 | (uint16_t)rxbuff[5]));
//}
