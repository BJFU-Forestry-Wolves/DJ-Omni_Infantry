/*
 * 文件：periph_DMmotor.h
 * 达妙协议对象。ctrl是待发送给定，para是反馈，tmp是驱动参数缓存。
 * 本轮只整理注释，保持上一版恢复及局部命名修正后的可执行代码不变。
 */

#ifndef PERIPH_DMMOTOR_H
#define PERIPH_DMMOTOR_H

#define MIT_MODE 			0x000  // MIT控制的CAN ID偏移，和软件枚举mit_mode=1不是一回事。
#define POS_MODE			0x100
#define SPD_MODE			0x200
#define PSI_MODE		  	0x300

#ifdef __cplusplus
extern "C" {
#endif

#include "util_pwm.h"

typedef enum
{
    Motor1,
    num
} motor_num;

typedef enum
{
	mit_mode = 1,
	pos_mode = 2,
	spd_mode = 3,
	psi_mode = 4
} mode_e;

typedef struct
{
	uint8_t read_flag;
	uint8_t write_flag;
	uint8_t save_flag;

    float UV_Value;
    float KT_Value;
    float OT_Value;
    float OC_Value;
    float ACC;
    float DEC;
    float MAX_SPD;
    uint32_t MST_ID;
    uint32_t ESC_ID;
    uint32_t TIMEOUT;	   // 通信超时计数，单位50us；2000对应100ms。仅改本地字段不会写驱动。
    uint32_t cmode;
    float  	 Damp;
    float    Inertia;
    uint32_t hw_ver;
    uint32_t sw_ver;
    uint32_t SN;
    uint32_t NPP;
    float    Rs;
    float    Ls;
    float    Flux;
    float    Gr;
    float    PMAX;		   // MIT位置映射端点，单位rad；不是机械限位。
    float    VMAX;		   // MIT速度映射端点，单位rad/s。
    float    TMAX;		   // MIT转矩映射端点，单位N*m。
    float    I_BW;
    float    KP_ASR;
    float    KI_ASR;
    float    KP_APR;
    float    KI_APR;
    float    OV_Value;
    float    GREF;
    float    Deta;
    float 	 V_BW;
    float 	 IQ_cl;
    float    VL_cl;
    uint32_t can_br;
    uint32_t sub_ver;
	float 	 u_off;
	float	 v_off;
	float	 k1;
	float 	 k2;
	float 	 m_off;
	float  	 dir;
	float	 p_m;
	float	 x_out;
} esc_inf_t;

typedef struct
{
    int id;  // 普通反馈D0低4位携带的驱动ID。
    int state;  // 驱动反馈状态：0失能、1使能、0xD通信超时。
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;  // 解析位置，单位rad。
    float vel;  // 解析速度，单位rad/s。
    float tor;  // 解析转矩，单位N*m。
    float Kp;
    float Kd;
    float Tmos;  // 驱动MOS温度，单位摄氏度。
    float Tcoil;  // 电机绕组温度反馈，单位摄氏度。
} motor_fbpara_t;

typedef struct
{
    uint8_t mode;
    float pos_set;  // MIT目标位置，单位rad。
    float vel_set;  // MIT目标速度，单位rad/s。
    float tor_set;  // MIT前馈转矩，单位N*m。
	float cur_set;
    float kp_set;  // MIT位置比例增益。
    float kd_set;  // MIT速度阻尼增益。
} motor_ctrl_t;

typedef struct
{
    uint16_t id;  // 主控控制地址；与反馈地址mst_id不是同一语义。
	uint16_t mst_id;  // 反馈CAN地址的本地记录，不会自动更新驱动配置或接收判断。
    motor_fbpara_t para;  // 最近一次普通MIT反馈的解析结果，暂停时可能是旧值。
    motor_ctrl_t ctrl;  // 主控待发送控制值；修改它不会立即发帧。
	esc_inf_t tmp;  // 驱动参数缓存/映射配置，不代表每项已成功从驱动读回。
} DMmotor_t;

void dm_motor_init(void);
void dm_motor_clear_para(DMmotor_t *motor);
void dm_motor_enable(CAN_HandleTypeDef* hcan, DMmotor_t *motor);
void dm_motor_disable(CAN_HandleTypeDef* hcan, DMmotor_t *motor);
void enable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void mit_ctrl(CAN_HandleTypeDef* hcan, DMmotor_t *motor, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void read_motor_data(uint16_t id, uint8_t rid);
void read_motor_ctrl_fbdata(uint16_t id);
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void save_motor_data(uint16_t id, uint8_t rid);
void dm_motor_fbdata(DMmotor_t *motor, uint8_t *rx_data);
void dm_motor_ctrl_send(CAN_HandleTypeDef* hcan, DMmotor_t *motor);
void dm_motor_detect(DMmotor_t *motor);
void dm_motor_init_test(void);
#endif

#ifdef __cplusplus
}
#endif
