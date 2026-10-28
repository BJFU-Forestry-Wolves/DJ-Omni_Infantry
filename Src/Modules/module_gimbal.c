/*
 * 文件：module_gimbal.c
 * 云台控制层：P轴生成MIT位置目标，Yaw轴使用角度/速度双环PID。
 * 本轮只整理注释，保持上一版恢复及局部命名修正后的可执行代码不变。
 */

#include "alg_pid.h"
#include "cmsis_os.h"
#include "sys_const.h"
#include "module_gimbal.h"
#include "protocol_common.h"
#include "app_ins.h"
#include "periph_DMmotor.h"
#include "module_dm4310.h"
#include "util_can.h"
#include "app_remote.h"

GimbalPitch_GimbalPitchTypeDef GimbalPitch_gimbalPitchControlData;
GimbalYaw_GimbalYawTypeDef GimbalYaw_gimbalYawControlData;

float TestM;

/**
 * 初始化P轴参考值、平滑状态与控制开关。
 * 保留原角度/速度PID参数初始化；当前达妙P轴控制主体直接计算MIT位置目标，不使用这两个PID输出。
 */
void GimbalPitch_InitGimbalPitch() {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();

    gimbalpitch->control_state = 1;
    gimbalpitch->output_state = 1;
    gimbalpitch->pitch_ref = 0.2;
    gimbalpitch->pitch_count = 0;
	gimbalpitch->pitch_ref_smooth =0;
    PID_InitPIDParam(&gimbalpitch->spdPIDParam, Const_GimbalPitchSpdParam[0][0], Const_GimbalPitchSpdParam[0][1], Const_GimbalPitchSpdParam[0][2], Const_GimbalPitchSpdParam[0][3],
                    Const_GimbalPitchSpdParam[0][4], Const_GimbalPitchSpdParam[1][0], Const_GimbalPitchSpdParam[1][1], Const_GimbalPitchSpdParam[2][0], Const_GimbalPitchSpdParam[2][1],
                    Const_GimbalPitchSpdParam[3][0], Const_GimbalPitchSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&gimbalpitch->angPIDParam, Const_GimbalPitchAngParam[0][0], Const_GimbalPitchAngParam[0][1], Const_GimbalPitchAngParam[0][2], Const_GimbalPitchAngParam[0][3],
                    Const_GimbalPitchAngParam[0][4], Const_GimbalPitchAngParam[1][0], Const_GimbalPitchAngParam[1][1], Const_GimbalPitchAngParam[2][0], Const_GimbalPitchAngParam[2][1],
                    Const_GimbalPitchAngParam[3][0], Const_GimbalPitchAngParam[3][1], PID_POSITION);

}

/**
 * 初始化Yaw轴控制/输出开关与角度、速度双环PID参数。
 */
void GimbalYaw_InitGimbalYaw() {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->control_state = 1;
    gimbalyaw->output_state = 1;
    gimbalyaw->yaw_ref = 0;
    gimbalyaw->yaw_count = 0;

    PID_InitPIDParam(&gimbalyaw->spdPIDParam, Const_GimbalYawSpdParam[0][0], Const_GimbalYawSpdParam[0][1], Const_GimbalYawSpdParam[0][2], Const_GimbalYawSpdParam[0][3],
                    Const_GimbalYawSpdParam[0][4], Const_GimbalYawSpdParam[1][0], Const_GimbalYawSpdParam[1][1], Const_GimbalYawSpdParam[2][0], Const_GimbalYawSpdParam[2][1],
                    Const_GimbalYawSpdParam[3][0], Const_GimbalYawSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&gimbalyaw->angPIDParam, Const_GimbalYawAngParam[0][0], Const_GimbalYawAngParam[0][1], Const_GimbalYawAngParam[0][2], Const_GimbalYawAngParam[0][3],
                    Const_GimbalYawAngParam[0][4], Const_GimbalYawAngParam[1][0], Const_GimbalYawAngParam[1][1], Const_GimbalYawAngParam[2][0], Const_GimbalYawAngParam[2][1],
                    Const_GimbalYawAngParam[3][0], Const_GimbalYawAngParam[3][1], PID_POSITION);

}

/**
 * 返回P轴控制结构体地址。
 */
GimbalPitch_GimbalPitchTypeDef* GimbalPitch_GetGimbalPitchPtr() {
    return &GimbalPitch_gimbalPitchControlData;
}

/**
 * 返回Yaw轴控制结构体地址。
 */
GimbalYaw_GimbalYawTypeDef* GimbalYaw_GetGimbalYawPtr() {
    return &GimbalYaw_gimbalYawControlData;
}

/**
 * 设置P轴计算开关。仅改变软件状态，不发送达妙失能命令。
 */
void GimbalPitch_SetGimbalPitchControlState(uint8_t state) {
    GimbalPitch_GimbalPitchTypeDef *gimbalPitch = GimbalPitch_GetGimbalPitchPtr();

    gimbalPitch->control_state = state;
}

/**
 * 设置Yaw轴PID计算开关。关闭计算不会自动清零先前电机输出。
 */
void GimbalYaw_SetGimbalYawControlState(uint8_t state) {
    GimbalYaw_GimbalYawTypeDef *gimbalYaw = GimbalYaw_GetGimbalYawPtr();

    gimbalYaw->control_state = state;
}

/**
 * 设置P轴软件输出标志。当前共享组发送会检查它；它不直接控制达妙使能或MIT发送。
 */
void GimbalPitch_SetGimbalPitchOutputState(uint8_t state) {
    GimbalPitch_GimbalPitchTypeDef *gimbalPitch = GimbalPitch_GetGimbalPitchPtr();

    gimbalPitch->output_state = state;
}

/**
 * 设置Yaw软件输出标志，由GimbalYaw_Output读取。
 */
void GimbalYaw_SetGimbalYawOutputState(uint8_t state) {
    GimbalYaw_GimbalYawTypeDef *gimbalYaw = GimbalYaw_GetGimbalYawPtr();

    gimbalYaw->output_state = state;
}

/**
 * 把输入增量累加到P轴位置参考，并用DMpitchrefMin/Max钳位。
 * 这是增量接口，不是直接设定绝对位置；当前参考以rad参与后续计算。
 */
void GimbalPitch_SetPitchRef(float pitch_ref) {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
//    float UMAX_PITCH_DEG = Const_DMPITCH_UMAXANGLE;
//	float DMAX_PITCH_DEG = Const_DMPITCH_DMAXANGLE;

    gimbalpitch->pitch_ref += pitch_ref;
	LimitMaxMin(gimbalpitch->pitch_ref, DMpitchrefMax,DMpitchrefMin);
}

/**
 * 直接赋值Yaw角度参考，后续与IMU累计Yaw角比较。
 * 与P轴累加接口不同；角度单位须与INS.YawTotalAngle保持一致。
 */
void GimbalYaw_SetYawRef(float yaw_ref) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->yaw_ref = yaw_ref;
}

/**
 * 依据底盘模式和相对Yaw角限制输入ref；超界且继续越界时返回0。
 * 当前GimbalYaw_Control没有调用此函数，不能把它当成已生效的Yaw限位。
 */
float Gimbal_LimitYaw(float ref) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();

	if (buscomm->cha_mode  == Cha_Gyro)
        return ref;
	else if (((ins->YawTotalAngle - buscomm->yaw_ref < -Const_YAW_MAXANGLE) && (ref > 0)) ||
             ((ins->YawTotalAngle - buscomm->yaw_ref >  Const_YAW_MAXANGLE) && (ref < 0)))
        return 0.0f;
    else return ref;
}

/**
 * 保存外部Yaw位置反馈到结构体；当前控制函数实际直接读取INS累计Yaw角。
 */
void GimbalYaw_SetIMUYawPositionFdb(float imu_yaw_position_fdb) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->yaw_position_fdb = imu_yaw_position_fdb;
}

/**
 * 保存外部Yaw速度反馈；当前速度环实际直接读取INS.Gyro[Z_INS]。
 */
void GimbalYaw_SetIMUYawSpeedFdb(float imu_yaw_speed_fdb) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->yaw_speed_fdb = imu_yaw_speed_fdb;
}

float out_put_data;
float imu_error;
float  IMUfdb =0.0;
/**
 * 计算达妙P轴MIT位置目标：参考平滑值减去IMU补偿，再进行软件钳位。
 * 此函数只更新motor[Motor1].ctrl，不发送CAN帧；周期发送由任务中的dm_motor_ctrl_send负责。
 * dm_motor_init_test会每轮重新写入全局DM_KP/DM_KD，直接在Watch修改ctrl增益会被覆盖。
 */
void GimbalPitch_Control() {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();

    if (gimbalpitch->control_state != 1) return;

    gimbalpitch->filter_alpha = 0.1f;  // alpha越小平滑越强，响应越慢；每轮设为0.1。
	 dm_motor_init_test();
     imu_error = ins->Roll * PI / 180.0f + Const_PITCH_MOTOR_INIT_OFFSETf;  // 把IMU的Roll从度转换为rad，再加装配零偏。

    gimbalpitch->pitch_ref_smooth += gimbalpitch->filter_alpha *(gimbalpitch->pitch_ref - gimbalpitch->pitch_ref_smooth);  // 一阶平滑：新值=旧值+alpha*(参考-旧值)。
      float target_pos = gimbalpitch->pitch_ref_smooth-imu_error*IMUfdb;  // 平滑参考减去IMUfdb比例的IMU补偿，得到P轴位置目标。

   // motor[Motor1].ctrl.vel_set = v_imu;
	motor[Motor1].ctrl.pos_set =Gimbal_DMLimitPitch(target_pos) ;  // 只写入经过软件钳位的位置给定，真正发送由任务负责。

}

float gimbalyaw_fdb;

/**
 * Yaw角度外环产生速度参考，速度内环产生Motor_YawMotor.output。
 * 位置反馈取INS累计Yaw角，速度反馈取Z轴陀螺仪；单位及PID参数沿用原工程。
 * 这里只更新输出缓存，实际CAN组帧由GimbalYaw_Output发送。
 */
void GimbalYaw_Control() {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();

    if (gimbalyaw->control_state != 1) return;

    PID_SetPIDRef(&gimbalyaw->angPID, gimbalyaw->yaw_ref);
    PID_SetPIDFdb(&gimbalyaw->angPID, ins->YawTotalAngle);
    PID_CalcPID(&gimbalyaw->angPID, &gimbalyaw->angPIDParam);
	gimbalyaw_fdb = ins->YawTotalAngle;
    PID_SetPIDRef(&gimbalyaw->spdPID, PID_GetPIDOutput(&gimbalyaw->angPID));
    PID_SetPIDFdb(&gimbalyaw->spdPID, ins->Gyro[Z_INS]);
    PID_CalcPID(&gimbalyaw->spdPID, &gimbalyaw->spdPIDParam);

    Motor_SetMotorOutput(&Motor_YawMotor, PID_GetPIDOutput(&gimbalyaw->spdPID));

}

/**
 * 发送CAN1的0x1FF共享组，包含Yaw与拨弹电机。
 * Motor_GimbalMotors是历史组名，并非达妙P轴对象。
 * 保留原行为：P与Yaw的output_state都必须为1；任一关闭都会阻断该共享组发送。
 */
void GimbalYaw_Output(void) {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    if (gimbalpitch->output_state != 1) return;
	  if (gimbalyaw->output_state != 1) return;
    Motor_SendMotorGroupOutput(&Motor_GimbalMotors);
	//  Motor_SendMotorGroupOutput(&Motor_PitchMotors1);
}

/**
 * 将达妙P轴目标位置钳位到Const_DMPITCH_DMAXANGLE与UMAXANGLE之间。
 * 输入和返回值均为rad；当前函数只使用这两个边界，不读取实际位置或硬件限位。
 */
float Gimbal_DMLimitPitch(float ref)
{
	Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
	INS_INSTypeDef *ins = INS_GetINSPtr();

	if (ref > Const_DMPITCH_UMAXANGLE)
	{
        return Const_DMPITCH_UMAXANGLE;
	}
	else if(ref < Const_DMPITCH_DMAXANGLE)
	{
			return Const_DMPITCH_DMAXANGLE;
	}else
          return ref;

}
