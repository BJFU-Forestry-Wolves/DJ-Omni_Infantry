/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_ins.c
 *  Description  : Attitude solution
 *  LastEditors  : Polaris
 *  Date         : 2023-01-24 01:43:30
 *  LastEditTime : 2023-05-05 17:32:53
 */


#include "app_ins.h"
#include "alg_quaternionEKF.h"
#include "alg_math.h"
#include "sys_dwt.h"
#include "mpu_9250.h"
//#include "periph_bmi088.h"

INS_INSTypeDef INS;
INS_DataTypeDef Param;
uint16_t update_flag_de=0;
double Yaw_Angle;
uint32_t decrease_time=0;
float test_t;
double accx1;
double accy1;
double accz1;
void Ins_Task(void const * argument) {
    //BMI088_BMI088DataTypeDef* bmi088 = BMI088_GetBMI088DataPtr();
    const float gravity[3] = {0, 0, 9.81f};
    static float dt = 0, t = 0;
    uint32_t INS_DWT_Count = 0;
    const float xb[3] = {1, 0, 0};
    const float yb[3] = {0, 1, 0};
    const float zb[3] = {0, 0, 1};

    while(1) {        
        static uint32_t count = 0;
        dt = DWT_GetDeltaT(&INS_DWT_Count);
			  test_t=dt;
        t += dt;
        if ((count % 1) == 0) {
            //BMI088_BMI088DecodeData();
            		READ_MPU9250_ACCEL();
		            osDelay(1);
		            READ_MPU9250_GYRO();
		            osDelay(1);
		            READ_MPU9250_MAG();
		            osDelay(1);
	if(Accel_x1<32764) accx1=Accel_x1/16384.0;//璁＄畻x杞村姞閫熷害
	else              accx1=1-(Accel_x1-49152)/16384.0f;
	if(Accel_y1<32764) accy1=Accel_y1/16384.0;//璁＄畻y杞村姞閫熷害
	else              accy1=1-(Accel_y1-49152)/16384.0f;
	if(Accel_z1<32764) accz1=Accel_z1/16384.0f;//璁＄畻z杞村姞閫熷害
	else              accz1=(Accel_z1-49152)/16384.0f;
					  	if(Gyro_x1<32768) Gyro_x1=-(Gyro_x1/16.4f);
	            if(Gyro_x1>32768) Gyro_x1=+(65535-Gyro_x1)/16.4f;
	            if(Gyro_y1<32768) Gyro_y1=-(Gyro_y1/16.4f);
	            if(Gyro_y1>32768) Gyro_y1=+(65535-Gyro_y1)/16.4f;
	            if(Gyro_z1<32768) Gyro_z1=-(Gyro_z1/16.4f);
	            if(Gyro_z1>32768) Gyro_z1=+(65535-Gyro_z1)/16.4f;
            INS.Accel[X_INS] = accx1*9.81f;
            INS.Accel[Y_INS] =accy1*9.81f;
            INS.Accel[Z_INS] = accz1*9.81f;
            INS.Gyro[X_INS] = Gyro_x1*3.14f/180.0f;
            INS.Gyro[Y_INS] = Gyro_y1*3.14f/180.0f;
            INS.Gyro[Z_INS] = Gyro_z1*3.14f/180.0f;

            // demo function
					  //坐标轴转化，先求出逆旋转矩阵
					  //后面用到的三轴加速度，角速度全部为转换坐标后的结果
             Param_Correction(&Param, INS.Gyro, INS.Accel);

            // Calculate the angle between the gravity acceleration vector and the XY axis of the b system, 
            // which can be used as a function extension.
					  //根据加速度计计算出对应的roll，pitch角
            INS.atanxz = -atan2f(INS.Accel[X_INS], INS.Accel[Z_INS]) * 180 / PI;
            INS.atanyz = atan2f(INS.Accel[Y_INS], INS.Accel[Z_INS]) * 180 / PI;
            // Core function, EKF update quaternion
            QuaternionEKF_Update(INS.Gyro[X_INS], INS.Gyro[Y_INS], INS.Gyro[Z_INS], INS.Accel[X_INS], INS.Accel[Y_INS], INS.Accel[Z_INS], dt);
            memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

            // The base vector of the airframe system is converted to the navigation coordinate system. In this example, 
            // the inertial system is selected as the navigation system
            BodyFrameToEarthFrame(xb, INS.xn, INS.q);
            BodyFrameToEarthFrame(yb, INS.yn, INS.q);
            BodyFrameToEarthFrame(zb, INS.zn, INS.q);

            // Convert the gravity from the navigation coordinate system n to the aircraft system b, 
            // and then calculate the motion acceleration according to the accelerometer data
            float gravity_b[3];
            EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
            for (uint8_t i = 0; i < 3; i++) {
                INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
            }
            BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);
            INS.Yaw = QEKF_INS.Yaw;
						decrease_time++;
						Yaw_Angle=INS.YawTotalAngle;
						if(update_flag_de){
						  Yaw_Angle-=17179869.2;
							update_flag_de=0;
						}
						Yaw_Angle-=0.021*decrease_time;
						if(decrease_time>=4294967200){
						  decrease_time=0;
							update_flag_de=1;
						}
						//INS.Yaw = INS.Yaw;
            INS.Pitch = QEKF_INS.Pitch;
            INS.Roll = QEKF_INS.Roll;
            INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
        }
        count++;
      osDelay(1);
    }
}

INS_INSTypeDef *INS_GetINSPtr() {
    return &INS;
}

void INS_Init() {
    Param.scale[X_INS] = 1;
    Param.scale[Y_INS] = 1;
    Param.scale[Z_INS] = 1;
    Param.Yaw = 0;
    Param.Pitch = 0;
    Param.Roll = 0;
    Param.flag = 1;

    QuaternionEKF_Init(10, 0.01f, 10000000, 1, 0.0085f);
    INS.AccelLPF = 0.0085f;
}


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q) {
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}


/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q) {
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}


/**
 * @brief reserved.It is used to correct IMU installation error and scale factor error, 
 *        i.e. the installation deviation of gyroscope axis and PTZ axis
 * @param param 
 * @param gyro  
 * @param accel 
 */
static void Param_Correction(INS_DataTypeDef *param, float gyro[3], float accel[3]) {
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag) {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
					//求得逆旋转矩阵
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];//加速度范围值处理直接读取到的速度数据
    //坐标系转换
    gyro[X_INS] = c_11 * gyro_temp[X_INS] +
              c_12 * gyro_temp[Y_INS] +
              c_13 * gyro_temp[Z_INS];
    gyro[Y_INS] = c_21 * gyro_temp[X_INS] +
              c_22 * gyro_temp[Y_INS] +
              c_23 * gyro_temp[Z_INS];
    gyro[Z_INS] = c_31 * gyro_temp[X_INS] +
              c_32 * gyro_temp[Y_INS] +
              c_33 * gyro_temp[Z_INS];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];
    //坐标系转换
    accel[X_INS] = c_11 * accel_temp[X_INS] +
               c_12 * accel_temp[Y_INS] +
               c_13 * accel_temp[Z_INS];
    accel[Y_INS] = c_21 * accel_temp[X_INS] +
               c_22 * accel_temp[Y_INS] +
               c_23 * accel_temp[Z_INS];
    accel[Z_INS] = c_31 * accel_temp[X_INS] +
               c_32 * accel_temp[Y_INS] +
               c_33 * accel_temp[Z_INS];
    // 记录上次数值用于低通滤波
    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}


/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt) {
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}


/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll) {
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}


/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q) {
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
