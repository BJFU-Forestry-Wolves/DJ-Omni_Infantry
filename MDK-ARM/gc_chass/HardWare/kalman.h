#ifndef __KALMAN_H
#define __KALMAN_H 

//卡尔曼解算法库

extern double Angle_X_Final;			//解算后俯仰角
extern double Angle_Y_Final;			//解算后横滚角
extern float temperature;			//陀螺仪温度数据
extern short aacx,aacy,aacz;		//加速度传感器原始数据  angular acceleration
extern short gyrox,gyroy,gyroz;		//陀螺仪原始数据  gyroscope


void Angle_Calcu(void);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);
void my_calman_filterroll(double Z_angle_roll,double gyro_roll  );
void my_calman_filterpitch(double Z_angle_pitch,double gyro_pitch);
void my_calman_filteryaw(double angle_yaw,double gyro_yaw);
#endif
