#include "kalman.h"
#include "MPU_9250.h"
#include "math.h"
#define gyro_z_time 100
//卡尔曼参数		
double gyro_z_init=0;    //读取误差的次数
double gyro_z_bias=0;
double gyro_y_bias=0;
double Q_angle = 0.001;		//角度数据置信度，角度噪声的协方差
double Q_gyro  = 0.003;		//角速度数据置信度，角速度噪声的协方差  
double R_angle = 0.01;		//加速度计测量噪声的协方差
 double dt =0.02     ;		//滤波算法计算周期，由定时器定时20ms//改了
double  C_0     = 1;			//H矩阵值
double Q_bias=14, Angle_err;	//Q_bias:陀螺仪的偏差  Angle_err:角度偏量 
double Q_biasy=10;
double Q_biasz=10;
double PCt_0, PCt_1, E;		//计算的过程量
double K_0, K_1, t_0, t_1;	//卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差 t_0/1:中间变量
double P[4] ={0,0,0,0};	//过程协方差矩阵的微分矩阵，中间变量
double PP[2][2] = { { 1, 0 },{ 0, 1 } };//过程协方差矩阵P
double yaw_start=0;
//卡尔曼解算法库

short aacx,aacy,aacz;		//加速度传感器原始数据 
short gyrox,gyroy,gyroz;	//陀螺仪原始数据 
float temperature;			//陀螺仪温度数据
double Accel_x;	     		//X轴加速度值暂存
double Accel_y;	            //Y轴加速度值暂存
double Accel_z;	            //Z轴加速度值    暂存
double Gyro_x;		        //X轴陀螺仪数据暂存
double Gyro_y;               //Y轴陀螺仪数据暂存
double Gyro_z;		        //Z轴陀螺仪数据暂存	
double Angle_x_temp;         //由加速度计算的x倾斜角度
double Angle_y_temp;  		//由加速度计算的y倾斜角度
double Angle_z_temp;
double Angle_X_Final=50; 		//X最终倾斜角度
double Angle_Y_Final=50; 		//Y最终倾斜角度
double Angle_X_Final0=0; 		//X最初倾斜角度
double Angle_Y_Final0=0; 		//Y最初倾斜角度
double Angle_Z_Final=50;
double Angle_Z_Final0=0;
double Roll_first;         //roll先验误差
double Pitch_first;        //pitch先验误差
double Gyro_roll;         //转换到地面坐标系的roll
double Gyro_pitch;        //转换到地面坐标系的pitch
double Gyro_yaw;          //转换到地面坐标系的yaw
double start_flag=0;
double start_flag1=0;
double test;
double YAW_angel=0;
double YAW_final_angle;
double yaw_last;
double Mag_x_body;
double Mag_y_body;
double Mag_z_body;
extern double yaw;
//角度计算
double pp0[2][2]={
     {10,10},
		 {10,10}
};//过程协方差矩阵初值
double A_max[2][2]={
     {1,-0.02},
		 {0,1}
};//A状态矩阵
double A_Tmax[2][2]={
     {1,0},
		 {-0.02,1}

};//A矩阵转置
double Q_max[2][2]={
      {0.001,0},
			{0,0.003}

};//角度协方差矩阵
double B_max[2][2]={
      {0.02 ,0},
			{0,0.02}


};//B矩阵
double error_first[2][2]={
     {0,0},
		 {0,0}


};//过程协方差矩阵
double P_final[2][2]={
      {10,10},
			{10,10}
};
double P_finaly[2][2]={
      {10,10},
			{10,10}
};
double P_finalz[2][2]={
      {10,10},
			{10,10}
};
double A_final[2][2]={
       {1,-0.02},
			 {0,1}
};
double A_finaly[2][2]={
       {1,-0.02},
			 {0,1}
};
double A_finalz[2][2]={
       {1,-0.02},
			 {0,1}
};
double jisuan_max[2][2];
double Kk[2][1];
double Kky[2][1];
double Kkz[2][1];
int16_t staa_flag=0;
double test;
void Angle_Calcu(void)	 
{
      
  float accx,accy,accz;//三方向角加速度值
	aacx  = Accel_x1; //x轴加速度值获取
	aacy  = Accel_y1; //y轴加速度值获取
	aacz  = Accel_z1; //z轴加速度值获取
	gyrox = Gyro_x1;  //x轴陀螺仪值获取
	gyroy = Gyro_y1;  //y轴陀螺仪值获取
	gyroz = Gyro_z1;  //z轴陀螺仪值获取
    //temperature=36.53+((double)TEMPData/340.0);//得到温度值
    Accel_x = aacx;//x轴加速度值暂存
    Accel_y = aacy;//y轴加速度值暂存
    Accel_z = aacz;//z轴加速度值暂存
    Gyro_x = gyrox;//x轴陀螺仪值暂存
    Gyro_y = gyroy;//y轴陀螺仪值暂存
    Gyro_z = gyroz;//z轴陀螺仪值暂存

	//角加速度原始值处理过程
	if(Accel_x!=0){
	staa_flag=1;
	}
	//加速度传感器配置寄存器0X1C内写入0x01,设置范围为±2g。换算关系：2^16/4 = 16384LSB/g
	if(Accel_x<32764) accx=Accel_x/16384.0;//计算x轴加速度
	else              accx=1-(Accel_x-49152)/16384.0f;
	if(Accel_y<32764) accy=Accel_y/16384.0;//计算y轴加速度
	else              accy=1-(Accel_y-49152)/16384.0f;
	if(Accel_z<32764) accz=Accel_z/16384.0f;//计算z轴加速度
	else              accz=(Accel_z-49152)/16384.0f;
	//加速度反正切公式计算三个轴和水平面坐标系之间的夹角
	//test = accz;

  if(staa_flag==1){
	 
	Angle_x_temp=(atan(accy/accz))*180/3.14;
	Angle_y_temp=(atan(accx/accz))*180/3.14;
	//判断计算后角度的正负号
	if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
	if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
	if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
	if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;
//   Angle_X_Final0=Angle_x_temp;
//	 Angle_Y_Final0=Angle_y_temp;
//	
	//角速度原始值处理过程
	//陀螺仪配置寄存器0X1B内写入0x18，设置范围为2000deg/s。换算关系：2^16/4000=16.4LSB/(°/S)
	////计算角速度
	if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4f);
	if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4f;
	if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4f);
	if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4f;
	if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4f);
	if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4f;
	if(gyro_z_init<=gyro_z_time){
	 gyro_z_bias+=Gyro_z;
	 gyro_z_init++;
	 gyro_y_bias+=Gyro_y;
	}	
	 	if(gyro_z_init>=gyro_z_time){
	yaw_start=1;
	}
	// 坐标系转换
	Gyro_roll=Gyro_x+tan(Angle_Y_Final)*sin(Angle_X_Final)*Gyro_y+tan(Angle_Y_Final)*cos(Angle_X_Final)*Gyro_z;
	Gyro_pitch=Gyro_y*cos(Angle_X_Final)-sin(Angle_X_Final)*Gyro_z;
	Gyro_yaw=(Gyro_y)*sin(Angle_X_Final)/cos(Angle_Y_Final)+(Gyro_z)*cos(Angle_X_Final)/cos(Angle_Y_Final);
	//test=gyro_z_bias/gyro_z_time;
	test=Mag_x*cos(Angle_Y_Final)*cos(Angle_Z_Final);
	Mag_x_body=Mag_x*cos(Angle_Y_Final)*cos(Angle_Z_Final)+Mag_y*(cos(Angle_Z_Final)*sin(Angle_X_Final)*sin(Angle_Y_Final)-sin(Angle_Z_Final)*cos(Angle_X_Final))+Mag_z*(cos(Angle_Z_Final)*sin(Angle_Y_Final)*cos(Angle_X_Final)+sin(Angle_Z_Final)*sin(Angle_X_Final));
	Mag_y_body=Mag_x*(sin(Angle_Z_Final)*cos(Angle_Y_Final))+Mag_y*(sin(Angle_X_Final)*sin(Angle_Y_Final)*sin(Angle_Z_Final)+cos(Angle_X_Final)*cos(Angle_Z_Final))+Mag_z*(sin(Angle_Z_Final)*sin(Angle_Y_Final)*cos(Angle_X_Final)-cos(Angle_Z_Final)*sin(Angle_X_Final));
	
	Angle_z_temp=atan(Mag_y_body/Mag_x_body)*180.0/3.14f;

    my_calman_filterroll(Angle_x_temp,Gyro_roll);
		my_calman_filterpitch(Angle_y_temp,Gyro_pitch);
		my_calman_filteryaw(yaw,Gyro_yaw);
//			Gyro_yaw=(Gyro_y-gyro_y_bias/gyro_z_time)*sin(Angle_X_Final)/cos(Angle_Y_Final)+(Gyro_z-gyro_z_bias/gyro_z_time)*cos(Angle_X_Final)/cos(Angle_Y_Final);
//  		YAW_angel+=(dt*(Gyro_yaw));
	
	}
} 




//void Kalman_Filter_X(float Accel,float Gyro) //卡尔曼函数		
//{
//	//步骤一，先验估计
//	//公式：X(k|k-1) = AX(k-1|k-1) + BU(k)
//	//X = (Angle,Q_bias)
//	//A(1,1) = 1,A(1,2) = -dt
//	//A(2,1) = 0,A(2,2) = 1
//	Angle_X_Final += (Gyro - Q_bias) * dt; //状态方程,角度值等于上次最优角度加角速度减零漂后积分
//	
//	//步骤二，计算过程协方差矩阵的微分矩阵
//	//公式：P(k|k-1)=AP(k-1|k-1)A^T + Q 
//	//Q(1,1) = cov(Angle,Angle)	Q(1,2) = cov(Q_bias,Angle)
//	//Q(2,1) = cov(Angle,Q_bias)	Q(2,2) = cov(Q_bias,Q_bias)
//	P[0]= Q_angle - PP[0][1] - PP[1][0];
//	P[1]= -PP[1][1];// 先验估计误差协方差
//	P[2]= -PP[1][1];
//	P[3]= Q_gyro;
//	PP[0][0] += P[0] * dt;   
//	PP[0][1] += P[1] * dt;   
//	PP[1][0] += P[2] * dt;
//	PP[1][1] += P[3] * dt;	
//	Angle_err = Accel - Angle_X_Final;	//Z(k)先验估计 计算角度偏差
//	
//	//步骤三，计算卡尔曼增益
//	//公式：Kg(k)= P(k|k-1)H^T/(HP(k|k-1)H^T+R)
//	//Kg = (K_0,K_1) 对应Angle,Q_bias增益
//	//H = (1,0)	可由z=HX+v求出z:Accel
//	PCt_0 = C_0 * PP[0][0];
//	PCt_1 = C_0 * PP[1][0];
//	E = R_angle + C_0 * PCt_0;
//	K_0 = PCt_0 / E;
//	K_1 = PCt_1 / E;
//	
//	//步骤四，后验估计误差协方差
//	//公式：P(k|k)=(I-Kg(k)H)P(k|k-1)
//	//也可写为：P(k|k)=P(k|k-1)-Kg(k)HP(k|k-1)
//	t_0 = PCt_0;
//	t_1 = C_0 * PP[0][1];
//	PP[0][0] -= K_0 * t_0;		
//	PP[0][1] -= K_0 * t_1;
//	PP[1][0] -= K_1 * t_0;
//	PP[1][1] -= K_1 * t_1;
//	
//	//步骤五，计算最优角速度值
//	//公式：X(k|k)= X(k|k-1)+Kg(k)(Z(k)-X(k|k-1))
//	Angle_X_Final += K_0 * Angle_err;	 //后验估计，给出最优估计值
//	Q_bias        += K_1 * Angle_err;	 //后验估计，跟新最优估计值偏差
//	Gyro_x         = Gyro - Q_bias;	 
//}

//void Kalman_Filter_Y(float Accel,float Gyro) 		
//{
//	Angle_Y_Final += (Gyro - Q_bias) * dt;
//	P[0]=Q_angle - PP[0][1] - PP[1][0]; 
//	P[1]=-PP[1][1];
//	P[2]=-PP[1][1];
//	P[3]=Q_gyro;	
//	PP[0][0] += P[0] * dt; 
//	PP[0][1] += P[1] * dt;  
//	PP[1][0] += P[2] * dt;
//	PP[1][1] += P[3] * dt;	
//	Angle_err = Accel - Angle_Y_Final;		
//	PCt_0 = C_0 * PP[0][0];
//	PCt_1 = C_0 * PP[1][0];	
//	E = R_angle + C_0 * PCt_0;	
//	K_0 = PCt_0 / E;
//	K_1 = PCt_1 / E;	
//	t_0 = PCt_0;
//	t_1 = C_0 * PP[0][1];
//	PP[0][0] -= K_0 * t_0;		
//	PP[0][1] -= K_0 * t_1;
//	PP[1][0] -= K_1 * t_0;
//	PP[1][1] -= K_1 * t_1;		
//	Angle_Y_Final	+= K_0 * Angle_err;
//	Q_bias	+= K_1 * Angle_err;	 
//	Gyro_y   = Gyro - Q_bias;	 
//}
void my_calman_filterroll(double Z_angle_roll,double gyro_roll  ){
  //1计算先验误差,q_bias是给定的值
	Angle_X_Final=Angle_X_Final+dt*(gyro_roll-Q_bias);
	//2计算过程协方差矩阵
	//计算过程矩阵 
	double apk_1[2][2];
	A_final[0][1]=0-dt;
	apk_1[0][0]=A_final[0][0]*P_final[0][0]+A_final[0][1]*P_final[1][0];
	apk_1[0][1]=A_final[0][0]*P_final[0][1]+A_final[0][1]*P_final[1][1];
	apk_1[1][0]=A_final[1][0]*P_final[0][0]+A_final[1][1]*P_final[1][0];
	apk_1[1][1]=A_final[1][0]*P_final[0][1]+A_final[1][1]*P_final[1][1];
	double A_finalT[2][2];
	A_finalT[0][0]=A_final[0][0];
	A_finalT[0][1]=A_final[1][0];
	A_finalT[1][0]=A_final[0][1];
	A_finalT[1][1]=A_final[1][1];
	double apk_2[2][2];
	apk_2[0][0]=apk_1[0][0]*A_finalT[0][0]+apk_1[0][1]*A_finalT[1][0];
	apk_2[0][1]=apk_1[0][0]*A_finalT[0][1]+apk_1[0][1]*A_finalT[1][1];
	apk_2[1][0]=apk_1[1][0]*A_finalT[0][0]+apk_1[1][1]*A_finalT[1][0];
	apk_2[1][1]=apk_1[1][0]*A_finalT[0][1]+apk_1[1][1]*A_finalT[1][1];
	P_final[0][0]=apk_2[0][0]+0.002;
	P_final[0][1]=apk_2[0][1];
	P_final[1][0]=apk_2[1][0];
	P_final[1][1]=apk_2[1][1]+0.003;
	//3Kk校正计算
	double cal_hp;
	cal_hp=P_final[0][0]+R_angle;
	Kk[0][0]=P_final[0][0]/cal_hp;
	Kk[1][0]=P_final[1][0]/cal_hp;
	//4Final_angle  计算
	Angle_X_Final=Angle_X_Final+Kk[0][0]*(Z_angle_roll-Angle_X_Final);
  Q_bias+=Kk[1][0]*(Z_angle_roll-Angle_X_Final);	
	//5Pk矫正
	double guochen3[2][2];
	guochen3[0][0]=1-Kk[0][0];
	guochen3[0][1]=0;	
	guochen3[1][0]=0;
	guochen3[1][1]=0;
	double guoc[2][2];
	guoc[0][0]=guochen3[0][0]*P_final[0][0]+guochen3[0][1]*P_final[1][0];
	guoc[0][1]=guochen3[0][0]*P_final[0][1]+guochen3[0][1]*P_final[1][1];
	guoc[1][0]=guochen3[1][0]*P_final[0][0]+guochen3[1][1]*P_final[1][0];
	guoc[1][1]=guochen3[1][0]*P_final[0][1]+guochen3[1][1]*P_final[1][1];
	P_final[0][0]=guoc[0][0];
	P_final[0][1]=guoc[0][1];
	P_final[1][0]=guoc[1][0];
	P_final[1][1]=guoc[1][1];
	
	
  

}
void my_calman_filterpitch(double Z_angle_pitch,double gyro_pitch){
  //1计算先验误差,q_bias是给定的值
	Angle_Y_Final=Angle_Y_Final+dt*(gyro_pitch-Q_bias);
	//2计算过程协方差矩阵
	//计算过程矩阵 
	double apk_1y[2][2];
	A_finaly[0][1]=0-dt;
	apk_1y[0][0]=A_finaly[0][0]*P_finaly[0][0]+A_finaly[0][1]*P_finaly[1][0];
	apk_1y[0][1]=A_finaly[0][0]*P_finaly[0][1]+A_finaly[0][1]*P_finaly[1][1];
	apk_1y[1][0]=A_finaly[1][0]*P_finaly[0][0]+A_finaly[1][1]*P_finaly[1][0];
	apk_1y[1][1]=A_finaly[1][0]*P_finaly[0][1]+A_finaly[1][1]*P_finaly[1][1];
	double A_finalTy[2][2];
	A_finalTy[0][0]=A_finaly[0][0];
	A_finalTy[0][1]=A_finaly[1][0];
	A_finalTy[1][0]=A_finaly[0][1];
	A_finalTy[1][1]=A_finaly[1][1];
	double apk_2y[2][2];
	apk_2y[0][0]=apk_1y[0][0]*A_finalTy[0][0]+apk_1y[0][1]*A_finalTy[1][0];
	apk_2y[0][1]=apk_1y[0][0]*A_finalTy[0][1]+apk_1y[0][1]*A_finalTy[1][1];
	apk_2y[1][0]=apk_1y[1][0]*A_finalTy[0][0]+apk_1y[1][1]*A_finalTy[1][0];
	apk_2y[1][1]=apk_1y[1][0]*A_finalTy[0][1]+apk_1y[1][1]*A_finalTy[1][1];
	P_finaly[0][0]=apk_2y[0][0]+0.002;
	P_finaly[0][1]=apk_2y[0][1];
	P_finaly[1][0]=apk_2y[1][0];
	P_finaly[1][1]=apk_2y[1][1]+0.003;
	//3Kk校正计算
	double cal_hpy;
	cal_hpy=P_finaly[0][0]+R_angle;
	Kky[0][0]=P_finaly[0][0]/cal_hpy;
	Kky[1][0]=P_finaly[1][0]/cal_hpy;
	//4Final_angle  计算
	Angle_Y_Final=Angle_Y_Final+Kky[0][0]*(Z_angle_pitch-Angle_Y_Final);
  Q_biasy+=Kky[1][0]*(Z_angle_pitch-Angle_Y_Final);	
	//5Pk矫正
	double guochen3y[2][2];
	guochen3y[0][0]=1-Kky[0][0];
	guochen3y[0][1]=0;	
	guochen3y[1][0]=0;
	guochen3y[1][1]=0;
	double guocy[2][2];
	guocy[0][0]=guochen3y[0][0]*P_finaly[0][0]+guochen3y[0][1]*P_finaly[1][0];
	guocy[0][1]=guochen3y[0][0]*P_finaly[0][1]+guochen3y[0][1]*P_finaly[1][1];
	guocy[1][0]=guochen3y[1][0]*P_finaly[0][0]+guochen3y[1][1]*P_finaly[1][0];
	guocy[1][1]=guochen3y[1][0]*P_finaly[0][1]+guochen3y[1][1]*P_finaly[1][1];
	P_finaly[0][0]=guocy[0][0];
	P_finaly[0][1]=guocy[0][1];
	P_finaly[1][0]=guocy[1][0];
	P_finaly[1][1]=guocy[1][1];
	

}
void my_calman_filteryaw(double angle_yaw,double gyro_yaw){
  Angle_Z_Final=Angle_Z_Final+dt*(gyro_yaw-Q_bias);
	//2计算过程协方差矩阵
	//计算过程矩阵 
	double apk_1z[2][2];
	A_finalz[0][1]=0-dt;
	apk_1z[0][0]=A_finalz[0][0]*P_finalz[0][0]+A_finalz[0][1]*P_finalz[1][0];
	apk_1z[0][1]=A_finalz[0][0]*P_finalz[0][1]+A_finalz[0][1]*P_finalz[1][1];
	apk_1z[1][0]=A_finalz[1][0]*P_finalz[0][0]+A_finalz[1][1]*P_finalz[1][0];
	apk_1z[1][1]=A_finalz[1][0]*P_finalz[0][1]+A_finalz[1][1]*P_finalz[1][1];
	double A_finalTz[2][2];
	A_finalTz[0][0]=A_finalz[0][0];
	A_finalTz[0][1]=A_finalz[1][0];
	A_finalTz[1][0]=A_finalz[0][1];
	A_finalTz[1][1]=A_finalz[1][1];
	double apk_2z[2][2];
	apk_2z[0][0]=apk_1z[0][0]*A_finalTz[0][0]+apk_1z[0][1]*A_finalTz[1][0];
	apk_2z[0][1]=apk_1z[0][0]*A_finalTz[0][1]+apk_1z[0][1]*A_finalTz[1][1];
	apk_2z[1][0]=apk_1z[1][0]*A_finalTz[0][0]+apk_1z[1][1]*A_finalTz[1][0];
	apk_2z[1][1]=apk_1z[1][0]*A_finalTz[0][1]+apk_1z[1][1]*A_finalTz[1][1];
	P_finalz[0][0]=apk_2z[0][0]+0.002;
	P_finalz[0][1]=apk_2z[0][1];
	P_finalz[1][0]=apk_2z[1][0];
	P_finalz[1][1]=apk_2z[1][1]+0.003;
	//3Kk校正计算
	double cal_hp;
	cal_hp=P_finalz[0][0]+R_angle;
	Kkz[0][0]=P_finalz[0][0]/cal_hp;
	Kkz[1][0]=P_finalz[1][0]/cal_hp;
	//4Final_angle  计算
	Angle_Z_Final=Angle_Z_Final+Kkz[0][0]*(angle_yaw-Angle_Z_Final);
  Q_bias+=Kk[1][0]*(angle_yaw-Angle_X_Final);	
	//5Pk矫正
	double guochen3z[2][2];
	guochen3z[0][0]=1-Kkz[0][0];
	guochen3z[0][1]=0;	
	guochen3z[1][0]=0;
	guochen3z[1][1]=0;
	double guocz[2][2];
	guocz[0][0]=guochen3z[0][0]*P_finalz[0][0]+guochen3z[0][1]*P_finalz[1][0];
	guocz[0][1]=guochen3z[0][0]*P_final[0][1]+guochen3z[0][1]*P_finalz[1][1];
	guocz[1][0]=guochen3z[1][0]*P_final[0][0]+guochen3z[1][1]*P_finalz[1][0];
	guocz[1][1]=guochen3z[1][0]*P_final[0][1]+guochen3z[1][1]*P_finalz[1][1];
	P_finalz[0][0]=guocz[0][0];
	P_finalz[0][1]=guocz[0][1];
	P_finalz[1][0]=guocz[1][0];
	P_finalz[1][1]=guocz[1][1];
}