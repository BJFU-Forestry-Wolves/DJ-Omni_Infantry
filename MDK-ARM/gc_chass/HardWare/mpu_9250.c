#include "mpu_9250.h"
#include "math.h"
#define MPU_9250_handle &hi2c1
uint8_t BUF[6];
double Accel_x1;
double Accel_y1;
double Accel_z1;
double Gyro_x1;
double Gyro_y1;
double Gyro_z1;
short Mag_x;
short Mag_y;
short Mag_z;
double yaw;
void MPU9250_WriteRegData(uint8_t reg_addr, uint8_t data)
{
    HAL_I2C_Mem_Write(MPU_9250_handle, GYRO_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 5);
}
void MPU9250_ReadRegData(uint8_t reg_addr, uint8_t *data, uint8_t num)
{
    HAL_I2C_Mem_Read(MPU_9250_handle, GYRO_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, data, num, 5);
}
void MPU9250_ReadMagRegData(uint8_t reg_addr, uint8_t *data, uint8_t num)
{
    HAL_I2C_Mem_Read(MPU_9250_handle, MAG_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, data, num, 5);
}
void Init_MPU9250(void)
{
//  Single_Write(GYRO_ADDRESS,PWR_MGMT_1, 0x00);	//解除休眠状态
//	Single_Write(GYRO_ADDRESS,SMPLRT_DIV, 0x07);
//	Single_Write(GYRO_ADDRESS,CONFIG, 0x06);
//	Single_Write(GYRO_ADDRESS,GYRO_CONFIG, 0x18);
//	Single_Write(GYRO_ADDRESS,ACCEL_CONFIG, 0x01);
	  MPU9250_WriteRegData(PWR_MGMT_1, 0x00);
	  MPU9250_WriteRegData(SMPLRT_DIV, 0x07);
	  MPU9250_WriteRegData(CONFIG, 0x06);
	  MPU9250_WriteRegData(GYRO_CONFIG, 0x18);
	  MPU9250_WriteRegData(ACCEL_CONFIG, 0x01);
}
void READ_MPU9250_ACCEL(void)
{ 

   MPU9250_ReadRegData(ACCEL_XOUT_L,&BUF[0],1);
   //BUF[1]=Single_Read(ACCEL_ADDRESS,ACCEL_XOUT_H);
	 MPU9250_ReadRegData(ACCEL_XOUT_H,&BUF[1],1);
   Accel_x1=	(BUF[1]<<8)|BUF[0];
   //Accel_x1/=164; 						   //读取计算X轴数据
   MPU9250_ReadRegData(ACCEL_YOUT_L,&BUF[2],1);
	 MPU9250_ReadRegData(ACCEL_YOUT_H,&BUF[3],1);
   //BUF[3]=Single_Read(ACCEL_ADDRESS,ACCEL_YOUT_H);
   Accel_y1=	(BUF[3]<<8)|BUF[2];
   //Accel_y1/=164; 						   //读取计算Y轴数据
	 MPU9250_ReadRegData(ACCEL_ZOUT_L,&BUF[4],1);
	 MPU9250_ReadRegData(ACCEL_ZOUT_H,&BUF[5],1);
   Accel_z1=	(BUF[5]<<8)|BUF[4];
   //Accel_z1/=164; 					       //读取计算Z轴数据
}
void READ_MPU9250_GYRO(void){
   MPU9250_ReadRegData(GYRO_XOUT_L,&BUF[0],1);
   //BUF[1]=Single_Read(ACCEL_ADDRESS,ACCEL_XOUT_H);
	 MPU9250_ReadRegData(GYRO_XOUT_H,&BUF[1],1);
   Gyro_x1=	((BUF[1]<<8)|BUF[0]);
   //Gyro_x1/=16.4; 						   //读取计算X轴数据
   MPU9250_ReadRegData(GYRO_YOUT_L,&BUF[2],1);
	 MPU9250_ReadRegData(GYRO_YOUT_H,&BUF[3],1);
   //BUF[3]=Single_Read(ACCEL_ADDRESS,ACCEL_YOUT_H);
   Gyro_y1=	((BUF[3]<<8)|BUF[2]);
   //Gyro_y1/=16.4; 						   //读取计算Y轴数据
	 MPU9250_ReadRegData(GYRO_ZOUT_L,&BUF[4],1);
	 MPU9250_ReadRegData(GYRO_ZOUT_H,&BUF[5],1);
   Gyro_z1=	((BUF[5]<<8)|BUF[4]);
   //Gyro_z1/=16.4; 					       //读取计算Z轴数据

}
void READ_MPU9250_MAG(void)
{ 
   //Single_Write(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
	 MPU9250_WriteRegData(0x37,0x02);
   //Delayms(10);
   HAL_Delay(10);	
   //Single_Write(MAG_ADDRESS,0x0A,0x01);
	 uint8_t send_text=0x01;
	 HAL_I2C_Mem_Write(MPU_9250_handle,MAG_ADDRESS,0x0A,I2C_MEMADD_SIZE_8BIT,&send_text,1,HAL_MAX_DELAY);
   //Delayms(10);	
	 HAL_Delay(10);
   //BUF[0]=Single_Read (MAG_ADDRESS,MAG_XOUT_L);
	 MPU9250_ReadMagRegData(MAG_XOUT_L,&BUF[0],1);
   MPU9250_ReadMagRegData(MAG_XOUT_H,&BUF[1],1);
   Mag_x=(BUF[1]<<8)|BUF[0];

   MPU9250_ReadMagRegData(MAG_YOUT_L,&BUF[2],1);
   MPU9250_ReadMagRegData(MAG_YOUT_H,&BUF[3],1);
   Mag_y=	(BUF[3]<<8)|BUF[2];
   						   //读取计算Y轴数据
   MPU9250_ReadMagRegData(MAG_ZOUT_L,&BUF[4],1);
   MPU9250_ReadMagRegData(MAG_ZOUT_H,&BUF[5],1);
   Mag_z=	(BUF[5]<<8)|BUF[4];
	 yaw=(atan((double)Mag_y/Mag_x))*180/3.14f;
 					       //读取计算Z轴数据
}