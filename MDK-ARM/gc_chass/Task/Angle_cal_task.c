#include "Angle_cal_task.h"
#include "stdio.h"
#include "usart.h"
#include "string.h"
#include "cmsis_os.h"
//extern UART_HandleTypeDef huart1;

uint32_t tim_baselast;
uint32_t tim_base0;
uint32_t tim_inter;
uint8_t buffer[1];
//static uint8_t tempData[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x80,0x7F};
void Angle_cal_task(void const * argument){
  while(1){
		READ_MPU9250_ACCEL();
		osDelay(1);
		READ_MPU9250_GYRO();
		osDelay(1);
		READ_MPU9250_MAG();
		osDelay(1);
//		INS_task();
		tim_baselast=tim_base0;
		tim_base0=HAL_GetTick();
		tim_inter=tim_base0-tim_baselast;
		//dt=tim_inter/1000.0f;
		Angle_Calcu();
		//buffer[0]=0x01;
		//HAL_UART_Transmit(&huart1,buffer,1,5);
		printf("%f,%f,%f\n",Angle_Z_Final,Angle_X_Final,Angle_Y_Final);
		osDelay(5);
		
	}

}
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (const uint8_t *)&ch, 1, 2);
  return ch;
}
// void usart_printf(const char *fmt,...) 
//{ 
//	static uint8_t tx_buf[256] = {0}; 
//	static va_list ap; 
//	static uint16_t len; 
//	va_start(ap, fmt); 
//	
//	//return length of string 
//	//�����ַ������� 
//	len = vsprintf((char *)tx_buf, fmt, ap); 
//	va_end(ap); 
//	HAL_UART_Transmit(&huart2,tx_buf,len,HAL_MAX_DELAY);
//}
