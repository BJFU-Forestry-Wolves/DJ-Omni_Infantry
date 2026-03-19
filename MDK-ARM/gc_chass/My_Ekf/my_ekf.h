#ifndef __MY_EKF_H
#define __MY_EKF_H
#include "main.h"
#include "math.h"
#define MAX 4
#define MAX1 10
void my_ekf(void);
void my_cal_Ak(void);
void multiplyMatrices(float first[4][4], float second[4][1],float result[4][1], float rowFirst,float columnFirst);
void transposeMatrix(float matrix[4][4], float transposed[4][4], int rows, int cols) ;
void multiplyMatrices2(float first[MAX][MAX], float second[MAX][MAX], float result[MAX][MAX], int rowFirst, int columnFirst, int rowSecond, int columnSecond);
void my_qk_cal(void);
void my_cal_pk(void );
void my_Kk_cal(void);
void transposeMatrix2(float matrix[3][4], float transposed[4][3], int rows, int cols); 
void multiplyMatrices_Hk(float first[3][4], float second[4][4], float result[3][4], int rowFirst, int columnFirst, int rowSecond, int columnSecond);
void multiplyMatrices_HkT(float first[3][4], float second[4][3], float result[3][3], int rowFirst, int columnFirst, int rowSecond, int columnSecond);
#endif