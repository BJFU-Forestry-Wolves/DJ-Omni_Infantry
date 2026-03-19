#include "my_ekf.h"
#include "mpu_9250.h"

extern float dt;
float my_q[4][1]={1,0,0,0};
float my_q_cal[4][1];
float my_A_max[4][4];
float my_A_tra[4][4];
float Pk[4][4];
float Hk[3][4];
float Hk_t[4][3];
float q_bias[4][1]={0.001,0.001,0.001,0.001};
float qk1,qk2,qk3,qk4;
float Vk[3][1]={0.001,0.001,0.001};
float Vk_T[1][3]={0.001,0.001,0.001};
float Rk=0.001;
float a11,a12,a13,a14,a21,a22,a23,a24,a31,a32,a33,a34,a41,a42,a43,a44;
void my_ekf(void){
	  my_cal_Ak();
	  my_qk_cal();
	  my_cal_pk();
}
void my_cal_Ak(void){
    my_A_max[0][0]=1;
	  my_A_max[0][1]=1-0.5f*dt*Gyro_x1;
	  my_A_max[0][2]=1-0.5f*dt*Gyro_y1;
	  my_A_max[0][3]=1-0.5f*dt*Gyro_z1;
	  my_A_max[1][0]=1+0.5f*dt*Gyro_x1;
	  my_A_max[1][1]=1;
	  my_A_max[1][2]=1+0.5f*dt*Gyro_z1;
	  my_A_max[1][3]=1-0.5f*dt*Gyro_y1;
	  my_A_max[2][0]=1+0.5f*dt*Gyro_y1;
	  my_A_max[2][1]=1-0.5f*dt*Gyro_z1;
	  my_A_max[2][2]=1;
	  my_A_max[2][3]=1+0.5f*dt*Gyro_x1;
	  my_A_max[3][0]=1+0.5f*dt*Gyro_z1;
	  my_A_max[3][1]=1+0.5f*dt*Gyro_y1;
	  my_A_max[3][2]=1-0.5f*dt*Gyro_x1;
	  my_A_max[3][3]=1;
}
void my_qk_cal(){
     multiplyMatrices(my_A_max,my_q,my_q_cal,4,4);
	   my_q[0][0]=my_q_cal[0][0];
	   my_q[1][0]=my_q_cal[1][0];
	   my_q[2][0]=my_q_cal[2][0];
	   my_q[3][0]=my_q_cal[3][0];

}
void my_cal_pk(void ){
	   //calculate Pk-
     transposeMatrix(my_A_max,my_A_tra,4,4);
	   float cal_max[4][4];
	   multiplyMatrices2(my_A_max,Pk,cal_max,4,4,4,4);
	   float cal_max2[4][4];
	   multiplyMatrices2(cal_max,my_A_tra,cal_max2,4,4,4,4);
	   for(uint16_t i=0;i<4;i++){
		  for(uint16_t j=0;j<4;j++){
			Pk[i][j]=cal_max2[i][j];
			}
		 }
      Pk[0][0]+=qk1;
		  Pk[1][1]+=qk2;
		  Pk[2][2]+=qk3;
		  Pk[3][3]+=qk4;
}
void my_Kk_cal(void){
     Hk[0][0]=-2*my_q[2][0];
	   Hk[0][1]=2*my_q[3][0];
	   Hk[0][2]=-2*my_q[0][0];
	   Hk[0][3]=2*my_q[1][0];
     Hk[1][0]=2*my_q[1][0];
	   Hk[1][1]=2*my_q[0][0];
	   Hk[1][2]=2*my_q[3][0];
	   Hk[1][3]=2*my_q[2][0];
     Hk[2][0]=2*my_q[0][0];
	   Hk[2][1]=-2*my_q[1][0];
	   Hk[2][2]=-2*my_q[2][0];
	   Hk[2][3]=2*my_q[3][0];
	   transposeMatrix2(Hk,Hk_t,3,4);
	   float temp_max[3][4];
	   multiplyMatrices_Hk(Hk,Pk,temp_max,3,4,4,3);
	   float temp_max2[3][3];
	   multiplyMatrices_HkT(temp_max,Hk_t,temp_max2,3,4,4,3);
	   
	   //multiplyMatrices_Hk(temp_max,Hk_t,temp_max,3,4,4,3);
	   
	   
	   
}
//void multiplyMatrices(float first[4][4], float second[4], float result[4], int rowFirst, int columnFirst, int rowSecond, int columnSecond) {
//    // 初始化结果矩阵
//    for (int i = 0; i < rowFirst; i++) {
//        for (int j = 0; j < columnSecond; j++) {
//            result[i][j] = 0;
//        }
//    }

//    // 矩阵相乘
//    for (int i = 0; i < rowFirst; i++) {
//        for (int j = 0; j < columnSecond; j++) {
//            for (int k = 0; k < columnFirst; k++) {
//                result[i][j] += first[i][k] * second[k][j];
//            }
//        }
//    }
//}
void multiplyMatrices(float first[4][4], float second[4][1],float result[4][1], float rowFirst,float columnFirst) {
    // 初始化结果矩阵
    for (int i = 0; i < rowFirst; i++) {
        result[i][0] = 0;
    }

    // 矩阵相乘
    for (int i = 0; i < rowFirst; i++) {
        for (int k = 0; k < columnFirst; k++) {
            result[i][0] += first[i][k] * second[k][0];
        }
    }
}
void transposeMatrix(float matrix[4][4], float transposed[4][4], int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            transposed[j][i] = matrix[i][j];
        }
    }


}
void multiplyMatrices2(float first[MAX][MAX], float second[MAX][MAX], float result[MAX][MAX], int rowFirst, int columnFirst, int rowSecond, int columnSecond) {
    // 初始化结果矩阵
    for (int i = 0; i < rowFirst; i++) {
        for (int j = 0; j < columnSecond; j++) {
            result[i][j] = 0;
        }
    }

    // 矩阵相乘
    for (int i = 0; i < rowFirst; i++) {
        for (int j = 0; j < columnSecond; j++) {
            for (int k = 0; k < columnFirst; k++) {
                result[i][j] += first[i][k] * second[k][j];
            }
        }
    }
}
		//use for Hk
void transposeMatrix2(float matrix[3][4], float transposed[4][3], int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            transposed[j][i] = matrix[i][j];
        }
    }
	}

void multiplyMatrices_Hk(float first[3][4], float second[4][4], float result[3][4], int rowFirst, int columnFirst, int rowSecond, int columnSecond) {
    // 初始化结果矩阵为0
    for (int i = 0; i < rowFirst; i++) {
        for (int j = 0; j < columnSecond; j++) {
            result[i][j] = 0;
        }
    }

    // 矩阵乘法
    for (int i = 0; i < rowFirst; i++) {
        for (int j = 0; j < columnSecond; j++) {
            for (int k = 0; k < columnFirst; k++) {
                result[i][j] += first[i][k] * second[k][j];
            }
        }
    }
}
void multiplyMatrices_HkT(float first[3][4], float second[4][3], float result[3][3], int rowFirst, int columnFirst, int rowSecond, int columnSecond) {
    // 初始化结果矩阵为0 
    for (int i = 0; i < rowFirst; i++) {
        for (int j = 0; j < columnSecond; j++) {
            result[i][j] = 0;
        }
    }

    // 矩阵乘法
    for (int i = 0; i < rowFirst; i++) {
        for (int j = 0; j < columnSecond; j++) {
            for (int k = 0; k < columnFirst; k++) {
                result[i][j] += first[i][k] * second[k][j];
            }
        }
    }
}
void multiplyMatrices_Kk(float first[1][3], float second[3][3], float result[1][3], int rowFirst, int columnFirst, int rowSecond, int columnSecond) {
    // 初始化结果矩阵为0 
    for (int i = 0; i < rowFirst; i++) {
        for (int j = 0; j < columnSecond; j++) {
            result[i][j] = 0;
        }
    }

    // 矩阵乘法
    for (int i = 0; i < rowFirst; i++) {
        for (int j = 0; j < columnSecond; j++) {
            for (int k = 0; k < columnFirst; k++) {
                result[i][j] += first[i][k] * second[k][j];
            }
        }
    }
}