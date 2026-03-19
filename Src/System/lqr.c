#include "lqr.h"
#include "app_ins.h"
#define lqr_b 2
#define lqr_j 5
lqr gimbal_lqr;
float A_value[4]={0,1,0,0-lqr_b/lqr_j};
float X_value[2]={0,0};//初始状态：角度和角角速度全为0
float K_value[2]={100.0f,31.2265f};
mat Mat_A;
mat Mat_B;
mat Mat_X;
mat Mat_K;
void mat_init(void){
   //mat_init(&Mat_A,)
	 Matrix_Init(&Mat_A,2,2,A_value);
	 Matrix_Init(&Mat_X,2,1,X_value);
	 Matrix_Init(&Mat_K,2,1,K_value);
}
void lqr_set_desire(float set_angle,lqr * follow_path){
   follow_path->desire=set_angle;
   X_value[0]=INS.Pitch;
	 X_value[1]=INS.Gyro[0];//pitch gyro
	 Matrix_Multiply(&Mat_K,&Mat_X,(arm_matrix_instance_f32*)&follow_path->output);

}