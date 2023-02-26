#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_
#include "main.h"
typedef struct
{
	float R;         //测量噪声
	float Q;         //过程噪声
	float p_past;    //上一时刻误差协方差
	float p_now;     //上一时刻过程对当前时刻估计值的协方差
	float K;         //卡尔曼增益
	float output;
	float output_Max;
}Kalman_TypeDef;
extern Kalman_TypeDef Klm_Motor[6];
extern void KalmanFilter_Init(Kalman_TypeDef *klm_typedef);
extern float KalmanFilter(Kalman_TypeDef *f, float input);
#endif /* INC_KALMAN_H_ */
