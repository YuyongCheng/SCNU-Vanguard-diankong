#include "kalman.h"
void KalmanFilter_Init(Kalman_TypeDef *klm_typedef)
{
	klm_typedef->K = 0;
	klm_typedef->Q = 0.01;
	klm_typedef->R = 0.3;
	klm_typedef->p_now = 0;
	klm_typedef->p_past = 0;
	klm_typedef->output_Max = 40;
}
float KalmanFilter(Kalman_TypeDef *f, float input)
{
	//简化一阶卡尔曼滤波，默认A=1，H=1，B=0
	if(fabs(input - (float)(f->output)) >= 8)
	{
		f->output = input;
	}
	f->p_now = f->p_past + f->Q;
	f->K = f->p_now / (f->p_now + f->R);
	f->output = f->output + f->K * (input - f->output);
//	if(f->output > f->output_Max)
//	{
//		f->output = f->output_Max;
//	}

	f->p_past = (1 - f->K) * f->p_now;
	return f->output;
}
Kalman_TypeDef Klm_Motor[6];
