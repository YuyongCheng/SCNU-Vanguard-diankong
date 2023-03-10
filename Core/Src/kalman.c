#include "kalman.h"
#include "math.h"
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

	f->p_past = (1 - f->K) * f->p_now;
	return f->output;
}
Kalman_TypeDef Klm_Motor[6];
IMU_fliter_TypeDef imu_fliter[3];
void IMU_fliter(IMU_fliter_TypeDef *f,float newangle,float newgyro)
{
	const float dt = 0.01;
	f->angle = f->angle - (f->Q_bias-newgyro)*dt;

	f->P[0][0] = f->P[0][0] + f->Q_angle - (f->P[0][1]-f->P[1][0])*dt + (f->P[1][1]*dt*dt);
	f->P[0][1] = f->P[0][1] - (f->P[1][1]*dt);
	f->P[1][0] = f->P[1][0] - (f->P[1][1]*dt);
	f->P[1][1] = f->P[1][0] + f->Q_gyro;

	f->K[0] = f->P[0][0]/(f->P[0][0] + f->R);
	f->K[1] = f->P[1][0]/(f->P[0][0] + f->R);
	if(f->K[0] == 1)
		f->K[0] = 0.5;

	f->angle = f->angle + f->K[0]*(newangle - f->angle);
	f->Q_bias = f->Q_bias + f->K[1]*(newangle - f->angle);
	f->output = f->angle;

	f->P[0][0] = f->P[0][0] - (f->K[0]*f->P[0][0]);
	f->P[0][1] = f->P[0][1] - (f->K[0]*f->P[0][1]);
	f->P[1][0] = f->P[1][0] - (f->K[1]*f->P[1][0]);
	f->P[1][1] = f->P[1][1] - (f->K[1]*f->P[1][1]);
}

void IMU_fliter_Init(IMU_fliter_TypeDef *f)
{
	f->P[0][0] = 1;
	f->P[0][1] = 1;
	f->P[1][0] = 1;
	f->P[1][1] = 1;
	f->K[0] = 0;
	f->K[1] = 0;
	f->Q_angle = 0.001;
	f->Q_gyro = 0.003;
	f->R = 0.03;
}
