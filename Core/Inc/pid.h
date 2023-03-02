#ifndef __PID_H
#define __PID_H
#include "main.h"
typedef enum
{
	Speed = 0,
	Angle = 1
}PID_Type;

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float Err_former;
	float Err_last;
	float Err_now;
	float Err_sum;
	float Err_diff;
	float Output;
	float Output_Max;
	float Err_sum_Max;
	int PID_Type;
}PID_TypeDef;
typedef struct
{
	uint16_t angle;
	int16_t speed;
	int16_t current;
	uint8_t temp;          //温度
	int can_id;
	float target_speed;
	float target_angle;

}Motor_TypeDef;
extern PID_TypeDef PID_Motor_Speed[8];
extern PID_TypeDef PID_Motor_Angle[8];
extern Motor_TypeDef Motor[8];
extern void PID_Init(void);
extern void PID_Origin(PID_TypeDef *hpid, float val_now, float target_now);
extern void PID_Incr(PID_TypeDef *hpid, float val_now, float target_now);
extern float PID_SpeedCtrl_Config[8][5];
extern float PID_AngleCtrl_Config[8][5];
extern void PID_Clear(PID_TypeDef *hpid);
#endif
