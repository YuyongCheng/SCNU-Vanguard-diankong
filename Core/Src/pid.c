#include "pid.h"
#include "math.h"
PID_TypeDef PID_Motor_Speed[8];
PID_TypeDef PID_Motor_Angle[8];
Motor_TypeDef Motor[8];
float PID_SpeedCtrl_Config[8][5] = {{ 45 , 16 , 0 , 5000, 500},
                                    { 0.5 , 0.0399999991 , 0 , 5000, 0},
                                    { 0.5 , 0.0399999991 , 0 , 5000, 0},
                                    { 0.5 , 0.0399999991 , 0 , 5000, 0},
	                            	{ 0.5 , 0.0399999991 , 0 , 5000, 0},
		                            { 2.5 , 30 , 0 , 500, 500},
									{ 2.5 , 10 , 0 , 500, 500},
									{ 2 , 0.07 , 0 , 500, 500}};//PID速度环参数
float PID_AngleCtrl_Config[8][5] = {{ 0 , 0 , 0 , 0, 0},
		                            { 70 , 36 , 0 , 5000, 500},
									{ 1 , 1 , 1 , 300, 500},
									{ 1 , 1 , 1 , 300, 500},
									{ 1 , 1 , 1 , 300, 500},
									{ 0.1 , 0 , 2 , 150, 10000},
									{ 0.1 , 0 , 2 , 150, 10000},
									{ 1 , 1 , 1 , 300, 500}};// PID角度环参数
void PID_Init()
{
	for(int i=0;i<8;i++)
	{
		PID_Motor_Speed[i].Kp = PID_SpeedCtrl_Config[i][0];
		PID_Motor_Speed[i].Ki = PID_SpeedCtrl_Config[i][1];
		PID_Motor_Speed[i].Kd = PID_SpeedCtrl_Config[i][2];
		PID_Motor_Speed[i].Output_Max = PID_SpeedCtrl_Config[i][3];
		PID_Motor_Speed[i].Err_sum_Max = PID_SpeedCtrl_Config[i][4];
		PID_Motor_Speed[i].PID_Type = Speed;
	}
	for(int i=0;i<8;i++)
	{
		PID_Motor_Angle[i].Kp = PID_AngleCtrl_Config[i][0];
		PID_Motor_Angle[i].Ki = PID_AngleCtrl_Config[i][1];
		PID_Motor_Angle[i].Kd = PID_AngleCtrl_Config[i][2];
		PID_Motor_Angle[i].Output_Max = PID_AngleCtrl_Config[i][3];
		PID_Motor_Angle[i].Err_sum_Max = PID_AngleCtrl_Config[i][4];
		PID_Motor_Angle[i].PID_Type = Angle;
	}
}
void PID_Clear(PID_TypeDef *hpid)
{
	hpid->Err_former = 0;
	hpid->Err_last = 0;
	hpid->Err_now = 0;
	hpid->Output = 0;
}
void PID_Origin(PID_TypeDef *hpid, float val_now, float target_now)
{
	if(target_now - val_now > 4096) val_now += 8192;
	if(val_now - target_now > 4096) val_now -= 8192;
	switch(hpid->PID_Type)
	{
	case Speed:
		hpid->Err_now = 2*target_now - val_now;
		break;
	case Angle:
		hpid->Err_now = target_now - val_now;
	}
	hpid->Err_sum += hpid->Err_now;
    if(hpid->Err_sum < -hpid->Err_sum_Max) hpid->Err_sum = -hpid->Err_sum_Max;
	if(hpid->Err_sum > hpid->Err_sum_Max) hpid->Err_sum = hpid->Err_sum_Max;

	hpid->Output = hpid->Kp*hpid->Err_now + hpid->Ki*hpid->Err_sum
			+ (hpid->Kd)*(hpid->Err_now - hpid->Err_last);
    if(hpid->Output > hpid->Output_Max)
		hpid->Output = hpid->Output_Max;
	if(hpid->Output < -hpid->Output_Max )
		hpid->Output = -hpid->Output_Max;

	hpid->Err_former = hpid->Err_last;
	hpid->Err_last = hpid->Err_now;
}

void PID_Incr(PID_TypeDef *hpid, float val_now, float target_now)
{
	hpid->Err_now = target_now - val_now;
    hpid->Output = hpid->Kp*(hpid->Err_now - hpid->Err_last) + hpid->Ki*(hpid->Err_now)
    		+ hpid->Kd*(hpid->Err_now - hpid->Err_last - hpid->Err_diff);
	if(hpid->Output > hpid->Output_Max) hpid->Output = hpid->Output_Max;
	if(hpid->Output < -hpid->Output_Max) hpid->Output = -hpid->Output_Max;
	hpid->Err_diff = hpid->Err_now - hpid->Err_last;
	hpid->Err_last = hpid->Err_now;
}


