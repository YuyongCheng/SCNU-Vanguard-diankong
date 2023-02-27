#include "Chassis.h"
#include "pid.h"
#include "math.h"

int16_t Chassis_ctrl[4] = {0};
float speed_x,speed_y,omega,speed_x_commend,speed_y_commend,angle_rad;

//按目标速度控制底盘
void Chassis_Ctrl()
{
				//功率限制
		Chassis_PowerCtrl();
		for(int i=0;i<4;i++)
			{
				PID_Incr(&PID_Motor_Speed[i],Motor[i].speed,Motor[i].target_speed);
				Chassis_ctrl[i] += PID_Motor_Speed[i].Output;
				if(Chassis_ctrl[i]>16384)
					Chassis_ctrl[i] = 16384;
				if(Chassis_ctrl[i]<-16384)
					Chassis_ctrl[i] = -16384;
			}

}

//以速度为输入的全向移动函数，线速度单位mps，角速度单位radps
void Chassis_Move()
{
	//麦轮底盘解算
	Motor[1].target_speed = -(speed_x - speed_y + omega*0.5*(Length + Width))*60/(2*Wheel_radius*PI)*3591/187;
	Motor[2].target_speed = (speed_x + speed_y - omega*0.5*(Length + Width))*60/(2*Wheel_radius*PI)*3591/187;
	Motor[3].target_speed = (speed_x - speed_y - omega*0.5*(Length + Width))*60/(2*Wheel_radius*PI)*3591/187;
	Motor[4].target_speed = -(speed_x + speed_y + omega*0.5*(Length + Width))*60/(2*Wheel_radius*PI)*3591/187;
	//限制最大转速
	for(char i=0;i<4;i++)
	{
		while(Motor[i].target_speed>Speed_rpm_Limit*3591/187)
			for(char i=0;i<4;i++)
		{
				Motor[i].target_speed*=0.9;
		}
	}
	//底盘控制
	Chassis_Ctrl();
}

//功率限制,当功率估计值超过限制值时同比例削减目标速度
void Chassis_PowerCtrl(void)
{
	float Chassis_Power = 0;
	float last_speed_sum = 0;
	float speed_sum = 0;
	for(char i=0;i<4;i++)
	{
		Chassis_Power += Motor[i+1].current*Motor[i+1].speed*PI*0.2/16384;
		last_speed_sum += Motor[i+1].speed;
		speed_sum += Motor[i+1].target_speed;
	}

	if(Chassis_Power > Power_limit)
	{
		//限制目标速度
		for(char i=0;i<4;i++)
		{
			Motor[i+1].target_speed *= (last_speed_sum*Power_limit)/(speed_sum*Chassis_Power);
		}
	}
}

//底盘跟随
void Chassis_Follow(void)
{
	PID_Origin(& PID_Motor_Angle[Chassis_Angle_ID], Motor[Motor_Yaw_ID].angle, 0);
	//底盘角度环,以底盘与云台的相对角度为输入
	omega = PID_Motor_Angle[Chassis_Angle_ID].Output_float;
}
//底盘摇摆
void Chassis_Swing(void)
{
	if(angle_rad > PI/4)
		omega = 2.0;
	else if(angle_rad < -PI/4)
		omega = -2.0;
	else if(omega != 2.0&&omega != -2.0)
		omega = 2.0;
}
//相对云台的速度解算
void Chassis_angleTransform(void)
{
	uint16_t temp;
	temp = Motor[Motor_Yaw_ID].angle;
	if(temp>=4096)
		temp = -(8192-temp);
	angle_rad = temp*PI/4096;
	angle_rad -= PI/2;
	if(angle_rad <= -PI)
		angle_rad += 2*PI;
	speed_x = speed_x_commend*cos(angle_rad) + speed_y_commend*sin(angle_rad);
	speed_y = speed_y_commend*cos(angle_rad) - speed_x_commend*sin(angle_rad);
}
