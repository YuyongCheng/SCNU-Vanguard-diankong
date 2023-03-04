#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "pid.h"
#ifndef PI
#define PI 3.14159265358979f
#endif
#define Length 0.38
#define Width 0.41
#define Wheel_radius 0.075
#define Power_limit 80.0
#define Speed_rpm_Limit 450.0

extern void Chassis_Ctrl(void);
extern void Chassis_Move(void);
extern void Chassis_PowerCtrl(void);
extern void Chassis_Follow(void);
extern void Chassis_Swing(void);
extern void Chassis_angleTransform(void);
extern void Chassis_Task(void);
extern int16_t Chassis_ctrl[4];
extern float speed_x,speed_y,omega,speed_x_commend,speed_y_commend,angle_rad,Chassis_Power;
#endif
