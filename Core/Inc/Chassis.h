#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "pid.h"
#ifndef PI
#define PI 3.14159265358979f
#endif
#define Length 0.38f
#define Width 0.41f
#define Wheel_radius 0.075f
#define Power_limit 80.0f
#define Speed_rpm_Limit 450.0f

extern void Chassis_Ctrl(void);
extern void Chassis_Move(void);
extern void Chassis_PowerCtrl(void);
extern void Chassis_Follow(void);
extern void Chassis_Swing(void);
extern void Chassis_angleTransform(void);
extern void Chassis_Task(void);
extern int16_t Chassis_ctrl[4];
#endif
