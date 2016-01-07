#ifndef __MOTORFUNCTION_H
#define __MOTORFUNCTION_H

#include "halABDecoder.h"
#include "sysConfig.h"
#include "halDriver.h"
#include "vTasks.h"

////////////////////////////////////////////////////////////////////////////////
//增量式PID 算法
//deltaU = a*e(k) - b*e(k-1) + c*e(k-2)
//a=Kp*(1+T/Ti+Td/T) 
//b=Kp*(1+2*Td/T)
//C=Kp*Td/T
////////////////////////////////////////////////////////////////////////////////
#define Kp     (0.000001)
#define KTi    (5.0)
#define KTd    (5.0)
#define KT     (MODULE_SPEED_CONTROL_PERIOD*0.001)

////////////////////////////////////////////////////////////////////////////////
//增量式PID 直接设定PID_KP PID_KI PID_KD
////////////////////////////////////////////////////////////////////////////////
#define PID_KP     (0.01)
#define PID_KI_1   (0.001)
#define PID_KI_2   (0.0015)
#define PID_KI_3   (0.002)
#define PID_KD     (0)

/*
#define PID_KI_1   (0.001)
#define PID_KI_2   (0.002)
#define PID_KI_3   (0.003)
#define PID_KD     (0)
*/

typedef struct typeWheelSpeed{
  float left;
  float right;
}typeWheelSpeed;

extern void CalculateWheelSpeed(signed long xValue, signed long yValue, u32 timeDiff);
extern float GetWheelLSpeed(void);
extern float GetWheelRSpeed(void);

extern void PIDParaInit(void);
extern float PIDControlLMotor();
extern float PIDControlRMotor();

extern typeWheelSpeed GetWheelGivenSpeed(void);
extern float GetWheelLGivenSpeed(void);
extern float GetWheelRGivenSpeed(void);
extern void SetWheelGivenSpeed(float valueL, float valueR);
extern void SetWheelLGivenSpeed(float value);
extern void SetWheelRGivenSpeed(float value);

extern void ModuleTurnAngle(float speed, float angle);
extern void ModuleGoStrait(float speed, float dis);

extern float CalFloatAbs(float data);

#endif

