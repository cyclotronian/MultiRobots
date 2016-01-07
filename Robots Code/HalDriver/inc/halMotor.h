#ifndef __HALMOTOR_H
#define __HALMOTOR_H

#include "stm32f10x.h"
#include "stdbool.h"

extern void halPWMInit(void);
extern void SetMotorPWM(float motorX, float motorY);
extern void EnableMotorRun(bool data);


#endif