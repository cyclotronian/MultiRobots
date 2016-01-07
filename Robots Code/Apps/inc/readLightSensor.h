#ifndef __READLIGHTSENSOR_H
#define __READLIGHTSENSOR_H

#include "halADC.h"
#include "stm32f10x.h"

#define LIGHT_SENSOR_READ_TIMES  (16)

extern float ReadLightSensor(void);
extern void LightSensorInit(void);


#endif


