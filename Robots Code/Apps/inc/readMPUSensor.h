#ifndef __READMPUSENSOR_H
#define __READMPUSENSOR_H
 
#include "halMPUInit.h"
#include "stm32f10x.h"
#include "math.h"
#include "halI2C.h"

typedef struct typeCalMagSensor{
  float x;
  float y;
}typeCalMagSensor;

extern typeMPUSensor ReadMPUSensor(void);
extern typeMagSensor ReadMagSensor(void);
extern u8 MPUSensorInit(void);
extern typeCalMagSensor CallibrateMagSensor(u16 Xmax, u16 Xmin, u16 Ymax, u16 Ymin);


#endif

