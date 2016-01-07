#include "vDemoTask.h"
#include "robot.h"
#include "vInfoList.h"
#include <math.h>

#define GETMAG

#ifdef GETMAG
//get magX and magY
typeMagSensor data[100], nowdata;
float minX, maxX, minY, maxY, magX, magY;
int datan;
void vMagTask( void *pvParameters ) {
  minX = minY = 100000;
  maxX = maxY = -100000;
  halt(0);
  for (datan = 0; datan < 48; ++ datan) {
    nowdata = ReadMagSensor();
    if (minX > nowdata.magX) minX = nowdata.magX;
    if (minY > nowdata.magY) minY = nowdata.magY;
    if (maxX < nowdata.magX) maxX = nowdata.magX;
    if (maxY < nowdata.magY) maxY = nowdata.magY;
    data[datan] = nowdata;
    ControlRobotRotate(15, 5);
    halt(0.5);
  }
  magX = (minX + maxX) / 2;
  magY = (minY + maxY) / 2;
  vTaskDelay(100000);
  asm("NOP");
}
#endif
#ifdef TESTMAG
// test mag
float robotDir;
void vMagTask( void *pvParameters ) {
  while (1) {
    robotDir = ReadMagSensorAngle2North();
    vTaskDelay(1000);
  }
}
#endif
