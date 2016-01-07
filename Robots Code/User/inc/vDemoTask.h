#ifndef __VDEMOTASK_H
#define __VDEMOTASK_H

#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "vTasks.h"
#include "apps.h"
#include "stdbool.h"

#include "vInfoList.h"

#define M_PI  3.14159265358979323846f
#define SLOW  100.0f
#define STOP   40.0f

extern void vDemoTask( void *pvParameters );
void halt(float time);


#endif
