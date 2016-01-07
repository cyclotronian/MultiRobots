#include "vLocationTask.h"
#include "robot.h"
#include "vInfoList.h"
#include <math.h>

typeCoordinate location;
void vLocationTask( void *pvParameters ) {
  halt(1);
  while(1) {
    location = GetCoordinate();
    vTaskDelay(100);
  }
}
