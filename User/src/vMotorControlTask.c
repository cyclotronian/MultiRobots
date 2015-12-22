#include "vMotorControlTask.h"

static u32 testCounter=0;
//extern int haltornot;
void vMotorControlTask( void *pvParameters ) {
  MotorRunEnable(true);
  while(1){    
    vTaskDelay(MODULE_SPEED_CONTROL_PERIOD);
    //SetMotorPWM(0.5,0.5);
    ControlWheelSpeedAlg(GetWheelGivenSpeed() , ReadWheelSpeed());
    testCounter++;
    if (testCounter == 50) {
      testCounter=0;
      SetLedStatus(LED_BLUE, LED_TOGGLE);
    }
  }
}


