#include "interruptMotorTask.h"

static u32 testCounter=0;
void interruptMotorTask( void *pvParameters ) {
  MotorRunEnable(true);
  while(1){    
    asm("NOP");
    if((haltornot == 1) && (getTime() > 15000)){
      MotorRunEnable(true);
      boardCastInfos();
      asm("NOP");
    }
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

