#include "vReadModuleSpeedTask.h"

static portTickType xLastTimeTick=0, xNowTimeTick=0, xTimeDiff=0;

void vReadModuleSpeedTask( void *pvParameters )  {
  while(1)  {
    static signed long readXValue, readYValue;
    vTaskDelay(MODULE_SPEED_READ_PERIOD); 
    //vTaskDelay(MODULE_SPEED_READ_PERIOD>>1);    
    halSetLedStatus(LED_BLUE, LED_TOGGLE);
    xNowTimeTick = xTaskGetTickCount();
    readXValue = halABXDecoderReadData();
    halABClearABXInhibitLogic();
    //vTaskDelay(MODULE_SPEED_READ_PERIOD>>1); 
    readYValue = halABYDecoderReadData();
    halABClearABYInhibitLogic();
    xTimeDiff = xNowTimeTick-xLastTimeTick;
    xLastTimeTick = xNowTimeTick;
    CalculateWheelSpeed(readXValue, readYValue, xTimeDiff);
    //testSpeedL = GetWheelLSpeed();
    //testSpeedR = GetWheelRSpeed();
    asm("NOP");
  }
}



