#include "vReadMPUTask.h"

void vReadMPUTask( void *pvParameters ){
  while(1)  {
    vTaskDelay(MPU_SAMPLE_PERIOD);
    halSetLedStatus(LED_YELLOW, LED_TOGGLE); 
    halReadMPUSensor();
    asm("NOP");
  }
}

