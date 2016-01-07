#include "vRFRxInfoTask.h"
#include "vBCastInfoTask.h"

void vRFRxInfoTask( void *pvParameters ){
  while(1)  {
    vTaskDelay(RF_DECODE_PERIOD);
    //halSetLedStatus(LED_RED, LED_TOGGLE);  
    //typeBcastInfo bcastInfo;  
    if (GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2)==Bit_RESET) {
      halSetLedStatus(LED_YELLOW, LED_OFF); 
    }
    else {
      halSetLedStatus(LED_YELLOW, LED_ON); 
    }
    
    asm("NOP");
  }
}

typeBcastInfo rfRxInfoBuf[20];
u8 rfRxInfoBufP=0;

u8 usartRxBuf[128];
u8 usartRxBufP=0;

void Usart0RxIsr(u8 data){
  if ((usartRxBufP==0) && (data != 0x7E)){
    usartRxBufP=0;
  }
  if ((usartRxBufP==1) && (data != 0x45)){
    usartRxBufP=0;
  }
  
  //halSetLedStatus(LED_YELLOW, LED_TOGGLE);
}


