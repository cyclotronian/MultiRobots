#include "led.h"

void LedInit(void){
  halLedInit();
}
void SetLedStatus(u8 led, u8 status){
  halSetLedStatus(led,status);
}


