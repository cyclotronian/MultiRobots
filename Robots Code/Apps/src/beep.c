#include "beep.h"

void BeepInit(void){
  halBeepInit();
}
void SetBeepStatus(u8 beepStatus){
   halSetBeepStatus(beepStatus);
}
