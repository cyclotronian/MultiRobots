#include "prvSetupHardware.h"

void prvSetupHardware(void) {
////////////////////////////////////////////////////////////////////////////////
  SystemInit();    //cpu clock    init
  halRCCInit();    //other clock  init
  

}

