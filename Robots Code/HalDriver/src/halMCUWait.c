#include "halMCUWait.h"

void halMCUWaitUs(unsigned long us){
  unsigned int i=0;
  while(us--) {
    i=10;
    while(i--);
  }
}

void halMCUWaitMs(unsigned long Ms){
   unsigned int i=0;  
   while(Ms--) {
     i=12000;  
     while(i--);
   }
} 


