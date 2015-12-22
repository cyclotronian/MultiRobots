#ifndef __HALBEEP_H
#define __HALBEEP_H

#include "stm32f10x.h"


#define BEEP_ON       (0)
#define BEEP_OFF      (1)
#define BEEP_TOGGLE   (2)

extern void halBeepInit(void);
extern void halSetBeepStatus(u8 status);

#endif

