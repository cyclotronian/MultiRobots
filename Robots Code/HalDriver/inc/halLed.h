#ifndef __HALLED_H
#define __HALLED_H

#include "stm32f10x.h"

#define LED_RED     (1)
#define LED_GREEN   (2)
#define LED_BLUE    (3)
#define LED_YELLOW  (4)

#define LED_ON      (0)
#define LED_OFF     (1)
#define LED_TOGGLE  (2)

extern void halLedInit(void);
extern void halSetLedStatus(unsigned char led, unsigned char status);

#endif