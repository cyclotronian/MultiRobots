#ifndef __HALUSART_H
#define __HALUSART_H

#include "stm32f10x.h"


extern void halUsart1Init(void);
extern void halUsart1TxByte(u8 data);
extern void halUsart1TxPacket(u8* data, u8 len);

#endif
