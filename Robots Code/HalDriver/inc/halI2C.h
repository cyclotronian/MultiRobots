#ifndef __HALI2C_H
#define __HALI2C_H

#include "stm32f10x.h"
#include "stdbool.h"
#include "halMCUWait.h"

//PG13->SCLK  PG14->DATA
#define halI2CSCLHigh()      GPIOG->BSRR = GPIO_Pin_13;   halMCUWaitUs(2)
#define halI2CSCLLow()       GPIOG->BRR  = GPIO_Pin_13;   halMCUWaitUs(2)

#define halI2CSDAHigh()      GPIOG->BSRR = GPIO_Pin_14;   halMCUWaitUs(2)
#define halI2CSDALow()       GPIOG->BRR  = GPIO_Pin_14;   halMCUWaitUs(2)

#define halI2CSDAValue()     ((GPIOG->IDR)&(1<<14))

#define halMakeI2CSDAInput()    GPIOG->CRH&=~(7<<24);  GPIOG->CRH|=(8<<24); halMCUWaitUs(2)
#define halMakeI2CSDAOutput()   GPIOG->CRH&=~(12<<24); GPIOG->CRH|=(3<<24); halMCUWaitUs(2)


extern void halI2CInit(void);
extern void halI2CStart(void);
extern void halI2CStop(void);
extern unsigned char halI2CReadRegPacket(u8 slaveAddr, u8 regAddr, u8 len,  unsigned char const *data);
extern unsigned char halI2CWriteRegPacket(u8 slaveAddr, u8 regAddr, u8 len,  unsigned char const *data);
                             

#endif
