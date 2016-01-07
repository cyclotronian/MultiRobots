#ifndef __UART_H
#define __UART_H


#include "halUsart.h"
#include "stm32f10x.h"

#define UART_TX_BUF_SIZE  (100)
#define UART_RX_BUF_SIZE  (100)


extern void UartInit(void);
extern void UartTxPacket(u8* data, u8 len);
extern void UartTxByte(u8 data);

#endif

