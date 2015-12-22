#include "uart.h"

void UartInit(void) {
  halUsart1Init();
}

void UartWriteTxBufPacket(u8* data, u8 len) {
  
}

void UartTxByte(u8 data){
  halUsart1TxByte(data);
}

void UartTxPacket(u8* data, u8 len) {
  halUsart1TxPacket(data, len);
}

