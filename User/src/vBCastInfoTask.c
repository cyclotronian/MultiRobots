#include "vBCastInfoTask.h"

/*
typedef struct typeUsartPacket{
  unsigned char header1;    //0x7E
  unsigned char header2;    //0x45
  unsigned char command;    //command
  unsigned char payloadLen; 
  unsigned char payload[USART_PAYLOAD_BUF_SIZE];
  unsigned char sum;
}typeUsartPacket;
*/

void RFTxData(u8* data, u8 len) {
  u8 dataBuf[128];
  u8 i;
  u8 sum=0;
  dataBuf[0] = 0x7E;
  dataBuf[1] = 0x45;
  dataBuf[2] = USART_RF_SEND_DATA;
  dataBuf[3] = NODE_ID;
  dataBuf[4] = NODE_ID>>8;
  dataBuf[5] = RF_BCAST_ADDRESS;
  dataBuf[6] = RF_BCAST_ADDRESS>>8;
  dataBuf[7] = len;
  memcpy(dataBuf+8, data, len);
  for (i=0;i<len+8;i++) {
    sum+=dataBuf[i];
  }
  dataBuf[len+8] = sum;
  halUsart1TxPacket(dataBuf, len+9);
}

void vBCastInfoTask( void *pvParameters ){
  while(1)  {
	
	asm("NOP");
}


