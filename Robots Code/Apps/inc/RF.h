#ifndef __RF_H
#define __RF_H

#include "uart.h"
#include "stm32f10x.h"
#include "stdbool.h"
#include "string.h"
#include "led.h"

#define RF_BROADCAST_ADDR        (0xFF)
#define RF_BROADCAST_INFO        (0xA1)
#define RF_DIST_2_BEACON1        (0xA2)
#define RF_DIST_2_BEACON2        (0xA3)
#define RF_BROADCAST_USONIC      (0xA4)

#define RF_REC_BUF_SIZE          (100)

typedef struct typeRFPacket{
  u8  header1;
  u8  header2;
  u8  command;
  u8  payloadLen;
  u8* payload;
  u8  sum;
}typeRFPacket;

extern void RFInit(void);
extern bool ReadRFIndicationPin1(void);
extern bool ReadRFIndicationPin2(void);
extern u8 RFTxPacket(u8 command, u8* packet, u8 packetLen);


extern void SetDistanse2B1(float dis);
extern void SetDistanse2B2(float dis);
extern float GetDistanse2B1(void);
extern float GetDistanse2B2(void);
extern u8 isReady(u8 id);

#endif


