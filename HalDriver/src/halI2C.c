#include "halI2C.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//��ס�����ѵ����ЩƬ���ڶ�ȡʱ����Ҫ�����д��NACK��������ЩƬ���Ǳ���д��NACK, MPU9250��������֮һ������
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PG13->SCLK  PG14->DATA
void halI2CGPIOConfig(void) {
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;            //��©���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_SetBits(GPIOG, GPIO_Pin_13 | GPIO_Pin_14);
}

//����״̬ SCLK->�ߵ�ƽ, SDA->�ߵ�ƽ
void halI2CInit(void) {
  halI2CGPIOConfig();
  halI2CSCLHigh();
  halI2CSDAHigh();
}
//����״̬ SCLK->�͵�ƽ, SDA->�͵�ƽ
void halI2CStart(void){
  halMCUWaitUs(20);
  halI2CSDAHigh();
  halI2CSCLHigh();
  halI2CSDALow();
  halI2CSCLLow();
  halMCUWaitUs(20);
}
//����״̬ SCLK->�ߵ�ƽ, SDA->�ߵ�ƽ
void halI2CStop(void){
  halI2CSDALow();
  halI2CSCLHigh();
  halI2CSDAHigh();
}
//����״̬ SCLK->�͵�ƽ, SDA->���
bool halI2CReadAck(void) {
  halMakeI2CSDAInput();
  halI2CSCLHigh();
  if (halI2CSDAValue() != 0) {
    halI2CSCLLow();
    halMakeI2CSDAOutput();
    return false;
  }
  else {
    halI2CSCLLow();
    halMakeI2CSDAOutput();
    return true;
  }
}

void halI2CWriteAck(void) {
  halI2CSDALow();
  halI2CSCLHigh();
  halI2CSCLLow();
}

void halI2CWriteNAck(void) {
  halI2CSDAHigh();
  halI2CSCLHigh(); 
  halI2CSCLLow();
}

bool halI2CWriteByte(u8 data) {
  u8 i;
  for (i=0;i<8;i++) {
    if ((data&0x80)==0x80) {
      halI2CSDAHigh();
    }
    else {
      halI2CSDALow();
    }
    halI2CSCLHigh();
    data <<= 1;
    halI2CSCLLow();
  }
  return (halI2CReadAck());
}

u8 halI2CReadByte(void) {
  u8 data=0, i;
  halMakeI2CSDAInput();
  for (i=0;i<8;i++) {
    data <<= 1;
    halI2CSCLHigh();
    if (halI2CSDAValue()!= 0) {
      data += 1;
    }
    //halMCUWaitUs(2);
    halI2CSCLLow();
  }
  halMakeI2CSDAOutput();
  return data;
}

bool halI2CWritePacket(u8* data, u8 len){
  u8 i;
  for (i=0;i<len;i++){
    if (halI2CWriteByte(*(data+i))==false) {
      return false;
    }
  }
  return true;
}

void halI2CReadPacket(u8* data, u8 len) {
  u8 i;
  for (i=0;i<len;i++) {
    *(data+i) = halI2CReadByte();
    if (i==(len-1)){
      halI2CWriteNAck();
    }
    else {
      halI2CWriteAck();
    }
  }
}

unsigned char halI2CWriteRegPacket(u8 slaveAddr, u8 regAddr, u8 len,  unsigned char const *data){
  halI2CStart();                                      //��ʼ
  if (halI2CWriteByte((slaveAddr<<1) + 0)==false) {   //д��SLAVE��ַ����ACK
    halI2CStop();
    return 1;
  }
  if (halI2CWriteByte(regAddr)==false) {              //д��Ĵ�����ַ����ACK
    halI2CStop();
    return 1;
  }
  if (halI2CWritePacket((u8*)data, len)==false) {     //д������
    halI2CStop();
    return 1;
  }
  halI2CStop();                                       //ֹͣ
  return 0;
}

unsigned char halI2CReadRegPacket(u8 slaveAddr, u8 regAddr, u8 len,  unsigned char const *data){
  halI2CStart();                                      //��ʼ
  if (halI2CWriteByte((slaveAddr<<1) + 0)==false) {   //д��SLAVE��ַ����ACK
    return 1;
  }
  if (halI2CWriteByte(regAddr)==false) {              //д��Ĵ�����ַ����ACK
    return 1;
  }
  
  halI2CStart();
  if (halI2CWriteByte((slaveAddr<<1) + 1)==false) {   //д��SLAVE��ַ����ACK
    return 1;
  }
  
  halI2CReadPacket((u8*)data, len);                   //��ȡ����
  
  halI2CStop();                                       //ֹͣ
  return 0;
}
