#include "RF.h"
#include <math.h>
#include "vInfoList.h"
////////////////////////////////////////////////////////////////////////////////
//RF init
////////////////////////////////////////////////////////////////////////////////
void setPos(float, float);
void RFInit(void) {
  
}
////////////////////////////////////////////////////////////////////////////////
//PB5 PD2 connect to CC2530
////////////////////////////////////////////////////////////////////////////////
bool ReadRFIndicationPin1(void) {
  if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)==Bit_RESET){
    return false;
  }
  return true;
}
bool ReadRFIndicationPin2(void) {
  if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)==Bit_RESET){
    return false;
  } 
  return true;
}
////////////////////////////////////////////////////////////////////////////////
//RF send packet
////////////////////////////////////////////////////////////////////////////////
u8 RFTxPacket(u8 command, u8* packet, u8 packetLen) {
  u8 sum=0, i=0;
  UartTxByte(0x7E);
  UartTxByte(0x45);
  UartTxByte(command);
  UartTxByte(packetLen);
  UartTxPacket(packet, packetLen);
  sum+=(0x7E+0x45+command+packetLen);
  for (i=0;i<packetLen;i++) {
    sum+=*(packet+i);
  }
  UartTxByte(sum);
  return 1;
}
////////////////////////////////////////////////////////////////////////////////
//RF receive packet
////////////////////////////////////////////////////////////////////////////////
//bool rfPacketRecDone=false;
static u8 rfRxBuf[RF_REC_BUF_SIZE];
static u8 rfRxBufWP=0;
static rbNode bcastinfo[ROBOTS];

void Usart0RxIsr(u8 data){
  if ((rfRxBufWP==0) && (data != 0x7E)){
    rfRxBufWP=0;
  }
  if ((rfRxBufWP==1) && (data != 0x45)){
    rfRxBufWP=0;
  }
  
  if(rfRxBufWP==0) {       //header1
    rfRxBuf[0] = data;
    rfRxBufWP++;
  }
  else if(rfRxBufWP==1) {  //header2
    rfRxBuf[rfRxBufWP] = data;
    rfRxBufWP++;
  }
  else if(rfRxBufWP==2) {  //command
    rfRxBuf[rfRxBufWP] = data;
    rfRxBufWP++;
  }
  else if(rfRxBufWP==3) {  //payload Len
    rfRxBuf[rfRxBufWP] = data;
    rfRxBufWP++;
    if (data>RF_REC_BUF_SIZE-4) {
      rfRxBufWP = 0;
    }
  }
  else if((rfRxBufWP>=4)&&(rfRxBufWP<(rfRxBuf[3]+4))) {  //payload 
    rfRxBuf[rfRxBufWP] = data;
    rfRxBufWP++;
  }
  else if (rfRxBufWP==(rfRxBuf[3]+4)) {  //sum
    rfRxBuf[rfRxBufWP] = data;
    u8 i,sum=0;
    float dis=0;
    for (i=0;i<rfRxBufWP;i++) {
      sum+=rfRxBuf[i];
    } 	
    if (sum==rfRxBuf[rfRxBufWP]) {                   //sum is right!	     
      if (rfRxBuf[2]==RF_DIST_2_BEACON1) {           //dis to B1
        memcpy(&dis, &rfRxBuf[4],4);
        SetDistanse2B1(dis);
      }
      else if (rfRxBuf[2]==RF_DIST_2_BEACON2) {      //dis to B2
        memcpy(&dis, &rfRxBuf[4],4);
        SetDistanse2B2(dis);
        setPos(GetDistanse2B1(),GetDistanse2B2());
      }
     else if (rfRxBuf[2]==RF_BROADCAST_INFO) {       //BCAST INFO
        //user add code to decode bcast info
	    //SetLedStatus(LED_YELLOW, LED_TOGGLE);
		u8 id = rfRxBuf[4] - 1;
	    bcastinfo[id].nodeID = rfRxBuf[4];              //nodeID
	    memcpy(&(bcastinfo[id].rpos.locationX),&rfRxBuf[5],4);
	    memcpy(&(bcastinfo[id].rpos.locationY),&rfRxBuf[9],4);
	    memcpy(&(bcastinfo[id].isReady),&rfRxBuf[13],4);
            //SetLeftWheelGivenSpeed(30);
            //SetRightWheelGivenSpeed(-30); 
            //vTaskDelay(1500);
            //halt(1);
        asm ("NOP");
        //user end
      }
    }
    rfRxBufWP =  0;
  }
}

u8 isReady(u8 id) {
  return bcastinfo[id-1].isReady;
}

//add by sundy
rbNode recBoardCastInfo(u8 id){
  return bcastinfo[id];
}

static float distanse2B1=0, distanse2B2=0;

float GetDistanse2B1(void) {
  return distanse2B1;
}

float GetDistanse2B2(void) {
  return distanse2B2;
}

void SetDistanse2B1(float dis) {
  distanse2B1=dis;
}
void SetDistanse2B2(float dis) {
  distanse2B2=dis;
}

struct getXY{
  float X;
  float Y;
};

struct getXY newPos;

void setPos(float distB1, float distB2){
  float distBeacon = DISTANSE_B1_2_B2;
  float height = DISTANSE_B_2_GROUND;
  newPos.Y = (distB1*distB1 - distB2*distB2 + distBeacon*distBeacon)/(2*distBeacon);
  newPos.X = sqrt(distB1*distB1 - height*height - newPos.Y*newPos.Y);
  asm("NOP");
}

float GetPosX(void) {
  return newPos.X*1000;
}

float GetPosY(void) {
  return newPos.Y*1000.0f;
}

