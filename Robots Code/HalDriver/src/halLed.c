#include "halLed.h"

////////LED使用的IO口/////////////////////////////////////////////////////////
//(GPIOF, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9)
//////////////////////////////////////////////////////////////////////////////
void halLedInit(void){
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOF, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
}
 
void halSetLedStatus(unsigned char led, unsigned char status){
  if (led==LED_GREEN) {
    if (status==LED_ON) {
      GPIO_SetBits(GPIOF,GPIO_Pin_6);
    }
    else if (status==LED_OFF){
      GPIO_ResetBits(GPIOF,GPIO_Pin_6);
    }
    else if (status==LED_TOGGLE){
      (GPIOF->ODR)^=(1<<6);
    }
  }
  else if (led==LED_BLUE) {
    if (status==LED_ON) {
      GPIO_SetBits(GPIOF,GPIO_Pin_7);
    }
    else if (status==LED_OFF){
      GPIO_ResetBits(GPIOF,GPIO_Pin_7);
    }
    else if (status==LED_TOGGLE){
      (GPIOF->ODR)^=(1<<7);
    }
  }
  else if (led==LED_YELLOW) {
    if (status==LED_ON) {
      GPIO_SetBits(GPIOF,GPIO_Pin_8);
    }
    else if (status==LED_OFF){
      GPIO_ResetBits(GPIOF,GPIO_Pin_8);
    }
    else if (status==LED_TOGGLE){
      (GPIOF->ODR)^=(1<<8);
    }
  }
  else if (led==LED_RED) {
    if (status==LED_ON) {
      GPIO_SetBits(GPIOF,GPIO_Pin_9);
    }
    else if (status==LED_OFF){
      GPIO_ResetBits(GPIOF,GPIO_Pin_9);
    }
    else if (status==LED_TOGGLE){
      (GPIOF->ODR)^=(1<<9);
    }
  }
}

unsigned char halGetLedStatus(unsigned char led) {
  unsigned int ledODR=(GPIOF->ODR);
  if (led==LED_GREEN) {
    return !(((ledODR>>6)+0x01)&0x01);
  }
  else if (led==LED_BLUE) {
    return !(((ledODR>>7)+0x01)&0x01);
  }
  else if (led==LED_YELLOW) {
    return !(((ledODR>>8)+0x01)&0x01);
  }
  else if (led==LED_RED) {
    return !(((ledODR>>9)+0x01)&0x01);
  }
  return 0;
}


