#include "halBeep.h"

////////////////////////////////////////////////////////////////////////////////
//PA8 -> BEPP contorl pin
////////////////////////////////////////////////////////////////////////////////
void halBeepInit(void){
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}

void halSetBeepStatus(u8 status){
  if (status == BEEP_ON) {
    GPIO_SetBits(GPIOA,GPIO_Pin_8);
  }
  else if (status == BEEP_OFF) {
    GPIO_ResetBits(GPIOA,GPIO_Pin_8);
  }
  else if (status == BEEP_TOGGLE) {
    (GPIOA->ODR)^=(1<<8);
  }

}

