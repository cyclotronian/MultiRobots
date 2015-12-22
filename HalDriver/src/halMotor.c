#include "halMotor.h"

void halPWMGPIOConfig(void) {
  //PB0     ------> TIM1_CH2N
  //PB1     ------> TIM1_CH3N
  //PA9     ------> TIM1_CH2
  //PA10    ------> TIM1_CH3
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //PA11     ------> PWM_EN_LEFT
  //PA12     ------> PWM_EN_RIGHT
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);  //TIM1¶Ë¿ÚÖØÐÂÓ³Éä
}

void halPWMConfig(void) {

  u32 TimerPeriod;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TimerPeriod = (72000000 / 100000) - 1;
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  TIM_OCInitTypeDef TIM_OCInitStructure;
/* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  /* 25% duty */
  TIM_OCInitStructure.TIM_Pulse = (uint16_t) (((uint32_t) 50 * (((72000000 / 100000 ) - 1) - 1)) / 100);
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);           /* Set OC1&OC1N duty */
  TIM_OCInitStructure.TIM_Pulse = (uint16_t) (((uint32_t) 50 * (((72000000 / 100000 ) - 1) - 1)) / 100);
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);           /* Set OC1&OC1N duty */

  TIM_BDTRInitTypeDef TIM1_BDTRInitStructure;
// Automatic Output enable, Break, dead time and lock configuration
  TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
  TIM1_BDTRInitStructure.TIM_DeadTime = 0xC0;        // 3.55us
  TIM1_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
  TIM1_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
  TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
  TIM_BDTRConfig(TIM1,&TIM1_BDTRInitStructure);


/* TIM IT enable */
  TIM_ITConfig(TIM1, TIM_IT_Update , ENABLE);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void halPWMSet(float percent) {

}

void halEnablePWMOutput(void) {
  GPIO_SetBits(GPIOA, GPIO_Pin_11);
  GPIO_SetBits(GPIOA, GPIO_Pin_12);
}

void halDisablePWMOutput(void) {
  GPIO_ResetBits(GPIOA, GPIO_Pin_11);
  GPIO_ResetBits(GPIOA, GPIO_Pin_12);
}

void halPWMInit(void) {
  halPWMGPIOConfig();
  halPWMConfig();
  halEnablePWMOutput();
}

void SetMotorPWM(float motorX, float motorY){
  /*
  TIM1->CCR2 = (unsigned long)motorX;
  TIM1->CCR3 = (unsigned long)motorY;
  */
  TIM1->CCR2 = (uint16_t) (((uint32_t) (motorY * (((72000000 / 100000 ) - 1) - 1))) );
  TIM1->CCR3 = (uint16_t) (((uint32_t) (motorX * (((72000000 / 100000 ) - 1) - 1))) );
}

void EnableMotorRun(bool data) {
  if (data==false) {
    GPIO_ResetBits(GPIOA, GPIO_Pin_11 | GPIO_Pin_12);
  }
  else {
    GPIO_SetBits(GPIOA, GPIO_Pin_11 | GPIO_Pin_12);
  }
}

