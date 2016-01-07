#include "halRCCInit.h"

////////////////////////////////////////////////////////////////////////////////
//RCC CLOCK ≥ı ºªØ
////////////////////////////////////////////////////////////////////////////////
void halRCCInit(void) {
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable DAC, USART2, TIM2, PWR, CAN and BKP clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC    | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
                         RCC_APB1Periph_USART2 | RCC_APB1Periph_PWR  | RCC_APB1Periph_SPI2 |
						RCC_APB1Periph_BKP, ENABLE);


  /* Enable GPIOx (x = A, B, C, D, E, F, G) ADC1, USART1, SPI1 and AFIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | RCC_APB2Periph_TIM1   |
		         RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO  | RCC_APB2Periph_USART1 |
						RCC_APB2Periph_ADC1  | RCC_APB2Periph_SPI1,ENABLE);  //RCC_APB2Periph_GPIO_CS|
                                                    /* Enable PWR and BKP clock */
//RCC_APB2Periph_USART1|
  /* Enable the FSMC AHB Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC , ENABLE);

  /* Enable the SDIO AHB Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO, ENABLE);

  /* Enable the DMA2 Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
}

